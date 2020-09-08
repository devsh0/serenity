/*
 * Copyright (c) 2020, SerenityOS Developers.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ID3v2Parser.h"
#include <AK/StringBuilder.h>

#define ID3_DBG 1
#define debug_fmt(fmt, args...) \
    if (ID3_DBG)                \
    dbg() << String::format(fmt, args)
#define debug(arg) \
    if (ID3_DBG)   \
    dbg() << arg

namespace Audio {

static String read_string(BinaryStream& stream, u8 encoding)
{
    // Don't remove this line. This is to satisfy the compiler.
    debug_fmt("Encoding=%d", encoding);
    StringBuilder builder;
    char c = '\0';
    stream >> c;
    // FIXME: For UTF-16, the null character is 2 bytes.
    while (c != '\0') {
        builder.append(c);
        stream >> c;
    }

    builder.append(c);
    return builder.build();
}

static bool parse_text_frame(BinaryStream& stream, MpegAudioContext& context, String id, size_t size)
{
    if (!stream.ensure_bytes(size))
        return false;

    // FIXME: Encoding is ignored.
    u8 encoding = 0;
    stream >> encoding;
    size--;

    StringBuilder data(size);
    for (size_t i = 0; i < size; i++) {
        char tmp = (char)stream.read_u8();
        data.append(tmp);
    }

    // TODO: Maybe collect data from a couple of other frames?
    if (id == "TALB")
        context.album_title = data.build();
    else if (id == "TIT2")
        context.title = data.build();
    else if (id == "TYER")
        context.year = data.build();
    else
        debug_fmt("%s ignored", id.characters());

    return true;
}

static bool parse_picture_frame(BinaryStream& stream, MpegAudioContext& context, size_t size)
{
    if (!stream.ensure_bytes(size)) {
        dbg() << String::format("%d: insufficient bytes remaining for picture frame!");
        return false;
    }

    size_t to_read = size;

    // FIXME: ignored
    u8 encoding = 0;
    stream >> encoding;
    to_read--;

    // FIXME: if MIME type is -->, then image data is a URL.
    String mime = read_string(stream, encoding);
    // String builder reports size 0 if the string only contains the null-byte. I have a suspicion that the behavior
    //  is not consistent. Meaning that if the string contains bytes other than the null-character and is terminated
    //  by the null-byte, then the constructed string's length will include the null byte.
    to_read -= (mime.length() ? mime.length() : 1);
    context.cover_art_mime = mime;

    // Ignoring picture type.
    stream.skip_bytes(1);
    to_read--;

    // Ignoring image description.
    String description = read_string(stream, encoding);
    to_read -= (description.length() ? description.length() : 1);

    if (context.cover_art.size() < to_read) {
        context.cover_art.resize(to_read);
        for (u32 i = 0; i < to_read; i++)
            stream >> context.cover_art[i];
    }
    return true;
}

static size_t parse_frame(BinaryStream& stream, MpegAudioContext& context)
{
    if (!stream.ensure_bytes(10)) {
        dbg() << String::format("%d: insufficient bytes remaining for ID3 header!", stream.offset());
        return 0;
    }

    char tmp_str[5] = { 0 };
    stream >> tmp_str[0] >> tmp_str[1] >> tmp_str[2] >> tmp_str[3];
    String frame_id(tmp_str);

    u32 frame_size = stream.read_network_order_u32();

    // FIXME: ignored.
    u8 status_flags = 0, encode_flags = 0;
    stream >> status_flags >> encode_flags;

    if (frame_id[0] == 'T' && frame_id[1] != 'X') {
        parse_text_frame(stream, context, frame_id, frame_size);
    } else if (frame_id == "APIC") {
        parse_picture_frame(stream, context, frame_size);
    }
    return 10 + frame_size;
}

static size_t parse_tag_header(BinaryStream& stream, MpegAudioContext& context)
{
    if (!stream.ensure_bytes(10))
        return 0;

    u8 id[3] = { 0 };
    stream >> id[0] >> id[1] >> id[2];

    u8 minor = 0, revision = 0;
    stream >> minor >> revision;

    if (minor >= 0xFF || revision >= 0xFF)
        return 0;

    StringBuilder builder;
    builder.append("ID3v2.");
    builder.append('0' + minor);
    builder.append('.');
    builder.append('0' + revision);
    context.id3_string = builder.build();

    // FIXME: ignored
    stream >> context.id3_flags;

    u32 tag_size = 0;
    for (int i = 0; i < 4; i++)
        tag_size = (tag_size << 7) | stream.read_u8();

    return tag_size;
}

inline bool valid_frame_ahead(BinaryStream& stream)
{
    auto buffer = stream.peek(4);
    if (stream.handle_read_failure())
        return false;
    for (u8 byte : buffer)
        if ((byte < 'A' || byte > 'Z') && (byte < '0' || byte > '9'))
            return false;
    return true;
}

inline bool valid_tag_ahead(BinaryStream& stream)
{
    auto buffer = stream.peek(3);
    if (stream.handle_read_failure())
        return false;
    return buffer[0] == 'I' && buffer[1] == 'D' && buffer[2] == '3';
}

// FIXME: we ignore text encoding everywhere, which will cause this to
//  break for non-ascii strings.
bool parse_id3(BinaryStream& stream, MpegAudioContext& context)
{
    while (valid_tag_ahead(stream)) {
        ssize_t tag_size = parse_tag_header(stream, context);
        debug_fmt("ID3v2 tag size=%d", tag_size);
        while (valid_frame_ahead(stream)) {
            tag_size -= parse_frame(stream, context);
        }

        // Consume the padded bytes.
        while (tag_size > 0) {
            u8 byte = 0;
            stream >> byte;
            if (byte != 0) {
                dbg() << String::format("%d: padding bytes ane not all 0s!", stream.offset());
                return false;
            }
            tag_size--;
        }
    }

    return true;
}
}
