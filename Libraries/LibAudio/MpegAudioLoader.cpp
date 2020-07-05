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

#include "MpegAudioLoader.h"
#include "ID3v2Parser.h"

#define MPEG_debug 1
#define debug_fmt(fmt, args...) \
    if (MPEG_debug)             \
    dbg() << String::format(fmt, args)
#define debug(arg)  \
    if (MPEG_debug) \
    dbg() << arg

namespace Audio {

MpegAudioLoader::MpegAudioLoader(BinaryStream& stream)
    : m_stream(stream)
{
}

// Usage: Bitrate[descriptor-1][version-1][layer-1]
static constexpr u32 bitrate_index[14][2][3] = {
    // Pick this                                            If you see this
    { { 32000, 32000, 32000 }, { 32000, 8000, 8000 } },         // 0b0001
    { { 64000, 48000, 40000 }, { 48000, 16000, 16000 } },       // 0b0010
    { { 96000, 56000, 48000 }, { 56000, 24000, 24000 } },       // 0b0011
    { { 128000, 64000, 56000 }, { 64000, 32000, 32000 } },      // 0b0100
    { { 160000, 80000, 64000 }, { 80000, 40000, 40000 } },      // 0b0101
    { { 192000, 96000, 80000 }, { 96000, 48000, 48000 } },      // 0b0110
    { { 224000, 112000, 96000 }, { 112000, 56000, 56000 } },    // 0b0111
    { { 256000, 128000, 112000 }, { 128000, 64000, 64000 } },   // 0b1000
    { { 288000, 160000, 128000 }, { 144000, 80000, 80000 } },   // 0b1001
    { { 320000, 192000, 160000 }, { 160000, 96000, 96000 } },   // 0b1010
    { { 352000, 224000, 192000 }, { 176000, 112000, 112000 } }, // 0b1011
    { { 384000, 256000, 224000 }, { 192000, 128000, 128000 } }, // 0b1100
    { { 416000, 320000, 256000 }, { 224000, 144000, 144000 } }, // 0b1101
    { { 448000, 384000, 320000 }, { 256000, 160000, 160000 } }  // 0b1110
};

// Usage: Frequency[descriptor][version-1]
static constexpr u32 frequency_index[3][3] = {
    { 44100, 22050, 11025 },
    { 48000, 24000, 12000 },
    { 32000, 16000, 8000 }
};

bool MpegAudioLoader::is_bad_frame_header()
{
    FrameSpec& frame_spec = m_context.frame_spec;
    if (frame_spec.is_protected) {
        m_stream >> frame_spec.crc[0] >> frame_spec.crc[1];
        if (m_stream.handle_read_failure())
            return true;
    }

    // TODO: CRC check.

    if (frame_spec.layer != 2)
        return false;

    if (frame_spec.channel_mode == CHANNEL_MODE::MONO) {
        switch (frame_spec.bitrate) {
        case 224000:
        case 256000:
        case 320000:
        case 384000:
            return true;
        }
    }

    if (frame_spec.channel_mode == CHANNEL_MODE::STEREO
        || frame_spec.channel_mode == CHANNEL_MODE::DUAL
        || frame_spec.channel_mode == CHANNEL_MODE::JOINT_STEREO) {
        switch (frame_spec.bitrate) {
        case 32000:
        case 48000:
        case 56000:
        case 80000:
            if (frame_spec.channel_mode == CHANNEL_MODE::STEREO
                || frame_spec.channel_mode == CHANNEL_MODE::DUAL)
                return true;

            // Bitrates not allowed only in intensity stereo mode.
            if (frame_spec.mode_extension.stereo_descriptor == 0x01
                || frame_spec.mode_extension.stereo_descriptor == 0x03)
                return true;
        }
    }

    return false;
}

inline bool MpegAudioLoader::read_bitrate()
{
    FrameSpec& frame_spec = m_context.frame_spec;
    auto bitrate_descriptor = m_stream.read_network_order_uint(4);
    if (bitrate_descriptor <= 0x00 || bitrate_descriptor >= 0x0F) {
        dbg() << String::format("%d: invalid bitrate descriptor: %d", m_stream.offset(), bitrate_descriptor);
        return false;
    }
    frame_spec.bitrate = bitrate_index[bitrate_descriptor - 1][frame_spec.mpeg_version - 1][frame_spec.layer - 1];
    return true;
}

bool MpegAudioLoader::read_frame_header()
{
    FrameSpec& frame_spec = m_context.frame_spec;
    if (!m_stream.ensure_bytes(4)) {
        dbg() << String::format("%d: insufficient bytes for frame header!", m_stream.offset());
        return false;
    }

    if (m_stream.read_u8() != 0xFF || m_stream.read_network_order_uint(3) < 0x07) {
        dbg() << String::format("%d: invalid sync word!", m_stream.offset());
        return false;
    }

    if (frame_spec.mpeg_version != 0) {
        // Frame headers in the audio file remain identical to the first frame except the bitrate and maybe padding.
        m_stream.skip_bits(5);
        if (!read_bitrate()) // 4 bits.
            return false;
        m_stream.skip_bits(2);
        frame_spec.has_padding = (u8)m_stream.read_network_order_uint(1) != 0;
        m_stream.skip_bits(9);
        return true;
    }

    auto version_descriptor = m_stream.read_network_order_uint(2);
    switch (version_descriptor) {
    case 0x00:
        frame_spec.is_mpeg25 = true;
        [[fallthrough]];
    case 0x02:
        frame_spec.mpeg_version = 2;
        break;
    case 0x03:
        frame_spec.mpeg_version = 1;
        break;
    default:
        dbg() << String::format("%d: unknown version descriptor=%d", m_stream.offset(), version_descriptor);
        return false;
    }

    auto layer_descriptor = m_stream.read_network_order_uint(2);
    if (layer_descriptor > 3) {
        dbg() << String::format("%d: unknown layer descriptor=%d", m_stream.offset(), layer_descriptor);
        return false;
    }
    frame_spec.layer = (u8)(4 - layer_descriptor);

    frame_spec.is_protected = m_stream.read_network_order_uint(1) == 0;

    // Read bitrate (4 bits).
    if (!read_bitrate())
        return false;

    auto frequency_descriptor = m_stream.read_network_order_uint(2);
    frame_spec.sample_rate = frequency_index[frequency_descriptor][(frame_spec.mpeg_version + frame_spec.is_mpeg25) - 1];

    frame_spec.has_padding = (u8)m_stream.read_network_order_uint(1) != 0;

    // Ignore private bit.
    m_stream.skip_bits(1);

    auto channel_descriptor = m_stream.read_network_order_uint(2);
    if (channel_descriptor > 0x03) {
        dbg() << String::format("%d: invalid channel descriptor!", m_stream.offset());
        return false;
    }
    frame_spec.channel_mode = (CHANNEL_MODE)channel_descriptor;

    frame_spec.mode_extension.frequency_range_descriptor = (u8)m_stream.read_network_order_uint(2);
    if (frame_spec.mode_extension.frequency_range_descriptor > 0x03) {
        dbg() << String::format("%d: unknown mode extension descriptor=%d", m_stream.offset(), frame_spec.mode_extension.frequency_range_descriptor);
        return false;
    }

    frame_spec.is_copyright_protected = (u8)m_stream.read_network_order_uint(1);
    frame_spec.is_original = m_stream.read_network_order_uint(1) != 0;
    frame_spec.emphasis_descriptor = (u8)m_stream.read_network_order_uint(2);
    return true;
}

bool MpegAudioLoader::read_side_info()
{
    FrameSpec& frame_spec = m_context.frame_spec;
    bool is_mono = frame_spec.channel_mode == CHANNEL_MODE::MONO;
    if (!m_stream.ensure_bytes(is_mono ? 17 : 32)) {
        dbg() << String::format("%d: insufficient bytes remaining for side info!", m_stream.offset());
        return false;
    }

    int channel_count = is_mono ? 1 : 2;

    frame_spec.main_data_begin = (u16)m_stream.read_network_order_uint(9);

    // Skip private info bits.
    m_stream.skip_bits(is_mono ? 5 : 3);

    // Scale factor sharing policies for each band.
    for (int i = 0; i < channel_count; i++) {
        bool* channel = frame_spec.sfs_policy_index[i];
        for (int j = 0; j < 4; j++)
            channel[j] = m_stream.read_network_order_uint(1) != 0;
    }

    return true;
}

bool MpegAudioLoader::parse_frame()
{
    if (!read_frame_header()) {
        dbg() << String::format("%d: unable to parse frame header!", m_stream.offset());
        return false;
    }

    FrameSpec& frame = m_context.frame_spec;
    if (is_bad_frame_header()) {
        dbg() << String::format("%d: bad frame header!", m_stream.offset());
        return false;
    }

    auto slot_size = frame.layer == 1 ? 4 : 1;
    frame.length = ((144 * frame.bitrate) / frame.sample_rate) + (frame.has_padding ? slot_size : 0);
    debug_fmt("MPEG audio frame length=%dB", frame.length);

    if (!read_side_info()) {
        dbg() << String::format("%d: corrupt frame!", m_stream.offset());
        return false;
    }

    return true;
}

bool MpegAudioLoader::decode_mpeg_audio()
{
    if (!parse_id3(m_stream, m_context)) {
        dbg() << String::format("%d: couldn't parse ID3 header!", m_stream.offset());
        return false;
    }

    if (!parse_frame()) {
        dbg() << String::format("%d: couldn't parse frame!", m_stream.offset());
        return false;
    }

    return true;
}
}
