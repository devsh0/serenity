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

#pragma once

#include <AK/BinaryStream.h>
#include <AK/String.h>
#include <AK/Vector.h>

namespace Audio {

enum class CHANNEL_MODE {
    STEREO = 0x00,
    JOINT_STEREO,
    DUAL,
    MONO
};

struct FrameSpec {
    // The header part.
    u8 mpeg_version { 0 };
    bool is_mpeg25 { false };
    u8 layer { 0 };
    bool is_protected { false };
    bool has_padding { false };
    u32 bitrate { 0 };
    u32 sample_rate { 0 };
    CHANNEL_MODE channel_mode;
    union {
        // Value   Layer 1 & 2      Layer 3-IS      Layer 3-MS
        // 00 	   bands 04 to 31   off             off
        // 01 	   bands 08 to 31   on              off
        // 10 	   bands 12 to 31   off             on
        // 11 	   bands 16 to 31   on              on
        u8 frequency_range_descriptor { 0 };
        u8 stereo_descriptor;
    } mode_extension;
    bool is_copyright_protected { false };
    bool is_original { false };
    u8 emphasis_descriptor = { 0 };

    u8 crc[2] = { 0 };
    u32 length { 0 };

    // The side info part.
    u16 main_data_begin { 0 };               // 9 bits.
    bool sfs_policy_index[2][4] = { false }; // Scale factor sharing policy index.
};

struct MpegAudioContext {
    FrameSpec frame_spec;

    String id3_string = "ID3v2";
    u8 id3_flags { 0 };
    String title;
    String year;
    String album_title;
    String cover_art_mime;
    Vector<u8> cover_art;
};

class MpegAudioLoader {
public:

    MpegAudioLoader(BinaryStream&);
    bool decode_mpeg_audio();
private:
    bool parse_frame();
    bool read_frame_header();
    bool read_bitrate();
    bool is_bad_frame_header();
    bool read_side_info();

    MpegAudioContext m_context;
    BinaryStream& m_stream;
};
}
