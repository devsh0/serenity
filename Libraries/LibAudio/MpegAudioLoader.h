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
#include <AK/CircularQueue.h>
#include <AK/String.h>
#include <AK/Vector.h>

namespace Audio {

enum class CHANNEL_MODE {
    STEREO = 0x00,
    JOINT_STEREO,
    DUAL,
    MONO
};

struct Frame {
    struct Header {
        u8 mpeg_version { 0 };
        bool is_mpeg25 { false };
        u8 layer { 0 };
        bool is_protected { false };
        bool has_padding { false };
        u32 bitrate { 0 };
        u32 sample_rate { 0 };
        CHANNEL_MODE channel_mode;

        union {
            // Value       Layer 1 & 2      Layer 3-IS      Layer 3-MS
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
    } header;

    u8 crc[2] = { 0 };
    u32 length { 0 };

    u16 main_data_begin { 0 }; // 9 bits.
    ssize_t main_data_begin_index { 0 };
    size_t main_data_length { 0 };
    bool is_decodable { false };
    bool scfsi[2][4] = { false };

    // Data[granule][channel]. Terrible name, but can't think of a better one.
    struct Data {
        // Sideinfo for a specific channel of a specific granule.
        struct Sideinfo {
            u16 part_23_length { 0 };          // 12 bits
            u16 big_value_region_length { 0 }; // 9 bits
            u8 global_gain { 0 };              // 8 bits
            u8 scalefactor_compress { 0 };     // 4 bits

            u8 slen1 { 0 };
            u8 slen2 { 0 };

            u8 window_switching_flag { 0 }; // 1 bit
            u8 window_switch_point_long { 0 };
            u8 window_switch_point_short { 0 };

            u8 block_type { 0 };          // 2 bits
            u8 mixed_block_flag { 0 };    // 1 bit
            u8 table_select[3] = { 0 };   // 5 bits per region.
            u8 subblock_gain[3] = { 0 };  // 3 short block windows, 3 bits each.
            u8 region0_count { 0 };       // 4 bits
            u8 region1_count { 0 };       // 3 bits
            u8 preflag { 0 };             // 1 bit
            u8 scalefactor_scale { 0 };   // 1 bit
            u8 count1_table_select { 0 }; // 1 bit
        } side_info;
        u8 long_window_sfband[21] = { 0 };
        u8 short_window_sfband[12][3] = { 0 };
        float samples[576] = { 0.0 };
        float count1 = { 0.0 };
    } data[2][2];
};

struct AudioMeta {
    String id3_string;
    u8 id3_flags { 0 };
    String title;
    String year;
    String album_title;
    String cover_art_mime;
    Vector<u8> cover_art;
};

struct MpegAudioContext {
    Frame current_frame;
    Vector<u8> reservoir;
    AudioMeta metadata;
};

class MpegAudioLoader {
public:
    explicit MpegAudioLoader(BinaryStream& stream)
        : m_stream(stream)
    {
    }

    bool decode_frame();

private:
    bool extract_frame();
    bool read_frame_header();
    bool bitrate_read_helper();
    bool is_corrupt_frame_header();
    bool read_side_info();
    bool shift_main_data_into_reservoir();
    bool extract_main_data();
    void extract_huffman_data(int granule, int channel);
    Vector<u32> huffman_decode(unsigned table);
    bool extract_scalefactors();

    MpegAudioContext m_context;
    BinaryStream& m_stream;

    // FIXME: Temporaries.
    size_t get_main_pos();
    int set_main_pos(Frame::Data& frame_data, size_t position);
};
}
