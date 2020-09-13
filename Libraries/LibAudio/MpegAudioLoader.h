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

struct SideInfoSpec {
    // Number of bits allocated in the main data segment for scale factors (part2)
    // and huffman codes (part3).
    u16 part_23_length { 0 }; // 12 bits

    // Frequency components in the granule range from 0 to Nyquist frequency. To
    // enhance encoding efficiency, different huffman tables are used to encode
    // frequencies. Frequencies are subdivided into regions covering different
    // ranges of frequency. These are Big Value Regions (low frequency content),
    // Count1 or Quad region (high frequency content), and Zero region (high
    // frequencies, quantized down to zero). This field stores the size of big
    // value region.
    u16 big_value_region_length { 0 }; // 9 bits

    // Quantization step size. Will be required while requantizing samples.
    u8 global_gain { 0 }; // 8 bits

    // Number of bits used for scalefactor bands.
    u8 scalefactor_compress { 0 }; // 4 bits

    u8 slen1 { 0 };
    u8 slen2 { 0 };

    // Whether window switching is enabled to switch between windows of long and short blocks.
    u8 window_switching_flag { 0 }; // 1 bit

    u8 window_switch_point_long { 0 };
    u8 window_switch_point_short { 0 };

    // Long or Short block type.
    u8 block_type { 0 }; // 2 bits

    // Specifies whether long and short blocks have been mixed in this granule.
    u8 mixed_block_flag { 0 }; // 1 bit

    // This field specifies which huffman code table, out of all those defined
    // in the spec, is to be used for the 3 big value regions.
    u8 table_select[3] = { 0 }; // 5 bits per region.

    // For each short block, indicates the gain offset from the global gain. Only
    // used when block type = 2 (short block), but transmitted regardless of the
    // value of block type.
    u8 subblock_gain[3] = { 0 }; // 3 short block windows, 3 bits each.

    u8 region0_count { 0 }; // 4 bits

    u8 region1_count { 0 }; // 3 bits

    // If this field is set, a value picked from a table is added to the scalefactors.
    u8 preflag { 0 }; // 1 bit

    u8 scalefactor_scale { 0 }; // 1 bit

    // Huffman code table for the count1 region.
    u8 count1_table_select { 0 }; // 1 bit
};

struct FrameSpec {
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

    // Side info.
    u16 main_data_begin { 0 }; // 9 bits.

    // scfsi[ch][scfsi_band] - Stores the scale factor selection information and
    // works similarly to Layers I and II. The main difference is the use of the
    // variable scfsi_band​to apply scfsi to groups of scale factors instead of
    // single scale factor. scfsi controls the use of scale factors to the granules.
    // 4 bits are transmitted per channel. If the bit at position x for band X is set,
    // then scale factor for that band is transmitted for one granule and the other
    // granule uses the same factors.
    bool scfsi[2][4] = { false };

    // side_info[granule][channel].
    SideInfoSpec side_info[2][2];
};

struct MpegAudioContext {
    FrameSpec frame_spec;
    Vector<u8> reservoir;
    ssize_t main_data_begin_index { 0 };
    size_t main_data_length { 0 };
    bool can_decode_frame { false };

    // Long window scale factor bands. factor[granule][chanel][band];
    u8 long_window_sfband[2][2][21] = { 0 };
    // Short window scale factor bands. factor[granule][channel][band][window]
    u8 short_window_sfband[2][2][12][3] = { 0 };

    String id3_string;
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
    bool decode_frame();

private:
    bool extract_frame();
    bool read_frame_header();
    bool bitrate_read_helper();
    bool is_corrupt_frame_header();
    bool read_side_info();
    bool shift_main_data_into_reservoir();
    bool extract_main_data();

    MpegAudioContext m_context;
    BinaryStream& m_stream;
    bool extract_scalefactors();
};
}
