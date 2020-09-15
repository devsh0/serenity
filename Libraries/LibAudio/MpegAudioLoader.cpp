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
#include "MpegTables.h"

#define MPEG_DEBUG 1
#define debug_fmt(fmt, args...) \
    if (MPEG_DEBUG)             \
    dbg() << String::format(fmt, args)
#define debug(arg)  \
    if (MPEG_DEBUG) \
    dbg() << arg

namespace Audio {

bool MpegAudioLoader::is_corrupt_frame_header()
{
    FrameSpec& spec = m_context.frame_spec;
    if (spec.is_protected) {
        m_stream >> spec.crc[0] >> spec.crc[1];
        if (m_stream.handle_read_failure())
            return true;
    }
    // TODO: CRC16 validation.

    // These constraints are only applicable if we're dealing with layer 2 audio.
    if (spec.layer != 2)
        return false;

    // Bitrates not allowed in layer 2 MONO.
    if (spec.channel_mode == CHANNEL_MODE::MONO) {
        switch (spec.bitrate) {
        case 224000:
        case 256000:
        case 320000:
        case 384000:
            return true;
        }
    }

    // Bitrates not allowed in layer 2 DUAL/STEREO.
    if (spec.channel_mode == CHANNEL_MODE::STEREO || spec.channel_mode == CHANNEL_MODE::DUAL || spec.channel_mode == CHANNEL_MODE::JOINT_STEREO) {
        switch (spec.bitrate) {
        case 32000:
        case 48000:
        case 56000:
        case 80000:
            if (spec.channel_mode == CHANNEL_MODE::STEREO || spec.channel_mode == CHANNEL_MODE::DUAL)
                return true;

            // Bitrates not allowed only in intensity stereo mode.
            if (spec.mode_extension.stereo_descriptor == 0x01 || spec.mode_extension.stereo_descriptor == 0x03)
                return true;
        }
    }

    return false;
}

inline bool MpegAudioLoader::bitrate_read_helper()
{
    FrameSpec& frame_spec = m_context.frame_spec;
    auto bitrate_descriptor = m_stream.read_network_order_bits(4);
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
        dbg() << m_stream.offset() << ": insufficient bytes for frame header!";
        return false;
    }

    if (m_stream.read_network_order_u8() != 0xFF || m_stream.read_network_order_bits(3) < 0x07) {
        dbg() << m_stream.offset() << ": invalid sync word!";
        return false;
    }

    // Frame headers in the audio file don't differ much from the first frame.
    if (frame_spec.mpeg_version != 0) {
        m_stream.skip_bits(5);
        // Read bitrate (4 bits).
        if (!bitrate_read_helper())
            return false;
        m_stream.skip_bits(2);
        frame_spec.has_padding = (u8)m_stream.read_network_order_bits(1) != 0;
        m_stream.skip_bits(9);
        return true;
    }

    auto version_descriptor = m_stream.read_network_order_bits(2);
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

    auto layer_descriptor = m_stream.read_network_order_bits(2);
    if (layer_descriptor > 3) {
        dbg() << String::format("%d: unknown layer descriptor=%d", m_stream.offset(), layer_descriptor);
        return false;
    }
    frame_spec.layer = (u8)(4 - layer_descriptor);
    frame_spec.is_protected = m_stream.read_network_order_bits(1) == 0;

    // Read bitrate (4 bits).
    if (!bitrate_read_helper())
        return false;
    auto frequency_descriptor = m_stream.read_network_order_bits(2);
    frame_spec.sample_rate = frequency_index[frequency_descriptor][(frame_spec.mpeg_version + frame_spec.is_mpeg25) - 1];

    // Frame is padded with 1 extra byte if the padding bit is NOT set.
    frame_spec.has_padding = (u8)m_stream.read_network_order_bits(1) != 0;

    // Ignore private bit.
    m_stream.skip_bits(1);
    auto channel_descriptor = m_stream.read_network_order_bits(2);
    if (channel_descriptor > 0x03) {
        dbg() << m_stream.offset() << ": invalid channel descriptor!";
        return false;
    }
    frame_spec.channel_mode = (CHANNEL_MODE)channel_descriptor;
    u8 frd = (u8)m_stream.read_network_order_bits(2);
    if (frd > 0x03) {
        dbg() << String::format("%d: unknown mode extension descriptor=%d", m_stream.offset(), frd);
        return false;
    }
    frame_spec.mode_extension.frequency_range_descriptor = frd;
    frame_spec.is_copyright_protected = (u8)m_stream.read_network_order_bits(1);
    frame_spec.is_original = m_stream.read_network_order_bits(1) != 0;
    frame_spec.emphasis_descriptor = (u8)m_stream.read_network_order_bits(2);
    return true;
}

bool MpegAudioLoader::read_side_info()
{
    static constexpr u8 scalefactor_compress_index[16][2] = {
        { 0, 0 }, { 0, 1 }, { 0, 2 }, { 0, 3 }, { 3, 0 }, { 1, 1 }, { 1, 2 }, { 1, 3 },
        { 2, 1 }, { 2, 2 }, { 2, 3 }, { 3, 1 }, { 3, 2 }, { 3, 3 }, { 4, 2 }, { 4, 3 }
    };

    FrameSpec& frame_spec = m_context.frame_spec;
    bool is_mono = frame_spec.channel_mode == CHANNEL_MODE::MONO;
    if (!m_stream.ensure_bytes(is_mono ? 17 : 32)) {
        dbg() << m_stream.offset() << ": insufficient bytes remaining for side info!";
        return false;
    }

    int channel_count = is_mono ? 1 : 2;

    frame_spec.main_data_begin = (u16)m_stream.read_network_order_bits(9);

    // Skip private info bits.
    m_stream.skip_bits(is_mono ? 5 : 3);

    // Scale factor sharing policies for each subband.
    for (int channel = 0; channel < channel_count; channel++) {
        bool* ch_ptr = frame_spec.scfsi[channel];
        for (int subband = 0; subband < 4; subband++)
            ch_ptr[subband] = m_stream.read_network_order_bits(1) != 0;
    }

    for (int granule = 0; granule < 2; granule++) {
        for (int channel = 0; channel < channel_count; channel++) {
            SideInfoSpec side_info;
            side_info.part_23_length = (u16)m_stream.read_network_order_bits(12);
            side_info.big_value_region_length = (u16)m_stream.read_network_order_bits(9);
            side_info.global_gain = (u8)m_stream.read_network_order_bits(8);
            side_info.scalefactor_compress = (u8)m_stream.read_network_order_bits(4);
            side_info.slen1 = scalefactor_compress_index[side_info.scalefactor_compress][0];
            side_info.slen2 = scalefactor_compress_index[side_info.scalefactor_compress][1];
            side_info.window_switching_flag = (u8)m_stream.read_network_order_bits(1);

            if (side_info.window_switching_flag) {
                side_info.block_type = (u8)m_stream.read_network_order_bits(2);
                if (side_info.block_type == 0) {
                    const char* format = "%d: window_switching_flag=%d, block_type=%d";
                    dbg() << String::format(format, m_stream.offset(), side_info.window_switching_flag, side_info.block_type);
                    return false;
                }
                side_info.mixed_block_flag = (u8)m_stream.read_network_order_bits(1);
                side_info.window_switch_point_long = (u8)(side_info.mixed_block_flag * 8);
                side_info.window_switch_point_short = (u8)(side_info.mixed_block_flag * 3);
                side_info.region0_count = (u8)(7 + (side_info.block_type == 2 && side_info.mixed_block_flag));

                // The standard suggests this value should be 63. But source code of many implementations
                // suggest that the standard is wrong on this.
                side_info.region1_count = (u8)(20 - side_info.region0_count);

                // region2 is empty when window switching is enabled.
                for (int i = 0; i < 2; i++)
                    side_info.table_select[i] = (u8)m_stream.read_network_order_bits(5);

                // Gain offset from the global gain for each short block window.
                for (int i = 0; i < 3; i++)
                    side_info.subblock_gain[i] = (u8)m_stream.read_network_order_bits(3);

            } else {
                // Window switching is disabled. All three regions have codes.
                for (int i = 0; i < 3; i++)
                    side_info.table_select[i] = (u8)m_stream.read_network_order_bits(5);

                side_info.region0_count = (u8)m_stream.read_network_order_bits(4);
                side_info.region1_count = (u8)m_stream.read_network_order_bits(3);
            }

            side_info.preflag = (u8)m_stream.read_network_order_bits(1);
            side_info.scalefactor_scale = (u8)m_stream.read_network_order_bits(1);
            side_info.count1_table_select = (u8)m_stream.read_network_order_bits(1);

            frame_spec.side_info[granule][channel] = side_info;
        }
    }
    return true;
}

bool MpegAudioLoader::shift_main_data_into_reservoir()
{
    auto& context = m_context;
    auto& frame_spec = context.frame_spec;
    auto& reservoir = context.reservoir;
    bool is_mono = frame_spec.channel_mode == CHANNEL_MODE::MONO;

    // main_data_length = frame length - (header length + CRC length + sideinfo length)
    context.main_data_length = frame_spec.length - (4 + (frame_spec.is_protected * 2) + (is_mono ? 17 : 32));
    if (!m_stream.ensure_bytes(context.main_data_length)) {
        dbg() << m_stream.offset() << ": Insufficient bytes remaining for main data!";
        return false;
    }

    Vector<u8> main_data;
    main_data.ensure_capacity(context.main_data_length);

    for (unsigned i = 0; i < context.main_data_length; i++)
        // We are byte aligned at this point. Byte order doesn't really matter.
        main_data.append(m_stream.read_network_order_u8());

    // FIXME: We never shift out redundant main data. This is heavy on memory.
    context.main_data_begin_index = reservoir.size() - frame_spec.main_data_begin;
    context.can_decode_frame = m_context.main_data_begin_index >= 0;
    reservoir.append(move(main_data));

    return true;
}

Vector<u32> MpegAudioLoader::huffman_decode(unsigned table)
{
}

// FIXME: This is a placeholder. The logic of this function doesn't make sense.
size_t get_main_pos()
{
    return 10;
}

// FIXME: This is a placeholder. The logic of this function doesn't make sense.
int set_main_pos(MpegAudioContext& context, size_t position)
{
    return (int)(context.long_window_sfband[0][0][0] + position);
}

void MpegAudioLoader::extract_huffman_data(int granule, int channel)
{
    auto& context = m_context;
    auto& side_info = context.frame_spec.side_info[granule][channel];

    // There's nothing to decode.
    if (side_info.part_23_length == 0)
        return;

    int region0 = 0;
    int region1 = 0;
    if (side_info.window_switching_flag && side_info.block_type == 2) {
        region0 = 36;
        region1 = 576;
    } else {
        region0 = band_index[context.frame_spec.sample_rate].long_bands[side_info.region0_count + 1];
        region1 = band_index[context.frame_spec.sample_rate].long_bands[side_info.region1_count + 1];
    }

    unsigned table = 0;
    Vector<u32> decoded;

    // Read big values.
    for (size_t sample = 0; sample < side_info.big_value_region_length * 2; sample++) {
        int index = sample < region0 ? 0 : (sample < region1 ? 1 : 2);
        table = side_info.table_select[index];
        decoded = huffman_decode(table);
        context.samples[granule][channel][sample++] = decoded[0];
        context.samples[granule][channel][sample] = decoded[1];
    }

    // Read count1/quad values.
    table = side_info.count1_table_select + 32;
    size_t sample = side_info.big_value_region_length * 2u;
    size_t bit_pos_end = 100; // FIXME: This is a placeholder.
    for (; sample <= 572 && get_main_pos() <= bit_pos_end; sample++) {
        decoded = huffman_decode(table);
        context.samples[granule][channel][sample++] = decoded[2];
        if (sample >= 576)
            break;
        context.samples[granule][channel][sample++] = decoded[3];
        if (sample >= 576)
            break;
        context.samples[granule][channel][sample++] = decoded[0];
        if (sample >= 576)
            break;
        context.samples[granule][channel][sample] = decoded[1];
    }

    // Check that we didn't read past the end of this section.
    bit_pos_end += 1;
    sample -= 4 * (get_main_pos() > bit_pos_end);
    context.count1[granule][channel] = sample;
    set_main_pos(context, bit_pos_end);
}

bool MpegAudioLoader::extract_scalefactors()
{
    // Only used for long blocks.
    static constexpr u8 group_to_band_range[4][2] = { { 0, 5 }, { 6, 10 }, { 11, 15 }, { 16, 20 } };

    auto& context = m_context;
    auto& frame_spec = context.frame_spec;
    int channel_count = 1 + (frame_spec.channel_mode != CHANNEL_MODE::MONO);
    // FIXME: Modify this when you start removing redundant main data from the reservoir.
    ByteBuffer buffer = ByteBuffer::wrap((context.reservoir.data() + context.main_data_begin_index), context.main_data_length);
    BinaryStream stream { buffer };

    auto read_long_scalefactors = [&](int granule, int channel, int group) {
        auto& side_info = context.frame_spec.side_info[granule][channel];
        int start_band = group_to_band_range[group][0];
        int end_band = group_to_band_range[group][1];
        for (int band = start_band; band <= end_band; band++) {
            u8 bits = band <= 10 ? side_info.slen1 : side_info.slen2;
            context.long_window_sfband[granule][channel][band] = (u8)stream.read_network_order_bits(bits);
        }
    };

    auto copy_long_scalefactors = [&](int granule, int channel, int group) {
        int start_band = group_to_band_range[group][0];
        int end_band = group_to_band_range[group][1];
        for (int band = start_band; band <= end_band; band++)
            context.long_window_sfband[granule][channel][band] = context.long_window_sfband[granule - 1][channel][band];
    };

    for (int granule = 0; granule < 2; granule++) {
        for (int channel = 0; channel < channel_count; channel++) {
            auto& side_info = frame_spec.side_info[granule][channel];
            // For short blocks, scalefactors are transmitted for each granule.
            if (side_info.block_type == 2 && side_info.window_switching_flag) {
                if (side_info.mixed_block_flag) {
                    // Short Blocks, mixed_block_flag set. `slen1` applies to long window scalefactor bands in the range 0-7 and short
                    // window scalefactor bands in the range 3-5. `slen2` applies to short window scalefactor bands in the range 6-11.
                    for (int band = 0; band <= 7; band++)
                        context.long_window_sfband[granule][channel][band] = (u8)stream.read_network_order_bits(side_info.slen1);
                    for (int band = 3; band <= 11; band++) {
                        u8 bits = band <= 5 ? side_info.slen1 : side_info.slen2;
                        for (int window = 0; window < 3; window++)
                            context.short_window_sfband[granule][channel][band][window] = (u8)stream.read_network_order_bits(bits);
                    }
                } else {
                    // Short block, mixed_block_flag not set. `slen1` applies to long window scalefactor bands in the rang 0-5 and `slen2`
                    // applies to short window scalefactor bands in the range 6-11.
                    for (int band = 0; band <= 11; band++) {
                        u8 bits = band <= 5 ? side_info.slen1 : side_info.slen2;
                        for (int window = 0; window < 3; window++)
                            context.short_window_sfband[granule][channel][band][window] = (u8)stream.read_network_order_bits(bits);
                    }
                }
            } else {
                // Block type 0 (Normal), 1 (Start), or 3 (Stop).
                for (int group = 0; group < 4; group++) {
                    bool scalefactors_shared = frame_spec.scfsi[channel][group];
                    if (scalefactors_shared)
                        copy_long_scalefactors(granule, channel, group);
                    else
                        read_long_scalefactors(granule, channel, group);
                }
            }

            if (stream.handle_read_failure()) {
                dbg() << "Failed to read scalefactors!";
                return false;
            }

            extract_huffman_data(granule, channel);
        }
    }
    return true;
}

bool MpegAudioLoader::extract_frame()
{
    if (!read_frame_header()) {
        dbg() << m_stream.offset() << ": unable to parse frame header!";
        return false;
    }

    FrameSpec& frame_spec = m_context.frame_spec;
    if (is_corrupt_frame_header()) {
        dbg() << m_stream.offset() << ": corrupt frame header!";
        return false;
    }

    auto slot_size = frame_spec.layer == 1 ? 4 : 1;
    frame_spec.length = ((144 * frame_spec.bitrate) / frame_spec.sample_rate) + (frame_spec.has_padding * slot_size);
    debug_fmt("MPEG audio frame length=%dB", frame_spec.length);

    if (!read_side_info()) {
        dbg() << m_stream.offset() << ": corrupt frame!";
        return false;
    }

    m_stream.byte_align_forward();
    return shift_main_data_into_reservoir();
}

bool MpegAudioLoader::decode_frame()
{
    if (!parse_id3(m_stream, m_context)) {
        dbg() << m_stream.offset() << ": couldn't parse ID3 header!";
        return false;
    }

    do {
        if (!extract_frame())
            return false;
    } while (!m_context.can_decode_frame);

    if (!extract_scalefactors())
        return false;

    return true;
}

}
