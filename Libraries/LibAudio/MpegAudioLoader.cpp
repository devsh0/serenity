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
    auto& frame = m_context.current_frame;
    auto& frame_header = frame.header;

    if (frame_header.is_protected) {
        m_stream >> frame.crc[0] >> frame.crc[1];
        if (m_stream.handle_read_failure())
            return true;
    }
    // TODO: CRC16 validation.

    // These constraints are only applicable if we're dealing with layer 2 audio.
    if (frame_header.layer != 2)
        return false;

    // Bitrates not allowed in layer 2 MONO.
    if (frame_header.channel_mode == CHANNEL_MODE::MONO) {
        switch (frame_header.bitrate) {
        case 224000:
        case 256000:
        case 320000:
        case 384000:
            return true;
        }
    }

    // Bitrates not allowed in layer 2 DUAL/STEREO.
    if (frame_header.channel_mode == CHANNEL_MODE::STEREO || frame_header.channel_mode == CHANNEL_MODE::DUAL || frame_header.channel_mode == CHANNEL_MODE::JOINT_STEREO) {
        switch (frame_header.bitrate) {
        case 32000:
        case 48000:
        case 56000:
        case 80000:
            if (frame_header.channel_mode == CHANNEL_MODE::STEREO || frame_header.channel_mode == CHANNEL_MODE::DUAL)
                return true;

            // Bitrates not allowed only in intensity stereo mode.
            if (frame_header.mode_extension.stereo_descriptor == 0x01 || frame_header.mode_extension.stereo_descriptor == 0x03)
                return true;
        }
    }

    return false;
}

inline bool MpegAudioLoader::bitrate_read_helper()
{
    auto& frame_header = m_context.current_frame.header;
    auto bitrate_descriptor = m_stream.read_network_order_bits(4);
    if (bitrate_descriptor <= 0x00 || bitrate_descriptor >= 0x0F) {
        dbg() << String::format("%d: invalid bitrate descriptor: %d", m_stream.offset(), bitrate_descriptor);
        return false;
    }
    frame_header.bitrate = bitrate_index[bitrate_descriptor - 1][frame_header.mpeg_version - 1][frame_header.layer - 1];
    return true;
}

bool MpegAudioLoader::read_frame_header()
{
    auto& frame_header = m_context.current_frame.header;
    if (!m_stream.ensure_bytes(4)) {
        dbg() << m_stream.offset() << ": insufficient bytes for frame header!";
        return false;
    }

    if (m_stream.read_network_order_u8() != 0xFF || m_stream.read_network_order_bits(3) < 0x07) {
        dbg() << m_stream.offset() << ": invalid sync word!";
        return false;
    }

    // Frame headers in the audio file don't differ much from the first frame.
    if (frame_header.mpeg_version != 0) {
        m_stream.skip_bits(5);
        // Read bitrate (4 bits).
        if (!bitrate_read_helper())
            return false;
        m_stream.skip_bits(2);
        frame_header.has_padding = (u8)m_stream.read_network_order_bits(1) != 0;
        m_stream.skip_bits(9);
        return true;
    }

    auto version_descriptor = m_stream.read_network_order_bits(2);
    switch (version_descriptor) {
    case 0x00:
        frame_header.is_mpeg25 = true;
        [[fallthrough]];
    case 0x02:
        frame_header.mpeg_version = 2;
        break;
    case 0x03:
        frame_header.mpeg_version = 1;
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
    frame_header.layer = (u8)(4 - layer_descriptor);
    frame_header.is_protected = m_stream.read_network_order_bits(1) == 0;

    // Read bitrate (4 bits).
    if (!bitrate_read_helper())
        return false;
    auto frequency_descriptor = m_stream.read_network_order_bits(2);
    frame_header.sample_rate = frequency_index[frequency_descriptor][(frame_header.mpeg_version + frame_header.is_mpeg25) - 1];

    // Frame is padded with 1 extra byte if the padding bit is NOT set.
    frame_header.has_padding = (u8)m_stream.read_network_order_bits(1) != 0;

    // Ignore private bit.
    m_stream.skip_bits(1);
    auto channel_descriptor = m_stream.read_network_order_bits(2);
    if (channel_descriptor > 0x03) {
        dbg() << m_stream.offset() << ": invalid channel descriptor!";
        return false;
    }
    frame_header.channel_mode = (CHANNEL_MODE)channel_descriptor;
    u8 frd = (u8)m_stream.read_network_order_bits(2);
    if (frd > 0x03) {
        dbg() << String::format("%d: unknown mode extension descriptor=%d", m_stream.offset(), frd);
        return false;
    }
    frame_header.mode_extension.frequency_range_descriptor = frd;
    frame_header.is_copyright_protected = (u8)m_stream.read_network_order_bits(1);
    frame_header.is_original = m_stream.read_network_order_bits(1) != 0;
    frame_header.emphasis_descriptor = (u8)m_stream.read_network_order_bits(2);
    return true;
}

bool MpegAudioLoader::read_side_info()
{
    static constexpr u8 scalefactor_compress_index[16][2] = {
        { 0, 0 }, { 0, 1 }, { 0, 2 }, { 0, 3 }, { 3, 0 }, { 1, 1 }, { 1, 2 }, { 1, 3 },
        { 2, 1 }, { 2, 2 }, { 2, 3 }, { 3, 1 }, { 3, 2 }, { 3, 3 }, { 4, 2 }, { 4, 3 }
    };

    auto& frame = m_context.current_frame;
    bool is_mono = frame.header.channel_mode == CHANNEL_MODE::MONO;
    if (!m_stream.ensure_bytes(is_mono ? 17 : 32)) {
        dbg() << m_stream.offset() << ": insufficient bytes remaining for side info!";
        return false;
    }

    int channel_count = is_mono ? 1 : 2;

    frame.main_data_begin = (u16)m_stream.read_network_order_bits(9);

    // Skip private info bits.
    m_stream.skip_bits(is_mono ? 5 : 3);

    // Scale factor sharing policies for each subband.
    for (int channel = 0; channel < channel_count; channel++) {
        bool* ch_ptr = frame.scfsi[channel];
        for (int subband = 0; subband < 4; subband++)
            ch_ptr[subband] = m_stream.read_network_order_bits(1) != 0;
    }

    for (int granule = 0; granule < 2; granule++) {
        for (int channel = 0; channel < channel_count; channel++) {
            Frame::Data::Sideinfo side_info;
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

            frame.data[granule][channel].side_info = side_info;
        }
    }
    return true;
}

bool MpegAudioLoader::shift_main_data_into_reservoir()
{
    auto& frame = m_context.current_frame;
    auto& reservoir = m_context.reservoir;
    bool is_mono = frame.header.channel_mode == CHANNEL_MODE::MONO;

    // main_data_length = frame length - (header length + [CRC length] + sideinfo length)
    frame.main_data_length = frame.length - (4 + (frame.header.is_protected * 2) + (is_mono ? 17 : 32));
    if (!m_stream.ensure_bytes(frame.main_data_length)) {
        dbg() << m_stream.offset() << ": Insufficient bytes remaining for main data!";
        return false;
    }

    Vector<u8> main_data;
    main_data.ensure_capacity(frame.main_data_length);

    // We are byte aligned at this point. Byte order doesn't really matter.
    for (unsigned i = 0; i < frame.main_data_length; i++)
        main_data.append(m_stream.read_network_order_u8());

    // FIXME: We never shift out redundant main data. This is heavy on memory.
    frame.main_data_begin_index = reservoir.size() - frame.main_data_begin;
    frame.is_decodable = frame.main_data_begin_index >= 0;
    reservoir.append(move(main_data));
    return true;
}

// FIXME: This is a placeholder. The logic of this function doesn't make sense.
Vector<u32> MpegAudioLoader::huffman_decode(unsigned table)
{
    return {table};
}

// FIXME: This is a placeholder. The logic of this function doesn't make sense.
size_t MpegAudioLoader::get_main_pos()
{
    return 10;
}

// FIXME: This is a placeholder. The logic of this function doesn't make sense.
int MpegAudioLoader::set_main_pos(Frame::Data& frame_data, size_t position)
{
    return (int)(frame_data.long_window_sfband[0] + position);
}

void MpegAudioLoader::extract_huffman_data(int granule, int channel)
{
    auto& current_frame = m_context.current_frame;
    auto& frame_data = current_frame.data[granule][channel];
    auto& side_info = frame_data.side_info;

    // There's nothing to decode.
    if (side_info.part_23_length == 0)
        return;

    size_t region0 = 0;
    size_t region1 = 0;
    if (side_info.window_switching_flag && side_info.block_type == 2) {
        region0 = 36;
        region1 = 576;
    } else {
        region0 = band_index[current_frame.header.sample_rate].long_bands[side_info.region0_count + 1];
        region1 = band_index[current_frame.header.sample_rate].long_bands[side_info.region1_count + 1];
    }

    unsigned table = 0;
    Vector<u32> decoded;

    // Read big values.
    for (size_t sample = 0; sample < side_info.big_value_region_length * 2; sample++) {
        int index = sample < region0 ? 0 : (sample < region1 ? 1 : 2);
        table = side_info.table_select[index];
        decoded = huffman_decode(table);
        frame_data.samples[sample++] = decoded[0];
        frame_data.samples[sample] = decoded[1];
    }

    // Read count1/quad values.
    table = side_info.count1_table_select + 32;
    size_t sample = side_info.big_value_region_length * 2u;
    size_t bit_pos_end = 100; // FIXME: This is a placeholder.
    for (; sample <= 572 && get_main_pos() <= bit_pos_end; sample++) {
        decoded = huffman_decode(table);
        frame_data.samples[sample++] = decoded[2];
        if (sample >= 576)
            break;
        frame_data.samples[sample++] = decoded[3];
        if (sample >= 576)
            break;
        frame_data.samples[sample++] = decoded[0];
        if (sample >= 576)
            break;
        frame_data.samples[sample] = decoded[1];
    }

    // Check that we didn't read past the end of this section.
    bit_pos_end += 1;
    sample -= 4 * (get_main_pos() > bit_pos_end);
    frame_data.count1 = sample;
    set_main_pos(frame_data, bit_pos_end);
}

bool MpegAudioLoader::extract_scalefactors()
{
    // Only used for long blocks.
    static constexpr u8 subband_range_index[4][2] = { { 0, 5 }, { 6, 10 }, { 11, 15 }, { 16, 20 } };

    auto& frame = m_context.current_frame;
    int channel_count = 1 + (frame.header.channel_mode != CHANNEL_MODE::MONO);
    // FIXME: Modify this when you start removing redundant main data from the reservoir.
    ByteBuffer buffer = ByteBuffer::wrap((m_context.reservoir.data() + frame.main_data_begin_index), frame.main_data_length);
    BinaryStream stream { buffer };

    auto read_long_scalefactors = [&](int granule, int channel, int subband) {
        auto& frame_data = frame.data[granule][channel];
        int start_band = subband_range_index[subband][0];
        int end_band = subband_range_index[subband][1];
        for (int band = start_band; band <= end_band; band++) {
            u8 bits = band <= 10 ? frame_data.side_info.slen1 : frame_data.side_info.slen2;
            frame_data.long_window_sfband[band] = (u8)stream.read_network_order_bits(bits);
        }
    };

    auto copy_long_scalefactors = [&](int granule, int channel, int subband) {
        auto& frame_data = frame.data[granule][channel];
        auto& src_data = frame.data[granule - 1][channel];
        int start_band = subband_range_index[subband][0];
        int end_band = subband_range_index[subband][1];
        for (int band = start_band; band <= end_band; band++)
            frame_data.long_window_sfband[band] = src_data.long_window_sfband[band];
    };

    for (int granule = 0; granule < 2; granule++) {
        for (int channel = 0; channel < channel_count; channel++) {
            auto& frame_data = frame.data[granule][channel];
            auto& side_info = frame_data.side_info;
            // For short blocks, scalefactors are transmitted for each granule.
            if (side_info.block_type == 2 && side_info.window_switching_flag) {
                if (side_info.mixed_block_flag) {
                    // Short Blocks, mixed_block_flag set. `slen1` applies to long window scalefactor bands in the range 0-7 and short
                    // window scalefactor bands in the range 3-5. `slen2` applies to short window scalefactor bands in the range 6-11.
                    for (int band = 0; band <= 7; band++)
                        frame_data.long_window_sfband[band] = (u8)stream.read_network_order_bits(side_info.slen1);
                    for (int band = 3; band <= 11; band++) {
                        u8 bits = band <= 5 ? side_info.slen1 : side_info.slen2;
                        for (int window = 0; window < 3; window++)
                            frame_data.short_window_sfband[band][window] = (u8)stream.read_network_order_bits(bits);
                    }
                } else {
                    // Short block, mixed_block_flag not set. `slen1` applies to long window scalefactor bands in the rang 0-5 and `slen2`
                    // applies to short window scalefactor bands in the range 6-11.
                    for (int band = 0; band <= 11; band++) {
                        u8 bits = band <= 5 ? side_info.slen1 : side_info.slen2;
                        for (int window = 0; window < 3; window++)
                            frame_data.short_window_sfband[band][window] = (u8)stream.read_network_order_bits(bits);
                    }
                }
            } else {
                // Block type 0 (Normal), 1 (Start), or 3 (Stop).
                for (int subband = 0; subband < 4; subband++) {
                    bool scalefactors_shared = frame.scfsi[channel][subband];
                    if (scalefactors_shared)
                        copy_long_scalefactors(granule, channel, subband);
                    else
                        read_long_scalefactors(granule, channel, subband);
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

    auto& frame = m_context.current_frame;
    if (is_corrupt_frame_header()) {
        dbg() << m_stream.offset() << ": corrupt frame header!";
        return false;
    }

    auto slot_size = frame.header.layer == 1 ? 4 : 1;
    frame.length = ((144 * frame.header.bitrate) / frame.header.sample_rate) + (frame.header.has_padding * slot_size);
    debug_fmt("MPEG audio frame length=%dB", frame.length);

    if (!read_side_info()) {
        dbg() << m_stream.offset() << ": corrupt frame!";
        return false;
    }

    m_stream.byte_align_forward();
    return shift_main_data_into_reservoir();
}

bool MpegAudioLoader::decode_frame()
{
    if (!parse_id3(m_stream, m_context.metadata)) {
        dbg() << m_stream.offset() << ": couldn't parse ID3 header!";
        return false;
    }

    do {
        if (!extract_frame())
            return false;
    } while (!m_context.current_frame.is_decodable);

    if (!extract_scalefactors())
        return false;

    return true;
}

}
