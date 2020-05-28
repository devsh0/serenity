/*
 * Copyright (c) 2020, The SerenityOS developers.
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

#include <AK/Bitmap.h>
#include <AK/BufferStream.h>
#include <AK/ByteBuffer.h>
#include <AK/LexicalPath.h>
#include <AK/MappedFile.h>
#include <AK/String.h>
#include <AK/Vector.h>
#include <LibGfx/Bitmap.h>
#include <LibGfx/JPGLoader.h>
#include <Libraries/LibM/math.h>

namespace Gfx {

    static bool load_jpg_impl (JPGLoadingContext&);
    RefPtr<Gfx::Bitmap> load_jpg (const StringView& path);
    RefPtr<Gfx::Bitmap> load_jpg_from_memory (const u8*, size_t);

    static const u8 zigzag_map[64]{
        0, 1, 8, 16, 9, 2, 3, 10,
        17, 24, 32, 25, 18, 11, 4, 5,
        12, 19, 26, 33, 40, 48, 41, 34,
        27, 20, 13, 6, 7, 14, 21, 28,
        35, 42, 49, 56, 57, 50, 43, 36,
        29, 22, 15, 23, 30, 37, 44, 51,
        58, 59, 52, 45, 38, 31, 39, 46,
        53, 60, 61, 54, 47, 55, 62, 63
    };

    using Marker = u16;

    void generate_huffman_codes (HuffmanTable& table) {
        unsigned code = 0;
        for (auto number_of_codes : table.code_counts) {
            for (int i = 0; i < number_of_codes; i++)
                table.codes.append(code++);
            code <<= 1;
        }
    }

    Optional<size_t> read_huffman_bits (HuffmanStreamState& hstream, size_t count = 1) {
        if (count > (8 * sizeof(int))) {
            dbg() << String::format("Can't read %i bits at once!", count);
            return {};
        }
        size_t value = 0;
        while (count--) {
            if (hstream.byte_offset >= hstream.stream.size()) {
                dbg() << String::format("Huffman stream exhausted. This could be an error!");
                return {};
            }
            u8 current_byte = hstream.stream[hstream.byte_offset];
            u8 current_bit = 1u & (u32)(current_byte >> (7 - hstream.bit_offset)); // MSB first.
            hstream.bit_offset++;
            value = (value << 1) | (size_t) current_bit;
            if (hstream.bit_offset == 8) {
                hstream.byte_offset++;
                hstream.bit_offset = 0;
            }
        }
        return value;
    }

    Optional<u8> get_next_symbol (HuffmanStreamState& hstream, const HuffmanTable& table) {
        unsigned code = 0;
        size_t code_cursor = 0;
        for (int i = 0; i < 16; i++) { // Codes can't be longer than 16 bits.
            auto result = read_huffman_bits(hstream);
            if (!result.has_value()) return {};
            code = (code << 1) | (i32) result.release_value();
            for (int j = 0; j < table.code_counts[i]; j++) {
                if (code == table.codes[code_cursor])
                    return table.symbols[code_cursor];
                code_cursor++;
            }
        }

        ASSERT_NOT_REACHED();
    }

    Optional<MCU> build_mcu (JPGLoadingContext& context) {
        MCU mcu;
        int component_count = context.component_count;
        for (int component_index = 0; component_index < component_count; component_index++) {
            auto& component = context.components[component_index];
            //    Component(s)        Table(Dest_ID)   Location
            //    Luma (Y)              DC (0)       dc_tables[0]
            //    Chroma (Cb, Cr)       DC (1)       dc_tables[1]
            //    Luma (Y)              AC (0)       ac_tables[0]
            //    Chroma (Cb, Cr)       AC (1)       ac_tables[1]
            auto& dc_table = context.dc_tables[component.dc_destination_id];
            auto& ac_table = context.ac_tables[component.ac_destination_id];

            auto symbol_read_result = get_next_symbol(context.huffman_stream, dc_table);
            if (!symbol_read_result.has_value()) return {};
            auto dc_symbol = symbol_read_result.release_value();
            if (dc_symbol > 11) {
                dbg() << String::format("DC coefficient too long: %i!", dc_symbol);
                return {};
            }

            // Symbol represents the length of this coefficient.
            auto read_result = read_huffman_bits(context.huffman_stream, dc_symbol);
            if (!read_result.has_value()) return {};
            i32 dc_coefficient = read_result.release_value();

            // If the symbol says that a coefficient is n bits long, it is guaranteed to be
            //    n bits long. However, it can be prefixed with zero indicating that it's a
            //    negative value. In that case, we need to calculate what -ve value that is.
            if (dc_symbol != 0 && dc_coefficient < (1 << (dc_symbol - 1)))
                dc_coefficient -= (1 << dc_symbol) - 1;

            i32* select_component = component.id == 1 ? mcu.y : (component.id == 2 ? mcu.cb : mcu.cr);
            auto& previous_dc = context.previous_dc_values[component_index];
            select_component[0] = previous_dc += dc_coefficient;

            // Compute the ac coefficients.
            for (int j = 1; j < 64; j++) {
                symbol_read_result = get_next_symbol(context.huffman_stream, ac_table);
                if (!symbol_read_result.has_value()) return {};
                auto ac_symbol = symbol_read_result.release_value();
                if (ac_symbol == 0) {
                    for (; j < 64; j++)
                        select_component[zigzag_map[j]] = 0;
                    continue;
                }

                u8 run_length = ac_symbol == 0xF0 ? 16 : ac_symbol >> 4;

                if (j + run_length >= 64) {
                    dbg() << String::format("Run-length exceeded boundaries. Cursor: %i, Skipping: %i!", j, run_length);
                    return {};
                }

                u8 coefficient_length = ac_symbol & 0x0F;
                if (coefficient_length > 10) {
                    dbg() << String::format("AC coefficient too long: %i!", coefficient_length);
                    return {};
                }

                for (int k = 0; k < run_length; k++)
                    select_component[zigzag_map[j++]] = 0;

                read_result = read_huffman_bits(context.huffman_stream, coefficient_length);
                if (!read_result.has_value()) return {};

                i32 ac_coefficient = read_result.release_value();
                if (ac_coefficient < (1 << (coefficient_length - 1)))
                    ac_coefficient -= (1 << coefficient_length) - 1;

                select_component[zigzag_map[j]] = ac_coefficient;
            }
        }

        return mcu;
    }

    Optional<Vector<MCU>> decode_huffman_stream (JPGLoadingContext& context) {
        Vector<MCU> mcus;
        mcus.ensure_capacity(context.mcu_meta.count);

        dbg() << "Image Width: " << context.frame.width;
        dbg() << "Image Height: " << context.frame.height;
        dbg() << "MCUs per row: " << context.mcu_meta.count_per_row;
        dbg() << "MCUs per column: " << context.mcu_meta.count_per_column;

        // Compute huffman codes for dc and ac tables.
        for (auto& dc_table : context.dc_tables)
            generate_huffman_codes(dc_table);

        for (auto& ac_table : context.ac_tables)
            generate_huffman_codes(ac_table);

        // Build MCUs.
        for (u32 i = 0; i < context.mcu_meta.count; i++) {
            if (context.dc_reset_interval > 0) {
                if (i % context.dc_reset_interval == 0) {
                    context.previous_dc_values[0] = 0;
                    context.previous_dc_values[1] = 0;
                    context.previous_dc_values[2] = 0;

                    // Advance the huffman stream cursor to the 0th bit of the next byte.
                    if (context.huffman_stream.byte_offset < context.huffman_stream.stream.size()) {
                        if (context.huffman_stream.bit_offset > 0) {
                            context.huffman_stream.bit_offset = 0;
                            context.huffman_stream.byte_offset++;
                        }
                    }
                }
            }

            auto result = build_mcu(context);
            if (!result.has_value()) {
                dbg() << "Failed to build MCU " << i + 1;
                dbg() << "Huffman stream byte offset " << context.huffman_stream.byte_offset;
                dbg() << "Huffman stream bit offset " << context.huffman_stream.bit_offset;
                return {};
            }
            // FIXME: This will happen a lot. Is the copy here going to make things slow?
            mcus.append(result.release_value());
        }

        return mcus;
    }

    static inline bool bounds_okay (const size_t cursor, const size_t delta, const size_t bound) {
        return (delta + cursor) < bound;
    }

    static inline bool is_valid_marker (Marker marker) {
        if (marker >= JPG_APPN0 && marker <= JPG_APPNF)
            return true;
        if (marker >= JPG_RESERVED1 && marker <= JPG_RESERVEDD)
            return true;
        if (marker >= JPG_RST0 && marker <= JPG_RST7)
            return true;
        switch (marker) {
            case JPG_COM:
            case JPG_DHP:
            case JPG_EXP:
            case JPG_DHT:
            case JPG_DQT:
            case JPG_RST:
            case JPG_SOF0:
            case JPG_SOI:
            case JPG_SOS:
                return true;
            default:
                return false;
        }
    }

    static inline u16 read_endian_swapped_word (BufferStream& stream) {
        u16 temp{0};
        stream >> temp;
        return (temp >> 8) | (temp << 8);
    }

    static inline Marker read_marker_at_cursor (BufferStream& stream) {
        u16 marker = read_endian_swapped_word(stream);
        if (stream.handle_read_failure())
            return JPG_INVALID;
        if (is_valid_marker(marker))
            return marker;
        if (marker != 0xFFFF)
            return JPG_INVALID;
        u8 next;
        do {
            stream >> next;
            if (stream.handle_read_failure() || next == 0x00)
                return JPG_INVALID;
        } while (next == 0xFF);
        marker = 0xFF00 | (u16) next;
        return is_valid_marker(marker) ? marker : JPG_INVALID;
    }

    static bool scan_huffman_stream (BufferStream& stream, JPGLoadingContext& context) {
        u8 last_byte;
        u8 current_byte;
        stream >> current_byte;

        for (;;) {
            last_byte = current_byte;
            stream >> current_byte;
            if (stream.handle_read_failure()) {
                dbg() << stream.offset() << ": EOI not found!";
                return false;
            }

            if (last_byte == 0xFF) {
                if (current_byte == 0xFF)
                    continue;
                if (current_byte == 0x00) {
                    stream >> current_byte;
                    context.huffman_stream.stream.append(last_byte);
                    continue;
                }
                Marker marker = 0xFF00 | current_byte;
                if (marker == JPG_EOI)
                    return true;
                if (marker >= JPG_RST0 && marker <= JPG_RST7) {
                    stream >> current_byte;
                    continue;
                }
                dbg() << stream.offset() << String::format(": Invalid marker: %x!", marker);
                return false;
            } else {
                context.huffman_stream.stream.append(last_byte);
            }
        }

        ASSERT_NOT_REACHED();
    }

    static bool read_start_of_scan (BufferStream& stream, JPGLoadingContext& context) {
        if (context.state < JPGLoadingContext::State::FrameDecoded) {
            dbg() << stream.offset() << ": SOS found before reading a SOF!";
            return false;
        }

        u16 bytes_to_read = read_endian_swapped_word(stream);
        if (stream.handle_read_failure())
            return false;
        bytes_to_read -= 2;
        if (!bounds_okay(stream.offset(), bytes_to_read, context.compressed_size))
            return false;
        u8 component_count;
        stream >> component_count;
        if (component_count != context.component_count) {
            dbg() << stream.offset()
                  << String::format(": Unsupported number of components: %i!", component_count);
            return false;
        }

        for (int i = 0; i < component_count; i++) {
            Component* component = nullptr;
            u8 component_id;
            stream >> component_id;
            component_id += context.has_zero_based_ids ? 1 : 0;

            if (component_id == context.components[0].id)
                component = &context.components[0];
            else if (component_id == context.components[1].id)
                component = &context.components[1];
            else if (component_id == context.components[2].id)
                component = &context.components[2];
            else {
                dbg() << stream.offset() << String::format(": Unsupported component id: %i!", component_id);
                return false;
            }

            u8 table_ids;
            stream >> table_ids;
            component->dc_destination_id = table_ids >> 4;
            component->ac_destination_id = table_ids & 0x0F;
        }

        u8 start_of_selection;
        stream >> start_of_selection;
        u8 end_of_selection;
        stream >> end_of_selection;
        u8 successive_approximation;
        stream >> successive_approximation;
        // The three values should be fixed for baseline JPEGs utilizing sequential DCT.
        if (start_of_selection != 0 || end_of_selection != 63 || successive_approximation != 0) {
            dbg() << stream.offset() << ": ERROR! Start of Selection: " << start_of_selection
                  << ", End of Selection: " << end_of_selection
                  << ", Successive Approximation: " << successive_approximation << "!";
            return false;
        }
        return true;
    }

    static bool read_reset_marker (BufferStream& stream, JPGLoadingContext& context) {
        u16 bytes_to_read = read_endian_swapped_word(stream);
        if (stream.handle_read_failure())
            return false;
        bytes_to_read -= 2;
        if (bytes_to_read != 2) {
            dbg() << stream.offset() << ": Malformed reset marker found!";
            return false;
        }
        context.dc_reset_interval = read_endian_swapped_word(stream);
        return true;
    }

    static bool read_huffman_table (BufferStream& stream, JPGLoadingContext& context) {
        i32 bytes_to_read = read_endian_swapped_word(stream);
        if (!bounds_okay(stream.offset(), bytes_to_read, context.compressed_size))
            return false;
        bytes_to_read -= 2;
        while (bytes_to_read > 0) {
            HuffmanTable table;
            u8 table_info;
            stream >> table_info;
            u8 table_type = table_info >> 4;
            u8 table_destination_id = table_info & 0x0F;
            if (table_type > 1) {
                dbg() << stream.offset() << String::format(": Unrecognized huffman table: %i!", table_type);
                return false;
            }
            if (table_destination_id > 3) {
                dbg() << stream.offset()
                      << String::format(": Invalid huffman table destination id: %i!", table_destination_id);
                return false;
            }
            table.type = table_type;
            table.destination_id = table_destination_id;
            u32 total_codes = 0;

            // Read code counts. At each index K, the value represents the number of K+1 bit codes in this header.
            for (int i = 0; i < 16; i++) {
                u8 count;
                stream >> count;
                total_codes += count;
                table.code_counts[i] = count;
            }

            table.codes.ensure_capacity(total_codes);

            // Read symbols. Read X bytes, where X is the sum of the counts of codes read in the previous step.
            for (u32 i = 0; i < total_codes; i++) {
                u8 symbol;
                stream >> symbol;
                table.symbols.append(symbol);
            }

            if (table_type == 0)
                context.dc_tables.append(move(table));
            else
                context.ac_tables.append(move(table));

            bytes_to_read -= 1 + 16 + total_codes;
        }
        if (bytes_to_read != 0) {
            dbg() << stream.offset() << ": Extra bytes detected in huffman header!";
            return false;
        }
        return true;
    }

    static inline void set_mcu_metadata (JPGLoadingContext& context) {
        context.mcu_meta.count_per_row = (context.frame.width + 7) / 8;
        context.mcu_meta.count_per_column = (context.frame.height + 7) / 8;
        context.mcu_meta.count = context.mcu_meta.count_per_row * context.mcu_meta.count_per_column;
    }

    static bool read_start_of_frame (BufferStream& stream, JPGLoadingContext& context) {
        if (context.state == JPGLoadingContext::FrameDecoded) {
            dbg() << stream.offset() << ": SOF repeated!";
            return false;
        }

        i32 bytes_to_read = read_endian_swapped_word(stream);
        if (stream.handle_read_failure())
            return false;

        bytes_to_read -= 2;
        if (!bounds_okay(stream.offset(), bytes_to_read, context.compressed_size))
            return false;

        stream >> context.frame.precision;
        if (context.frame.precision != 8) {
            dbg() << stream.offset() << ": SOF precision != 8!";
            return false;
        }

        context.frame.height = read_endian_swapped_word(stream);
        context.frame.width = read_endian_swapped_word(stream);
        if (!context.frame.width || !context.frame.height) {
            dbg() << stream.offset() << ": ERROR! Image height: " << context.frame.height << ", Image width: "
                  << context.frame.width << "!";
            return false;
        }
        set_mcu_metadata(context);

        stream >> context.component_count;
        if (context.component_count != 1 && context.component_count != 3) {
            dbg() << stream.offset() << ": Unsupported number of components in SOF: "
                  << context.component_count << "!";
            return false;
        }

        for (int i = 0; i < context.component_count; i++) {
            Component& component = context.components[i];
            stream >> component.id;
            if (i == 0) context.has_zero_based_ids = component.id == 0;
            component.id += context.has_zero_based_ids ? 1 : 0;
            u8 subsample_factors;
            stream >> subsample_factors;
            component.hsample_factor = subsample_factors >> 4;
            component.vsample_factor = subsample_factors & 0x0F;
            stream >> component.qtable_id;
            if (component.qtable_id > 1) {
                dbg() << stream.offset() << ": Unsupported quantization table id: "
                      << component.qtable_id << "!";
                return false;
            }
        }
        return true;
    }

    static bool read_quantization_table (BufferStream& stream, JPGLoadingContext& context) {
        i32 bytes_to_read = read_endian_swapped_word(stream);
        if (stream.handle_read_failure())
            return false;
        bytes_to_read -= 2;
        if (!bounds_okay(stream.offset(), bytes_to_read, context.compressed_size))
            return false;
        while (bytes_to_read > 0) {
            u8 info_byte;
            stream >> info_byte;
            u8 element_unit_hint = info_byte >> 4;
            if (element_unit_hint > 1) {
                dbg() << stream.offset()
                      << String::format(": Unsupported unit hint in quantization table: %i!", element_unit_hint);
                return false;
            }
            u8 table_id = info_byte & 0x0F;
            if (table_id > 1) {
                dbg() << stream.offset() << String::format(": Unsupported quantization table id: %i!", table_id);
                return false;
            }
            u32* table = table_id == 0 ? context.luma_table : context.chroma_table;
            for (int i = 0; i < 64; i++) {
                if (element_unit_hint == 0) {
                    u8 tmp;
                    stream >> tmp;
                    table[i] = tmp;
                } else
                    table[zigzag_map[i]] = read_endian_swapped_word(stream);
            }
            bytes_to_read -= 1 + (element_unit_hint == 0 ? 64 : 128);
        }
        if (bytes_to_read != 0) {
            dbg() << stream.offset() << ": Invalid length for one or more quantization tables!";
            return false;
        }

        return true;
    }

    static bool skip_marker_with_length (BufferStream& stream) {
        u16 bytes_to_skip = read_endian_swapped_word(stream);
        bytes_to_skip -= 2;
        if (stream.handle_read_failure())
            return false;
        stream.advance(bytes_to_skip);
        return !stream.handle_read_failure();
    }

    static bool parse_header (BufferStream& stream, JPGLoadingContext& context) {
        auto marker = read_marker_at_cursor(stream);
        if (stream.handle_read_failure())
            return false;
        if (marker != JPG_SOI) {
            dbg() << stream.offset() << String::format(": SOI not found: %x!", marker);
            return false;
        }
        for (;;) {
            marker = read_marker_at_cursor(stream);
            switch (marker) {
                case JPG_INVALID:
                case JPG_RST0:
                case JPG_RST1:
                case JPG_RST2:
                case JPG_RST3:
                case JPG_RST4:
                case JPG_RST5:
                case JPG_RST6:
                case JPG_RST7:
                case JPG_SOI:
                case JPG_EOI:
                    dbg() << stream.offset() << String::format(": Unexpected marker %x!", marker);
                    return false;
                case JPG_SOF0:
                    if (!read_start_of_frame(stream, context))
                        return false;
                    context.state = JPGLoadingContext::FrameDecoded;
                    break;
                case JPG_DQT:
                    if (!read_quantization_table(stream, context))
                        return false;
                    break;
                case JPG_RST:
                    if (!read_reset_marker(stream, context))
                        return false;
                    break;
                case JPG_DHT:
                    if (!read_huffman_table(stream, context))
                        return false;
                    break;
                case JPG_SOS:
                    return read_start_of_scan(stream, context);
                default:
                    if (!skip_marker_with_length(stream)) {
                        dbg() << stream.offset() << String::format(": Error skipping marker: %x!", marker);
                        return false;
                    }
                    break;
            }
        }

        ASSERT_NOT_REACHED();
    }

    void dequantize(JPGLoadingContext& context, Vector<MCU>& mcus) {
        for (u32 i = 0; i < context.mcu_meta.count; i++) {
            auto& mcu = mcus[i];
            for (u32 j = 0; j < context.component_count; j++) {
                auto& component = context.components[j];
                const u32* table = component.qtable_id == 0 ? context.luma_table : context.chroma_table;
                int* mcu_component = j == 0 ? mcu.y : (j == 1 ? mcu.cb : mcu.cr);
                for (u32 k = 0; k < 64; k++)
                    mcu_component[k] *= table[k];
            }
        }
    }


    void inverse_dct(const JPGLoadingContext& context, Vector<MCU>& mcus) {
        static const float m0 = 2.0 * cos(1.0 / 16.0 * 2.0 * M_PI);
        static const float m1 = 2.0 * cos(2.0 / 16.0 * 2.0 * M_PI);
        static const float m3 = 2.0 * cos(2.0 / 16.0 * 2.0 * M_PI);
        static const float m5 = 2.0 * cos(3.0 / 16.0 * 2.0 * M_PI);
        static const float m2 = m0 - m5;
        static const float m4 = m0 + m5;
        static const float s0 = cos(0.0 / 16.0 * M_PI) / sqrt(8);
        static const float s1 = cos(1.0 / 16.0 * M_PI) / 2.0;
        static const float s2 = cos(2.0 / 16.0 * M_PI) / 2.0;
        static const float s3 = cos(3.0 / 16.0 * M_PI) / 2.0;
        static const float s4 = cos(4.0 / 16.0 * M_PI) / 2.0;
        static const float s5 = cos(5.0 / 16.0 * M_PI) / 2.0;
        static const float s6 = cos(6.0 / 16.0 * M_PI) / 2.0;
        static const float s7 = cos(7.0 / 16.0 * M_PI) / 2.0;

        for (u32 i = 0; i < context.mcu_meta.count; i++) {
            auto& mcu = mcus[i];
            for (int j = 0; j < context.component_count; j++) {
                i32* component = j == 0 ? mcu.y : (j == 1 ? mcu.cb : mcu.cr);
                for (u32 k = 0; k < 8; ++k) {
                    const float g0 = component[0 * 8 + k] * s0;
                    const float g1 = component[4 * 8 + k] * s4;
                    const float g2 = component[2 * 8 + k] * s2;
                    const float g3 = component[6 * 8 + k] * s6;
                    const float g4 = component[5 * 8 + k] * s5;
                    const float g5 = component[1 * 8 + k] * s1;
                    const float g6 = component[7 * 8 + k] * s7;
                    const float g7 = component[3 * 8 + k] * s3;

                    const float f0 = g0;
                    const float f1 = g1;
                    const float f2 = g2;
                    const float f3 = g3;
                    const float f4 = g4 - g7;
                    const float f5 = g5 + g6;
                    const float f6 = g5 - g6;
                    const float f7 = g4 + g7;

                    const float e0 = f0;
                    const float e1 = f1;
                    const float e2 = f2 - f3;
                    const float e3 = f2 + f3;
                    const float e4 = f4;
                    const float e5 = f5 - f7;
                    const float e6 = f6;
                    const float e7 = f5 + f7;
                    const float e8 = f4 + f6;

                    const float d0 = e0;
                    const float d1 = e1;
                    const float d2 = e2 * m1;
                    const float d3 = e3;
                    const float d4 = e4 * m2;
                    const float d5 = e5 * m3;
                    const float d6 = e6 * m4;
                    const float d7 = e7;
                    const float d8 = e8 * m5;

                    const float c0 = d0 + d1;
                    const float c1 = d0 - d1;
                    const float c2 = d2 - d3;
                    const float c3 = d3;
                    const float c4 = d4 + d8;
                    const float c5 = d5 + d7;
                    const float c6 = d6 - d8;
                    const float c7 = d7;
                    const float c8 = c5 - c6;

                    const float b0 = c0 + c3;
                    const float b1 = c1 + c2;
                    const float b2 = c1 - c2;
                    const float b3 = c0 - c3;
                    const float b4 = c4 - c8;
                    const float b5 = c8;
                    const float b6 = c6 - c7;
                    const float b7 = c7;

                    component[0 * 8 + k] = b0 + b7;
                    component[1 * 8 + k] = b1 + b6;
                    component[2 * 8 + k] = b2 + b5;
                    component[3 * 8 + k] = b3 + b4;
                    component[4 * 8 + k] = b3 - b4;
                    component[5 * 8 + k] = b2 - b5;
                    component[6 * 8 + k] = b1 - b6;
                    component[7 * 8 + k] = b0 - b7;
                }
                for (u32 l = 0; l < 8; ++l) {
                    const float g0 = component[l * 8 + 0] * s0;
                    const float g1 = component[l * 8 + 4] * s4;
                    const float g2 = component[l * 8 + 2] * s2;
                    const float g3 = component[l * 8 + 6] * s6;
                    const float g4 = component[l * 8 + 5] * s5;
                    const float g5 = component[l * 8 + 1] * s1;
                    const float g6 = component[l * 8 + 7] * s7;
                    const float g7 = component[l * 8 + 3] * s3;

                    const float f0 = g0;
                    const float f1 = g1;
                    const float f2 = g2;
                    const float f3 = g3;
                    const float f4 = g4 - g7;
                    const float f5 = g5 + g6;
                    const float f6 = g5 - g6;
                    const float f7 = g4 + g7;

                    const float e0 = f0;
                    const float e1 = f1;
                    const float e2 = f2 - f3;
                    const float e3 = f2 + f3;
                    const float e4 = f4;
                    const float e5 = f5 - f7;
                    const float e6 = f6;
                    const float e7 = f5 + f7;
                    const float e8 = f4 + f6;

                    const float d0 = e0;
                    const float d1 = e1;
                    const float d2 = e2 * m1;
                    const float d3 = e3;
                    const float d4 = e4 * m2;
                    const float d5 = e5 * m3;
                    const float d6 = e6 * m4;
                    const float d7 = e7;
                    const float d8 = e8 * m5;

                    const float c0 = d0 + d1;
                    const float c1 = d0 - d1;
                    const float c2 = d2 - d3;
                    const float c3 = d3;
                    const float c4 = d4 + d8;
                    const float c5 = d5 + d7;
                    const float c6 = d6 - d8;
                    const float c7 = d7;
                    const float c8 = c5 - c6;

                    const float b0 = c0 + c3;
                    const float b1 = c1 + c2;
                    const float b2 = c1 - c2;
                    const float b3 = c0 - c3;
                    const float b4 = c4 - c8;
                    const float b5 = c8;
                    const float b6 = c6 - c7;
                    const float b7 = c7;

                    component[l * 8 + 0] = b0 + b7;
                    component[l * 8 + 1] = b1 + b6;
                    component[l * 8 + 2] = b2 + b5;
                    component[l * 8 + 3] = b3 + b4;
                    component[l * 8 + 4] = b3 - b4;
                    component[l * 8 + 5] = b2 - b5;
                    component[l * 8 + 6] = b1 - b6;
                    component[l * 8 + 7] = b0 - b7;
                }
            }
        }
    }

    void ycbcr_to_rgb(const JPGLoadingContext& context, Vector<MCU>& mcus) {
        for (u32 i = 0; i < context.mcu_meta.count; i++) {
            i32* y = mcus[i].y;
            i32* cb = mcus[i].cb;
            i32* cr = mcus[i].cr;
            for (u32 j = 0; j < 64; j++) {
                int r = y[j] + 1.402 * cr[j] + 128;
                int g = y[j] - 0.344f * cb[j] - 0.714f * cr[j] + 128;
                int b = y[j] + 1.772 * cb[j] + 128;
                y[j]  = r < 0 ? 0 : (r > 255 ? 255 : r);
                cb[j] = g < 0 ? 0 : (g > 255 ? 255 : g);
                cr[j] = b < 0 ? 0 : (b > 255 ? 255 : b);
            }
        }
    }

    static void compose_bitmap (JPGLoadingContext& context, const Vector<MCU>& mcus) {
        context.bitmap = Bitmap::create_purgeable(BitmapFormat::RGB32, {context.frame.width, context.frame.height});

        for (u32 y = context.frame.height - 1; y < context.frame.height; y--) {
            const u32 mcu_row = y / 8;
            const u32 pixel_row = y % 8;
            for (u32 x = 0; x < context.frame.width; x++) {
                const u32 mcu_column = x / 8;
                auto& mcu = mcus[mcu_row * context.mcu_meta.count_per_row + mcu_column];
                const u32 pixel_column = x % 8;
                const u32 pixel_index = pixel_row * 8 + pixel_column;
                const Color color {(u8)mcu.y[pixel_index], (u8)mcu.cb[pixel_index], (u8)mcu.cr[pixel_index]};
                context.bitmap->set_pixel(x, y, color);
            }
        }
    }

    static bool load_jpg_impl (JPGLoadingContext& context) {
        ByteBuffer buffer = ByteBuffer::wrap(context.compressed_data, context.compressed_size);
        BufferStream stream(buffer);
        if (!parse_header(stream, context))
            return false;
        if (!scan_huffman_stream(stream, context))
            return false;

        auto result = decode_huffman_stream(context);
        if (!result.has_value()) {
            dbg() << stream.offset() << ": Failed to decode MCUs!";
            return false;
        }
        else {
            auto mcus = result.release_value();
            dbg() << String::format("%i MCUs decoded successfully :^)", mcus.size());
            dequantize(context, mcus);
            inverse_dct(context, mcus);
            ycbcr_to_rgb(context, mcus);
            compose_bitmap(context, mcus);
        }
        return true;
    }

    JPGImageDecoderPlugin::JPGImageDecoderPlugin (const u8* data, size_t size) {
        m_context = make<JPGLoadingContext>();
        m_context->compressed_data = data;
        m_context->compressed_size = size;
        m_context->huffman_stream.stream.ensure_capacity(500 * KB);
    }

    JPGImageDecoderPlugin::~JPGImageDecoderPlugin () {
    }

    Size JPGImageDecoderPlugin::size () {
        if (m_context->state == JPGLoadingContext::State::Error)
            return {};
        if (m_context->state >= JPGLoadingContext::State::FrameDecoded)
            return {m_context->frame.width, m_context->frame.height};

        return {};
    }

    RefPtr<Gfx::Bitmap> JPGImageDecoderPlugin::bitmap () {
        if (m_context->state == JPGLoadingContext::State::Error)
            return nullptr;
        if (m_context->state < JPGLoadingContext::State::BitmapDecoded) {
            if (!load_jpg_impl(*m_context)) {
                m_context->state = JPGLoadingContext::State::Error;
                return nullptr;
            }
            m_context->state = JPGLoadingContext::State::BitmapDecoded;
        }

        return m_context->bitmap;
    }

    void JPGImageDecoderPlugin::set_volatile () {
        if (m_context->bitmap)
            m_context->bitmap->set_volatile();
    }

    bool JPGImageDecoderPlugin::set_nonvolatile () {
        if (!m_context->bitmap)
            return false;
        return m_context->bitmap->set_nonvolatile();
    }

    bool JPGImageDecoderPlugin::sniff () {
        return false;
    }

    bool JPGImageDecoderPlugin::is_animated () {
        return false;
    }

    size_t JPGImageDecoderPlugin::loop_count () {
        return 0;
    }

    size_t JPGImageDecoderPlugin::frame_count () {
        return 1;
    }

    ImageFrameDescriptor JPGImageDecoderPlugin::frame (size_t i) {
        if (i > 0) {
            return {bitmap(), 0};
        }
        return {};
    }

    RefPtr<Gfx::Bitmap> load_jpg (const StringView& path) {
        MappedFile mapped_file(path);
        if (!mapped_file.is_valid())
            return nullptr;
        JPGImageDecoderPlugin jpg_decoder((const u8*) mapped_file.data(), mapped_file.size());
        auto bitmap = jpg_decoder.bitmap();
        if (bitmap)
            bitmap->set_mmap_name( String::format("Gfx::Bitmap [%dx%d] - Decoded JPG: %s",
                bitmap->width(), bitmap->height(), LexicalPath::canonicalized_path(path).characters()));
        return bitmap;
    }
}
