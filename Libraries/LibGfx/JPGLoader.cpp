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
#include <AK/HashMap.h>

namespace Gfx {

    static bool load_jpg_impl (JPGLoadingContext&);
    RefPtr<Gfx::Bitmap> load_jpg (const StringView& path);
    RefPtr<Gfx::Bitmap> load_jpg_from_memory (const u8*, size_t);

    static const u8 zigzag_map[64] {
        0, 1, 8, 16, 9, 2, 3, 10,
        17, 24, 32, 25, 18, 11, 4, 5,
        12, 19, 26, 33, 40, 48, 41, 34,
        27, 20, 13, 6, 7, 14, 21, 28,
        35, 42, 49, 56, 57, 50, 43, 36,
        29, 22, 15, 23, 30, 37, 44, 51,
        58, 59, 52, 45, 38, 31, 39, 46,
        53, 60, 61, 54, 47, 55, 62, 63
    };

    void generate_huffman_codes (HuffmanTableSpec& table) {
        unsigned code = 0;
        for (auto number_of_codes : table.code_counts) {
            for (int i = 0; i < number_of_codes; i++)
                table.codes.append(code++);
            code <<= 1;
        }
    }

    Optional<size_t> read_huffman_bits (HuffmanStreamState& hstream, size_t count = 1) {
        if (count > (8 * sizeof(size_t))) {
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
            u8 current_bit = 1u & (u32) (current_byte >> (7 - hstream.bit_offset)); // MSB first.
            hstream.bit_offset++;
            value = (value << 1) | (size_t) current_bit;
            if (hstream.bit_offset == 8) {
                hstream.byte_offset++;
                hstream.bit_offset = 0;
            }
        }
        return value;
    }

    Optional<u8> get_next_symbol (HuffmanStreamState& hstream, const HuffmanTableSpec& table) {
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

        // Let's not crash the browser now...rather just close all operations gracefully.
        // ASSERT_NOT_REACHED();
        dbg() << "If you're seeing this...the jpeg decoder needs to support more kinds of JPEGs!";
        return {};
    }

    Optional<i32> decode_dc (const HuffmanTableSpec& table_spec, JPGLoadingContext& context) {
        auto symbol_or_error = get_next_symbol(context.huffman_stream, table_spec);
        if (!symbol_or_error.has_value()) return {};

        // For DC coefficients, symbol encodes the length of the coefficient.
        auto dc_length = symbol_or_error.release_value();
        if (dc_length > 11) {
            dbg() << String::format("DC coefficient too long: %i!", dc_length);
            return {};
        }

        auto coeff_or_error = read_huffman_bits(context.huffman_stream, dc_length);
        if (!coeff_or_error.has_value()) return {};

        // DC coefficients are encoded as the difference between previous and current DC values.
        i32 dc_diff = coeff_or_error.release_value();

        // If MSB in diff is 0, the difference is -ve. Otherwise +ve.
        if (dc_length != 0 && dc_diff < (1 << (dc_length - 1)))
            dc_diff -= (1 << dc_length) - 1;

        return dc_diff;
    }

    // output["symbol"] = Symbol, output["run_length"] = Run Length,
    // output["coeff_length"] = Coefficient Length, output["coeff"] = Coefficient.
    Optional<HashMap<String, i32>> decode_ac (const HuffmanTableSpec& table_spec, JPGLoadingContext& context) {
        HashMap<String, i32> output;
        auto symbol_or_error = get_next_symbol(context.huffman_stream, table_spec);
        if (!symbol_or_error.has_value()) return {};

        // AC symbols encode 2 pieces of information, the high 4 bits represent
        // number of zeroes to be stuffed before reading the coefficient. Low 4
        // bits represent the magnitude of the coefficient.
        auto ac_symbol = symbol_or_error.release_value();
        output.set("symbol", ac_symbol);
        if (ac_symbol == 0) return output;

        // ac_symbol = 0xF0 means we need to skip 16 zeroes.
        output.set("run_length", ac_symbol == 0xF0 ? 16 : ac_symbol >> 4);

        u8 coeff_length = ac_symbol & 0x0F;
        if (coeff_length > 10) {
            dbg() << String::format("AC coefficient too long: %i!", coeff_length);
            return {};
        }
        output.set("coeff_length", coeff_length);

        if (coeff_length > 0) {
            auto coeff_or_error = read_huffman_bits(context.huffman_stream, coeff_length);
            if (!coeff_or_error.has_value()) return {};
            i32 ac_coefficient = coeff_or_error.release_value();
            if (ac_coefficient < (1 << (coeff_length - 1)))
                ac_coefficient -= (1 << coeff_length) - 1;
            output.set("coeff", ac_coefficient);
        }

        return output;
    }

    /*
     * Build the macroblocks possible by reading single (MCU) subsampled pair of CbCr.
     * Depending on the sampling factors, we may not see triples of Y, Cb, Cr in that
     * order. If sample factors differ from one, we'll read more than one block of y-
     * coefficients before we get to read a cb-cr block.
     *
     * In the function below, `hcursor` and `vcursor` denote the location of the block
     * we're building in the macroblock matrix. `vfactor_i` and `hfactor_i` are cursors
     * that iterate over the vertical and horizontal subsampling factors, respectively.
     * When we finish one iteration of the innermost loop, we'll have the coefficients
     * of one of the components of block at position `mb_index`. When the outermost loop
     * finishes first iteration, we'll have all the luminance coefficients for all the
     * macroblocks that share the chrominance data. Next two iterations (assuming that
     * we are dealing with three components) will fill up the blocks with chroma data.
     */
    bool build_macroblocks (JPGLoadingContext& context, Vector<Macroblock>& macroblocks, u8 hcursor, u8 vcursor) {
        for (u32 cindex = 0; cindex < context.component_count; cindex++) {
            auto& component = context.components[cindex];
            for (u8 vfactor_i = 0; vfactor_i < component.vsample_factor; vfactor_i++) {
                for (u8 hfactor_i = 0; hfactor_i < component.hsample_factor; hfactor_i++) {
                    u32 mb_index = (vcursor + vfactor_i) * context.mblock_meta.hpadded_count + (hfactor_i + hcursor);
                    Macroblock& block = macroblocks[mb_index];
                    i32* select_component = component.id == 1 ? block.y : (component.id == 2 ? block.cb : block.cr);

                    auto dc_diff_or_error = decode_dc(context.dc_tables[component.dc_destination_id], context);
                    if (!dc_diff_or_error.has_value()) return false;
                    auto& previous_dc = context.previous_dc_values[cindex];
                    select_component[0] = previous_dc += dc_diff_or_error.value();

                    // Compute the AC coefficients.
                    for (int j = 1; j < 64;) {
                        auto ac_or_error = decode_ac(context.ac_tables[component.ac_destination_id], context);
                        if (!ac_or_error.has_value()) return false;
                        auto output = ac_or_error.release_value();

                        if (output.get("symbol").value() == 0) {
                            for (; j < 64; j++)
                                select_component[zigzag_map[j]] = 0;
                            continue;
                        }

                        auto run_length = output.get("run_length").release_value();
                        if (j + run_length >= 64) {
                            dbg() << String::format("Run-length exceeded boundaries. Cursor: %i, Skipping: %i!", j, run_length);
                            return false;
                        }

                        for (int k = 0; k < run_length; k++)
                            select_component[zigzag_map[j++]] = 0;

                        // If coeff_length == 0, we need to read another symbol to decode the coefficient.
                        if (output.get("coeff_length").value() != 0)
                            select_component[zigzag_map[j++]] = output.get("coeff").release_value();
                    }
                }
            }
        }

        return true;
    }

    Optional<Vector<Macroblock>> decode_huffman_stream (JPGLoadingContext& context) {
        Vector<Macroblock> macroblocks;
        macroblocks.resize(context.mblock_meta.padded_total);

        dbg() << "Image width: " << context.frame.width;
        dbg() << "Image height: " << context.frame.height;
        dbg() << "Macroblocks in a row: " << context.mblock_meta.hpadded_count;
        dbg() << "Macroblocks in a column: " << context.mblock_meta.vpadded_count;


        // Compute huffman codes for DC and AC tables.
        for (auto& dc_table : context.dc_tables)
            generate_huffman_codes(dc_table);

        for (auto& ac_table : context.ac_tables)
            generate_huffman_codes(ac_table);

        for (u32 vcursor = 0; vcursor < context.mblock_meta.vcount; vcursor += context.vsample_factor) {
            for (u32 hcursor = 0; hcursor < context.mblock_meta.hcount; hcursor += context.hsample_factor) {
                u32 i = vcursor * context.mblock_meta.hpadded_count + hcursor;
                if (context.dc_reset_interval > 0) {
                    if (i % context.dc_reset_interval == 0) {
                        context.previous_dc_values[0] = 0;
                        context.previous_dc_values[1] = 0;
                        context.previous_dc_values[2] = 0;

                        // Restart markers are stored in byte boundaries. Advance the huffman stream cursor to
                        //  the 0th bit of the next byte.
                        if (context.huffman_stream.byte_offset < context.huffman_stream.stream.size()) {
                            if (context.huffman_stream.bit_offset > 0) {
                                context.huffman_stream.bit_offset = 0;
                                context.huffman_stream.byte_offset++;
                            }

                            // Skip the restart marker (RSTn).
                            context.huffman_stream.byte_offset++;
                        }
                    }
                }

                if (!build_macroblocks(context, macroblocks, hcursor, vcursor)) {
                    dbg() << "Failed to build Macroblock " << i + 1;
                    dbg() << "Huffman stream byte offset " << context.huffman_stream.byte_offset;
                    dbg() << "Huffman stream bit offset " << context.huffman_stream.bit_offset;
                    return {};
                }
            }
        }

        return macroblocks;
    }

    static inline bool bounds_okay (const size_t cursor, const size_t delta, const size_t bound) {
        return (delta + cursor) < bound;
    }

    static inline bool is_valid_marker (const Marker marker) {
        if (marker >= Marker::JPG_APPN0 && marker <= Marker::JPG_APPNF) {
            if (marker != Marker::JPG_APPN0)
                dbg() << String::format("%04x not supported yet. The decoder may fail!", marker);
            return true;
        }
        if (marker >= Marker::JPG_RESERVED1 && marker <= Marker::JPG_RESERVEDD)
            return true;
        if (marker >= Marker::JPG_RST0 && marker <= Marker::JPG_RST7)
            return true;
        switch (marker) {
            case Marker::JPG_COM:
            case Marker::JPG_DHP:
            case Marker::JPG_EXP:
            case Marker::JPG_DHT:
            case Marker::JPG_DQT:
            case Marker::JPG_RST:
            case Marker::JPG_SOF0:
            case Marker::JPG_SOF2:
            case Marker::JPG_SOI:
            case Marker::JPG_SOS:
                return true;
            default:
                return false;
        }
    }

    static inline u16 read_endian_swapped_word (BufferStream& stream) {
        u16 temp = 0;
        stream >> temp;
        return (temp >> 8) | (temp << 8);
    }

    static inline Marker read_marker_at_cursor (BufferStream& stream) {
        Marker marker = (Marker)read_endian_swapped_word(stream);
        if (stream.handle_read_failure())
            return Marker::JPG_INVALID;
        if (is_valid_marker(marker))
            return marker;
        if ((u16)marker != 0xFFFF)
            return Marker::JPG_INVALID;
        u8 next;
        do {
            stream >> next;
            if (stream.handle_read_failure() || next == 0x00)
                return Marker::JPG_INVALID;
        } while (next == 0xFF);
        marker = (Marker)(0xFF00 | (u16) next);
        return is_valid_marker(marker) ? marker : Marker::JPG_INVALID;
    }

    void inline print_scan_spec (const JPGLoadingContext& context) {
        dbg() << "Start of Selection: " << context.scan_spec.spectral_start << ", " <<
              "End of Selection: " << context.scan_spec.spectral_end << ", " <<
              "Successive Hi: " << context.scan_spec.approx_hi << ", " <<
              "Successive Lo: " << context.scan_spec.approx_lo;
    }

    static bool read_and_validate_scan_spec (BufferStream& stream, JPGLoadingContext& context) {
        stream >> context.scan_spec.spectral_start;
        stream >> context.scan_spec.spectral_end;
        u8 successive_approx;
        stream >> successive_approx;
        context.scan_spec.approx_hi = successive_approx >> 4;
        context.scan_spec.approx_lo = successive_approx & 0x0F;
        context.scan_spec.successive_approx_used = context.scan_spec.approx_hi != 0 || context.scan_spec.approx_lo != 0;

        print_scan_spec(context);

        if (context.is_progressive) {
            if ((context.scan_spec.spectral_start == 0 || context.scan_spec.spectral_end == 0) &&
                (context.scan_spec.spectral_start != context.scan_spec.spectral_end))
                return false;

            if (context.scan_spec.spectral_start > context.scan_spec.spectral_end)
                return false;

            // AC scans should always be non-interleaved.
            if (context.scan_spec.spectral_start != 0 && context.scan_spec.component_count != 1)
                return false;

            if (context.scan_spec.spectral_end > 63)
                return false;

            if (context.scan_spec.approx_lo > 13)
                return false;

            if (context.scan_spec.approx_hi != 0 && context.scan_spec.approx_hi != context.scan_spec.approx_lo + 1)
                return false;

            context.scan_spec.type = context.scan_spec.spectral_start == 0 ? ScanType::DC : ScanType::AC;
            context.scan_spec.refining = context.scan_spec.successive_approx_used && context.scan_spec.approx_hi != 0;
            return true;
        }

        // For baseline sequential scans, these values must be fixed.
        return context.scan_spec.spectral_start == 0 && context.scan_spec.spectral_end == 63 && successive_approx == 0;
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
        if (component_count > context.component_count) {
            dbg() << stream.offset() << String::format(": Unsupported number of components: %i!", component_count);
            return false;
        }
        context.scan_spec.component_count = component_count;

        dbg() << "Component in this scan: " << component_count;

        context.scan_spec.components.clear();

        for (int i = 0; i < component_count; i++) {
            ComponentSpec* component = nullptr;
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
            context.scan_spec.components.append(component);
        }

        return read_and_validate_scan_spec(stream, context);
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
            HuffmanTableSpec table;
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

            auto& table_vec = table_type == 0 ? context.dc_tables : context.ac_tables;
            // If table exists at that slot, replace it with the new one.
            if (table_vec.size() > table.destination_id)
                table_vec[table.destination_id] = move(table);
            else table_vec.append(move(table));

            bytes_to_read -= 1 + 16 + total_codes;
        }
        if (bytes_to_read != 0) {
            dbg() << stream.offset() << ": Extra bytes detected in huffman header!";
            return false;
        }
        return true;
    }

    static inline bool validate_luma_and_modify_context (const ComponentSpec& luma, JPGLoadingContext& context) {
        if ((luma.hsample_factor == 1 || luma.hsample_factor == 2) &&
            (luma.vsample_factor == 1 || luma.vsample_factor == 2)) {
            context.mblock_meta.hpadded_count += luma.hsample_factor == 1 ? 0 : context.mblock_meta.hcount % 2;
            context.mblock_meta.vpadded_count += luma.vsample_factor == 1 ? 0 : context.mblock_meta.vcount % 2;
            context.mblock_meta.padded_total = context.mblock_meta.hpadded_count * context.mblock_meta.vpadded_count;
            // For easy reference to relevant sample factors.
            context.hsample_factor = luma.hsample_factor;
            context.vsample_factor = luma.vsample_factor;
            dbg() << String::format("Horizontal Subsampling Factor: %i", luma.hsample_factor);
            dbg() << String::format("Vertical Subsampling Factor: %i", luma.vsample_factor);
            return true;
        }
        return false;
    }

    static inline void set_macroblock_metadata (JPGLoadingContext& context) {
        context.mblock_meta.hcount = (context.frame.width + 7) / 8;
        context.mblock_meta.vcount = (context.frame.height + 7) / 8;
        context.mblock_meta.hpadded_count = context.mblock_meta.hcount;
        context.mblock_meta.vpadded_count = context.mblock_meta.vcount;
        context.mblock_meta.total = context.mblock_meta.hcount * context.mblock_meta.vcount;
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
        set_macroblock_metadata(context);

        stream >> context.component_count;
        if (context.component_count != 1 && context.component_count != 3) {
            dbg() << stream.offset() << ": Unsupported number of components in SOF: "
                  << context.component_count << "!";
            return false;
        }

        for (int i = 0; i < context.component_count; i++) {
            ComponentSpec& component = context.components[i];

            stream >> component.id;
            if (i == 0) context.has_zero_based_ids = component.id == 0;
            component.id += context.has_zero_based_ids ? 1 : 0;

            u8 subsample_factors;
            stream >> subsample_factors;
            component.hsample_factor = subsample_factors >> 4;
            component.vsample_factor = subsample_factors & 0x0F;

            if (component.id == 1) {
                // By convention, downsampling is applied only on chroma components. So we should
                //  hope to see the maximum sampling factor in the luma component.
                if (!validate_luma_and_modify_context(component, context)) {
                    dbg() << stream.offset() << ": Unsupported luma subsampling factors: "
                          << "horizontal: " << component.hsample_factor << ", vertical: " << component.vsample_factor;
                    return false;
                }
            } else {
                if (component.hsample_factor != 1 || component.vsample_factor != 1) {
                    dbg() << stream.offset() << ": Unsupported chroma subsampling factors: "
                          << "horizontal: " << component.hsample_factor << ", vertical: " << component.vsample_factor;
                    return false;
                }
            }

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
                    table[zigzag_map[i]] = tmp;
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

    void dequantize (JPGLoadingContext& context, Vector<Macroblock>& macroblocks) {
        for (u32 vcursor = 0; vcursor < context.mblock_meta.vcount; vcursor += context.vsample_factor) {
            for (u32 hcursor = 0; hcursor < context.mblock_meta.hcount; hcursor += context.hsample_factor) {
                for (u8 cindex = 0; cindex < context.component_count; cindex++) {
                    auto& component = context.components[cindex];
                    const u32* table = component.qtable_id == 0 ? context.luma_table : context.chroma_table;
                    for (u32 vfactor_i = 0; vfactor_i < component.vsample_factor; vfactor_i++) {
                        for (u32 hfactor_i = 0; hfactor_i < component.hsample_factor; hfactor_i++) {
                            u32 mb_index =
                                (vcursor + vfactor_i) * context.mblock_meta.hpadded_count + (hfactor_i + hcursor);
                            Macroblock& block = macroblocks[mb_index];
                            int* block_component = cindex == 0 ? block.y : (cindex == 1 ? block.cb : block.cr);
                            for (u32 k = 0; k < 64; k++)
                                block_component[k] *= table[k];
                        }
                    }
                }
            }
        }
    }


    void inverse_dct (const JPGLoadingContext& context, Vector<Macroblock>& macroblocks) {
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

        for (u32 vcursor = 0; vcursor < context.mblock_meta.vcount; vcursor += context.vsample_factor) {
            for (u32 hcursor = 0; hcursor < context.mblock_meta.hcount; hcursor += context.hsample_factor) {
                for (u8 cindex = 0; cindex < context.component_count; cindex++) {
                    auto& component = context.components[cindex];
                    for (u8 vfactor_i = 0; vfactor_i < component.vsample_factor; vfactor_i++) {
                        for (u8 hfactor_i = 0; hfactor_i < component.hsample_factor; hfactor_i++) {
                            u32 mb_index = (vcursor + vfactor_i) * context.mblock_meta.hpadded_count + (hfactor_i + hcursor);
                            Macroblock& block = macroblocks[mb_index];
                            i32* block_component = cindex == 0 ? block.y : (cindex == 1 ? block.cb : block.cr);
                            for (u32 k = 0; k < 8; ++k) {
                                const float g0 = block_component[0 * 8 + k] * s0;
                                const float g1 = block_component[4 * 8 + k] * s4;
                                const float g2 = block_component[2 * 8 + k] * s2;
                                const float g3 = block_component[6 * 8 + k] * s6;
                                const float g4 = block_component[5 * 8 + k] * s5;
                                const float g5 = block_component[1 * 8 + k] * s1;
                                const float g6 = block_component[7 * 8 + k] * s7;
                                const float g7 = block_component[3 * 8 + k] * s3;

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

                                block_component[0 * 8 + k] = b0 + b7;
                                block_component[1 * 8 + k] = b1 + b6;
                                block_component[2 * 8 + k] = b2 + b5;
                                block_component[3 * 8 + k] = b3 + b4;
                                block_component[4 * 8 + k] = b3 - b4;
                                block_component[5 * 8 + k] = b2 - b5;
                                block_component[6 * 8 + k] = b1 - b6;
                                block_component[7 * 8 + k] = b0 - b7;
                            }
                            for (u32 l = 0; l < 8; ++l) {
                                const float g0 = block_component[l * 8 + 0] * s0;
                                const float g1 = block_component[l * 8 + 4] * s4;
                                const float g2 = block_component[l * 8 + 2] * s2;
                                const float g3 = block_component[l * 8 + 6] * s6;
                                const float g4 = block_component[l * 8 + 5] * s5;
                                const float g5 = block_component[l * 8 + 1] * s1;
                                const float g6 = block_component[l * 8 + 7] * s7;
                                const float g7 = block_component[l * 8 + 3] * s3;

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

                                block_component[l * 8 + 0] = b0 + b7;
                                block_component[l * 8 + 1] = b1 + b6;
                                block_component[l * 8 + 2] = b2 + b5;
                                block_component[l * 8 + 3] = b3 + b4;
                                block_component[l * 8 + 4] = b3 - b4;
                                block_component[l * 8 + 5] = b2 - b5;
                                block_component[l * 8 + 6] = b1 - b6;
                                block_component[l * 8 + 7] = b0 - b7;
                            }
                        }
                    }
                }
            }
        }
    }

    void ycbcr_to_rgb (const JPGLoadingContext& context, Vector<Macroblock>& macroblocks) {
        for (u32 vcursor = 0; vcursor < context.mblock_meta.vcount; vcursor += context.vsample_factor) {
            for (u32 hcursor = 0; hcursor < context.mblock_meta.hcount; hcursor += context.hsample_factor) {
                const u32 chroma_block_index = vcursor * context.mblock_meta.hpadded_count + hcursor;
                const Macroblock& chroma = macroblocks[chroma_block_index];
                // Overflows are intentional.
                for (u8 vfactor_i = context.vsample_factor - 1; vfactor_i < context.vsample_factor; --vfactor_i) {
                    for (u8 hfactor_i = context.hsample_factor - 1; hfactor_i < context.hsample_factor; --hfactor_i) {
                        u32 mb_index =
                            (vcursor + vfactor_i) * context.mblock_meta.hpadded_count + (hcursor + hfactor_i);
                        i32* y = macroblocks[mb_index].y;
                        i32* cb = macroblocks[mb_index].cb;
                        i32* cr = macroblocks[mb_index].cr;
                        for (u8 i = 7; i < 8; --i) {
                            for (u8 j = 7; j < 8; --j) {
                                const u8 pixel = i * 8 + j;
                                const u32 chroma_pxrow = (i / context.vsample_factor) + 4 * vfactor_i;
                                const u32 chroma_pxcol = (j / context.hsample_factor) + 4 * hfactor_i;
                                const u32 chroma_pixel = chroma_pxrow * 8 + chroma_pxcol;
                                int r = y[pixel] + 1.402f * chroma.cr[chroma_pixel] + 128;
                                int g = y[pixel] - 0.344f * chroma.cb[chroma_pixel] - 0.714f * chroma.cr[chroma_pixel] + 128;
                                int b = y[pixel] + 1.772f * chroma.cb[chroma_pixel] + 128;
                                y[pixel] = r < 0 ? 0 : (r > 255 ? 255 : r);
                                cb[pixel] = g < 0 ? 0 : (g > 255 ? 255 : g);
                                cr[pixel] = b < 0 ? 0 : (b > 255 ? 255 : b);
                            }
                        }
                    }
                }
            }
        }
    }

    static void compose_bitmap (JPGLoadingContext& context, const Vector<Macroblock>& macroblocks) {
        context.bitmap = Bitmap::create_purgeable(BitmapFormat::RGB32, { context.frame.width, context.frame.height });

        for (u32 y = context.frame.height - 1; y < context.frame.height; y--) {
            const u32 block_row = y / 8;
            const u32 pixel_row = y % 8;
            for (u32 x = 0; x < context.frame.width; x++) {
                const u32 block_column = x / 8;
                auto& block = macroblocks[block_row * context.mblock_meta.hpadded_count + block_column];
                const u32 pixel_column = x % 8;
                const u32 pixel_index = pixel_row * 8 + pixel_column;
                const Color color { (u8) block.y[pixel_index], (u8) block.cb[pixel_index], (u8) block.cr[pixel_index] };
                context.bitmap->set_pixel(x, y, color);
            }
        }
    }

    static bool parse_header (BufferStream& stream, JPGLoadingContext& context) {
        auto marker = read_marker_at_cursor(stream);
        if (stream.handle_read_failure())
            return false;
        if (marker != Marker::JPG_SOI) {
            dbg() << stream.offset() << String::format(": SOI not found: %x!", marker);
            return false;
        }
        for (;;) {
            marker = read_marker_at_cursor(stream);
            switch (marker) {
                case Marker::JPG_INVALID:
                case Marker::JPG_RST0:
                case Marker::JPG_RST1:
                case Marker::JPG_RST2:
                case Marker::JPG_RST3:
                case Marker::JPG_RST4:
                case Marker::JPG_RST5:
                case Marker::JPG_RST6:
                case Marker::JPG_RST7:
                case Marker::JPG_SOI:
                case Marker::JPG_EOI:
                    dbg() << stream.offset() << String::format(": Unexpected marker %x!", marker);
                    return false;
                case Marker::JPG_SOF2:
                    dbg() << "Proceeding to decode progressive JPEG";
                    context.is_progressive = true;
                    [[fallthrough]];
                case Marker::JPG_SOF0:
                    dbg() << "Proceeding to decode baseline JPEG";
                    if (!read_start_of_frame(stream, context))
                        return false;
                    context.state = JPGLoadingContext::FrameDecoded;
                    break;
                case Marker::JPG_DQT:
                    if (!read_quantization_table(stream, context))
                        return false;
                    break;
                case Marker::JPG_RST:
                    if (!read_reset_marker(stream, context))
                        return false;
                    break;
                case Marker::JPG_DHT:
                    if (!read_huffman_table(stream, context))
                        return false;
                    break;
                case Marker::JPG_SOS:
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

    static Marker scan_huffman_stream (BufferStream& stream, JPGLoadingContext& context) {
        u8 last_byte;
        u8 current_byte;
        stream >> current_byte;

        for (;;) {
            last_byte = current_byte;
            stream >> current_byte;
            if (stream.handle_read_failure()) {
                dbg() << stream.offset() << ": EOI not found!";
                return Marker::JPG_INVALID;
            }

            if (last_byte == 0xFF) {
                if (current_byte == 0xFF)
                    continue;
                if (current_byte == 0x00) {
                    stream >> current_byte;
                    context.huffman_stream.stream.append(last_byte);
                    continue;
                }
                Marker marker = (Marker)(0xFF00 | current_byte);
                if (marker == Marker::JPG_EOI || marker == Marker::JPG_DHT || marker == Marker::JPG_SOS)
                    return marker;
                if (marker >= Marker::JPG_RST0 && marker <= Marker::JPG_RST7) {
                    context.huffman_stream.stream.append((u16)marker);
                    stream >> current_byte;
                    continue;
                }
                dbg() << stream.offset() << String::format(": Invalid marker: %x!", marker);
                return Marker::JPG_INVALID;
            } else {
                context.huffman_stream.stream.append(last_byte);
            }
        }

        ASSERT_NOT_REACHED();
    }

    static bool load_jpg_impl (JPGLoadingContext& context) {
        ByteBuffer buffer = ByteBuffer::wrap(context.compressed_data, context.compressed_size);
        BufferStream stream(buffer);
        if (!parse_header(stream, context))
            return false;

        Marker scan_terminator;
        while ((scan_terminator = scan_huffman_stream(stream, context)) != Marker::JPG_EOI) {
            if (scan_terminator == Marker::JPG_INVALID) {
                dbg() << "Something went wrong while scanning huffman stream!";
                return false;
            }
            if (scan_terminator == Marker::JPG_SOS) {
                dbg() << "Reading new scan segment...";
                read_start_of_scan(stream, context);
            }
            if (scan_terminator == Marker::JPG_DHT) {
                dbg() << "Reading new huffman table specs...";
                read_huffman_table(stream, context);
            }
        }

        auto result = decode_huffman_stream(context);
        if (!result.has_value()) {
            dbg() << stream.offset() << ": Failed to decode Macroblocks!";
            return false;
        } else {
            auto macroblocks = result.release_value();
            dbg() << String::format("%i macroblocks decoded successfully :^)", macroblocks.size());
            dequantize(context, macroblocks);
            inverse_dct(context, macroblocks);
            ycbcr_to_rgb(context, macroblocks);
            compose_bitmap(context, macroblocks);
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
            return { m_context->frame.width, m_context->frame.height };

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
            return { bitmap(), 0 };
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
            bitmap->set_mmap_name(String::format("Gfx::Bitmap [%dx%d] - Decoded JPG: %s",
                                                 bitmap->width(), bitmap->height(),
                                                 LexicalPath::canonicalized_path(path).characters()));
        return bitmap;
    }
}
