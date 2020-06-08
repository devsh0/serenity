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

#pragma once

#include <AK/Vector.h>
#include <LibGfx/Bitmap.h>
#include <LibGfx/ImageDecoder.h>

enum class Marker {
    JPG_INVALID = 0X0000,

    JPG_APPN0 = 0XFFE0,
    JPG_APPN1 = 0XFFE1,
    JPG_APPN2 = 0XFFE2,
    JPG_APPN3 = 0XFFE3,
    JPG_APPN4 = 0XFFE4,
    JPG_APPN5 = 0XFFE5,
    JPG_APPN6 = 0XFFE6,
    JPG_APPN7 = 0XFFE7,
    JPG_APPN8 = 0XFFE8,
    JPG_APPN9 = 0XFFE9,
    JPG_APPNA = 0XFFEA,
    JPG_APPNB = 0XFFEB,
    JPG_APPNC = 0XFFEC,
    JPG_APPND = 0XFFED,
    JPG_APPNE = 0xFFEE,
    JPG_APPNF = 0xFFEF,

    JPG_RESERVED1 = 0xFFF1,
    JPG_RESERVED2 = 0xFFF2,
    JPG_RESERVED3 = 0xFFF3,
    JPG_RESERVED4 = 0xFFF4,
    JPG_RESERVED5 = 0xFFF5,
    JPG_RESERVED6 = 0xFFF6,
    JPG_RESERVED7 = 0xFFF7,
    JPG_RESERVED8 = 0xFFF8,
    JPG_RESERVED9 = 0xFFF9,
    JPG_RESERVEDA = 0xFFFA,
    JPG_RESERVEDB = 0xFFFB,
    JPG_RESERVEDC = 0xFFFC,
    JPG_RESERVEDD = 0xFFFD,

    JPG_RST0 = 0xFFD0,
    JPG_RST1 = 0xFFD1,
    JPG_RST2 = 0xFFD2,
    JPG_RST3 = 0xFFD3,
    JPG_RST4 = 0xFFD4,
    JPG_RST5 = 0xFFD5,
    JPG_RST6 = 0xFFD6,
    JPG_RST7 = 0xFFD7,

    JPG_DHP = 0xFFDE,
    JPG_EXP = 0xFFDF,
    JPG_DHT = 0XFFC4,
    JPG_DQT = 0XFFDB,
    JPG_EOI = 0xFFD9,
    JPG_RST = 0XFFDD,
    JPG_SOF0 = 0XFFC0,
    JPG_SOF2 = 0xFFC2,
    JPG_SOI = 0XFFD8,
    JPG_SOS = 0XFFDA,
    JPG_COM = 0xFFFE
};

namespace Gfx {

    /*
     * MCU means group of data units that are coded together. A data unit is an 8x8
     * block of component data. In interleaved scans, number of non-interleaved data
     * units of a component C is Ch * Cv, where Ch and Cv represent the horizontal &
     * vertical subsampling factors of the component, respectively. A MacroBlock is
     * an 8x8 block of RGB values before encoding, and 8x8 block of YCbCr values when
     * we're done decoding the huffman stream.
     */
    struct Macroblock {
        union {
            i32 y[64] = { 0 };
            i32 r[64];
        };

        union {
            i32 cb[64] = { 0 };
            i32 g[64];
        };

        union {
            i32 cr[64] = { 0 };
            i32 b[64];
        };
    };

    struct MacroblockMeta {
        u32 total;
        u32 padded_total;
        u32 hcount; // Macroblocks in a line.
        u32 vcount; // Lines of macroblocks.
        u32 hpadded_count; // Macroblocks in a line after filling partial MCUs horizontally.
        u32 vpadded_count; // Lines of macroblocks after filling partial MCUs vertically.
    };

    struct ComponentSpec {
        i8 id { -1 };
        u8 horizontal_sampling { 1 };
        u8 vertical_sampling { 1 };
        u8 ac_destination_id;
        u8 dc_destination_id;
        u8 qtable_id; // Quantization table id.
    };

    enum class ScanType {
        DC,
        AC
    };

    struct ScanSpec {
        ScanType type = { ScanType::DC };
        Vector<ComponentSpec*> components;
        u8 component_count;
        u8 spectral_start;
        u8 spectral_end;
        int end_of_band = { 0 };
        bool successive_approx_used { false };
        bool refining { false };
        u8 approx_hi;
        u8 approx_lo;
    };

    struct FrameSpec {
        u8 precision;
        u16 height;
        u16 width;
    };

    struct HuffmanTableSpec {
        u8 type;
        u8 destination_id;
        u8 code_counts[16] = { 0 };
        Vector<u8> symbols;
        Vector<u16> codes;
    };

    struct HuffmanStreamState {
        Vector<u8> stream;
        u8 bit_offset { 0 };
        size_t byte_offset { 0 };
    };

    struct JPGLoadingContext {
        enum State {
            NotDecoded = 0,
            Error,
            FrameDecoded,
            BitmapDecoded
        };

        State state { State::NotDecoded };
        const u8* compressed_data { nullptr };
        size_t compressed_size { 0 };
        u32 luma_table[64];
        u32 chroma_table[64];
        bool is_progressive { false };
        ScanSpec scan_spec;
        MacroblockMeta block_meta; // Macroblock meta.
        FrameSpec frame;
        u8 horizontal_sampling;
        u8 vertical_sampling;
        bool has_zero_based_ids;
        u8 component_count;
        ComponentSpec components[3];
        RefPtr<Gfx::Bitmap> bitmap;
        u16 dc_reset_interval;
        Vector<HuffmanTableSpec> dc_tables;
        Vector<HuffmanTableSpec> ac_tables;
        HuffmanStreamState huffman_stream;
        i32 previous_dc_values[3] = { 0 };
    };

    class JPGImageDecoderPlugin final : public ImageDecoderPlugin {
    public:
        virtual ~JPGImageDecoderPlugin() override;
        JPGImageDecoderPlugin(const u8*, size_t);
        virtual Size size() override;
        virtual RefPtr<Gfx::Bitmap> bitmap() override;
        virtual void set_volatile() override;
        [[nodiscard]] virtual bool set_nonvolatile() override;
        virtual bool sniff() override;
        virtual bool is_animated() override;
        virtual size_t loop_count() override;
        virtual size_t frame_count() override;
        virtual ImageFrameDescriptor frame(size_t i) override;

    private:
        OwnPtr<JPGLoadingContext> m_context;
    };
}
