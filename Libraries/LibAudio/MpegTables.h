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

#include <AK/Types.h>

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

struct scalefactor_band_index {
    unsigned long_bands[23] = { 0 };
    unsigned short_bands[14] = { 0 };
};

static constexpr scalefactor_band_index band_index[3] = {
    {
        {0,4,8,12,16,20,24,30,36,44,52,62,74,90,110,134,162,196,238,288,342,418,576},
        {0,4,8,12,16,22,30,40,52,66,84,106,136,192}
    },
    {
        {0,4,8,12,16,20,24,30,36,42,50,60,72,88,106,128,156,190,230,276,330,384,576},
        {0,4,8,12,16,22,28,38,50,64,80,100,126,192}
    },
    {
        {0,4,8,12,16,20,24,30,36,44,54,66,82,102,126,156,194,240,296,364,448,550,576},
        {0,4,8,12,16,22,30,42,58,78,104,138,180,192}
    }
};

static constexpr u16 huffman_table[] = {
    // huffman_table_1[7]
    0x0201,  0x0000,  0x0201,  0x0010,  0x0201,  0x0001,  0x0011,

    // huffman_table_2[17]
    0x0201,  0x0000,  0x0401,  0x0201, 0x0010, 0x0001, 0x0201, 0x0011, 0x0401, 0x0201, 0x0020,
    0x0021, 0x0201, 0x0012, 0x0201, 0x0002, 0x0022,

    // huffman_table_3[17]
    0x0401, 0x0201, 0x0000, 0x0001, 0x0201, 0x0011, 0x0201, 0x0010, 0x0401, 0x0201, 0x0020,
    0x0021, 0x0201, 0x0012, 0x0201, 0x0002, 0x0022,

    // huffman_table_5[31]
    0x0201, 0x0000, 0x0401, 0x0201, 0x0010, 0x0001, 0x0201, 0x0011, 0x0801, 0x0401, 0x0201,
    0x0020, 0x0002, 0x0201, 0x0021, 0x0012, 0x0801, 0x0401, 0x0201, 0x0022, 0x0030, 0x0201,
    0x0003, 0x0013, 0x0201, 0x0031, 0x0201, 0x0032, 0x0201, 0x0023, 0x0033,

    // huffman_table_6[31]
    0x0601, 0x0401, 0x0201, 0x0000, 0x0010, 0x0011, 0x0601, 0x0201, 0x0001, 0x0201, 0x0020,
    0x0021, 0x0601, 0x0201, 0x0012, 0x0201, 0x0002, 0x0022, 0x0401, 0x0201, 0x0031, 0x0013,
    0x0401, 0x0201, 0x0030, 0x0032, 0x0201, 0x0023, 0x0201, 0x0003, 0x0033,

    // huffman_table_7[71]
    0x0201, 0x0000, 0x0401, 0x0201, 0x0010, 0x0001, 0x0801, 0x0201, 0x0011, 0x0401, 0x0201,
    0x0020, 0x0002, 0x0021, 0x1201, 0x0601, 0x0201, 0x0012, 0x0201, 0x0022, 0x0030, 0x0401,
    0x0201, 0x0031, 0x0013, 0x0401, 0x0201, 0x0003, 0x0032, 0x0201, 0x0023, 0x0004, 0x0a01,
    0x0401, 0x0201, 0x0040, 0x0041, 0x0201, 0x0014, 0x0201, 0x0042, 0x0024, 0x0c01, 0x0601,
    0x0401, 0x0201, 0x0033, 0x0043, 0x0050, 0x0401, 0x0201, 0x0034, 0x0005, 0x0051, 0x0601,
    0x0201, 0x0015, 0x0201, 0x0052, 0x0025, 0x0401, 0x0201, 0x0044, 0x0035, 0x0401, 0x0201,
    0x0053, 0x0054, 0x0201, 0x0045, 0x0055,

    // huffman_table_8[71]
    0x0601, 0x0201, 0x0000, 0x0201, 0x0010, 0x0001, 0x0201, 0x0011, 0x0401, 0x0201, 0x0021,
    0x0012, 0x0e01, 0x0401, 0x0201, 0x0020, 0x0002, 0x0201, 0x0022, 0x0401, 0x0201, 0x0030,
    0x0003, 0x0201, 0x0031, 0x0013, 0x0e01, 0x0801, 0x0401, 0x0201, 0x0032, 0x0023, 0x0201,
    0x0040, 0x0004, 0x0201, 0x0041, 0x0201, 0x0014, 0x0042, 0x0c01, 0x0601, 0x0201, 0x0024,
    0x0201, 0x0033, 0x0050, 0x0401, 0x0201, 0x0043, 0x0034, 0x0051, 0x0601, 0x0201, 0x0015,
    0x0201, 0x0005, 0x0052, 0x0601, 0x0201, 0x0025, 0x0201, 0x0044, 0x0035, 0x0201, 0x0053,
    0x0201, 0x0045, 0x0201, 0x0054, 0x0055,

    // huffman_table_9[71]
    0x0801, 0x0401, 0x0201, 0x0000, 0x0010, 0x0201, 0x0001, 0x0011, 0x0a01, 0x0401, 0x0201,
    0x0020, 0x0021, 0x0201, 0x0012, 0x0201, 0x0002, 0x0022, 0x0c01, 0x0601, 0x0401, 0x0201,
    0x0030, 0x0003, 0x0031, 0x0201, 0x0013, 0x0201, 0x0032, 0x0023, 0x0c01, 0x0401, 0x0201,
    0x0041, 0x0014, 0x0401, 0x0201, 0x0040, 0x0033, 0x0201, 0x0042, 0x0024, 0x0a01, 0x0601,
    0x0401, 0x0201, 0x0004, 0x0050, 0x0043, 0x0201, 0x0034, 0x0051, 0x0801, 0x0401, 0x0201,
    0x0015, 0x0052, 0x0201, 0x0025, 0x0044, 0x0601, 0x0401, 0x0201, 0x0005, 0x0054, 0x0053,
    0x0201, 0x0035, 0x0201, 0x0045, 0x0055,

    // huffman_table_10[127]
    0x0201, 0x0000, 0x0401, 0x0201, 0x0010, 0x0001, 0x0a01, 0x0201, 0x0011, 0x0401, 0x0201,
    0x0020, 0x0002, 0x0201, 0x0021, 0x0012, 0x1c01, 0x0801, 0x0401, 0x0201, 0x0022, 0x0030,
    0x0201, 0x0031, 0x0013, 0x0801, 0x0401, 0x0201, 0x0003, 0x0032, 0x0201, 0x0023, 0x0040,
    0x0401, 0x0201, 0x0041, 0x0014, 0x0401, 0x0201, 0x0004, 0x0033, 0x0201, 0x0042, 0x0024,
    0x1c01, 0x0a01, 0x0601, 0x0401, 0x0201, 0x0050, 0x0005, 0x0060, 0x0201, 0x0061, 0x0016,
    0x0c01, 0x0601, 0x0401, 0x0201, 0x0043, 0x0034, 0x0051, 0x0201, 0x0015, 0x0201, 0x0052,
    0x0025, 0x0401, 0x0201, 0x0026, 0x0036, 0x0071, 0x1401, 0x0801, 0x0201, 0x0017, 0x0401,
    0x0201, 0x0044, 0x0053, 0x0006, 0x0601, 0x0401, 0x0201, 0x0035, 0x0045, 0x0062, 0x0201,
    0x0070, 0x0201, 0x0007, 0x0064, 0x0e01, 0x0401, 0x0201, 0x0072, 0x0027, 0x0601, 0x0201,
    0x0063, 0x0201, 0x0054, 0x0055, 0x0201, 0x0046, 0x0073, 0x0801, 0x0401, 0x0201, 0x0037,
    0x0065, 0x0201, 0x0056, 0x0074, 0x0601, 0x0201, 0x0047, 0x0201, 0x0066, 0x0075, 0x0401,
    0x0201, 0x0057, 0x0076, 0x0201, 0x0067, 0x0077,

    // huffman_table_11[127]
    0x0601, 0x0201, 0x0000, 0x0201, 0x0010, 0x0001, 0x0801, 0x0201, 0x0011, 0x0401, 0x0201,
    0x0020, 0x0002, 0x0012, 0x1801, 0x0801, 0x0201, 0x0021, 0x0201, 0x0022, 0x0201, 0x0030,
    0x0003, 0x0401, 0x0201, 0x0031, 0x0013, 0x0401, 0x0201, 0x0032, 0x0023, 0x0401, 0x0201,
    0x0040, 0x0004, 0x0201, 0x0041, 0x0014, 0x1e01, 0x1001, 0x0a01, 0x0401, 0x0201, 0x0042,
    0x0024, 0x0401, 0x0201, 0x0033, 0x0043, 0x0050, 0x0401, 0x0201, 0x0034, 0x0051, 0x0061,
    0x0601, 0x0201, 0x0016, 0x0201, 0x0006, 0x0026, 0x0201, 0x0062, 0x0201, 0x0015, 0x0201,
    0x0005, 0x0052, 0x1001, 0x0a01, 0x0601, 0x0401, 0x0201, 0x0025, 0x0044, 0x0060, 0x0201,
    0x0063, 0x0036, 0x0401, 0x0201, 0x0070, 0x0017, 0x0071, 0x1001, 0x0601, 0x0401, 0x0201,
    0x0007, 0x0064, 0x0072, 0x0201, 0x0027, 0x0401, 0x0201, 0x0053, 0x0035, 0x0201, 0x0054,
    0x0045, 0x0a01, 0x0401, 0x0201, 0x0046, 0x0073, 0x0201, 0x0037, 0x0201, 0x0065, 0x0056,
    0x0a01, 0x0601, 0x0401, 0x0201, 0x0055, 0x0057, 0x0074, 0x0201, 0x0047, 0x0066, 0x0401,
    0x0201, 0x0075, 0x0076, 0x0201, 0x0067, 0x0077,

    // huffman_table_12[127]
    0x0c01, 0x0401, 0x0201, 0x0010, 0x0001, 0x0201, 0x0011, 0x0201, 0x0000, 0x0201, 0x0020,
    0x0002, 0x1001, 0x0401, 0x0201, 0x0021, 0x0012, 0x0401, 0x0201, 0x0022, 0x0031, 0x0201,
    0x0013, 0x0201, 0x0030, 0x0201, 0x0003, 0x0040, 0x1a01, 0x0801, 0x0401, 0x0201, 0x0032,
    0x0023, 0x0201, 0x0041, 0x0033, 0x0a01, 0x0401, 0x0201, 0x0014, 0x0042, 0x0201, 0x0024,
    0x0201, 0x0004, 0x0050, 0x0401, 0x0201, 0x0043, 0x0034, 0x0201, 0x0051, 0x0015, 0x1c01,
    0x0e01, 0x0801, 0x0401, 0x0201, 0x0052, 0x0025, 0x0201, 0x0053, 0x0035, 0x0401, 0x0201,
    0x0060, 0x0016, 0x0061, 0x0401, 0x0201, 0x0062, 0x0026, 0x0601, 0x0401, 0x0201, 0x0005,
    0x0006, 0x0044, 0x0201, 0x0054, 0x0045, 0x1201, 0x0a01, 0x0401, 0x0201, 0x0063, 0x0036,
    0x0401, 0x0201, 0x0070, 0x0007, 0x0071, 0x0401, 0x0201, 0x0017, 0x0064, 0x0201, 0x0046,
    0x0072, 0x0a01, 0x0601, 0x0201, 0x0027, 0x0201, 0x0055, 0x0073, 0x0201, 0x0037, 0x0056,
    0x0801, 0x0401, 0x0201, 0x0065, 0x0074, 0x0201, 0x0047, 0x0066, 0x0401, 0x0201, 0x0075,
    0x0057, 0x0201, 0x0076, 0x0201, 0x0067, 0x0077,

    // huffman_table_13[511]
    0x0201, 0x0000, 0x0601, 0x0201, 0x0010, 0x0201, 0x0001, 0x0011, 0x1c01, 0x0801, 0x0401,
    0x0201, 0x0020, 0x0002, 0x0201, 0x0021, 0x0012, 0x0801, 0x0401, 0x0201, 0x0022, 0x0030,
    0x0201, 0x0003, 0x0031, 0x0601, 0x0201, 0x0013, 0x0201, 0x0032, 0x0023, 0x0401, 0x0201,
    0x0040, 0x0004, 0x0041, 0x4601, 0x1c01, 0x0e01, 0x0601, 0x0201, 0x0014, 0x0201, 0x0033,
    0x0042, 0x0401, 0x0201, 0x0024, 0x0050, 0x0201, 0x0043, 0x0034, 0x0401, 0x0201, 0x0051,
    0x0015, 0x0401, 0x0201, 0x0005, 0x0052, 0x0201, 0x0025, 0x0201, 0x0044, 0x0053, 0x0e01,
    0x0801, 0x0401, 0x0201, 0x0060, 0x0006, 0x0201, 0x0061, 0x0016, 0x0401, 0x0201, 0x0080,
    0x0008, 0x0081, 0x1001, 0x0801, 0x0401, 0x0201, 0x0035, 0x0062, 0x0201, 0x0026, 0x0054,
    0x0401, 0x0201, 0x0045, 0x0063, 0x0201, 0x0036, 0x0070, 0x0601, 0x0401, 0x0201, 0x0007,
    0x0055, 0x0071, 0x0201, 0x0017, 0x0201, 0x0027, 0x0037, 0x4801, 0x1801, 0x0c01, 0x0401,
    0x0201, 0x0018, 0x0082, 0x0201, 0x0028, 0x0401, 0x0201, 0x0064, 0x0046, 0x0072, 0x0801,
    0x0401, 0x0201, 0x0084, 0x0048, 0x0201, 0x0090, 0x0009, 0x0201, 0x0091, 0x0019, 0x1801,
    0x0e01, 0x0801, 0x0401, 0x0201, 0x0073, 0x0065, 0x0201, 0x0056, 0x0074, 0x0401, 0x0201,
    0x0047, 0x0066, 0x0083, 0x0601, 0x0201, 0x0038, 0x0201, 0x0075, 0x0057, 0x0201, 0x0092,
    0x0029, 0x0e01, 0x0801, 0x0401, 0x0201, 0x0067, 0x0085, 0x0201, 0x0058, 0x0039, 0x0201,
    0x0093, 0x0201, 0x0049, 0x0086, 0x0601, 0x0201, 0x00a0, 0x0201, 0x0068, 0x000a, 0x0201,
    0x00a1, 0x001a, 0x4401, 0x1801, 0x0c01, 0x0401, 0x0201, 0x00a2, 0x002a, 0x0401, 0x0201,
    0x0095, 0x0059, 0x0201, 0x00a3, 0x003a, 0x0801, 0x0401, 0x0201, 0x004a, 0x0096, 0x0201,
    0x00b0, 0x000b, 0x0201, 0x00b1, 0x001b, 0x1401, 0x0801, 0x0201, 0x00b2, 0x0401, 0x0201,
    0x0076, 0x0077, 0x0094, 0x0601, 0x0401, 0x0201, 0x0087, 0x0078, 0x00a4, 0x0401, 0x0201,
    0x0069, 0x00a5, 0x002b, 0x0c01, 0x0601, 0x0401, 0x0201, 0x005a, 0x0088, 0x00b3, 0x0201,
    0x003b, 0x0201, 0x0079, 0x00a6, 0x0601, 0x0401, 0x0201, 0x006a, 0x00b4, 0x00c0, 0x0401,
    0x0201, 0x000c, 0x0098, 0x00c1, 0x3c01, 0x1601, 0x0a01, 0x0601, 0x0201, 0x001c, 0x0201,
    0x0089, 0x00b5, 0x0201, 0x005b, 0x00c2, 0x0401, 0x0201, 0x002c, 0x003c, 0x0401, 0x0201,
    0x00b6, 0x006b, 0x0201, 0x00c4, 0x004c, 0x1001, 0x0801, 0x0401, 0x0201, 0x00a8, 0x008a,
    0x0201, 0x00d0, 0x000d, 0x0201, 0x00d1, 0x0201, 0x004b, 0x0201, 0x0097, 0x00a7, 0x0c01,
    0x0601, 0x0201, 0x00c3, 0x0201, 0x007a, 0x0099, 0x0401, 0x0201, 0x00c5, 0x005c, 0x00b7,
    0x0401, 0x0201, 0x001d, 0x00d2, 0x0201, 0x002d, 0x0201, 0x007b, 0x00d3, 0x3401, 0x1c01,
    0x0c01, 0x0401, 0x0201, 0x003d, 0x00c6, 0x0401, 0x0201, 0x006c, 0x00a9, 0x0201, 0x009a,
    0x00d4, 0x0801, 0x0401, 0x0201, 0x00b8, 0x008b, 0x0201, 0x004d, 0x00c7, 0x0401, 0x0201,
    0x007c, 0x00d5, 0x0201, 0x005d, 0x00e0, 0x0a01, 0x0401, 0x0201, 0x00e1, 0x001e, 0x0401,
    0x0201, 0x000e, 0x002e, 0x00e2, 0x0801, 0x0401, 0x0201, 0x00e3, 0x006d, 0x0201, 0x008c,
    0x00e4, 0x0401, 0x0201, 0x00e5, 0x00ba, 0x00f0, 0x2601, 0x1001, 0x0401, 0x0201, 0x00f1,
    0x001f, 0x0601, 0x0401, 0x0201, 0x00aa, 0x009b, 0x00b9, 0x0201, 0x003e, 0x0201, 0x00d6,
    0x00c8, 0x0c01, 0x0601, 0x0201, 0x004e, 0x0201, 0x00d7, 0x007d, 0x0201, 0x00ab, 0x0201,
    0x005e, 0x00c9, 0x0601, 0x0201, 0x000f, 0x0201, 0x009c, 0x006e, 0x0201, 0x00f2, 0x002f,
    0x2001, 0x1001, 0x0601, 0x0401, 0x0201, 0x00d8, 0x008d, 0x003f, 0x0601, 0x0201, 0x00f3,
    0x0201, 0x00e6, 0x00ca, 0x0201, 0x00f4, 0x004f, 0x0801, 0x0401, 0x0201, 0x00bb, 0x00ac,
    0x0201, 0x00e7, 0x00f5, 0x0401, 0x0201, 0x00d9, 0x009d, 0x0201, 0x005f, 0x00e8, 0x1e01,
    0x0c01, 0x0601, 0x0201, 0x006f, 0x0201, 0x00f6, 0x00cb, 0x0401, 0x0201, 0x00bc, 0x00ad,
    0x00da, 0x0801, 0x0201, 0x00f7, 0x0401, 0x0201, 0x007e, 0x007f, 0x008e, 0x0601, 0x0401,
    0x0201, 0x009e, 0x00ae, 0x00cc, 0x0201, 0x00f8, 0x008f, 0x1201, 0x0801, 0x0401, 0x0201,
    0x00db, 0x00bd, 0x0201, 0x00ea, 0x00f9, 0x0401, 0x0201, 0x009f, 0x00eb, 0x0201, 0x00be,
    0x0201, 0x00cd, 0x00fa, 0x0e01, 0x0401, 0x0201, 0x00dd, 0x00ec, 0x0601, 0x0401, 0x0201,
    0x00e9, 0x00af, 0x00dc, 0x0201, 0x00ce, 0x00fb, 0x0801, 0x0401, 0x0201, 0x00bf, 0x00de,
    0x0201, 0x00cf, 0x00ee, 0x0401, 0x0201, 0x00df, 0x00ef, 0x0201, 0x00ff, 0x0201, 0x00ed,
    0x0201, 0x00fd, 0x0201, 0x00fc, 0x00fe,

    // huffman_table_15[511]
    0x1001, 0x0601, 0x0201, 0x0000, 0x0201, 0x0010, 0x0001, 0x0201, 0x0011, 0x0401, 0x0201,
    0x0020, 0x0002, 0x0201, 0x0021, 0x0012, 0x3201, 0x1001, 0x0601, 0x0201, 0x0022, 0x0201,
    0x0030, 0x0031, 0x0601, 0x0201, 0x0013, 0x0201, 0x0003, 0x0040, 0x0201, 0x0032, 0x0023,
    0x0e01, 0x0601, 0x0401, 0x0201, 0x0004, 0x0014, 0x0041, 0x0401, 0x0201, 0x0033, 0x0042,
    0x0201, 0x0024, 0x0043, 0x0a01, 0x0601, 0x0201, 0x0034, 0x0201, 0x0050, 0x0005, 0x0201,
    0x0051, 0x0015, 0x0401, 0x0201, 0x0052, 0x0025, 0x0401, 0x0201, 0x0044, 0x0053, 0x0061,
    0x5a01, 0x2401, 0x1201, 0x0a01, 0x0601, 0x0201, 0x0035, 0x0201, 0x0060, 0x0006, 0x0201,
    0x0016, 0x0062, 0x0401, 0x0201, 0x0026, 0x0054, 0x0201, 0x0045, 0x0063, 0x0a01, 0x0601,
    0x0201, 0x0036, 0x0201, 0x0070, 0x0007, 0x0201, 0x0071, 0x0055, 0x0401, 0x0201, 0x0017,
    0x0064, 0x0201, 0x0072, 0x0027, 0x1801, 0x1001, 0x0801, 0x0401, 0x0201, 0x0046, 0x0073,
    0x0201, 0x0037, 0x0065, 0x0401, 0x0201, 0x0056, 0x0080, 0x0201, 0x0008, 0x0074, 0x0401,
    0x0201, 0x0081, 0x0018, 0x0201, 0x0082, 0x0028, 0x1001, 0x0801, 0x0401, 0x0201, 0x0047,
    0x0066, 0x0201, 0x0083, 0x0038, 0x0401, 0x0201, 0x0075, 0x0057, 0x0201, 0x0084, 0x0048,
    0x0601, 0x0401, 0x0201, 0x0090, 0x0019, 0x0091, 0x0401, 0x0201, 0x0092, 0x0076, 0x0201,
    0x0067, 0x0029, 0x5c01, 0x2401, 0x1201, 0x0a01, 0x0401, 0x0201, 0x0085, 0x0058, 0x0401,
    0x0201, 0x0009, 0x0077, 0x0093, 0x0401, 0x0201, 0x0039, 0x0094, 0x0201, 0x0049, 0x0086,
    0x0a01, 0x0601, 0x0201, 0x0068, 0x0201, 0x00a0, 0x000a, 0x0201, 0x00a1, 0x001a, 0x0401,
    0x0201, 0x00a2, 0x002a, 0x0201, 0x0095, 0x0059, 0x1a01, 0x0e01, 0x0601, 0x0201, 0x00a3,
    0x0201, 0x003a, 0x0087, 0x0401, 0x0201, 0x0078, 0x00a4, 0x0201, 0x004a, 0x0096, 0x0601,
    0x0401, 0x0201, 0x0069, 0x00b0, 0x00b1, 0x0401, 0x0201, 0x001b, 0x00a5, 0x00b2, 0x0e01,
    0x0801, 0x0401, 0x0201, 0x005a, 0x002b, 0x0201, 0x0088, 0x0097, 0x0201, 0x00b3, 0x0201,
    0x0079, 0x003b, 0x0801, 0x0401, 0x0201, 0x006a, 0x00b4, 0x0201, 0x004b, 0x00c1, 0x0401,
    0x0201, 0x0098, 0x0089, 0x0201, 0x001c, 0x00b5, 0x5001, 0x2201, 0x1001, 0x0601, 0x0401,
    0x0201, 0x005b, 0x002c, 0x00c2, 0x0601, 0x0401, 0x0201, 0x000b, 0x00c0, 0x00a6, 0x0201,
    0x00a7, 0x007a, 0x0a01, 0x0401, 0x0201, 0x00c3, 0x003c, 0x0401, 0x0201, 0x000c, 0x0099,
    0x00b6, 0x0401, 0x0201, 0x006b, 0x00c4, 0x0201, 0x004c, 0x00a8, 0x1401, 0x0a01, 0x0401,
    0x0201, 0x008a, 0x00c5, 0x0401, 0x0201, 0x00d0, 0x005c, 0x00d1, 0x0401, 0x0201, 0x00b7,
    0x007b, 0x0201, 0x001d, 0x0201, 0x000d, 0x002d, 0x0c01, 0x0401, 0x0201, 0x00d2, 0x00d3,
    0x0401, 0x0201, 0x003d, 0x00c6, 0x0201, 0x006c, 0x00a9, 0x0601, 0x0401, 0x0201, 0x009a,
    0x00b8, 0x00d4, 0x0401, 0x0201, 0x008b, 0x004d, 0x0201, 0x00c7, 0x007c, 0x4401, 0x2201,
    0x1201, 0x0a01, 0x0401, 0x0201, 0x00d5, 0x005d, 0x0401, 0x0201, 0x00e0, 0x000e, 0x00e1,
    0x0401, 0x0201, 0x001e, 0x00e2, 0x0201, 0x00aa, 0x002e, 0x0801, 0x0401, 0x0201, 0x00b9,
    0x009b, 0x0201, 0x00e3, 0x00d6, 0x0401, 0x0201, 0x006d, 0x003e, 0x0201, 0x00c8, 0x008c,
    0x1001, 0x0801, 0x0401, 0x0201, 0x00e4, 0x004e, 0x0201, 0x00d7, 0x007d, 0x0401, 0x0201,
    0x00e5, 0x00ba, 0x0201, 0x00ab, 0x005e, 0x0801, 0x0401, 0x0201, 0x00c9, 0x009c, 0x0201,
    0x00f1, 0x001f, 0x0601, 0x0401, 0x0201, 0x00f0, 0x006e, 0x00f2, 0x0201, 0x002f, 0x00e6,
    0x2601, 0x1201, 0x0801, 0x0401, 0x0201, 0x00d8, 0x00f3, 0x0201, 0x003f, 0x00f4, 0x0601,
    0x0201, 0x004f, 0x0201, 0x008d, 0x00d9, 0x0201, 0x00bb, 0x00ca, 0x0801, 0x0401, 0x0201,
    0x00ac, 0x00e7, 0x0201, 0x007e, 0x00f5, 0x0801, 0x0401, 0x0201, 0x009d, 0x005f, 0x0201,
    0x00e8, 0x008e, 0x0201, 0x00f6, 0x00cb, 0x2201, 0x1201, 0x0a01, 0x0601, 0x0401, 0x0201,
    0x000f, 0x00ae, 0x006f, 0x0201, 0x00bc, 0x00da, 0x0401, 0x0201, 0x00ad, 0x00f7, 0x0201,
    0x007f, 0x00e9, 0x0801, 0x0401, 0x0201, 0x009e, 0x00cc, 0x0201, 0x00f8, 0x008f, 0x0401,
    0x0201, 0x00db, 0x00bd, 0x0201, 0x00ea, 0x00f9, 0x1001, 0x0801, 0x0401, 0x0201, 0x009f,
    0x00dc, 0x0201, 0x00cd, 0x00eb, 0x0401, 0x0201, 0x00be, 0x00fa, 0x0201, 0x00af, 0x00dd,
    0x0e01, 0x0601, 0x0401, 0x0201, 0x00ec, 0x00ce, 0x00fb, 0x0401, 0x0201, 0x00bf, 0x00ed,
    0x0201, 0x00de, 0x00fc, 0x0601, 0x0401, 0x0201, 0x00cf, 0x00fd, 0x00ee, 0x0401, 0x0201,
    0x00df, 0x00fe, 0x0201, 0x00ef, 0x00ff,

    // huffman_table_16[511]
    0x0201, 0x0000, 0x0601, 0x0201, 0x0010, 0x0201, 0x0001, 0x0011, 0x2a01, 0x0801, 0x0401,
    0x0201, 0x0020, 0x0002, 0x0201, 0x0021, 0x0012, 0x0a01, 0x0601, 0x0201, 0x0022, 0x0201,
    0x0030, 0x0003, 0x0201, 0x0031, 0x0013, 0x0a01, 0x0401, 0x0201, 0x0032, 0x0023, 0x0401,
    0x0201, 0x0040, 0x0004, 0x0041, 0x0601, 0x0201, 0x0014, 0x0201, 0x0033, 0x0042, 0x0401,
    0x0201, 0x0024, 0x0050, 0x0201, 0x0043, 0x0034, 0x8a01, 0x2801, 0x1001, 0x0601, 0x0401,
    0x0201, 0x0005, 0x0015, 0x0051, 0x0401, 0x0201, 0x0052, 0x0025, 0x0401, 0x0201, 0x0044,
    0x0035, 0x0053, 0x0a01, 0x0601, 0x0401, 0x0201, 0x0060, 0x0006, 0x0061, 0x0201, 0x0016,
    0x0062, 0x0801, 0x0401, 0x0201, 0x0026, 0x0054, 0x0201, 0x0045, 0x0063, 0x0401, 0x0201,
    0x0036, 0x0070, 0x0071, 0x2801, 0x1201, 0x0801, 0x0201, 0x0017, 0x0201, 0x0007, 0x0201,
    0x0055, 0x0064, 0x0401, 0x0201, 0x0072, 0x0027, 0x0401, 0x0201, 0x0046, 0x0065, 0x0073,
    0x0a01, 0x0601, 0x0201, 0x0037, 0x0201, 0x0056, 0x0008, 0x0201, 0x0080, 0x0081, 0x0601,
    0x0201, 0x0018, 0x0201, 0x0074, 0x0047, 0x0201, 0x0082, 0x0201, 0x0028, 0x0066, 0x1801,
    0x0e01, 0x0801, 0x0401, 0x0201, 0x0083, 0x0038, 0x0201, 0x0075, 0x0084, 0x0401, 0x0201,
    0x0048, 0x0090, 0x0091, 0x0601, 0x0201, 0x0019, 0x0201, 0x0009, 0x0076, 0x0201, 0x0092,
    0x0029, 0x0e01, 0x0801, 0x0401, 0x0201, 0x0085, 0x0058, 0x0201, 0x0093, 0x0039, 0x0401,
    0x0201, 0x00a0, 0x000a, 0x001a, 0x0801, 0x0201, 0x00a2, 0x0201, 0x0067, 0x0201, 0x0057,
    0x0049, 0x0601, 0x0201, 0x0094, 0x0201, 0x0077, 0x0086, 0x0201, 0x00a1, 0x0201, 0x0068,
    0x0095, 0xdc01, 0x7e01, 0x3201, 0x1a01, 0x0c01, 0x0601, 0x0201, 0x002a, 0x0201, 0x0059,
    0x003a, 0x0201, 0x00a3, 0x0201, 0x0087, 0x0078, 0x0801, 0x0401, 0x0201, 0x00a4, 0x004a,
    0x0201, 0x0096, 0x0069, 0x0401, 0x0201, 0x00b0, 0x000b, 0x00b1, 0x0a01, 0x0401, 0x0201,
    0x001b, 0x00b2, 0x0201, 0x002b, 0x0201, 0x00a5, 0x005a, 0x0601, 0x0201, 0x00b3, 0x0201,
    0x00a6, 0x006a, 0x0401, 0x0201, 0x00b4, 0x004b, 0x0201, 0x000c, 0x00c1, 0x1e01, 0x0e01,
    0x0601, 0x0401, 0x0201, 0x00b5, 0x00c2, 0x002c, 0x0401, 0x0201, 0x00a7, 0x00c3, 0x0201,
    0x006b, 0x00c4, 0x0801, 0x0201, 0x001d, 0x0401, 0x0201, 0x0088, 0x0097, 0x003b, 0x0401,
    0x0201, 0x00d1, 0x00d2, 0x0201, 0x002d, 0x00d3, 0x1201, 0x0601, 0x0401, 0x0201, 0x001e,
    0x002e, 0x00e2, 0x0601, 0x0401, 0x0201, 0x0079, 0x0098, 0x00c0, 0x0201, 0x001c, 0x0201,
    0x0089, 0x005b, 0x0e01, 0x0601, 0x0201, 0x003c, 0x0201, 0x007a, 0x00b6, 0x0401, 0x0201,
    0x004c, 0x0099, 0x0201, 0x00a8, 0x008a, 0x0601, 0x0201, 0x000d, 0x0201, 0x00c5, 0x005c,
    0x0401, 0x0201, 0x003d, 0x00c6, 0x0201, 0x006c, 0x009a, 0x5801, 0x5601, 0x2401, 0x1001,
    0x0801, 0x0401, 0x0201, 0x008b, 0x004d, 0x0201, 0x00c7, 0x007c, 0x0401, 0x0201, 0x00d5,
    0x005d, 0x0201, 0x00e0, 0x000e, 0x0801, 0x0201, 0x00e3, 0x0401, 0x0201, 0x00d0, 0x00b7,
    0x007b, 0x0601, 0x0401, 0x0201, 0x00a9, 0x00b8, 0x00d4, 0x0201, 0x00e1, 0x0201, 0x00aa,
    0x00b9, 0x1801, 0x0a01, 0x0601, 0x0401, 0x0201, 0x009b, 0x00d6, 0x006d, 0x0201, 0x003e,
    0x00c8, 0x0601, 0x0401, 0x0201, 0x008c, 0x00e4, 0x004e, 0x0401, 0x0201, 0x00d7, 0x00e5,
    0x0201, 0x00ba, 0x00ab, 0x0c01, 0x0401, 0x0201, 0x009c, 0x00e6, 0x0401, 0x0201, 0x006e,
    0x00d8, 0x0201, 0x008d, 0x00bb, 0x0801, 0x0401, 0x0201, 0x00e7, 0x009d, 0x0201, 0x00e8,
    0x008e, 0x0401, 0x0201, 0x00cb, 0x00bc, 0x009e, 0x00f1, 0x0201, 0x001f, 0x0201, 0x000f,
    0x002f, 0x4201, 0x3801, 0x0201, 0x00f2, 0x3401, 0x3201, 0x1401, 0x0801, 0x0201, 0x00bd,
    0x0201, 0x005e, 0x0201, 0x007d, 0x00c9, 0x0601, 0x0201, 0x00ca, 0x0201, 0x00ac, 0x007e,
    0x0401, 0x0201, 0x00da, 0x00ad, 0x00cc, 0x0a01, 0x0601, 0x0201, 0x00ae, 0x0201, 0x00db,
    0x00dc, 0x0201, 0x00cd, 0x00be, 0x0601, 0x0401, 0x0201, 0x00eb, 0x00ed, 0x00ee, 0x0601,
    0x0401, 0x0201, 0x00d9, 0x00ea, 0x00e9, 0x0201, 0x00de, 0x0401, 0x0201, 0x00dd, 0x00ec,
    0x00ce, 0x003f, 0x00f0, 0x0401, 0x0201, 0x00f3, 0x00f4, 0x0201, 0x004f, 0x0201, 0x00f5,
    0x005f, 0x0a01, 0x0201, 0x00ff, 0x0401, 0x0201, 0x00f6, 0x006f, 0x0201, 0x00f7, 0x007f,
    0x0c01, 0x0601, 0x0201, 0x008f, 0x0201, 0x00f8, 0x00f9, 0x0401, 0x0201, 0x009f, 0x00fa,
    0x00af, 0x0801, 0x0401, 0x0201, 0x00fb, 0x00bf, 0x0201, 0x00fc, 0x00cf, 0x0401, 0x0201,
    0x00fd, 0x00df, 0x0201, 0x00fe, 0x00ef,

    // huffman_table_24[512]
    0x3c01, 0x0801, 0x0401, 0x0201, 0x0000, 0x0010, 0x0201, 0x0001, 0x0011, 0x0e01, 0x0601,
    0x0401, 0x0201, 0x0020, 0x0002, 0x0021, 0x0201, 0x0012, 0x0201, 0x0022, 0x0201, 0x0030,
    0x0003, 0x0e01, 0x0401, 0x0201, 0x0031, 0x0013, 0x0401, 0x0201, 0x0032, 0x0023, 0x0401,
    0x0201, 0x0040, 0x0004, 0x0041, 0x0801, 0x0401, 0x0201, 0x0014, 0x0033, 0x0201, 0x0042,
    0x0024, 0x0601, 0x0401, 0x0201, 0x0043, 0x0034, 0x0051, 0x0601, 0x0401, 0x0201, 0x0050,
    0x0005, 0x0015, 0x0201, 0x0052, 0x0025, 0xfa01, 0x6201, 0x2201, 0x1201, 0x0a01, 0x0401,
    0x0201, 0x0044, 0x0053, 0x0201, 0x0035, 0x0201, 0x0060, 0x0006, 0x0401, 0x0201, 0x0061,
    0x0016, 0x0201, 0x0062, 0x0026, 0x0801, 0x0401, 0x0201, 0x0054, 0x0045, 0x0201, 0x0063,
    0x0036, 0x0401, 0x0201, 0x0071, 0x0055, 0x0201, 0x0064, 0x0046, 0x2001, 0x0e01, 0x0601,
    0x0201, 0x0072, 0x0201, 0x0027, 0x0037, 0x0201, 0x0073, 0x0401, 0x0201, 0x0070, 0x0007,
    0x0017, 0x0a01, 0x0401, 0x0201, 0x0065, 0x0056, 0x0401, 0x0201, 0x0080, 0x0008, 0x0081,
    0x0401, 0x0201, 0x0074, 0x0047, 0x0201, 0x0018, 0x0082, 0x1001, 0x0801, 0x0401, 0x0201,
    0x0028, 0x0066, 0x0201, 0x0083, 0x0038, 0x0401, 0x0201, 0x0075, 0x0057, 0x0201, 0x0084,
    0x0048, 0x0801, 0x0401, 0x0201, 0x0091, 0x0019, 0x0201, 0x0092, 0x0076, 0x0401, 0x0201,
    0x0067, 0x0029, 0x0201, 0x0085, 0x0058, 0x5c01, 0x2201, 0x1001, 0x0801, 0x0401, 0x0201,
    0x0093, 0x0039, 0x0201, 0x0094, 0x0049, 0x0401, 0x0201, 0x0077, 0x0086, 0x0201, 0x0068,
    0x00a1, 0x0801, 0x0401, 0x0201, 0x00a2, 0x002a, 0x0201, 0x0095, 0x0059, 0x0401, 0x0201,
    0x00a3, 0x003a, 0x0201, 0x0087, 0x0201, 0x0078, 0x004a, 0x1601, 0x0c01, 0x0401, 0x0201,
    0x00a4, 0x0096, 0x0401, 0x0201, 0x0069, 0x00b1, 0x0201, 0x001b, 0x00a5, 0x0601, 0x0201,
    0x00b2, 0x0201, 0x005a, 0x002b, 0x0201, 0x0088, 0x00b3, 0x1001, 0x0a01, 0x0601, 0x0201,
    0x0090, 0x0201, 0x0009, 0x00a0, 0x0201, 0x0097, 0x0079, 0x0401, 0x0201, 0x00a6, 0x006a,
    0x00b4, 0x0c01, 0x0601, 0x0201, 0x001a, 0x0201, 0x000a, 0x00b0, 0x0201, 0x003b, 0x0201,
    0x000b, 0x00c0, 0x0401, 0x0201, 0x004b, 0x00c1, 0x0201, 0x0098, 0x0089, 0x4301, 0x2201,
    0x1001, 0x0801, 0x0401, 0x0201, 0x001c, 0x00b5, 0x0201, 0x005b, 0x00c2, 0x0401, 0x0201,
    0x002c, 0x00a7, 0x0201, 0x007a, 0x00c3, 0x0a01, 0x0601, 0x0201, 0x003c, 0x0201, 0x000c,
    0x00d0, 0x0201, 0x00b6, 0x006b, 0x0401, 0x0201, 0x00c4, 0x004c, 0x0201, 0x0099, 0x00a8,
    0x1001, 0x0801, 0x0401, 0x0201, 0x008a, 0x00c5, 0x0201, 0x005c, 0x00d1, 0x0401, 0x0201,
    0x00b7, 0x007b, 0x0201, 0x001d, 0x00d2, 0x0901, 0x0401, 0x0201, 0x002d, 0x00d3, 0x0201,
    0x003d, 0x00c6, 0x55fa, 0x0401, 0x0201, 0x006c, 0x00a9, 0x0201, 0x009a, 0x00d4, 0x2001,
    0x1001, 0x0801, 0x0401, 0x0201, 0x00b8, 0x008b, 0x0201, 0x004d, 0x00c7, 0x0401, 0x0201,
    0x007c, 0x00d5, 0x0201, 0x005d, 0x00e1, 0x0801, 0x0401, 0x0201, 0x001e, 0x00e2, 0x0201,
    0x00aa, 0x00b9, 0x0401, 0x0201, 0x009b, 0x00e3, 0x0201, 0x00d6, 0x006d, 0x1401, 0x0a01,
    0x0601, 0x0201, 0x003e, 0x0201, 0x002e, 0x004e, 0x0201, 0x00c8, 0x008c, 0x0401, 0x0201,
    0x00e4, 0x00d7, 0x0401, 0x0201, 0x007d, 0x00ab, 0x00e5, 0x0a01, 0x0401, 0x0201, 0x00ba,
    0x005e, 0x0201, 0x00c9, 0x0201, 0x009c, 0x006e, 0x0801, 0x0201, 0x00e6, 0x0201, 0x000d,
    0x0201, 0x00e0, 0x000e, 0x0401, 0x0201, 0x00d8, 0x008d, 0x0201, 0x00bb, 0x00ca, 0x4a01,
    0x0201, 0x00ff, 0x4001, 0x3a01, 0x2001, 0x1001, 0x0801, 0x0401, 0x0201, 0x00ac, 0x00e7,
    0x0201, 0x007e, 0x00d9, 0x0401, 0x0201, 0x009d, 0x00e8, 0x0201, 0x008e, 0x00cb, 0x0801,
    0x0401, 0x0201, 0x00bc, 0x00da, 0x0201, 0x00ad, 0x00e9, 0x0401, 0x0201, 0x009e, 0x00cc,
    0x0201, 0x00db, 0x00bd, 0x1001, 0x0801, 0x0401, 0x0201, 0x00ea, 0x00ae, 0x0201, 0x00dc,
    0x00cd, 0x0401, 0x0201, 0x00eb, 0x00be, 0x0201, 0x00dd, 0x00ec, 0x0801, 0x0401, 0x0201,
    0x00ce, 0x00ed, 0x0201, 0x00de, 0x00ee, 0x000f, 0x0401, 0x0201, 0x00f0, 0x001f, 0x00f1,
    0x0401, 0x0201, 0x00f2, 0x002f, 0x0201, 0x00f3, 0x003f, 0x1201, 0x0801, 0x0401, 0x0201,
    0x00f4, 0x004f, 0x0201, 0x00f5, 0x005f, 0x0401, 0x0201, 0x00f6, 0x006f, 0x0201, 0x00f7,
    0x0201, 0x007f, 0x008f, 0x0a01, 0x0401, 0x0201, 0x00f8, 0x00f9, 0x0401, 0x0201, 0x009f,
    0x00af, 0x00fa, 0x0801, 0x0401, 0x0201, 0x00fb, 0x00bf, 0x0201, 0x00fc, 0x00cf, 0x0401,
    0x0201, 0x00fd, 0x00df, 0x0201, 0x00fe, 0x00ef,

    // huffman_table_32[31]
    0x0201, 0x0000, 0x0801, 0x0401, 0x0201, 0x0008, 0x0004, 0x0201, 0x0001, 0x0002, 0x0801,
    0x0401, 0x0201, 0x000c, 0x000a, 0x0201, 0x0003, 0x0006, 0x0601, 0x0201, 0x0009, 0x0201,
    0x0005, 0x0007, 0x0401, 0x0201, 0x000e, 0x000d, 0x0201, 0x000f, 0x000b,

    // huffman_table_33[31]
    0x1001, 0x0801, 0x0401, 0x0201, 0x0000, 0x0001, 0x0201, 0x0002, 0x0003, 0x0401, 0x0201,
    0x0004, 0x0005, 0x0201, 0x0006, 0x0007, 0x0801, 0x0401, 0x0201, 0x0008, 0x0009, 0x0201,
    0x000a, 0x000b, 0x0401, 0x0201, 0x000c, 0x000d, 0x0201, 0x000e, 0x000f,
};
