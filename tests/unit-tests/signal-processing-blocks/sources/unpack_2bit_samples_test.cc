/*!
 * \file unpack_2bit_samples_test.cc
 * \brief  This file implements unit tests for the unpack_2bit_samples
 *      custom block
 * \author Cillian O'Driscoll, 2015. cillian.odriscoll (at) gmail.com
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "unpack_2bit_samples.h"
#include <gnuradio/blocks/stream_to_vector.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <cstddef>

#ifdef GR_GREATER_38
#include <gnuradio/blocks/vector_sink.h>
#include <gnuradio/blocks/vector_source.h>
#else
#include <gnuradio/blocks/vector_sink_b.h>
#include <gnuradio/blocks/vector_source_b.h>
#include <gnuradio/blocks/vector_source_s.h>
#endif

std::vector<uint8_t> packData(std::vector<int8_t> const &raw_data,
    bool big_endian)
{
    std::vector<uint8_t> packed_data(raw_data.size() / 4);

    int shift = (big_endian ? 6 : 0);
    unsigned int j = 0;

    for (signed char i : raw_data)
        {
            auto val = static_cast<unsigned>((i - 1) / 2 & 0x03);

            packed_data[j] |= val << shift;

            if (big_endian)
                {
                    shift -= 2;
                    if (shift < 0)
                        {
                            shift = 6;
                            j++;
                        }
                }
            else
                {
                    shift += 2;
                    if (shift > 6)
                        {
                            shift = 0;
                            j++;
                        }
                }
        }

    return packed_data;
}


TEST(Unpack2bitSamplesTest, CheckBigEndianByte)
{
    bool big_endian_bytes = true;
    size_t item_size = 1;
    bool big_endian_items = false;

    std::vector<int8_t> raw_data = {-1, 3, 1, -1, -3, 1, 3, 1};
    std::vector<uint8_t> packed_data = packData(raw_data, big_endian_bytes);
    std::vector<uint8_t> unpacked_data;

    gr::top_block_sptr top_block = gr::make_top_block("Unpack2bitSamplesTest");

    gr::blocks::vector_source_b::sptr source =
        gr::blocks::vector_source_b::make(packed_data);

    auto unpacker =
        make_unpack_2bit_samples(big_endian_bytes,
            item_size,
            big_endian_items);

    gr::blocks::stream_to_vector::sptr stov =
        gr::blocks::stream_to_vector::make(item_size, raw_data.size());

    gr::blocks::vector_sink_b::sptr sink =
        gr::blocks::vector_sink_b::make(raw_data.size());

    top_block->connect(source, 0, unpacker, 0);
    top_block->connect(unpacker, 0, stov, 0);
    top_block->connect(stov, 0, sink, 0);

    top_block->run();
    top_block->stop();

    unpacked_data = sink->data();

    EXPECT_EQ(raw_data.size(), unpacked_data.size());

    for (size_t i = 0; i < raw_data.size(); ++i)
        {
            EXPECT_EQ(raw_data[i], static_cast<int8_t>(unpacked_data[i]));
        }
}


TEST(Unpack2bitSamplesTest, CheckLittleEndianByte)
{
    bool big_endian_bytes = false;
    size_t item_size = 1;
    bool big_endian_items = false;

    std::vector<int8_t> raw_data = {-1, 3, 1, -1, -3, 1, 3, 1};
    std::vector<uint8_t> packed_data = packData(raw_data, big_endian_bytes);
    std::vector<uint8_t> unpacked_data;

    gr::top_block_sptr top_block = gr::make_top_block("Unpack2bitSamplesTest");

    gr::blocks::vector_source_b::sptr source =
        gr::blocks::vector_source_b::make(packed_data);

    auto unpacker =
        make_unpack_2bit_samples(big_endian_bytes,
            item_size,
            big_endian_items);

    gr::blocks::stream_to_vector::sptr stov =
        gr::blocks::stream_to_vector::make(item_size, raw_data.size());

    gr::blocks::vector_sink_b::sptr sink =
        gr::blocks::vector_sink_b::make(raw_data.size());

    top_block->connect(source, 0, unpacker, 0);
    top_block->connect(unpacker, 0, stov, 0);
    top_block->connect(stov, 0, sink, 0);

    top_block->run();
    top_block->stop();

    unpacked_data = sink->data();

    EXPECT_EQ(raw_data.size(), unpacked_data.size());

    for (size_t i = 0; i < raw_data.size(); ++i)
        {
            EXPECT_EQ(raw_data[i], static_cast<int8_t>(unpacked_data[i]));
        }
}


TEST(Unpack2bitSamplesTest, CheckBigEndianShortBigEndianByte)
{
    bool big_endian_bytes = true;
    size_t item_size = 2;
    bool big_endian_items = true;

    std::vector<int8_t> raw_data = {-1, 3, 1, -1, -3, 1, 3, 1};
    std::vector<uint8_t> packed_data = packData(raw_data, big_endian_bytes);
    // change the order of each pair of bytes:
    for (size_t ii = 0; ii < packed_data.size(); ii += item_size)
        {
            unsigned int kk = ii + item_size - 1;
            unsigned int jj = ii;
            while (kk > jj)
                {
                    uint8_t tmp = packed_data[jj];
                    packed_data[jj] = packed_data[kk];
                    packed_data[kk] = tmp;
                    --kk;
                    ++jj;
                }
        }

    // Now create a new big endian buffer:
    std::vector<int16_t> packed_data_short(
        reinterpret_cast<int16_t *>(&packed_data[0]),
        reinterpret_cast<int16_t *>(&packed_data[0]) + packed_data.size() / item_size);

    std::vector<uint8_t> unpacked_data;

    gr::top_block_sptr top_block = gr::make_top_block("Unpack2bitSamplesTest");

    gr::blocks::vector_source_s::sptr source =
        gr::blocks::vector_source_s::make(packed_data_short);

    auto unpacker =
        make_unpack_2bit_samples(big_endian_bytes,
            item_size,
            big_endian_items);

    gr::blocks::stream_to_vector::sptr stov =
        gr::blocks::stream_to_vector::make(1, raw_data.size());

    gr::blocks::vector_sink_b::sptr sink =
        gr::blocks::vector_sink_b::make(raw_data.size());

    top_block->connect(source, 0, unpacker, 0);
    top_block->connect(unpacker, 0, stov, 0);
    top_block->connect(stov, 0, sink, 0);

    top_block->run();
    top_block->stop();

    unpacked_data = sink->data();

    EXPECT_EQ(raw_data.size(), unpacked_data.size());

    for (size_t i = 0; i < raw_data.size(); ++i)
        {
            EXPECT_EQ(raw_data[i], static_cast<int8_t>(unpacked_data[i]));
        }
}


TEST(Unpack2bitSamplesTest, CheckBigEndianShortLittleEndianByte)
{
    bool big_endian_bytes = false;
    size_t item_size = 2;
    bool big_endian_items = true;

    std::vector<int8_t> raw_data = {-1, 3, 1, -1, -3, 1, 3, 1};
    std::vector<uint8_t> packed_data = packData(raw_data, big_endian_bytes);
    // change the order of each pair of bytes:
    for (size_t ii = 0; ii < packed_data.size(); ii += item_size)
        {
            unsigned int kk = ii + item_size - 1;
            unsigned int jj = ii;
            while (kk > jj)
                {
                    uint8_t tmp = packed_data[jj];
                    packed_data[jj] = packed_data[kk];
                    packed_data[kk] = tmp;
                    --kk;
                    ++jj;
                }
        }

    // Now create a new big endian buffer:
    std::vector<int16_t> packed_data_short(
        reinterpret_cast<int16_t *>(&packed_data[0]),
        reinterpret_cast<int16_t *>(&packed_data[0]) + packed_data.size() / item_size);

    std::vector<uint8_t> unpacked_data;

    gr::top_block_sptr top_block = gr::make_top_block("Unpack2bitSamplesTest");

    gr::blocks::vector_source_s::sptr source =
        gr::blocks::vector_source_s::make(packed_data_short);

    auto unpacker =
        make_unpack_2bit_samples(big_endian_bytes,
            item_size,
            big_endian_items);

    gr::blocks::stream_to_vector::sptr stov =
        gr::blocks::stream_to_vector::make(1, raw_data.size());

    gr::blocks::vector_sink_b::sptr sink =
        gr::blocks::vector_sink_b::make(raw_data.size());

    top_block->connect(source, 0, unpacker, 0);
    top_block->connect(unpacker, 0, stov, 0);
    top_block->connect(stov, 0, sink, 0);

    top_block->run();
    top_block->stop();

    unpacked_data = sink->data();

    EXPECT_EQ(raw_data.size(), unpacked_data.size());

    for (size_t i = 0; i < raw_data.size(); ++i)
        {
            EXPECT_EQ(raw_data[i], static_cast<int8_t>(unpacked_data[i]));
        }
}
