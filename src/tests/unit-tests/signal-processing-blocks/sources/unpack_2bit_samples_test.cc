/*!
 * \file unpack_2bit_samples_test.cc
 * \brief  This file implements unit tests for the unpack_2bit_samples
 *      custom block
 * \author Cillian O'Driscoll, 2015. cillian.odriscoll (at) gmail.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include <gtest/gtest.h>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/vector_source_b.h>
#include <gnuradio/blocks/vector_source_s.h>
#include <gnuradio/blocks/vector_sink_b.h>
#include <gnuradio/blocks/stream_to_vector.h>
#include "unpack_2bit_samples.h"

std::vector<uint8_t> packData(std::vector<int8_t> const &raw_data,
    bool big_endian)
{
    std::vector<uint8_t> packed_data(raw_data.size() / 4);

    int shift = (big_endian ? 6 : 0);
    unsigned int j = 0;

    for (unsigned int i = 0; i < raw_data.size(); ++i)
        {
            unsigned val = static_cast<unsigned>((raw_data[i] - 1) / 2 & 0x03);

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


    boost::shared_ptr<gr::block> unpacker =
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

    for (unsigned int i = 0; i < raw_data.size(); ++i)
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


    boost::shared_ptr<gr::block> unpacker =
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

    for (unsigned int i = 0; i < raw_data.size(); ++i)
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
    for (unsigned int ii = 0; ii < packed_data.size(); ii += item_size)
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


    boost::shared_ptr<gr::block> unpacker =
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

    for (unsigned int i = 0; i < raw_data.size(); ++i)
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
    for (unsigned int ii = 0; ii < packed_data.size(); ii += item_size)
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


    boost::shared_ptr<gr::block> unpacker =
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

    for (unsigned int i = 0; i < raw_data.size(); ++i)
        {
            EXPECT_EQ(raw_data[i], static_cast<int8_t>(unpacked_data[i]));
        }
}
