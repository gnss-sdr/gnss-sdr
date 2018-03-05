/*!
 * \file unpack_2bit_samples.h
 *
 * \brief Unpacks 2 bit samples
 *  samples may be packed in any of the following ways:
 *  1) Into bytes [ item == byte ]
 *      1a) Big endian ordering within the byte
 *      1b) Little endian ordering within the byte
 *  2) Into shorts [ item == short ]
 *      2a) Big endian ordering of bytes, big endian within the byte
 *      2b) Big endian ordering of bytes, little endian within the byte
 *      2c) Little endian ordering of bytes, big endian within the byte
 *      2d) Little endian ordering of bytes, little endian within the byte
 *
 *  Within a byte the two possibilities look like this:
 *    7       6       5       4       3       2       1       0     : Bit number
 *  x_n,1   x_n,0  x_n+1,1 x_n+1,0 x_n+2,1 x_n+2,0 x_n+3,1 x_n+3,0  : Little endian
 * x_n+3,1 x_n+3,0 x_n+2,1 x_n+2,0 x_n+1,0 x_n+1,0  x_n,1   x_n, 0  : Big Endian
 *
 *  For a short (uint16_t) the bytes are either transmitted as follows:
 *
 *   1       0         : Byte number
 * Byte_n   Byte_n+1   : Little endian
 * Byte_n+1 Byte_n     : Bit endian
 *
 *  The two bit values are assumed to have the following mapping:
 *
 *  x_1  x_0     Value
 *   0    0       +1
 *   0    1       +3
 *   1    0       -3
 *   1    1       -1
 *
 *   Letting x denote the two's complement interpretation of x_1 x_0, then:
 *
 *   Value = 2*x + 1
 *
 *   We want to output the data in the order:
 *
 *   Value_0, Value_1, Value_2, ..., Value_n, Value_n+1, Value_n+2, ...
 *
 * \author Cillian O'Driscoll cillian.odriscoll (at) gmail . com 
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

#ifndef GNSS_SDR_UNPACK_2BIT_SAMPLES_H
#define GNSS_SDR_UNPACK_2BIT_SAMPLES_H

#include <gnuradio/sync_interpolator.h>
#include <cstdint>

class unpack_2bit_samples;

typedef boost::shared_ptr<unpack_2bit_samples> unpack_2bit_samples_sptr;

unpack_2bit_samples_sptr make_unpack_2bit_samples(bool big_endian_bytes,
    size_t item_size,
    bool big_endian_items,
    bool reverse_interleaving = false);

/*!
 * \brief This class takes 2 bit samples that have been packed into bytes or
 * shorts as input and generates a byte for each sample. It generates eight
 * times as much data as is input (every two bits become 16 bits)
 */
class unpack_2bit_samples : public gr::sync_interpolator
{
private:
    friend unpack_2bit_samples_sptr
    make_unpack_2bit_samples_sptr(bool big_endian_bytes,
        size_t item_size,
        bool big_endian_items,
        bool reverse_interleaving);
    bool big_endian_bytes_;
    size_t item_size_;
    bool big_endian_items_;
    bool swap_endian_items_;
    bool swap_endian_bytes_;
    bool reverse_interleaving_;
    std::vector<int8_t> work_buffer_;

public:
    unpack_2bit_samples(bool big_endianBytes,
        size_t item_size,
        bool big_endian_items,
        bool reverse_interleaving);

    ~unpack_2bit_samples();

    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);
};

#endif
