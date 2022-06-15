/*!
 * \file unpack_2bit_samples.cc
 *
 * \brief Unpacks 2 bit samples that have been packed into bytes or shorts
 * \author Cillian O'Driscoll cillian.odriscoll (at) gmail.com
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
#include <gnuradio/io_signature.h>

struct byte_2bit_struct
{
    signed sample_0 : 2;  // <- 2 bits wide only
    signed sample_1 : 2;  // <- 2 bits wide only
    signed sample_2 : 2;  // <- 2 bits wide only
    signed sample_3 : 2;  // <- 2 bits wide only
};


union byte_and_samples
{
    int8_t byte;
    byte_2bit_struct samples;
};


bool systemIsBigEndian()
{
    union
    {
        uint32_t i;
        char c[4];
    } test_int = {0x01020304};

    return test_int.c[0] == 1;
}


bool systemBytesAreBigEndian()
{
    byte_and_samples b{};
    b.byte = static_cast<int8_t>(0x01);
    if (*reinterpret_cast<char *>(&b.byte) == 1)
        {
            return false;
        }

    return true;
}


void swapEndianness(int8_t const *in, std::vector<int8_t> &out, size_t item_size, unsigned int ninput_items)
{
    unsigned int i;
    unsigned int j = 0;
    int k = 0;
    int l = 0;
    size_t skip = item_size - 1;

    for (i = 0; i < ninput_items; ++i)
        {
            k = static_cast<int>(j + skip);
            l = static_cast<int>(j);
            while (k >= l)
                {
                    out[j++] = in[k--];
                }
        }
}


unpack_2bit_samples_sptr make_unpack_2bit_samples(bool big_endian_bytes,
    size_t item_size,
    bool big_endian_items,
    bool reverse_interleaving)
{
    return unpack_2bit_samples_sptr(
        new unpack_2bit_samples(big_endian_bytes,
            item_size,
            big_endian_items,
            reverse_interleaving));
}


unpack_2bit_samples::unpack_2bit_samples(bool big_endian_bytes,
    size_t item_size,
    bool big_endian_items,
    bool reverse_interleaving)
    : sync_interpolator("unpack_2bit_samples",
          gr::io_signature::make(1, 1, item_size),
          gr::io_signature::make(1, 1, sizeof(char)),
          4 * item_size),  // we make 4 bytes out for every byte in
      item_size_(item_size),
      big_endian_bytes_(big_endian_bytes),
      big_endian_items_(big_endian_items),
      swap_endian_items_(false),
      reverse_interleaving_(reverse_interleaving)
{
    bool big_endian_system = systemIsBigEndian();

    // Only swap the item bytes if the item size > 1 byte and the system
    // endianness is not the same as the item endianness:
    swap_endian_items_ = (item_size_ > 1) &&
                         (big_endian_system != big_endian_items);

    bool big_endian_bytes_system = systemBytesAreBigEndian();

    swap_endian_bytes_ = (big_endian_bytes_system != big_endian_bytes_);
}


int unpack_2bit_samples::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    auto const *in = reinterpret_cast<signed char const *>(input_items[0]);
    auto *out = reinterpret_cast<int8_t *>(output_items[0]);

    size_t ninput_bytes = noutput_items / 4;
    size_t ninput_items = ninput_bytes / item_size_;

    // Handle endian swap if needed
    if (swap_endian_items_)
        {
            work_buffer_ = std::vector<int8_t>(ninput_bytes);
            swapEndianness(in, work_buffer_, item_size_, ninput_items);

            in = const_cast<signed char const *>(&work_buffer_[0]);
        }

    // Here the in pointer can be interpreted as a stream of bytes to be
    // converted. But we now have two possibilities:
    // 1) The samples in a byte are in big endian order
    // 2) The samples in a byte are in little endian order

    byte_and_samples raw_byte{};
    int n = 0;

    if (!reverse_interleaving_)
        {
            if (swap_endian_bytes_)
                {
                    for (size_t i = 0; i < ninput_bytes; ++i)
                        {
                            // Read packed input sample (1 byte = 4 samples)
                            raw_byte.byte = in[i];

                            out[n++] = static_cast<int8_t>(2 * raw_byte.samples.sample_3 + 1);
                            out[n++] = static_cast<int8_t>(2 * raw_byte.samples.sample_2 + 1);
                            out[n++] = static_cast<int8_t>(2 * raw_byte.samples.sample_1 + 1);
                            out[n++] = static_cast<int8_t>(2 * raw_byte.samples.sample_0 + 1);
                        }
                }
            else
                {
                    for (size_t i = 0; i < ninput_bytes; ++i)
                        {
                            // Read packed input sample (1 byte = 4 samples)
                            raw_byte.byte = in[i];

                            out[n++] = static_cast<int8_t>(2 * raw_byte.samples.sample_0 + 1);
                            out[n++] = static_cast<int8_t>(2 * raw_byte.samples.sample_1 + 1);
                            out[n++] = static_cast<int8_t>(2 * raw_byte.samples.sample_2 + 1);
                            out[n++] = static_cast<int8_t>(2 * raw_byte.samples.sample_3 + 1);
                        }
                }
        }
    else
        {
            if (swap_endian_bytes_)
                {
                    for (size_t i = 0; i < ninput_bytes; ++i)
                        {
                            // Read packed input sample (1 byte = 4 samples)
                            raw_byte.byte = in[i];

                            out[n++] = static_cast<int8_t>(2 * raw_byte.samples.sample_2 + 1);
                            out[n++] = static_cast<int8_t>(2 * raw_byte.samples.sample_3 + 1);
                            out[n++] = static_cast<int8_t>(2 * raw_byte.samples.sample_0 + 1);
                            out[n++] = static_cast<int8_t>(2 * raw_byte.samples.sample_1 + 1);
                        }
                }
            else
                {
                    for (size_t i = 0; i < ninput_bytes; ++i)
                        {
                            // Read packed input sample (1 byte = 4 samples)
                            raw_byte.byte = in[i];

                            out[n++] = static_cast<int8_t>(2 * raw_byte.samples.sample_1 + 1);
                            out[n++] = static_cast<int8_t>(2 * raw_byte.samples.sample_0 + 1);
                            out[n++] = static_cast<int8_t>(2 * raw_byte.samples.sample_3 + 1);
                            out[n++] = static_cast<int8_t>(2 * raw_byte.samples.sample_2 + 1);
                        }
                }
        }

    return noutput_items;
}
