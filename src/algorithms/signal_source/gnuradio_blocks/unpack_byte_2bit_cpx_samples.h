/*!
 * \file unpack_byte_2bit_cpx_samples.h
 *
 * \brief Unpacks byte samples to 2 bits complex samples.
 *     Packing Order
 *     Most Significant Nibble  - Sample n
 *     Least Significant Nibble - Sample n+1
 *     Packing order in Nibble Q1 Q0 I1 I0
 * \author Javier Arribas jarribas (at) cttc.es
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

#ifndef GNSS_SDR_UNPACK_BYTE_2BIT_CPX_SAMPLES_H
#define GNSS_SDR_UNPACK_BYTE_2BIT_CPX_SAMPLES_H

#include <gnuradio/sync_interpolator.h>

class unpack_byte_2bit_cpx_samples;

typedef boost::shared_ptr<unpack_byte_2bit_cpx_samples> unpack_byte_2bit_cpx_samples_sptr;

unpack_byte_2bit_cpx_samples_sptr make_unpack_byte_2bit_cpx_samples();

/*!
 * \brief This class implements conversion between byte packet samples to 2bit_cpx samples
 *  1 byte = 2 x complex 2bit I, + 2bit Q samples
 */
class unpack_byte_2bit_cpx_samples : public gr::sync_interpolator
{
private:
    friend unpack_byte_2bit_cpx_samples_sptr make_unpack_byte_2bit_cpx_samples_sptr();

public:
    unpack_byte_2bit_cpx_samples();
    ~unpack_byte_2bit_cpx_samples();
    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);
};

#endif
