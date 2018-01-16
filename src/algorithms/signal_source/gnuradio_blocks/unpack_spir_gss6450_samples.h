/*!
 * \file unpack_intspir_1bit_samples.h
 *
 * \brief Unpacks SPIR int samples to NSR 1 bit samples
 * \author Fran Fabra fabra (at) ice.csic.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is not part of GNSS-SDR.
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

#ifndef GNSS_SDR_UNPACK_SPIR_GSS6450_SAMPLES_H
#define GNSS_SDR_UNPACK_SPIR_GSS6450_SAMPLES_H

#include <gnuradio/block.h>

class unpack_spir_gss6450_samples;

typedef boost::shared_ptr<unpack_spir_gss6450_samples> unpack_spir_gss6450_samples_sptr;

unpack_spir_gss6450_samples_sptr make_unpack_spir_gss6450_samples(unsigned int n_chann, unsigned int sel_ch, int samp_item, size_t item_size);


class unpack_spir_gss6450_samples: public gr::block
{
public:
    int general_work (int noutput_items,
                          gr_vector_const_void_star &input_items,
                          gr_vector_void_star &output_items);
    friend unpack_spir_gss6450_samples_sptr make_unpack_spir_gss6450_samples_sptr(unsigned int n_chann, unsigned int sel_ch, int samp_item, size_t item_size);
    unpack_spir_gss6450_samples(unsigned int n_chann, unsigned int sel_ch, int samp_item, size_t item_size);
    ~unpack_spir_gss6450_samples();

private:
    unsigned int d_channels;
    unsigned int d_sel_ch;
    unsigned int ch_processing;
    int d_samp_item;
    int samp_frame;
    int adc_bits;
    size_t item_size_;
    void process_sample(gr_complex* out);
    void swap_data(int& data);
    void compute_two_complement(int& data);
    bool i_;
    bool new_sample;
    int i_data;
    int q_data;
};

#endif
