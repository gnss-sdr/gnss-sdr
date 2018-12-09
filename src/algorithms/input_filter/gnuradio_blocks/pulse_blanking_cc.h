/*!
 * \file pulse_blanking_cc.h
 * \brief Implements a pulse blanking algorithm
 * \author Javier Arribas (jarribas(at)cttc.es)
 *         Antonio Ramos  (antonio.ramosdet(at)gmail.com)
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018 (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_PULSE_BLANKING_H_
#define GNSS_SDR_PULSE_BLANKING_H_

#include <boost/shared_ptr.hpp>
#include <gnuradio/block.h>
#include <cstdint>

class pulse_blanking_cc;

typedef boost::shared_ptr<pulse_blanking_cc> pulse_blanking_cc_sptr;

pulse_blanking_cc_sptr make_pulse_blanking_cc(float pfa, int32_t length_, int32_t n_segments_est, int32_t n_segments_reset);


class pulse_blanking_cc : public gr::block
{
private:
    int32_t length_;
    int32_t n_segments;
    int32_t n_segments_est;
    int32_t n_segments_reset;
    int32_t n_deg_fred;
    bool last_filtered;
    float noise_power_estimation;
    float thres_;
    float pfa;
    gr_complex *zeros_;

public:
    pulse_blanking_cc(float pfa, int32_t length_, int32_t n_segments_est, int32_t n_segments_reset);

    ~pulse_blanking_cc();

    int general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

    void forecast(int noutput_items, gr_vector_int &ninput_items_required);
};

#endif
