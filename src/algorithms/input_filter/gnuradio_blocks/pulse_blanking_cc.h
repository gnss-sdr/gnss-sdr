/*!
 * \file pulse_blanking_cc.h
 * \brief Implements a simple pulse blanking algorithm
 * \author Javier Arribas (jarribas(at)cttc.es)
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017 (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_PULSE_BLANKING_H_
#define GNSS_SDR_PULSE_BLANKING_H_

#include <boost/shared_ptr.hpp>
#include <gnuradio/block.h>

class pulse_blanking_cc;

typedef boost::shared_ptr<pulse_blanking_cc> pulse_blanking_cc_sptr;

pulse_blanking_cc_sptr make_pulse_blanking_cc(double Pfa);

/*!
 * \brief This class adapts a short (16-bits) interleaved sample stream
 * into a std::complex<short> stream
 */
class pulse_blanking_cc : public gr::block
{
private:
    friend pulse_blanking_cc_sptr pulse_blanking_cc(double Pfa);
    double d_Pfa;
public:
    pulse_blanking_cc(double Pfa);

    int general_work (int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
            gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);
};

#endif
