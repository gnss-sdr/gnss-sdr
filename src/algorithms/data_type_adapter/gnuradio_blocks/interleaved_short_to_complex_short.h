/*!
 * \file interleaved_short_to_complex_short.h
 * \brief Adapts a short (16-bits) interleaved sample stream into a std::complex<short> stream
 * \author Carles Fernandez Prades, cfernandez(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_INTERLEAVED_SHORT_TO_COMPLEX_SHORT_H_
#define GNSS_SDR_INTERLEAVED_SHORT_TO_COMPLEX_SHORT_H_

#include <boost/shared_ptr.hpp>
#include <gnuradio/sync_decimator.h>

class interleaved_short_to_complex_short;

typedef boost::shared_ptr<interleaved_short_to_complex_short> interleaved_short_to_complex_short_sptr;

interleaved_short_to_complex_short_sptr make_interleaved_short_to_complex_short();

/*!
 * \brief This class adapts a short (16-bits) interleaved sample stream
 * into a std::complex<short> stream
 */
class interleaved_short_to_complex_short : public gr::sync_decimator
{
private:
    friend interleaved_short_to_complex_short_sptr make_interleaved_short_to_complex_short();

public:
    interleaved_short_to_complex_short();

    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);
};

#endif
