/*!
 * \file cshort_to_float_x2.h
 * \brief Adapts a std::complex<short> stream into two float streams
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

#ifndef GNSS_SDR_CSHORT_TO_FLOAT_X2_H_
#define GNSS_SDR_CSHORT_TO_FLOAT_X2_H_


#include <boost/shared_ptr.hpp>
#include <gnuradio/sync_block.h>

class cshort_to_float_x2;

typedef boost::shared_ptr<cshort_to_float_x2> cshort_to_float_x2_sptr;

cshort_to_float_x2_sptr make_cshort_to_float_x2();

/*!
 * \brief This class adapts a std::complex<short> stream
 * into two 32-bits (float) streams
 */
class cshort_to_float_x2 : public gr::sync_block
{
private:
    friend cshort_to_float_x2_sptr make_cshort_to_float_x2();

public:
    cshort_to_float_x2();

    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);
};

#endif
