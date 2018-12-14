/*!
 * \file short_x2_to_cshort.h
 * \brief Adapts two short streams into a std::complex<short> stream
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

#ifndef GNSS_SDR_SHORT_X2_TO_CSHORT_H_
#define GNSS_SDR_SHORT_X2_TO_CSHORT_H_


#include <boost/shared_ptr.hpp>
#include <gnuradio/sync_block.h>

class short_x2_to_cshort;

typedef boost::shared_ptr<short_x2_to_cshort> short_x2_to_cshort_sptr;

short_x2_to_cshort_sptr make_short_x2_to_cshort();

/*!
 * \brief This class adapts two short streams into a std::complex<short> stream
 */
class short_x2_to_cshort : public gr::sync_block
{
private:
    friend short_x2_to_cshort_sptr make_short_x2_to_cshort();

public:
    short_x2_to_cshort();

    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);
};

#endif
