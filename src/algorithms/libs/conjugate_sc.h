/*!
 * \file conjugate_sc.h
 * \brief Conjugate a stream of lv_16sc_t ( std::complex<short> )
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

#ifndef GNSS_SDR_CONJUGATE_SC_H_
#define GNSS_SDR_CONJUGATE_SC_H_

#include <boost/shared_ptr.hpp>
#include <gnuradio/sync_block.h>

class conjugate_sc;

typedef boost::shared_ptr<conjugate_sc> conjugate_sc_sptr;

conjugate_sc_sptr make_conjugate_sc();

/*!
 * \brief This class adapts a std::complex<short> stream
 * into two 32-bits (float) streams
 */
class conjugate_sc : public gr::sync_block
{
private:
    friend conjugate_sc_sptr make_conjugate_sc();

public:
    conjugate_sc();

    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);
};

#endif
