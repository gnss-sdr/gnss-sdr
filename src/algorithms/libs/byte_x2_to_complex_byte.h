/*!
 * \file byte_x2_to_complex_byte.h
 * \brief Adapts two signed char streams into a std::complex<signed char> stream
 * \author Carles Fernandez Prades, cfernandez(at)cttc.es
 *
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

#ifndef GNSS_SDR_BYTE_X2_TO_COMPLEX_BYTE_H_
#define GNSS_SDR_BYTE_X2_TO_COMPLEX_BYTE_H_


#include <boost/shared_ptr.hpp>
#include <gnuradio/sync_block.h>

class byte_x2_to_complex_byte;

typedef boost::shared_ptr<byte_x2_to_complex_byte> byte_x2_to_complex_byte_sptr;

byte_x2_to_complex_byte_sptr make_byte_x2_to_complex_byte();

/*!
 * \brief This class adapts two signed char streams
 * into a std::complex<signed char> stream
 */
class byte_x2_to_complex_byte : public gr::sync_block
{
private:
    friend byte_x2_to_complex_byte_sptr make_byte_x2_to_complex_byte();

public:
    byte_x2_to_complex_byte();

    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);
};

#endif
