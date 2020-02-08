/*!
 * \file byte_x2_to_complex_byte.h
 * \brief Adapts two signed char streams into a std::complex<signed char> stream
 * \author Carles Fernandez Prades, cfernandez(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_BYTE_X2_TO_COMPLEX_BYTE_H_
#define GNSS_SDR_BYTE_X2_TO_COMPLEX_BYTE_H_


#include <boost/shared_ptr.hpp>
#include <gnuradio/sync_block.h>
#include <gnuradio/types.h>  // for gr_vector_const_void_star

class byte_x2_to_complex_byte;

using byte_x2_to_complex_byte_sptr = boost::shared_ptr<byte_x2_to_complex_byte>;

byte_x2_to_complex_byte_sptr make_byte_x2_to_complex_byte();

/*!
 * \brief This class adapts two signed char streams
 * into a std::complex<signed char> stream
 */
class byte_x2_to_complex_byte : public gr::sync_block
{
private:
    friend byte_x2_to_complex_byte_sptr make_byte_x2_to_complex_byte();
    byte_x2_to_complex_byte();

public:
    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);
};

#endif
