/*!
 * \file complex_byte_to_float_x2.h
 * \brief Adapts a std::complex<signed char> stream into two 16-bits (short) streams
 * \author Carles Fernandez Prades, cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_COMPLEX_BYTE_TO_FLOAT_X2_H
#define GNSS_SDR_COMPLEX_BYTE_TO_FLOAT_X2_H


#if GNURADIO_USES_STD_POINTERS
#include <memory>
#else
#include <boost/shared_ptr.hpp>
#endif
#include <gnuradio/sync_block.h>
#include <gnuradio/types.h>  // for gr_vector_const_void_star

class complex_byte_to_float_x2;

#if GNURADIO_USES_STD_POINTERS
using complex_byte_to_float_x2_sptr = std::shared_ptr<complex_byte_to_float_x2>;
#else
using complex_byte_to_float_x2_sptr = boost::shared_ptr<complex_byte_to_float_x2>;
#endif

complex_byte_to_float_x2_sptr make_complex_byte_to_float_x2();

/*!
 * \brief This class adapts a std::complex<signed char> stream
 * into two 16-bits (short) streams
 */
class complex_byte_to_float_x2 : public gr::sync_block
{
public:
    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend complex_byte_to_float_x2_sptr make_complex_byte_to_float_x2();
    complex_byte_to_float_x2();
};

#endif
