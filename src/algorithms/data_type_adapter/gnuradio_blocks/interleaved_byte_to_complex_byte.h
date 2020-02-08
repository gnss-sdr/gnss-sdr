/*!
 * \file interleaved_byte_to_complex_byte.h
 * \brief Adapts an 8-bits interleaved sample stream into a 16-bits complex stream
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

#ifndef GNSS_SDR_INTERLEAVED_BYTE_TO_COMPLEX_BYTE_H_
#define GNSS_SDR_INTERLEAVED_BYTE_TO_COMPLEX_BYTE_H_


#include <boost/shared_ptr.hpp>
#include <gnuradio/sync_decimator.h>

class interleaved_byte_to_complex_byte;

using interleaved_byte_to_complex_byte_sptr = boost::shared_ptr<interleaved_byte_to_complex_byte>;

interleaved_byte_to_complex_byte_sptr make_interleaved_byte_to_complex_byte();

/*!
 * \brief This class adapts an 8-bits interleaved sample stream
 * into a 16-bits complex stream (std::complex<unsigned char>)
 */
class interleaved_byte_to_complex_byte : public gr::sync_decimator
{
public:
    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend interleaved_byte_to_complex_byte_sptr make_interleaved_byte_to_complex_byte();
    interleaved_byte_to_complex_byte();
};

#endif  // GNSS_SDR_INTERLEAVED_BYTE_TO_COMPLEX_BYTE_H_
