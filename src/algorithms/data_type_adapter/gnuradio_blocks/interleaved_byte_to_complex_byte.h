/*!
 * \file interleaved_byte_to_complex_byte.h
 * \brief Adapts an 8-bits interleaved sample stream into a 16-bits complex stream
 * \author Carles Fernandez Prades, cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_INTERLEAVED_BYTE_TO_COMPLEX_BYTE_H
#define GNSS_SDR_INTERLEAVED_BYTE_TO_COMPLEX_BYTE_H

#include "gnss_block_interface.h"
#include <gnuradio/sync_decimator.h>

/** \addtogroup Data_Type
 * \{ */
/** \addtogroup data_type_gnuradio_blocks data_type_gr_blocks
 * GNU Radio Blocks for data type conversion
 * \{ */


class interleaved_byte_to_complex_byte;

using interleaved_byte_to_complex_byte_sptr = gnss_shared_ptr<interleaved_byte_to_complex_byte>;

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


/** \} */
/** \} */
#endif  // GNSS_SDR_INTERLEAVED_BYTE_TO_COMPLEX_BYTE_H
