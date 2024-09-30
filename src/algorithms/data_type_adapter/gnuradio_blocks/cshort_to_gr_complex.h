/*!
 * \file cshort_to_gr_complex.h
 * \brief Adapts a complex short (16 + 16 bits) sample stream into a
 *   std::complex<float> stream (32 + 32 bits)
 * \author Carles Fernandez Prades, 2014 cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_CSHORT_TO_GR_COMPLEX_H
#define GNSS_SDR_CSHORT_TO_GR_COMPLEX_H

#include "gnss_block_interface.h"
#include <gnuradio/sync_block.h>

/** \addtogroup Data_Type
 * \{ */
/** \addtogroup data_type_gnuradio_blocks
 * \{ */


class cshort_to_gr_complex;

using cshort_to_gr_complex_sptr = gnss_shared_ptr<cshort_to_gr_complex>;

cshort_to_gr_complex_sptr make_cshort_to_gr_complex();

/*!
 * \brief This class adapts a short (16-bits) interleaved sample stream
 * into a std::complex<float> stream
 */
class cshort_to_gr_complex : public gr::sync_block
{
public:
    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend cshort_to_gr_complex_sptr make_cshort_to_gr_complex();
    cshort_to_gr_complex();
};


/** \} */
/** \} */
#endif  // GNSS_SDR_CSHORT_TO_GR_COMPLEX_H