/*!
 * \file cshort_to_float_x2.h
 * \brief Adapts a std::complex<short> stream into two float streams
 * \author Carles Fernandez Prades, cfernandez(at)cttc.es
 *
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

#ifndef GNSS_SDR_CSHORT_TO_FLOAT_X2_H
#define GNSS_SDR_CSHORT_TO_FLOAT_X2_H

#include "gnss_block_interface.h"
#include <gnuradio/sync_block.h>
#include <gnuradio/types.h>  // for gr_vector_const_void_star

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */


class cshort_to_float_x2;

using cshort_to_float_x2_sptr = gnss_shared_ptr<cshort_to_float_x2>;

cshort_to_float_x2_sptr make_cshort_to_float_x2();

/*!
 * \brief This class adapts a std::complex<short> stream
 * into two 32-bits (float) streams
 */
class cshort_to_float_x2 : public gr::sync_block
{
public:
    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend cshort_to_float_x2_sptr make_cshort_to_float_x2();
    cshort_to_float_x2();
};


/** \} */
/** \} */
#endif  // GNSS_SDR_CSHORT_TO_FLOAT_X2_H
