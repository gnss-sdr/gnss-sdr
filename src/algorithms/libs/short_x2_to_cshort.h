/*!
 * \file short_x2_to_cshort.h
 * \brief Adapts two short streams into a std::complex<short> stream
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

#ifndef GNSS_SDR_SHORT_X2_TO_CSHORT_H
#define GNSS_SDR_SHORT_X2_TO_CSHORT_H


#include "gnss_block_interface.h"
#include <gnuradio/sync_block.h>
#include <gnuradio/types.h>  // for gr_vector_const_void_star

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */


class short_x2_to_cshort;

using short_x2_to_cshort_sptr = gnss_shared_ptr<short_x2_to_cshort>;

short_x2_to_cshort_sptr make_short_x2_to_cshort();

/*!
 * \brief This class adapts two short streams into a std::complex<short> stream
 */
class short_x2_to_cshort : public gr::sync_block
{
public:
    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend short_x2_to_cshort_sptr make_short_x2_to_cshort();
    short_x2_to_cshort();
};


/** \} */
/** \} */
#endif  // GNSS_SDR_SHORT_X2_TO_CSHORT_H
