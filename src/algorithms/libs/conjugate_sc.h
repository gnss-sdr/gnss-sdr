/*!
 * \file conjugate_sc.h
 * \brief Conjugate a stream of lv_16sc_t ( std::complex<short> )
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

#ifndef GNSS_SDR_CONJUGATE_SC_H
#define GNSS_SDR_CONJUGATE_SC_H

#include "gnss_block_interface.h"
#include <gnuradio/sync_block.h>
#include <gnuradio/types.h>  // for gr_vector_const_void_star

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */


class conjugate_sc;

using conjugate_sc_sptr = gnss_shared_ptr<conjugate_sc>;

conjugate_sc_sptr make_conjugate_sc();

/*!
 * \brief This class adapts a std::complex<short> stream
 * into two 32-bits (float) streams
 */
class conjugate_sc : public gr::sync_block
{
public:
    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend conjugate_sc_sptr make_conjugate_sc();
    conjugate_sc();
};


/** \} */
/** \} */
#endif  // GNSS_SDR_CONJUGATE_SC_H
