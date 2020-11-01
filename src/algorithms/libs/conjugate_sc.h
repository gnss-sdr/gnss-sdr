/*!
 * \file conjugate_sc.h
 * \brief Conjugate a stream of lv_16sc_t ( std::complex<short> )
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

#ifndef GNSS_SDR_CONJUGATE_SC_H
#define GNSS_SDR_CONJUGATE_SC_H

#if GNURADIO_USES_STD_POINTERS
#include <memory>
#else
#include <boost/shared_ptr.hpp>
#endif
#include <gnuradio/sync_block.h>
#include <gnuradio/types.h>  // for gr_vector_const_void_star

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */


class conjugate_sc;

#if GNURADIO_USES_STD_POINTERS
using conjugate_sc_sptr = std::shared_ptr<conjugate_sc>;
#else
using conjugate_sc_sptr = boost::shared_ptr<conjugate_sc>;
#endif

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
