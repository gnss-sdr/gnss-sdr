/*!
 * \file conjugate_ic.h
 * \brief Conjugate a stream of lv_8sc_t ( std::complex<char> )
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

#ifndef GNSS_SDR_CONJUGATE_IC_H
#define GNSS_SDR_CONJUGATE_IC_H

#include <gnuradio/sync_block.h>
#include <gnuradio/types.h>  // for gr_vector_const_void_star
#if GNURADIO_USES_STD_POINTERS
#include <memory>
#else
#include <boost/shared_ptr.hpp>
#endif

class conjugate_ic;

#if GNURADIO_USES_STD_POINTERS
using conjugate_ic_sptr = std::shared_ptr<conjugate_ic>;
#else
using conjugate_ic_sptr = boost::shared_ptr<conjugate_ic>;
#endif

conjugate_ic_sptr make_conjugate_ic();

/*!
 * \brief This class adapts a std::complex<short> stream
 * into two 32-bits (float) streams
 */
class conjugate_ic : public gr::sync_block
{
public:
    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend conjugate_ic_sptr make_conjugate_ic();
    conjugate_ic();
};

#endif
