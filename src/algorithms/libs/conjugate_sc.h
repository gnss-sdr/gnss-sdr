/*!
 * \file conjugate_sc.h
 * \brief Conjugate a stream of lv_16sc_t ( std::complex<short> )
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

#ifndef GNSS_SDR_CONJUGATE_SC_H
#define GNSS_SDR_CONJUGATE_SC_H

#include <boost/shared_ptr.hpp>
#include <gnuradio/sync_block.h>
#include <gnuradio/types.h>  // for gr_vector_const_void_star

class conjugate_sc;

using conjugate_sc_sptr = boost::shared_ptr<conjugate_sc>;

conjugate_sc_sptr make_conjugate_sc();

/*!
 * \brief This class adapts a std::complex<short> stream
 * into two 32-bits (float) streams
 */
class conjugate_sc : public gr::sync_block
{
private:
    friend conjugate_sc_sptr make_conjugate_sc();
    conjugate_sc();

public:
    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);
};

#endif
