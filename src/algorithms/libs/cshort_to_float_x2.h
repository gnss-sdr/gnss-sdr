/*!
 * \file cshort_to_float_x2.h
 * \brief Adapts a std::complex<short> stream into two float streams
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

#ifndef GNSS_SDR_CSHORT_TO_FLOAT_X2_H_
#define GNSS_SDR_CSHORT_TO_FLOAT_X2_H_


#include <boost/shared_ptr.hpp>
#include <gnuradio/sync_block.h>
#include <gnuradio/types.h>  // for gr_vector_const_void_star

class cshort_to_float_x2;

using cshort_to_float_x2_sptr = boost::shared_ptr<cshort_to_float_x2>;

cshort_to_float_x2_sptr make_cshort_to_float_x2();

/*!
 * \brief This class adapts a std::complex<short> stream
 * into two 32-bits (float) streams
 */
class cshort_to_float_x2 : public gr::sync_block
{
private:
    friend cshort_to_float_x2_sptr make_cshort_to_float_x2();
    cshort_to_float_x2();

public:
    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);
};

#endif
