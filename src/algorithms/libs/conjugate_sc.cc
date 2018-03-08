/*!
 * \file conjugate_sc.h
 * \brief Conjugate a stream of lv_16sc_t ( std::complex<short> )
 * \author Carles Fernandez Prades, cfernandez(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "conjugate_sc.h"
#include <gnuradio/io_signature.h>
#include <volk_gnsssdr/volk_gnsssdr.h>


conjugate_sc_sptr make_conjugate_sc()
{
    return conjugate_sc_sptr(new conjugate_sc());
}


conjugate_sc::conjugate_sc() : gr::sync_block("conjugate_sc",
                                   gr::io_signature::make(1, 1, sizeof(lv_16sc_t)),
                                   gr::io_signature::make(1, 1, sizeof(lv_16sc_t)))
{
    const int alignment_multiple = volk_gnsssdr_get_alignment() / sizeof(lv_16sc_t);
    set_alignment(std::max(1, alignment_multiple));
}


int conjugate_sc::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    const lv_16sc_t *in = reinterpret_cast<const lv_16sc_t *>(input_items[0]);
    lv_16sc_t *out = reinterpret_cast<lv_16sc_t *>(output_items[0]);
    volk_gnsssdr_16ic_conjugate_16ic(out, in, noutput_items);
    return noutput_items;
}
