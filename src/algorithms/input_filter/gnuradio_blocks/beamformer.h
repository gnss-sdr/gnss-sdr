/*!
 * \file beamformer.h
 *
 * \brief Simple spatial filter using RAW array input and beamforming coefficients
 * \author Javier Arribas jarribas (at) cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_BEAMFORMER_H
#define GNSS_SDR_BEAMFORMER_H

#include <gnuradio/sync_block.h>

class beamformer;
typedef boost::shared_ptr<beamformer> beamformer_sptr;

beamformer_sptr make_beamformer();

/*!
 * \brief This class implements a real-time software-defined spatial filter using the CTTC GNSS experimental antenna array input and a set of dynamically reloadable weights
 */
class beamformer : public gr::sync_block
{
private:
    friend beamformer_sptr
    make_beamformer_sptr();

    gr_complex *weight_vector;

public:
    beamformer();
    ~beamformer();
    int work(int noutput_items, gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);
};

#endif
