/*!
 * \file beamformer.h
 *
 * \brief Simple spatial filter using RAW array input and beamforming coefficients
 * \author Javier Arribas jarribas (at) cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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
#include <vector>

class beamformer;

using beamformer_sptr = boost::shared_ptr<beamformer>;

beamformer_sptr make_beamformer_sptr();

const int GNSS_SDR_BEAMFORMER_CHANNELS = 8;

/*!
 * \brief This class implements a real-time software-defined spatial filter using the CTTC GNSS experimental antenna array input and a set of dynamically reloadable weights
 */
class beamformer : public gr::sync_block
{
public:
    ~beamformer() = default;
    int work(int noutput_items, gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend beamformer_sptr make_beamformer_sptr();
    beamformer();
    std::vector<gr_complex> weight_vector = std::vector<gr_complex>(GNSS_SDR_BEAMFORMER_CHANNELS, gr_complex(1.0, 0.0));
};

#endif
