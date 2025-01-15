/*!
 * \file beamformer.h
 *
 * \brief Simple spatial filter using RAW array input and beamforming coefficients
 * \author Javier Arribas jarribas (at) cttc.es
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

#ifndef GNSS_SDR_BEAMFORMER_H
#define GNSS_SDR_BEAMFORMER_H

#include "gnss_block_interface.h"
#include <gnuradio/sync_block.h>
#include <vector>

/** \addtogroup Input_Filter
 * \{ */
/** \addtogroup Input_filter_gnuradio_blocks
 * \{ */


class beamformer;

using beamformer_sptr = gnss_shared_ptr<beamformer>;

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


/** \} */
/** \} */
#endif  // GNSS_SDR_BEAMFORMER_H
