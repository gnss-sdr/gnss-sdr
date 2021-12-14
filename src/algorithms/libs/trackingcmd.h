/*!
 * \file trackingcmd.h
 * \brief Class that stores information to update the GNSS signal tracking estimations
 * \author Javier Arribas, 2021. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_TRACKINGCMD_H_
#define GNSS_SDR_TRACKINGCMD_H_

#include <cstdint>

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */

class TrackingCmd
{
public:
    TrackingCmd();

    bool enable_carrier_nco_cmd = false;
    bool enable_code_nco_cmd = false;
    double code_freq_chips = 0.0;
    double carrier_freq_hz = 0.0;
    double carrier_freq_rate_hz_s = 0.0;
    uint64_t sample_counter = 0UL;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_TRACKINGCMD_H_
