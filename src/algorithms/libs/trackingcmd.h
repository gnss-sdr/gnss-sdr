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
    bool enable_pll_vtl_feedack = false;
    bool enable_dll_vtl_feedack = false;
    bool enable_smooth_pr = false;
    double pll_vtl_freq_hz = 0.0;
    double dll_vtl_freq_hz = 0.0;
    double PVT_sample_counter = 0.0;
    double carrier_freq_rate_hz_s = 0.0;
    uint32_t channel_id = 0;
    uint32_t prn_id = 0;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_TRACKINGCMD_H_
