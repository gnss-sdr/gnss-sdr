/*!
 * \file sbas_l1_dll_pll_tracking.h
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for SBAS L1 signals to a TrackingInterface
 * \author Miguel Gómez López, 2026. mgomezl(at)ing.uc3m.es
 * \author Víctor Castillo Agüero, 2026. victorcastilloaguero(at)gmail.com
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
 *
 * SBAS-specific note: the encoded channel symbol period is 2 ms (rate-1/2
 * FEC at 500 sps), so coherent integration is limited to 2 code periods.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_SBAS_L1_DLL_PLL_TRACKING_H
#define GNSS_SDR_SBAS_L1_DLL_PLL_TRACKING_H

#include "dll_pll_tracking_adapter.h"
#include <string>

class ConfigurationInterface;

/** \addtogroup Tracking
 * Classes for GNSS signal tracking.
 * \{ */
/** \addtogroup Tracking_adapters tracking_adapters
 * Wrap GNU Radio blocks for GNSS signal tracking with a TrackingInterface
 * \{ */


/*!
 * \brief This class implements a code DLL + carrier PLL tracking loop
 * block adapter for SBAS L1 signals (PRN 120–138).
 */
class SbasL1DllPllTracking : public DllPllTrackingAdapter
{
public:
    SbasL1DllPllTracking(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~SbasL1DllPllTracking() override = default;
};

/** \} */
/** \} */

#endif  // GNSS_SDR_SBAS_L1_DLL_PLL_TRACKING_H
