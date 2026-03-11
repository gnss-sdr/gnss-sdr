/*!
 * \file qzss_l5_dll_pll_tracking.h
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for QZSS L5 signals to a TrackingInterface
 * \author Carles Fernandez, 2026. cfernandez(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
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

#ifndef GNSS_SDR_QZSS_L5_DLL_PLL_TRACKING_H
#define GNSS_SDR_QZSS_L5_DLL_PLL_TRACKING_H

#include "base_dll_pll_tracking.h"

/** \addtogroup Tracking
 * Classes for GNSS signal tracking.
 * \{ */
/** \addtogroup Tracking_adapters tracking_adapters
 * Wrap GNU Radio blocks for GNSS signal tracking with a TrackingInterface
 * \{ */


/*!
 * \brief This class implements a code DLL + carrier PLL tracking loop
 * block adapter for QZSS L5 signals
 */
class QzssL5DllPllTracking : public BaseDllPllTracking
{
public:
    //! Constructor
    QzssL5DllPllTracking(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    //! Returns "QZSS_L5_DLL_PLL_Tracking"
    inline std::string implementation() override
    {
        return "QZSS_L5_DLL_PLL_Tracking";
    }

private:
    void configure_tracking_parameters(const ConfigurationInterface* configuration) override;
    void create_tracking_block() override;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_QZSS_L5_DLL_PLL_TRACKING_H
