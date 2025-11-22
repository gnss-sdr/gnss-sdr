/*!
 * \file beidou_b3i_dll_pll_tracking.h
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for Beidou B3I to a TrackingInterface
 * \author Damian Miralles, 2019. dmiralles2009(at)gmail.com
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
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_BEIDOU_B3I_DLL_PLL_TRACKING_H
#define GNSS_SDR_BEIDOU_B3I_DLL_PLL_TRACKING_H

#include "base_dll_pll_tracking.h"

/** \addtogroup Tracking
 * Classes for GNSS signal tracking.
 * \{ */
/** \addtogroup Tracking_adapters tracking_adapters
 * Wrap GNU Radio blocks for GNSS signal tracking with a TrackingInterface
 * \{ */

class ConfigurationInterface;

/*!
 * \brief This class Adapts a DLL+PLL VEML (Very Early Minus Late) tracking
 * loop block to a TrackingInterface for BeiDou B3I signals
 */
class BeidouB3iDllPllTracking : public BaseDllPllTracking
{
public:
    //! Constructor
    BeidouB3iDllPllTracking(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    //! Returns "BEIDOU_B3I_DLL_PLL_Tracking"
    inline std::string implementation() override
    {
        return "BEIDOU_B3I_DLL_PLL_Tracking";
    }

private:
    void configure_tracking_parameters(const ConfigurationInterface* configuration) override;
    void create_tracking_block() override;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_B3I_DLL_PLL_TRACKING_H
