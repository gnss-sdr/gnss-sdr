/*!
 * \file beidou_b1c_dll_pll_veml_tracking.h
 * \brief Adapts a DLL+PLL VEML tracking loop block to a TrackingInterface
 *   for BeiDou B1C signals
 * \author GNSS-SDR contributors
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

#ifndef GNSS_SDR_BEIDOU_B1C_DLL_PLL_VEML_TRACKING_H
#define GNSS_SDR_BEIDOU_B1C_DLL_PLL_VEML_TRACKING_H

#include "base_dll_pll_tracking.h"

class BeidouB1cDllPllVemlTracking : public BaseDllPllTracking
{
public:
    BeidouB1cDllPllVemlTracking(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    inline std::string implementation() override
    {
        return "BEIDOU_B1C_DLL_PLL_VEML_Tracking";
    }

private:
    void configure_tracking_parameters(const ConfigurationInterface* configuration) override;
    void create_tracking_block() override;
};

#endif  // GNSS_SDR_BEIDOU_B1C_DLL_PLL_VEML_TRACKING_H
