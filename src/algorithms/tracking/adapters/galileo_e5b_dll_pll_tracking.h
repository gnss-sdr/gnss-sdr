/*!
 * \file galileo_e5b_dll_pll_tracking.h
 * \brief Adapts a code DLL + carrier PLL
 *  tracking block to a TrackingInterface for Galileo E5b signals
 * \author Piyush Gupta, 2020. piyush04111999@gmail.com
 * \based on work from:
 *          <ul>
 *          <li> Javier Arribas, 2011. jarribas@cttc.es
 *          <li> Luis Esteve, 2012. luis@epsilon-formacion.com
 *          <li> Marc Sales, 2014. marcsales92@gmail.com
 *          </ul>
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

#ifndef GNSS_SDR_GALILEO_E5B_DLL_PLL_TRACKING_H
#define GNSS_SDR_GALILEO_E5B_DLL_PLL_TRACKING_H

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
 * loop block to a TrackingInterface for Galileo E5b signals
 */
class GalileoE5bDllPllTracking : public BaseDllPllTracking
{
public:
    //! Constructor
    GalileoE5bDllPllTracking(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    //! Returns "Galileo_E5b_DLL_PLL_Tracking"
    inline std::string implementation() override
    {
        return "Galileo_E5b_DLL_PLL_Tracking";
    }

private:
    void configure_tracking_parameters(const ConfigurationInterface* configuration) override;
    void create_tracking_block() override;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E5B_DLL_PLL_TRACKING_H
