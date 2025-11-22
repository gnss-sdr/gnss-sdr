/*!
 * \file galileo_e1_dll_pll_veml_tracking.h
 * \brief  Adapts a DLL+PLL VEML (Very Early Minus Late) tracking loop block
 *   to a TrackingInterface for Galileo E1 signals
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
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

#ifndef GNSS_SDR_GALILEO_E1_DLL_PLL_VEML_TRACKING_H
#define GNSS_SDR_GALILEO_E1_DLL_PLL_VEML_TRACKING_H

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
 * loop block to a TrackingInterface for Galileo E1 signals
 */
class GalileoE1DllPllVemlTracking : public BaseDllPllTracking
{
public:
    //! Constructor
    GalileoE1DllPllVemlTracking(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    //! Returns "Galileo_E1_DLL_PLL_VEML_Tracking"
    inline std::string implementation() override
    {
        return "Galileo_E1_DLL_PLL_VEML_Tracking";
    }

private:
    void configure_tracking_parameters(const ConfigurationInterface* configuration) override;
    void create_tracking_block() override;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E1_DLL_PLL_VEML_TRACKING_H
