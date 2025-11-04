/*!
 * \file galileo_e5a_dll_pll_tracking_fpga.h
 * \brief Adapts a code DLL + carrier PLL
 *  tracking block to a TrackingInterface for Galileo E5a signals for the FPGA
 * \author Marc Majoral, 2019. mmajoral(at)cttc.cat
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

#ifndef GNSS_SDR_GALILEO_E5A_DLL_PLL_TRACKING_FPGA_H
#define GNSS_SDR_GALILEO_E5A_DLL_PLL_TRACKING_FPGA_H

#include "base_dll_pll_tracking_fpga.h"

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_adapters
 * \{ */

/*!
 * \brief Adapter for a Galileo E5a DLL+PLL tracking loop implemented in FPGA
 */
class GalileoE5aDllPllTrackingFpga : public BaseDllPllTrackingFpga
{
public:
    /*!
     * \brief Constructor
     */
    GalileoE5aDllPllTrackingFpga(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    /*!
     * \brief Destructor
     */
    ~GalileoE5aDllPllTrackingFpga() override;

    /*!
     * \brief Returns "Galileo_E5a_DLL_PLL_Tracking_FPGA"
     */
    std::string implementation() override
    {
        return "Galileo_E5a_DLL_PLL_Tracking_FPGA";
    }

private:
    // Pointer to local PRN codes (pilot/data)
    int32_t* prn_codes_ptr_;
    int32_t* data_codes_ptr_;

    // Pilot tracking flag
    bool track_pilot_;
};


// /** \} */
// /** \} */
#endif  // GNSS_SDR_GALILEO_E5A_DLL_PLL_TRACKING_FPGA_H
