/*!
 * \file galileo_e1_dll_pll_veml_tracking_fpga.h
 * \brief Adapts a DLL+PLL VEML (Very Early Minus Late) tracking loop block
 *  to a TrackingInterface for Galileo E1 signals for the FPGA
 * \author Marc Majoral, 2019. mmajoral(at)cttc.cat
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

#ifndef GNSS_SDR_GALILEO_E1_DLL_PLL_VEML_TRACKING_FPGA_H
#define GNSS_SDR_GALILEO_E1_DLL_PLL_VEML_TRACKING_FPGA_H

#include "base_dll_pll_tracking_fpga.h"

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_adapters
 * \{ */

/*!
 * \brief Adapter for a Galileo E1 DLL+PLL VEML tracking loop block in FPGA
 */
class GalileoE1DllPllVemlTrackingFpga : public BaseDllPllTrackingFpga
{
public:
    GalileoE1DllPllVemlTrackingFpga(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~GalileoE1DllPllVemlTrackingFpga() override;

    std::string implementation() override
    {
        return "Galileo_E1_DLL_PLL_VEML_Tracking_FPGA";
    }

private:
    bool find_alternative_device(std::string& device_io_name);
    int32_t* prn_codes_ptr_;
    int32_t* data_codes_ptr_;
    bool track_pilot_;
};

// /** \} */
// /** \} */
#endif  // GNSS_SDR_GALILEO_E1_DLL_PLL_VEML_TRACKING_FPGA_H
