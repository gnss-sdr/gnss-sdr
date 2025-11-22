/*!
 * \file gps_l1_ca_dll_pll_tracking_fpga.h
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for GPS L1 C/A to a TrackingInterface for the FPGA
 * \author Marc Majoral, 2019, mmajoral(at)cttc.es
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

#ifndef GNSS_SDR_GPS_L1_CA_DLL_PLL_TRACKING_FPGA_H
#define GNSS_SDR_GPS_L1_CA_DLL_PLL_TRACKING_FPGA_H


#include "base_dll_pll_tracking_fpga.h"

// /** \addtogroup Tracking
//  * \{ */
// /** \addtogroup Tracking_adapters
//  * \{ */

class ConfigurationInterface;

/*!
 * \brief Adapter for a GPS L1 C/A DLL+PLL tracking loop for FPGA
 */
class GpsL1CaDllPllTrackingFpga : public BaseDllPllTrackingFpga
{
public:
    GpsL1CaDllPllTrackingFpga(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~GpsL1CaDllPllTrackingFpga() override;

    /*!
     * \brief Returns "GPS_L1_CA_DLL_PLL_Tracking_FPGA"
     */
    std::string implementation() override
    {
        return "GPS_L1_CA_DLL_PLL_Tracking_FPGA";
    }

private:
    bool find_alternative_device(std::string& device_io_name);
    int32_t* ca_codes_ptr_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L1_CA_DLL_PLL_TRACKING_FPGA_H
