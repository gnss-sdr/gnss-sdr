/*!
 * \file gps_l5_dll_pll_tracking_fpga.h
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for GPS L5 to a TrackingInterface for the FPGA
 * \author Marc Majoral, 2019. mmajoral(at)cttc.cat
 *         Javier Arribas, 2019. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_GPS_L5_DLL_PLL_TRACKING_FPGA_H
#define GNSS_SDR_GPS_L5_DLL_PLL_TRACKING_FPGA_H

#include "base_dll_pll_tracking_fpga.h"

// /** \addtogroup Tracking
//  * \{ */
// /** \addtogroup Tracking_adapters
//  * \{ */

/*!
 * \brief Adapter for a GPS L5 DLL+PLL tracking loop for FPGA devices
 */
class GpsL5DllPllTrackingFpga : public BaseDllPllTrackingFpga
{
public:
    /*!
     * \brief Constructor
     */
    GpsL5DllPllTrackingFpga(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    /*!
     * \brief Destructor
     */
    ~GpsL5DllPllTrackingFpga() override;

    /*!
     * \brief Returns "GPS_L5_DLL_PLL_Tracking_FPGA"
     */
    std::string implementation() override
    {
        return "GPS_L5_DLL_PLL_Tracking_FPGA";
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
#endif  // GNSS_SDR_GPS_L5_DLL_PLL_TRACKING_FPGA_H