/*!
 * \file gps_l2_m_dll_pll_tracking_fpga.h
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for GPS L2C to a TrackingInterface for the FPGA
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

#ifndef GNSS_SDR_GPS_L2_M_DLL_PLL_TRACKING_FPGA_H
#define GNSS_SDR_GPS_L2_M_DLL_PLL_TRACKING_FPGA_H

#include "base_dll_pll_tracking_fpga.h"

//** \addtogroup Tracking
//  * \{ */
// /** \addtogroup Tracking_adapters
//  * \{ */

/*!
 * \brief Adapter for a GPS L2M DLL+PLL tracking loop implemented in FPGA
 */
class GpsL2MDllPllTrackingFpga : public BaseDllPllTrackingFpga
{
public:
    /*!
     * \brief Constructor
     */
    GpsL2MDllPllTrackingFpga(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    /*!
     * \brief Destructor
     */
    ~GpsL2MDllPllTrackingFpga() override;

    /*!
     * \brief Returns "GPS_L2_M_DLL_PLL_Tracking_FPGA"
     */
    std::string implementation() override
    {
        return "GPS_L2_M_DLL_PLL_Tracking_FPGA";
    }

private:
    bool find_alternative_device(std::string& device_io_name);
    int32_t* prn_codes_ptr_;  // Pointer to local PRN codes
};

// /** \} */
// /** \} */
#endif  // GNSS_SDR_GPS_L2_M_DLL_PLL_TRACKING_FPGA_H
