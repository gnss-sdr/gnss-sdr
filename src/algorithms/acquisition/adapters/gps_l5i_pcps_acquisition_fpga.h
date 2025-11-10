/*!
 * \file gps_l5i_pcps_acquisition_fpga.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  GPS L5i signals for the FPGA
 * \authors <ul>
 *          <li> Marc Majoral, 2019. mmajoral(at)cttc.es
 *          <li> Javier Arribas, 2019. jarribas(at)cttc.es
 *          </ul>
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GPS_L5I_PCPS_ACQUISITION_FPGA_H
#define GNSS_SDR_GPS_L5I_PCPS_ACQUISITION_FPGA_H

#include "base_pcps_acquisition_fpga.h"

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_adapters
 * \{ */

/*!
 * \brief This class adapts a PCPS acquisition block off-loaded on an FPGA
 * to an AcquisitionInterface for GPS L5i signals
 */
class GpsL5iPcpsAcquisitionFpga : public BasePcpsAcquisitionFpga
{
public:
    /*!
     * \brief Constructor
     */
    GpsL5iPcpsAcquisitionFpga(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    /*!
     * \brief Returns "GPS_L5i_PCPS_Acquisition_FPGA"
     */
    inline std::string implementation() override
    {
        return "GPS_L5i_PCPS_Acquisition_FPGA";
    }

private:
    static const uint32_t DEFAULT_FPGA_BLK_EXP = 13;  // default block exponent
    void generate_gps_l5i_prn_codes();
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L5I_PCPS_ACQUISITION_FPGA_H
