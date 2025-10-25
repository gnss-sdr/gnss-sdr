/*!
 * \file galileo_e5b_pcps_acquisition_fpga.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E5b data and pilot Signals for the FPGA
 * \author Piyush Gupta, 2020. piyush04111999@gmail.com
 * \note Code added as part of GSoC 2020 Program.
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

#ifndef GNSS_SDR_GALILEO_E5B_PCPS_ACQUISITION_FPGA_H
#define GNSS_SDR_GALILEO_E5B_PCPS_ACQUISITION_FPGA_H

#include "base_pcps_acquisition_fpga.h"

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class adapts a PCPS acquisition block off-loaded on an FPGA
 * to an AcquisitionInterface for Galileo E5b signals
 */
class GalileoE5bPcpsAcquisitionFpga : public BasePcpsAcquisitionFpga
{
public:
    /*!
     * \brief Constructor
     */
    GalileoE5bPcpsAcquisitionFpga(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    /*!
     * \brief Returns "Galileo_E5b_Pcps_Acquisition_FPGA"
     */
    inline std::string implementation() override
    {
        return "Galileo_E5b_PCPS_Acquisition_FPGA";
    }

private:
    static const uint32_t DEFAULT_FPGA_BLK_EXP = 13;  // default block exponent
    void generate_galileo_e5b_prn_codes();
    bool acq_pilot_;
    bool acq_iq_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E5B_PCPS_ACQUISITION_FPGA_H
