/*!
 * \file galileo_e1_pcps_ambiguous_acquisition_fpga.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E1 Signals for the FPGA
 * \author Marc Majoral, 2019. mmajoral(at)cttc.es
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

#ifndef GNSS_SDR_GALILEO_E1_PCPS_AMBIGUOUS_ACQUISITION_FPGA_H
#define GNSS_SDR_GALILEO_E1_PCPS_AMBIGUOUS_ACQUISITION_FPGA_H

#include "base_pcps_acquisition_fpga.h"

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_adapters
 * \{ */


/*!
 * \brief This class adapts a PCPS acquisition block off-loaded on an FPGA
 * to an AcquisitionInterface for Galileo E1 Signals
 */
class GalileoE1PcpsAmbiguousAcquisitionFpga : public BasePcpsAcquisitionFpga
{
public:
    /*!
     * \brief Constructor
     */
    GalileoE1PcpsAmbiguousAcquisitionFpga(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    /*!
     * \brief Returns "Galileo_E1_PCPS_Ambiguous_Acquisition_FPGA"
     */
    inline std::string implementation() override
    {
        return "Galileo_E1_PCPS_Ambiguous_Acquisition_FPGA";
    }

private:
    static const uint32_t DEFAULT_FPGA_BLK_EXP = 13;  // default block exponent
    void generate_galileo_e1_prn_codes();
    const bool acquire_pilot_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E1_PCPS_AMBIGUOUS_ACQUISITION_FPGA_H
