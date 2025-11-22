/*!
 * \file glonass_l2_ca_pcps_acquisition.h
 * \brief  Adapts a PCPS acquisition block to an AcquisitionInterface for
 * Glonass L2 C/A signals
 * \author Damian Miralles, 2018, dmiralles2009@gmail.com
 *
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

#ifndef GNSS_SDR_GLONASS_L2_CA_PCPS_ACQUISITION_H
#define GNSS_SDR_GLONASS_L2_CA_PCPS_ACQUISITION_H

#include "base_pcps_acquisition.h"

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_adapters
 * \{ */

/*!
 * \brief This class adapts a PCPS acquisition block to an AcquisitionInterface
 *  for GLONASS L2 C/A signals
 */
class GlonassL2CaPcpsAcquisition : public BasePcpsAcquisition
{
public:
    GlonassL2CaPcpsAcquisition(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~GlonassL2CaPcpsAcquisition() = default;

    /*!
     * \brief Returns "GLONASS_L2_CA_PCPS_Acquisition"
     */
    inline std::string implementation() override
    {
        return "GLONASS_L2_CA_PCPS_Acquisition";
    }

private:
    void code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq) override;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GLONASS_L2_CA_PCPS_ACQUISITION_H
