/*!
 * \file glonass_l1_ca_pcps_acquisition.h
 * \brief  Adapts a PCPS acquisition block to an AcquisitionInterface for
 * Glonass L1 C/A signals
 * \author Gabriel Araujo, 2017. gabriel.araujo.5000(at)gmail.com
 * \author Luis Esteve, 2017. luis(at)epsilon-formacion.com
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

#ifndef GNSS_SDR_GLONASS_L1_CA_PCPS_ACQUISITION_H
#define GNSS_SDR_GLONASS_L1_CA_PCPS_ACQUISITION_H

#include "base_pcps_acquisition.h"

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class adapts a PCPS acquisition block to an AcquisitionInterface
 *  for GPS L1 C/A signals
 */
class GlonassL1CaPcpsAcquisition : public BasePcpsAcquisition
{
public:
    GlonassL1CaPcpsAcquisition(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~GlonassL1CaPcpsAcquisition() = default;

    /*!
     * \brief Returns "GLONASS_L1_CA_PCPS_Acquisition"
     */
    inline std::string implementation() override
    {
        return "GLONASS_L1_CA_PCPS_Acquisition";
    }

private:
    void code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq) override;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GLONASS_L1_CA_PCPS_ACQUISITION_H
