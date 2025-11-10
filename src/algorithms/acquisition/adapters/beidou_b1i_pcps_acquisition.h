/*!
 * \file beidou_b1i_pcps_acquisition.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Beidou B1I signals
 * \authors <ul>
 *          <li> Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
 *          </ul>
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

#ifndef GNSS_SDR_BEIDOU_B1I_PCPS_ACQUISITION_H
#define GNSS_SDR_BEIDOU_B1I_PCPS_ACQUISITION_H

#include "base_pcps_acquisition.h"

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_adapters
 * \{ */

/*!
 * \brief This class adapts a PCPS acquisition block to an AcquisitionInterface
 *  for GPS L1 C/A signals
 */
class BeidouB1iPcpsAcquisition : public BasePcpsAcquisition
{
public:
    BeidouB1iPcpsAcquisition(const ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_streams,
        unsigned int out_streams);

    ~BeidouB1iPcpsAcquisition() = default;

    /*!
     * \brief Returns "BEIDOU_B1I_PCPS_Acquisition"
     */
    inline std::string implementation() override
    {
        return "BEIDOU_B1I_PCPS_Acquisition";
    }

private:
    void code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq) override;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_B1I_PCPS_ACQUISITION_H
