/*!
 * \file qzss_l1_pcps_acquisition.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  QZSS L1 signals
 * \authors <ul>
 *          <li> Carles Fernandez, 2026. cfernandez(at)cttc.es
 *          </ul>
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_QZSS_L1_PCPS_ACQUISITION_H
#define GNSS_SDR_QZSS_L1_PCPS_ACQUISITION_H

#include "base_pcps_acquisition.h"

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_adapters
 * \{ */

/*!
 * \brief This class adapts a PCPS acquisition block to an AcquisitionInterface
 *  for QZSS L1 signals
 */
class QzssL1PcpsAcquisition : public BasePcpsAcquisition
{
public:
    QzssL1PcpsAcquisition(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~QzssL1PcpsAcquisition() = default;

    /*!
     * \brief Returns "QZSS_L1_PCPS_Acquisition"
     */
    inline std::string implementation() override
    {
        return "QZSS_L1_PCPS_Acquisition";
    }

private:
    void code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq) override;
};


/** \} */
/** \} */


#endif  // GNSS_SDR_QZSS_L1_PCPS_ACQUISITION_H