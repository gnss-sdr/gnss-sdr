/*!
 * \file galileo_e1_pcps_tong_ambiguous_acquisition.h
 * \brief Adapts a PCPS Tong acquisition block to an AcquisitionInterface for
 *  Galileo E1 Signals
 * \author Marc Molina, 2013. marc.molina.pena(at)gmail.com
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

#ifndef GNSS_SDR_GALILEO_E1_PCPS_TONG_AMBIGUOUS_ACQUISITION_H
#define GNSS_SDR_GALILEO_E1_PCPS_TONG_AMBIGUOUS_ACQUISITION_H

#include "base_pcps_acquisition_custom.h"

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_adapters
 * \{ */

/*!
 * \brief Adapts a PCPS Tong acquisition block to an AcquisitionInterface
 *  for Galileo E1 Signals
 */
class GalileoE1PcpsTongAmbiguousAcquisition : public BasePcpsAcquisitionCustom
{
public:
    GalileoE1PcpsTongAmbiguousAcquisition(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~GalileoE1PcpsTongAmbiguousAcquisition() = default;

    /*!
     * \brief Returns "Galileo_E1_PCPS_Tong_Ambiguous_Acquisition"
     */
    inline std::string implementation() override
    {
        return "Galileo_E1_PCPS_Tong_Ambiguous_Acquisition";
    }

private:
    void code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq) override;

    const bool cboc_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E1_PCPS_TONG_AMBIGUOUS_ACQUISITION_H
