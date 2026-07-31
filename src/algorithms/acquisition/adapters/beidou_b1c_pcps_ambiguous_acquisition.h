/*!
 * \file beidou_b1c_pcps_ambiguous_acquisition.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  BeiDou B1C Signals
 * \author GNSS-SDR contributors
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

#ifndef GNSS_SDR_BEIDOU_B1C_PCPS_AMBIGUOUS_ACQUISITION_H
#define GNSS_SDR_BEIDOU_B1C_PCPS_AMBIGUOUS_ACQUISITION_H

#include "base_pcps_acquisition.h"

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_adapters
 * \{ */

class BeidouB1cPcpsAmbiguousAcquisition : public BasePcpsAcquisition
{
public:
    BeidouB1cPcpsAmbiguousAcquisition(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~BeidouB1cPcpsAmbiguousAcquisition() override = default;

    inline std::string implementation() override
    {
        return "BEIDOU_B1C_PCPS_Ambiguous_Acquisition";
    }

    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override;

private:
    void code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq) override;

    const bool acquire_pilot_;
    const bool qmboc_;
    Gnss_Synchro* gnss_synchro_;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_B1C_PCPS_AMBIGUOUS_ACQUISITION_H
