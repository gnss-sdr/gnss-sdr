/*!
 * \file galileo_e6_pcps_acquisition.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E6 B/C Signals
 * \author Carles Fernandez-Prades, 2020. cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_GALILEO_E6_PCPS_ACQUISITION_H
#define GNSS_SDR_GALILEO_E6_PCPS_ACQUISITION_H

#include "base_pcps_acquisition.h"

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_adapters
 * \{ */


/*!
 * \brief This class adapts a PCPS acquisition block to an
 *  AcquisitionInterface for Galileo E6 Signals
 */
class GalileoE6PcpsAcquisition : public BasePcpsAcquisition
{
public:
    GalileoE6PcpsAcquisition(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~GalileoE6PcpsAcquisition() = default;

    /*!
     * \brief Returns "Galileo_E6_PCPS_Acquisition"
     */
    inline std::string implementation() override
    {
        return "Galileo_E6_PCPS_Acquisition";
    }

private:
    void code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq) override;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E6_PCPS_ACQUISITION_H
