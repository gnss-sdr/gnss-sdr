/*!
 * \file galileo_e5b_pcps_acquisition.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E5b data and pilot Signals
 * \author Piyush Gupta, 2020. piyush04111999@gmail.com
 * \note Code added as part of GSoC 2020 program.
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

#ifndef GNSS_SDR_GALILEO_E5B_PCPS_ACQUISITION_H
#define GNSS_SDR_GALILEO_E5B_PCPS_ACQUISITION_H


#include "base_pcps_acquisition.h"

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_adapters
 * \{ */

class GalileoE5bPcpsAcquisition : public BasePcpsAcquisition
{
public:
    /*!
     * \brief Constructor
     */
    GalileoE5bPcpsAcquisition(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    /*!
     * \brief Destructor
     */
    ~GalileoE5bPcpsAcquisition() = default;

    /*!
     * \brief Returns "GALILEO_E5b_PCPS_Acquisition"
     */

    inline std::string implementation() override
    {
        return "Galileo_E5b_PCPS_Acquisition";
    }

private:
    void code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq) override;

    bool acq_pilot_;
    const bool acq_iq_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E5B_PCPS_ACQUISITION_H
