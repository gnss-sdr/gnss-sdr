/*!
 * \file galileo_e5a_noncoherent_iq_acquisition_caf.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E5a data and pilot Signals
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \based on work from:
 *          <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          <li> Marc Molina, 2013. marc.molina.pena@gmail.com
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

#ifndef GNSS_SDR_GALILEO_E5A_NONCOHERENT_IQ_ACQUISITION_CAF_H
#define GNSS_SDR_GALILEO_E5A_NONCOHERENT_IQ_ACQUISITION_CAF_H

#include "base_pcps_acquisition_custom.h"

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_adapters
 * \{ */

class GalileoE5aNoncoherentIQAcquisitionCaf : public BasePcpsAcquisitionCustom
{
public:
    GalileoE5aNoncoherentIQAcquisitionCaf(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~GalileoE5aNoncoherentIQAcquisitionCaf() = default;

    /*!
     * \brief Returns "Galileo_E5a_Noncoherent_IQ_Acquisition_CAF"
     */
    inline std::string implementation() override
    {
        return "Galileo_E5a_Noncoherent_IQ_Acquisition_CAF";
    }

    /*!
     * \brief Sets local Galileo E5a code for PCPS acquisition algorithm.
     */
    void set_local_code() override;

private:
    // We don't implement this function since we override set_local_code
    void code_gen_complex_sampled(own::span<std::complex<float>> /*dest*/, uint32_t /*prn*/, int32_t /*sampling_freq*/) override {}

    const int zero_padding_;
    const int caf_window_hz_;

    std::vector<std::complex<float>> codeI_;
    std::vector<std::complex<float>> codeQ_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E5A_NONCOHERENT_IQ_ACQUISITION_CAF_H
