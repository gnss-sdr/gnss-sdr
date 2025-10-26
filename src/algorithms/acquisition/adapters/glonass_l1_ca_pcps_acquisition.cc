/*!
 * \file glonass_l1_ca_pcps_acquisition.cc
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

#include "glonass_l1_ca_pcps_acquisition.h"
#include "GLONASS_L1_L2_CA.h"
#include "glonass_l1_signal_replica.h"


GlonassL1CaPcpsAcquisition::GlonassL1CaPcpsAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : BasePcpsAcquisition(configuration, role, in_streams, out_streams, GLONASS_L1_CA_CODE_RATE_CPS, 100e6, GLONASS_L1_CA_CODE_LENGTH_CHIPS, 1)
{
}


void GlonassL1CaPcpsAcquisition::code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t /*prn*/, int32_t sampling_freq)
{
    glonass_l1_ca_code_gen_complex_sampled(dest, sampling_freq, 0);
}
