/*!
 * \file glonass_l2_ca_pcps_acquisition.cc
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

#include "glonass_l2_ca_pcps_acquisition.h"
#include "GLONASS_L1_L2_CA.h"
#include "glonass_l2_signal_replica.h"


GlonassL2CaPcpsAcquisition::GlonassL2CaPcpsAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BasePcpsAcquisition(configuration,
          role,
          in_streams,
          out_streams,
          GLONASS_L2_CA_CODE_RATE_CPS,
          100e6,
          GLONASS_L2_CA_CODE_LENGTH_CHIPS,
          GLONASS_L2_CA_CODE_PEROD_MS)
{
}


void GlonassL2CaPcpsAcquisition::code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t /*prn*/, int32_t sampling_freq)
{
    glonass_l2_ca_code_gen_complex_sampled(dest, sampling_freq, 0);
}
