/*!
 * \file beidou_b3i_pcps_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  BeiDou B3I signals
 * \author Damian Miralles, 2019. dmiralles2009@gmail.com
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

#include "beidou_b3i_pcps_acquisition.h"
#include "Beidou_B3I.h"
#include "beidou_b3i_signal_replica.h"


BeidouB3iPcpsAcquisition::BeidouB3iPcpsAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : BasePcpsAcquisition(configuration, role, in_streams, out_streams, BEIDOU_B3I_CODE_RATE_CPS, 100e6, BEIDOU_B3I_CODE_LENGTH_CHIPS, 1)
{
}


void BeidouB3iPcpsAcquisition::code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    beidou_b3i_code_gen_complex_sampled(dest, prn, sampling_freq, 0);
}
