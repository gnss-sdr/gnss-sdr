/*!
 * \file galileo_e6_pcps_acquisition.cc
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

#include "galileo_e6_pcps_acquisition.h"
#include "Galileo_E6.h"
#include "galileo_e6_signal_replica.h"


GalileoE6PcpsAcquisition::GalileoE6PcpsAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BasePcpsAcquisition(configuration,
          role,
          in_streams,
          out_streams,
          GALILEO_E6_B_CODE_CHIP_RATE_CPS,
          GALILEO_E6_OPT_ACQ_FS_SPS,
          GALILEO_E6_B_CODE_LENGTH_CHIPS,
          GALILEO_E6_CODE_PERIOD_MS)
{
}


void GalileoE6PcpsAcquisition::code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    galileo_e6_b_code_gen_complex_sampled(dest, prn, sampling_freq, 0);
}
