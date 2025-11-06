/*!
 * \file galileo_e5b_pcps_acquisition.cc
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

#include "galileo_e5b_pcps_acquisition.h"
#include "Galileo_E5b.h"
#include "galileo_e5_signal_replica.h"

GalileoE5bPcpsAcquisition::GalileoE5bPcpsAcquisition(const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BasePcpsAcquisition(configuration,
          role,
          in_streams,
          out_streams,
          GALILEO_E5B_CODE_CHIP_RATE_CPS,
          GALILEO_E5B_OPT_ACQ_FS_SPS,
          GALILEO_E5B_CODE_LENGTH_CHIPS,
          GALILEO_E5B_CODE_PERIOD_MS),
      acq_pilot_(configuration->property(role + ".acquire_pilot", false)),
      acq_iq_(configuration->property(role + ".acquire_iq", false))
{
    if (acq_iq_)
        {
            acq_pilot_ = false;
        }
}


void GalileoE5bPcpsAcquisition::code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    std::array<char, 3> signal_{};
    signal_[0] = '7';
    signal_[2] = '\0';

    if (acq_iq_)
        {
            signal_[1] = 'X';
        }
    else if (acq_pilot_)
        {
            signal_[1] = 'Q';
        }
    else
        {
            signal_[1] = 'I';
        }

    galileo_e5_b_code_gen_complex_sampled(dest, prn, signal_, sampling_freq, 0);
}
