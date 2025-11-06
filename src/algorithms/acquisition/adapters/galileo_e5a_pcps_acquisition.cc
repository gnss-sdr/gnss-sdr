/*!
 * \file galileo_e5a_pcps_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E5a data and pilot Signals
 * \author Antonio Ramos, 2018. antonio.ramos(at)cttc.es
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

#include "galileo_e5a_pcps_acquisition.h"
#include "Galileo_E5a.h"
#include "acq_conf.h"
#include "configuration_interface.h"
#include "galileo_e5_signal_replica.h"
#include "gnss_sdr_flags.h"
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <algorithm>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

#if HAS_STD_SPAN
#include <span>
namespace own = std;
#else
#include <gsl-lite/gsl-lite.hpp>
namespace own = gsl_lite;
#endif

GalileoE5aPcpsAcquisition::GalileoE5aPcpsAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BasePcpsAcquisition(configuration,
          role,
          in_streams,
          out_streams,
          GALILEO_E5A_CODE_CHIP_RATE_CPS,
          GALILEO_E5A_OPT_ACQ_FS_SPS,
          GALILEO_E5A_CODE_LENGTH_CHIPS,
          GALILEO_E5A_CODE_PERIOD_MS),
      acq_pilot_(configuration->property(role + ".acquire_pilot", false)),
      acq_iq_(configuration->property(role + ".acquire_iq", false))
{
    if (acq_iq_)
        {
            acq_pilot_ = false;
        }
}


void GalileoE5aPcpsAcquisition::code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    std::array<char, 3> signal_{};
    signal_[0] = '5';
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

    galileo_e5_a_code_gen_complex_sampled(dest, prn, signal_, sampling_freq, 0);
}
