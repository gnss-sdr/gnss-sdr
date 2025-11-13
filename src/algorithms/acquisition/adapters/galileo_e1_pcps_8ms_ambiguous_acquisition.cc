/*!
 * \file galileo_e1_pcps_8ms_ambiguous_acquisition.cc
 * \brief Adapts a Galileo PCPS 8ms acquisition block to an
 * AcquisitionInterface for Galileo E1 Signals
 * \author Marc Molina, 2013. marc.molina.pena(at)gmail.com
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

#include "galileo_e1_pcps_8ms_ambiguous_acquisition.h"
#include "Galileo_E1.h"
#include "galileo_e1_signal_replica.h"
#include "galileo_pcps_8ms_acquisition_cc.h"
#include <boost/math/distributions/exponential.hpp>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


GalileoE1Pcps8msAmbiguousAcquisition::GalileoE1Pcps8msAmbiguousAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BasePcpsAcquisitionCustom(
          configuration,
          role,
          in_streams,
          out_streams,
          GALILEO_E1_CODE_CHIP_RATE_CPS,
          GALILEO_E1_B_CODE_LENGTH_CHIPS,
          GALILEO_E1_CODE_PERIOD_MS,
          true,
          true),
      cboc_(configuration->property(role + ".cboc", false))
{
    if (is_type_gr_complex())
        {
            acquisition_cc_ = galileo_pcps_8ms_make_acquisition_cc(acq_parameters_);
            DLOG(INFO) << "acquisition(" << acquisition_cc_->unique_id() << ")";
        }
}


void GalileoE1Pcps8msAmbiguousAcquisition::code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    std::array<char, 3> Signal_{};
    Signal_[0] = gnss_synchro_->Signal[0];
    Signal_[1] = gnss_synchro_->Signal[1];
    Signal_[2] = '\0';

    galileo_e1_code_gen_complex_sampled(dest, Signal_, cboc_, prn, sampling_freq, 0, false);
}
