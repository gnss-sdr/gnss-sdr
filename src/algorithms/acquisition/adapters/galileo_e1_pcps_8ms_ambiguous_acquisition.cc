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

namespace
{
unsigned int get_sampled_ms(const ConfigurationInterface* configuration, const std::string& role)
{
    unsigned int sampled_ms = configuration->property(role + ".coherent_integration_time_ms", GALILEO_E1_CODE_PERIOD_MS);

    if (sampled_ms % GALILEO_E1_CODE_PERIOD_MS != 0)
        {
            sampled_ms = static_cast<int>(sampled_ms / GALILEO_E1_CODE_PERIOD_MS) * GALILEO_E1_CODE_PERIOD_MS;
            LOG(WARNING) << "coherent_integration_time should be multiple of "
                         << "Galileo code length (" << GALILEO_E1_CODE_PERIOD_MS << " ms). coherent_integration_time = "
                         << sampled_ms << " ms will be used.";
        }

    return sampled_ms;
}
}  // namespace


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
          get_sampled_ms(configuration, role),
          GALILEO_E1_CODE_CHIP_RATE_CPS,
          GALILEO_E1_B_CODE_LENGTH_CHIPS,
          GALILEO_E1_CODE_PERIOD_MS,
          true,
          true),
      cboc_(configuration->property(role + ".cboc", false))
{
    if (is_type_gr_complex())
        {
            const auto samples_per_ms = static_cast<int>(code_length_) / GALILEO_E1_CODE_PERIOD_MS;
            const std::string default_dump_filename("./acquisition.dat");
            const auto dump_filename = configuration->property(role + ".dump_filename", default_dump_filename);
            const auto enable_monitor_output = configuration->property("AcquisitionMonitor.enable_monitor", false);
            const auto dump = configuration->property(role + ".dump", false);
            const auto max_dwells = configuration->property(role + ".max_dwells", 1U);

            acquisition_cc_ = galileo_pcps_8ms_make_acquisition_cc(sampled_ms_, max_dwells,
                doppler_max_, doppler_step_, fs_in_, samples_per_ms, code_length_,
                dump, dump_filename, enable_monitor_output);

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
