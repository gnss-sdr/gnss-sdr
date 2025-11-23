/*!
 * \file gps_l1_ca_pcps_quicksync_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  GPS L1 C/A signals using the QuickSync Algorithm
 * \author Damian Miralles, 2014. dmiralles2009@gmail.com
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

#include "gps_l1_ca_pcps_quicksync_acquisition.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include "gps_sdr_signal_replica.h"
#include "pcps_quicksync_acquisition_cc.h"
#include <boost/math/distributions/exponential.hpp>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

namespace
{
uint32_t get_folding_factor(const ConfigurationInterface* configuration, const std::string& role)
{
    const int64_t fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", static_cast<int64_t>(4000000));
    const auto fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    const auto code_length = static_cast<unsigned int>(round(fs_in / (GPS_L1_CA_CODE_RATE_CPS / GPS_L1_CA_CODE_LENGTH_CHIPS)));
    const auto folding_factor = static_cast<unsigned int>(ceil(sqrt(log2(code_length))));
    return configuration->property(role + ".folding_factor", folding_factor);
}
}  // namespace


GpsL1CaPcpsQuickSyncAcquisition::GpsL1CaPcpsQuickSyncAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : GpsL1CaPcpsQuickSyncAcquisition(configuration, role, in_streams, out_streams, get_folding_factor(configuration, role))
{
}


GpsL1CaPcpsQuickSyncAcquisition::GpsL1CaPcpsQuickSyncAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams,
    uint32_t folding_factor)
    : BasePcpsAcquisitionCustom(
          configuration,
          role,
          in_streams,
          out_streams,
          GPS_L1_CA_CODE_RATE_CPS,
          GPS_L1_CA_CODE_LENGTH_CHIPS,
          GPS_L1_CA_CODE_PERIOD_MS * folding_factor,
          true,
          ThresholdComputeQuickSync(folding_factor))
{
    if (is_type_gr_complex())
        {
            // const int samples_per_ms = round(code_length_ / acq_parameters_.sampled_ms);
            const unsigned int max_dwells = acq_parameters_.bit_transition_flag ? 2 : acq_parameters_.max_dwells;
            acquisition_cc_ = pcps_quicksync_make_acquisition_cc(acq_parameters_, folding_factor, max_dwells);
            DLOG(INFO) << "acquisition(" << acquisition_cc_->unique_id() << ")";
        }
}


void GpsL1CaPcpsQuickSyncAcquisition::code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    gps_l1_ca_code_gen_complex_sampled(dest, prn, sampling_freq, 0);
}
