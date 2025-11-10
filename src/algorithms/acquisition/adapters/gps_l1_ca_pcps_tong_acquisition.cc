/*!
 * \file gps_l1_ca_pcps_tong_acquisition.cc
 * \brief Adapts a PCPS Tong acquisition block to an AcquisitionInterface for
 *  GPS L1 C/A signals
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

#include "gps_l1_ca_pcps_tong_acquisition.h"
#include "GPS_L1_CA.h"
#include "gps_sdr_signal_replica.h"
#include "pcps_tong_acquisition_cc.h"
#include <boost/math/distributions/exponential.hpp>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


GpsL1CaPcpsTongAcquisition::GpsL1CaPcpsTongAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BasePcpsAcquisitionCustom(
          configuration,
          role,
          in_streams,
          out_streams,
          configuration->property(role + ".coherent_integration_time_ms", GPS_L1_CA_CODE_PERIOD_MS),
          GPS_L1_CA_CODE_RATE_CPS,
          GPS_L1_CA_CODE_LENGTH_CHIPS,
          GPS_L1_CA_CODE_PERIOD_MS,
          true,
          true)
{
    if (is_type_gr_complex())
        {
            const std::string default_dump_filename("./acquisition.dat");
            const auto dump_filename = configuration->property(role + ".dump_filename", default_dump_filename);
            const auto enable_monitor_output = configuration->property("AcquisitionMonitor.enable_monitor", false);
            const auto dump = configuration->property(role + ".dump", false);
            const auto tong_init_val = configuration->property(role + ".tong_init_val", 1U);
            const auto tong_max_val = configuration->property(role + ".tong_max_val", 2U);
            const auto tong_max_dwells = configuration->property(role + ".tong_max_dwells", tong_max_val + 1U);

            acquisition_cc_ = pcps_tong_make_acquisition_cc(sampled_ms_, doppler_max_, doppler_step_, fs_in_,
                code_length_, code_length_, tong_init_val, tong_max_val, tong_max_dwells,
                dump, dump_filename, enable_monitor_output);

            DLOG(INFO) << "acquisition(" << acquisition_cc_->unique_id() << ")";
        }
}


void GpsL1CaPcpsTongAcquisition::code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    gps_l1_ca_code_gen_complex_sampled(dest, prn, sampling_freq, 0);
}
