/*!
 * \file gps_l1_ca_pcps_opencl_acquisition.cc
 * \brief Adapts an OpenCL PCPS acquisition block to an
 *  AcquisitionInterface for GPS L1 C/A signals
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

#include "gps_l1_ca_pcps_opencl_acquisition.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "gps_sdr_signal_replica.h"
#include "pcps_opencl_acquisition_cc.h"
#include <boost/math/distributions/exponential.hpp>
#include <algorithm>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


GpsL1CaPcpsOpenClAcquisition::GpsL1CaPcpsOpenClAcquisition(
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
          true),
      opencl_ready_(false)
{
    if (is_type_gr_complex())
        {
            const auto bit_transition_flag = configuration->property(role + ".bit_transition_flag", false);

            unsigned int max_dwells = 2;

            if (!bit_transition_flag)
                {
                    max_dwells = configuration->property(role + ".max_dwells", 1);
                }

            const std::string default_dump_filename = "./acquisition.dat";
            const auto dump_filename = configuration->property(role + ".dump_filename", default_dump_filename);
            const auto dump = configuration->property(role + ".dump", false);

            auto acquisition_cc = pcps_make_opencl_acquisition_cc(sampled_ms_, max_dwells,
                doppler_max_, doppler_step_, fs_in_, code_length_, code_length_,
                bit_transition_flag, dump, dump_filename, false);

            opencl_ready_ = acquisition_cc->opencl_ready();
            acquisition_cc_ = std::move(acquisition_cc);

            DLOG(INFO) << "acquisition(" << acquisition_cc_->unique_id() << ")";
        }
}


void GpsL1CaPcpsOpenClAcquisition::code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    gps_l1_ca_code_gen_complex_sampled(dest, prn, sampling_freq, 0);
}
