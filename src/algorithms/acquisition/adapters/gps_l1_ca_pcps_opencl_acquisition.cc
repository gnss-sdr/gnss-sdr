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
          GPS_L1_CA_CODE_RATE_CPS,
          GPS_L1_CA_CODE_LENGTH_CHIPS,
          GPS_L1_CA_CODE_PERIOD_MS,
          true,
          true),
      opencl_ready_(false)
{
    if (is_type_gr_complex())
        {
            const unsigned int max_dwells = acq_parameters_.bit_transition_flag ? 2 : acq_parameters_.max_dwells;

            auto acquisition_cc = pcps_make_opencl_acquisition_cc(acq_parameters_.sampled_ms, max_dwells,
                acq_parameters_.doppler_max, acq_parameters_.doppler_step, acq_parameters_.fs_in, code_length_, code_length_,
                acq_parameters_.bit_transition_flag, acq_parameters_.dump, acq_parameters_.dump_filename, false);

            opencl_ready_ = acquisition_cc->opencl_ready();
            acquisition_cc_ = std::move(acquisition_cc);

            DLOG(INFO) << "acquisition(" << acquisition_cc_->unique_id() << ")";
        }
}


void GpsL1CaPcpsOpenClAcquisition::code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    gps_l1_ca_code_gen_complex_sampled(dest, prn, sampling_freq, 0);
}
