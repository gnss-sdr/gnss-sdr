/*!
 * \file gps_l1_ca_pcps_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  GPS L1 C/A Signals
 * \authors <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          </ul>
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

#include "gps_l1_ca_pcps_acquisition_fine_doppler.h"
#include "GPS_L1_CA.h"
#include "acq_conf.h"
#include "gps_sdr_signal_replica.h"
#include "pcps_acquisition_fine_doppler_cc.h"

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

GpsL1CaPcpsAcquisitionFineDoppler::GpsL1CaPcpsAcquisitionFineDoppler(
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
          false,
          false)
{
    if (is_type_gr_complex())
        {
            Acq_Conf acq_parameters = acq_parameters_;
            acq_parameters.samples_per_ms = static_cast<float>(vector_length_);

            acquisition_cc_ = pcps_make_acquisition_fine_doppler_cc(acq_parameters);

            DLOG(INFO) << "acquisition(" << acquisition_cc_->unique_id() << ")";
        }
}


void GpsL1CaPcpsAcquisitionFineDoppler::code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    gps_l1_ca_code_gen_complex_sampled(dest, prn, sampling_freq, 0);
}
