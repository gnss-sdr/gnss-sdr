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
          configuration->property(role + ".coherent_integration_time_ms", GPS_L1_CA_CODE_PERIOD_MS),
          GPS_L1_CA_CODE_RATE_CPS,
          GPS_L1_CA_CODE_LENGTH_CHIPS,
          GPS_L1_CA_CODE_PERIOD_MS,
          false,
          false)
{
    if (is_type_gr_complex())
        {
            const std::string default_dump_filename = "./acquisition.mat";

            Acq_Conf acq_parameters = Acq_Conf();
            acq_parameters.fs_in = fs_in_;
            acq_parameters.samples_per_chip = static_cast<unsigned int>(ceil(GPS_L1_CA_CHIP_PERIOD_S * static_cast<float>(fs_in_)));
            acq_parameters.dump = configuration->property(role + ".dump", false);
            acq_parameters.dump_filename = configuration->property(role + ".dump_filename", default_dump_filename);
            acq_parameters.doppler_max = doppler_max_;
            acq_parameters.doppler_step = doppler_step_;
            acq_parameters.sampled_ms = sampled_ms_;
            acq_parameters.max_dwells = configuration->property(role + ".max_dwells", 1);
            acq_parameters.blocking_on_standby = configuration->property(role + ".blocking_on_standby", false);
            acq_parameters.samples_per_ms = static_cast<float>(vector_length_);

            acquisition_cc_ = pcps_make_acquisition_fine_doppler_cc(acq_parameters);

            DLOG(INFO) << "acquisition(" << acquisition_cc_->unique_id() << ")";
        }
}


void GpsL1CaPcpsAcquisitionFineDoppler::code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    gps_l1_ca_code_gen_complex_sampled(dest, prn, sampling_freq, 0);
}
