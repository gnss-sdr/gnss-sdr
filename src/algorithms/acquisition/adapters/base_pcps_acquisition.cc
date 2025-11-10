/*!
 * \file gps_l1_ca_pcps_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  GPS L1 C/A signals
 * \authors <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          <li> Marc Molina, 2013. marc.molina.pena(at)gmail.com
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

#include "base_pcps_acquisition.h"
#include "acq_conf.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"

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

namespace
{
Acq_Conf get_acq_conf(const ConfigurationInterface* configuration, const std::string& role, double chip_rate, double opt_freq, uint32_t ms_per_code)
{
    Acq_Conf acq_parameters;
    acq_parameters.ms_per_code = ms_per_code;
    acq_parameters.SetFromConfiguration(configuration, role, chip_rate, opt_freq);

#if USE_GLOG_AND_GFLAGS
    if (FLAGS_doppler_max != 0)
        {
            acq_parameters.doppler_max = FLAGS_doppler_max;
        }
    if (FLAGS_doppler_step != 0)
        {
            acq_parameters.doppler_step = static_cast<float>(FLAGS_doppler_step);
        }
#else
    if (absl::GetFlag(FLAGS_doppler_max) != 0)
        {
            acq_parameters.doppler_max = absl::GetFlag(FLAGS_doppler_max);
        }
    if (absl::GetFlag(FLAGS_doppler_step) != 0)
        {
            acq_parameters.doppler_step = static_cast<float>(absl::GetFlag(FLAGS_doppler_step));
        }
#endif

    return acq_parameters;
}
}  // namespace

BasePcpsAcquisition::BasePcpsAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams,
    double chip_rate,
    double opt_freq,
    double code_length_chips,
    uint32_t ms_per_code) : acq_parameters_(get_acq_conf(configuration, role, chip_rate, opt_freq, ms_per_code)),
                            gnss_synchro_(nullptr),
                            role_(role),
                            vector_length_(std::floor(acq_parameters_.sampled_ms * acq_parameters_.samples_per_ms) * (acq_parameters_.bit_transition_flag ? 2.0 : 1.0)),
                            code_length_(static_cast<unsigned int>(std::floor(static_cast<double>(acq_parameters_.resampled_fs) / (chip_rate / code_length_chips)))),
                            code_(vector_length_),
                            acquisition_(pcps_make_acquisition(acq_parameters_))
{
    DLOG(INFO) << "role " << role;
    DLOG(INFO) << "acquisition(" << acquisition_->unique_id() << ")";

    if (acq_parameters_.item_type == "cbyte")
        {
            cbyte_to_float_x2_ = make_complex_byte_to_float_x2();
            float_to_complex_ = gr::blocks::float_to_complex::make();
        }

    if (in_streams > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams > 0)
        {
            LOG(ERROR) << "This implementation does not provide an output stream";
        }
}


void BasePcpsAcquisition::stop_acquisition()
{
    acquisition_->set_active(false);
}


void BasePcpsAcquisition::set_threshold(float threshold)
{
    acquisition_->set_threshold(threshold);
}


void BasePcpsAcquisition::set_doppler_center(int doppler_center)
{
    acquisition_->set_doppler_center(doppler_center);
}


void BasePcpsAcquisition::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    acquisition_->set_gnss_synchro(gnss_synchro_);
}


signed int BasePcpsAcquisition::mag()
{
    return acquisition_->mag();
}


void BasePcpsAcquisition::init()
{
    acquisition_->init();
}


void BasePcpsAcquisition::reset()
{
    acquisition_->set_active(true);
}


void BasePcpsAcquisition::set_state(int state)
{
    acquisition_->set_state(state);
}


void BasePcpsAcquisition::connect(gr::top_block_sptr top_block)
{
    if (acq_parameters_.item_type == "gr_complex" || acq_parameters_.item_type == "cshort")
        {
            // nothing to connect
        }
    else if (acq_parameters_.item_type == "cbyte")
        {
            // Since a byte-based acq implementation is not available,
            // we just convert cshorts to gr_complex
            top_block->connect(cbyte_to_float_x2_, 0, float_to_complex_, 0);
            top_block->connect(cbyte_to_float_x2_, 1, float_to_complex_, 1);
            top_block->connect(float_to_complex_, 0, acquisition_, 0);
        }
    else
        {
            LOG(WARNING) << acq_parameters_.item_type << " unknown acquisition item type";
        }
}


void BasePcpsAcquisition::disconnect(gr::top_block_sptr top_block)
{
    if (acq_parameters_.item_type == "gr_complex" || acq_parameters_.item_type == "cshort")
        {
            // nothing to disconnect
        }
    else if (acq_parameters_.item_type == "cbyte")
        {
            top_block->disconnect(cbyte_to_float_x2_, 0, float_to_complex_, 0);
            top_block->disconnect(cbyte_to_float_x2_, 1, float_to_complex_, 1);
            top_block->disconnect(float_to_complex_, 0, acquisition_, 0);
        }
    else
        {
            LOG(WARNING) << acq_parameters_.item_type << " unknown acquisition item type";
        }
}


gr::basic_block_sptr BasePcpsAcquisition::get_left_block()
{
    if (acq_parameters_.item_type == "gr_complex" || acq_parameters_.item_type == "cshort")
        {
            return acquisition_;
        }
    if (acq_parameters_.item_type == "cbyte")
        {
            return cbyte_to_float_x2_;
        }

    LOG(WARNING) << acq_parameters_.item_type << " unknown acquisition item type";
    return nullptr;
}


gr::basic_block_sptr BasePcpsAcquisition::get_right_block()
{
    return acquisition_;
}


void BasePcpsAcquisition::set_resampler_latency(uint32_t latency_samples)
{
    acquisition_->set_resampler_latency(latency_samples);
}


void BasePcpsAcquisition::set_local_code()
{
    volk_gnsssdr::vector<std::complex<float>> code(code_length_);

    const auto sampling_freq = acq_parameters_.use_automatic_resampler ? acq_parameters_.resampled_fs : acq_parameters_.fs_in;
    code_gen_complex_sampled(code, gnss_synchro_->PRN, sampling_freq);

    const auto num_codes = acq_parameters_.sampled_ms / acq_parameters_.ms_per_code;

    own::span<gr_complex> code_span(code_.data(), vector_length_);
    for (unsigned int i = 0; i < num_codes; i++)
        {
            std::copy_n(code.data(), code_length_, code_span.subspan(i * code_length_, code_length_).data());
        }

    acquisition_->set_local_code(code_.data());
}
