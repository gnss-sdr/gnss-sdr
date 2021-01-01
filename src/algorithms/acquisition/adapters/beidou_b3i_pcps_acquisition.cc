/*!
 * \file beidou_b3i_pcps_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  BeiDou B3I signals
 * \author Damian Miralles, 2019. dmiralles2009@gmail.com
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

#include "beidou_b3i_pcps_acquisition.h"
#include "Beidou_B3I.h"
#include "acq_conf.h"
#include "beidou_b3i_signal_replica.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include <glog/logging.h>
#include <algorithm>

#if HAS_STD_SPAN
#include <span>
namespace own = std;
#else
#include <gsl/gsl>
namespace own = gsl;
#endif

using google::LogMessage;

BeidouB3iPcpsAcquisition::BeidouB3iPcpsAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : role_(role),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    acq_parameters_.ms_per_code = 1;
    acq_parameters_.SetFromConfiguration(configuration, role, BEIDOU_B3I_CODE_RATE_CPS, 100e6);

    LOG(INFO) << "role " << role;

    if (FLAGS_doppler_max != 0)
        {
            acq_parameters_.doppler_max = FLAGS_doppler_max;
        }
    doppler_max_ = acq_parameters_.doppler_max;
    doppler_step_ = static_cast<unsigned int>(acq_parameters_.doppler_step);
    item_type_ = acq_parameters_.item_type;
    item_size_ = acq_parameters_.it_size;
    fs_in_ = acq_parameters_.fs_in;

    num_codes_ = acq_parameters_.sampled_ms;
    code_length_ = static_cast<unsigned int>(std::floor(static_cast<double>(fs_in_) / (BEIDOU_B3I_CODE_RATE_CPS / BEIDOU_B3I_CODE_LENGTH_CHIPS)));
    vector_length_ = static_cast<unsigned int>(std::floor(acq_parameters_.sampled_ms * acq_parameters_.samples_per_ms) * (acq_parameters_.bit_transition_flag ? 2.0 : 1.0));
    code_ = std::vector<std::complex<float>>(vector_length_);

    acquisition_ = pcps_make_acquisition(acq_parameters_);
    DLOG(INFO) << "acquisition(" << acquisition_->unique_id() << ")";

    if (item_type_ == "cbyte")
        {
            cbyte_to_float_x2_ = make_complex_byte_to_float_x2();
            float_to_complex_ = gr::blocks::float_to_complex::make();
        }

    channel_ = 0;
    threshold_ = 0.0;
    gnss_synchro_ = nullptr;

    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 0)
        {
            LOG(ERROR) << "This implementation does not provide an output stream";
        }
}


void BeidouB3iPcpsAcquisition::stop_acquisition()
{
    acquisition_->set_active(false);
}


void BeidouB3iPcpsAcquisition::set_threshold(float threshold)
{
    threshold_ = threshold;

    acquisition_->set_threshold(threshold_);
}


void BeidouB3iPcpsAcquisition::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;

    acquisition_->set_doppler_max(doppler_max_);
}


void BeidouB3iPcpsAcquisition::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;

    acquisition_->set_doppler_step(doppler_step_);
}


void BeidouB3iPcpsAcquisition::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;

    acquisition_->set_gnss_synchro(gnss_synchro_);
}


signed int BeidouB3iPcpsAcquisition::mag()
{
    return acquisition_->mag();
}


void BeidouB3iPcpsAcquisition::init()
{
    acquisition_->init();
}


void BeidouB3iPcpsAcquisition::set_local_code()
{
    std::vector<std::complex<float>> code(code_length_);

    beidou_b3i_code_gen_complex_sampled(code, gnss_synchro_->PRN, fs_in_, 0);

    own::span<gr_complex> code_span(code_.data(), vector_length_);
    for (unsigned int i = 0; i < num_codes_; i++)
        {
            std::copy_n(code.data(), code_length_, code_span.subspan(i * code_length_, code_length_).data());
        }

    acquisition_->set_local_code(code_.data());
}


void BeidouB3iPcpsAcquisition::reset()
{
    acquisition_->set_active(true);
}


void BeidouB3iPcpsAcquisition::set_state(int state)
{
    acquisition_->set_state(state);
}


void BeidouB3iPcpsAcquisition::connect(gr::top_block_sptr top_block)
{
    if (item_type_ == "gr_complex" || item_type_ == "cshort")
        {
            // nothing to connect
        }
    else if (item_type_ == "cbyte")
        {
            // Since a byte-based acq implementation is not available,
            // we just convert cshorts to gr_complex
            top_block->connect(cbyte_to_float_x2_, 0, float_to_complex_, 0);
            top_block->connect(cbyte_to_float_x2_, 1, float_to_complex_, 1);
            top_block->connect(float_to_complex_, 0, acquisition_, 0);
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }
}


void BeidouB3iPcpsAcquisition::disconnect(gr::top_block_sptr top_block)
{
    if (item_type_ == "gr_complex" || item_type_ == "cshort")
        {
            // nothing to disconnect
        }
    else if (item_type_ == "cbyte")
        {
            top_block->disconnect(cbyte_to_float_x2_, 0, float_to_complex_, 0);
            top_block->disconnect(cbyte_to_float_x2_, 1, float_to_complex_, 1);
            top_block->disconnect(float_to_complex_, 0, acquisition_, 0);
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }
}


gr::basic_block_sptr BeidouB3iPcpsAcquisition::get_left_block()
{
    if (item_type_ == "gr_complex" || item_type_ == "cshort")
        {
            return acquisition_;
        }
    if (item_type_ == "cbyte")
        {
            return cbyte_to_float_x2_;
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
            return nullptr;
        }
}


gr::basic_block_sptr BeidouB3iPcpsAcquisition::get_right_block()
{
    return acquisition_;
}


void BeidouB3iPcpsAcquisition::set_resampler_latency(uint32_t latency_samples)
{
    acquisition_->set_resampler_latency(latency_samples);
}
