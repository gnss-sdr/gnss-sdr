/*!
 * \file base_ca_pcps_acquisition_custom.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface
 * \authors <ul>
 *          <li> Mathieu Favreau, 2025. favreau.mathieu(at)hotmail.com
 *          </ul>
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2025  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "base_pcps_acquisition_custom.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include <boost/math/distributions/exponential.hpp>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


namespace
{
const std::string default_dump_filename("./acquisition.dat");

Acq_Conf get_acq_conf(const ConfigurationInterface* configuration, const std::string& role, double chip_rate, double opt_freq, uint32_t ms_per_code)
{
    Acq_Conf acq_parameters;
    acq_parameters.ms_per_code = ms_per_code;
    acq_parameters.sampled_ms = ms_per_code;               // Set as default value
    acq_parameters.dump_filename = default_dump_filename;  // Set as default value
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


BasePcpsAcquisitionCustom::BasePcpsAcquisitionCustom(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams,
    double chip_rate,
    double code_length_chips,
    unsigned int ms_per_code,
    bool use_stream_to_vector,
    bool compute_threshold_from_pfa)
    : acq_parameters_(get_acq_conf(configuration, role, chip_rate, 0, ms_per_code)),
      ms_per_code_(ms_per_code),
      code_length_(static_cast<unsigned int>(round(acq_parameters_.fs_in / (chip_rate / code_length_chips)))),
      vector_length_(code_length_ * static_cast<int>(acq_parameters_.sampled_ms / ms_per_code)),
      gnss_synchro_(nullptr),
      channel_(0),
      code_(vector_length_),
      role_(role),
      is_type_gr_complex_(acq_parameters_.item_type == "gr_complex"),
      item_size_(is_type_gr_complex_ ? sizeof(gr_complex) : 0),
      use_stream_to_vector_(use_stream_to_vector),
      compute_threshold_from_pfa_(compute_threshold_from_pfa)
{
    DLOG(INFO) << "role " << role_;

    if (is_type_gr_complex_)
        {
            if (use_stream_to_vector_)
                {
                    stream_to_vector_ = gr::blocks::stream_to_vector::make(item_size_, vector_length_);
                    DLOG(INFO) << "stream_to_vector(" << stream_to_vector_->unique_id() << ")";
                }
        }
    else
        {
            LOG(WARNING) << acq_parameters_.item_type << " unknown acquisition item type";
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


void BasePcpsAcquisitionCustom::connect(gr::top_block_sptr top_block)
{
    if (is_type_gr_complex_ && use_stream_to_vector_)
        {
            top_block->connect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
}


void BasePcpsAcquisitionCustom::disconnect(gr::top_block_sptr top_block)
{
    if (is_type_gr_complex_ && use_stream_to_vector_)
        {
            top_block->disconnect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
}


gr::basic_block_sptr BasePcpsAcquisitionCustom::get_left_block()
{
    if (use_stream_to_vector_)
        {
            return stream_to_vector_;
        }
    return acquisition_cc_;
}


gr::basic_block_sptr BasePcpsAcquisitionCustom::get_right_block()
{
    return acquisition_cc_;
}


void BasePcpsAcquisitionCustom::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;

    if (is_type_gr_complex_)
        {
            acquisition_cc_->set_gnss_synchro(gnss_synchro);
        }
}

void BasePcpsAcquisitionCustom::set_channel(unsigned int channel)
{
    channel_ = channel;

    if (is_type_gr_complex_)
        {
            acquisition_cc_->set_channel(channel);
        }
}


void BasePcpsAcquisitionCustom::set_channel_fsm(std::weak_ptr<ChannelFsm> channel_fsm)
{
    if (is_type_gr_complex_)
        {
            acquisition_cc_->set_channel_fsm(channel_fsm);
        }
}


void BasePcpsAcquisitionCustom::init()
{
    if (is_type_gr_complex_)
        {
            acquisition_cc_->init();
        }
}


signed int BasePcpsAcquisitionCustom::mag()
{
    if (is_type_gr_complex_)
        {
            return acquisition_cc_->mag();
        }
    return 0;
}


void BasePcpsAcquisitionCustom::reset()
{
    if (is_type_gr_complex_)
        {
            acquisition_cc_->set_active(true);
        }
}


void BasePcpsAcquisitionCustom::stop_acquisition()
{
    if (is_type_gr_complex_)
        {
            acquisition_cc_->set_state(0);
            acquisition_cc_->set_active(false);
        }
}


void BasePcpsAcquisitionCustom::set_state(int state)
{
    if (is_type_gr_complex_)
        {
            acquisition_cc_->set_state(state);
        }
}


void BasePcpsAcquisitionCustom::set_threshold(float threshold)
{
    if (is_type_gr_complex_)
        {
            if (compute_threshold_from_pfa_ && acq_parameters_.pfa != 0)
                {
                    threshold = calculate_threshold(acq_parameters_.pfa);
                    DLOG(INFO) << "Channel " << channel_ << " Threshold = " << threshold;
                }

            acquisition_cc_->set_threshold(threshold);
        }
}


void BasePcpsAcquisitionCustom::set_local_code()
{
    if (is_type_gr_complex())
        {
            std::vector<std::complex<float>> code(code_length_);
            code_gen_complex_sampled(code, gnss_synchro_->PRN, acq_parameters_.fs_in);

            const auto num_codes = acq_parameters_.sampled_ms / ms_per_code_;

            own::span<gr_complex> code_span(code_.data(), vector_length_);
            for (unsigned int i = 0; i < num_codes; i++)
                {
                    std::copy_n(code.data(), code_length_, code_span.subspan(i * code_length_, code_length_).data());
                }

            acquisition_cc_->set_local_code(code_.data());
        }
}


float BasePcpsAcquisitionCustom::calculate_threshold(float pfa) const
{
    // Calculate the threshold
    unsigned int frequency_bins = 0;
    for (int doppler = -acq_parameters_.doppler_max; doppler <= acq_parameters_.doppler_max; doppler += static_cast<int>(acq_parameters_.doppler_step))
        {
            frequency_bins++;
        }

    DLOG(INFO) << "Channel " << channel_ << "  Pfa = " << pfa;

    const auto ncells = vector_length_ * frequency_bins;
    const auto exponent = 1 / static_cast<double>(ncells);
    const auto val = pow(1.0 - pfa, exponent);
    const auto lambda = static_cast<double>(vector_length_);
    boost::math::exponential_distribution<double> mydist(lambda);
    const auto threshold = static_cast<float>(quantile(mydist, val));

    return threshold;
}
