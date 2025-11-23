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

Acq_Conf get_acq_conf(
    const ConfigurationInterface* configuration,
    const std::string& role,
    double chip_rate,
    double code_length_chips,
    double opt_freq,
    uint32_t ms_per_code,
    uint32_t max_sampled_ms,
    const ThresholdComputeInterface& threshold_compute)
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
            acq_parameters.doppler_step = FLAGS_doppler_step;
        }
#else
    if (absl::GetFlag(FLAGS_doppler_max) != 0)
        {
            acq_parameters.doppler_max = absl::GetFlag(FLAGS_doppler_max);
        }
    if (absl::GetFlag(FLAGS_doppler_step) != 0)
        {
            acq_parameters.doppler_step = absl::GetFlag(FLAGS_doppler_step);
        }
#endif

    if (acq_parameters.sampled_ms > max_sampled_ms)
        {
            acq_parameters.sampled_ms = max_sampled_ms;
            DLOG(INFO) << "Coherent integration time should be " << max_sampled_ms << " ms or less. Changing to " << max_sampled_ms << "ms ";
            std::cout << "Too high coherent integration time. Changing to " << max_sampled_ms << "ms\n";
        }

    acq_parameters.num_codes = acq_parameters.sampled_ms / ms_per_code;
    acq_parameters.code_length = static_cast<unsigned int>(round(acq_parameters.fs_in / (chip_rate / code_length_chips)));
    acq_parameters.vector_length = acq_parameters.code_length * acq_parameters.num_codes;
    acq_parameters.threshold = threshold_compute.calculate_threshold(acq_parameters);

    return acq_parameters;
}
}  // namespace


float ThresholdComputeBasic::calculate_threshold(const Acq_Conf& acq_parameters) const
{
    return acq_parameters.threshold;
}

float ThresholdComputeDoppler::calculate_threshold(const Acq_Conf& acq_parameters) const
{
    if (acq_parameters.pfa != 0)
        {
            // Calculate the threshold
            unsigned int frequency_bins = 0;
            for (int doppler = -acq_parameters.doppler_max; doppler <= acq_parameters.doppler_max; doppler += acq_parameters.doppler_step)
                {
                    frequency_bins++;
                }

            const auto ncells = acq_parameters.vector_length * frequency_bins;
            const auto exponent = 1 / static_cast<double>(ncells);
            const auto val = pow(1.0 - acq_parameters.pfa, exponent);
            const auto lambda = static_cast<double>(acq_parameters.vector_length);
            boost::math::exponential_distribution<double> mydist(lambda);
            const auto threshold = static_cast<float>(quantile(mydist, val));

            return threshold;
        }

    return acq_parameters.threshold;
}

ThresholdComputeQuickSync::ThresholdComputeQuickSync(uint32_t folding_factor) : folding_factor_(folding_factor)
{
}

float ThresholdComputeQuickSync::calculate_threshold(const Acq_Conf& acq_parameters) const
{
    if (acq_parameters.pfa != 0)
        {
            // Calculate the threshold
            unsigned int frequency_bins = 0;
            for (int doppler = -acq_parameters.doppler_max; doppler <= acq_parameters.doppler_max; doppler += static_cast<int>(acq_parameters.doppler_step))
                {
                    frequency_bins++;
                }

            const auto ncells = (acq_parameters.code_length / folding_factor_) * frequency_bins;
            const auto exponent = 1.0 / static_cast<double>(ncells);
            const auto val = pow(1.0 - acq_parameters.pfa, exponent);
            const auto lambda = static_cast<double>(acq_parameters.code_length) / static_cast<double>(folding_factor_);
            boost::math::exponential_distribution<double> mydist(lambda);
            const auto threshold = static_cast<float>(quantile(mydist, val));
            return threshold;
        }

    return acq_parameters.threshold;
}

BasePcpsAcquisitionCustom::BasePcpsAcquisitionCustom(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams,
    double chip_rate,
    double code_length_chips,
    unsigned int ms_per_code,
    bool use_stream_to_vector,
    const ThresholdComputeInterface& threshold_compute,
    uint32_t max_sampled_ms)
    : acq_parameters_(get_acq_conf(configuration, role, chip_rate, code_length_chips, 0, ms_per_code, max_sampled_ms, threshold_compute)),
      gnss_synchro_(nullptr),
      channel_(0),
      code_(acq_parameters_.vector_length),
      role_(role),
      is_type_gr_complex_(acq_parameters_.item_type == "gr_complex"),
      item_size_(is_type_gr_complex_ ? sizeof(gr_complex) : 0),
      use_stream_to_vector_(use_stream_to_vector)
{
    DLOG(INFO) << "role " << role_;

    if (is_type_gr_complex_)
        {
            if (use_stream_to_vector_)
                {
                    stream_to_vector_ = gr::blocks::stream_to_vector::make(item_size_, acq_parameters_.vector_length);
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
            acquisition_cc_->set_active(false);
        }
}


void BasePcpsAcquisitionCustom::set_local_code()
{
    if (is_type_gr_complex())
        {
            const auto code_length = acq_parameters_.code_length;
            std::vector<std::complex<float>> code(code_length);
            code_gen_complex_sampled(code, gnss_synchro_->PRN, acq_parameters_.fs_in);

            own::span<gr_complex> code_span(code_.data(), acq_parameters_.vector_length);
            for (unsigned int i = 0; i < acq_parameters_.num_codes; i++)
                {
                    std::copy_n(code.data(), code_length, code_span.subspan(i * code_length, code_length).data());
                }

            acquisition_cc_->set_local_code(code_.data());
        }
}
