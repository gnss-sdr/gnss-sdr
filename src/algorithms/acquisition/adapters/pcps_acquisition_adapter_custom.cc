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

#include "pcps_acquisition_adapter_custom.h"
#include "GPS_L1_CA.h"
#include "Galileo_E1.h"
#include "Galileo_E5a.h"
#include "configuration_interface.h"
#include "galileo_e1_signal_replica.h"
#include "galileo_e5_signal_replica.h"
#include "galileo_e5a_noncoherent_iq_acquisition_caf_cc.h"
#include "galileo_pcps_8ms_acquisition_cc.h"
#include "gnss_sdr_flags.h"
#include "gps_sdr_signal_replica.h"
#include "pcps_acquisition_fine_doppler_cc.h"
#include "pcps_assisted_acquisition_cc.h"
#include "pcps_cccwsr_acquisition_cc.h"
#include "pcps_quicksync_acquisition_cc.h"
#include "pcps_tong_acquisition_cc.h"
#include "signal_flag.h"
#include <boost/math/distributions/exponential.hpp>
#include <memory>
#include <stdexcept>

#if OPENCL_BLOCKS
#include "pcps_opencl_acquisition_cc.h"
#endif

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


namespace
{

class ThresholdComputeInterface
{
public:
    virtual float calculate_threshold(const Acq_Conf& acq_parameters) const = 0;
};

class ThresholdComputeBasic : public ThresholdComputeInterface
{
public:
    float calculate_threshold(const Acq_Conf& acq_parameters) const override;
};

class ThresholdComputeDoppler : public ThresholdComputeInterface
{
public:
    float calculate_threshold(const Acq_Conf& acq_parameters) const override;
};

class ThresholdComputeQuickSync : public ThresholdComputeInterface
{
public:
    float calculate_threshold(const Acq_Conf& acq_parameters) const override;
};

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

            const auto ncells = (acq_parameters.code_length / acq_parameters.folding_factor) * frequency_bins;
            const auto exponent = 1.0 / static_cast<double>(ncells);
            const auto val = pow(1.0 - acq_parameters.pfa, exponent);
            const auto lambda = static_cast<double>(acq_parameters.code_length) / static_cast<double>(acq_parameters.folding_factor);
            boost::math::exponential_distribution<double> mydist(lambda);
            const auto threshold = static_cast<float>(quantile(mydist, val));
            return threshold;
        }

    return acq_parameters.threshold;
}

const std::string default_dump_filename("./acquisition.dat");

Acq_Conf get_acq_conf(
    const ConfigurationInterface* configuration,
    const std::string& role,
    const std::string& implementation,
    signal_flag sig_flag)
{
    Acq_Conf acq_parameters;
    double chip_rate;
    double code_length_chips;
    uint32_t ms_per_code;

    switch (sig_flag)
        {
        case GPS_1C:
            chip_rate = GPS_L1_CA_CODE_RATE_CPS;
            code_length_chips = GPS_L1_CA_CODE_LENGTH_CHIPS;
            ms_per_code = GPS_L1_CA_CODE_PERIOD_MS;
            break;
        case GAL_1B:
            chip_rate = GALILEO_E1_CODE_CHIP_RATE_CPS;
            code_length_chips = GALILEO_E1_B_CODE_LENGTH_CHIPS;
            ms_per_code = GALILEO_E1_CODE_PERIOD_MS;
            acq_parameters.cboc = configuration->property(role + ".cboc", false);
            break;
        case GAL_E5a:
            chip_rate = GALILEO_E5A_CODE_CHIP_RATE_CPS;
            code_length_chips = GALILEO_E5A_CODE_LENGTH_CHIPS;
            ms_per_code = GALILEO_E5A_CODE_PERIOD_MS;
            break;
        default:
            throw std::runtime_error("Invalid signal");
        }

    std::unique_ptr<ThresholdComputeInterface> threshold_compute;
    std::optional<uint32_t> default_folding_factor;
    uint32_t max_sampled_ms = std::numeric_limits<uint32_t>::max();

    if (implementation == "GPS_L1_CA_PCPS_QuickSync_Acquisition")
        {
            const auto fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", static_cast<int64_t>(4000000));
            const auto fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
            const auto code_length = static_cast<unsigned int>(round(fs_in / (GPS_L1_CA_CODE_RATE_CPS / GPS_L1_CA_CODE_LENGTH_CHIPS)));
            default_folding_factor = static_cast<unsigned int>(ceil(sqrt(log2(code_length))));
        }
    else if (implementation == "Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition")
        {
            default_folding_factor = 2;
        }
    else if (implementation == "Galileo_E5a_Noncoherent_IQ_Acquisition_CAF")
        {
            acq_parameters.zero_padding = configuration->property(role + ".Zero_padding", 0);

            if (acq_parameters.zero_padding > 0)
                {
                    DLOG(INFO) << "Zero padding activated. Changing to 1ms code + 1ms zero padding ";
                    std::cout << "Zero padding activated. Changing to 1ms code + 1ms zero padding\n";
                    max_sampled_ms = 2;
                }

            max_sampled_ms = 3;
        }

    if (default_folding_factor.has_value())
        {
            acq_parameters.folding_factor = configuration->property(role + ".folding_factor", default_folding_factor.value());
            ms_per_code *= acq_parameters.folding_factor;
            threshold_compute = std::make_unique<ThresholdComputeQuickSync>();
        }
    else if (implementation == "GPS_L1_CA_PCPS_Assisted_Acquisition" ||
             implementation == "GPS_L1_CA_PCPS_Acquisition_Fine_Doppler" ||
             implementation == "Galileo_E1_PCPS_CCCWSR_Ambiguous_Acquisition")
        {
            threshold_compute = std::make_unique<ThresholdComputeBasic>();
        }
    else
        {
            threshold_compute = std::make_unique<ThresholdComputeDoppler>();
        }

    acq_parameters.ms_per_code = ms_per_code;
    acq_parameters.sampled_ms = ms_per_code;               // Set as default value
    acq_parameters.dump_filename = default_dump_filename;  // Set as default value
    acq_parameters.SetFromConfiguration(configuration, role, chip_rate, 0);

    if (implementation == "GPS_L1_CA_PCPS_Acquisition_Fine_Doppler")
        {
            acq_parameters.samples_per_ms = static_cast<float>(acq_parameters.vector_length);
        }

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
    acq_parameters.threshold = threshold_compute->calculate_threshold(acq_parameters);

    return acq_parameters;
}

bool use_stream_to_vector(const std::string& implementation)
{
    if (implementation == "GPS_L1_CA_PCPS_Acquisition_Fine_Doppler" ||
        implementation == "GPS_L1_CA_PCPS_Assisted_Acquisition" ||
        implementation == "Galileo_E5a_Noncoherent_IQ_Acquisition_CAF")
        {
            return false;
        }

    return true;
}

acquisition_impl_interface_sptr get_acquisition_impl(const ConfigurationInterface* configuration, const std::string& role, const std::string& implementation, const Acq_Conf& acq_parameters)
{
    if (implementation == "GPS_L1_CA_PCPS_Acquisition_Fine_Doppler")
        {
            return pcps_make_acquisition_fine_doppler_cc(acq_parameters);
        }
    else if (implementation == "GPS_L1_CA_PCPS_Assisted_Acquisition")
        {
            return pcps_make_assisted_acquisition_cc(acq_parameters);
        }
    else if (implementation == "GPS_L1_CA_PCPS_QuickSync_Acquisition" ||
             implementation == "Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition")
        {
            const unsigned int max_dwells = acq_parameters.bit_transition_flag ? 2 : acq_parameters.max_dwells;
            return pcps_quicksync_make_acquisition_cc(acq_parameters, acq_parameters.folding_factor, max_dwells);
        }
    else if (implementation == "GPS_L1_CA_PCPS_Tong_Acquisition" ||
             implementation == "Galileo_E1_PCPS_Tong_Ambiguous_Acquisition")
        {
            const auto tong_init_val = configuration->property(role + ".tong_init_val", 1U);
            const auto tong_max_val = configuration->property(role + ".tong_max_val", 2U);
            const auto tong_max_dwells = configuration->property(role + ".tong_max_dwells", tong_max_val + 1U);
            return pcps_tong_make_acquisition_cc(acq_parameters, tong_init_val, tong_max_val, tong_max_dwells);
        }
#if OPENCL_BLOCKS
    else if (implementation == "GPS_L1_CA_PCPS_OpenCl_Acquisition")
        {
            const unsigned int max_dwells = acq_parameters.bit_transition_flag ? 2 : acq_parameters.max_dwells;
            return pcps_make_opencl_acquisition_cc(acq_parameters, max_dwells);
        }
#endif
    else if (implementation == "Galileo_E1_PCPS_8ms_Ambiguous_Acquisition")
        {
            return galileo_pcps_8ms_make_acquisition_cc(acq_parameters);
        }
    else if (implementation == "Galileo_E1_PCPS_CCCWSR_Ambiguous_Acquisition")
        {
            return pcps_cccwsr_make_acquisition_cc(acq_parameters);
        }
    else if (implementation == "Galileo_E5a_Noncoherent_IQ_Acquisition_CAF")
        {
            const auto sig = configuration->property("Channel.signal", std::string("5X"));
            const auto both_signal_components = (sig.at(0) == '5' && sig.at(1) == 'X');
            const auto caf_window_hz = configuration->property(role + ".CAF_window_hz", 0);
            return galileo_e5a_noncoherentIQ_make_acquisition_caf_cc(acq_parameters, both_signal_components, caf_window_hz, acq_parameters.zero_padding);
        }

    return nullptr;
}


}  // namespace


PcpsAcquisitionAdapterCustom::PcpsAcquisitionAdapterCustom(
    const ConfigurationInterface* configuration,
    const std::string& role,
    const std::string& implementation,
    unsigned int in_streams,
    unsigned int out_streams,
    signal_flag sig_flag)
    : acq_parameters_(get_acq_conf(configuration, role, implementation, sig_flag)),
      acquisition_cc_(get_acquisition_impl(configuration, role, implementation, acq_parameters_)),
      gnss_synchro_(nullptr),
      channel_(0),
      code_(acq_parameters_.vector_length),
      sig_flag_(sig_flag),
      role_(role),
      implementation_(implementation),
      is_type_gr_complex_(acq_parameters_.item_type == "gr_complex"),
      item_size_(is_type_gr_complex_ ? sizeof(gr_complex) : 0),
      use_stream_to_vector_(use_stream_to_vector(implementation))
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


void PcpsAcquisitionAdapterCustom::connect(gr::top_block_sptr top_block)
{
    if (is_type_gr_complex_ && use_stream_to_vector_)
        {
            top_block->connect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
}


void PcpsAcquisitionAdapterCustom::disconnect(gr::top_block_sptr top_block)
{
    if (is_type_gr_complex_ && use_stream_to_vector_)
        {
            top_block->disconnect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
}


gr::basic_block_sptr PcpsAcquisitionAdapterCustom::get_left_block()
{
    if (use_stream_to_vector_)
        {
            return stream_to_vector_;
        }
    return acquisition_cc_;
}


gr::basic_block_sptr PcpsAcquisitionAdapterCustom::get_right_block()
{
    return acquisition_cc_;
}


void PcpsAcquisitionAdapterCustom::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;

    if (is_type_gr_complex_)
        {
            acquisition_cc_->set_gnss_synchro(gnss_synchro);
        }
}

void PcpsAcquisitionAdapterCustom::set_channel(unsigned int channel)
{
    channel_ = channel;

    if (is_type_gr_complex_)
        {
            acquisition_cc_->set_channel(channel);
        }
}


void PcpsAcquisitionAdapterCustom::set_channel_fsm(std::weak_ptr<ChannelFsm> channel_fsm)
{
    if (is_type_gr_complex_)
        {
            acquisition_cc_->set_channel_fsm(channel_fsm);
        }
}


signed int PcpsAcquisitionAdapterCustom::mag()
{
    if (is_type_gr_complex_)
        {
            return acquisition_cc_->mag();
        }
    return 0;
}


void PcpsAcquisitionAdapterCustom::reset()
{
    if (is_type_gr_complex_)
        {
            acquisition_cc_->set_active(true);
        }
}


void PcpsAcquisitionAdapterCustom::stop_acquisition()
{
    if (is_type_gr_complex_)
        {
            acquisition_cc_->set_active(false);
        }
}


void PcpsAcquisitionAdapterCustom::set_local_code()
{
    if (is_type_gr_complex_)
        {
            if (implementation_ == "Galileo_E1_PCPS_CCCWSR_Ambiguous_Acquisition")
                {
                    std::vector<std::complex<float>> code_pilot_(acq_parameters_.vector_length);
                    auto& code_data_ = code_;

                    std::array<char, 3> signal = {{'1', 'B', '\0'}};
                    galileo_e1_code_gen_complex_sampled(code_data_, signal, acq_parameters_.cboc, gnss_synchro_->PRN, acq_parameters_.fs_in, 0, false);

                    std::array<char, 3> signal_C = {{'1', 'C', '\0'}};
                    galileo_e1_code_gen_complex_sampled(code_pilot_, signal_C, acq_parameters_.cboc, gnss_synchro_->PRN, acq_parameters_.fs_in, 0, false);

                    acquisition_cc_->set_local_code(code_data_.data(), code_pilot_.data());
                }
            else if (implementation_ == "Galileo_E5a_Noncoherent_IQ_Acquisition_CAF")
                {
                    const auto code_length = acq_parameters_.code_length;
                    const auto vector_length = acq_parameters_.vector_length;

                    auto& codeI_ = code_;
                    std::vector<std::complex<float>> codeQ_(code_length);
                    std::vector<std::complex<float>> codeI(code_length);
                    std::vector<std::complex<float>> codeQ(code_length);

                    if (gnss_synchro_->Signal[0] == '5' && gnss_synchro_->Signal[1] == 'X')
                        {
                            std::array<char, 3> a = {{'5', 'I', '\0'}};
                            galileo_e5_a_code_gen_complex_sampled(codeI, gnss_synchro_->PRN, a, acq_parameters_.fs_in, 0);

                            std::array<char, 3> b = {{'5', 'Q', '\0'}};
                            galileo_e5_a_code_gen_complex_sampled(codeQ, gnss_synchro_->PRN, b, acq_parameters_.fs_in, 0);
                        }
                    else
                        {
                            std::array<char, 3> signal_type_ = {{'5', 'X', '\0'}};
                            galileo_e5_a_code_gen_complex_sampled(codeI, gnss_synchro_->PRN, signal_type_, acq_parameters_.fs_in, 0);
                        }
                    // WARNING: 3ms are coherently integrated. Secondary sequence (1,1,1)
                    // is generated, and modulated in the 'block'.
                    own::span<gr_complex> codeI_span(codeI_.data(), vector_length);
                    own::span<gr_complex> codeQ_span(codeQ_.data(), vector_length);
                    if (acq_parameters_.zero_padding == 0)  // if no zero_padding
                        {
                            for (unsigned int i = 0; i < acq_parameters_.sampled_ms; i++)
                                {
                                    std::copy_n(codeI.data(), code_length, codeI_span.subspan(i * code_length, code_length).data());
                                    if (gnss_synchro_->Signal[0] == '5' && gnss_synchro_->Signal[1] == 'X')
                                        {
                                            std::copy_n(codeQ.data(), code_length, codeQ_span.subspan(i * code_length, code_length).data());
                                        }
                                }
                        }
                    else
                        {
                            // 1ms code + 1ms zero padding
                            std::copy_n(codeI.data(), code_length, codeI_.data());
                            if (gnss_synchro_->Signal[0] == '5' && gnss_synchro_->Signal[1] == 'X')
                                {
                                    std::copy_n(codeQ.data(), code_length, codeQ_.data());
                                }
                        }

                    acquisition_cc_->set_local_code(codeI_.data(), codeQ_.data());
                }
            else
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
}

void PcpsAcquisitionAdapterCustom::code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    switch (sig_flag_)
        {
        case GPS_1C:
            gps_l1_ca_code_gen_complex_sampled(dest, prn, sampling_freq, 0);
            break;
        case GAL_1B:
            {
                std::array<char, 3> Signal_{};
                Signal_[0] = gnss_synchro_->Signal[0];
                Signal_[1] = gnss_synchro_->Signal[1];
                Signal_[2] = '\0';

                galileo_e1_code_gen_complex_sampled(dest, Signal_, acq_parameters_.cboc, prn, sampling_freq, 0, false);
            }
            break;
        default:
            throw std::runtime_error("Invalid signal");
        }
}
