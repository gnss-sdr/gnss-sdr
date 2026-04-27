/*!
 * \file dll_pll_tracking_adapter_fpga.cc
 * \brief Adapts an FPGA-offloaded DLL/PLL tracking block to a TrackingInterface
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


#include "dll_pll_tracking_adapter_fpga.h"
#include "GPS_L1_CA.h"
#include "GPS_L2C.h"
#include "GPS_L5.h"
#include "Galileo_E1.h"
#include "Galileo_E5a.h"
#include "configuration_interface.h"
#include "display.h"
#include "galileo_e1_signal_replica.h"
#include "galileo_e5_signal_replica.h"
#include "gps_l2c_signal_replica.h"
#include "gps_l5_signal_replica.h"
#include "gps_sdr_signal_replica.h"
#include "uio_fpga.h"
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

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
constexpr int32_t LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY = 0x0C000000;
constexpr int32_t LOCAL_CODE_FPGA_CORRELATOR_SELECT_COUNT = 0x20000000;
constexpr uint32_t GPS_NUM_PRNS = 32;
constexpr int32_t GPS_CA_BIT_DURATION_MS = 20;

struct tracking_fpga_signal_info
{
    char system;
    std::array<char, 3> signal;
    std::string map_signal;
    double code_rate_cps;
    double code_length_chips;
    uint32_t num_prns;
    uint32_t code_samples_per_chip;
};


tracking_fpga_signal_info get_tracking_fpga_signal_info(signal_flag sig_flag)
{
    switch (sig_flag)
        {
        case GPS_1C:
            return {'G', {'1', 'C', '\0'}, "1C", GPS_L1_CA_CODE_RATE_CPS, GPS_L1_CA_CODE_LENGTH_CHIPS, GPS_NUM_PRNS, 1};
        case GPS_2S:
            return {'G', {'2', 'S', '\0'}, "2S", GPS_L2_M_CODE_RATE_CPS, GPS_L2_M_CODE_LENGTH_CHIPS, GPS_NUM_PRNS, 1};
        case GPS_L5:
            return {'G', {'L', '5', '\0'}, "L5", GPS_L5I_CODE_RATE_CPS, GPS_L5I_CODE_LENGTH_CHIPS, GPS_NUM_PRNS, 1};
        case GAL_1B:
            return {'E', {'1', 'B', '\0'}, "1B", GALILEO_E1_CODE_CHIP_RATE_CPS, GALILEO_E1_B_CODE_LENGTH_CHIPS, GALILEO_E1_NUMBER_OF_CODES, 2};
        case GAL_E5a:
            return {'E', {'5', 'X', '\0'}, "5X", GALILEO_E5A_CODE_CHIP_RATE_CPS, GALILEO_E5A_CODE_LENGTH_CHIPS, GALILEO_E5A_NUMBER_OF_CODES, 1};
        default:
            break;
        }

    return {};
}


void warn_narrow_bw(const std::string& signal_name, const Dll_Pll_Conf_Fpga& conf)
{
    if ((conf.extend_correlation_symbols > 1) &&
        (conf.pll_bw_narrow_hz > conf.pll_bw_hz ||
            conf.dll_bw_narrow_hz > conf.dll_bw_hz))
        {
            std::cout << TEXT_RED
                      << "WARNING: " << signal_name << ". PLL/DLL narrow bandwidth is greater than wide one."
                      << TEXT_RESET << '\n';
        }
}
}  // namespace


DllPllTrackingAdapterFpga::DllPllTrackingAdapterFpga(const ConfigurationInterface* configuration,
    const std::string& role,
    const std::string& implementation,
    unsigned int in_streams,
    unsigned int out_streams,
    signal_flag sig_flag)
    : prn_codes_ptr_(nullptr),
      data_codes_ptr_(nullptr),
      sig_flag_(sig_flag),
      role_(role),
      implementation_(implementation),
      channel_(0),
      signal_base_channel_index_(0)
{
    trk_params_.SetFromConfiguration(configuration, role_);
    if (in_streams > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }

    configure_signal_parameters(configuration);
    generate_prn_codes();
    tracking_fpga_sc_sptr_ = dll_pll_veml_make_tracking_fpga(trk_params_);
    DLOG(INFO) << "tracking(" << tracking_fpga_sc_sptr_->unique_id() << ")";
}


DllPllTrackingAdapterFpga::~DllPllTrackingAdapterFpga()
{
    volk_gnsssdr_free(prn_codes_ptr_);
    if (data_codes_ptr_ != nullptr)
        {
            volk_gnsssdr_free(data_codes_ptr_);
        }
}


void DllPllTrackingAdapterFpga::configure_signal_parameters(const ConfigurationInterface* configuration)
{
    const auto sig_info = get_tracking_fpga_signal_info(sig_flag_);
    trk_params_.vector_length = static_cast<uint32_t>(
        std::round(static_cast<double>(trk_params_.fs_in) /
                   (sig_info.code_rate_cps / sig_info.code_length_chips)));

    switch (sig_flag_)
        {
        case GPS_1C:
            if (trk_params_.extend_correlation_symbols < 1)
                {
                    trk_params_.extend_correlation_symbols = 1;
                    std::cout << TEXT_RED
                              << "WARNING: GPS L1 C/A. extend_correlation_symbols must be >= 1. Set to 1 ms."
                              << TEXT_RESET << '\n';
                }
            else if (trk_params_.extend_correlation_symbols > GPS_CA_BIT_DURATION_MS)
                {
                    trk_params_.extend_correlation_symbols = GPS_CA_BIT_DURATION_MS;
                    std::cout << TEXT_RED
                              << "WARNING: GPS L1 C/A. extend_correlation_symbols limited to 20 ms."
                              << TEXT_RESET << '\n';
                }
            trk_params_.track_pilot = false;
            break;
        case GPS_2S:
            trk_params_.extend_correlation_symbols = configuration->property(role_ + ".extend_correlation_symbols", 1);
            if (trk_params_.extend_correlation_symbols != 1)
                {
                    trk_params_.extend_correlation_symbols = 1;
                    std::cout << TEXT_RED
                              << "WARNING: Extended coherent integration is not supported in GPS L2. "
                              << "Coherent integration set to 20 ms (1 symbol)."
                              << TEXT_RESET << '\n';
                }
            trk_params_.track_pilot = configuration->property(role_ + ".track_pilot", false);
            if (trk_params_.track_pilot)
                {
                    trk_params_.track_pilot = false;
                    std::cout << TEXT_RED
                              << "WARNING: GPS L2M does not have a pilot signal. Data tracking enabled."
                              << TEXT_RESET << '\n';
                }
            break;
        case GPS_L5:
            if (trk_params_.extend_correlation_symbols < 1)
                {
                    trk_params_.extend_correlation_symbols = 1;
                    std::cout << TEXT_RED
                              << "WARNING: GPS L5. extend_correlation_symbols must be >= 1. "
                              << "Coherent integration set to 1 symbol (1 ms)."
                              << TEXT_RESET << '\n';
                }
            else if (!trk_params_.track_pilot &&
                     trk_params_.extend_correlation_symbols > GPS_L5I_NH_CODE_LENGTH)
                {
                    trk_params_.extend_correlation_symbols = GPS_L5I_NH_CODE_LENGTH;
                    std::cout << TEXT_RED
                              << "WARNING: GPS L5. extend_correlation_symbols must be <= 10 when tracking "
                              << "the data component. Set to 10 symbols (10 ms)."
                              << TEXT_RESET << '\n';
                }
            warn_narrow_bw("GPS L5", trk_params_);
            break;
        case GAL_1B:
            if (trk_params_.extend_correlation_symbols < 1)
                {
                    trk_params_.extend_correlation_symbols = 1;
                    std::cout << TEXT_RED
                              << "WARNING: Galileo E1. extend_correlation_symbols must be >= 1. "
                              << "Set to 1 symbol (4 ms)." << TEXT_RESET << '\n';
                }
            else if (!trk_params_.track_pilot && trk_params_.extend_correlation_symbols > 1)
                {
                    trk_params_.extend_correlation_symbols = 1;
                    std::cout << TEXT_RED
                              << "WARNING: Galileo E1. Extended coherent integration is only supported "
                              << "when tracking the pilot component. Set to 4 ms (1 symbol)."
                              << TEXT_RESET << '\n';
                }
            warn_narrow_bw("Galileo E1", trk_params_);
            break;
        case GAL_E5a:
            if (trk_params_.extend_correlation_symbols < 1)
                {
                    trk_params_.extend_correlation_symbols = 1;
                    std::cout << TEXT_RED
                              << "WARNING: Galileo E5a. extend_correlation_symbols must be >= 1. "
                              << "Coherent integration set to 1 symbol (1 ms)."
                              << TEXT_RESET << '\n';
                }
            else if (!trk_params_.track_pilot &&
                     trk_params_.extend_correlation_symbols > GALILEO_E5A_I_SECONDARY_CODE_LENGTH)
                {
                    trk_params_.extend_correlation_symbols = GALILEO_E5A_I_SECONDARY_CODE_LENGTH;
                    std::cout << TEXT_RED
                              << "WARNING: Galileo E5a. extend_correlation_symbols cannot exceed 20 "
                              << "when tracking the data component. Set to 20 ms (20 symbols)."
                              << TEXT_RESET << '\n';
                }
            warn_narrow_bw("Galileo E5a", trk_params_);
            break;
        default:
            break;
        }

    trk_params_.system = sig_info.system;
    std::copy_n(sig_info.signal.data(), 3, trk_params_.signal);
    configure_fpga_tracking_channel_mapping(sig_info.map_signal);

    trk_params_.code_length_chips = static_cast<uint32_t>(sig_info.code_length_chips);
    trk_params_.code_samples_per_chip = sig_info.code_samples_per_chip;
    trk_params_.extended_correlation_in_fpga = false;
    trk_params_.extend_fpga_integration_periods = 1;
    trk_params_.fpga_integration_period = 1;

    if (sig_flag_ == GPS_1C &&
        trk_params_.extend_correlation_symbols > 1 &&
        trk_params_.extend_correlation_symbols <= GPS_CA_BIT_DURATION_MS &&
        (GPS_CA_BIT_DURATION_MS % trk_params_.extend_correlation_symbols) == 0)
        {
            trk_params_.extended_correlation_in_fpga = true;
            trk_params_.fpga_integration_period = trk_params_.extend_correlation_symbols;
        }

    if ((sig_flag_ == GPS_L5 || sig_flag_ == GAL_E5a) && trk_params_.track_pilot)
        {
            const auto secondary_code_length = (sig_flag_ == GPS_L5) ? GPS_L5I_NH_CODE_LENGTH : GALILEO_E5A_I_SECONDARY_CODE_LENGTH;
            const auto extend = trk_params_.extend_correlation_symbols;
            if (extend > 1)
                {
                    if (extend <= secondary_code_length && (secondary_code_length % extend) == 0)
                        {
                            trk_params_.extended_correlation_in_fpga = true;
                            trk_params_.fpga_integration_period = extend;
                        }
                    else if (extend % secondary_code_length == 0)
                        {
                            trk_params_.extended_correlation_in_fpga = true;
                            trk_params_.extend_fpga_integration_periods = extend / secondary_code_length;
                            trk_params_.fpga_integration_period = secondary_code_length;
                        }
                }
        }
}


void DllPllTrackingAdapterFpga::generate_prn_codes()
{
    const auto sig_info = get_tracking_fpga_signal_info(sig_flag_);
    const auto code_length_chips = static_cast<uint32_t>(sig_info.code_length_chips);
    const auto code_samples_per_chip = sig_info.code_samples_per_chip;
    const auto code_samples = code_length_chips * code_samples_per_chip;

    prn_codes_ptr_ = static_cast<int32_t*>(
        volk_gnsssdr_malloc(code_samples * sig_info.num_prns * sizeof(int32_t),
            volk_gnsssdr_get_alignment()));

    if (trk_params_.track_pilot && (sig_flag_ == GAL_1B || sig_flag_ == GAL_E5a || sig_flag_ == GPS_L5))
        {
            data_codes_ptr_ = static_cast<int32_t*>(
                volk_gnsssdr_malloc(code_samples * sig_info.num_prns * sizeof(int32_t),
                    volk_gnsssdr_get_alignment()));
        }

    switch (sig_flag_)
        {
        case GPS_1C:
            for (uint32_t prn = 1; prn <= sig_info.num_prns; ++prn)
                {
                    gps_l1_ca_code_gen_int(
                        own::span<int32_t>(&prn_codes_ptr_[static_cast<size_t>(code_length_chips) * (prn - 1)],
                            &prn_codes_ptr_[static_cast<size_t>(code_length_chips * prn)]),
                        prn, 0);

                    for (uint32_t k = 0; k < code_length_chips; ++k)
                        {
                            auto tmp = prn_codes_ptr_[static_cast<size_t>(code_length_chips) * (prn - 1) + k];
                            if (tmp < 0)
                                {
                                    tmp = 0;
                                }
                            tmp |= LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                            prn_codes_ptr_[static_cast<size_t>(code_length_chips) * (prn - 1) + k] = tmp;
                        }
                }
            break;
        case GPS_2S:
            {
                volk_gnsssdr::vector<float> ca_codes_f(code_length_chips, 0.0F);
                for (uint32_t prn = 1; prn <= sig_info.num_prns; ++prn)
                    {
                        gps_l2c_m_code_gen_float(ca_codes_f, prn);
                        for (uint32_t s = 0; s < code_length_chips; ++s)
                            {
                                int32_t bit_val = (ca_codes_f[s] < 0.0F) ? 0 : 1;
                                bit_val |= LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                                prn_codes_ptr_[code_length_chips * (prn - 1) + s] = bit_val;
                            }
                    }
            }
            break;
        case GPS_L5:
            {
                volk_gnsssdr::vector<float> tracking_code(code_length_chips, 0.0F);
                volk_gnsssdr::vector<float> data_code(code_length_chips, 0.0F);
                for (uint32_t prn = 1; prn <= sig_info.num_prns; ++prn)
                    {
                        if (trk_params_.track_pilot)
                            {
                                gps_l5q_code_gen_float(tracking_code, prn);
                                gps_l5i_code_gen_float(data_code, prn);
                            }
                        else
                            {
                                gps_l5i_code_gen_float(tracking_code, prn);
                            }

                        for (uint32_t s = 0; s < code_length_chips; ++s)
                            {
                                int32_t tracking_val = (tracking_code[s] < 0.0F) ? 0 : 1;
                                tracking_val |= LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                                prn_codes_ptr_[code_length_chips * (prn - 1) + s] = tracking_val;

                                if (trk_params_.track_pilot)
                                    {
                                        int32_t data_val = (data_code[s] < 0.0F) ? 0 : 1;
                                        data_val |= LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY |
                                                    LOCAL_CODE_FPGA_CORRELATOR_SELECT_COUNT;
                                        data_codes_ptr_[code_length_chips * (prn - 1) + s] = data_val;
                                    }
                            }
                    }
            }
            break;
        case GAL_1B:
            {
                volk_gnsssdr::vector<float> ca_codes_f(code_samples);
                volk_gnsssdr::vector<float> data_codes_f(code_samples);
                for (uint32_t prn = 1; prn <= sig_info.num_prns; prn++)
                    {
                        if (trk_params_.track_pilot)
                            {
                                galileo_e1_code_gen_sinboc11_float(ca_codes_f, {'1', 'C', '\0'}, prn);
                                galileo_e1_code_gen_sinboc11_float(data_codes_f, {'1', 'B', '\0'}, prn);
                            }
                        else
                            {
                                galileo_e1_code_gen_sinboc11_float(ca_codes_f, {'1', 'B', '\0'}, prn);
                            }

                        for (uint32_t s = 0; s < code_samples; s++)
                            {
                                int32_t pilot = (ca_codes_f[s] < 0) ? 0 : 1;
                                pilot |= LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                                prn_codes_ptr_[code_samples * (prn - 1) + s] = pilot;

                                if (trk_params_.track_pilot)
                                    {
                                        int32_t data = (data_codes_f[s] < 0) ? 0 : 1;
                                        data |= LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY |
                                                LOCAL_CODE_FPGA_CORRELATOR_SELECT_COUNT;
                                        data_codes_ptr_[code_samples * (prn - 1) + s] = data;
                                    }
                            }
                    }
            }
            break;
        case GAL_E5a:
            {
                volk_gnsssdr::vector<gr_complex> aux_code(code_length_chips, gr_complex(0.0F, 0.0F));
                for (uint32_t prn = 1; prn <= sig_info.num_prns; ++prn)
                    {
                        galileo_e5_a_code_gen_complex_primary(aux_code, prn, {'5', 'X', '\0'});
                        for (uint32_t s = 0; s < code_length_chips; ++s)
                            {
                                const auto sample = trk_params_.track_pilot ? aux_code[s].imag() : aux_code[s].real();
                                int32_t val = (sample < 0.0F) ? 0 : 1;
                                val |= LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                                prn_codes_ptr_[code_length_chips * (prn - 1) + s] = val;

                                if (trk_params_.track_pilot)
                                    {
                                        int32_t data_val = (aux_code[s].real() < 0.0F) ? 0 : 1;
                                        data_val |= LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY |
                                                    LOCAL_CODE_FPGA_CORRELATOR_SELECT_COUNT;
                                        data_codes_ptr_[code_length_chips * (prn - 1) + s] = data_val;
                                    }
                            }
                    }
            }
            break;
        default:
            break;
        }

    trk_params_.ca_codes = prn_codes_ptr_;
    trk_params_.data_codes = data_codes_ptr_;
}


void DllPllTrackingAdapterFpga::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        {
            // FPGA tracking block connections are handled internally.
        }
}


void DllPllTrackingAdapterFpga::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        {
            // No GNU Radio connections to remove.
        }
}


gr::basic_block_sptr DllPllTrackingAdapterFpga::get_left_block()
{
    return tracking_fpga_sc_sptr_;
}


gr::basic_block_sptr DllPllTrackingAdapterFpga::get_right_block()
{
    return tracking_fpga_sc_sptr_;
}


void DllPllTrackingAdapterFpga::start_tracking()
{
    tracking_fpga_sc_sptr_->start_tracking();
}


void DllPllTrackingAdapterFpga::stop_tracking()
{
    tracking_fpga_sc_sptr_->stop_tracking();
}


void DllPllTrackingAdapterFpga::configure_fpga_tracking_channel_mapping(const std::string& signal)
{
    signal_ = signal;
    device_name_ = signal_to_device_.at(signal_);

    std::lock_guard<std::mutex> lock(channel_counts_mtx_);
    auto it = channel_counts_.find(signal_);
    if (it != channel_counts_.end())
        {
            it->second++;
        }
    else
        {
            channel_counts_[signal_] = 1;
        }

    set_signal_channel_base_index_locked();
}


void DllPllTrackingAdapterFpga::set_signal_channel_base_index_locked()
{
    uint32_t signal_base_channel_index = 0;
    for (const auto& [label, num_correlators] : channel_counts_)
        {
            if (label != signal_)
                {
                    const auto it = signal_to_device_.find(label);
                    if (it != signal_to_device_.end() && it->second != device_name_)
                        {
                            signal_base_channel_index += num_correlators;
                        }
                }
        }

    signal_base_channel_index -= get_num_alternative_devices_locked();
    signal_base_channel_index_ = signal_base_channel_index;
}


uint32_t DllPllTrackingAdapterFpga::get_num_alternative_devices_locked() const
{
    uint32_t num_alternative_devices = 0;
    for (const auto& [signal_type, alternative_device_name] : signal_to_alternative_device_)
        {
            if (alternative_device_name == device_name_)
                {
                    uint32_t num_channels = 0;
                    auto it = channel_counts_.find(signal_type);
                    if (it != channel_counts_.end())
                        {
                            num_channels = it->second;
                            const auto device_name = signal_to_device_.at(signal_type);
                            const auto num_devices_available = get_num_devices(device_name);
                            if (num_devices_available < num_channels)
                                {
                                    num_alternative_devices += num_channels - num_devices_available;
                                }
                        }
                }
        }

    return num_alternative_devices;
}


void DllPllTrackingAdapterFpga::set_channel(unsigned int channel)
{
    channel_ = channel;
    std::string device_io_name;

    if (find_uio_dev_file_name(device_io_name, device_name_, channel_ - signal_base_channel_index_) >= 0)
        {
            tracking_fpga_sc_sptr_->set_channel(channel_, device_io_name);
            return;
        }

    auto it = signal_to_alternative_device_.find(signal_);
    if (it != signal_to_alternative_device_.end())
        {
            const auto alternate_device_channel_index = channel_ - signal_base_channel_index_ - get_num_devices(it->second);
            if (find_uio_dev_file_name(device_io_name, it->second, alternate_device_channel_index) >= 0)
                {
                    tracking_fpga_sc_sptr_->set_channel(channel_, device_io_name);
                    return;
                }
        }

    std::cerr << "Cannot map an FPGA tracking multicorrelator device to channel " << channel_ << std::endl;
    std::cerr << "Exiting the program.\n";
    std::exit(0);
}


void DllPllTrackingAdapterFpga::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_fpga_sc_sptr_->set_gnss_synchro(p_gnss_synchro);
}
