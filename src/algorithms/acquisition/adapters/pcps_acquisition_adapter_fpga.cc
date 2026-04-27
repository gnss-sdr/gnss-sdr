/*!
 * \file pcps_acquisition_adapter_fpga.cc
 * \brief Adapts an FPGA-offloaded PCPS acquisition block to an AcquisitionInterface
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


#include "pcps_acquisition_adapter_fpga.h"
#include "GPS_L1_CA.h"
#include "GPS_L2C.h"
#include "GPS_L5.h"
#include "Galileo_E1.h"
#include "Galileo_E5a.h"
#include "Galileo_E5b.h"
#include "configuration_interface.h"
#include "galileo_e1_signal_replica.h"
#include "galileo_e5_signal_replica.h"
#include "gnss_sdr_fft.h"
#include "gnss_sdr_flags.h"
#include "gps_l2c_signal_replica.h"
#include "gps_l5_signal_replica.h"
#include "gps_sdr_signal_replica.h"
#include <volk/volk.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <iostream>
#include <utility>
#include <vector>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

namespace
{
constexpr uint32_t QUANT_BITS_LOCAL_CODE = 16;
constexpr uint32_t SELECT_LSBITS = 0x0000FFFF;
constexpr uint32_t SELECT_MSBITS = 0xFFFF0000;
constexpr uint32_t SELECT_ALL_CODE_BITS = 0xFFFFFFFF;
constexpr uint32_t SHL_CODE_BITS = 65536;
constexpr uint32_t ACQ_BUFF_0 = 0;
constexpr uint32_t ACQ_BUFF_1 = 1;

struct fpga_signal_info
{
    double code_rate_cps;
    double code_length_chips;
    uint32_t opt_acq_fs_sps;
    uint32_t default_fpga_blk_exp;
    uint32_t acq_buff;
    uint32_t num_prns;
};


fpga_signal_info get_fpga_signal_info(signal_flag sig_flag)
{
    switch (sig_flag)
        {
        case GPS_1C:
            return {GPS_L1_CA_CODE_RATE_CPS, GPS_L1_CA_CODE_LENGTH_CHIPS, GPS_L1_CA_OPT_ACQ_FS_SPS, 10, ACQ_BUFF_0, 32};
        case GPS_2S:
            return {GPS_L2_M_CODE_RATE_CPS, GPS_L2_M_CODE_LENGTH_CHIPS, 0, 13, ACQ_BUFF_1, 32};
        case GPS_L5:
            return {GPS_L5I_CODE_RATE_CPS, GPS_L5I_CODE_LENGTH_CHIPS, 0, 13, ACQ_BUFF_1, 32};
        case GAL_1B:
            return {GALILEO_E1_CODE_CHIP_RATE_CPS, GALILEO_E1_B_CODE_LENGTH_CHIPS, GALILEO_E1_OPT_ACQ_FS_SPS, 13, ACQ_BUFF_0, GALILEO_E1_NUMBER_OF_CODES};
        case GAL_E5a:
            return {GALILEO_E5A_CODE_CHIP_RATE_CPS, GALILEO_E5A_CODE_LENGTH_CHIPS, 0, 13, ACQ_BUFF_1, GALILEO_E5A_NUMBER_OF_CODES};
        case GAL_E5b:
            return {GALILEO_E5B_CODE_CHIP_RATE_CPS, GALILEO_E5B_CODE_LENGTH_CHIPS, 0, 13, ACQ_BUFF_1, GALILEO_E5B_NUMBER_OF_CODES};
        default:
            break;
        }

    return {};
}


void pack_fft_codes(const std::complex<float>* fft_code, uint32_t nsamples_total, uint32_t prn, volk_gnsssdr::vector<uint32_t>& all_fft_codes)
{
    float max = 0;
    for (uint32_t i = 0; i < nsamples_total; i++)
        {
            if (std::abs(fft_code[i].real()) > max)
                {
                    max = std::abs(fft_code[i].real());
                }
            if (std::abs(fft_code[i].imag()) > max)
                {
                    max = std::abs(fft_code[i].imag());
                }
        }

    for (uint32_t i = 0; i < nsamples_total; i++)
        {
            const auto tmp = static_cast<int32_t>(std::floor(fft_code[i].real() * (std::pow(2, QUANT_BITS_LOCAL_CODE - 1) - 1) / max));
            const auto tmp2 = static_cast<int32_t>(std::floor(fft_code[i].imag() * (std::pow(2, QUANT_BITS_LOCAL_CODE - 1) - 1) / max));
            const auto local_code = (tmp & SELECT_LSBITS) | ((tmp2 * SHL_CODE_BITS) & SELECT_MSBITS);
            all_fft_codes[i + (nsamples_total * (prn - 1))] = local_code & SELECT_ALL_CODE_BITS;
        }
}


void generate_code(signal_flag sig_flag,
    const ConfigurationInterface* configuration,
    const std::string& role,
    const Acq_Conf_Fpga& acq_parameters,
    uint32_t prn,
    volk_gnsssdr::vector<std::complex<float>>& code)
{
    switch (sig_flag)
        {
        case GPS_1C:
            gps_l1_ca_code_gen_complex_sampled(code, prn, acq_parameters.resampled_fs, 0);
            break;
        case GPS_2S:
            gps_l2c_m_code_gen_complex_sampled(code, prn, acq_parameters.fs_in);
            break;
        case GPS_L5:
            gps_l5i_code_gen_complex_sampled(code, prn, acq_parameters.fs_in);
            break;
        case GAL_1B:
            {
                const bool acquire_pilot = configuration->property(role + ".acquire_pilot", false);
                const bool cboc = false;
                const auto signal = acquire_pilot ? std::array<char, 3>{'1', 'C', '\0'} : std::array<char, 3>{'1', 'B', '\0'};
                galileo_e1_code_gen_complex_sampled(code, signal, cboc, prn, acq_parameters.resampled_fs, 0, false);
            }
            break;
        case GAL_E5a:
            {
                const bool acquire_iq = configuration->property(role + ".acquire_iq", false);
                const bool acquire_pilot = acquire_iq ? false : configuration->property(role + ".acquire_pilot", false);
                std::array<char, 3> signal = {'5', 'I', '\0'};
                if (acquire_iq)
                    {
                        signal[1] = 'X';
                    }
                else if (acquire_pilot)
                    {
                        signal[1] = 'Q';
                    }
                galileo_e5_a_code_gen_complex_sampled(code, prn, signal, acq_parameters.fs_in, 0);
            }
            break;
        case GAL_E5b:
            {
                const bool acquire_iq = configuration->property(role + ".acquire_iq", false);
                const bool acquire_pilot = acquire_iq ? false : configuration->property(role + ".acquire_pilot", false);
                std::array<char, 3> signal = {'7', 'I', '\0'};
                if (acquire_iq)
                    {
                        signal[1] = 'X';
                    }
                else if (acquire_pilot)
                    {
                        signal[1] = 'Q';
                    }
                galileo_e5_b_code_gen_complex_sampled(code, prn, signal, acq_parameters.fs_in, 0);
            }
            break;
        default:
            break;
        }
}
}  // namespace


PcpsAcquisitionAdapterFpga::PcpsAcquisitionAdapterFpga(
    const ConfigurationInterface* configuration,
    const std::string& role,
    const std::string& implementation,
    unsigned int in_streams,
    unsigned int out_streams,
    signal_flag sig_flag) : sig_flag_(sig_flag),
                            role_(role),
                            implementation_(implementation)
{
    if (in_streams > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams > 0)
        {
            LOG(ERROR) << "This implementation does not provide an output stream";
        }

    const auto sig_info = get_fpga_signal_info(sig_flag_);
    acq_parameters_.SetFromConfiguration(configuration, role_, sig_info.default_fpga_blk_exp, sig_info.code_rate_cps, sig_info.code_length_chips);

    std::vector<std::pair<uint32_t, uint32_t>> downsampling_filter_specs;
    uint32_t max_FFT_size;
    acquisition_fpga_ = pcps_make_acquisition_fpga(&acq_parameters_, sig_info.acq_buff, downsampling_filter_specs, max_FFT_size);

    if (sig_info.acq_buff == ACQ_BUFF_0)
        {
            const bool acq_configuration_valid = acq_parameters_.ConfigureAutomaticResampler(downsampling_filter_specs, max_FFT_size, sig_info.opt_acq_fs_sps);
            if (!acq_configuration_valid)
                {
                    std::cout << "The FPGA acquisition IP does not support the required sampling frequency of " << acq_parameters_.fs_in << " SPS for the L1/E1 band. Please update the sampling frequency in the configuration file." << std::endl;
                    std::exit(0);
                }
        }
    else
        {
            const bool acq_configuration_valid = acq_parameters_.Is_acq_config_valid(max_FFT_size);
            if (!acq_configuration_valid)
                {
                    std::cout << "The FPGA acquisition IP does not support the required sampling frequency of " << acq_parameters_.fs_in << " SPS for the L2/L5 band. Please update the sampling frequency in the configuration file." << std::endl;
                    std::exit(0);
                }
        }

#if USE_GLOG_AND_GFLAGS
    if (FLAGS_doppler_max != 0)
        {
            acq_parameters_.doppler_max = FLAGS_doppler_max;
        }
    if (FLAGS_doppler_step != 0)
        {
            acq_parameters_.doppler_step = static_cast<float>(FLAGS_doppler_step);
        }
#else
    if (absl::GetFlag(FLAGS_doppler_max) != 0)
        {
            acq_parameters_.doppler_max = absl::GetFlag(FLAGS_doppler_max);
        }
    if (absl::GetFlag(FLAGS_doppler_step) != 0)
        {
            acq_parameters_.doppler_step = static_cast<float>(absl::GetFlag(FLAGS_doppler_step));
        }
#endif

    generate_prn_codes(configuration);
    init();
    DLOG(INFO) << "Initialized FPGA acquisition adapter for role " << role_;
}


void PcpsAcquisitionAdapterFpga::generate_prn_codes(const ConfigurationInterface* configuration)
{
    const auto sig_info = get_fpga_signal_info(sig_flag_);
    const uint32_t code_length = acq_parameters_.code_length;
    const uint32_t nsamples_total = acq_parameters_.fft_size;

    auto fft_if = gnss_fft_fwd_make_unique(nsamples_total);
    volk_gnsssdr::vector<std::complex<float>> code(nsamples_total);
    volk_gnsssdr::vector<std::complex<float>> fft_code(nsamples_total);
    d_all_fft_codes_ = volk_gnsssdr::vector<uint32_t>(nsamples_total * sig_info.num_prns);

    for (uint32_t prn = 1; prn <= sig_info.num_prns; prn++)
        {
            generate_code(sig_flag_, configuration, role_, acq_parameters_, prn, code);

            if (acq_parameters_.enable_zero_padding)
                {
                    std::copy(code.begin(), code.begin() + code_length, code.begin() + code_length);
                    std::fill(code.begin() + (2 * code_length), code.end(), std::complex<float>(0.0, 0.0));
                }

            std::copy_n(code.data(), nsamples_total, fft_if->get_inbuf());
            fft_if->execute();
            volk_32fc_conjugate_32fc(fft_code.data(), fft_if->get_outbuf(), nsamples_total);
            pack_fft_codes(fft_code.data(), nsamples_total, prn, d_all_fft_codes_);
        }

    acq_parameters_.all_fft_codes = d_all_fft_codes_.data();
}


void PcpsAcquisitionAdapterFpga::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        {
            // FPGA acquisition blocks do not connect to GNU Radio flowgraphs.
        }
}


void PcpsAcquisitionAdapterFpga::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        {
            // No GNU Radio connections to remove.
        }
}


gr::basic_block_sptr PcpsAcquisitionAdapterFpga::get_left_block()
{
    return nullptr;
}


gr::basic_block_sptr PcpsAcquisitionAdapterFpga::get_right_block()
{
    return nullptr;
}


void PcpsAcquisitionAdapterFpga::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    if (acquisition_fpga_)
        {
            acquisition_fpga_->set_gnss_synchro(gnss_synchro);
        }
}


void PcpsAcquisitionAdapterFpga::set_channel(unsigned int channel)
{
    if (acquisition_fpga_)
        {
            acquisition_fpga_->set_channel(channel);
        }
}


void PcpsAcquisitionAdapterFpga::set_channel_fsm(std::weak_ptr<ChannelFsm> channel_fsm)
{
    if (acquisition_fpga_)
        {
            acquisition_fpga_->set_channel_fsm(std::move(channel_fsm));
        }
}


void PcpsAcquisitionAdapterFpga::set_doppler_center(int doppler_center)
{
    if (acquisition_fpga_)
        {
            acquisition_fpga_->set_doppler_center(doppler_center);
        }
}


void PcpsAcquisitionAdapterFpga::reset()
{
    if (acquisition_fpga_)
        {
            acquisition_fpga_->set_active(true);
        }
}


void PcpsAcquisitionAdapterFpga::stop_acquisition()
{
    if (acquisition_fpga_)
        {
            acquisition_fpga_->stop_acquisition();
        }
}


void PcpsAcquisitionAdapterFpga::init()
{
    if (acquisition_fpga_)
        {
            acquisition_fpga_->init();
        }
}


signed int PcpsAcquisitionAdapterFpga::mag()
{
    if (acquisition_fpga_)
        {
            return acquisition_fpga_->mag();
        }
    return 0;
}


void PcpsAcquisitionAdapterFpga::set_local_code()
{
    if (acquisition_fpga_)
        {
            acquisition_fpga_->set_local_code();
        }
}
