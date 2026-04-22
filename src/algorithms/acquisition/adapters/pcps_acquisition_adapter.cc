/*!
 * \file base_ca_pcps_acquisition.h
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

#include "pcps_acquisition_adapter.h"
#include "Beidou_B1I.h"
#include "Beidou_B3I.h"
#include "GLONASS_L1_L2_CA.h"
#include "GPS_L1_CA.h"
#include "GPS_L2C.h"
#include "GPS_L5.h"
#include "Galileo_E1.h"
#include "Galileo_E5a.h"
#include "Galileo_E5b.h"
#include "Galileo_E6.h"
#include "acq_conf.h"
#include "beidou_b1i_signal_replica.h"
#include "beidou_b3i_signal_replica.h"
#include "configuration_interface.h"
#include "galileo_e1_signal_replica.h"
#include "galileo_e5_signal_replica.h"
#include "galileo_e6_signal_replica.h"
#include "glonass_l1_signal_replica.h"
#include "glonass_l2_signal_replica.h"
#include "gnss_sdr_flags.h"
#include "gps_l2c_signal_replica.h"
#include "gps_l5_signal_replica.h"
#include "gps_sdr_signal_replica.h"
#include "qzss.h"
#include "qzss_signal_replica.h"
#include "signal_flag.h"

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

struct signal_info
{
    double chip_rate;
    double opt_freq;
    double code_length_chips;
    uint32_t ms_per_code;
};


signal_info get_signal_info(signal_flag sig_flag)
{
    switch (sig_flag)
        {
        case GPS_1C:
            return {GPS_L1_CA_CODE_RATE_CPS, GPS_L1_CA_OPT_ACQ_FS_SPS, GPS_L1_CA_CODE_LENGTH_CHIPS, GPS_L1_CA_CODE_PERIOD_MS};
        case GPS_2S:
            return {GPS_L2_M_CODE_RATE_CPS, GPS_L2C_OPT_ACQ_FS_SPS, GPS_L2_M_CODE_LENGTH_CHIPS, GPS_L2_M_CODE_PERIOD_MS};
        case GPS_L5:
            return {GPS_L5I_CODE_RATE_CPS, GPS_L5_OPT_ACQ_FS_SPS, GPS_L5I_CODE_LENGTH_CHIPS, GPS_L5I_PERIOD_MS};
        case GAL_1B:
            return {GALILEO_E1_CODE_CHIP_RATE_CPS, GALILEO_E1_OPT_ACQ_FS_SPS, GALILEO_E1_B_CODE_LENGTH_CHIPS, GALILEO_E1_CODE_PERIOD_MS};
        case GAL_E5a:
            return {GALILEO_E5A_CODE_CHIP_RATE_CPS, GALILEO_E5A_OPT_ACQ_FS_SPS, GALILEO_E5A_CODE_LENGTH_CHIPS, GALILEO_E5A_CODE_PERIOD_MS};
        case GAL_E5b:
            return {GALILEO_E5B_CODE_CHIP_RATE_CPS, GALILEO_E5B_OPT_ACQ_FS_SPS, GALILEO_E5B_CODE_LENGTH_CHIPS, GALILEO_E5B_CODE_PERIOD_MS};
        case GAL_E6:
            return {GALILEO_E6_B_CODE_CHIP_RATE_CPS, GALILEO_E6_OPT_ACQ_FS_SPS, GALILEO_E6_B_CODE_LENGTH_CHIPS, GALILEO_E6_CODE_PERIOD_MS};
        case GLO_1G:
            return {GLONASS_L1_CA_CODE_RATE_CPS, 100e6, GLONASS_L1_CA_CODE_LENGTH_CHIPS, GLONASS_L1_CA_CODE_PERIOD_MS};
        case GLO_2G:
            return {GLONASS_L2_CA_CODE_RATE_CPS, 100e6, GLONASS_L2_CA_CODE_LENGTH_CHIPS, GLONASS_L2_CA_CODE_PERIOD_MS};
        case BDS_B1:
            return {BEIDOU_B1I_CODE_RATE_CPS, BEIDOU_B1I_OPT_ACQ_FS_SPS, BEIDOU_B1I_CODE_LENGTH_CHIPS, BEIDOU_B1I_CODE_PERIOD_MS};
        case BDS_B3:
            return {BEIDOU_B3I_CODE_RATE_CPS, BEIDOU_B3I_OPT_ACQ_FS_SPS, BEIDOU_B3I_CODE_LENGTH_CHIPS, BEIDOU_B3I_CODE_PERIOD_MS};
        case QZS_J1:
            return {QZSS_L1_CHIP_RATE, QZSS_L1_OPT_ACQ_FS_SPS, QZSS_L1_CODE_LENGTH, QZSS_L1_PERIOD_MS};
        case QZS_J5:
            return {QZSS_L5_CHIP_RATE, QZSS_L5_OPT_ACQ_FS_SPS, QZSS_L5_CODE_LENGTH, QZSS_L5I_PERIOD_MS};
        default:
            break;
        }

    return {};
}


void code_gen_complex_sampled(signal_flag sig_flag, const Acq_Conf& conf, const Gnss_Synchro& gnss_synchro, own::span<std::complex<float>> dest, int32_t sampling_freq)
{
    switch (sig_flag)
        {
        case GPS_1C:
            gps_l1_ca_code_gen_complex_sampled(dest, gnss_synchro.PRN, sampling_freq, 0);
            break;
        case GPS_2S:
            gps_l2c_m_code_gen_complex_sampled(dest, gnss_synchro.PRN, sampling_freq);
            break;
        case GPS_L5:
            gps_l5i_code_gen_complex_sampled(dest, gnss_synchro.PRN, sampling_freq);
            break;
        case GAL_1B:
            {
                const auto sig = conf.acquire_pilot ? std::array<char, 3>{'1', 'C', '\0'} : std::array<char, 3>{gnss_synchro.Signal[0], gnss_synchro.Signal[1], '\0'};
                galileo_e1_code_gen_complex_sampled(dest, sig, conf.cboc, gnss_synchro.PRN, sampling_freq, 0, false);
            }
            break;
        case GAL_E5a:
            {
                std::array<char, 3> signal_{'5', 'I', '\0'};

                if (conf.acquire_iq)
                    {
                        signal_[1] = 'X';
                    }
                else if (conf.acquire_pilot)
                    {
                        signal_[1] = 'Q';
                    }

                galileo_e5_a_code_gen_complex_sampled(dest, gnss_synchro.PRN, signal_, sampling_freq, 0);
            }
            break;
        case GAL_E5b:
            {
                std::array<char, 3> signal_{'7', 'I', '\0'};

                if (conf.acquire_iq)
                    {
                        signal_[1] = 'X';
                    }
                else if (conf.acquire_pilot)
                    {
                        signal_[1] = 'Q';
                    }

                galileo_e5_b_code_gen_complex_sampled(dest, gnss_synchro.PRN, signal_, sampling_freq, 0);
            }
            break;
        case GAL_E6:
            galileo_e6_b_code_gen_complex_sampled(dest, gnss_synchro.PRN, sampling_freq, 0);
            break;
        case GLO_1G:
            glonass_l1_ca_code_gen_complex_sampled(dest, sampling_freq, 0);
            break;
        case GLO_2G:
            glonass_l2_ca_code_gen_complex_sampled(dest, sampling_freq, 0);
            break;
        case BDS_B1:
            beidou_b1i_code_gen_complex_sampled(dest, gnss_synchro.PRN, sampling_freq, 0);
            break;
        case BDS_B3:
            beidou_b3i_code_gen_complex_sampled(dest, gnss_synchro.PRN, sampling_freq, 0);
            break;
        case QZS_J1:
            qzss_l1_code_gen_complex_sampled(dest, gnss_synchro.PRN, sampling_freq);
            break;
        case QZS_J5:
            qzss_l5i_code_gen_complex_sampled(dest, gnss_synchro.PRN, sampling_freq);
            break;
        default:
            break;
        }
}


Acq_Conf get_acq_conf(const ConfigurationInterface* configuration, const std::string& role, signal_flag sig_flag)
{
    const auto sig_info = get_signal_info(sig_flag);

    Acq_Conf acq_parameters;
    acq_parameters.ms_per_code = sig_info.ms_per_code;
    acq_parameters.sampled_ms = sig_info.ms_per_code;  // Set as default value
    acq_parameters.SetFromConfiguration(configuration, role, sig_info.chip_rate, sig_info.opt_freq);

    if (sig_flag == GAL_1B)
        {
            acq_parameters.acquire_pilot = configuration->property(role + ".acquire_pilot", acq_parameters.acquire_pilot);
            acq_parameters.cboc = configuration->property(role + ".cboc", acq_parameters.cboc);
        }

    if (sig_flag == GAL_E5a || sig_flag == GAL_E5b)
        {
            acq_parameters.acquire_pilot = configuration->property(role + ".acquire_pilot", acq_parameters.acquire_pilot);
            acq_parameters.acquire_iq = configuration->property(role + ".acquire_iq", acq_parameters.acquire_iq);
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

    acq_parameters.vector_length = std::floor(acq_parameters.sampled_ms * acq_parameters.samples_per_ms) * (acq_parameters.bit_transition_flag ? 2.0 : 1.0);
    acq_parameters.code_length = static_cast<unsigned int>(std::floor(static_cast<double>(acq_parameters.resampled_fs) / (static_cast<double>(sig_info.chip_rate) / sig_info.code_length_chips)));

    return acq_parameters;
}
}  // namespace

PcpsAcquisitionAdapter::PcpsAcquisitionAdapter(
    const ConfigurationInterface* configuration,
    const std::string& role,
    const std::string& implementation,
    unsigned int in_streams,
    unsigned int out_streams,
    signal_flag sig_flag) : acq_parameters_(get_acq_conf(configuration, role, sig_flag)),
                            sig_flag_(sig_flag),
                            role_(role),
                            implementation_(implementation),
                            gnss_synchro_(nullptr),
                            code_(acq_parameters_.vector_length),
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


void PcpsAcquisitionAdapter::stop_acquisition()
{
    acquisition_->set_active(false);
}


void PcpsAcquisitionAdapter::set_doppler_center(int doppler_center)
{
    acquisition_->set_doppler_center(doppler_center);
}


void PcpsAcquisitionAdapter::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    acquisition_->set_gnss_synchro(gnss_synchro_);
}


signed int PcpsAcquisitionAdapter::mag()
{
    return acquisition_->mag();
}


void PcpsAcquisitionAdapter::reset()
{
    acquisition_->set_active(true);
}


void PcpsAcquisitionAdapter::connect(gr::top_block_sptr top_block)
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


void PcpsAcquisitionAdapter::disconnect(gr::top_block_sptr top_block)
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


gr::basic_block_sptr PcpsAcquisitionAdapter::get_left_block()
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


gr::basic_block_sptr PcpsAcquisitionAdapter::get_right_block()
{
    return acquisition_;
}


void PcpsAcquisitionAdapter::set_resampler_latency(uint32_t latency_samples)
{
    acquisition_->set_resampler_latency(latency_samples);
}


void PcpsAcquisitionAdapter::set_local_code()
{
    volk_gnsssdr::vector<std::complex<float>> code(acq_parameters_.code_length);

    const auto sampling_freq = acq_parameters_.use_automatic_resampler ? acq_parameters_.resampled_fs : acq_parameters_.fs_in;
    code_gen_complex_sampled(sig_flag_, acq_parameters_, *gnss_synchro_, code, sampling_freq);

    const auto num_codes = acq_parameters_.sampled_ms / acq_parameters_.ms_per_code;

    own::span<gr_complex> code_span(code_.data(), acq_parameters_.vector_length);
    for (unsigned int i = 0; i < num_codes; i++)
        {
            std::copy_n(code.data(), acq_parameters_.code_length, code_span.subspan(i * acq_parameters_.code_length, acq_parameters_.code_length).data());
        }

    acquisition_->set_local_code(code_.data());
}
