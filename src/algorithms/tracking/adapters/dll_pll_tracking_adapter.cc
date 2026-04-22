/*!
 * \file dll_pll_tracking_adapter.cc
 * \brief Base class providing shared logic for DLL+PLL VEML tracking adapters.
 * \authors Carles Fernandez, 2025. carles.fernandez(at)cttc.cat
 *          Mathieu Favreau, 2026. favreau.mathieu(at)hotmail.com
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

#include "dll_pll_tracking_adapter.h"
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
#include "configuration_interface.h"
#include "display.h"
#include "qzss.h"

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

namespace
{

struct signal_info
{
    char system;
    std::array<char, 3> sig;
    std::string sig_name;
    double code_chip_rate;
    double code_length_chips;
    int32_t max_extend_correlation_symbols;
    bool has_pilot;
    bool disable_track_pilot;
};

signal_info get_signal_info(signal_flag sig_flag)
{
    switch (sig_flag)
        {
        case GPS_1C:
            return {'G', {'1', 'C', '\0'}, "GPS L1 C/A", GPS_L1_CA_CODE_RATE_CPS, GPS_L1_CA_CODE_LENGTH_CHIPS, 20, false, true};
        case GPS_2S:
            return {'G', {'2', 'S', '\0'}, "GPS L2", GPS_L2_M_CODE_RATE_CPS, GPS_L2_M_CODE_LENGTH_CHIPS, 1, false, true};
        case GPS_L5:
            return {'G', {'L', '5', '\0'}, "GPS L5", GPS_L5I_CODE_RATE_CPS, GPS_L5I_CODE_LENGTH_CHIPS, GPS_L5I_NH_CODE_LENGTH, true, false};
        case GAL_1B:
            return {'E', {'1', 'B', '\0'}, "Galileo E1", GALILEO_E1_CODE_CHIP_RATE_CPS, GALILEO_E1_B_CODE_LENGTH_CHIPS, 1, true, false};
        case GAL_E5a:
            return {'E', {'5', 'X', '\0'}, "Galileo E5a", GALILEO_E5A_CODE_CHIP_RATE_CPS, GALILEO_E5A_CODE_LENGTH_CHIPS, GALILEO_E5A_I_SECONDARY_CODE_LENGTH, true, false};
        case GAL_E5b:
            return {'E', {'7', 'X', '\0'}, "Galileo E5b", GALILEO_E5B_CODE_CHIP_RATE_CPS, GALILEO_E5B_CODE_LENGTH_CHIPS, GALILEO_E5B_I_SECONDARY_CODE_LENGTH, true, false};
        case GAL_E6:
            return {'E', {'E', '6', '\0'}, "Galileo E6", GALILEO_E6_B_CODE_CHIP_RATE_CPS, GALILEO_E6_B_CODE_LENGTH_CHIPS, 1, true, false};
        case GLO_1G:
            return {'R', {'1', 'G', '\0'}, "Glonass L1", GLONASS_L1_CA_CODE_RATE_CPS, GLONASS_L1_CA_CODE_LENGTH_CHIPS, 10, false, true};
        case GLO_2G:
            return {'R', {'2', 'G', '\0'}, "Glonass L2", GLONASS_L2_CA_CODE_RATE_CPS, GLONASS_L2_CA_CODE_LENGTH_CHIPS, 10, false, true};
        case BDS_B1:
            return {'C', {'B', '1', '\0'}, "BEIDOU B1I", BEIDOU_B1I_CODE_RATE_CPS, BEIDOU_B1I_CODE_LENGTH_CHIPS, 20, false, true};
        case BDS_B3:
            return {'C', {'B', '3', '\0'}, "BEIDOU B3I", BEIDOU_B3I_CODE_RATE_CPS, BEIDOU_B3I_CODE_LENGTH_CHIPS, 20, false, false};  // Does false, false make sense?
        case QZS_J1:
            return {'J', {'J', '1', '\0'}, "QZSS L1 C/A", QZSS_L1_CHIP_RATE, QZSS_L1_CODE_LENGTH, 20, false, true};
        case QZS_J5:
            return {'J', {'J', '5', '\0'}, "QZSS L5", QZSS_L5_CHIP_RATE, QZSS_L5_CODE_LENGTH, QZSS_L5I_NH_CODE_LENGTH, true, false};
        default:
            break;
        }

    return {};
}

void check_and_configure_trk_params(const ConfigurationInterface* configuration, const std::string& role, const signal_info& sig_info, Dll_Pll_Conf& trk_params)
{
    trk_params.SetFromConfiguration(configuration, role);
    trk_params.vector_length = static_cast<int>(std::round(trk_params.fs_in / (sig_info.code_chip_rate / sig_info.code_length_chips)));
    trk_params.system = sig_info.system;
    std::copy_n(sig_info.sig.data(), 3, trk_params.signal);
    trk_params.track_pilot = configuration->property(role + ".track_pilot", sig_info.has_pilot);

    const bool track_data = !sig_info.has_pilot || !trk_params.track_pilot;

    if (trk_params.extend_correlation_symbols < 1)
        {
            trk_params.extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: " << sig_info.sig_name << ": extend_correlation_symbols must be bigger than 0. Coherent integration has been set to 1 symbol";
        }
    else if (track_data && trk_params.extend_correlation_symbols > sig_info.max_extend_correlation_symbols)
        {
            trk_params.extend_correlation_symbols = sig_info.max_extend_correlation_symbols;
            std::cout << TEXT_RED << "WARNING: " << sig_info.sig_name;

            if (sig_info.max_extend_correlation_symbols == 1)
                {
                    if (sig_info.has_pilot)
                        {
                            std::cout << ": extended coherent integration is not allowed when tracking the data component. Coherent integration has been set to 1 symbol";
                        }
                    else
                        {
                            std::cout << ": extended coherent integration is not allowed. Coherent integration has been set to 1 symbol";
                        }
                }
            else
                {
                    if (sig_info.has_pilot)
                        {
                            std::cout << ": extend_correlation_symbols limited to " << sig_info.max_extend_correlation_symbols
                                      << " symbols when tracking the data component. Coherent integration has been set to " << sig_info.max_extend_correlation_symbols << " symbols";
                        }
                    else
                        {
                            std::cout << ": extend_correlation_symbols limited to " << sig_info.max_extend_correlation_symbols
                                      << " symbols. Coherent integration has been set to " << sig_info.max_extend_correlation_symbols << " symbols";
                        }
                }

            std::cout << TEXT_RESET << '\n';
        }

    if (sig_info.disable_track_pilot && trk_params.track_pilot)
        {
            std::cout << TEXT_RED << "WARNING: " << sig_info.sig_name << " does not have pilot signal. Data tracking has been enabled instead" << TEXT_RESET << '\n';
            trk_params.track_pilot = false;
        }

    if ((trk_params.extend_correlation_symbols > 1) && (trk_params.pll_bw_narrow_hz > trk_params.pll_bw_hz || trk_params.dll_bw_narrow_hz > trk_params.dll_bw_hz))
        {
            std::cout << TEXT_RED << "WARNING: " << sig_info.sig_name << ". PLL or DLL narrow tracking bandwidth is higher than wide tracking one" << TEXT_RESET << '\n';
        }
}

}  // namespace

DllPllTrackingAdapter::DllPllTrackingAdapter(
    const ConfigurationInterface* configuration,
    std::string role,
    std::string implementation,
    unsigned int in_streams,
    unsigned int out_streams,
    signal_flag sig_flag)
    : role_(std::move(role)),
      implementation_(std::move(implementation)),
      item_size_(sizeof(gr_complex))
{
    DLOG(INFO) << "role " << role_;

    const auto sig_info = get_signal_info(sig_flag);

    if (!sig_info.sig_name.empty())
        {
            check_and_configure_trk_params(configuration, role_, sig_info, trk_params_);

            if (trk_params_.item_type == "gr_complex")
                {
                    tracking_sptr_ = dll_pll_veml_make_tracking(trk_params_);
                    DLOG(INFO) << "Tracking block (" << tracking_sptr_->unique_id() << ")";
                }
            else
                {
                    item_size_ = 0;
                    LOG(WARNING) << trk_params_.item_type << " unknown tracking item type.";
                }
        }
    else
        {
            item_size_ = 0;
            LOG(ERROR) << "Invalid signal in DLL PLL Tracking Adapter.";
        }


    if (in_streams > 1)
        {
            LOG(ERROR) << "Only one input stream is supported.";
        }
    if (out_streams > 1)
        {
            LOG(ERROR) << "Only one output stream is supported.";
        }
}


void DllPllTrackingAdapter::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* no connection needed */
        }
}


void DllPllTrackingAdapter::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* no disconnection needed */
        }
}


gr::basic_block_sptr DllPllTrackingAdapter::get_left_block()
{
    return tracking_sptr_;
}


gr::basic_block_sptr DllPllTrackingAdapter::get_right_block()
{
    return tracking_sptr_;
}


void DllPllTrackingAdapter::set_channel(unsigned int channel)
{
    tracking_sptr_->set_channel(channel);
}


void DllPllTrackingAdapter::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_sptr_->set_gnss_synchro(p_gnss_synchro);
}


void DllPllTrackingAdapter::start_tracking()
{
    tracking_sptr_->start_tracking();
}


void DllPllTrackingAdapter::stop_tracking()
{
    tracking_sptr_->stop_tracking();
}
