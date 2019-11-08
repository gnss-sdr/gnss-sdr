/*!
 * \file gps_l5_dll_pll_tracking.cc
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for GPS L5 to a TrackingInterface
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "gps_l5_dll_pll_tracking.h"
#include "GPS_L5.h"
#include "configuration_interface.h"
#include "display.h"
#include "dll_pll_conf.h"
#include "gnss_sdr_flags.h"
#include <glog/logging.h>
#include <array>

GpsL5DllPllTracking::GpsL5DllPllTracking(
    ConfigurationInterface* configuration, const std::string& role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    Dll_Pll_Conf trk_param = Dll_Pll_Conf();
    DLOG(INFO) << "role " << role;
    //################# CONFIGURATION PARAMETERS ########################
    trk_param.SetFromConfiguration(configuration, role);
    if (trk_param.smoother_length < 1)
        {
            trk_param.smoother_length = 1;
            std::cout << TEXT_RED << "WARNING: GPS L5. smoother_length must be bigger than 0. It has been set to 1" << TEXT_RESET << std::endl;
        }
    if (FLAGS_pll_bw_hz != 0.0) trk_param.pll_bw_hz = static_cast<float>(FLAGS_pll_bw_hz);
    if (FLAGS_dll_bw_hz != 0.0) trk_param.dll_bw_hz = static_cast<float>(FLAGS_dll_bw_hz);
    if (trk_param.extend_correlation_symbols < 1)
        {
            trk_param.extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: GPS L5. extend_correlation_symbols must be bigger than 0. Coherent integration has been set to 1 symbol (1 ms)" << TEXT_RESET << std::endl;
        }
    else if (!trk_param.track_pilot and trk_param.extend_correlation_symbols > GPS_L5I_NH_CODE_LENGTH)
        {
            trk_param.extend_correlation_symbols = GPS_L5I_NH_CODE_LENGTH;
            std::cout << TEXT_RED << "WARNING: GPS L5. extend_correlation_symbols must be lower than 11 when tracking the data component. Coherent integration has been set to 10 symbols (10 ms)" << TEXT_RESET << std::endl;
        }
    if ((trk_param.extend_correlation_symbols > 1) and (trk_param.pll_bw_narrow_hz > trk_param.pll_bw_hz or trk_param.dll_bw_narrow_hz > trk_param.dll_bw_hz))
        {
            std::cout << TEXT_RED << "WARNING: GPS L5. PLL or DLL narrow tracking bandwidth is higher than wide tracking one" << TEXT_RESET << std::endl;
        }
    int vector_length = std::round(static_cast<double>(trk_param.fs_in) / (GPS_L5I_CODE_RATE_CPS / static_cast<double>(GPS_L5I_CODE_LENGTH_CHIPS)));
    trk_param.vector_length = vector_length;
    trk_param.very_early_late_space_chips = 0.0;
    trk_param.very_early_late_space_narrow_chips = 0.0;
    trk_param.system = 'G';
    std::array<char, 3> sig_{'L', '5', '\0'};
    std::memcpy(trk_param.signal, sig_.data(), 3);


    //################# MAKE TRACKING GNURadio object ###################
    if (trk_param.item_type != "fc32")
        {
            LOG(WARNING) << trk_param.item_type << " unknown tracking item type changing to gr_complex";
            trk_param.item_type = "fc32";
        }
    tracking_ = dll_pll_veml_make_tracking(trk_param);
    channel_ = 0;
    DLOG(INFO) << "tracking(" << tracking_->unique_id() << ")";
    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void GpsL5DllPllTracking::stop_tracking()
{
    tracking_->stop_tracking();
}


void GpsL5DllPllTracking::start_tracking()
{
    tracking_->start_tracking();
}


/*
 * Set tracking channel unique ID
 */
void GpsL5DllPllTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_->set_channel(channel);
}


void GpsL5DllPllTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_->set_gnss_synchro(p_gnss_synchro);
}


void GpsL5DllPllTracking::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to connect, now the tracking uses gr_sync_decimator
}


void GpsL5DllPllTracking::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr GpsL5DllPllTracking::get_left_block()
{
    return tracking_;
}


gr::basic_block_sptr GpsL5DllPllTracking::get_right_block()
{
    return tracking_;
}
