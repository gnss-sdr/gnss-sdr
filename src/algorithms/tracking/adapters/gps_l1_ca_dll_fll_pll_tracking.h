/*!
 * \file gps_l1_ca_dll_fll_pll_tracking.h
 * \brief Interface of an adapter of a code DLL + carrier FLL/PLL tracking
 * loop for GPS L1 C/A to a TrackingInterface
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * This is the interface of a code Delay Locked Loop (DLL) + carrier
 * Phase Locked Loop (PLL) helped with a carrier Frequency Locked Loop (FLL)
 * according to the algorithms described in:
 * E.D. Kaplan and C. Hegarty, Understanding GPS. Principles and
 * Applications, Second Edition, Artech House Publishers, 2005.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GPS_L1_CA_DLL_FLL_PLL_TRACKING_H_
#define GNSS_SDR_GPS_L1_CA_DLL_FLL_PLL_TRACKING_H_

#include <string>
#include <gnuradio/msg_queue.h>
#include "tracking_interface.h"
#include "gps_l1_ca_dll_fll_pll_tracking_cc.h"


class ConfigurationInterface;

/*!
 * \brief This class implements a code DLL + carrier PLL/FLL Assisted tracking loop
 */
class GpsL1CaDllFllPllTracking : public TrackingInterface
{
public:
  GpsL1CaDllFllPllTracking(ConfigurationInterface* configuration,
            std::string role,
            unsigned int in_streams,
            unsigned int out_streams,
            boost::shared_ptr<gr::msg_queue> queue);

    virtual ~GpsL1CaDllFllPllTracking();

    std::string role()
    {
        return role_;
    }

    //! Returns "GPS_L1_CA_DLL_FLL_PLL_Tracking"
    std::string implementation()
    {
        return "GPS_L1_CA_DLL_FLL_PLL_Tracking";
    }
    size_t item_size()
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block);
    void disconnect(gr::top_block_sptr top_block);
    gr::basic_block_sptr get_left_block();
    gr::basic_block_sptr get_right_block();

    void set_channel(unsigned int channel);
    void set_channel_queue(concurrent_queue<int> *channel_internal_queue);
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro);
    void start_tracking();

private:
    gps_l1_ca_dll_fll_pll_tracking_cc_sptr tracking_;
    size_t item_size_;
    unsigned int channel_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    boost::shared_ptr<gr::msg_queue> queue_;
    concurrent_queue<int> *channel_internal_queue_;
};

#endif // GPS_L1_CA_DLL_FLL_PLL_TRACKING_H_
