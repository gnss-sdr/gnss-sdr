/*!
 * \file galileo_e5a_dll_fll_pll_tracking_cc.h
 * \brief Adapts code DLL + carrier PLL aided with FLL
 *  tracking block to TrackingInterface for Galileo E5a signals
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2014  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#ifndef GNSS_SDR_GALILEO_E5A_DLL_FLL_PLL_TRACKING_H_
#define GNSS_SDR_GALILEO_E5A_DLL_FLL_PLL_TRACKING_H_

#include <string>
#include <gnuradio/msg_queue.h>
#include "tracking_interface.h"
#include "galileo_e5a_dll_fll_pll_tracking_cc.h"

class ConfigurationInterface;

class GalileoE5aDllFllPllTracking: public TrackingInterface
{
public:

    GalileoE5aDllFllPllTracking(ConfigurationInterface* configuration,
            std::string role,
            unsigned int in_streams,
            unsigned int out_streams,
            boost::shared_ptr<gr::msg_queue> queue);

    virtual ~GalileoE5aDllFllPllTracking();

    std::string role()
    {
        return role_;
    }

    //! Returns "Galileo_E5a_DLL_FLL_PLL_Tracking"
    std::string implementation()
    {
        return "Galileo_E5a_DLL_FLL_PLL_Tracking";
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
    galileo_e5a_dll_fll_pll_tracking_cc_sptr tracking_;
    size_t item_size_;
    unsigned int channel_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    boost::shared_ptr<gr::msg_queue> queue_;
    concurrent_queue<int> *channel_internal_queue_;
};

#endif /* GNSS_SDR_GALILEO_E5A_DLL_FLL_PLL_TRACKING_H_ */
