/*!
 * \file gnss_flowgraph.h
 * \brief Interface of a GNSS receiver flowgraph.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *
 * It contains a signal source,
 * a signal conditioner, a set of channels, a pvt and an output filter.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_GNSS_FLOWGRAPH_H_
#define GNSS_SDR_GNSS_FLOWGRAPH_H_

#include <string>
#include <vector>
#include <queue>
#include <gnuradio/gr_top_block.h>
#include <gnuradio/gr_msg_queue.h>
#include "GPS_L1_CA.h"
#include "gnss_signal.h"

class GNSSBlockInterface;
class ChannelInterface;
class ConfigurationInterface;
class GNSSBlockFactory;

/*! \brief This class represents a GNSS flowgraph.
 *
 * It contains a signal source,
 * a signal conditioner, a set of channels, a PVT and an output filter.
 */
class GNSSFlowgraph
{
public:
    /*!
     * \brief Constructor that initializes the receiver flowgraph
     */
    GNSSFlowgraph(ConfigurationInterface* configuration,
            gr_msg_queue_sptr queue);

    /*!
     * \brief Virtual destructor
     */
    virtual ~GNSSFlowgraph();

    //! \brief Start the flowgraph
    void start();

    //! \brief Stop the flowgraph
    void stop();

    /*!
     * \brief Connects the defined blocks in the flowgraph
     *
     * Signal Source > Signal conditioner > Channels >> Observables >> PVT > Output filter
     */
    void connect();

    void wait();

    /*!
     * \brief Applies an action to the flowgraph
     *
     * \param[in] who   Who generated the action
     * \param[in] what  What is the action 0: acquisition failed
     */
    void apply_action(unsigned int who, unsigned int what);

    void set_configuration(ConfigurationInterface* configuration);

    GNSSBlockInterface* signal_source();
    GNSSBlockInterface* signal_conditioner();
    ChannelInterface* channel(unsigned int index);
    GNSSBlockInterface* observables();
    GNSSBlockInterface* pvt();
    GNSSBlockInterface* output_filter();

    unsigned int applied_actions()
    {
        return applied_actions_;
    }
    bool connected()
    {
        return connected_;
    }
    bool running()
    {
        return running_;
    }

private:
    void init();
    /*!
     * \brief Populates the SV PRN list available for acquisition and tracking
     */
    void set_signals_list();
    /*!
     * \brief Initializes the channels state (start acquisition or keep stanby) using the configuration parameters (number of channels and max channels in acquisition)
     */
    void set_channels_state();
    bool connected_;
    bool running_;
    unsigned int channels_count_;
    unsigned int acq_channels_count_;
    unsigned int max_acq_channels_;
    unsigned int applied_actions_;
    std::string config_file_;
    ConfigurationInterface *configuration_;
    GNSSBlockFactory *block_factory_;
    std::vector<GNSSBlockInterface*>* blocks_;
    gr_top_block_sptr top_block_;
    gr_msg_queue_sptr queue_;
    std::list<Gnss_Signal> available_GNSS_signals_;
    std::vector<unsigned int> channels_state_;
};

#endif /*GNSS_SDR_GNSS_FLOWGRAPH_H_*/
