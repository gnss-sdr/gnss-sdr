/*!
 * \file gnss_flowgraph.h
 * \brief Interface of a GNSS receiver flowgraph.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *         Carles Fernandez-Prades, 2014. cfernandez(at)cttc.es
 *
 * It contains a signal source,
 * a signal conditioner, a set of channels, a pvt and an output filter.
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

#ifndef GNSS_SDR_GNSS_FLOWGRAPH_H_
#define GNSS_SDR_GNSS_FLOWGRAPH_H_

#include <list>
#include <memory>
#include <queue>
#include <string>
#include <vector>
#include <gnuradio/top_block.h>
#include <gnuradio/msg_queue.h>
#include "GPS_L1_CA.h"
#include "gnss_signal.h"
#include "gnss_sdr_sample_counter.h"

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
    GNSSFlowgraph(std::shared_ptr<ConfigurationInterface> configuration, gr::msg_queue::sptr queue);

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

    void set_configuration(std::shared_ptr<ConfigurationInterface> configuration);

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
    /*!
     * \brief Sends a GNURadio asyncronous message from telemetry to PVT
     *
     * It is used to assist the receiver with external ephemeris data
     */
    bool send_telemetry_msg(pmt::pmt_t msg);
private:
    void init(); // Populates the SV PRN list available for acquisition and tracking
    void set_signals_list();
    void set_channels_state(); // Initializes the channels state (start acquisition or keep standby)
                               // using the configuration parameters (number of channels and max channels in acquisition)
    Gnss_Signal search_next_signal(std::string searched_signal, bool pop);
    bool connected_;
    bool running_;
    int sources_count_;

    unsigned int channels_count_;
    unsigned int acq_channels_count_;
    unsigned int max_acq_channels_;
    unsigned int applied_actions_;
    std::string config_file_;
    std::shared_ptr<ConfigurationInterface> configuration_;

    std::vector<std::shared_ptr<GNSSBlockInterface>> sig_source_;
    std::vector<std::shared_ptr<GNSSBlockInterface>> sig_conditioner_;

    std::shared_ptr<GNSSBlockInterface> observables_;
    std::shared_ptr<GNSSBlockInterface> pvt_;

    std::vector<std::shared_ptr<ChannelInterface>> channels_;
    gnss_sdr_sample_counter_sptr ch_out_sample_counter;
    gr::top_block_sptr top_block_;
    gr::msg_queue::sptr queue_;
    std::list<Gnss_Signal> available_GNSS_signals_;
    std::vector<unsigned int> channels_state_;
};

#endif /*GNSS_SDR_GNSS_FLOWGRAPH_H_*/
