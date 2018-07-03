/*!
 * \file gnss_flowgraph.h
 * \brief Interface of a GNSS receiver flow graph.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *         Carles Fernandez-Prades, 2014. cfernandez(at)cttc.es
 *         Álvaro Cebrián Juan, 2018. acebrianjuan(at)gmail.com
 *
 * It contains a signal source,
 * a signal conditioner, a set of channels, an observables block and a pvt.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_GNSS_FLOWGRAPH_H_
#define GNSS_SDR_GNSS_FLOWGRAPH_H_

#include "GPS_L1_CA.h"
#include "gnss_signal.h"
#include "gnss_sdr_sample_counter.h"
#include "gnss_synchro_monitor.h"
#include <gnuradio/top_block.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_source.h>
#include <gnuradio/blocks/throttle.h>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#if ENABLE_FPGA
#include "gnss_sdr_time_counter.h"
#endif

class GNSSBlockInterface;
class ChannelInterface;
class ConfigurationInterface;
class GNSSBlockFactory;

/*! \brief This class represents a GNSS flow graph.
 *
 * It contains a signal source,
 * a signal conditioner, a set of channels, a PVT and an output filter.
 */
class GNSSFlowgraph
{
public:
    /*!
     * \brief Constructor that initializes the receiver flow graph
     */
    GNSSFlowgraph(std::shared_ptr<ConfigurationInterface> configuration, gr::msg_queue::sptr queue);

    /*!
     * \brief Destructor
     */
    ~GNSSFlowgraph();

    //! \brief Start the flow graph
    void start();

    //! \brief Stop the flow graph
    void stop();

    /*!
     * \brief Connects the defined blocks in the flow graph
     *
     * Signal Source > Signal conditioner > Channels >> Observables >> PVT > Output filter
     */
    void connect();

    void disconnect();

    void wait();

    void start_acquisition_helper();

    /*!
     * \brief Applies an action to the flow graph
     *
     * \param[in] who   Who generated the action
     * \param[in] what  What is the action. 0: acquisition failed; 1: acquisition success; 2: tracking lost
     */
    void apply_action(unsigned int who, unsigned int what);

    void set_configuration(std::shared_ptr<ConfigurationInterface> configuration);

    unsigned int applied_actions() const
    {
        return applied_actions_;
    }

    bool connected() const
    {
        return connected_;
    }

    bool running() const
    {
        return running_;
    }

    /*!
     * \brief Sends a GNURadio asynchronous message from telemetry to PVT
     *
     * It is used to assist the receiver with external ephemeris data
     */
    bool send_telemetry_msg(pmt::pmt_t msg);

private:
    void init();  // Populates the SV PRN list available for acquisition and tracking
    void set_signals_list();
    void set_channels_state();  // Initializes the channels state (start acquisition or keep standby)
                                // using the configuration parameters (number of channels and max channels in acquisition)
    Gnss_Signal search_next_signal(std::string searched_signal, bool pop, bool tracked = false);
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
#if ENABLE_FPGA
    gnss_sdr_time_counter_sptr time_counter_;
#endif
    gr::blocks::null_source::sptr null_source_;
    gr::blocks::throttle::sptr throttle_;
    gr::top_block_sptr top_block_;
    gr::msg_queue::sptr queue_;

    std::list<Gnss_Signal> available_GPS_1C_signals_;
    std::list<Gnss_Signal> available_GPS_2S_signals_;
    std::list<Gnss_Signal> available_GPS_L5_signals_;
    std::list<Gnss_Signal> available_SBAS_1C_signals_;
    std::list<Gnss_Signal> available_GAL_1B_signals_;
    std::list<Gnss_Signal> available_GAL_5X_signals_;
    std::list<Gnss_Signal> available_GLO_1G_signals_;
    std::list<Gnss_Signal> available_GLO_2G_signals_;
    enum StringValue
    {
        evGPS_1C,
        evGPS_2S,
        evGPS_L5,
        evSBAS_1C,
        evGAL_1B,
        evGAL_5X,
        evGLO_1G,
        evGLO_2G
    };
    std::map<std::string, StringValue> mapStringValues_;

    std::vector<unsigned int> channels_state_;
    std::mutex signal_list_mutex;

    bool enable_monitor_;
    gr::basic_block_sptr GnssSynchroMonitor_;
};

#endif /*GNSS_SDR_GNSS_FLOWGRAPH_H_*/
