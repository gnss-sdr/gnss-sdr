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

#ifndef GNSS_SDR_GNSS_FLOWGRAPH_H_
#define GNSS_SDR_GNSS_FLOWGRAPH_H_

#include "channel_status_msg_receiver.h"
#include "concurrent_queue.h"
#include "gnss_sdr_sample_counter.h"
#include "gnss_signal.h"
#include "pvt_interface.h"
#include <gnuradio/blocks/null_sink.h>  // for null_sink
#include <gnuradio/runtime_types.h>     // for basic_block_sptr, top_block_sptr
#include <pmt/pmt.h>                    // for pmt_t
#include <list>                         // for list
#include <map>                          // for map
#include <memory>                       // for for shared_ptr, dynamic_pointer_cast
#include <mutex>                        // for mutex
#include <string>                       // for string
#include <utility>                      // for pair
#include <vector>                       // for vector
#if ENABLE_FPGA
#include "gnss_sdr_fpga_sample_counter.h"
#endif

class ChannelInterface;
class ConfigurationInterface;
class GNSSBlockInterface;
class Gnss_Satellite;

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
    GNSSFlowgraph(std::shared_ptr<ConfigurationInterface> configuration, const std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);  // NOLINT(performance-unnecessary-value-param)

    /*!
     * \brief Destructor
     */
    ~GNSSFlowgraph();

    /*!
     * \brief Start the flow graph
     */
    void start();

    /*!
     * \brief Stop the flow graph
     */
    void stop();

    /*!
     * \brief Connects the defined blocks in the flow graph
     *
     * Signal Source > Signal conditioner > Channels >> Observables >> PVT > Output filter
     */
    void connect();

    /*!
     * \brief Disconnect the blocks in the flow graph
     */
    void disconnect();

    /*!
     * \brief Wait for a flowgraph to complete.
     *
     * Flowgraphs complete when either
     * (1) all blocks indicate that they are done, or
     * (2) after stop() has been called to request shutdown.
     */
    void wait();

    /*!
     * \brief Manage satellite acquisition
     *
     * \param[in] who   Channel ID
     */
    void acquisition_manager(unsigned int who);

    /*!
     * \brief Applies an action to the flow graph
     *
     * \param[in] who   Who generated the action
     * \param[in] what  What is the action. 0: acquisition failed; 1: acquisition success; 2: tracking lost
     */
    void apply_action(unsigned int who, unsigned int what);

    /*!
     * \brief Set flow graph configuratiob
     */
    void set_configuration(std::shared_ptr<ConfigurationInterface> configuration);

    bool connected() const
    {
        return connected_;
    }

    bool running() const
    {
        return running_;
    }

    /*!
     * \brief Sends a GNU Radio asynchronous message from telemetry to PVT
     *
     * It is used to assist the receiver with external ephemeris data
     */
    bool send_telemetry_msg(const pmt::pmt_t& msg);

    /*!
     * \brief Returns a smart pointer to the PVT object
     */
    std::shared_ptr<PvtInterface> get_pvt()
    {
        return std::dynamic_pointer_cast<PvtInterface>(pvt_);
    }

    /*!
     * \brief Priorize visible satellites in the specified vector
     */
    void priorize_satellites(const std::vector<std::pair<int, Gnss_Satellite>>& visible_satellites);

#ifdef ENABLE_FPGA
    void start_acquisition_helper();

    void perform_hw_reset();
#endif

private:
    void init();  // Populates the SV PRN list available for acquisition and tracking
    void set_signals_list();
    void set_channels_state();  // Initializes the channels state (start acquisition or keep standby)
                                // using the configuration parameters (number of channels and max channels in acquisition)
    Gnss_Signal search_next_signal(const std::string& searched_signal,
        const bool pop,
        bool& is_primary_frequency,
        bool& assistance_available,
        float& estimated_doppler,
        double& RX_time);

    void push_back_signal(const Gnss_Signal& gs);
    void remove_signal(const Gnss_Signal& gs);

    double project_doppler(std::string searched_signal, double primary_freq_doppler_hz);
    bool connected_;
    bool running_;
    int sources_count_;

    unsigned int channels_count_;
    unsigned int acq_channels_count_;
    unsigned int max_acq_channels_;
    std::string config_file_;
    std::shared_ptr<ConfigurationInterface> configuration_;

    std::vector<std::shared_ptr<GNSSBlockInterface>> sig_source_;
    std::vector<std::shared_ptr<GNSSBlockInterface>> sig_conditioner_;
    std::vector<gr::blocks::null_sink::sptr> null_sinks_;

    std::shared_ptr<GNSSBlockInterface> observables_;
    std::shared_ptr<GNSSBlockInterface> pvt_;

    std::map<std::string, gr::basic_block_sptr> acq_resamplers_;
    std::vector<std::shared_ptr<ChannelInterface>> channels_;
    gnss_sdr_sample_counter_sptr ch_out_sample_counter;
#if ENABLE_FPGA
    gnss_sdr_fpga_sample_counter_sptr ch_out_fpga_sample_counter;
#endif
    gr::top_block_sptr top_block_;
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue_;

    std::list<Gnss_Signal> available_GPS_1C_signals_;
    std::list<Gnss_Signal> available_GPS_2S_signals_;
    std::list<Gnss_Signal> available_GPS_L5_signals_;
    std::list<Gnss_Signal> available_SBAS_1C_signals_;
    std::list<Gnss_Signal> available_GAL_1B_signals_;
    std::list<Gnss_Signal> available_GAL_5X_signals_;
    std::list<Gnss_Signal> available_GLO_1G_signals_;
    std::list<Gnss_Signal> available_GLO_2G_signals_;
    std::list<Gnss_Signal> available_BDS_B1_signals_;
    std::list<Gnss_Signal> available_BDS_B3_signals_;
    enum StringValue
    {
        evGPS_1C,
        evGPS_2S,
        evGPS_L5,
        evSBAS_1C,
        evGAL_1B,
        evGAL_5X,
        evGLO_1G,
        evGLO_2G,
        evBDS_B1,
        evBDS_B3
    };
    std::map<std::string, StringValue> mapStringValues_;

    std::vector<unsigned int> channels_state_;
    channel_status_msg_receiver_sptr channels_status_;  // class that receives and stores the current status of the receiver channels
    std::mutex signal_list_mutex;

    bool enable_monitor_;
    gr::basic_block_sptr GnssSynchroMonitor_;
    std::vector<std::string> split_string(const std::string& s, char delim);
};

#endif /* GNSS_SDR_GNSS_FLOWGRAPH_H_ */
