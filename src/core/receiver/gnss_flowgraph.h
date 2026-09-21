/*!
 * \file gnss_flowgraph.h
 * \brief Interface of a GNSS receiver flow graph.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *         Carles Fernandez-Prades, 2014-2020. cfernandez(at)cttc.es
 *         Álvaro Cebrián Juan, 2018. acebrianjuan(at)gmail.com
 *
 * It contains a signal source,
 * a signal conditioner, a set of channels, an observables block and a pvt.
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GNSS_FLOWGRAPH_H
#define GNSS_SDR_GNSS_FLOWGRAPH_H

#include "channel_status_msg_receiver.h"
#include "concurrent_queue.h"
#include "galileo_e6_has_msg_receiver.h"
#include "galileo_tow_map.h"
#include "gnss_sdr_sample_counter.h"
#include "gnss_signal.h"
#include "osnma_msg_receiver.h"
#include "pvt_interface.h"
#include "satellite_visibility.h"
#include <gnuradio/blocks/null_sink.h>  // for null_sink
#include <gnuradio/runtime_types.h>     // for basic_block_sptr, top_block_sptr
#include <pmt/pmt.h>                    // for pmt_t
#include <array>                        // for array
#include <chrono>                       // for steady_clock
#include <ctime>                        // for time_t
#include <list>                         // for list
#include <map>                          // for map
#include <memory>                       // for for shared_ptr, dynamic_pointer_cast
#include <mutex>                        // for mutex
#include <set>                          // for set
#include <string>                       // for string
#include <unordered_map>                // for unordered_map
#include <utility>                      // for pair
#include <vector>                       // for vector
#if ENABLE_FPGA
#include "gnss_sdr_fpga_sample_counter.h"
#endif

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver
 * \{ */


class ChannelInterface;
class ConfigurationInterface;
class GNSSBlockInterface;
class Gnss_Satellite;
class SignalSourceInterface;

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
    GNSSFlowgraph(std::shared_ptr<ConfigurationInterface> configuration, std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);

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
    void set_configuration(const std::shared_ptr<ConfigurationInterface>& configuration);

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
     * \brief Prioritize visible satellites in the specified vector
     */
    void priorize_satellites(const std::vector<std::pair<int, Gnss_Satellite>>& visible_satellites);

    /*!
     * \brief Re-evaluates satellite visibility when warranted (new fix, new
     * ephemeris/almanac, or recompute interval elapsed) so that subsequent
     * search_next_signal() calls favor visible satellites.
     * No-op unless GNSS-SDR.enable_visibility_aware_search=true.
     */
    void MaybeUpdateVisibility();

    /*!
     * \brief Sets the telecommand position/time reference and immediately
     * refreshes visibility. Called by the control thread.
     */
    void UpdateVisibilityReference(time_t utc_time, const std::array<float, 3>& LLH);

    /*!
     * \brief Whether GNSS-SDR.enable_visibility_aware_search is on, so callers can
     * skip the legacy get_visible_sats()/priorize_satellites() startup reorder that
     * MaybeUpdateVisibility() supersedes.
     */
    bool visibility_aware_search_enabled() const;

    /*!
     * \brief Stops any channel whose decoded satellite is already tracked, on the
     * same signal, by another channel with higher C/N0. The stopped channel's
     * assignment returns to the search pool and the channel is re-dispatched.
     *
     * GLONASS channels are assigned a frequency channel and only learn their
     * orbital slot from the navigation message, so a false lock can be relabelled
     * as a satellite another channel already tracks; duplicate observations of
     * one satellite break the PVT solution.
     */
    void stop_duplicated_satellite_channels();

#if ENABLE_FPGA
    void start_acquisition_helper();

    void perform_hw_reset();
#endif

private:
    void init();  // Populates the SV PRN list available for acquisition and tracking
    int connect_desktop_flowgraph();

    int connect_signal_sources();
    int connect_signal_conditioners();
    int connect_channels();
    int connect_observables();
    int connect_pvt();
    int connect_sample_counter();
    int connect_galileo_tow_map();

    int connect_signal_sources_to_signal_conditioners();
    int connect_signal_conditioners_to_channels();
    int connect_channels_to_observables();
    int connect_observables_to_pvt();
    int connect_monitors();
    int connect_osnma();
    int connect_gal_e6_has();
    int connect_gnss_synchro_monitor();
    int connect_acquisition_monitor();
    int connect_tracking_monitor();
    int connect_navdata_monitor();

#if ENABLE_FPGA
    int connect_fpga_flowgraph();
    int connect_fpga_sample_counter();
#endif

    int assign_channels();
    void check_signal_conditioners();

    void set_signals_list();
    void keep_one_glonass_slot_per_frequency(std::set<unsigned int>& available_prns);

    void set_channels_state();  // Initializes the channels state (start acquisition or keep standby)
                                // using the configuration parameters (number of channels and max channels in acquisition)
    //! On assistance, estimated_doppler is returned already projected to the
    //! searched signal's carrier frequency (see project_doppler()).
    Gnss_Signal search_next_signal(const std::string& searched_signal,
        bool& is_primary_frequency,
        bool& assistance_available,
        float& estimated_doppler,
        double& RX_time,
        bool& signal_available);

    void push_back_signal(const Gnss_Signal& gs);
    void remove_signal(const Gnss_Signal& gs);

    // Visibility-aware replacement for available_signals.front()/pop_front() in
    // search_next_signal(): erases and returns the next searchable entry from the
    // visible or maybe-visible bucket selected by the search-ratio counter, falling
    // back to the other bucket when the selected one is empty; FIFO order is kept
    // within each bucket. Excluded entries are never picked but stay queued until
    // the next visibility recompute. Sets picked=false when nothing is searchable.
    Gnss_Signal pop_by_visibility(std::list<Gnss_Signal>& available_signals, const std::string& searched_signal, bool& picked, double cooldown_receiver_time_s);
    void print_help();
    void check_desktop_conf_in_fpga_env();

    double project_doppler(const std::string& searched_signal, const std::string& assist_signal, double assist_doppler_hz);
    bool is_multiband() const;

    std::vector<std::string> split_string(const std::string& s, char delim);
    std::vector<bool> signal_conditioner_connected_;

    gr::top_block_sptr top_block_;

    std::shared_ptr<ConfigurationInterface> configuration_;
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue_;

    std::vector<std::shared_ptr<SignalSourceInterface>> sig_source_;
    std::vector<std::shared_ptr<GNSSBlockInterface>> sig_conditioner_;
    std::vector<std::shared_ptr<ChannelInterface>> channels_;
    std::shared_ptr<GNSSBlockInterface> observables_;
    std::shared_ptr<GNSSBlockInterface> pvt_;

    std::map<std::string, gr::basic_block_sptr> acq_resamplers_;
    std::vector<gr::blocks::null_sink::sptr> null_sinks_;

    gr::basic_block_sptr GnssSynchroMonitor_;
    gr::basic_block_sptr GnssSynchroAcquisitionMonitor_;
    gr::basic_block_sptr GnssSynchroTrackingMonitor_;
    gr::basic_block_sptr NavDataMonitor_;
    channel_status_msg_receiver_sptr channels_status_;  // class that receives and stores the current status of the receiver channels
    galileo_e6_has_msg_receiver_sptr gal_e6_has_rx_;
    galileo_tow_map_sptr galileo_tow_map_;
    osnma_msg_receiver_sptr osnma_rx_;

    gnss_sdr_sample_counter_sptr ch_out_sample_counter_;
#if ENABLE_FPGA
    gnss_sdr_fpga_sample_counter_sptr ch_out_fpga_sample_counter_;
#endif

    std::vector<unsigned int> channels_state_;  // 0 - Idle, 1 - Assigned, 2 - Acquisition, 3 - Tracking

    std::unordered_map<std::string, std::list<Gnss_Signal>> available_signals_map_;

    std::unique_ptr<SatelliteVisibility> satellite_visibility_;
    std::unordered_map<std::string, uint32_t> visibility_pick_counter_;  // per signal_str, ratio-based visible/maybe-visible cycling
    // Signals whose pool pop_by_visibility() found fully excluded. Cached because
    // acquisition_manager() re-queries every idle channel at ~10 Hz and each scan
    // copies channels_status_ under a lock shared with the DSP threads. Invalidated
    // per signal by push_back_signal(), and entirely when MaybeUpdateVisibility()
    // changes the classification.
    std::set<std::string> signals_with_nothing_searchable_;
    // Rate-limits the "no assist-tracked satellite available" log in
    // search_next_signal(): the condition is re-evaluated for every idle channel at
    // ~10 Hz and would otherwise flood the log.
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> no_assist_log_throttle_;

    // Acquisition retry cooldown per PRN and signal, set with
    // GNSS-SDR.acquisition_max_retry_rate_hz (0 = disabled). Paced by the
    // receiver time reported by the observables block, which keeps running
    // without a PVT fix and follows the signal timeline in file replays.
    // The map stores the receiver time of the last attempt.
    double acquisition_retry_min_interval_s_;
    std::unordered_map<std::string, double> last_acquisition_attempt_rx_time_s_;
    bool InAcquisitionCooldown(const Gnss_Signal& gs, double receiver_time_s) const;
    void MarkAcquisitionAttempt(const Gnss_Signal& gs, double receiver_time_s);

    enum StringValue
    {
        evGPS_1C,
        evGPS_2S,
        evGPS_L5,
        evSBAS_1C,
        evGAL_1B,
        evGAL_5X,
        evGAL_7X,
        evGAL_E6,
        evGLO_1G,
        evGLO_2G,
        evBDS_B1,
        evBDS_B1C,
        evBDS_B2A,
        evBDS_B3,
        evQZS_J1,
        evQZS_J5
    };
    std::map<std::string, StringValue> mapStringValues_;

    std::string config_file_;
    std::string help_hint_;

    std::mutex signal_list_mutex_;

    int sources_count_;
    int channels_count_;
    int acq_channels_count_;
    int max_acq_channels_;

    bool connected_;
    bool running_;
    bool multiband_;
    bool enable_monitor_;
    bool enable_acquisition_monitor_;
    bool enable_tracking_monitor_;
    bool enable_navdata_monitor_;
    bool enable_fpga_offloading_;
    bool enable_osnma_rx_;
    bool enable_e6_has_rx_;
    bool enable_secondary_signal_status_gating_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_FLOWGRAPH_H
