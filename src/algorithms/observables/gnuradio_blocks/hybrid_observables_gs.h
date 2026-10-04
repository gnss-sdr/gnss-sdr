/*!
 * \file hybrid_observables_gs.h
 * \brief Interface of the observables computation block
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 * \author Javier Arribas 2013. jarribas(at)cttc.es
 * \author Antonio Ramos 2018. antonio.ramos(at)cttc.es
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


#ifndef GNSS_SDR_HYBRID_OBSERVABLES_GS_H
#define GNSS_SDR_HYBRID_OBSERVABLES_GS_H

#include "gnss_block_interface.h"
#include "gnss_time.h"  // for timetags produced by Tracking
#include "obs_conf.h"
#include <boost/circular_buffer.hpp>  // for boost::circular_buffer
#include <gnuradio/block.h>           // for block
#include <gnuradio/types.h>           // for gr_vector_int
#include <cstddef>                    // for size_t
#include <cstdint>                    // for int32_t
#include <fstream>                    // for std::ofstream
#include <memory>                     // for std::shared, std:unique_ptr
#include <queue>                      // for std::queue
#include <string>                     // for std::string
#include <typeinfo>                   // for typeid
#include <vector>                     // for std::vector

/** \addtogroup Observables
 * \{ */
/** \addtogroup Observables_gnuradio_blocks obs_gr_blocks
 * GNU Radio blocks for the computation of GNSS observables
 * \{ */


class Gnss_Satellite;
class Gnss_Synchro;
class hybrid_observables_gs;

template <class T>
class Gnss_circular_deque;

using hybrid_observables_gs_sptr = gnss_shared_ptr<hybrid_observables_gs>;

hybrid_observables_gs_sptr hybrid_observables_gs_make(const Obs_Conf& conf_);

/*!
 * \brief This class implements a block that computes observables
 */
class hybrid_observables_gs : public gr::block
{
public:
    ~hybrid_observables_gs() noexcept;
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);
    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items, gr_vector_void_star& output_items);

private:
    friend hybrid_observables_gs_sptr hybrid_observables_gs_make(const Obs_Conf& conf_);

    explicit hybrid_observables_gs(const Obs_Conf& conf_);

    const size_t d_double_type_hash_code = typeid(double).hash_code();
    const size_t d_int_type_hash_code = typeid(int).hash_code();

    static Gnss_Satellite pretty_satellite(char system, uint32_t prn);

    // Shortest interruption of the observation stream of a channel that can be
    // attributed to a loss of lock, in seconds. Reacquiring a satellite and
    // decoding its telemetry again takes seconds, so shorter interruptions are
    // transient failures to interpolate the tracking history and leave the
    // carrier phase untouched.
    static constexpr double MIN_REACQUISITION_GAP_S = 1.0;

    // Tells whether the carrier phase ambiguity of a channel may have changed
    // since its previous valid observation: the accumulated carrier phase
    // restarts from scratch whenever the channel (re)acquires a satellite, so
    // an observation that resumes after a loss of lock carries a new ambiguity.
    // Epochs are counts of observation epochs, epoch_interval_s apart.
    static bool phase_stream_is_discontinuous(bool has_previous_observation,
        uint64_t last_valid_epoch,
        uint64_t current_epoch,
        uint32_t last_valid_prn,
        uint32_t current_prn,
        double epoch_interval_s);

    // Tells whether the carrier phase has stepped by half a cycle because the
    // Costas loop ambiguity resolved by the Telemetry Decoder changed: it adds
    // half a cycle to the carrier phase while the PLL is locked at 180 degrees.
    // A new ambiguity (carrier_phase_discontinuous) supersedes this report.
    static bool half_cycle_ambiguity_changed(bool has_previous_observation,
        bool carrier_phase_discontinuous,
        uint32_t last_valid_prn,
        uint32_t current_prn,
        bool last_pll_180_locked,
        bool current_pll_180_locked);

    // Linearly interpolates the carrier phase between two tracking samples, in
    // the polarity frame of the later one. When the PLL-180 half-cycle
    // correction toggles between them, their phases differ by half a cycle on
    // top of the true motion, and interpolating across that step would leak a
    // fraction of it into the output with no slip flag raised. time_factor is
    // the position of the epoch between the two samples, in [0, 1].
    static double interpolate_carrier_phase(double phase_early_rads,
        bool pll_180_early,
        double phase_late_rads,
        bool pll_180_late,
        double time_factor);

    // Longest time between the two tracking samples used to interpolate an
    // observation, in seconds. The Telemetry Decoders deliver one sample per
    // telemetry symbol, which lasts 20 ms at most (GPS L1 C/A, GPS L2C, Galileo
    // E5a, BeiDou D1), so samples further apart are not consecutive: the
    // channel stopped delivering valid words in between. For the same reason,
    // it is the time by which every channel delivering valid words can be
    // interpolated again after the tracking history is emptied.
    static constexpr double MAX_INTERPOLATION_INTERVAL_S = 0.05;

    // Largest disagreement between the TOW increment and the receiver time
    // increment of two consecutive tracking samples, in ms. Both advance at the
    // same pace but for the code Doppler and the sampling clock error, which
    // amount to microseconds in MAX_INTERPOLATION_INTERVAL_S, while the TOW
    // reported by a Telemetry Decoder steps in whole ms.
    static constexpr double MAX_TOW_INCREMENT_ERROR_MS = 0.5;

    // Largest difference between the TOWs reported at the same epoch by
    // channels that agree on the time, in ms. It covers the spread of the
    // signal travel times, from MEO satellites at the zenith to geosynchronous
    // ones at the horizon (with margin for a receiver in Earth orbit), plus the
    // resolution of the TOW, which is reported at the telemetry symbol closest
    // to the epoch.
    static constexpr int64_t MAX_TOW_SPREAD_MS = 300;

    // Largest difference between the receiver time and the TOW on which the
    // channels agree, in ms, not counting the rounding of the receiver time to
    // the observables interval. It is far above the offsets found in normal
    // operation (a signal travel time, plus the drift of a free-running
    // receiver clock) and far below the error of a TOW decoded from a false
    // frame synchronization.
    static constexpr int64_t MAX_RX_TIME_ERROR_MS = 1000;

    // Time that the channels need to keep contradicting the receiver time
    // before it is set again from them, in seconds.
    static constexpr double RX_TIME_CONTRADICTION_TIME_S = 1.0;

    // Difference between two TOWs in ms, unwrapped across the end of the week:
    // the result is in [-half a week, half a week).
    static int64_t tow_difference_ms(uint32_t tow_ms, uint32_t tow_ref_ms);

    // Brings a time into [0, one week) and then rounds it up to a multiple of
    // the observables interval, which is how the receiver time is kept. The
    // time can be negative or exceed a week, as it happens when a clock offset
    // is applied next to the TOW rollover.
    static uint32_t align_rx_time_ms(int64_t tow_ms, uint32_t interval_ms);

    // Tells whether an observation can be interpolated between two tracking
    // samples of a channel: they must be consecutive, and their TOW must have
    // advanced as much as the receiver time did. The history of a channel only
    // keeps the samples with a valid TOW, so when a channel stops delivering
    // them and then resumes with the same satellite (the Telemetry Decoder lost
    // and recovered the frame synchronization, or the satellite was
    // reacquired), the last sample before the gap and the first one after it
    // become neighbors. They do not bracket a continuous piece of signal: their
    // carrier phases are unrelated if the tracking loops restarted in between,
    // and the TOW might have stepped if any of the two frame synchronizations
    // was false.
    static bool tracking_samples_are_consecutive(double rx_time_early_s,
        uint32_t tow_early_ms,
        double rx_time_late_s,
        uint32_t tow_late_ms);

    // Finds the largest group of channels reporting the same time, and returns
    // its size and its latest TOW. A channel can report a wrong TOW, for
    // instance after a false frame synchronization of its Telemetry Decoder,
    // and such errors are unrelated from one channel to another, whereas the
    // channels reporting the right TOW agree with each other within
    // MAX_TOW_SPREAD_MS. If several groups have the same size, the one with the
    // latest TOW is returned, and is_unique is false if they are separate
    // groups: there is no reason to prefer the one returned. tow_ms holds the
    // TOW of each of the channels with a valid TOW, in [0, one week).
    static uint32_t find_tow_consensus(const std::vector<uint32_t>& tow_ms,
        uint32_t& latest_tow_ms,
        bool& is_unique);

    void msg_handler_pvt_to_observables(const pmt::pmt_t& msg);
    double compute_T_rx_s(const Gnss_Synchro& a) const;
    bool interp_trk_obs(Gnss_Synchro& interpolated_obs, uint32_t ch, uint64_t rx_clock) const;
    bool has_fresh_trk_data(uint32_t ch, uint64_t rx_clock) const;
    void update_TOW(const std::vector<Gnss_Synchro>& data);
    void set_T_rx_TOW_ms(uint32_t tow_ms);
    bool rx_time_is_on_hold() const;
    void compute_pranges(std::vector<Gnss_Synchro>& data) const;
    void smooth_pseudoranges(std::vector<Gnss_Synchro>& data);
    void detect_cycle_slips(std::vector<Gnss_Synchro>& data, uint64_t rx_clock);

    void set_tag_timestamp_in_sdr_timeframe(const std::vector<Gnss_Synchro>& data, uint64_t rx_clock);

    void propagate_sensor_data(const std::vector<Gnss_Synchro>& data);

    int32_t save_matfile() const;

    Obs_Conf d_conf;

    std::unique_ptr<Gnss_circular_deque<Gnss_Synchro>> d_gnss_synchro_history;  // Tracking observable history

    boost::circular_buffer<uint64_t> d_Rx_clock_buffer;  // time history

    std::vector<std::queue<GnssTime>> d_SourceTagTimestamps;
    std::queue<GnssTime> d_TimeChannelTagTimestamps;

    std::queue<gr::tag_t> d_sensor_data_tags;
    std::uint64_t d_trq_last_sample{0};

    std::vector<bool> d_channel_last_pll_lock;
    std::vector<double> d_channel_last_pseudorange_smooth;
    std::vector<double> d_channel_last_carrier_phase_rads;
    std::vector<bool> d_channel_has_previous_observation;     // carrier phase continuity bookkeeping
    std::vector<bool> d_channel_phase_discontinuity_pending;  // reported by the tracking block
    std::vector<bool> d_channel_last_pll_180_locked;          // Costas loop ambiguity state
    std::vector<uint64_t> d_channel_last_valid_epoch;
    std::vector<uint32_t> d_channel_last_valid_prn;
    std::vector<Gnss_Synchro> d_last_trk_data;  // latest tracking data per channel, for the Monitor

    std::string d_dump_filename;

    std::ofstream d_dump_file;

    double d_smooth_filter_M;
    double d_T_rx_step_s;
    double d_last_rx_clock_round20ms_error;

    uint64_t d_epoch_counter{0};

    uint32_t d_T_rx_TOW_ms;
    uint32_t d_T_rx_step_ms;
    uint32_t d_T_rx_contradicted_ms{0};     // time that the channels have been contradicting the rx time
    uint32_t d_T_rx_TOW_unconfirmed_ms{0};  // time that the rx time has been running, if no group of channels agreed on it
    uint32_t d_T_status_report_timer_ms;
    uint32_t d_T_time_report_timer_ms{0};
    uint32_t d_nchannels_in;
    uint32_t d_nchannels_out;

    bool d_T_rx_TOW_set;               // rx time follow GPST
    bool d_T_rx_TOW_confirmed{false};  // a group of channels agreed on the rx time
    bool d_always_output_gs;
    bool d_dump;
    bool d_dump_mat;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_HYBRID_OBSERVABLES_GS_H
