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

    /*!
     * \brief Shortest interruption of the observation stream of a channel that
     * can be attributed to a loss of lock, in seconds.
     *
     * Reacquiring a satellite and decoding its telemetry again takes seconds,
     * so shorter interruptions are transient failures to interpolate the
     * tracking history and leave the carrier phase untouched.
     */
    static constexpr double MIN_REACQUISITION_GAP_S = 1.0;

    /*!
     * \brief Tells whether the carrier phase of a channel can be assumed to be
     * continuous with respect to its previous observation.
     *
     * The accumulated carrier phase of a tracking channel restarts from scratch
     * whenever the channel (re)acquires a satellite, so an observation that
     * resumes after a loss of lock carries a new ambiguity.
     *
     * \param has_previous_observation Whether the channel produced a valid
     *        observation since it was last reset
     * \param last_valid_epoch Epoch count when the channel last produced a
     *        valid observation
     * \param current_epoch Epoch count of the observation being checked
     * \param last_valid_prn PRN of the last valid observation of the channel
     * \param current_prn PRN of the observation being checked
     * \param epoch_interval_s Time between consecutive observation epochs
     * \return true if the carrier phase ambiguity may have changed
     */
    static bool phase_stream_is_discontinuous(bool has_previous_observation,
        uint64_t last_valid_epoch,
        uint64_t current_epoch,
        uint32_t last_valid_prn,
        uint32_t current_prn,
        double epoch_interval_s);

    /*!
     * \brief Tells whether the carrier phase has stepped by half a cycle because
     * the Costas loop phase ambiguity resolved by the Telemetry Decoder changed.
     *
     * The Telemetry Decoder compensates that ambiguity by adding half a cycle to
     * the carrier phase while the PLL is locked at 180 degrees, so a change of
     * its determination steps the reported carrier phase.
     *
     * \param has_previous_observation Whether the channel produced a valid
     *        observation since it was last reset
     * \param carrier_phase_discontinuous Whether the carrier phase ambiguity has
     *        already changed for another reason, which supersedes this one
     * \param last_valid_prn PRN of the last valid observation of the channel
     * \param current_prn PRN of the observation being checked
     * \param last_pll_180_locked Ambiguity resolved for the previous observation
     * \param current_pll_180_locked Ambiguity resolved for this observation
     * \return true if the carrier phase stepped by half a cycle
     */
    static bool half_cycle_ambiguity_changed(bool has_previous_observation,
        bool carrier_phase_discontinuous,
        uint32_t last_valid_prn,
        uint32_t current_prn,
        bool last_pll_180_locked,
        bool current_pll_180_locked);

private:
    friend hybrid_observables_gs_sptr hybrid_observables_gs_make(const Obs_Conf& conf_);

    explicit hybrid_observables_gs(const Obs_Conf& conf_);

    const size_t d_double_type_hash_code = typeid(double).hash_code();
    const size_t d_int_type_hash_code = typeid(int).hash_code();

    static Gnss_Satellite pretty_satellite(char system, uint32_t prn);

    void msg_handler_pvt_to_observables(const pmt::pmt_t& msg);
    double compute_T_rx_s(const Gnss_Synchro& a) const;
    bool interp_trk_obs(Gnss_Synchro& interpolated_obs, uint32_t ch, uint64_t rx_clock) const;
    bool has_fresh_trk_data(uint32_t ch, uint64_t rx_clock) const;
    void update_TOW(const std::vector<Gnss_Synchro>& data);
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
    uint32_t d_T_status_report_timer_ms;
    uint32_t d_nchannels_in;
    uint32_t d_nchannels_out;

    bool d_T_rx_TOW_set;  // rx time follow GPST
    bool d_always_output_gs;
    bool d_dump;
    bool d_dump_mat;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_HYBRID_OBSERVABLES_GS_H
