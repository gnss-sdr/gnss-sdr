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
    ~hybrid_observables_gs();
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);
    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items, gr_vector_void_star& output_items);

private:
    friend hybrid_observables_gs_sptr hybrid_observables_gs_make(const Obs_Conf& conf_);

    explicit hybrid_observables_gs(const Obs_Conf& conf_);

    const size_t d_double_type_hash_code = typeid(double).hash_code();
    const size_t d_int_type_hash_code = typeid(int).hash_code();

    void msg_handler_pvt_to_observables(const pmt::pmt_t& msg);
    double compute_T_rx_s(const Gnss_Synchro& a) const;
    bool interp_trk_obs(Gnss_Synchro& interpolated_obs, uint32_t ch, uint64_t rx_clock) const;
    void update_TOW(const std::vector<Gnss_Synchro>& data);
    void compute_pranges(std::vector<Gnss_Synchro>& data) const;
    void smooth_pseudoranges(std::vector<Gnss_Synchro>& data);

    void set_tag_timestamp_in_sdr_timeframe(const std::vector<Gnss_Synchro>& data, uint64_t rx_clock);
    int32_t save_matfile() const;

    Obs_Conf d_conf;

    std::unique_ptr<Gnss_circular_deque<Gnss_Synchro>> d_gnss_synchro_history;  // Tracking observable history

    boost::circular_buffer<uint64_t> d_Rx_clock_buffer;  // time history

    std::vector<std::queue<GnssTime>> d_SourceTagTimestamps;
    std::queue<GnssTime> d_TimeChannelTagTimestamps;

    std::vector<bool> d_channel_last_pll_lock;
    std::vector<double> d_channel_last_pseudorange_smooth;
    std::vector<double> d_channel_last_carrier_phase_rads;

    std::string d_dump_filename;

    std::ofstream d_dump_file;

    double d_smooth_filter_M;
    double d_T_rx_step_s;
    double d_last_rx_clock_round20ms_error;

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
