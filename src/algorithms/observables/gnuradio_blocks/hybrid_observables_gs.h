/*!
 * \file hybrid_observables_gs.h
 * \brief Interface of the observables computation block
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 * \author Javier Arribas 2013. jarribas(at)cttc.es
 * \author Antonio Ramos 2018. antonio.ramos(at)cttc.es
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
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_HYBRID_OBSERVABLES_GS_H
#define GNSS_SDR_HYBRID_OBSERVABLES_GS_H

#include "obs_conf.h"
#include <boost/circular_buffer.hpp>  // for boost::circular_buffer
#include <gnuradio/block.h>           // for block
#include <gnuradio/types.h>           // for gr_vector_int
#include <cstdint>                    // for int32_t
#include <fstream>                    // for std::ofstream
#include <map>                        // for std::map
#include <memory>                     // for std:shared_ptr
#include <string>                     // for std::string
#include <vector>                     // for std::vector
#if GNURADIO_USES_STD_POINTERS
#include <memory>
#else
#include <boost/shared_ptr.hpp>
#endif

class Gnss_Synchro;
class hybrid_observables_gs;

template <class T>
class Gnss_circular_deque;

#if GNURADIO_USES_STD_POINTERS
using hybrid_observables_gs_sptr = std::shared_ptr<hybrid_observables_gs>;
#else
using hybrid_observables_gs_sptr = boost::shared_ptr<hybrid_observables_gs>;
#endif

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

    Obs_Conf d_conf;

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
        evBDS_B2,
        evBDS_B3
    };

    std::map<std::string, StringValue> mapStringValues_;
    std::vector<bool> d_channel_last_pll_lock;
    std::vector<double> d_channel_last_pseudorange_smooth;
    std::vector<double> d_channel_last_carrier_phase_rads;
    double d_smooth_filter_M;
    void smooth_pseudoranges(std::vector<Gnss_Synchro>& data);

    bool T_rx_TOW_set;  // rx time follow GPST
    bool d_dump;
    bool d_dump_mat;
    uint32_t T_rx_TOW_ms;
    uint32_t T_rx_step_ms;
    uint32_t T_status_report_timer_ms;
    uint32_t d_nchannels_in;
    uint32_t d_nchannels_out;
    std::string d_dump_filename;
    std::ofstream d_dump_file;
    boost::circular_buffer<uint64_t> d_Rx_clock_buffer;                         // time history
    std::shared_ptr<Gnss_circular_deque<Gnss_Synchro>> d_gnss_synchro_history;  // Tracking observable history
    void msg_handler_pvt_to_observables(const pmt::pmt_t& msg);
    double compute_T_rx_s(const Gnss_Synchro& a);
    bool interp_trk_obs(Gnss_Synchro& interpolated_obs, const uint32_t& ch, const uint64_t& rx_clock);
    void update_TOW(const std::vector<Gnss_Synchro>& data);
    void compute_pranges(std::vector<Gnss_Synchro>& data);
    int32_t save_matfile();
};

#endif  // GNSS_SDR_HYBRID_OBSERVABLES_GS_H
