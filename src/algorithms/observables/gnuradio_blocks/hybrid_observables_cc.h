/*!
 * \file hybrid_observables_cc.h
 * \brief Interface of the observables computation block
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 * \author Javier Arribas 2013. jarribas(at)cttc.es
 * \author Antonio Ramos 2018. antonio.ramos(at)cttc.es
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


#ifndef GNSS_SDR_HYBRID_OBSERVABLES_CC_H
#define GNSS_SDR_HYBRID_OBSERVABLES_CC_H

#include "gnss_synchro.h"
#include "gnss_circular_deque.h"
#include <gnuradio/block.h>
#include <boost/dynamic_bitset.hpp>
#include <fstream>
#include <string>
#include <utility>


class hybrid_observables_cc;

typedef boost::shared_ptr<hybrid_observables_cc> hybrid_observables_cc_sptr;

hybrid_observables_cc_sptr
hybrid_make_observables_cc(unsigned int nchannels_in, unsigned int nchannels_out, bool dump, bool dump_mat, std::string dump_filename);

/*!
 * \brief This class implements a block that computes observables
 */
class hybrid_observables_cc : public gr::block
{
public:
    ~hybrid_observables_cc();
    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items, gr_vector_void_star& output_items);
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

private:
    friend hybrid_observables_cc_sptr
    hybrid_make_observables_cc(uint32_t nchannels_in, uint32_t nchannels_out, bool dump, bool dump_mat, std::string dump_filename);
    hybrid_observables_cc(uint32_t nchannels_in, uint32_t nchannels_out, bool dump, bool dump_mat, std::string dump_filename);
    bool interpolate_data(Gnss_Synchro& out, const uint32_t& ch, const double& ti);
    bool interp_trk_obs(Gnss_Synchro& interpolated_obs, const uint32_t& ch, const uint64_t& rx_clock);
    double compute_T_rx_s(const Gnss_Synchro& a);
    void compute_pranges(std::vector<Gnss_Synchro>& data);
    void update_TOW(std::vector<Gnss_Synchro>& data);
    int32_t save_matfile();

    //time history
    boost::circular_buffer<uint64_t> d_Rx_clock_buffer;
    //Tracking observable history
    Gnss_circular_deque<Gnss_Synchro>* d_gnss_synchro_history;
    uint32_t T_rx_clock_step_samples;
    //rx time follow GPST
    bool T_rx_TOW_set;
    uint32_t T_rx_TOW_ms;
    uint32_t T_rx_TOW_offset_ms;
    bool d_dump;
    bool d_dump_mat;
    uint32_t d_nchannels_in;
    uint32_t d_nchannels_out;
    std::string d_dump_filename;
    std::ofstream d_dump_file;
};

#endif
