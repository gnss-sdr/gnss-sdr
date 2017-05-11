/*!
 * \file hybrid_observables_cc.h
 * \brief Interface of the observables computation block for Galileo E1
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 * \author Javier Arribas 2013. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_HYBRID_OBSERVABLES_CC_H
#define GNSS_SDR_HYBRID_OBSERVABLES_CC_H

#include <fstream>
#include <string>
#include <gnuradio/block.h>
#include "gnss_synchro.h"


class hybrid_observables_cc;

typedef boost::shared_ptr<hybrid_observables_cc> hybrid_observables_cc_sptr;

hybrid_observables_cc_sptr
hybrid_make_observables_cc(unsigned int n_channels, bool dump, std::string dump_filename, unsigned int deep_history);

/*!
 * \brief This class implements a block that computes Galileo observables
 */
class hybrid_observables_cc : public gr::block
{
public:
    ~hybrid_observables_cc ();
    int general_work (int noutput_items, gr_vector_int &ninput_items,
            gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

private:
    friend hybrid_observables_cc_sptr
    hybrid_make_observables_cc(unsigned int nchannels, bool dump, std::string dump_filename, unsigned int deep_history);
    hybrid_observables_cc(unsigned int nchannels, bool dump, std::string dump_filename, unsigned int deep_history);

    //Tracking observable history
    std::vector<std::deque<Gnss_Synchro>> d_gnss_synchro_history_queue;

    double T_rx_s;
    double T_rx_step_s;
    int d_max_noutputs;
    bool d_dump;
    unsigned int d_nchannels;
    unsigned int history_deep;
    std::string d_dump_filename;
    std::ofstream d_dump_file;
};

#endif
