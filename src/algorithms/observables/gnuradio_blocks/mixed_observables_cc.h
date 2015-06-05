/*!
 * \file mixed_observables_cc.h
 * \brief Interface of the pseudorange computation block for MIXED observables (Multi-frequency and Multi-system)
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_MIXED_OBSERVABLES_CC_H_
#define	GNSS_SDR_MIXED_OBSERVABLES_CC_H_

#include <fstream>
#include <queue>
#include <string>
#include <utility>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <gnuradio/block.h>
#include <gnuradio/msg_queue.h>
#include "concurrent_queue.h"
#include "gps_navigation_message.h"
#include "rinex_printer.h"
#include "GPS_L1_CA.h"
#include "gnss_synchro.h"

class mixed_observables_cc;

typedef boost::shared_ptr<mixed_observables_cc> mixed_observables_cc_sptr;

mixed_observables_cc_sptr
mixed_make_observables_cc(unsigned int n_channels, boost::shared_ptr<gr::msg_queue> queue, bool dump, std::string dump_filename, int output_rate_ms, bool flag_averaging);

/*!
 * \brief This class implements a block that computes MIXED observables (Multi-frequency and Multi-system)
 */
class mixed_observables_cc : public gr::block
{
public:
    ~mixed_observables_cc ();
    int general_work (int noutput_items, gr_vector_int &ninput_items,
            gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

private:
    friend mixed_observables_cc_sptr
    mixed_make_observables_cc(unsigned int nchannels, boost::shared_ptr<gr::msg_queue> queue, bool dump, std::string dump_filename, int output_rate_ms, bool flag_averaging);
    mixed_observables_cc(unsigned int nchannels, boost::shared_ptr<gr::msg_queue> queue, bool dump, std::string dump_filename, int output_rate_ms, bool flag_averaging);

    // class private vars
    boost::shared_ptr<gr::msg_queue> d_queue;
    bool d_dump;
    bool d_flag_averaging;
    unsigned int d_nchannels;
    int d_output_rate_ms;
    std::string d_dump_filename;
    std::ofstream d_dump_file;
};

#endif
