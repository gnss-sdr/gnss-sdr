/*!
 * \file gps_l1_ca_pvt_cc.h
 * \brief Interface of a Position Velocity and Time computation block for Galileo E1
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#ifndef GNSS_SDR_GALILEO_E1_PVT_CC_H
#define	GNSS_SDR_GALILEO_E1_PVT_CC_H

#include <fstream>
#include <gnuradio/block.h>
#include <gnuradio/msg_queue.h>
#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include "galileo_navigation_message.h"
#include "galileo_ephemeris.h"
#include "galileo_utc_model.h"
#include "galileo_iono.h"
#include "nmea_printer.h"
#include "kml_printer.h"
#include "rinex_printer.h"
#include "galileo_e1_ls_pvt.h"
#include "GPS_L1_CA.h"
#include "Galileo_E1.h"


class galileo_e1_pvt_cc;

typedef boost::shared_ptr<galileo_e1_pvt_cc> galileo_e1_pvt_cc_sptr;

galileo_e1_pvt_cc_sptr
galileo_e1_make_pvt_cc(unsigned int n_channels, boost::shared_ptr<gr::msg_queue> queue, bool dump, std::string dump_filename, int averaging_depth, bool flag_averaging, int output_rate_ms, int display_rate_ms, bool flag_nmea_tty_port, std::string nmea_dump_filename, std::string nmea_dump_devname);

/*!
 * \brief This class implements a block that computes the PVT solution
 */
class galileo_e1_pvt_cc : public gr::block
{
private:
    friend galileo_e1_pvt_cc_sptr
    galileo_e1_make_pvt_cc(unsigned int nchannels, boost::shared_ptr<gr::msg_queue> queue, bool dump, std::string dump_filename, int averaging_depth, bool flag_averaging, int output_rate_ms, int display_rate_ms, bool flag_nmea_tty_port, std::string nmea_dump_filename, std::string nmea_dump_devname);
    galileo_e1_pvt_cc(unsigned int nchannels, boost::shared_ptr<gr::msg_queue> queue, bool dump, std::string dump_filename, int averaging_depth, bool flag_averaging, int output_rate_ms, int display_rate_ms, bool flag_nmea_tty_port, std::string nmea_dump_filename, std::string nmea_dump_devname);
    boost::shared_ptr<gr::msg_queue> d_queue;
    bool d_dump;
    bool b_rinex_header_writen;
    Rinex_Printer *rp;
    unsigned int d_nchannels;
    std::string d_dump_filename;
    std::ofstream d_dump_file;
    int d_averaging_depth;
    bool d_flag_averaging;
    int d_output_rate_ms;
    int d_display_rate_ms;
    long unsigned int d_sample_counter;
    long unsigned int d_last_sample_nav_output;
    Kml_Printer d_kml_dump;
    Nmea_Printer *d_nmea_printer;
    double d_rx_time;
    galileo_e1_ls_pvt *d_ls_pvt;  /*modify PVT/libs/galileo_e1_ls_pvt*/

    bool pseudoranges_pairCompare_min( std::pair<int,Gnss_Synchro> a, std::pair<int,Gnss_Synchro> b);
public:
    ~galileo_e1_pvt_cc (); //!< Default destructor
    /*!
     * \brief Set the queue for getting navigation messages from the GpsL1CaTelemetryDecoder
     */

    int general_work (int noutput_items, gr_vector_int &ninput_items,
            gr_vector_const_void_star &input_items, gr_vector_void_star &output_items); //!< PVT Signal Processing
};

#endif
