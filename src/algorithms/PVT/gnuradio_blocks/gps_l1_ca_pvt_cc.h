/*!
 * \file gps_l1_ca_pvt_cc.h
 * \brief Interface of a Position Velocity and Time computation block for GPS L1 C/A
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

#ifndef GNSS_SDR_GPS_L1_CA_PVT_CC_H
#define	GNSS_SDR_GPS_L1_CA_PVT_CC_H

#include <fstream>
#include <gnuradio/gr_block.h>
#include <gnuradio/gr_msg_queue.h>
#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include "concurrent_queue.h"
#include "gps_navigation_message.h"
#include "kml_printer.h"
#include "rinex_printer.h"
#include "gps_l1_ca_ls_pvt.h"
#include "GPS_L1_CA.h"

class gps_l1_ca_pvt_cc;

typedef boost::shared_ptr<gps_l1_ca_pvt_cc> gps_l1_ca_pvt_cc_sptr;

gps_l1_ca_pvt_cc_sptr
gps_l1_ca_make_pvt_cc(unsigned int n_channels, gr_msg_queue_sptr queue, bool dump, std::string dump_filename, int averaging_depth, bool flag_averaging);

/*!
 * \brief This class implements a block that computes the PVT solution
 */
class gps_l1_ca_pvt_cc : public gr_block
{

private:

  friend gps_l1_ca_pvt_cc_sptr
  gps_l1_ca_make_pvt_cc(unsigned int nchannels, gr_msg_queue_sptr queue, bool dump, std::string dump_filename, int averaging_depth, bool flag_averaging);

  gps_l1_ca_pvt_cc(unsigned int nchannels, gr_msg_queue_sptr queue, bool dump, std::string dump_filename, int averaging_depth, bool flag_averaging);

  gr_msg_queue_sptr d_queue;
  bool d_dump;
  bool b_rinex_header_writen;
  Rinex_Printer *rp;

  unsigned int d_nchannels;

  std::string d_dump_filename;
  std::ofstream d_dump_file;

  int d_averaging_depth;
  bool d_flag_averaging;

  long unsigned int d_sample_counter;

  kml_printer d_kml_dump;

  concurrent_queue<gps_navigation_message> *d_nav_queue; // Navigation ephemeris queue
  gps_navigation_message d_last_nav_msg;                 // Last navigation message

  double d_ephemeris_clock_s;
  double d_ephemeris_timestamp_ms;
  gps_l1_ca_ls_pvt *d_ls_pvt;


public:

  ~gps_l1_ca_pvt_cc (); //!< Default destructor

  /*!
   * \brief Set the queue for getting navigation messages from the GpsL1CaTelemetryDecoder
   */
  void set_navigation_queue(concurrent_queue<gps_navigation_message> *nav_queue){d_nav_queue=nav_queue;}

  int general_work (int noutput_items, gr_vector_int &ninput_items,
      gr_vector_const_void_star &input_items, gr_vector_void_star &output_items); //!< PVT Signal Processing
};

#endif
