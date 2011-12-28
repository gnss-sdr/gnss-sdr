/*!
 * \file gps_l1_ca_pvt_cc.cc
 * \brief Position Velocity and Time computation for GPS L1 C/A
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
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

#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>
#include <bitset>
#include <cmath>
#include "math.h"
#include <gnuradio/gr_io_signature.h>
#include <glog/log_severity.h>
#include <glog/logging.h>
#include "gps_l1_ca_pvt_cc.h"
#include "control_message_factory.h"


using google::LogMessage;


gps_l1_ca_pvt_cc_sptr
gps_l1_ca_make_pvt_cc(unsigned int nchannels, gr_msg_queue_sptr queue, bool dump, std::string dump_filename, int averaging_depth, bool flag_averaging) {

  return gps_l1_ca_pvt_cc_sptr(new gps_l1_ca_pvt_cc(nchannels, queue, dump, dump_filename, averaging_depth, flag_averaging));
}


gps_l1_ca_pvt_cc::gps_l1_ca_pvt_cc(unsigned int nchannels, gr_msg_queue_sptr queue, bool dump, std::string dump_filename, int averaging_depth, bool flag_averaging) :
		        gr_block ("gps_l1_ca_pvt_cc", gr_make_io_signature (nchannels, nchannels,  sizeof(gnss_pseudorange)),
		            gr_make_io_signature(1, 1, sizeof(gr_complex))) {

  // initialize internal vars
  d_queue = queue;
  d_dump = dump;
  d_nchannels = nchannels;
  d_dump_filename=dump_filename;
  std::string kml_dump_filename;
  kml_dump_filename=d_dump_filename;
  kml_dump_filename.append(".kml");
  d_kml_dump.set_headers(kml_dump_filename);
  d_dump_filename.append(".dat");

  d_averaging_depth=averaging_depth;
  d_flag_averaging=flag_averaging;
  /*!
   * \todo Enable RINEX printer: The current RINEX printer need a complete refactoring and some bug fixing work
   */
  //d_rinex_printer.set_headers("GNSS-SDR");
  d_ls_pvt=new gps_l1_ca_ls_pvt(nchannels,d_dump_filename,d_dump);
  d_ls_pvt->set_averaging_depth(d_averaging_depth);
  d_ephemeris_clock_s=0.0;

  d_sample_counter=0;
}

gps_l1_ca_pvt_cc::~gps_l1_ca_pvt_cc() {
    d_kml_dump.close_file();
    delete d_ls_pvt;
}

bool pseudoranges_pairCompare_min( std::pair<int,gnss_pseudorange> a, std::pair<int,gnss_pseudorange> b)
{
  return (a.second.pseudorange_m) < (b.second.pseudorange_m);
}

int gps_l1_ca_pvt_cc::general_work (int noutput_items, gr_vector_int &ninput_items,
		gr_vector_const_void_star &input_items,	gr_vector_void_star &output_items) {

	d_sample_counter++;

	std::map<int,gnss_pseudorange> gnss_pseudoranges_map;
	std::map<int,gnss_pseudorange>::iterator gnss_pseudoranges_iter;

	gnss_pseudorange **in = (gnss_pseudorange **)  &input_items[0]; //Get the input pointer

	for (unsigned int i=0;i<d_nchannels;i++)
	{
		if (in[i][0].valid==true)
		{
			gnss_pseudoranges_map.insert(std::pair<int,gnss_pseudorange>(in[i][0].SV_ID,in[i][0])); //record the valid pseudorange in a map
		}
	}

	//debug print
	std::cout << std::setprecision(16);
	for(gnss_pseudoranges_iter = gnss_pseudoranges_map.begin();
			gnss_pseudoranges_iter != gnss_pseudoranges_map.end();
			gnss_pseudoranges_iter++)
	{
		std::cout<<"Pseudoranges(SV ID,pseudorange [m]) =("<<gnss_pseudoranges_iter->first<<","<<gnss_pseudoranges_iter->second.pseudorange_m<<")"<<std::endl;
	}


	// ############ 1. READ EPHEMERIS FROM QUEUE ######################
    // find the minimum index (nearest satellite, will be the reference)
	gnss_pseudoranges_iter=std::min_element(gnss_pseudoranges_map.begin(),gnss_pseudoranges_map.end(),pseudoranges_pairCompare_min);

	gps_navigation_message nav_msg;
	while (d_nav_queue->try_pop(nav_msg)==true)
	{
		std::cout<<"New ephemeris record has arrived from SAT ID "<<nav_msg.d_satellite_PRN<<std::endl;
		d_last_nav_msg=nav_msg;
		d_ls_pvt->d_ephemeris[nav_msg.d_channel_ID]=nav_msg;
		// **** update pseudoranges clock ****
		if (nav_msg.d_satellite_PRN==gnss_pseudoranges_iter->second.SV_ID)
		{
			d_ephemeris_clock_s=d_last_nav_msg.d_TOW;
			d_ephemeris_timestamp_ms=d_last_nav_msg.d_subframe1_timestamp_ms;
		}
		// **** write ephemeris to RINES NAV file
		//d_rinex_printer.LogRinex2Nav(nav_msg);
	}

	// ############ 2. COMPUTE THE PVT ################################
	// write the pseudoranges to RINEX OBS file
	// 1- need a valid clock
	if (d_ephemeris_clock_s>0 and d_last_nav_msg.d_satellite_PRN>0)
	{
		//d_rinex_printer.LogRinex2Obs(d_last_nav_msg,d_ephemeris_clock_s+((double)pseudoranges_timestamp_ms-d_ephemeris_timestamp_ms)/1000.0,pseudoranges);
		// compute on the fly PVT solution
		//std::cout<<"diff_clock_ephemerids="<<(gnss_pseudoranges_iter->second.timestamp_ms-d_ephemeris_timestamp_ms)/1000.0<<"\r\n";
		if (d_ls_pvt->get_PVT(gnss_pseudoranges_map,
				d_ephemeris_clock_s+(gnss_pseudoranges_iter->second.timestamp_ms-d_ephemeris_timestamp_ms)/1000.0,
				d_flag_averaging)==true)
		{
			d_kml_dump.print_position(d_ls_pvt,d_flag_averaging);
		}
	}

	consume_each(1); //one by one
	return 0;
}


