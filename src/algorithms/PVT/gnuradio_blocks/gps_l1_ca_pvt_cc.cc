/*!
 * \file gps_l1_ca_pvt_cc.cc
 * \brief Implementation of a Position Velocity and Time computation block for GPS L1 C/A
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

#include "gps_l1_ca_pvt_cc.h"
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
#include "control_message_factory.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "gnss_synchro.h"

using google::LogMessage;


gps_l1_ca_pvt_cc_sptr
gps_l1_ca_make_pvt_cc(unsigned int nchannels, gr_msg_queue_sptr queue, bool dump, std::string dump_filename, int averaging_depth, bool flag_averaging, int output_rate_ms, int display_rate_ms, bool flag_nmea_tty_port, std::string nmea_dump_filename, std::string nmea_dump_devname)
{
    return gps_l1_ca_pvt_cc_sptr(new gps_l1_ca_pvt_cc(nchannels, queue, dump, dump_filename, averaging_depth, flag_averaging, output_rate_ms, display_rate_ms, flag_nmea_tty_port, nmea_dump_filename, nmea_dump_devname));
}


gps_l1_ca_pvt_cc::gps_l1_ca_pvt_cc(unsigned int nchannels, gr_msg_queue_sptr queue, bool dump, std::string dump_filename, int averaging_depth, bool flag_averaging, int output_rate_ms, int display_rate_ms, bool flag_nmea_tty_port, std::string nmea_dump_filename, std::string nmea_dump_devname) :
		                		        gr_block ("gps_l1_ca_pvt_cc", gr_make_io_signature (nchannels, nchannels,  sizeof(Gnss_Synchro)),
		                		                gr_make_io_signature(1, 1, sizeof(gr_complex)))
{

    d_output_rate_ms = output_rate_ms;
    d_display_rate_ms = display_rate_ms;
    d_queue = queue;
    d_dump = dump;
    d_nchannels = nchannels;
    d_dump_filename = dump_filename;
    std::string dump_ls_pvt_filename = dump_filename;

    //initialize kml_printer
    std::string kml_dump_filename;
    kml_dump_filename = d_dump_filename;
    kml_dump_filename.append(".kml");
    d_kml_dump.set_headers(kml_dump_filename);

    //initialize nmea_printer
    d_nmea_printer = new Nmea_Printer(nmea_dump_filename, flag_nmea_tty_port, nmea_dump_devname);

    d_dump_filename.append("_raw.dat");
    dump_ls_pvt_filename.append("_ls_pvt.dat");
    d_averaging_depth = averaging_depth;
    d_flag_averaging = flag_averaging;

    d_ls_pvt = new gps_l1_ca_ls_pvt(nchannels,dump_ls_pvt_filename,d_dump);
    d_ls_pvt->set_averaging_depth(d_averaging_depth);
    d_ephemeris_clock_s = 0.0;

    d_sample_counter = 0;
    d_last_sample_nav_output=0;
    d_tx_time=0.0;

    b_rinex_header_writen = false;
    rp = new Rinex_Printer();

    for (unsigned int i=0; i<nchannels; i++)
        {
            nav_data_map[i] = Gps_Navigation_Message();
        }

    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                    {
                            d_dump_file.exceptions (std::ifstream::failbit | std::ifstream::badbit );
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            std::cout << "PVT dump enabled Log file: " << d_dump_filename.c_str() << std::endl;
                    }
                    catch (std::ifstream::failure e)
                    {
                            std::cout << "Exception opening PVT dump file " << e.what() << std::endl;
                    }
                }
        }

}



gps_l1_ca_pvt_cc::~gps_l1_ca_pvt_cc()
{
    d_kml_dump.close_file();
    delete d_ls_pvt;
    delete rp;
    delete d_nmea_printer;
}



bool pseudoranges_pairCompare_min( std::pair<int,Gnss_Synchro> a, std::pair<int,Gnss_Synchro> b)
{
    return (a.second.Pseudorange_m) < (b.second.Pseudorange_m);
}



int gps_l1_ca_pvt_cc::general_work (int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,	gr_vector_void_star &output_items)
{
    d_sample_counter++;

    std::map<int,Gnss_Synchro> gnss_pseudoranges_map;
    std::map<int,double> pseudoranges;
    std::map<int,Gnss_Synchro>::iterator gnss_pseudoranges_iter;

    Gnss_Synchro **in = (Gnss_Synchro **)  &input_items[0]; //Get the input pointer

    for (unsigned int i=0; i<d_nchannels; i++)
        {
            if (in[i][0].Flag_valid_pseudorange == true)
                {
                    gnss_pseudoranges_map.insert(std::pair<int,Gnss_Synchro>(in[i][0].PRN, in[i][0])); // store valid pseudoranges in a map
                }
        }

    for(gnss_pseudoranges_iter = gnss_pseudoranges_map.begin();
            gnss_pseudoranges_iter != gnss_pseudoranges_map.end();
            gnss_pseudoranges_iter++)
        {
            double pr = gnss_pseudoranges_iter->second.Pseudorange_m;
            pseudoranges[gnss_pseudoranges_iter->first] = pr;
        }

    // ############ 1. READ EPHEMERIS FROM QUEUE ######################
    // find the minimum index (nearest satellite, will be the reference)
    gnss_pseudoranges_iter = std::min_element(gnss_pseudoranges_map.begin(), gnss_pseudoranges_map.end(), pseudoranges_pairCompare_min);

    Gps_Navigation_Message nav_msg;
    while (d_nav_queue->try_pop(nav_msg) == true)
        {
            std::cout << "New ephemeris record has arrived from SAT ID "
                    << nav_msg.i_satellite_PRN << " (Block "
                    <<  nav_msg.satelliteBlock[nav_msg.i_satellite_PRN]
                                               << ")" << std::endl;
            d_last_nav_msg = nav_msg;
            if (nav_msg.b_valid_ephemeris_set_flag == true)
                {
                    d_ls_pvt->d_ephemeris[nav_msg.i_channel_ID] = nav_msg;
                    nav_data_map[nav_msg.i_channel_ID] = nav_msg;
                }
            // **** update pseudoranges clock ****
            if (nav_msg.i_satellite_PRN == gnss_pseudoranges_iter->second.PRN)
                {
                    d_ephemeris_clock_s = d_last_nav_msg.d_TOW;
                    d_ephemeris_timestamp_ms = d_last_nav_msg.d_subframe_timestamp_ms;
                }
        }

    // ############ 2. COMPUTE THE PVT ################################
    // write the pseudoranges to RINEX OBS file
    // 1- need a valid clock
    if (d_ephemeris_clock_s > 0 and d_last_nav_msg.i_satellite_PRN > 0 and d_last_nav_msg.b_valid_ephemeris_set_flag == true)
        {
            double clock_error;
            double satellite_tx_time_using_timestamps;
            //for GPS L1 C/A: t_tx = TOW + N_symbols_from_TOW*T_symbol
            //Notice that the TOW is decoded AFTER processing the subframe -> we need to add ONE subframe duration to t_tx
            d_tx_time = d_ephemeris_clock_s + gnss_pseudoranges_iter->second.Pseudorange_symbol_shift/(double)GPS_CA_TELEMETRY_RATE_SYMBOLS_SECOND + GPS_SUBFRAME_SECONDS;
            //Perform an extra check to verify the TOW update (the ephemeris queue is ASYNCHRONOUS to the GNU Radio Gnss_Synchro sample stream)
            //-> compute the t_tx_timestamps using the symbols timestamp (it is affected by code Doppler, but it is not wrapped like N_symbols_from_TOW)
            satellite_tx_time_using_timestamps = d_ephemeris_clock_s + (gnss_pseudoranges_iter->second.Pseudorange_timestamp_ms - d_ephemeris_timestamp_ms)/1000.0;
            //->compute the absolute error between both T_tx
            clock_error = std::abs(d_tx_time - satellite_tx_time_using_timestamps);
            // -> The symbol counter N_symbols_from_TOW will be reset every new received telemetry word, if the TOW is not updated, both t_tx and t_tx_timestamps times will difer by more than 1 seconds.
            if (clock_error < 1)
                {
                    // compute on the fly PVT solution
                    //mod 8/4/2012 Set the PVT computation rate in this block
                    if ((d_sample_counter % d_output_rate_ms) == 0)
                        {
                            if (d_ls_pvt->get_PVT(gnss_pseudoranges_map,d_tx_time,d_flag_averaging) == true)
                                {
                                    d_kml_dump.print_position(d_ls_pvt, d_flag_averaging);
                                    d_nmea_printer->Print_Nmea_Line(d_ls_pvt, d_flag_averaging);

                                    if (!b_rinex_header_writen) //  & we have utc data in nav message!
                                        {
                                            rp->rinex_nav_header(rp->navFile, d_last_nav_msg);
                                            rp->rinex_obs_header(rp->obsFile, d_last_nav_msg);
                                            b_rinex_header_writen = true; // do not write header anymore
                                        }
                                    if(b_rinex_header_writen) // Put here another condition to separate annotations (e.g 30 s)
                                        {
                                    	    // Limit the RINEX navigation output rate to 1/6 seg
                                    		// Notice that d_sample_counter period is 1ms (for GPS correlators)

                                    		if ((d_sample_counter-d_last_sample_nav_output)>=6000)
                                    		{
                                    			rp->log_rinex_nav(rp->navFile, nav_data_map);
                                    			d_last_sample_nav_output=d_sample_counter;
                                    		}
                                            rp->log_rinex_obs(rp->obsFile, d_last_nav_msg, d_tx_time, pseudoranges);
                                        }
                                }
                        }

                    if (((d_sample_counter % d_display_rate_ms) == 0) and d_ls_pvt->b_valid_position == true)
                        {
                            std::cout << "Position at " << boost::posix_time::to_simple_string(d_ls_pvt->d_position_UTC_time)
                            << " is Lat = " << d_ls_pvt->d_latitude_d << " [deg], Long = " << d_ls_pvt->d_longitude_d
                            << " [deg], Height= " << d_ls_pvt->d_height_m << " [m]" << std::endl;

                            std::cout << "Dilution of Precision at " << boost::posix_time::to_simple_string(d_ls_pvt->d_position_UTC_time)
                            << " is HDOP = " << d_ls_pvt->d_HDOP << " and VDOP = " << d_ls_pvt->d_VDOP << std::endl;
                        }

                    if(d_dump == true)
                        {
                            // MULTIPLEXED FILE RECORDING - Record results to file
                            try
                            {
                                    double tmp_double;
                                    for (unsigned int i=0; i<d_nchannels ; i++)
                                        {
                                            tmp_double = in[i][0].Pseudorange_m;
                                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                                            tmp_double = in[i][0].Pseudorange_symbol_shift;
                                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                                            d_dump_file.write((char*)&d_tx_time, sizeof(double));
                                        }
                            }
                            catch (std::ifstream::failure e)
                            {
                                    std::cout << "Exception writing observables dump file " << e.what() << std::endl;
                            }
                        }
                }
        }
    consume_each(1); //one by one
    return 0;
}


