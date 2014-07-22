/*!
 * \file hybrid_pvt_cc.cc
 * \brief Implementation of a Position Velocity and Time computation block for GPS L1 C/A
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2014  (see AUTHORS file for a list of contributors)
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

#include "hybrid_pvt_cc.h"
#include <algorithm>
#include <bitset>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <gnuradio/gr_complex.h>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include "control_message_factory.h"
#include "gnss_synchro.h"
#include "concurrent_map.h"

using google::LogMessage;

extern concurrent_map<Galileo_Ephemeris> global_galileo_ephemeris_map;
extern concurrent_map<Galileo_Iono> global_galileo_iono_map;
extern concurrent_map<Galileo_Utc_Model> global_galileo_utc_model_map;

extern concurrent_map<Gps_Ephemeris> global_gps_ephemeris_map;
extern concurrent_map<Gps_Iono> global_gps_iono_map;
extern concurrent_map<Gps_Utc_Model> global_gps_utc_model_map;

hybrid_pvt_cc_sptr
hybrid_make_pvt_cc(unsigned int nchannels, boost::shared_ptr<gr::msg_queue> queue, bool dump, std::string dump_filename, int averaging_depth, bool flag_averaging, int output_rate_ms, int display_rate_ms, bool flag_nmea_tty_port, std::string nmea_dump_filename, std::string nmea_dump_devname)
{
    return hybrid_pvt_cc_sptr(new hybrid_pvt_cc(nchannels, queue, dump, dump_filename, averaging_depth, flag_averaging, output_rate_ms, display_rate_ms, flag_nmea_tty_port, nmea_dump_filename, nmea_dump_devname));
}


hybrid_pvt_cc::hybrid_pvt_cc(unsigned int nchannels, boost::shared_ptr<gr::msg_queue> queue, bool dump, std::string dump_filename, int averaging_depth, bool flag_averaging, int output_rate_ms, int display_rate_ms, bool flag_nmea_tty_port, std::string nmea_dump_filename, std::string nmea_dump_devname) :
		                		                gr::block("hybrid_pvt_cc", gr::io_signature::make(nchannels, nchannels,  sizeof(Gnss_Synchro)),
		                		                        gr::io_signature::make(1, 1, sizeof(gr_complex)))
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

    d_ls_pvt = new hybrid_ls_pvt(nchannels, dump_ls_pvt_filename, d_dump);
    d_ls_pvt->set_averaging_depth(d_averaging_depth);

    d_sample_counter = 0;
    valid_solution_counter = 0;
    d_last_sample_nav_output = 0;
    d_rx_time = 0.0;
    d_TOW_at_curr_symbol_constellation = 0.0;
    b_rinex_header_writen = false;
    rp = new Rinex_Printer();

    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                    {
                            d_dump_file.exceptions (std::ifstream::failbit | std::ifstream::badbit );
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "PVT dump enabled Log file: " << d_dump_filename.c_str();
                    }
                    catch (const std::ifstream::failure& e)
                    {
                            LOG(WARNING) << "Exception opening PVT dump file " << e.what();
                    }
                }
        }
}



hybrid_pvt_cc::~hybrid_pvt_cc()
{
    d_kml_dump.close_file();
    delete d_ls_pvt;
    delete rp;
    delete d_nmea_printer;
}



bool hybrid_pvt_cc::pseudoranges_pairCompare_min( std::pair<int,Gnss_Synchro> a, std::pair<int,Gnss_Synchro> b)
{
    return (a.second.Pseudorange_m) < (b.second.Pseudorange_m);
}



int hybrid_pvt_cc::general_work (int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,	gr_vector_void_star &output_items)
{
    d_sample_counter++;

    std::map<int,Gnss_Synchro> gnss_pseudoranges_map;

    Gnss_Synchro **in = (Gnss_Synchro **)  &input_items[0]; //Get the input pointer

    for (unsigned int i = 0; i < d_nchannels; i++)
        {
            if (in[i][0].Flag_valid_pseudorange == true)
                {
                    gnss_pseudoranges_map.insert(std::pair<int,Gnss_Synchro>(in[i][0].PRN, in[i][0])); // store valid pseudoranges in a map
                    //d_rx_time = in[i][0].d_TOW_at_current_symbol; // all the channels have the same RX timestamp (common RX time pseudoranges)
                    d_TOW_at_curr_symbol_constellation=in[i][0].d_TOW_at_current_symbol; // d_TOW_at_current_symbol not corrected by delta t (just for debug)
                    d_rx_time = in[i][0].d_TOW_hybrid_at_current_symbol; // hybrid rx time, all the channels have the same RX timestamp (common RX time pseudoranges)
                    //std::cout<<"CH PVT = "<< i  << ", d_TOW = " << d_TOW_at_curr_symbol_constellation<<", rx_time_hybrid_PVT = " << d_rx_time << " same RX timestamp (common RX time pseudoranges)"<< std::endl;

                }
        }

    // ############ 1. READ GALILEO EPHEMERIS/UTC_MODE/IONO FROM GLOBAL MAPS ####

    if (global_galileo_ephemeris_map.size() > 0)
        {
            d_ls_pvt->galileo_ephemeris_map = global_galileo_ephemeris_map.get_map_copy();
        }

    if (global_galileo_utc_model_map.size() > 0)
        {
            // UTC MODEL data is shared for all the Galileo satellites. Read always at ID=0
            global_galileo_utc_model_map.read(0, d_ls_pvt->galileo_utc_model);
        }

    if (global_galileo_iono_map.size() > 0)
        {
            // IONO data is shared for all the Galileo satellites. Read always at ID=0
            global_galileo_iono_map.read(0, d_ls_pvt->galileo_iono);
        }

    // ############ 1. READ GPS EPHEMERIS/UTC_MODE/IONO FROM GLOBAL MAPS ####

    if (global_gps_ephemeris_map.size() > 0)
        {
            d_ls_pvt->gps_ephemeris_map = global_gps_ephemeris_map.get_map_copy();
        }

    if (global_gps_utc_model_map.size() > 0)
        {
            // UTC MODEL data is shared for all the Galileo satellites. Read always at ID=0
            global_gps_utc_model_map.read(0, d_ls_pvt->gps_utc_model);
        }

    if (global_gps_iono_map.size() > 0)
        {
            // IONO data is shared for all the Galileo satellites. Read always at ID=0
            global_gps_iono_map.read(0, d_ls_pvt->gps_iono);
        }


    // ############ 2 COMPUTE THE PVT ################################
    // ToDo: relax this condition because the receiver shuld work even with NO GALILEO SATELLITES
    //if (gnss_pseudoranges_map.size() > 0 and d_ls_pvt->galileo_ephemeris_map.size() > 0 and d_ls_pvt->gps_ephemeris_map.size() > 0)
    if (gnss_pseudoranges_map.size() > 0)
        {
    	//std::cout << "Both GPS and Galileo ephemeris map have been filled " << std::endl;
            // compute on the fly PVT solution
            if ((d_sample_counter % d_output_rate_ms) == 0)
                {
                    //bool pvt_result;
                    d_ls_pvt->get_PVT(gnss_pseudoranges_map, d_rx_time, d_flag_averaging);
                    //std::cout << "pvt_result  = " << pvt_result  << std::endl;

                    if (d_ls_pvt->b_valid_position == true)
                        {
                    	    valid_solution_counter ++;


                    		d_kml_dump.print_position_hybrid(d_ls_pvt, d_flag_averaging);

                    		/* *********************************** COMPUTE STATISTICS OVER THE FIRST 1000 SOLUTOINS **********************************/
									  if (valid_solution_counter<=1000)
                    					//if (d_ls_pvt->d_valid_observations >= 14)
                    					//if	(d_ls_pvt->d_valid_GPS_obs >= 7 or d_ls_pvt->d_valid_GAL_obs >=7)
									  {
										  GDOP_vect.push_back (d_ls_pvt->d_GDOP);
										  //PDOP_vect.push_back (d_ls_pvt->d_PDOP);

										  X_vect.push_back (d_ls_pvt->X); // add the last X value to X_vect
										  Y_vect.push_back (d_ls_pvt->Y); // add the last X value to X_vect
										  Z_vect.push_back (d_ls_pvt->Z);

										  longitude_vect.push_back (d_ls_pvt->d_longitude_d);
										  latitude_vect.push_back (d_ls_pvt->d_latitude_d);
										  h_vect.push_back (d_ls_pvt->d_height_m);

										  tot_obs_vect.push_back (d_ls_pvt->d_valid_observations);
										  Gal_obs_vect.push_back (d_ls_pvt->d_valid_GAL_obs);
										  GPS_obs_vect.push_back (d_ls_pvt->d_valid_GPS_obs);

										  time_vect.push_back (d_ls_pvt->d_position_UTC_time);
										  //valid_solution_16_sat_counter ++;
									  }

                    					//if(valid_solution_16_sat_counter ==1000)

									  if (valid_solution_counter==1000)
									  {
										  //compute ECEF average and standard deviation over the first 1000 solutions
										  GDOP_sum=std::accumulate( GDOP_vect.begin(),  GDOP_vect.end(), 0.0);
										  GDOP_mean =  GDOP_sum /  GDOP_vect.size();

										  longitude_vect_sum = std::accumulate( longitude_vect.begin(),  longitude_vect.end(), 0.0);
										  longitude_mean =  longitude_vect_sum /  longitude_vect.size();

										  //IFEN true solutions
										  double ref_longitude= 11.80800563;
										  double ref_latitude= 48.17149767;

										  double ref_X=4171691.011;
										  double ref_Y=872120.003;
										  double ref_Z=4730005.761;

//										  //REAL CAPTURE reference solutions (obtained with 4 Galileo)
//										  double ref_longitude= 1.987686994;
//										  double ref_latitude= 41.274786935;
//										  double ref_X=4797680.560;
//										  double ref_Y= 166506.414;
//										  double ref_Z=4185453.947;

										  //compute mean value for precision
										  latitude_vect_sum = std::accumulate( latitude_vect.begin(),  latitude_vect.end(), 0.0);
										  latitude_mean =  latitude_vect_sum /  latitude_vect.size();

										  X_vect_sum = std::accumulate(X_vect.begin(), X_vect.end(), 0.0);
										  X_vect_mean = X_vect_sum / X_vect.size();

										  X_vect_sq_sum = std::inner_product(X_vect.begin(), X_vect.end(), X_vect.begin(), 0.0);
										  X_vect_stdev = std::sqrt(X_vect_sq_sum / X_vect.size() - X_vect_mean * X_vect_mean);

										  Y_vect_sum = std::accumulate(Y_vect.begin(), Y_vect.end(), 0.0);
										  Y_vect_mean = Y_vect_sum / Y_vect.size();


										  Y_vect_sq_sum = std::inner_product(Y_vect.begin(), Y_vect.end(), Y_vect.begin(), 0.0);
										  Y_vect_stdev = std::sqrt(Y_vect_sq_sum / Y_vect.size() - Y_vect_mean * Y_vect_mean);

										  Z_vect_sum = std::accumulate(Z_vect.begin(), Z_vect.end(), 0.0);
										  Z_vect_mean = Z_vect_sum / Z_vect.size();

										  Z_vect_sq_sum = std::inner_product(Z_vect.begin(), Z_vect.end(), Z_vect.begin(), 0.0);
										  Z_vect_stdev = std::sqrt(Z_vect_sq_sum / Z_vect.size() - Z_vect_mean * Z_vect_mean);


									  std::fstream file_solutions;
									  std::ofstream file_statitics;

									  //std::fstream file_RES_X;
									  file_solutions.open ("GNSS_SDR_solutions.txt", std::fstream::out);
									  //file_statitics.open ("GNSS_SDR_statitics.txt", std::ofstream::out | std::ofstream::app);
									  file_statitics.open ("GNSS_SDR_statitics.txt", std::ofstream::out);

									  file_solutions.setf(file_solutions.fixed, file_solutions.floatfield);
									  file_statitics.setf(file_statitics.fixed, file_statitics.floatfield);


									  file_statitics << std::setprecision(9);

									  file_solutions << std::setw(13)<< "time" << std::setw(26)<<"X [m]" << std::setw(20)<< "Y [m]"<< std::setw(20)<< "Z [m]" <<     // X Y Z solutions
											  std::setw(20)<<"Long [deg]" << std::setw(15)<< "Lat [deg]"<< std::setw(15)<< "h [m]"<<      // long, lat, h solutions
											  std::setw(18)<<"E(Acc) [m]" << std::setw(18)<< "N(Acc) [m]"<<std::setw(18)<<"Up(Acc) [m]"<<  //ENU residual for accuracy columns 7:8:9
											  std::setw(18)<<"E(Pre) [m]" << std::setw(18)<< "N(Pre) [m]"<<std::setw(18)<<"Up(Pre) [m]"<<  //ENU residual  for precision columns 10:11:12
											  std::setw(18)<<"Tot Sat" << std::setw(18)<< "Gal"<<std::setw(18)<<"GPS"<<					  //number of satellites
											  std::setw(18)<<"GDOP" <<std::endl;														  //GDOP



									  std::cout << "X mean over 1000 solutions [m]= " << X_vect_mean << ", X dev st over 1000 solutins=  " << X_vect_stdev << std::endl;
									  std::cout << "Y mean over 1000 solutions [m]= " << Y_vect_mean << ", Y dev st over 1000 solutins= " << Y_vect_stdev << std::endl;
									  std::cout << "Z mean over 1000 solutions [m]= " << Z_vect_mean << ", Z dev st over 1000 solutins= " << Z_vect_stdev << std::endl;

									  //file_statitics << "X mean over 1000 solutions [m]= " << X_vect_mean << ", X dev st over 1000 solutins=  " << X_vect_stdev << std::endl;
									  //file_statitics << "Y mean over 1000 solutions [m]= " << Y_vect_mean << ", Y dev st over 1000 solutins= " << Y_vect_stdev << std::endl;
									  //file_statitics << "Z mean over 1000 solutions [m]= " << Z_vect_mean << ", Z dev st over 1000 solutins= " << Z_vect_stdev << std::endl << std::endl;

									  file_statitics << "Num of GPS observation " << d_ls_pvt->d_valid_GPS_obs <<  std::endl;
									  file_statitics << "Num of GALILEO observation " << d_ls_pvt->d_valid_GAL_obs <<  std::endl;

									  file_statitics << "GDOP mean= " << GDOP_mean << std::endl << std::endl;

									  file_statitics << "ENU computed at (IFEN true coordinates): ref Longitude = " << ref_longitude << ", Ref Latitude = " << ref_latitude << " for Accuracy"<<std::endl;
									  file_statitics << "ENU computed at (average coordinates) mean Longitude = " << longitude_mean << ", mean Latitude = " << latitude_mean << " for Precision"<<std::endl;

									  //file_statitics << "Residual computed with respect (IFEN true coordinates) X Y Z= " << ref_X << '\t'<< ref_Y << '\t'<< ref_Z << std::endl;


									  // 1- Rotation matrix from ECEF coordinates to ENU coordinates
									  // ref: http://www.navipedia.net/index.php/Transformations_between_ECEF_and_ENU_coordinates
									  arma::mat R = arma::zeros(3,3);//matrix with REFERENCE position
									  arma::mat M = arma::zeros(3,3);//matrix with MEAN position

									  //matrix with MEAN position
									  M(0,0) = -sin(GPS_TWO_PI*(longitude_mean/360.0));
									  M(0,1) = -sin(GPS_TWO_PI*(latitude_mean/360.0))*cos(GPS_TWO_PI*(longitude_mean/360.0));
									  M(0,2) = cos(GPS_TWO_PI*(latitude_mean/360.0))*cos(GPS_TWO_PI*(longitude_mean/360.0));

									  M(1,0) = cos((GPS_TWO_PI*longitude_mean)/360.0);
									  M(1,1) = -sin((GPS_TWO_PI*latitude_mean)/360.0)*sin((GPS_TWO_PI*longitude_mean)/360.0);
									  M(1,2) = cos((GPS_TWO_PI*latitude_mean/360.0))*sin((GPS_TWO_PI*longitude_mean)/360.0);

									  M(2,0) = 0;
									  M(2,1) = cos((GPS_TWO_PI*latitude_mean)/360.0);
									  M(2,2) = sin((GPS_TWO_PI*latitude_mean/360.0));

									  //matrix with REFERENCE position
									  R(0,0) = -sin(GPS_TWO_PI*(ref_longitude/360.0));
									  R(0,1) = -sin(GPS_TWO_PI*(ref_latitude/360.0))*cos(GPS_TWO_PI*(ref_longitude/360.0));
									  R(0,2) = cos(GPS_TWO_PI*(ref_latitude/360.0))*cos(GPS_TWO_PI*(ref_longitude/360.0));

									  R(1,0) = cos((GPS_TWO_PI*ref_longitude)/360.0);
									  R(1,1) = -sin((GPS_TWO_PI*ref_latitude)/360.0)*sin((GPS_TWO_PI*ref_longitude)/360.0);
									  R(1,2) = cos((GPS_TWO_PI*ref_latitude/360.0))*sin((GPS_TWO_PI*ref_longitude)/360.0);

									  R(2,0) = 0;
									  R(2,1) = cos((GPS_TWO_PI*ref_latitude)/360.0);
									  R(2,2) = sin((GPS_TWO_PI*ref_latitude/360.0));



									  for(unsigned i=0; i<X_vect.size(); i++ )
											  {
											  //compute residual with respect to the REFERENCE SOLUTIONS (true IFEN coordinates)
											  X_vect_res.push_back (X_vect[i] - ref_X); //accuracy
											  Y_vect_res.push_back (Y_vect[i] - ref_Y); //accuracy
											  Z_vect_res.push_back (Z_vect[i] - ref_Z); //accuracy

											  //compute residual with respect to the MEAN SOLUTIONS
											  X_vect_res_precision.push_back (X_vect[i] - X_vect_mean); //precision
											  Y_vect_res_precision.push_back (Y_vect[i] - Y_vect_mean); //precision
											  Z_vect_res_precision.push_back (Z_vect[i] - Z_vect_mean); //precision

											  //apply to residual value the rotation from ECEF to ENU IN REFERENCE POSITION (matrix R)
											  E_res.push_back (R(0,0)*X_vect_res[i] +  R(1,0)*Y_vect_res[i] + R(2,0) * Z_vect_res[i]);
											  N_res.push_back (R(0,1)*X_vect_res[i] +  R(1,1)*Y_vect_res[i] + R(2,1) * Z_vect_res[i]);
											  Up_res.push_back (R(0,2)*X_vect_res[i] +  R(1,2)*Y_vect_res[i] + R(2,2) * Z_vect_res[i]);

											  //apply to residual value the rotation from ECEF to ENU MEAN POSITOIN (matrix R)
											  E_res_precision.push_back (M(0,0)*X_vect_res_precision[i] +  M(1,0)*Y_vect_res_precision[i] + M(2,0) * Z_vect_res_precision[i]);
											  N_res_precision.push_back (M(0,1)*X_vect_res_precision[i] +  M(1,1)*Y_vect_res_precision[i] + M(2,1) * Z_vect_res_precision[i]);
											  Up_res_precision.push_back (M(0,2)*X_vect_res_precision[i] +  M(1,2)*Y_vect_res_precision[i] + M(2,2) * Z_vect_res_precision[i]);

											  file_solutions << std::setprecision(5);
											  file_solutions << std::setw(13)<< time_vect[i]<<
													  	  	   std::setw(22)<< X_vect[i]<< std::setw(20)<< Y_vect[i]<< std::setw(20)<< Z_vect[i] <<                                                           	// X Y Z solutions
													           std::setw(15)<< longitude_vect [i]<<std::setw(15)<< latitude_vect [i]<<  std::setw(15)<< h_vect [i]<<                                        	// long, lat, h solutions
													           std::setw(18)<< E_res[i]<< std::setw(18)<< N_res[i]<< std::setw(18)<< Up_res[i]<<																//ENU residual for accuracy columns 7:8:9
													           std::setw(18)<< E_res_precision[i]<< std::setw(18)<< N_res_precision[i]<< std::setw(18)<<Up_res_precision[i]<<  									//ENU residual  for precision columns 10:11:12
													           std::setw(18)<< std::setprecision(0)<<tot_obs_vect[i] <<  std::setw(18)<< std::setprecision(0)<<Gal_obs_vect[i] <<  std::setw(18)<<std::setprecision(0)<< GPS_obs_vect[i] <<   // number of satellites
													           std::setw(18)<<std::setprecision(2)<<GDOP_vect[i] <<std::endl;
											  }


										  //compute ENU average and standard deviation over the first 1000 residual solutions ACCURACY
										  E_res_sum = std::accumulate(E_res.begin(), E_res.end(), 0.0);
										  E_res_mean = E_res_sum / E_res.size();

										  E_res_sq_sum = std::inner_product(E_res.begin(), E_res.end(), E_res.begin(), 0.0);
										  E_res_stdev = std::sqrt(E_res_sq_sum / E_res.size() - E_res_mean * E_res_mean);

										  N_res_sum = std::accumulate(N_res.begin(), N_res.end(), 0.0);
										  N_res_mean = N_res_sum / N_res.size();

										  N_res_sq_sum = std::inner_product(N_res.begin(), N_res.end(), N_res.begin(), 0.0);
										  N_res_stdev = std::sqrt(N_res_sq_sum / N_res.size() - N_res_mean * N_res_mean);

										  Up_res_sum = std::accumulate(Up_res.begin(), Up_res.end(), 0.0);
										  Up_res_mean = Up_res_sum / Up_res.size();

										  Up_res_sq_sum = std::inner_product(Up_res.begin(), Up_res.end(), Up_res.begin(), 0.0);
										  Up_res_stdev = std::sqrt(Up_res_sq_sum / Up_res.size() - Up_res_mean * Up_res_mean);

										  //compute ENU average and standard deviation over the first 1000 residual solutions PRECISION
										  E_res_sum_precision = std::accumulate(E_res_precision.begin(), E_res_precision.end(), 0.0);
										  E_res_mean_precision = E_res_sum_precision / E_res_precision.size();

										  E_res_sq_sum_precision = std::inner_product(E_res_precision.begin(), E_res_precision.end(), E_res_precision.begin(), 0.0);
										  E_res_stdev_precision = std::sqrt(E_res_sq_sum_precision / E_res_precision.size() - E_res_mean_precision * E_res_mean_precision);

										  N_res_sum_precision = std::accumulate(N_res_precision.begin(), N_res_precision.end(), 0.0);
										  N_res_mean_precision = N_res_sum_precision / N_res_precision.size();

										  N_res_sq_sum_precision = std::inner_product(N_res_precision.begin(), N_res_precision.end(), N_res_precision.begin(), 0.0);
										  N_res_stdev_precision = std::sqrt(N_res_sq_sum_precision / N_res_precision.size() - N_res_mean_precision * N_res_mean_precision);

										  Up_res_sum_precision = std::accumulate(Up_res_precision.begin(), Up_res_precision.end(), 0.0);
										  Up_res_mean_precision = Up_res_sum_precision / Up_res_precision.size();

										  Up_res_sq_sum_precision = std::inner_product(Up_res_precision.begin(), Up_res_precision.end(), Up_res_precision.begin(), 0.0);
										  Up_res_stdev_precision = std::sqrt(Up_res_sq_sum_precision / Up_res_precision.size() - Up_res_mean_precision * Up_res_mean_precision);

										  DRMS = std::sqrt( E_res_stdev * E_res_stdev + N_res_stdev*N_res_stdev);
										  DUE_DRMS = 2*DRMS;
										  CEP = 0.62 * N_res_stdev + 0.56 * E_res_stdev;
										  MRSE = std::sqrt( E_res_stdev * E_res_stdev + N_res_stdev*N_res_stdev + Up_res_stdev*Up_res_stdev);
										  SEP = 0.51* (E_res_stdev * E_res_stdev + N_res_stdev*N_res_stdev + Up_res_stdev*Up_res_stdev);

										  file_statitics <<  std::endl;
										  file_statitics << "ACCURACY (respect true position)" << std::endl;
										  file_statitics << "East offset [m] = " << E_res_mean << ", East st. dev =  " << E_res_stdev << std::endl;
										  file_statitics << "Nord offset [m] = " << N_res_mean << ",Noth st. dev =  " << N_res_stdev << std::endl;
										  file_statitics << "Up offset [m] = " << Up_res_mean << ", Up st. dev =  " << Up_res_stdev << std::endl;
										  file_statitics <<  std::endl;
										  file_statitics <<  "DRMS= " << DRMS <<  std::endl;
										  file_statitics <<  "DUE_DRMS= " << DUE_DRMS <<  std::endl;
										  file_statitics <<  "CEP= " << CEP <<  std::endl;
										  file_statitics <<  "MRSE= " << MRSE <<  std::endl;
										  file_statitics <<  "SEP= " << SEP <<  std::endl;
										  file_statitics <<  std::endl;
										  file_statitics << "PRECISION (respect average solution)" << std::endl;
										  file_statitics << "East offset [m] = " << E_res_mean_precision << ", East st. dev = " << E_res_stdev_precision << std::endl;
										  file_statitics << "Nord offset [m] = " << N_res_mean_precision << ", ,Noth st. dev = " << N_res_stdev_precision << std::endl;
										  file_statitics << "Up offset  [m]= " << Up_res_mean_precision << ", Up st. dev = " << Up_res_stdev_precision << std::endl;
										  file_statitics << "----------------------------------------------------------------------------------------------" << std::endl;
										  file_solutions.close();
									      file_statitics.close();

									  }/* END *********************************** COMPUTE STATISTICS OVER THE FIRST 1000 SOLUTOINS **********************************/





                    		//ToDo: Implement Galileo RINEX and Galileo NMEA outputs
                            //                            d_nmea_printer->Print_Nmea_Line(d_ls_pvt, d_flag_averaging);
                            //
                            //                            if (!b_rinex_header_writen) //  & we have utc data in nav message!
                            //                                {
                            //                                    std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter;
                            //                                    gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin();
                            //                                    if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end())
                            //                                        {
                            //                                            rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second,d_rx_time);
                            //                                            rp->rinex_nav_header(rp->navFile,d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model);
                            //                                            b_rinex_header_writen = true; // do not write header anymore
                            //                                        }
                            //                                }
                            //                            if(b_rinex_header_writen) // Put here another condition to separate annotations (e.g 30 s)
                            //                                {
                            //                                    // Limit the RINEX navigation output rate to 1/6 seg
                            //                                    // Notice that d_sample_counter period is 1ms (for GPS correlators)
                            //                                    if ((d_sample_counter-d_last_sample_nav_output)>=6000)
                            //                                        {
                            //                                            rp->log_rinex_nav(rp->navFile, d_ls_pvt->gps_ephemeris_map);
                            //                                            d_last_sample_nav_output=d_sample_counter;
                            //                                        }
                            //                                    std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter;
                            //                                    gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin();
                            //                                    if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end())
                            //                                        {
                            //                                            rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, d_rx_time, gnss_pseudoranges_map);
                            //                                        }
                            //                                }
                        }
                }

            // DEBUG MESSAGE: Display position in console output
            if (((d_sample_counter % d_display_rate_ms) == 0) and d_ls_pvt->b_valid_position == true)
                {
                    std::cout << "Position at " << boost::posix_time::to_simple_string(d_ls_pvt->d_position_UTC_time)
                              << " using "<<d_ls_pvt->d_valid_observations<<" observations is Lat = " << d_ls_pvt->d_latitude_d << " [deg], Long = " << d_ls_pvt->d_longitude_d
                              << " [deg], Height= " << d_ls_pvt->d_height_m << " [m]" << std::endl;

                    LOG(INFO) << "Position at " << boost::posix_time::to_simple_string(d_ls_pvt->d_position_UTC_time)
                              << " using "<<d_ls_pvt->d_valid_observations<<" observations is Lat = " << d_ls_pvt->d_latitude_d << " [deg], Long = " << d_ls_pvt->d_longitude_d
                              << " [deg], Height= " << d_ls_pvt->d_height_m << " [m]";

                    LOG(INFO) << "Dilution of Precision at " << boost::posix_time::to_simple_string(d_ls_pvt->d_position_UTC_time)
                              << " using "<<d_ls_pvt->d_valid_observations<<" observations is HDOP = " << d_ls_pvt->d_HDOP << " VDOP = "
                              << d_ls_pvt->d_VDOP <<" TDOP = " << d_ls_pvt->d_TDOP
                              << " GDOP = " << d_ls_pvt->d_GDOP;





                }

            // MULTIPLEXED FILE RECORDING - Record results to file
            if(d_dump == true)
                {
                    try
                    {
                            double tmp_double;
                            for (unsigned int i = 0; i < d_nchannels; i++)
                                {
                                    tmp_double = in[i][0].Pseudorange_m;
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    tmp_double = 0;
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    d_dump_file.write((char*)&d_rx_time, sizeof(double));
                                }
                    }
                    catch (const std::ifstream::failure& e)
                    {
                            LOG(WARNING) << "Exception writing observables dump file " << e.what();
                    }
                }
        }

    consume_each(1); //one by one
    return 0;
}


