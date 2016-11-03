/*!
 * \file galileo_e1_ls_pvt.cc
 * \brief Implementation of a Least Squares Position, Velocity, and Time
 * (PVT) solver, based on K.Borre's Matlab receiver.
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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

#include "hybrid_ls_pvt.h"
#include <glog/logging.h>
#include "Galileo_E1.h"


using google::LogMessage;

hybrid_ls_pvt::hybrid_ls_pvt(int nchannels, std::string dump_filename, bool flag_dump_to_file) : Ls_Pvt()
{
    // init empty ephemeris for all the available GNSS channels
    d_nchannels = nchannels;
    d_dump_filename = dump_filename;
    d_flag_dump_enabled = flag_dump_to_file;
    d_galileo_current_time = 0;
    count_valid_position = 0;
    d_flag_averaging = false;
    // ############# ENABLE DATA FILE LOG #################
    if (d_flag_dump_enabled == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                    {
                            d_dump_file.exceptions (std::ifstream::failbit | std::ifstream::badbit);
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "PVT lib dump enabled Log file: " << d_dump_filename.c_str();
                    }
                    catch (const std::ifstream::failure &e)
                    {
                            LOG(WARNING) << "Exception opening PVT lib dump file " << e.what();
                    }
                }
        }
}


hybrid_ls_pvt::~hybrid_ls_pvt()
{
    d_dump_file.close();
}


bool hybrid_ls_pvt::get_PVT(std::map<int,Gnss_Synchro> gnss_pseudoranges_map, double hybrid_current_time, bool flag_averaging)
{
    std::map<int,Gnss_Synchro>::iterator gnss_pseudoranges_iter;
    std::map<int,Galileo_Ephemeris>::iterator galileo_ephemeris_iter;
    std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter;
    std::map<int,Gps_CNAV_Ephemeris>::iterator gps_cnav_ephemeris_iter;

    int valid_pseudoranges = gnss_pseudoranges_map.size();

    arma::mat W = arma::eye(valid_pseudoranges, valid_pseudoranges); // channels weights matrix
    arma::vec obs = arma::zeros(valid_pseudoranges);                 // pseudoranges observation vector
    arma::mat satpos = arma::zeros(3, valid_pseudoranges);           // satellite positions matrix

    int Galileo_week_number = 0;
    int GPS_week = 0;
    double utc = 0.0;
    double GST = 0.0;
    //double utc_tx_corrected = 0.0; //utc computed at tx_time_corrected, added for Galileo constellation (in GPS utc is directly computed at TX_time_corrected_s)
    double TX_time_corrected_s = 0.0;
    double SV_clock_bias_s = 0.0;

    d_flag_averaging = flag_averaging;

    // ********************************************************************************
    // ****** PREPARE THE LEAST SQUARES DATA (SV POSITIONS MATRIX AND OBS VECTORS) ****
    // ********************************************************************************
    int valid_obs = 0; //valid observations counter
    int obs_counter = 0;

    for(gnss_pseudoranges_iter = gnss_pseudoranges_map.begin();
            gnss_pseudoranges_iter != gnss_pseudoranges_map.end();
            gnss_pseudoranges_iter++)
        {
            if(gnss_pseudoranges_iter->second.System == 'E')
                {
                    // 1 Gal - find the ephemeris for the current GALILEO SV observation. The SV PRN ID is the map key
                    galileo_ephemeris_iter = galileo_ephemeris_map.find(gnss_pseudoranges_iter->second.PRN);
                    if (galileo_ephemeris_iter != galileo_ephemeris_map.end())
                        {
                            /*!
                             * \todo Place here the satellite CN0 (power level, or weight factor)
                             */
                            W(obs_counter, obs_counter) = 1;

                            // COMMON RX TIME PVT ALGORITHM
                            double Rx_time = hybrid_current_time;
                            double Tx_time = Rx_time - gnss_pseudoranges_iter->second.Pseudorange_m / GALILEO_C_m_s;

                            // 2- compute the clock drift using the clock model (broadcast) for this SV
                            SV_clock_bias_s = galileo_ephemeris_iter->second.sv_clock_drift(Tx_time);

                            // 3- compute the current ECEF position for this SV using corrected TX time
                            TX_time_corrected_s = Tx_time - SV_clock_bias_s;
                            galileo_ephemeris_iter->second.satellitePosition(TX_time_corrected_s);

                            satpos(0,obs_counter) = galileo_ephemeris_iter->second.d_satpos_X;
                            satpos(1,obs_counter) = galileo_ephemeris_iter->second.d_satpos_Y;
                            satpos(2,obs_counter) = galileo_ephemeris_iter->second.d_satpos_Z;

                            // 5- fill the observations vector with the corrected pseudoranges
                            obs(obs_counter) = gnss_pseudoranges_iter->second.Pseudorange_m + SV_clock_bias_s * GALILEO_C_m_s;
                            d_visible_satellites_IDs[valid_obs] = galileo_ephemeris_iter->second.i_satellite_PRN;
                            d_visible_satellites_CN0_dB[valid_obs] = gnss_pseudoranges_iter->second.CN0_dB_hz;
                            valid_obs++;

                            Galileo_week_number = galileo_ephemeris_iter->second.WN_5; //for GST
                            GST = galileo_ephemeris_iter->second.Galileo_System_Time(Galileo_week_number, hybrid_current_time);

                            // SV ECEF DEBUG OUTPUT
                            DLOG(INFO) << "ECEF satellite SV ID=" << galileo_ephemeris_iter->second.i_satellite_PRN
                                    << " X=" << galileo_ephemeris_iter->second.d_satpos_X
                                    << " [m] Y=" << galileo_ephemeris_iter->second.d_satpos_Y
                                    << " [m] Z=" << galileo_ephemeris_iter->second.d_satpos_Z
                                    << " [m] PR_obs=" << obs(obs_counter) << " [m]";
                        }

                    else // the ephemeris are not available for this SV
                        {
                            // no valid pseudorange for the current SV
                            W(obs_counter, obs_counter) = 0; // SV de-activated
                            obs(obs_counter) = 1;            // to avoid algorithm problems (divide by zero)
                            DLOG(INFO) << "No ephemeris data for SV " << gnss_pseudoranges_iter->second.PRN;
                        }
                }

            else if(gnss_pseudoranges_iter->second.System == 'G')
                {
                    //std::cout << "Satellite System: " << gnss_pseudoranges_iter->second.System <<std::endl;
                    // 1 GPS - find the ephemeris for the current GPS SV observation. The SV PRN ID is the map key
                    std::string sig_(gnss_pseudoranges_iter->second.Signal);
                    if(sig_.compare("1C") == 0)
                    {
                        gps_ephemeris_iter = gps_ephemeris_map.find(gnss_pseudoranges_iter->second.PRN);
                        if (gps_ephemeris_iter != gps_ephemeris_map.end())
                            {
                                /*!
                                 * \todo Place here the satellite CN0 (power level, or weight factor)
                                 */
                                W(obs_counter, obs_counter) = 1;

                                // COMMON RX TIME PVT ALGORITHM MODIFICATION (Like RINEX files)
                                // first estimate of transmit time
                                double Rx_time = hybrid_current_time;
                                double Tx_time = Rx_time - gnss_pseudoranges_iter->second.Pseudorange_m / GPS_C_m_s;

                                // 2- compute the clock drift using the clock model (broadcast) for this SV
                                SV_clock_bias_s = gps_ephemeris_iter->second.sv_clock_drift(Tx_time);

                                // 3- compute the current ECEF position for this SV using corrected TX time
                                TX_time_corrected_s = Tx_time - SV_clock_bias_s;
                                gps_ephemeris_iter->second.satellitePosition(TX_time_corrected_s);

                                satpos(0, obs_counter) = gps_ephemeris_iter->second.d_satpos_X;
                                satpos(1, obs_counter) = gps_ephemeris_iter->second.d_satpos_Y;
                                satpos(2, obs_counter) = gps_ephemeris_iter->second.d_satpos_Z;

                                // 5- fill the observations vector with the corrected pseudoranges
                                obs(obs_counter) = gnss_pseudoranges_iter->second.Pseudorange_m + SV_clock_bias_s * GPS_C_m_s;
                                d_visible_satellites_IDs[valid_obs] = gps_ephemeris_iter->second.i_satellite_PRN;
                                d_visible_satellites_CN0_dB[valid_obs] = gnss_pseudoranges_iter->second.CN0_dB_hz;
                                valid_obs++;
                                GPS_week = gps_ephemeris_iter->second.i_GPS_week;

                                // SV ECEF DEBUG OUTPUT
                                DLOG(INFO) << "(new)ECEF satellite SV ID=" << gps_ephemeris_iter->second.i_satellite_PRN
                                        << " X=" << gps_ephemeris_iter->second.d_satpos_X
                                        << " [m] Y=" << gps_ephemeris_iter->second.d_satpos_Y
                                        << " [m] Z=" << gps_ephemeris_iter->second.d_satpos_Z
                                        << " [m] PR_obs=" << obs(obs_counter) << " [m]";
                            }
                        else // the ephemeris are not available for this SV
                            {
                                // no valid pseudorange for the current SV
                                W(obs_counter, obs_counter) = 0; // SV de-activated
                                obs(obs_counter) = 1;            // to avoid algorithm problems (divide by zero)
                                DLOG(INFO) << "No ephemeris data for SV " << gnss_pseudoranges_iter->second.PRN;
                            }
                    }
                    if(sig_.compare("2S") == 0)
                    {
                        gps_cnav_ephemeris_iter = gps_cnav_ephemeris_map.find(gnss_pseudoranges_iter->second.PRN);
                        if (gps_cnav_ephemeris_iter != gps_cnav_ephemeris_map.end())
                            {
                                /*!
                                 * \todo Place here the satellite CN0 (power level, or weight factor)
                                 */
                                W(obs_counter, obs_counter) = 1;

                                // COMMON RX TIME PVT ALGORITHM MODIFICATION (Like RINEX files)
                                // first estimate of transmit time
                                double Rx_time = hybrid_current_time;
                                double Tx_time = Rx_time - gnss_pseudoranges_iter->second.Pseudorange_m / GPS_C_m_s;

                                // 2- compute the clock drift using the clock model (broadcast) for this SV
                                SV_clock_bias_s = gps_cnav_ephemeris_iter->second.sv_clock_drift(Tx_time);

                                // 3- compute the current ECEF position for this SV using corrected TX time
                                TX_time_corrected_s = Tx_time - SV_clock_bias_s;
                                gps_cnav_ephemeris_iter->second.satellitePosition(TX_time_corrected_s);

                                satpos(0, obs_counter) = gps_cnav_ephemeris_iter->second.d_satpos_X;
                                satpos(1, obs_counter) = gps_cnav_ephemeris_iter->second.d_satpos_Y;
                                satpos(2, obs_counter) = gps_cnav_ephemeris_iter->second.d_satpos_Z;

                                // 5- fill the observations vector with the corrected pseudoranges
                                obs(obs_counter) = gnss_pseudoranges_iter->second.Pseudorange_m + SV_clock_bias_s * GPS_C_m_s;
                                d_visible_satellites_IDs[valid_obs] = gps_cnav_ephemeris_iter->second.i_satellite_PRN;
                                d_visible_satellites_CN0_dB[valid_obs] = gnss_pseudoranges_iter->second.CN0_dB_hz;
                                valid_obs++;
                                GPS_week = gps_cnav_ephemeris_iter->second.i_GPS_week;

                                // SV ECEF DEBUG OUTPUT
                                DLOG(INFO) << "(new)ECEF satellite SV ID=" << gps_cnav_ephemeris_iter->second.i_satellite_PRN
                                        << " X=" << gps_cnav_ephemeris_iter->second.d_satpos_X
                                        << " [m] Y=" << gps_cnav_ephemeris_iter->second.d_satpos_Y
                                        << " [m] Z=" << gps_cnav_ephemeris_iter->second.d_satpos_Z
                                        << " [m] PR_obs=" << obs(obs_counter) << " [m]";
                            }
                        else // the ephemeris are not available for this SV
                            {
                                // no valid pseudorange for the current SV
                                W(obs_counter, obs_counter) = 0; // SV de-activated
                                obs(obs_counter) = 1;            // to avoid algorithm problems (divide by zero)
                                DLOG(INFO) << "No ephemeris data for SV " << gnss_pseudoranges_iter->second.PRN;
                            }
                    }
                }
            obs_counter++;
        }

    // ********************************************************************************
    // ****** SOLVE LEAST SQUARES******************************************************
    // ********************************************************************************
    d_valid_observations = valid_obs;

    LOG(INFO) << "HYBRID PVT: valid observations=" << valid_obs;

    if(valid_obs >= 4)
        {
            arma::vec mypos;
            DLOG(INFO) << "satpos=" << satpos;
            DLOG(INFO) << "obs=" << obs;
            DLOG(INFO) << "W=" << W;

            mypos = leastSquarePos(satpos, obs, W);
            d_rx_dt_m = mypos(3) / GPS_C_m_s; // Convert RX time offset from meters to seconds
            double secondsperweek = 604800.0;
            // Compute GST and Gregorian time
            if( GST != 0.0)
                {
                    utc = galileo_utc_model.GST_to_UTC_time(GST, Galileo_week_number);
                }
            else
                {
                    utc = gps_utc_model.utc_time(TX_time_corrected_s, GPS_week) + secondsperweek * static_cast<double>(GPS_week);
                }

            // get time string Gregorian calendar
            boost::posix_time::time_duration t = boost::posix_time::seconds(utc);
            // 22 August 1999 00:00 last Galileo start GST epoch (ICD sec 5.1.2)
            boost::posix_time::ptime p_time(boost::gregorian::date(1999, 8, 22), t);
            d_position_UTC_time = p_time;

            DLOG(INFO) << "HYBRID Position at TOW=" << hybrid_current_time << " in ECEF (X,Y,Z) = " << mypos;

            cart2geo(static_cast<double>(mypos(0)), static_cast<double>(mypos(1)), static_cast<double>(mypos(2)), 4);
            //ToDo: Find an Observables/PVT random bug with some satellite configurations that gives an erratic PVT solution (i.e. height>50 km)
            if (d_height_m > 50000)
                {
                    b_valid_position = false;
                    LOG(INFO) << "Hybrid Position at " << boost::posix_time::to_simple_string(p_time)
                    << " is Lat = " << d_latitude_d << " [deg], Long = " << d_longitude_d
                    << " [deg], Height= " << d_height_m << " [m]" << " RX time offset= " << mypos(3) << " [s]";
                    return false;
                }

            LOG(INFO) << "Hybrid Position at " << boost::posix_time::to_simple_string(p_time)
            << " is Lat = " << d_latitude_d << " [deg], Long = " << d_longitude_d
            << " [deg], Height= " << d_height_m << " [m]" << " RX time offset= " << d_rx_dt_m << " [s]";

            // ###### Compute DOPs ########
            hybrid_ls_pvt::compute_DOP();

            // ######## LOG FILE #########
            if(d_flag_dump_enabled == true)
                {
                    // MULTIPLEXED FILE RECORDING - Record results to file
                    try
                    {
                            double tmp_double;
                            //  PVT GPS time
                            tmp_double = hybrid_current_time;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // ECEF User Position East [m]
                            tmp_double = mypos(0);
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // ECEF User Position North [m]
                            tmp_double = mypos(1);
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // ECEF User Position Up [m]
                            tmp_double = mypos(2);
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // User clock offset [s]
                            tmp_double = mypos(3);
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // GEO user position Latitude [deg]
                            tmp_double = d_latitude_d;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // GEO user position Longitude [deg]
                            tmp_double = d_longitude_d;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // GEO user position Height [m]
                            tmp_double = d_height_m;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                    }
                    catch (const std::ifstream::failure& e)
                    {
                            LOG(WARNING) << "Exception writing PVT LS dump file " << e.what();
                    }
                }

            // MOVING AVERAGE PVT
            pos_averaging(flag_averaging);
        }
    else
        {
            b_valid_position = false;
        }
    return b_valid_position;
}
