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

#include "galileo_e1_ls_pvt.h"
#include <glog/logging.h>
#include "Galileo_E1.h"


using google::LogMessage;

galileo_e1_ls_pvt::galileo_e1_ls_pvt(int nchannels, std::string dump_filename, bool flag_dump_to_file) : Ls_Pvt()
{
    // init empty ephemeris for all the available GNSS channels
    d_nchannels = nchannels;
    d_ephemeris = new Galileo_Navigation_Message[nchannels];
    d_dump_filename = dump_filename;
    d_flag_dump_enabled = flag_dump_to_file;
    d_galileo_current_time = 0;
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


galileo_e1_ls_pvt::~galileo_e1_ls_pvt()
{
    d_dump_file.close();
    delete[] d_ephemeris;
}



bool galileo_e1_ls_pvt::get_PVT(std::map<int,Gnss_Synchro> gnss_pseudoranges_map, double galileo_current_time, bool flag_averaging)
{
    std::map<int,Gnss_Synchro>::iterator gnss_pseudoranges_iter;
    std::map<int,Galileo_Ephemeris>::iterator galileo_ephemeris_iter;

    arma::vec W;      // channels weight vector
    arma::vec obs;    // pseudoranges observation vector
    arma::mat satpos; // satellite positions matrix

    int Galileo_week_number = 0;
    double utc = 0.0;
    double GST = 0.0;
    double TX_time_corrected_s = 0.0;
    double SV_clock_bias_s = 0.0;

    d_flag_averaging = flag_averaging;

    // ********************************************************************************
    // ****** PREPARE THE LEAST SQUARES DATA (SV POSITIONS MATRIX AND OBS VECTORS) ****
    // ********************************************************************************
    int valid_obs = 0; //valid observations counter

    for(gnss_pseudoranges_iter = gnss_pseudoranges_map.begin();
            gnss_pseudoranges_iter != gnss_pseudoranges_map.end();
            gnss_pseudoranges_iter++)
        {
            // 1- find the ephemeris for the current SV observation. The SV PRN ID is the map key
            galileo_ephemeris_iter = galileo_ephemeris_map.find(gnss_pseudoranges_iter->first);
            if (galileo_ephemeris_iter != galileo_ephemeris_map.end())
                {
                    /*!
                     * \todo Place here the satellite CN0 (power level, or weight factor)
                     */
                    W.resize(valid_obs + 1, 1);
                    W(valid_obs) = 1;

                    // COMMON RX TIME PVT ALGORITHM
                    double Rx_time = galileo_current_time;
                    double Tx_time = Rx_time - gnss_pseudoranges_iter->second.Pseudorange_m / GALILEO_C_m_s;

                    // 2- compute the clock drift using the clock model (broadcast) for this SV, including relativistic effect
                    SV_clock_bias_s = galileo_ephemeris_iter->second.sv_clock_drift(Tx_time);

                    // 3- compute the current ECEF position for this SV using corrected TX time
                    TX_time_corrected_s = Tx_time - SV_clock_bias_s;
                    galileo_ephemeris_iter->second.satellitePosition(TX_time_corrected_s);

                    //store satellite positions in a matrix
                    satpos.resize(3, valid_obs + 1);
                    satpos(0, valid_obs) = galileo_ephemeris_iter->second.d_satpos_X;
                    satpos(1, valid_obs) = galileo_ephemeris_iter->second.d_satpos_Y;
                    satpos(2, valid_obs) = galileo_ephemeris_iter->second.d_satpos_Z;

                    // 4- fill the observations vector with the corrected pseudoranges
                    obs.resize(valid_obs + 1, 1);
                    obs(valid_obs) = gnss_pseudoranges_iter->second.Pseudorange_m + SV_clock_bias_s * GALILEO_C_m_s - d_rx_dt_s * GALILEO_C_m_s;
                    d_visible_satellites_IDs[valid_obs] = galileo_ephemeris_iter->second.i_satellite_PRN;
                    d_visible_satellites_CN0_dB[valid_obs] = gnss_pseudoranges_iter->second.CN0_dB_hz;


                    Galileo_week_number = galileo_ephemeris_iter->second.WN_5; //for GST
                    GST = galileo_ephemeris_map.find(gnss_pseudoranges_iter->first)->second.Galileo_System_Time(Galileo_week_number, galileo_current_time);

                    // SV ECEF DEBUG OUTPUT
                    DLOG(INFO) << "ECEF satellite SV ID=" << galileo_ephemeris_iter->second.i_satellite_PRN
                               << " X=" << galileo_ephemeris_iter->second.d_satpos_X
                               << " [m] Y=" << galileo_ephemeris_iter->second.d_satpos_Y
                               << " [m] Z=" << galileo_ephemeris_iter->second.d_satpos_Z
                               << " [m] PR_obs=" << obs(valid_obs) << " [m]";

                    valid_obs++;
                }
            else // the ephemeris are not available for this SV
                {
                    DLOG(INFO) << "No ephemeris data for SV "<< gnss_pseudoranges_iter->first;
                }
        }

    // ********************************************************************************
    // ****** SOLVE LEAST SQUARES******************************************************
    // ********************************************************************************
    d_valid_observations = valid_obs;
    LOG(INFO) << "Galileo PVT: valid observations=" << valid_obs;

    if (valid_obs >= 4)
        {
            arma::vec rx_position_and_time;
            DLOG(INFO) << "satpos=" << satpos;
            DLOG(INFO) << "obs="<< obs;
            DLOG(INFO) << "W=" << W;
            try
            {
                    // check if this is the initial position computation
                    if (d_rx_dt_s == 0)
                        {
                            // execute Bancroft's algorithm to estimate initial receiver position and time
                            DLOG(INFO) << " Executing Bancroft algorithm...";
                            rx_position_and_time = bancroftPos(satpos.t(), obs);
                            d_rx_pos = rx_position_and_time.rows(0, 2); // save ECEF position for the next iteration
                            d_rx_dt_s = rx_position_and_time(3) / GALILEO_C_m_s; // save time for the next iteration [meters]->[seconds]
                        }
                    // Execute WLS using previous position as the initialization point
                    rx_position_and_time = leastSquarePos(satpos, obs, W);

                    d_rx_pos = rx_position_and_time.rows(0, 2); // save ECEF position for the next iteration
                    d_rx_dt_s += rx_position_and_time(3) / GALILEO_C_m_s; // accumulate the rx time error for the next iteration [meters]->[seconds]

                    // Compute Gregorian time
                    utc = galileo_utc_model.GST_to_UTC_time(GST, Galileo_week_number);
                    // get time string Gregorian calendar
                    boost::posix_time::time_duration t = boost::posix_time::seconds(utc);
                    // 22 August 1999 00:00 last Galileo start GST epoch (ICD sec 5.1.2)
                    boost::posix_time::ptime p_time(boost::gregorian::date(1999, 8, 22), t);
                    d_position_UTC_time = p_time;

                    DLOG(INFO) << "Galileo Position at TOW=" << galileo_current_time << " in ECEF (X,Y,Z) = " << rx_position_and_time;

                    cart2geo(static_cast<double>(rx_position_and_time(0)), static_cast<double>(rx_position_and_time(1)), static_cast<double>(rx_position_and_time(2)), 4);
                    d_rx_dt_s = rx_position_and_time(3)/GALILEO_C_m_s; // Convert RX time offset from meters to seconds
                    DLOG(INFO) << "Galileo Position at " << boost::posix_time::to_simple_string(p_time)
                               << " is Lat = " << d_latitude_d << " [deg], Long = " << d_longitude_d
                               << " [deg], Height= " << d_height_m << " [m]" << " RX time offset= " << d_rx_dt_s << " [s]";

                    // ###### Compute DOPs ########
                    compute_DOP();

                    // ######## LOG FILE #########
                    if(d_flag_dump_enabled == true)
                        {
                            // MULTIPLEXED FILE RECORDING - Record results to file
                            try
                            {
                                    double tmp_double;
                                    //  PVT GPS time
                                    tmp_double = galileo_current_time;
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    // ECEF User Position East [m]
                                    tmp_double = rx_position_and_time(0);
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    // ECEF User Position North [m]
                                    tmp_double = rx_position_and_time(1);
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    // ECEF User Position Up [m]
                                    tmp_double = rx_position_and_time(2);
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    // User clock offset [s]
                                    tmp_double = rx_position_and_time(3);
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
                                    LOG(WARNING) << "Exception writing PVT LS dump file "<< e.what();
                            }
                        }

                    // MOVING AVERAGE PVT
                    galileo_e1_ls_pvt::pos_averaging(flag_averaging);
            }
            catch(const std::exception & e)
            {
                    d_rx_dt_s = 0; //reset rx time estimation
                    LOG(WARNING) << "Problem with the solver, invalid solution!" << e.what();
                    b_valid_position = false;
            }
        }
    else
        {
            b_valid_position = false;
        }
    return b_valid_position;
}

