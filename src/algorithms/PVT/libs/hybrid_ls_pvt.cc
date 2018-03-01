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
#include "Galileo_E1.h"
#include "GPS_L1_CA.h"
#include "GPS_L2C.h"
#include <glog/logging.h>


using google::LogMessage;

hybrid_ls_pvt::hybrid_ls_pvt(int nchannels, std::string dump_filename, bool flag_dump_to_file) : Ls_Pvt()
{
    // init empty ephemeris for all the available GNSS channels
    d_nchannels = nchannels;
    d_dump_filename = dump_filename;
    d_flag_dump_enabled = flag_dump_to_file;
    d_galileo_current_time = 0;
    count_valid_position = 0;
    this->set_averaging_flag(false);
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
    if (d_dump_file.is_open() == true)
        {
            try
            {
                    d_dump_file.close();
            }
            catch(const std::exception & ex)
            {
                    LOG(WARNING) << "Exception in destructor closing the dump file " << ex.what();
            }
        }
}


bool hybrid_ls_pvt::get_PVT(std::map<int,Gnss_Synchro> gnss_observables_map, double hybrid_current_time, bool flag_averaging)
{
    std::map<int,Gnss_Synchro>::iterator gnss_observables_iter;
    std::map<int,Galileo_Ephemeris>::iterator galileo_ephemeris_iter;
    std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter;
    std::map<int,Gps_CNAV_Ephemeris>::iterator gps_cnav_ephemeris_iter;

    arma::vec W;      // channels weight vector
    arma::vec obs;    // pseudoranges observation vector
    arma::mat satpos; // satellite positions matrix

    int Galileo_week_number = 0;
    int GPS_week = 0;
    double utc = 0.0;
    double GST = 0.0;
    double secondsperweek = 604800.0;

    //double utc_tx_corrected = 0.0; //utc computed at tx_time_corrected, added for Galileo constellation (in GPS utc is directly computed at TX_time_corrected_s)
    double TX_time_corrected_s = 0.0;
    double SV_clock_bias_s = 0.0;

    this->set_averaging_flag(flag_averaging);

    // ********************************************************************************
    // ****** PREPARE THE LEAST SQUARES DATA (SV POSITIONS MATRIX AND OBS VECTORS) ****
    // ********************************************************************************
    int valid_obs = 0; //valid observations counter

    for(gnss_observables_iter = gnss_observables_map.begin();
            gnss_observables_iter != gnss_observables_map.end();
            gnss_observables_iter++)
        {
            switch(gnss_observables_iter->second.System)
            {
            case 'E':
                {
                    // 1 Gal - find the ephemeris for the current GALILEO SV observation. The SV PRN ID is the map key
                    galileo_ephemeris_iter = galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                    if (galileo_ephemeris_iter != galileo_ephemeris_map.end())
                        {
                            /*!
                             * \todo Place here the satellite CN0 (power level, or weight factor)
                             */
                            W.resize(valid_obs + 1, 1);
                            W(valid_obs) = 1;

                            // COMMON RX TIME PVT ALGORITHM
                            double Rx_time = hybrid_current_time;
                            double Tx_time = Rx_time - gnss_observables_iter->second.Pseudorange_m / GALILEO_C_m_s;

                            // 2- compute the clock drift using the clock model (broadcast) for this SV
                            SV_clock_bias_s = galileo_ephemeris_iter->second.sv_clock_drift(Tx_time);

                            // 3- compute the current ECEF position for this SV using corrected TX time
                            TX_time_corrected_s = Tx_time - SV_clock_bias_s;
                            galileo_ephemeris_iter->second.satellitePosition(TX_time_corrected_s);

                            //store satellite positions in a matrix
                            satpos.resize(3, valid_obs + 1);
                            satpos(0, valid_obs) = galileo_ephemeris_iter->second.d_satpos_X;
                            satpos(1, valid_obs) = galileo_ephemeris_iter->second.d_satpos_Y;
                            satpos(2, valid_obs) = galileo_ephemeris_iter->second.d_satpos_Z;

                            // 4- fill the observations vector with the corrected observables
                            obs.resize(valid_obs + 1, 1);
                            obs(valid_obs) = gnss_observables_iter->second.Pseudorange_m + SV_clock_bias_s * GALILEO_C_m_s - this->get_time_offset_s() * GALILEO_C_m_s;
                            this->set_visible_satellites_ID(valid_obs, galileo_ephemeris_iter->second.i_satellite_PRN);
                            this->set_visible_satellites_CN0_dB(valid_obs, gnss_observables_iter->second.CN0_dB_hz);

                            Galileo_week_number = galileo_ephemeris_iter->second.WN_5; //for GST
                            GST = galileo_ephemeris_iter->second.Galileo_System_Time(Galileo_week_number, hybrid_current_time);

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
                            DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                        }
                    break;
                }
            case 'G':
                {
                    // 1 GPS - find the ephemeris for the current GPS SV observation. The SV PRN ID is the map key
                    std::string sig_(gnss_observables_iter->second.Signal);
                    if(sig_.compare("1C") == 0)
                        {
                            gps_ephemeris_iter = gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                            if (gps_ephemeris_iter != gps_ephemeris_map.end())
                                {
                                    /*!
                                     * \todo Place here the satellite CN0 (power level, or weight factor)
                                     */
                                    W.resize(valid_obs + 1, 1);
                                    W(valid_obs) = 1;

                                    // COMMON RX TIME PVT ALGORITHM MODIFICATION (Like RINEX files)
                                    // first estimate of transmit time
                                    double Rx_time = hybrid_current_time;
                                    double Tx_time = Rx_time - gnss_observables_iter->second.Pseudorange_m / GPS_C_m_s;

                                    // 2- compute the clock drift using the clock model (broadcast) for this SV, not including relativistic effect
                                    SV_clock_bias_s = gps_ephemeris_iter->second.sv_clock_drift(Tx_time); //- gps_ephemeris_iter->second.d_TGD;

                                    // 3- compute the current ECEF position for this SV using corrected TX time and obtain clock bias including relativistic effect
                                    TX_time_corrected_s = Tx_time - SV_clock_bias_s;
                                    double dtr = gps_ephemeris_iter->second.satellitePosition(TX_time_corrected_s);

                                    //store satellite positions in a matrix
                                    satpos.resize(3, valid_obs + 1);
                                    satpos(0, valid_obs) = gps_ephemeris_iter->second.d_satpos_X;
                                    satpos(1, valid_obs) = gps_ephemeris_iter->second.d_satpos_Y;
                                    satpos(2, valid_obs) = gps_ephemeris_iter->second.d_satpos_Z;

                                    // 4- fill the observations vector with the corrected pseudoranges
                                    // compute code bias: TGD for single frequency
                                    // See IS-GPS-200E section 20.3.3.3.3.2
                                    double sqrt_Gamma=GPS_L1_FREQ_HZ/GPS_L2_FREQ_HZ;
                                    double Gamma=sqrt_Gamma*sqrt_Gamma;
                                    double P1_P2=(1.0-Gamma)*(gps_ephemeris_iter->second.d_TGD* GPS_C_m_s);
                                    double Code_bias_m= P1_P2/(1.0-Gamma);
                                    obs.resize(valid_obs + 1, 1);
                                    obs(valid_obs) = gnss_observables_iter->second.Pseudorange_m + dtr * GPS_C_m_s-Code_bias_m-this->get_time_offset_s() * GPS_C_m_s;
                                    this->set_visible_satellites_ID(valid_obs, gps_ephemeris_iter->second.i_satellite_PRN);
                                    this->set_visible_satellites_CN0_dB(valid_obs, gnss_observables_iter->second.CN0_dB_hz);

                                    // SV ECEF DEBUG OUTPUT
                                    LOG(INFO) << "(new)ECEF GPS L1 CA satellite SV ID=" << gps_ephemeris_iter->second.i_satellite_PRN
                                               << " TX Time corrected="<<TX_time_corrected_s                                                << " X=" << gps_ephemeris_iter->second.d_satpos_X
                                               << " [m] Y=" << gps_ephemeris_iter->second.d_satpos_Y
                                               << " [m] Z=" << gps_ephemeris_iter->second.d_satpos_Z
                                               << " [m] PR_obs=" << obs(valid_obs) << " [m]";

                                    valid_obs++;
                                    // compute the UTC time for this SV (just to print the associated UTC timestamp)
                                    GPS_week = gps_ephemeris_iter->second.i_GPS_week;
                                }
                            else // the ephemeris are not available for this SV
                                {
                                    DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->first;
                                }
                        }
                    if(sig_.compare("2S") == 0)
                        {
                            gps_cnav_ephemeris_iter = gps_cnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                            if (gps_cnav_ephemeris_iter != gps_cnav_ephemeris_map.end())
                                {
                                    /*!
                                     * \todo Place here the satellite CN0 (power level, or weight factor)
                                     */
                                    W.resize(valid_obs + 1, 1);
                                    W(valid_obs) = 1;

                                    // COMMON RX TIME PVT ALGORITHM MODIFICATION (Like RINEX files)
                                    // first estimate of transmit time
                                    double Rx_time = hybrid_current_time;
                                    double Tx_time = Rx_time - gnss_observables_iter->second.Pseudorange_m / GPS_C_m_s;

                                    // 2- compute the clock drift using the clock model (broadcast) for this SV
                                    SV_clock_bias_s = gps_cnav_ephemeris_iter->second.sv_clock_drift(Tx_time);

                                    // 3- compute the current ECEF position for this SV using corrected TX time
                                    TX_time_corrected_s = Tx_time - SV_clock_bias_s;
                                    //std::cout<<"TX time["<<gps_cnav_ephemeris_iter->second.i_satellite_PRN<<"]="<<TX_time_corrected_s<<std::endl;
                                    double dtr = gps_cnav_ephemeris_iter->second.satellitePosition(TX_time_corrected_s);

                                    //store satellite positions in a matrix
                                    satpos.resize(3, valid_obs + 1);
                                    satpos(0, valid_obs) = gps_cnav_ephemeris_iter->second.d_satpos_X;
                                    satpos(1, valid_obs) = gps_cnav_ephemeris_iter->second.d_satpos_Y;
                                    satpos(2, valid_obs) = gps_cnav_ephemeris_iter->second.d_satpos_Z;

                                    // 4- fill the observations vector with the corrected observables
                                    obs.resize(valid_obs + 1, 1);
                                    obs(valid_obs) = gnss_observables_iter->second.Pseudorange_m + dtr*GPS_C_m_s + SV_clock_bias_s * GPS_C_m_s;
                                    this->set_visible_satellites_ID(valid_obs, gps_cnav_ephemeris_iter->second.i_satellite_PRN);
                                    this->set_visible_satellites_CN0_dB(valid_obs, gnss_observables_iter->second.CN0_dB_hz);

                                    GPS_week = gps_cnav_ephemeris_iter->second.i_GPS_week;
                                    GPS_week=GPS_week%1024; //Necessary due to the increase of WN bits in CNAV message (10 in GPS NAV and 13 in CNAV)

                                    // SV ECEF DEBUG OUTPUT
                                    LOG(INFO) << "(new)ECEF GPS L2M satellite SV ID=" << gps_cnav_ephemeris_iter->second.i_satellite_PRN
                                                << " TX Time corrected="<<TX_time_corrected_s
                                                << " X=" << gps_cnav_ephemeris_iter->second.d_satpos_X
                                               << " [m] Y=" << gps_cnav_ephemeris_iter->second.d_satpos_Y
                                               << " [m] Z=" << gps_cnav_ephemeris_iter->second.d_satpos_Z
                                               << " [m] PR_obs=" << obs(valid_obs) << " [m]";

                                    valid_obs++;
                                }
                            else // the ephemeris are not available for this SV
                                {
                                    DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                                }
                        }
                    break;
                }
            default :
                DLOG(INFO) << "Hybrid observables: Unknown GNSS";
                break;
            }
        }

    // ********************************************************************************
    // ****** SOLVE LEAST SQUARES******************************************************
    // ********************************************************************************
    this->set_num_valid_observations(valid_obs);

    LOG(INFO) << "HYBRID PVT: valid observations=" << valid_obs;

    if(valid_obs >= 4)
        {
            arma::vec rx_position_and_time;
            DLOG(INFO) << "satpos=" << satpos;
            DLOG(INFO) << "obs=" << obs;
            DLOG(INFO) << "W=" << W;
            try
            {
                    // check if this is the initial position computation
                    if (this->get_time_offset_s() == 0)
                        {
                            // execute Bancroft's algorithm to estimate initial receiver position and time
                            DLOG(INFO) << " Executing Bancroft algorithm...";
                            rx_position_and_time = bancroftPos(satpos.t(), obs);
                            this->set_rx_pos(rx_position_and_time.rows(0, 2)); // save ECEF position for the next iteration
                            this->set_time_offset_s(rx_position_and_time(3) / GPS_C_m_s); // save time for the next iteration [meters]->[seconds]
                        }

                    // Execute WLS using previous position as the initialization point
                    rx_position_and_time = leastSquarePos(satpos, obs, W);

                    this->set_rx_pos(rx_position_and_time.rows(0, 2)); // save ECEF position for the next iteration
                    this->set_time_offset_s(this->get_time_offset_s() + rx_position_and_time(3) / GPS_C_m_s); // accumulate the rx time error for the next iteration [meters]->[seconds]

                    DLOG(INFO) << "Hybrid Position at TOW=" << hybrid_current_time << " in ECEF (X,Y,Z,t[meters]) = " << rx_position_and_time;
                    DLOG(INFO) << "Accumulated rx clock error=" << this->get_time_offset_s() << " clock error for this iteration=" << rx_position_and_time(3) / GPS_C_m_s << " [s]";

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
                    this->set_position_UTC_time(p_time);

                    cart2geo(static_cast<double>(rx_position_and_time(0)), static_cast<double>(rx_position_and_time(1)), static_cast<double>(rx_position_and_time(2)), 4);

                    DLOG(INFO) << "Hybrid Position at " << boost::posix_time::to_simple_string(p_time)
                               << " is Lat = " << this->get_latitude() << " [deg], Long = " << this->get_longitude()
                               << " [deg], Height= " << this->get_height() << " [m]" << " RX time offset= " << this->get_time_offset_s() << " [s]";

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
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                                    // ECEF User Position East [m]
                                    tmp_double = rx_position_and_time(0);
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                                    // ECEF User Position North [m]
                                    tmp_double = rx_position_and_time(1);
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                                    // ECEF User Position Up [m]
                                    tmp_double = rx_position_and_time(2);
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                                    // User clock offset [s]
                                    tmp_double = rx_position_and_time(3);
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                                    // GEO user position Latitude [deg]
                                    tmp_double = this->get_latitude();
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                                    // GEO user position Longitude [deg]
                                    tmp_double = this->get_longitude();
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                                    // GEO user position Height [m]
                                    tmp_double = this->get_height();
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                            }
                            catch (const std::ifstream::failure& e)
                            {
                                    LOG(WARNING) << "Exception writing PVT LS dump file " << e.what();
                            }
                        }

                    // MOVING AVERAGE PVT
                    this->perform_pos_averaging();
            }
            catch(const std::exception & e)
            {
                    this->set_time_offset_s(0.0); //reset rx time estimation
                    LOG(WARNING) << "Problem with the solver, invalid solution!" << e.what();
                    this->set_valid_position(false);
            }
        }
    else
        {
            this->set_valid_position(false);
        }
    return this->is_valid_position();
}
