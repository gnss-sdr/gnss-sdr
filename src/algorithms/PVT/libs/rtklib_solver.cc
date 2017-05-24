/*!
 * \file rtklib_solver.cc
 * \brief PVT solver based on rtklib library functions adapted to the GNSS-SDR
 *  data flow and structures
 * \authors <ul>
 *          <li> 2017, Javier Arribas
 *          <li> 2017, Carles Fernandez
 *          <li> 2007-2013, T. Takasu
 *          </ul>
 *
 * This is a derived work from RTKLIB http://www.rtklib.com/
 * The original source code at https://github.com/tomojitakasu/RTKLIB is
 * released under the BSD 2-clause license with an additional exclusive clause
 * that does not apply here. This additional clause is reproduced below:
 *
 * " The software package includes some companion executive binaries or shared
 * libraries necessary to execute APs on Windows. These licenses succeed to the
 * original ones of these software. "
 *
 * Neither the executive binaries nor the shared libraries are required by, used
 * or included in GNSS-SDR.
 *
 * -------------------------------------------------------------------------
 * Copyright (C) 2007-2013, T. Takasu
 * Copyright (C) 2017, Javier Arribas
 * Copyright (C) 2017, Carles Fernandez
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * -----------------------------------------------------------------------*/

#include "rtklib_solver.h"
#include <glog/logging.h>
#include "rtklib_conversions.h"
#include "GPS_L1_CA.h"
#include "Galileo_E1.h"


using google::LogMessage;

rtklib_solver::rtklib_solver(int nchannels, std::string dump_filename, bool flag_dump_to_file,  rtk_t & rtk)
{
    // init empty ephemeris for all the available GNSS channels
    d_nchannels = nchannels;
    d_dump_filename = dump_filename;
    d_flag_dump_enabled = flag_dump_to_file;
    count_valid_position = 0;
    d_flag_averaging = false;
    rtk_ = rtk;

    pvt_sol = {{0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0}, '0', '0', '0', 0, 0, 0 };

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


rtklib_solver::~rtklib_solver()
{
    d_dump_file.close();
}


bool rtklib_solver::get_PVT(std::map<int,Gnss_Synchro> gnss_observables_map, double Rx_time, bool flag_averaging)
{
    std::map<int,Gnss_Synchro>::iterator gnss_observables_iter;
    std::map<int,Galileo_Ephemeris>::iterator galileo_ephemeris_iter;
    std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter;
    std::map<int,Gps_CNAV_Ephemeris>::iterator gps_cnav_ephemeris_iter;

    d_flag_averaging = flag_averaging;

    // ********************************************************************************
    // ****** PREPARE THE DATA (SV EPHEMERIS AND OBSERVATIONS) ************************
    // ********************************************************************************
    int valid_obs = 0; //valid observations counter

    obsd_t obs_data[MAXOBS];
    eph_t eph_data[MAXOBS];

    for(gnss_observables_iter = gnss_observables_map.begin();
            gnss_observables_iter != gnss_observables_map.end();
            gnss_observables_iter++)
    {
        switch(gnss_observables_iter->second.System)
        {
        case 'E':
        {
            std::string sig_(gnss_observables_iter->second.Signal);
            // Galileo E1
            if(sig_.compare("1B") == 0)
            {
                // 1 Gal - find the ephemeris for the current GALILEO SV observation. The SV PRN ID is the map key
                galileo_ephemeris_iter = galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                if (galileo_ephemeris_iter != galileo_ephemeris_map.end())
                {
                    //convert ephemeris from GNSS-SDR class to RTKLIB structure
                    eph_data[valid_obs] = eph_to_rtklib(galileo_ephemeris_iter->second);
                    //convert observation from GNSS-SDR class to RTKLIB structure
                    obsd_t newobs = {{0,0}, '0', '0', {}, {}, {}, {}, {}, {}};
                    obs_data[valid_obs] = insert_obs_to_rtklib(newobs,
                            gnss_observables_iter->second,
                            galileo_ephemeris_iter->second.WN_5,
                            0);
                    valid_obs++;
                }
                else // the ephemeris are not available for this SV
                {
                    DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                }

            }
            // Galileo E5
            if(sig_.compare("5X") == 0)
            {

                // 1 Gal - find the ephemeris for the current GALILEO SV observation. The SV PRN ID is the map key
                galileo_ephemeris_iter = galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                if (galileo_ephemeris_iter != galileo_ephemeris_map.end())
                {
                    bool found_E1_obs=false;
                    for (int i = 0; i < valid_obs; i++)
                    {
                        if (eph_data[i].sat == (static_cast<int>(gnss_observables_iter->second.PRN+NSATGPS+NSATGLO)))
                        {
                            obs_data[i] = insert_obs_to_rtklib(obs_data[i],
                                    gnss_observables_iter->second,
                                    galileo_ephemeris_iter->second.WN_5,
                                    2);//Band 3 (L5/E5)
                            found_E1_obs=true;
                            break;
                        }
                    }
                    if (!found_E1_obs)
                    {
                        //insert Galileo E5 obs as new obs and also insert its ephemeris
                        //convert ephemeris from GNSS-SDR class to RTKLIB structure
                        eph_data[valid_obs] = eph_to_rtklib(galileo_ephemeris_iter->second);
                        //convert observation from GNSS-SDR class to RTKLIB structure
                        obsd_t newobs = {{0,0}, '0', '0', {}, {}, {}, {}, {}, {}};
                        obs_data[valid_obs] = insert_obs_to_rtklib(newobs,
                                gnss_observables_iter->second,
                                galileo_ephemeris_iter->second.WN_5,
                                2); //Band 3 (L5/E5)
                        valid_obs++;
                    }
                }
                else // the ephemeris are not available for this SV
                {
                    DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                }

            }
            break;
        }
        case 'G':
        {
            // GPS L1
            // 1 GPS - find the ephemeris for the current GPS SV observation. The SV PRN ID is the map key
            std::string sig_(gnss_observables_iter->second.Signal);
            if(sig_.compare("1C") == 0)
            {
                gps_ephemeris_iter = gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                if (gps_ephemeris_iter != gps_ephemeris_map.end())
                {
                    //convert ephemeris from GNSS-SDR class to RTKLIB structure
                    eph_data[valid_obs] = eph_to_rtklib(gps_ephemeris_iter->second);
                    //convert observation from GNSS-SDR class to RTKLIB structure
                    obsd_t newobs = {{0,0}, '0', '0', {}, {}, {}, {}, {}, {}};
                    obs_data[valid_obs] = insert_obs_to_rtklib(newobs,
                            gnss_observables_iter->second,
                            gps_ephemeris_iter->second.i_GPS_week,
                            0);
                    valid_obs++;
                }
                else // the ephemeris are not available for this SV
                {
                    DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->first;
                }
            }
            //GPS L2
            if(sig_.compare("2S") == 0)
            {
                gps_cnav_ephemeris_iter = gps_cnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                if (gps_cnav_ephemeris_iter != gps_cnav_ephemeris_map.end())
                {
                    // 1. Find the same satellite in GPS L1 band
                    gps_ephemeris_iter = gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                    if (gps_ephemeris_iter != gps_ephemeris_map.end())
                    {
                        // 2. If found, replace the existing GPS L1 ephemeris with the GPS L2 ephemeris
                        // (more precise!), and attach the L2 observation to the L1 observation in RTKLIB structure
                        for (int i = 0; i < valid_obs; i++)
                        {
                            if (eph_data[i].sat == static_cast<int>(gnss_observables_iter->second.PRN))
                            {
                                eph_data[i] = eph_to_rtklib(gps_cnav_ephemeris_iter->second);
                                obs_data[i] = insert_obs_to_rtklib(obs_data[i],
                                        gnss_observables_iter->second,
                                        gps_cnav_ephemeris_iter->second.i_GPS_week,
                                        1);//Band 2 (L2)
                                break;
                            }
                        }
                    }
                    else
                    {
                        // 3. If not found, insert the GPS L2 ephemeris and the observation
                        //convert ephemeris from GNSS-SDR class to RTKLIB structure
                        eph_data[valid_obs] = eph_to_rtklib(gps_cnav_ephemeris_iter->second);
                        //convert observation from GNSS-SDR class to RTKLIB structure
                        obsd_t newobs = {{0,0}, '0', '0', {}, {}, {}, {}, {}, {}};
                        obs_data[valid_obs] = insert_obs_to_rtklib(newobs,
                                gnss_observables_iter->second,
                                gps_cnav_ephemeris_iter->second.i_GPS_week,
                                1);//Band 2 (L2)
                        valid_obs++;
                    }
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

        // **********************************************************************
        // ****** SOLVE PVT******************************************************
        // **********************************************************************

        b_valid_position = false;
        if (valid_obs > 0)
        {
            int result = 0;
            nav_t nav_data;
            nav_data.eph = eph_data;
            nav_data.n = valid_obs;
            for (int i = 0; i < MAXSAT; i++)
            {
                nav_data.lam[i][0] = SPEED_OF_LIGHT / FREQ1; /* L1/E1 */
                nav_data.lam[i][1] = SPEED_OF_LIGHT / FREQ2; /* L2 */
                nav_data.lam[i][2] = SPEED_OF_LIGHT / FREQ5; /* L5/E5 */
            }

            result = rtkpos(&rtk_, obs_data, valid_obs, &nav_data);
            if(result == 0)
            {
                LOG(INFO) << "RTKLIB rtkpos error message: " << rtk_.errbuf;
                d_rx_dt_s = 0; //reset rx time estimation
                d_valid_observations = 0;
            }
            else
            {
                d_valid_observations = rtk_.sol.ns; //record the number of valid satellites used by the PVT solver
                pvt_sol = rtk_.sol;
                b_valid_position = true;
                arma::vec rx_position_and_time(4);
                rx_position_and_time(0) = pvt_sol.rr[0];
                rx_position_and_time(1) = pvt_sol.rr[1];
                rx_position_and_time(2) = pvt_sol.rr[2];
                rx_position_and_time(3) = pvt_sol.dtr[0];
                d_rx_pos = rx_position_and_time.rows(0, 2); // save ECEF position for the next iteration
                d_rx_dt_s += rx_position_and_time(3) / GPS_C_m_s; // accumulate the rx time error for the next iteration [meters]->[seconds]
                DLOG(INFO) << "RTKLIB Position at TOW=" << Rx_time << " in ECEF (X,Y,Z,t[meters]) = " << rx_position_and_time;

                boost::posix_time::ptime p_time;
                gtime_t rtklib_utc_time = gpst2utc(pvt_sol.time);
                p_time = boost::posix_time::from_time_t(rtklib_utc_time.time);
                p_time+=boost::posix_time::microseconds(round(rtklib_utc_time.sec * 1e6));
                d_position_UTC_time = p_time;
                cart2geo(static_cast<double>(rx_position_and_time(0)), static_cast<double>(rx_position_and_time(1)), static_cast<double>(rx_position_and_time(2)), 4);

                DLOG(INFO) << "RTKLIB Position at " << boost::posix_time::to_simple_string(p_time)
                << " is Lat = " << d_latitude_d << " [deg], Long = " << d_longitude_d
                << " [deg], Height= " << d_height_m << " [m]" << " RX time offset= " << d_rx_dt_s << " [s]";

                // ######## LOG FILE #########
                if(d_flag_dump_enabled == true)
                {
                    // MULTIPLEXED FILE RECORDING - Record results to file
                    try
                    {
                        double tmp_double;
                        //  PVT GPS time
                        tmp_double = Rx_time;
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
                        LOG(WARNING) << "Exception writing PVT LS dump file " << e.what();
                    }
                }
            }
        }
        return b_valid_position;
    }
