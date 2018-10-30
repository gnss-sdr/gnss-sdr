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
#include "rtklib_conversions.h"
#include "GPS_L1_CA.h"
#include "Galileo_E1.h"
#include "GLONASS_L1_L2_CA.h"
#include <glog/logging.h>


using google::LogMessage;

rtklib_solver::rtklib_solver(int nchannels, std::string dump_filename, bool flag_dump_to_file, rtk_t& rtk)
{
    // init empty ephemeris for all the available GNSS channels
    d_nchannels = nchannels;
    d_dump_filename = dump_filename;
    d_flag_dump_enabled = flag_dump_to_file;
    count_valid_position = 0;
    this->set_averaging_flag(false);
    rtk_ = rtk;
    for (unsigned int i = 0; i < 4; i++) dop_[i] = 0.0;
    pvt_sol = {{0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, '0', '0', '0', 0, 0, 0};

    // ############# ENABLE DATA FILE LOG #################
    if (d_flag_dump_enabled == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                        {
                            d_dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "PVT lib dump enabled Log file: " << d_dump_filename.c_str();
                        }
                    catch (const std::ifstream::failure& e)
                        {
                            LOG(WARNING) << "Exception opening RTKLIB dump file " << e.what();
                        }
                }
        }
}


rtklib_solver::~rtklib_solver()
{
    if (d_dump_file.is_open() == true)
        {
            try
                {
                    d_dump_file.close();
                }
            catch (const std::exception& ex)
                {
                    LOG(WARNING) << "Exception in destructor closing the RTKLIB dump file " << ex.what();
                }
        }
}


double rtklib_solver::get_gdop() const
{
    return dop_[0];
}


double rtklib_solver::get_pdop() const
{
    return dop_[1];
}


double rtklib_solver::get_hdop() const
{
    return dop_[2];
}


double rtklib_solver::get_vdop() const
{
    return dop_[3];
}


bool rtklib_solver::get_PVT(const std::map<int, Gnss_Synchro>& gnss_observables_map, bool flag_averaging)
{
    std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
    std::map<int, Galileo_Ephemeris>::const_iterator galileo_ephemeris_iter;
    std::map<int, Gps_Ephemeris>::const_iterator gps_ephemeris_iter;
    std::map<int, Gps_CNAV_Ephemeris>::const_iterator gps_cnav_ephemeris_iter;
    std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_ephemeris_iter;
    const Glonass_Gnav_Utc_Model gnav_utc = this->glonass_gnav_utc_model;

    this->set_averaging_flag(flag_averaging);

    // ********************************************************************************
    // ****** PREPARE THE DATA (SV EPHEMERIS AND OBSERVATIONS) ************************
    // ********************************************************************************
    int valid_obs = 0;      // valid observations counter
    int glo_valid_obs = 0;  // GLONASS L1/L2 valid observations counter

    obsd_t obs_data[MAXOBS];
    eph_t eph_data[MAXOBS];
    geph_t geph_data[MAXOBS];

    // Workaround for NAV/CNAV clash problem
    bool gps_dual_band = false;
    bool band1 = false;
    bool band2 = false;
    for (gnss_observables_iter = gnss_observables_map.cbegin();
         gnss_observables_iter != gnss_observables_map.cend();
         ++gnss_observables_iter)
        {
            switch (gnss_observables_iter->second.System)
                {
                case 'G':
                    {
                        std::string sig_(gnss_observables_iter->second.Signal);
                        if (sig_.compare("1C") == 0)
                            {
                                band1 = true;
                            }
                        if (sig_.compare("2S") == 0)
                            {
                                band2 = true;
                            }
                    }
                    break;
                default:
                    {
                    }
                }
        }
    if (band1 == true and band2 == true) gps_dual_band = true;

    for (gnss_observables_iter = gnss_observables_map.cbegin();
         gnss_observables_iter != gnss_observables_map.cend();
         ++gnss_observables_iter)  // CHECK INCONSISTENCY when combining GLONASS + other system
        {
            switch (gnss_observables_iter->second.System)
                {
                case 'E':
                    {
                        std::string sig_(gnss_observables_iter->second.Signal);
                        // Galileo E1
                        if (sig_.compare("1B") == 0)
                            {
                                // 1 Gal - find the ephemeris for the current GALILEO SV observation. The SV PRN ID is the map key
                                galileo_ephemeris_iter = galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (galileo_ephemeris_iter != galileo_ephemeris_map.cend())
                                    {
                                        // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                        eph_data[valid_obs] = eph_to_rtklib(galileo_ephemeris_iter->second);
                                        // convert observation from GNSS-SDR class to RTKLIB structure
                                        obsd_t newobs = {{0, 0}, '0', '0', {}, {}, {}, {}, {}, {}};
                                        obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                            gnss_observables_iter->second,
                                            galileo_ephemeris_iter->second.WN_5,
                                            0);
                                        valid_obs++;
                                    }
                                else  // the ephemeris are not available for this SV
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                                    }
                            }

                        // Galileo E5
                        if (sig_.compare("5X") == 0)
                            {
                                // 1 Gal - find the ephemeris for the current GALILEO SV observation. The SV PRN ID is the map key
                                galileo_ephemeris_iter = galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (galileo_ephemeris_iter != galileo_ephemeris_map.cend())
                                    {
                                        bool found_E1_obs = false;
                                        for (int i = 0; i < valid_obs; i++)
                                            {
                                                if (eph_data[i].sat == (static_cast<int>(gnss_observables_iter->second.PRN + NSATGPS + NSATGLO)))
                                                    {
                                                        obs_data[i + glo_valid_obs] = insert_obs_to_rtklib(obs_data[i + glo_valid_obs],
                                                            gnss_observables_iter->second,
                                                            galileo_ephemeris_iter->second.WN_5,
                                                            2);  // Band 3 (L5/E5)
                                                        found_E1_obs = true;
                                                        break;
                                                    }
                                            }
                                        if (!found_E1_obs)
                                            {
                                                // insert Galileo E5 obs as new obs and also insert its ephemeris
                                                // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                                eph_data[valid_obs] = eph_to_rtklib(galileo_ephemeris_iter->second);
                                                // convert observation from GNSS-SDR class to RTKLIB structure
                                                unsigned char default_code_ = static_cast<unsigned char>(CODE_NONE);
                                                obsd_t newobs = {{0, 0}, '0', '0', {}, {},
                                                    {default_code_, default_code_, default_code_},
                                                    {}, {0.0, 0.0, 0.0}, {}};
                                                obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    galileo_ephemeris_iter->second.WN_5,
                                                    2);  // Band 3 (L5/E5)
                                                valid_obs++;
                                            }
                                    }
                                else  // the ephemeris are not available for this SV
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
                        if (sig_.compare("1C") == 0)
                            {
                                gps_ephemeris_iter = gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (gps_ephemeris_iter != gps_ephemeris_map.cend())
                                    {
                                        // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                        eph_data[valid_obs] = eph_to_rtklib(gps_ephemeris_iter->second);
                                        // convert observation from GNSS-SDR class to RTKLIB structure
                                        obsd_t newobs = {{0, 0}, '0', '0', {}, {}, {}, {}, {}, {}};
                                        obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                            gnss_observables_iter->second,
                                            gps_ephemeris_iter->second.i_GPS_week,
                                            0);
                                        valid_obs++;
                                    }
                                else  // the ephemeris are not available for this SV
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->first;
                                    }
                            }
                        // GPS L2 (todo: solve NAV/CNAV clash)
                        if ((sig_.compare("2S") == 0) and (gps_dual_band == false))
                            {
                                gps_cnav_ephemeris_iter = gps_cnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (gps_cnav_ephemeris_iter != gps_cnav_ephemeris_map.cend())
                                    {
                                        // 1. Find the same satellite in GPS L1 band
                                        gps_ephemeris_iter = gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                        if (gps_ephemeris_iter != gps_ephemeris_map.cend())
                                            {
                                                /* By the moment, GPS L2 observables are not used in pseudorange computations if GPS L1 is available
                                                // 2. If found, replace the existing GPS L1 ephemeris with the GPS L2 ephemeris
                                                // (more precise!), and attach the L2 observation to the L1 observation in RTKLIB structure
                                                for (int i = 0; i < valid_obs; i++)
                                                    {
                                                        if (eph_data[i].sat == static_cast<int>(gnss_observables_iter->second.PRN))
                                                            {
                                                                eph_data[i] = eph_to_rtklib(gps_cnav_ephemeris_iter->second);
                                                                obs_data[i + glo_valid_obs] = insert_obs_to_rtklib(obs_data[i + glo_valid_obs],
                                                                    gnss_observables_iter->second,
                                                                    eph_data[i].week,
                                                                    1);  // Band 2 (L2)
                                                                break;
                                                            }
                                                    }
                                                */
                                            }
                                        else
                                            {
                                                // 3. If not found, insert the GPS L2 ephemeris and the observation
                                                // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                                eph_data[valid_obs] = eph_to_rtklib(gps_cnav_ephemeris_iter->second);
                                                // convert observation from GNSS-SDR class to RTKLIB structure
                                                unsigned char default_code_ = static_cast<unsigned char>(CODE_NONE);
                                                obsd_t newobs = {{0, 0}, '0', '0', {}, {},
                                                    {default_code_, default_code_, default_code_},
                                                    {}, {0.0, 0.0, 0.0}, {}};
                                                obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    gps_cnav_ephemeris_iter->second.i_GPS_week,
                                                    1);  // Band 2 (L2)
                                                valid_obs++;
                                            }
                                    }
                                else  // the ephemeris are not available for this SV
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                                    }
                            }
                        // GPS L5
                        if (sig_.compare("L5") == 0)
                            {
                                gps_cnav_ephemeris_iter = gps_cnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (gps_cnav_ephemeris_iter != gps_cnav_ephemeris_map.cend())
                                    {
                                        // 1. Find the same satellite in GPS L1 band
                                        gps_ephemeris_iter = gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                        if (gps_ephemeris_iter != gps_ephemeris_map.cend())
                                            {
                                                // 2. If found, replace the existing GPS L1 ephemeris with the GPS L5 ephemeris
                                                // (more precise!), and attach the L5 observation to the L1 observation in RTKLIB structure
                                                for (int i = 0; i < valid_obs; i++)
                                                    {
                                                        if (eph_data[i].sat == static_cast<int>(gnss_observables_iter->second.PRN))
                                                            {
                                                                eph_data[i] = eph_to_rtklib(gps_cnav_ephemeris_iter->second);
                                                                obs_data[i + glo_valid_obs] = insert_obs_to_rtklib(obs_data[i],
                                                                    gnss_observables_iter->second,
                                                                    gps_cnav_ephemeris_iter->second.i_GPS_week,
                                                                    2);  // Band 3 (L5)
                                                                break;
                                                            }
                                                    }
                                            }
                                        else
                                            {
                                                // 3. If not found, insert the GPS L5 ephemeris and the observation
                                                // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                                eph_data[valid_obs] = eph_to_rtklib(gps_cnav_ephemeris_iter->second);
                                                // convert observation from GNSS-SDR class to RTKLIB structure
                                                unsigned char default_code_ = static_cast<unsigned char>(CODE_NONE);
                                                obsd_t newobs = {{0, 0}, '0', '0', {}, {},
                                                    {default_code_, default_code_, default_code_},
                                                    {}, {0.0, 0.0, 0.0}, {}};
                                                obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    gps_cnav_ephemeris_iter->second.i_GPS_week,
                                                    2);  // Band 3 (L5)
                                                valid_obs++;
                                            }
                                    }
                                else  // the ephemeris are not available for this SV
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                                    }
                            }
                        break;
                    }
                case 'R':  //TODO This should be using rtk lib nomenclature
                    {
                        std::string sig_(gnss_observables_iter->second.Signal);
                        // GLONASS GNAV L1
                        if (sig_.compare("1G") == 0)
                            {
                                // 1 Glo - find the ephemeris for the current GLONASS SV observation. The SV Slot Number (PRN ID) is the map key
                                glonass_gnav_ephemeris_iter = glonass_gnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (glonass_gnav_ephemeris_iter != glonass_gnav_ephemeris_map.cend())
                                    {
                                        // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                        geph_data[glo_valid_obs] = eph_to_rtklib(glonass_gnav_ephemeris_iter->second, gnav_utc);
                                        // convert observation from GNSS-SDR class to RTKLIB structure
                                        obsd_t newobs = {{0, 0}, '0', '0', {}, {}, {}, {}, {}, {}};
                                        obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                            gnss_observables_iter->second,
                                            glonass_gnav_ephemeris_iter->second.d_WN,
                                            0);  // Band 0 (L1)
                                        glo_valid_obs++;
                                    }
                                else  // the ephemeris are not available for this SV
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                                    }
                            }
                        // GLONASS GNAV L2
                        if (sig_.compare("2G") == 0)
                            {
                                // 1 GLONASS - find the ephemeris for the current GLONASS SV observation. The SV PRN ID is the map key
                                glonass_gnav_ephemeris_iter = glonass_gnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (glonass_gnav_ephemeris_iter != glonass_gnav_ephemeris_map.cend())
                                    {
                                        bool found_L1_obs = false;
                                        for (int i = 0; i < glo_valid_obs; i++)
                                            {
                                                if (geph_data[i].sat == (static_cast<int>(gnss_observables_iter->second.PRN + NSATGPS)))
                                                    {
                                                        obs_data[i + valid_obs] = insert_obs_to_rtklib(obs_data[i + valid_obs],
                                                            gnss_observables_iter->second,
                                                            glonass_gnav_ephemeris_iter->second.d_WN,
                                                            1);  //Band 1 (L2)
                                                        found_L1_obs = true;
                                                        break;
                                                    }
                                            }
                                        if (!found_L1_obs)
                                            {
                                                // insert GLONASS GNAV L2 obs as new obs and also insert its ephemeris
                                                // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                                geph_data[glo_valid_obs] = eph_to_rtklib(glonass_gnav_ephemeris_iter->second, gnav_utc);
                                                // convert observation from GNSS-SDR class to RTKLIB structure
                                                obsd_t newobs = {{0, 0}, '0', '0', {}, {}, {}, {}, {}, {}};
                                                obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    glonass_gnav_ephemeris_iter->second.d_WN,
                                                    1);  // Band 1 (L2)
                                                glo_valid_obs++;
                                            }
                                    }
                                else  // the ephemeris are not available for this SV
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                                    }
                            }
                        break;
                    }
                default:
                    DLOG(INFO) << "Hybrid observables: Unknown GNSS";
                    break;
                }
        }

    // **********************************************************************
    // ****** SOLVE PVT******************************************************
    // **********************************************************************

    this->set_valid_position(false);
    if ((valid_obs + glo_valid_obs) > 3)
        {
            int result = 0;
            nav_t nav_data;
            nav_data.eph = eph_data;
            nav_data.geph = geph_data;
            nav_data.n = valid_obs;
            nav_data.ng = glo_valid_obs;

            for (int i = 0; i < MAXSAT; i++)
                {
                    nav_data.lam[i][0] = SPEED_OF_LIGHT / FREQ1; /* L1/E1 */
                    nav_data.lam[i][1] = SPEED_OF_LIGHT / FREQ2; /* L2 */
                    nav_data.lam[i][2] = SPEED_OF_LIGHT / FREQ5; /* L5/E5 */
                }

            result = rtkpos(&rtk_, obs_data, valid_obs + glo_valid_obs, &nav_data);

            if (result == 0)
                {
                    LOG(INFO) << "RTKLIB rtkpos error";
                    DLOG(INFO) << "RTKLIB rtkpos error message: " << rtk_.errbuf;
                    this->set_time_offset_s(0.0);  //reset rx time estimation
                    this->set_num_valid_observations(0);
                }
            else
                {
                    this->set_num_valid_observations(rtk_.sol.ns);  //record the number of valid satellites used by the PVT solver
                    pvt_sol = rtk_.sol;
                    // DOP computation
                    unsigned int used_sats = 0;
                    for (unsigned int i = 0; i < MAXSAT; i++)
                        {
                            if (rtk_.ssat[i].vs == 1) used_sats++;
                        }

                    std::vector<double> azel;
                    azel.reserve(used_sats * 2);
                    unsigned int index_aux = 0;
                    for (unsigned int i = 0; i < MAXSAT; i++)
                        {
                            if (rtk_.ssat[i].vs == 1)
                                {
                                    azel[2 * index_aux] = rtk_.ssat[i].azel[0];
                                    azel[2 * index_aux + 1] = rtk_.ssat[i].azel[1];
                                    index_aux++;
                                }
                        }
                    if (index_aux > 0) dops(index_aux, azel.data(), 0.0, dop_);

                    this->set_valid_position(true);
                    arma::vec rx_position_and_time(4);
                    rx_position_and_time(0) = pvt_sol.rr[0];  // [m]
                    rx_position_and_time(1) = pvt_sol.rr[1];  // [m]
                    rx_position_and_time(2) = pvt_sol.rr[2];  // [m]

                    //todo: fix this ambiguity in the RTKLIB units in receiver clock offset!
                    if (rtk_.opt.mode == PMODE_SINGLE)
                        {
                            rx_position_and_time(3) = pvt_sol.dtr[0];  // if the RTKLIB solver is set to SINGLE, the dtr is already expressed in [s]
                        }
                    else
                        {
                            rx_position_and_time(3) = pvt_sol.dtr[0] / GPS_C_m_s;  // the receiver clock offset is expressed in [meters], so we convert it into [s]
                        }
                    this->set_rx_pos(rx_position_and_time.rows(0, 2));  // save ECEF position for the next iteration
                    //observable fix:
                    //double offset_s = this->get_time_offset_s();
                    //this->set_time_offset_s(offset_s + (rx_position_and_time(3) / GPS_C_m_s));  // accumulate the rx time error for the next iteration [meters]->[seconds]
                    this->set_time_offset_s(rx_position_and_time(3));

                    DLOG(INFO) << "RTKLIB Position at RX TOW = " << gnss_observables_map.begin()->second.RX_time
                               << " in ECEF (X,Y,Z,t[meters]) = " << rx_position_and_time;

                    boost::posix_time::ptime p_time;
                    // gtime_t rtklib_utc_time = gpst2utc(pvt_sol.time); //Corrected RX Time (Non integer multiply of 1 ms of granularity)
                    // Uncorrected RX Time (integer multiply of 1 ms and the same observables time reported in RTCM and RINEX)
                    gtime_t rtklib_time = gpst2time(adjgpsweek(nav_data.eph[0].week), gnss_observables_map.begin()->second.RX_time);
                    gtime_t rtklib_utc_time = gpst2utc(rtklib_time);
                    p_time = boost::posix_time::from_time_t(rtklib_utc_time.time);
                    p_time += boost::posix_time::microseconds(static_cast<long>(round(rtklib_utc_time.sec * 1e6)));
                    this->set_position_UTC_time(p_time);
                    cart2geo(static_cast<double>(rx_position_and_time(0)), static_cast<double>(rx_position_and_time(1)), static_cast<double>(rx_position_and_time(2)), 4);

                    DLOG(INFO) << "RTKLIB Position at " << boost::posix_time::to_simple_string(p_time)
                               << " is Lat = " << this->get_latitude() << " [deg], Long = " << this->get_longitude()
                               << " [deg], Height= " << this->get_height() << " [m]"
                               << " RX time offset= " << this->get_time_offset_s() << " [s]";

                    // ######## LOG FILE #########
                    if (d_flag_dump_enabled == true)
                        {
                            // MULTIPLEXED FILE RECORDING - Record results to file
                            try
                                {
                                    double tmp_double;
                                    uint32_t tmp_uint32;
                                    // TOW
                                    tmp_uint32 = gnss_observables_map.begin()->second.TOW_at_current_symbol_ms;
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_uint32), sizeof(uint32_t));
                                    // WEEK
                                    tmp_uint32 = adjgpsweek(nav_data.eph[0].week);
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_uint32), sizeof(uint32_t));
                                    // PVT GPS time
                                    tmp_double = gnss_observables_map.begin()->second.RX_time;
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                                    // User clock offset [s]
                                    tmp_double = rx_position_and_time(3);
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));

                                    // ECEF POS X,Y,X [m] + ECEF VEL X,Y,X [m/s] (6 x double)
                                    tmp_double = pvt_sol.rr[0];
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                                    tmp_double = pvt_sol.rr[1];
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                                    tmp_double = pvt_sol.rr[2];
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                                    tmp_double = pvt_sol.rr[3];
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                                    tmp_double = pvt_sol.rr[4];
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                                    tmp_double = pvt_sol.rr[5];
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));

                                    // position variance/covariance (m^2) {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx} (6 x double)
                                    tmp_double = pvt_sol.qr[0];
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                                    tmp_double = pvt_sol.qr[1];
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                                    tmp_double = pvt_sol.qr[2];
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                                    tmp_double = pvt_sol.qr[3];
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                                    tmp_double = pvt_sol.qr[4];
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                                    tmp_double = pvt_sol.qr[5];
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));

                                    // GEO user position Latitude [deg]
                                    tmp_double = get_latitude();
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                                    // GEO user position Longitude [deg]
                                    tmp_double = get_longitude();
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                                    // GEO user position Height [m]
                                    tmp_double = get_height();
                                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));

                                    // NUMBER OF VALID SATS
                                    d_dump_file.write(reinterpret_cast<char*>(&pvt_sol.ns), sizeof(uint8_t));
                                    // RTKLIB solution status
                                    d_dump_file.write(reinterpret_cast<char*>(&pvt_sol.stat), sizeof(uint8_t));
                                    // RTKLIB solution type (0:xyz-ecef,1:enu-baseline)
                                    d_dump_file.write(reinterpret_cast<char*>(&pvt_sol.type), sizeof(uint8_t));
                                    // AR ratio factor for validation
                                    d_dump_file.write(reinterpret_cast<char*>(&pvt_sol.ratio), sizeof(float));
                                    // AR ratio threshold for validation
                                    d_dump_file.write(reinterpret_cast<char*>(&pvt_sol.thres), sizeof(float));

                                    // GDOP / PDOP/ HDOP/ VDOP
                                    d_dump_file.write(reinterpret_cast<char*>(&dop_[0]), sizeof(double));
                                    d_dump_file.write(reinterpret_cast<char*>(&dop_[1]), sizeof(double));
                                    d_dump_file.write(reinterpret_cast<char*>(&dop_[2]), sizeof(double));
                                    d_dump_file.write(reinterpret_cast<char*>(&dop_[3]), sizeof(double));
                                }
                            catch (const std::ifstream::failure& e)
                                {
                                    LOG(WARNING) << "Exception writing RTKLIB dump file " << e.what();
                                }
                        }
                }
        }
    return is_valid_position();
}
