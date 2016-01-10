/*!
 * \file gps_l1_ca_ls_pvt.h
 * \brief Interface of a Least Squares Position, Velocity, and Time (PVT)
 * solver for GPS L1 C/A, based on K.Borre's Matlab receiver.
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_GPS_L1_CA_LS_PVT_H_
#define GNSS_SDR_GPS_L1_CA_LS_PVT_H_

#include <fstream>
#include <map>
#include <string>
#include "ls_pvt.h"
#include "GPS_L1_CA.h"
#include "gnss_synchro.h"
#include "gps_ephemeris.h"
#include "gps_navigation_message.h"
#include "gps_utc_model.h"
#include "sbas_telemetry_data.h"
#include "sbas_ionospheric_correction.h"
#include "sbas_satellite_correction.h"
#include "sbas_ephemeris.h"


/*!
 * \brief This class implements a simple PVT Least Squares solution for GPS L1 C/A signals
 */
class gps_l1_ca_ls_pvt : public Ls_Pvt
{
public:
    gps_l1_ca_ls_pvt(int nchannels, std::string dump_filename, bool flag_dump_to_file);
    ~gps_l1_ca_ls_pvt();

    bool get_PVT(std::map<int,Gnss_Synchro> gnss_pseudoranges_map, double GPS_current_time, bool flag_averaging);
    int d_nchannels; //!< Number of available channels for positioning

    Gps_Navigation_Message* d_ephemeris;

    // new ephemeris storage
    std::map<int,Gps_Ephemeris> gps_ephemeris_map; //!< Map storing new Gps_Ephemeris
    Gps_Utc_Model gps_utc_model;
    Gps_Iono gps_iono;

    Sbas_Ionosphere_Correction sbas_iono;
    std::map<int,Sbas_Satellite_Correction> sbas_sat_corr_map;
    std::map<int,Sbas_Ephemeris> sbas_ephemeris_map;

    double d_GPS_current_time;

    bool d_flag_dump_enabled;
    bool d_flag_averaging;

    std::string d_dump_filename;
    std::ofstream d_dump_file;
};

#endif
