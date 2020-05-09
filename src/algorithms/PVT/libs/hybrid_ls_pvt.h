/*!
 * \file hybrid_ls_pvt.h
 * \brief Interface of a Least Squares Position, Velocity, and Time (PVT)
 * solver, based on K.Borre's Matlab receiver.
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_HYBRID_LS_PVT_H
#define GNSS_SDR_HYBRID_LS_PVT_H

#include "galileo_almanac.h"
#include "galileo_navigation_message.h"
#include "gnss_synchro.h"
#include "gps_cnav_navigation_message.h"
#include "gps_navigation_message.h"
#include "ls_pvt.h"
#include "rtklib_rtkcmn.h"
#include <fstream>
#include <map>
#include <string>

/*!
 * \brief This class implements a simple PVT Least Squares solution
 */
class Hybrid_Ls_Pvt : public Ls_Pvt
{
public:
    Hybrid_Ls_Pvt(int nchannels, std::string dump_filename, bool flag_dump_to_file);
    ~Hybrid_Ls_Pvt();

    bool get_PVT(std::map<int, Gnss_Synchro> gnss_observables_map, double hybrid_current_time, bool flag_averaging);

    std::map<int, Galileo_Ephemeris> galileo_ephemeris_map;  //!< Map storing new Galileo_Ephemeris
    std::map<int, Gps_Ephemeris> gps_ephemeris_map;          //!< Map storing new GPS_Ephemeris
    std::map<int, Gps_CNAV_Ephemeris> gps_cnav_ephemeris_map;

    Galileo_Utc_Model galileo_utc_model;
    Galileo_Iono galileo_iono;
    Galileo_Almanac galileo_almanac;

    Gps_Utc_Model gps_utc_model;
    Gps_Iono gps_iono;

    Gps_CNAV_Iono gps_cnav_iono;
    Gps_CNAV_Utc_Model gps_cnav_utc_model;

private:
    bool d_flag_dump_enabled;
    std::string d_dump_filename;
    std::ofstream d_dump_file;
    int d_nchannels;  // Number of available channels for positioning
    double d_galileo_current_time;
};

#endif
