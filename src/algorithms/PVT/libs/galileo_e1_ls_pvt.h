/*!
 * \file galileo_e1_ls_pvt.h
 * \brief Interface of a Least Squares Position, Velocity, and Time (PVT)
 * solver, based on K.Borre's Matlab receiver.
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

#ifndef GNSS_SDR_GALILEO_E1_LS_PVT_H_
#define GNSS_SDR_GALILEO_E1_LS_PVT_H_

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include "ls_pvt.h"
#include "galileo_navigation_message.h"
#include "gnss_synchro.h"
#include "galileo_ephemeris.h"
#include "galileo_utc_model.h"


/*!
 * \brief This class implements a simple PVT Least Squares solution
 */
class galileo_e1_ls_pvt : public Ls_Pvt
{
public:
    galileo_e1_ls_pvt(int nchannels,std::string dump_filename, bool flag_dump_to_file);
    ~galileo_e1_ls_pvt();

    bool get_PVT(std::map<int,Gnss_Synchro> gnss_pseudoranges_map, double galileo_current_time, bool flag_averaging);

    int d_nchannels;  //!< Number of available channels for positioning

    Galileo_Navigation_Message* d_ephemeris;
    std::map<int,Galileo_Ephemeris> galileo_ephemeris_map; //!< Map storing new Galileo_Ephemeris
    Galileo_Utc_Model galileo_utc_model;
    Galileo_Iono galileo_iono;
    Galileo_Almanac galileo_almanac;

    double d_galileo_current_time;

    bool d_flag_dump_enabled;
    bool d_flag_averaging;

    std::string d_dump_filename;
    std::ofstream d_dump_file;
};

#endif
