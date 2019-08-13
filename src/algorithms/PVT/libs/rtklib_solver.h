/*!
 * \file rtklib_solver.h
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
 * Copyright (C) 2017-2019, Javier Arribas
 * Copyright (C) 2017-2019, Carles Fernandez
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
 * -------------------------------------------------------------------------*/

#ifndef GNSS_SDR_RTKLIB_SOLVER_H_
#define GNSS_SDR_RTKLIB_SOLVER_H_


#include "beidou_dnav_almanac.h"
#include "beidou_dnav_ephemeris.h"
#include "beidou_dnav_iono.h"
#include "beidou_dnav_utc_model.h"
#include "galileo_almanac.h"
#include "galileo_ephemeris.h"
#include "galileo_iono.h"
#include "galileo_utc_model.h"
#include "glonass_gnav_almanac.h"
#include "glonass_gnav_ephemeris.h"
#include "glonass_gnav_utc_model.h"
#include "gnss_synchro.h"
#include "gps_almanac.h"
#include "gps_cnav_ephemeris.h"
#include "gps_cnav_iono.h"
#include "gps_cnav_utc_model.h"
#include "gps_ephemeris.h"
#include "gps_iono.h"
#include "gps_utc_model.h"
#include "monitor_pvt.h"
#include "pvt_solution.h"
#include "rtklib.h"
#include <array>
#include <fstream>
#include <map>
#include <string>


/*!
 * \brief This class implements a PVT solution based on RTKLIB
 */
class Rtklib_Solver : public Pvt_Solution
{
public:
    Rtklib_Solver(int nchannels, std::string dump_filename, bool flag_dump_to_file, bool flag_dump_to_mat, const rtk_t& rtk);
    ~Rtklib_Solver();

    bool get_PVT(const std::map<int, Gnss_Synchro>& gnss_observables_map, bool flag_averaging);

    sol_t pvt_sol{};
    std::array<ssat_t, MAXSAT> pvt_ssat{};
    double get_hdop() const;
    double get_vdop() const;
    double get_pdop() const;
    double get_gdop() const;
    Monitor_Pvt get_monitor_pvt() const;

    std::map<int, Galileo_Ephemeris> galileo_ephemeris_map;            //!< Map storing new Galileo_Ephemeris
    std::map<int, Gps_Ephemeris> gps_ephemeris_map;                    //!< Map storing new GPS_Ephemeris
    std::map<int, Gps_CNAV_Ephemeris> gps_cnav_ephemeris_map;          //!< Map storing new GPS_CNAV_Ephemeris
    std::map<int, Glonass_Gnav_Ephemeris> glonass_gnav_ephemeris_map;  //!< Map storing new GLONASS GNAV Ephemeris
    std::map<int, Beidou_Dnav_Ephemeris> beidou_dnav_ephemeris_map;    //!< Map storing new BeiDou DNAV Ephmeris

    Galileo_Utc_Model galileo_utc_model;
    Galileo_Iono galileo_iono;
    std::map<int, Galileo_Almanac> galileo_almanac_map;

    Gps_Utc_Model gps_utc_model;
    Gps_Iono gps_iono;
    std::map<int, Gps_Almanac> gps_almanac_map;

    Gps_CNAV_Iono gps_cnav_iono;
    Gps_CNAV_Utc_Model gps_cnav_utc_model;

    Glonass_Gnav_Utc_Model glonass_gnav_utc_model;  //!< Map storing GLONASS GNAV UTC Model
    Glonass_Gnav_Almanac glonass_gnav_almanac;      //!< Map storing GLONASS GNAV Almanac Model

    Beidou_Dnav_Utc_Model beidou_dnav_utc_model;
    Beidou_Dnav_Iono beidou_dnav_iono;
    std::map<int, Beidou_Dnav_Almanac> beidou_dnav_almanac_map;

private:
    rtk_t rtk_{};
    Monitor_Pvt monitor_pvt{};
    std::array<obsd_t, MAXOBS> obs_data{};
    std::array<double, 4> dop_{};
    std::string d_dump_filename;
    std::ofstream d_dump_file;
    int d_nchannels;  // Number of available channels for positioning
    bool d_flag_dump_enabled;
    bool d_flag_dump_mat_enabled;
    bool save_matfile();
};

#endif
