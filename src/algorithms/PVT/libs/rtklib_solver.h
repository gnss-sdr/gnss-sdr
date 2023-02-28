/*!
 * \file rtklib_solver.h
 * \brief PVT solver based on rtklib library functions adapted to the GNSS-SDR
 *  data flow and structures
 * \authors <ul>
 *          <li> 2017, Javier Arribas
 *          <li> 2017-2023, Carles Fernandez
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
 * -----------------------------------------------------------------------------
 * Copyright (C) 2007-2013, T. Takasu
 * Copyright (C) 2017-2019, Javier Arribas
 * Copyright (C) 2017-2023, Carles Fernandez
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_RTKLIB_SOLVER_H
#define GNSS_SDR_RTKLIB_SOLVER_H


#include "beidou_dnav_almanac.h"
#include "beidou_dnav_ephemeris.h"
#include "beidou_dnav_iono.h"
#include "beidou_dnav_utc_model.h"
#include "galileo_almanac.h"
#include "galileo_ephemeris.h"
#include "galileo_has_data.h"
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
#include "rtklib_conversions.h"
#include <array>
#include <cstdint>
#include <fstream>
#include <map>
#include <string>
#include <utility>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs pvt_libs
 * Library for the computation of PVT solutions.
 * \{ */


/*!
 * \brief This class implements a PVT solution based on RTKLIB
 */
class Rtklib_Solver : public Pvt_Solution
{
public:
    Rtklib_Solver(const rtk_t& rtk,
        const std::string& dump_filename,
        uint32_t type_of_rx,
        bool flag_dump_to_file,
        bool flag_dump_to_mat,
        bool use_e6_for_pvt = true);
    ~Rtklib_Solver();

    bool get_PVT(const std::map<int, Gnss_Synchro>& gnss_observables_map, bool flag_averaging);

    double get_hdop() const override;
    double get_vdop() const override;
    double get_pdop() const override;
    double get_gdop() const override;
    Monitor_Pvt get_monitor_pvt() const;
    void store_has_data(const Galileo_HAS_data& new_has_data);
    void update_has_corrections(const std::map<int, Gnss_Synchro>& obs_map);

    sol_t pvt_sol{};
    std::array<ssat_t, MAXSAT> pvt_ssat{};

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
    bool save_matfile() const;

    void check_has_orbit_clock_validity(const std::map<int, Gnss_Synchro>& obs_map);
    void get_has_biases(const std::map<int, Gnss_Synchro>& obs_map);
    void get_current_has_obs_correction(const std::string& signal, uint32_t tow_obs, int prn);

    std::array<obsd_t, MAXOBS> d_obs_data{};
    std::array<double, 4> d_dop{};
    std::map<int, int> d_rtklib_freq_index;
    std::map<std::string, int> d_rtklib_band_index;

    std::map<std::string, std::map<int, HAS_orbit_corrections>> d_has_orbit_corrections_store_map;  // first key is system, second key is PRN
    std::map<std::string, std::map<int, HAS_clock_corrections>> d_has_clock_corrections_store_map;  // first key is system, second key is PRN

    std::map<std::string, std::map<int, std::pair<float, uint32_t>>> d_has_code_bias_store_map;   // first key is signal, second key is PRN
    std::map<std::string, std::map<int, std::pair<float, uint32_t>>> d_has_phase_bias_store_map;  // first key is signal, second key is PRN

    std::map<std::string, std::map<int, HAS_obs_corrections>> d_has_obs_corr_map;  // first key is signal, second key is PRN

    std::string d_dump_filename;
    std::ofstream d_dump_file;
    rtk_t d_rtk{};
    nav_t d_nav_data{};
    Monitor_Pvt d_monitor_pvt{};
    uint32_t d_type_of_rx;
    bool d_flag_dump_enabled;
    bool d_flag_dump_mat_enabled;
    bool d_use_e6_for_pvt;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_RTKLIB_SOLVER_H
