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


#include "beidou_cnav1_ephemeris.h"
#include "beidou_cnav1_iono.h"
#include "beidou_cnav1_navigation_message.h"
#include "beidou_cnav1_utc_model.h"
#include "beidou_dnav_almanac.h"
#include "beidou_dnav_ephemeris.h"
#include "beidou_dnav_iono.h"
#include "beidou_dnav_utc_model.h"
#include "galileo_almanac.h"
#include "galileo_ephemeris.h"
#include "galileo_ephemeris_store.h"
#include "galileo_has_data.h"
#include "galileo_iono.h"
#include "galileo_reduced_ced.h"
#include "galileo_utc_model.h"
#include "glonass_gnav_almanac.h"
#include "glonass_gnav_ephemeris.h"
#include "glonass_gnav_utc_model.h"
#include "gnss_synchro.h"
#include "gps_almanac.h"
#include "gps_cnav_eop.h"
#include "gps_cnav_ephemeris.h"
#include "gps_cnav_iono.h"
#include "gps_cnav_utc_model.h"
#include "gps_ephemeris.h"
#include "gps_iono.h"
#include "gps_utc_model.h"
#include "monitor_pvt.h"
#include "pvt_conf.h"
#include "pvt_kf.h"
#include "pvt_solution.h"
#include "qzss_cnav_eop.h"
#include "qzss_cnav_iono.h"
#include "qzss_cnav_utc_model.h"
#include "qzss_iono.h"
#include "qzss_utc_model.h"
#include "rtklib.h"
#include "rtklib_conversions.h"
#include "sbas_raw_message.h"
#include "sbas_rtklib_corrections.h"
#include "sensor_data/sensor_data_aggregator.h"
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
        const Pvt_Conf& conf,
        const std::string& dump_filename,
        uint32_t signal_enabled_flags,
        bool flag_dump_to_file,
        bool flag_dump_to_mat);

    ~Rtklib_Solver() noexcept override;

    bool get_PVT(const std::map<int, Gnss_Synchro>& gnss_observables_map, double kf_update_interval_s, const SensorDataAggregator& sensor_data_aggregator, bool dump_this_epoch = true);

    double get_hdop() const override;
    double get_vdop() const override;
    double get_pdop() const override;
    double get_gdop() const override;
    Monitor_Pvt get_monitor_pvt() const;
    void store_has_data(const Galileo_HAS_data& new_has_data);
    void clear_has_corrections();
    void update_has_corrections(const std::map<int, Gnss_Synchro>& obs_map);
    void store_sbas_message(const Sbas_Raw_Message& message);
    void store_gps_ephemeris(const Gps_Ephemeris& ephemeris);
    void clear_gps_ephemerides();
    bool store_galileo_ephemeris(const Galileo_Ephemeris& ephemeris);
    Galileo_Nav_Message_Type galileo_nav_message_type_for_pvt() const;
    bool is_galileo_signal_used_in_pvt(const std::string& signal) const;
    bool get_galileo_signal_health(uint32_t prn, const std::string& signal, uint32_t observation_tow, bool& healthy) const;
    std::map<int, Galileo_Ephemeris> get_galileo_ephemeris_map_for_pvt() const;
    bool select_galileo_ephemeris(uint32_t prn, const std::string& signal, uint32_t observation_tow,
        Galileo_Ephemeris& ephemeris, bool& from_reduced_ced) const;

    sol_t pvt_sol{};
    std::array<ssat_t, MAXSAT> pvt_ssat{};

    Galileo_Ephemeris_Store galileo_ephemeris_store;                   //!< Source-aware Galileo ephemeris storage
    std::map<int, Galileo_Ephemeris> galileo_ephemeris_map;            //!< Compatibility PVT view; source-aware storage is authoritative
    std::map<int, Galileo_Reduced_CED> galileo_reduced_ced_map;        //!< Map storing provisional Galileo Reduced CED
    std::map<int, Gps_Ephemeris> gps_ephemeris_map;                    //!< Map storing new GPS_Ephemeris
    std::map<int, Gps_Ephemeris> gps_previous_ephemeris_map;           //!< Previous GPS/QZSS issue retained for SBAS IODE handover
    std::map<int, Gps_CNAV_Ephemeris> gps_cnav_ephemeris_map;          //!< Map storing new GPS_CNAV_Ephemeris
    std::map<int, Glonass_Gnav_Ephemeris> glonass_gnav_ephemeris_map;  //!< Map storing new GLONASS GNAV Ephemeris
    std::map<int, Beidou_Dnav_Ephemeris> beidou_dnav_ephemeris_map;    //!< Map storing new BeiDou DNAV Ephmeris
    std::map<int, Beidou_Cnav1_Ephemeris> beidou_cnav1_ephemeris_map;  //!< Map storing BeiDou B-CNAV1 ephemeris

    Galileo_Utc_Model galileo_utc_model;
    Galileo_Iono galileo_iono;
    std::map<int, Galileo_Almanac> galileo_almanac_map;

    Gps_Utc_Model gps_utc_model;
    Gps_Iono gps_iono;
    std::map<int, Gps_Almanac> gps_almanac_map;

    Gps_CNAV_Eop gps_cnav_eop;
    Gps_CNAV_Iono gps_cnav_iono;
    Gps_CNAV_Utc_Model gps_cnav_utc_model;

    Qzss_Utc_Model qzss_utc_model;
    Qzss_Iono qzss_iono;

    Qzss_CNAV_Eop qzss_cnav_eop;
    Qzss_CNAV_Iono qzss_cnav_iono;
    Qzss_CNAV_Utc_Model qzss_cnav_utc_model;

    Glonass_Gnav_Utc_Model glonass_gnav_utc_model;  //!< Map storing GLONASS GNAV UTC Model
    Glonass_Gnav_Almanac glonass_gnav_almanac;      //!< Map storing GLONASS GNAV Almanac Model

    Beidou_Dnav_Utc_Model beidou_dnav_utc_model;
    Beidou_Dnav_Iono beidou_dnav_iono;
    Beidou_Cnav1_Iono beidou_cnav1_iono;
    Beidou_Cnav1_Utc_Model beidou_cnav1_utc_model;
    std::map<int, Beidou_Dnav_Almanac> beidou_dnav_almanac_map;
    std::map<int, Bds3_B1c_PageData> beidou_cnav1_page_data_map;

private:
    friend class GalileoEphemerisSourceTest_E6SlotsFollowRtklibGalileoPolicy_Test;

    bool save_matfile() const;
    bool galileo_ephemeris_is_usable(const Galileo_Ephemeris& ephemeris, uint32_t observation_tow) const;
    void update_galileo_observation_wavelengths(const obsd_t& observation);

    void check_has_orbit_clock_validity(const std::map<int, Gnss_Synchro>& obs_map);
    void get_has_biases(const std::map<int, Gnss_Synchro>& obs_map);
    void get_current_has_obs_correction(const std::string& signal, uint32_t tow_obs, int prn, uint8_t mask_id, uint8_t iod_set_id);
    void clear_applied_has_phase_bias_discontinuity(const HAS_obs_corrections* has_correction, int prn);
    bool has_active_has_do_not_use(const std::string& system, int prn, uint32_t tow_obs) const;
    bool has_active_has_orbit_clock(const std::string& system, int prn, uint16_t sis_iod, uint32_t tow_obs) const;
    bool get_active_has_context(const std::string& system, int prn, uint16_t sis_iod, uint32_t tow_obs, uint8_t& mask_id, uint8_t& iod_set_id) const;

    std::array<obsd_t, MAXOBS> d_obs_data{};
    std::array<double, 4> d_dop{};
    std::map<int, int> d_rtklib_freq_index;
    std::map<std::string, int> d_rtklib_band_index;

    std::map<std::string, std::map<int, HAS_orbit_corrections>> d_has_orbit_corrections_store_map;  // first key is system, second key is PRN
    std::map<std::string, std::map<int, HAS_clock_corrections>> d_has_clock_corrections_store_map;  // first key is system, second key is PRN
    std::map<std::string, std::map<int, uint32_t>> d_has_satellite_do_not_use_until;                // first key is system, second key is PRN

    std::map<std::string, std::map<int, HAS_bias_corrections>> d_has_code_bias_store_map;         // first key is signal, second key is PRN
    std::map<std::string, std::map<int, HAS_bias_corrections>> d_has_phase_bias_store_map;        // first key is signal, second key is PRN
    std::map<std::string, std::map<int, uint8_t>> d_has_phase_discontinuity_indicator_store_map;  // first key is signal, second key is PRN
    std::map<std::string, std::map<int, bool>> d_has_phase_bias_discontinuity_map;                // first key is signal, second key is PRN

    std::map<std::string, std::map<int, HAS_obs_corrections>> d_has_obs_corr_map;  // first key is signal, second key is PRN

    std::string d_dump_filename;
    std::ofstream d_dump_file;
    rtk_t d_rtk{};
    Sbas_Rtklib_Corrections d_sbas_corrections;
    nav_t d_nav_data{};
    Monitor_Pvt d_monitor_pvt{};
    Pvt_Conf d_conf;
    Pvt_Kf d_pvt_kf;
    uint32_t d_signal_enabled_flags;
    Galileo_Nav_Message_Type d_galileo_nav_message_type_for_pvt{Galileo_Nav_Message_Type::INAV};
    bool d_flag_dump_enabled;
    bool d_flag_dump_mat_enabled;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_RTKLIB_SOLVER_H
