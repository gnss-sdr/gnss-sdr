/*!
 * \file gnss_sdr_supl_client.h
 * \brief class that implements a C++ interface to external Secure User Location Protocol (SUPL) client library.
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 *
 * TODO: put here supl.c author info
 * class that implements a C++ interface to external Secure User Location Protocol (SUPL) client library.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_SUPL_CLIENT_H_
#define GNSS_SDR_SUPL_CLIENT_H_

extern "C"
{
#include "supl.h"
}
#include "GPS_L1_CA.h"
#include "gps_ephemeris.h"
#include "gps_iono.h"
#include "gps_almanac.h"
#include "gps_utc_model.h"
#include "gps_acq_assist.h"
#include "gps_ref_time.h"
#include "gps_ref_location.h"
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <glog/logging.h>
#include <fstream>
#include <map>
#include <string>

/*!
 * \brief class that implements a C++ interface to external Secure User Location Protocol (SUPL) client library..
 */
class gnss_sdr_supl_client
{
private:
    // GSM CELL INFO
    int mcc;
    int mns;
    int lac;
    int ci;
    // assistance protocol structure
    supl_ctx_t ctx;
    // assistance data
    supl_assist_t assist;

public:
    // SUPL SERVER INFO
    std::string server_name;
    int server_port;
    int request;
    // ephemeris map
    std::map<int, Gps_Ephemeris> gps_ephemeris_map;
    // almanac map
    std::map<int, Gps_Almanac> gps_almanac_map;

    // ionospheric model
    Gps_Iono gps_iono;
    // reference time
    Gps_Ref_Time gps_time;
    // UTC model
    Gps_Utc_Model gps_utc;
    // reference location
    Gps_Ref_Location gps_ref_loc;
    // Acquisition Assistance map
    std::map<int, Gps_Acq_Assist> gps_acq_map;

    /*
     * \brief Initiates the TCP SSL SUPL connection to the SUPL server and request assistance data using the provided GSM Base station parameters
     * \param i_mcc Current network MCC (Mobile country code), 3 digits.
     * \param i_mns Current network MNC (Mobile Network code), 2 or 3 digits.
     * \param i_lac Current network LAC (Location area code),16 bits, 1-65520 are valid values.
     * \param i_ci Cell Identity (16 bits, 0-65535 are valid values).
     * \return Error code -> 0 no errors.
     */
    int get_assistance(int i_mcc, int i_mns, int i_lac, int i_ci);
    /*
     * \brief Read the received SUPL data and stores it into the corresponding class members (gps_ephemeris_map, gps_almanac_map, gps_iono, gps_time, gps_utc, gps_acq_map, and gps_ref_loc)
     *
     */
    void read_supl_data();

    /*!
     * \brief Read ephemeris map from XML file
     */
    bool load_ephemeris_xml(const std::string file_name);

    /*!
     * \brief Save ephemeris map to XML file.
     */
    bool save_ephemeris_map_xml(const std::string file_name,
        std::map<int, Gps_Ephemeris> eph_map);

    /*!
     * \brief Read utc model from XML file
     */
    bool load_utc_xml(const std::string file_name);

    /*!
     * \brief Save utc model map to XML file
     * To be called by ControlThread::gps_utc_model_data_write_to_XML()
     */
    bool save_utc_map_xml(const std::string file_name,
        std::map<int, Gps_Utc_Model> utc_map);

    /*!
     * \brief Read iono from XML file
     */
    bool load_iono_xml(const std::string file_name);

    /*!
     * \brief Save iono map to XML file
     */
    bool save_iono_map_xml(const std::string file_name,
        std::map<int, Gps_Iono> iono_map);

    /*!
     * \brief Read ref time from XML file
     */
    bool load_ref_time_xml(const std::string file_name);

    /*!
     * \brief Save ref time map to XML file
     */
    bool save_ref_time_map_xml(const std::string file_name,
        std::map<int, Gps_Ref_Time> ref_time_map);

    /*!
     * \brief Read ref location from XML file
     */
    bool load_ref_location_xml(const std::string file_name);

    /*!
     * \brief Save ref location map to XML file
     */
    bool save_ref_location_map_xml(std::string file_name,
        std::map<int, Gps_Ref_Location> ref_location_map);

    /*
     * Prints SUPL data to std::cout. Use it for debug purposes only.
     */
    void print_assistance();
    gnss_sdr_supl_client();
    ~gnss_sdr_supl_client();
};

#endif
