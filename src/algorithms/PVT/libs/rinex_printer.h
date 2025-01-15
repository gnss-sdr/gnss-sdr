/*!
 * \file rinex_printer.h
 * \brief Interface of a RINEX 2.11 / 3.01 printer
 * See ftp://igs.org/pub/data/format/rinex301.pdf
 *
 * Receiver Independent EXchange Format (RINEX):
 * The first proposal for the Receiver Independent Exchange Format RINEX
 * was developed by the Astronomical Institute of the University of Berne
 * for the easy exchange of the GPS data to be collected during the large
 * European GPS campaign EUREF 89, which involved more than 60 GPS receivers
 * of 4 different manufacturers.
 * The governing aspect during the development was the fact that most geodetic
 * processing software for GPS data use a well-defined set of observables:
 * 1) The carrier-phase measurement at one or both carriers (actually being a
 * measurement on the beat frequency between the received carrier of the
 * satellite signal and a receiver-generated reference frequency).
 * 2) The pseudorange (code) measurement , equivalent to the difference
 * of the time of reception (expressed in the time frame of the receiver)
 * and the time of transmission (expressed in the time frame of the satellite)
 * of a distinct satellite signal.
 * 3) The observation time being the reading of the receiver clock at the
 * instant of validity of the carrier-phase and/or the code measurements.
 * Note: A collection of the formats currently used by the IGS can be found
 * here: https://kb.igs.org/hc/en-us/articles/201096516-IGS-Formats
 * \author Carles Fernandez Prades, 2011. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_RINEX_PRINTER_H
#define GNSS_SDR_RINEX_PRINTER_H

#include <boost/date_time/posix_time/posix_time.hpp>
#include <cstdint>        // for int32_t
#include <cstdlib>        // for strtol, strtod
#include <fstream>        // for fstream
#include <iomanip>        // for setprecision
#include <map>            // for map
#include <sstream>        // for stringstream
#include <string>         // for string
#include <unordered_map>  // for unordered_map
#include <vector>         // for vector


/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */


class Beidou_Dnav_Ephemeris;
class Beidou_Dnav_Iono;
class Beidou_Dnav_Utc_Model;
class Galileo_Ephemeris;
class Galileo_Iono;
class Galileo_Utc_Model;
class Glonass_Gnav_Almanac;
class Glonass_Gnav_Ephemeris;
class Glonass_Gnav_Utc_Model;
class Gnss_Synchro;
class Gps_CNAV_Ephemeris;
class Gps_CNAV_Iono;
class Gps_CNAV_Utc_Model;
class Gps_Ephemeris;
class Gps_Iono;
class Gps_Navigation_Message;
class Gps_Utc_Model;
class Rtklib_Solver;


/*!
 * \brief Class that handles the generation of Receiver
 * INdependent EXchange format (RINEX) files
 */
class Rinex_Printer
{
public:
    /*!
     * \brief Constructor. Creates GNSS Navigation and Observables RINEX files.
     */
    explicit Rinex_Printer(int version = 0,
        const std::string& base_path = ".",
        const std::string& base_name = "-");

    /*!
     * \brief Destructor. Removes created files if empty.
     */
    ~Rinex_Printer();

    /*!
     * \brief Print RINEX annotation. If it is the first annotation, it also
     * prints the RINEX headers for navigation and observation files. If it is
     * not the first annotation, it only annotates the observation, and updates
     * the navigation header if UTC data was not available when writing it for
     * the first time. The meaning of type_of_rx is as follows:
     *
     * type_of_rx    | Signals
     * ------------- | -------------
     *     0   |  Unknown
     *     1   |  GPS L1 C/A
     *     2   |  GPS L2C
     *     3   |  GPS L5
     *     4   |  Galileo E1B
     *     5   |  Galileo E5a
     *     6   |  Galileo E5b
     *     7   |  GPS L1 C/A + GPS L2C
     *     8   |  GPS L1 C/A + GPS L5
     *     9   |  GPS L1 C/A + Galileo E1B
     *    10   |  GPS L1 C/A + Galileo E5a
     *    11   |  GPS L1 C/A + Galileo E5b
     *    12   |  Galileo E1B + GPS L2C
     *    13   |  Galileo E5a + GPS L5
     *    14   |  Galileo E1B + Galileo E5a
     *    15   |  Galileo E1B + Galileo E5b
     *    16   |  GPS L2C + GPS L5
     *    17   |  GPS L2C + Galileo E5a
     *    20   |  GPS L5 + Galileo E5b
     *    21   |  GPS L1 C/A + Galileo E1B + GPS L2C
     *    22   |  GPS L1 C/A + Galileo E1B + GPS L5
     *    23   |  GLONASS L1 C/A
     *    24   |  GLONASS L2 C/A
     *    25   |  GLONASS L1 C/A + GLONASS L2 C/A
     *    26   |  GPS L1 C/A + GLONASS L1 C/A
     *    27   |  Galileo E1B + GLONASS L1 C/A
     *    28   |  GPS L2C + GLONASS L1 C/A
     *    29   |  GPS L1 C/A + GLONASS L2 C/A
     *    30   |  Galileo E1B + GLONASS L2 C/A
     *    31   |  GPS L2C + GLONASS L2 C/A
     *    32   |  GPS L1 C/A + Galileo E1B + GPS L5 + Galileo E5a
     *    33   |  GPS L1 C/A + Galileo E1B + Galileo E5a
     *    100   |  Galileo E6B
     *    101   |  Galileo E1B + Galileo E6B
     *    102   |  Galileo E5a + Galileo E6B
     *    103   |  Galileo E5b + Galileo E6B
     *    104   |  Galileo E1B + Galileo E5a + Galileo E6B
     *    105   |  Galileo E1B + Galileo E5b + Galileo E6B
     *    106   |  GPS L1 C/A + Galileo E1B + Galileo E6B
     *    107   |  GPS L1 C/A + Galileo E6B
     *    108   |  GPS L1 C/A + Galileo E1B + GPS L5 + Galileo E5a + Galileo E6B
     *    500   |  BeiDou B1I
     *    501   |  BeiDou B1I + GPS L1 C/A
     *    502   |  BeiDou B1I + Galileo E1B
     *    503   |  BeiDou B1I + GLONASS L1 C/A
     *    504   |  BeiDou B1I + GPS L1 C/A + Galileo E1B
     *    505   |  BeiDou B1I + GPS L1 C/A + GLONASS L1 C/A + Galileo E1B
     *    506   |  BeiDou B1I + Beidou B3I
     *    600   |  BeiDou B3I
     *    601   |  BeiDou B3I + GPS L2C
     *    602   |  BeiDou B3I + GLONASS L2 C/A
     *    603   |  BeiDou B3I + GPS L2C + GLONASS L2 C/A
     *    604   |  BeiDou B3I + GPS L1 C/A
     *    605   |  BeiDou B3I + Galileo E1B
     *    606   |  BeiDou B3I + GLONASS L1 C/A
     *    607   |  BeiDou B3I + GPS L1 C/A + Galileo E1B
     *    608   |  BeiDou B3I + GPS L1 C/A + Galileo E1B + BeiDou B1I
     *    609   |  BeiDou B3I + GPS L1 C/A + Galileo E1B + GLONASS L1 C/A
     *    610   |  BeiDou B3I + GPS L1 C/A + Galileo E1B + GLONASS L1 C/A + BeiDou B1I
     *    1000  |  GPS L1 C/A + GPS L2C + GPS L5
     *    1001  |  GPS L1 C/A + Galileo E1B + GPS L2C + GPS L5 + Galileo E5a
     *
     */
    void print_rinex_annotation(const Rtklib_Solver* pvt_solver,
        const std::map<int, Gnss_Synchro>& gnss_observables_map,
        double rx_time,
        int type_of_rx,
        bool flag_write_RINEX_obs_output);

    /*!
     * \brief Print RINEX annotation for GPS NAV message
     */
    void log_rinex_nav_gps_nav(int type_of_rx,
        const std::map<int32_t, Gps_Ephemeris>& new_eph);

    /*!
     * \brief Print RINEX annotation for GPS CNAV message
     */
    void log_rinex_nav_gps_cnav(int type_of_rx,
        const std::map<int32_t, Gps_CNAV_Ephemeris>& new_cnav_eph);

    /*!
     * \brief Print RINEX annotation for Galileo NAV message
     */
    void log_rinex_nav_gal_nav(int type_of_rx,
        const std::map<int32_t, Galileo_Ephemeris>& new_gal_eph);

    /*!
     * \brief Print RINEX annotation for Glonass GNAV message
     */
    void log_rinex_nav_glo_gnav(int type_of_rx,
        const std::map<int32_t, Glonass_Gnav_Ephemeris>& new_glo_eph);

    /*!
     * \brief Print RINEX annotation for BeiDou DNAV message
     */
    void log_rinex_nav_bds_dnav(int type_of_rx,
        const std::map<int32_t, Beidou_Dnav_Ephemeris>& new_bds_eph);

    /*!
     * \brief Set processing for signals older than 2009
     */
    void set_pre_2009_file(bool pre_2009_file);

    /*!
     * \brief Returns true is the RINEX file headers are already written
     */
    inline bool is_rinex_header_written() const
    {
        return d_rinex_header_written;
    }

    /*!
     * \brief Returns name of RINEX navigation file(s)
     */
    inline std::vector<std::string> get_navfilename() const
    {
        return output_navfilename;
    }

    /*!
     * \brief Returns name of RINEX observation file
     */
    inline std::string get_obsfilename() const
    {
        return obsfilename;
    }


private:
    const std::unordered_map<std::string, std::string> satelliteSystem = {
        {"GPS", "G"},
        {"GLONASS", "R"},
        {"SBAS payload", "S"},
        {"Galileo", "E"},
        {"Beidou", "C"},
        {"Mixed", "M"}};  // RINEX v3.02 codes

    /*
     * Generates the GPS Observation data header
     */
    void rinex_obs_header(std::fstream& out,
        const Gps_Ephemeris& eph,
        double d_TOW_first_observation);

    /*
     * Generates the GPS L2 Observation data header
     */
    void rinex_obs_header(std::fstream& out,
        const Gps_CNAV_Ephemeris& eph,
        double d_TOW_first_observation,
        const std::string& gps_bands = "2S");

    /*
     * Generates the dual frequency GPS L1 & L2/L5 Observation data header
     */
    void rinex_obs_header(std::fstream& out,
        const Gps_Ephemeris& eph,
        const Gps_CNAV_Ephemeris& eph_cnav,
        double d_TOW_first_observation,
        const std::string& gps_bands = "1C 2S");

    /*
     * Generates the Galileo Observation data header.
     * Example: bands("1B"), bands("1B 5X"), bands("5X"), ... Default: "1B".
     */
    void rinex_obs_header(std::fstream& out,
        const Galileo_Ephemeris& eph,
        double d_TOW_first_observation,
        const std::string& bands = "1B");

    /*
     * Generates the Mixed (GPS/Galileo) Observation data header.
     * Example: galileo_bands("1B"), galileo_bands("1B 5X"),
     * galileo_bands("5X"), ... Default: "1B".
     */
    void rinex_obs_header(std::fstream& out,
        const Gps_Ephemeris& gps_eph,
        const Galileo_Ephemeris& galileo_eph,
        double d_TOW_first_observation,
        const std::string& galileo_bands = "1B");

    /*
     * Generates the Mixed (GPS/Galileo) Observation data header.
     * Example: galileo_bands("1B"), galileo_bands("1B 5X"), galileo_bands("5X"), ... Default: "1B".
     */
    void rinex_obs_header(std::fstream& out,
        const Gps_Ephemeris& gps_eph,
        const Gps_CNAV_Ephemeris& eph_cnav,
        const Galileo_Ephemeris& galileo_eph,
        double d_TOW_first_observation,
        const std::string& gps_bands = "1C 2S",
        const std::string& galileo_bands = "1B");

    /*
     * Generates the Mixed (GPS/Galileo) Observation data header.
     * Example: galileo_bands("1B"), galileo_bands("1B 5X"), galileo_bands("5X"), ... Default: "1B".
     */
    void rinex_obs_header(std::fstream& out,
        const Gps_CNAV_Ephemeris& eph_cnav,
        const Galileo_Ephemeris& galileo_eph,
        double d_TOW_first_observation,
        const std::string& gps_bands = "2S",
        const std::string& galileo_bands = "1B");

    /*
     * Generates the GLONASS GNAV Observation data header.
     * Example: bands("1C"), bands("1C 2C"), bands("2C"), ... Default: "1C".
     */
    void rinex_obs_header(std::fstream& out,
        const Glonass_Gnav_Ephemeris& eph,
        double d_TOW_first_observation,
        const std::string& bands = "1G");

    /*
     * Generates the Mixed (GPS L1 C/A /GLONASS) Observation data header.
     * Example: galileo_bands("1C"), galileo_bands("1B 5X"), galileo_bands("5X"), ... Default: "1B".
     */
    void rinex_obs_header(std::fstream& out,
        const Gps_Ephemeris& gps_eph,
        const Glonass_Gnav_Ephemeris& glonass_gnav_eph,
        double d_TOW_first_observation,
        const std::string& glonass_bands = "1C");

    /*
     * Generates the Mixed (Galileo/GLONASS) Observation data header.
     * Example: galileo_bands("1C"), galileo_bands("1B 5X"), galileo_bands("5X"), ... Default: "1B".
     */
    void rinex_obs_header(std::fstream& out,
        const Galileo_Ephemeris& galileo_eph,
        const Glonass_Gnav_Ephemeris& glonass_gnav_eph,
        double d_TOW_first_observation,
        const std::string& galileo_bands = "1B",
        const std::string& glonass_bands = "1C");

    /*
     * Generates the Mixed (GPS L2C/GLONASS) Observation data header.
     * Example: galileo_bands("1G")... Default: "1G".
     */
    void rinex_obs_header(std::fstream& out,
        const Gps_CNAV_Ephemeris& gps_cnav_eph,
        const Glonass_Gnav_Ephemeris& glonass_gnav_eph,
        double d_TOW_first_observation,
        const std::string& glonass_bands = "1G");

    /*
     * Generates the a Beidou B1I Observation data header. Example: beidou_bands("B1")
     */
    void rinex_obs_header(std::fstream& out,
        const Beidou_Dnav_Ephemeris& eph,
        double d_TOW_first_observation,
        const std::string& bands);

    /*
     * Generates the SBAS raw data header
     */
    void rinex_sbs_header(std::fstream& out) const;

    /*
     * Writes GPS L1 observables into the RINEX file
     */
    void log_rinex_obs(std::fstream& out,
        const Gps_Ephemeris& eph,
        double obs_time,
        const std::map<int32_t, Gnss_Synchro>& observables) const;

    /*
     * Writes GPS L2 observables into the RINEX file
     */
    void log_rinex_obs(std::fstream& out,
        const Gps_CNAV_Ephemeris& eph,
        double obs_time,
        const std::map<int32_t, Gnss_Synchro>& observables) const;

    /*
     * Writes dual frequency GPS L1 and L2 observables into the RINEX file
     */
    void log_rinex_obs(std::fstream& out,
        const Gps_Ephemeris& eph,
        const Gps_CNAV_Ephemeris& eph_cnav,
        double obs_time,
        const std::map<int32_t, Gnss_Synchro>& observables,
        bool triple_band = false) const;

    /*
     * Writes Galileo observables into the RINEX file.
     * Example: galileo_bands("1B"), galileo_bands("1B 5X"), galileo_bands("5X"), ... Default: "1B".
     */
    void log_rinex_obs(std::fstream& out,
        const Galileo_Ephemeris& eph,
        double obs_time,
        const std::map<int32_t, Gnss_Synchro>& observables,
        const std::string& galileo_bands = "1B") const;

    /*
     * Writes Mixed GPS / Galileo observables into the RINEX file
     */
    void log_rinex_obs(std::fstream& out,
        const Gps_Ephemeris& gps_eph,
        const Galileo_Ephemeris& galileo_eph,
        double gps_obs_time,
        const std::map<int32_t, Gnss_Synchro>& observables) const;

    /*
     * Writes Mixed GPS / Galileo observables into the RINEX file
     */
    void log_rinex_obs(std::fstream& out,
        const Gps_CNAV_Ephemeris& eph,
        const Galileo_Ephemeris& galileo_eph,
        double gps_obs_time,
        const std::map<int32_t, Gnss_Synchro>& observables) const;

    /*
     * Writes Mixed GPS / Galileo observables into the RINEX file
     */
    void log_rinex_obs(std::fstream& out,
        const Gps_Ephemeris& gps_eph,
        const Gps_CNAV_Ephemeris& gps_cnav_eph,
        const Galileo_Ephemeris& galileo_eph,
        double gps_obs_time,
        const std::map<int32_t, Gnss_Synchro>& observables,
        bool triple_band = false) const;

    /*
     * Writes GLONASS GNAV observables into the RINEX file.
     * Example: glonass_bands("1C"), galileo_bands("1B 5X"), galileo_bands("5X"), ... Default: "1B".
     */
    void log_rinex_obs(std::fstream& out,
        const Glonass_Gnav_Ephemeris& eph,
        double obs_time,
        const std::map<int32_t, Gnss_Synchro>& observables,
        const std::string& glonass_bands = "1C") const;

    /*
     * Writes Mixed GPS L1 C/A - GLONASS observables into the RINEX file
     */
    void log_rinex_obs(std::fstream& out,
        const Gps_Ephemeris& gps_eph,
        const Glonass_Gnav_Ephemeris& glonass_gnav_eph,
        double gps_obs_time,
        const std::map<int32_t, Gnss_Synchro>& observables) const;

    /*
     * Writes Mixed GPS L2C - GLONASS observables into the RINEX file
     */
    void log_rinex_obs(std::fstream& out,
        const Gps_CNAV_Ephemeris& gps_eph,
        const Glonass_Gnav_Ephemeris& glonass_gnav_eph,
        double gps_obs_time,
        const std::map<int32_t, Gnss_Synchro>& observables) const;

    /*
     * Writes Mixed Galileo/GLONASS observables into the RINEX file
     */
    void log_rinex_obs(std::fstream& out,
        const Galileo_Ephemeris& galileo_eph,
        const Glonass_Gnav_Ephemeris& glonass_gnav_eph,
        double galileo_obs_time,
        const std::map<int32_t, Gnss_Synchro>& observables) const;

    /*
     * Writes BDS B1I observables into the RINEX file
     */
    void log_rinex_obs(std::fstream& out,
        const Beidou_Dnav_Ephemeris& eph,
        double obs_time,
        const std::map<int32_t, Gnss_Synchro>& observables,
        const std::string& bds_bands) const;

    /*
     * Generates the GPS L1 C/A Navigation Data header
     */
    void rinex_nav_header(std::fstream& out,
        const Gps_Iono& iono,
        const Gps_Utc_Model& utc_model,
        const Gps_Ephemeris& eph) const;

    /*
     * Generates the GPS L2C(M) Navigation Data header
     */
    void rinex_nav_header(std::fstream& out,
        const Gps_CNAV_Iono& iono,
        const Gps_CNAV_Utc_Model& utc_model) const;

    /*
     * Generates the Galileo Navigation Data header
     */
    void rinex_nav_header(std::fstream& out,
        const Galileo_Iono& iono,
        const Galileo_Utc_Model& utc_model) const;

    /*
     * Generates the Mixed (GPS/Galileo) Navigation Data header
     */
    void rinex_nav_header(std::fstream& out,
        const Gps_Iono& gps_iono,
        const Gps_Utc_Model& gps_utc_model,
        const Gps_Ephemeris& eph,
        const Galileo_Iono& galileo_iono,
        const Galileo_Utc_Model& galileo_utc_model) const;

    /*
     * Generates the Mixed (GPS CNAV/Galileo) Navigation Data header
     */
    void rinex_nav_header(std::fstream& out,
        const Gps_CNAV_Iono& iono,
        const Gps_CNAV_Utc_Model& utc_model,
        const Galileo_Iono& galileo_iono,
        const Galileo_Utc_Model& galileo_utc_model) const;

    /*
     * Generates the GLONASS L1, L2 C/A Navigation Data header
     */
    void rinex_nav_header(std::fstream& out,
        const Glonass_Gnav_Utc_Model& utc_model,
        const Glonass_Gnav_Ephemeris& glonass_gnav_eph);

    /*
     * Generates the Mixed (Galileo/GLONASS) Navigation Data header
     */
    void rinex_nav_header(std::fstream& out,
        const Galileo_Iono& galileo_iono,
        const Galileo_Utc_Model& galileo_utc_model,
        const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model,
        const Glonass_Gnav_Almanac& glonass_gnav_almanac) const;

    /*
     * Generates the Mixed (GPS L1 C/A/GLONASS L1, L2) Navigation Data header
     */
    void rinex_nav_header(std::fstream& out,
        const Gps_Iono& gps_iono,
        const Gps_Utc_Model& gps_utc_model,
        const Gps_Ephemeris& eph,
        const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model,
        const Glonass_Gnav_Almanac& glonass_gnav_almanac);

    /*
     * Generates the Mixed (GPS L2C C/A/GLONASS L1, L2) Navigation Data header
     */
    void rinex_nav_header(std::fstream& out,
        const Gps_CNAV_Iono& gps_iono,
        const Gps_CNAV_Utc_Model& gps_utc_model,
        const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model,
        const Glonass_Gnav_Almanac& glonass_gnav_almanac);

    /*
     * Generates the BDS B1I or B3I Navigation Data header
     */
    void rinex_nav_header(std::fstream& out,
        const Beidou_Dnav_Iono& iono,
        const Beidou_Dnav_Utc_Model& utc_model) const;

    /*
     * Generates the Mixed GPS L1,L5 + BDS B1I, B3I Navigation Data header
     */
    void rinex_nav_header(std::fstream& out,
        const Gps_Iono& gps_iono,
        const Gps_Utc_Model& gps_utc_model,
        const Gps_Ephemeris& eph,
        const Beidou_Dnav_Iono& bds_dnav_iono,
        const Beidou_Dnav_Utc_Model& bds_dnav_utc_model) const;

    /*
     * Generates the Mixed GPS L2C + BDS B1I, B3I Navigation Data header
     */
    void rinex_nav_header(std::fstream& out,
        const Gps_CNAV_Iono& gps_cnav_iono,
        const Gps_CNAV_Utc_Model& gps_cnav_utc_model,
        const Beidou_Dnav_Iono& bds_dnav_iono,
        const Beidou_Dnav_Utc_Model& bds_dnav_utc_model);

    /*
     * Generates the Mixed GLONASS L1,L2 + BDS B1I, B3I Navigation Data header
     */
    void rinex_nav_header(std::fstream& out,
        const Glonass_Gnav_Utc_Model& glo_gnav_utc_model,
        const Beidou_Dnav_Iono& bds_dnav_iono,
        const Beidou_Dnav_Utc_Model& bds_dnav_utc_model) const;

    /*
     * Generates the Mixed (Galileo/BDS B1I, B3I) Navigation Data header
     */
    void rinex_nav_header(std::fstream& out,
        const Galileo_Iono& galileo_iono,
        const Galileo_Utc_Model& galileo_utc_model,
        const Beidou_Dnav_Iono& bds_dnav_iono,
        const Beidou_Dnav_Utc_Model& bds_dnav_utc_model) const;

    /*
     * Writes data from the GPS L1 C/A navigation message into the RINEX file
     */
    void log_rinex_nav(std::fstream& out,
        const std::map<int32_t, Gps_Ephemeris>& eph_map) const;

    /*
     * Writes data from the GPS L2 navigation message into the RINEX file
     */
    void log_rinex_nav(std::fstream& out,
        const std::map<int32_t, Gps_CNAV_Ephemeris>& eph_map);

    /*
     * Writes data from the Galileo navigation message into the RINEX file
     */
    void log_rinex_nav(std::fstream& out,
        const std::map<int32_t, Galileo_Ephemeris>& eph_map) const;

    /*
     * Writes data from the Mixed (GPS/Galileo) navigation message into the RINEX file
     */
    void log_rinex_nav(std::fstream& out,
        const std::map<int32_t, Gps_Ephemeris>& gps_eph_map,
        const std::map<int32_t, Galileo_Ephemeris>& galileo_eph_map);

    /*
     * Writes data from the Mixed (GPS/Galileo) navigation message into the RINEX file
     */
    void log_rinex_nav(std::fstream& out,
        const std::map<int32_t, Gps_CNAV_Ephemeris>& gps_cnav_eph_map,
        const std::map<int32_t, Galileo_Ephemeris>& galileo_eph_map);

    /*
     * Writes data from the GLONASS GNAV navigation message into the RINEX file
     */
    void log_rinex_nav(std::fstream& out,
        const std::map<int32_t, Glonass_Gnav_Ephemeris>& eph_map) const;

    /*
     * Writes data from the Mixed (GPS/GLONASS GNAV) navigation message into the RINEX file
     */
    void log_rinex_nav(std::fstream& out,
        const std::map<int32_t, Gps_Ephemeris>& gps_eph_map,
        const std::map<int32_t, Glonass_Gnav_Ephemeris>& glonass_gnav_eph_map) const;

    /*
     * Writes data from the Mixed (GPS/GLONASS GNAV) navigation message into the RINEX file
     */
    void log_rinex_nav(std::fstream& out,
        const std::map<int32_t, Gps_CNAV_Ephemeris>& gps_cnav_eph_map,
        const std::map<int32_t, Glonass_Gnav_Ephemeris>& glonass_gnav_eph_map);

    /*
     * Writes data from the Mixed (Galileo/ GLONASS GNAV) navigation message into the RINEX file
     */
    void log_rinex_nav(std::fstream& out,
        const std::map<int32_t, Galileo_Ephemeris>& galileo_eph_map,
        const std::map<int32_t, Glonass_Gnav_Ephemeris>& glonass_gnav_eph_map);

    /*
     * Writes data from the Beidou B1I navigation message into the RINEX file
     */
    void log_rinex_nav(std::fstream& out,
        const std::map<int32_t, Beidou_Dnav_Ephemeris>& eph_map) const;

    /*
     * Computes the BDS Time and returns a boost::posix_time::ptime object
     *  \details Function used to convert the observation time into BDT time which is used
     *  as the default time for RINEX files
     *  \param eph BeiDou DNAV Ephemeris object
     *  \param obs_time Observation time in BDT seconds of week
     */
    boost::posix_time::ptime compute_BDS_time(const Beidou_Dnav_Ephemeris& eph, double obs_time) const;

    /*
     * Computes the UTC time and returns a boost::posix_time::ptime object
     */
    boost::posix_time::ptime compute_UTC_time(const Gps_Navigation_Message& nav_msg) const;

    /*
     * Computes the GPS time and returns a boost::posix_time::ptime object
     */
    boost::posix_time::ptime compute_GPS_time(const Gps_Ephemeris& eph, double obs_time) const;

    /*
     * Computes the GPS time and returns a boost::posix_time::ptime object
     */
    boost::posix_time::ptime compute_GPS_time(const Gps_CNAV_Ephemeris& eph, double obs_time) const;

    /*
     * Computes the Galileo time and returns a boost::posix_time::ptime object
     */
    boost::posix_time::ptime compute_Galileo_time(const Galileo_Ephemeris& eph, double obs_time) const;

    /*
     * Computes the UTC Time and returns a boost::posix_time::ptime object
     *  \details Function used as a method to convert the observation time into UTC time which is used
     *  as the default time for RINEX files
     *  \param eph GLONASS GNAV Ephemeris object
     *  \param obs_time Observation time in GPS seconds of week
     */
    boost::posix_time::ptime compute_UTC_time(const Glonass_Gnav_Ephemeris& eph, double obs_time) const;

    /*
     * Computes number of leap seconds of GPS relative to UTC
     *  \param eph GLONASS GNAV Ephemeris object
     *  \param gps_obs_time Observation time in GPS seconds of week
     */
    double get_leap_second(const Glonass_Gnav_Ephemeris& eph, double gps_obs_time) const;

    /*
     * Represents GPS time in the date time format. Leap years are considered, but leap seconds are not.
     */
    void to_date_time(int gps_week,
        int gps_tow,
        int& year,
        int& month,
        int& day,
        int& hour,
        int& minute,
        int& second) const;

    /*
     * Writes raw SBAS messages into the RINEX file
     */
    // void log_rinex_sbs(std::fstream & out, const Sbas_Raw_Msg & sbs_message);

    void update_nav_header(std::fstream& out,
        const Gps_Utc_Model& utc_model,
        const Gps_Iono& gps_iono, const Gps_Ephemeris& eph) const;

    void update_nav_header(std::fstream& out,
        const Gps_CNAV_Utc_Model& utc_model,
        const Gps_CNAV_Iono& iono) const;

    void update_nav_header(std::fstream& out,
        const Gps_Iono& gps_iono,
        const Gps_Utc_Model& gps_utc_model,
        const Gps_Ephemeris& eph,
        const Galileo_Iono& galileo_iono,
        const Galileo_Utc_Model& galileo_utc_model) const;

    void update_nav_header(std::fstream& out,
        const Gps_CNAV_Utc_Model& utc_model,
        const Gps_CNAV_Iono& iono,
        const Galileo_Iono& galileo_iono,
        const Galileo_Utc_Model& galileo_utc_model) const;

    void update_nav_header(std::fstream& out,
        const Galileo_Iono& galileo_iono,
        const Galileo_Utc_Model& utc_model) const;

    void update_nav_header(std::fstream& out,
        const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model,
        const Glonass_Gnav_Almanac& glonass_gnav_almanac) const;

    void update_nav_header(std::fstream& out,
        const Gps_Iono& gps_iono,
        const Gps_Utc_Model& gps_utc,
        const Gps_Ephemeris& eph,
        const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model,
        const Glonass_Gnav_Almanac& glonass_gnav_almanac) const;

    void update_nav_header(std::fstream& out,
        const Gps_CNAV_Iono& gps_iono,
        const Gps_CNAV_Utc_Model& gps_utc_model,
        const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model,
        const Glonass_Gnav_Almanac& glonass_gnav_almanac) const;

    void update_nav_header(std::fstream& out,
        const Galileo_Iono& galileo_iono,
        const Galileo_Utc_Model& galileo_utc_model,
        const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model,
        const Glonass_Gnav_Almanac& glonass_gnav_almanac) const;

    void update_nav_header(std::fstream& out,
        const Beidou_Dnav_Utc_Model& utc_model,
        const Beidou_Dnav_Iono& beidou_dnav_iono) const;

    void update_obs_header(std::fstream& out,
        const Gps_Utc_Model& utc_model) const;

    void update_obs_header(std::fstream& out,
        const Gps_CNAV_Utc_Model& utc_model) const;

    void update_obs_header(std::fstream& out,
        const Galileo_Utc_Model& galileo_utc_model) const;

    void update_obs_header(std::fstream& out,
        const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model) const;

    void update_obs_header(std::fstream& out,
        const Beidou_Dnav_Utc_Model& utc_model) const;

    /*
     * Generation of RINEX signal strength indicators
     */
    int signalStrength(double snr) const;

    /* Creates RINEX file names according to the naming convention
     *
     * See ftp://igs.org/pub/data/format/rinex301.pdf
     * Section 4, page 6
     *
     * \param[in] type of RINEX file. Can be:
     * "RINEX_FILE_TYPE_OBS" - Observation file.
     * "RINEX_FILE_TYPE_GPS_NAV" - GPS navigation message file.
     * "RINEX_FILE_TYPE_MET" - Meteorological data file.
     * "RINEX_FILE_TYPE_GLO_NAV" - GLONASS navigation file.
     * "RINEX_FILE_TYPE_GAL_NAV"  - Galileo navigation message file.
     * "RINEX_FILE_TYPE_MIXED_NAV" - Mixed GNSS navigation message file.
     * "RINEX_FILE_TYPE_GEO_NAV" - SBAS Payload navigation message file.
     * "RINEX_FILE_TYPE_SBAS" - SBAS broadcast data file.
     * "RINEX_FILE_TYPE_CLK" - Clock file.
     */
    std::string createFilename(const std::string& type, const std::string& base_name) const;

    /*
     * Generates the data for the PGM / RUN BY / DATE line
     */
    std::string getLocalTime() const;

    /*
     *  Checks that the line is 80 characters length
     */
    void lengthCheck(const std::string& line) const;

    /*
     * If the string is bigger than length, truncate it from the right.
     * otherwise, add pad characters to its right.
     *
     * Left-justifies the input in a string of the specified
     * length. If the new length (\a length) is larger than the
     * current length, the string is extended by the pad
     * character (\a pad). The default pad character is a
     * blank.
     * \param[in] s string to be modified.
     * \param[in] length new desired length of string.
     * \param[in] pad character to pad string with (blank by default).
     * \return a reference to \a s.  */
    inline std::string& leftJustify(std::string& s,
        std::string::size_type length,
        char pad = ' ') const;

    /*
     * If the string is bigger than length, truncate it from the right.
     * otherwise, add pad characters to its right.
     *
     * Left-justifies the receiver in a string of the specified
     * length (const version). If the new length (\a length) is larger
     * than the current length, the string is extended by the pad
     * character (\a pad). The default pad character is a
     * blank.
     * \param[in] s string to be modified.
     * \param[in] length new desired length of string.
     * \param[in] pad character to pad string with (blank by default).
     * \return a reference to \a s.  */
    inline std::string leftJustify(const std::string& s,
        std::string::size_type length,
        char pad = ' ') const
    {
        std::string t(s);
        return leftJustify(t, length, pad);
    }


    /*
     * Right-justifies the receiver in a string of the specified
     * length. If the receiver's data is shorter than the
     * requested length (\a length), it is padded on the left with
     * the pad character (\a pad). The default pad
     * character is a blank. */
    inline std::string& rightJustify(std::string& s,
        std::string::size_type length,
        char pad = ' ') const;

    /*
     * Right-justifies the receiver in a string of the specified
     * length (const version). If the receiver's data is shorter than the
     * requested length (\a length), it is padded on the left with
     * the pad character (\a pad). The default pad
     * character is a blank.*/
    inline std::string rightJustify(const std::string& s,
        std::string::size_type length,
        char pad = ' ') const
    {
        std::string t(s);
        return rightJustify(t, length, pad);
    }


    /*
     * Convert a double to a scientific notation number.
     * @param d the double to convert
     * @param length length (in characters) of output, including exponent
     * @param expLen length (in characters) of the exponent, with sign
     * @param showSign if true, reserves 1 character for +/- sign
     * @param checkSwitch if true, keeps the exponential sanity check for
     * exponentials above three characters in length.  If false, it removes
     * that check.
     */
    inline std::string doub2sci(double d,
        std::string::size_type length,
        std::string::size_type expLen,
        bool showSign = true,
        bool checkSwitch = true) const;


    /*
     * Convert scientific notation to FORTRAN notation.
     * As an example, the string "1.5636E5" becomes " .15636D6".
     * Note that the first character of the string will be '-' if
     * the number is negative or ' ' if the first character is positive.
     * @param aStr string with number to convert
     * @param startPos start position of number in string
     * @param length length (in characters) of number, including exponent.
     * @param expLen length (in characters of exponent, not including sign.
     * @param checkSwitch will keep the method running as originally programmed
     * when set to true.  If false, the method will always resize exponentials,
     * produce an exponential with an E instead of a D, and always have a leading
     * zero.  For example -> 0.87654E-0004 or -0.1234E00005.
     */
    inline std::string& sci2for(std::string& aStr,
        std::string::size_type startPos = 0,
        std::string::size_type length = std::string::npos,
        std::string::size_type expLen = 3,
        bool checkSwitch = true) const;


    /*
     * Convert double precision floating point to a string
     * containing the number in FORTRAN notation.
     * As an example, the number 156360 becomes ".15636D6".
     * @param d number to convert.
     * @param length length (in characters) of number, including exponent.
     * @param expLen length (in characters of exponent, including sign.
     * @param checkSwitch if true, keeps the exponential sanity check for
     * exponentials above three characters in length.  If false, it removes
     * that check.
     * @return a string containing \a d in FORTRAN notation.
     */
    inline std::string doub2for(double d,
        std::string::size_type length,
        std::string::size_type expLen,
        bool checkSwitch = true) const;


    /*
     * Convert a string to a double precision floating point number.
     * @param s string containing a number.
     * @return double representation of string.
     */
    inline double asDouble(const std::string& s) const
    {
        return strtod(s.c_str(), nullptr);
    }


    inline int toInt(const std::string& bitString, int sLength) const;

    /*
     * Convert a string to an integer.
     * @param s string containing a number.
     * @return int64_t  integer representation of string.
     */
    inline int64_t asInt(const std::string& s) const
    {
        return strtol(s.c_str(), nullptr, 10);
    }


    /*
     * Convert a double to a string in fixed notation.
     * @param x double.
     * @param precision the number of decimal places you want displayed.
     * @return string representation of \a x.
     */
    inline std::string asString(double x,
        std::string::size_type precision = 17) const;


    /*
     * Convert a long double to a string in fixed notation.
     * @param x long double.
     * @param precision the number of decimal places you want displayed.
     * @return string representation of \a x.
     */
    inline std::string asString(long double x,
        std::string::size_type precision = 21) const;


    /*
     * Convert any old object to a string.
     * The class must have stream operators defined.
     * @param x object to turn into a string.
     * @return string representation of \a x.
     */
    template <class X>
    inline std::string asString(const X x) const;

    inline std::string asFixWidthString(int x, int width, char fill_digit) const;

    std::map<std::string, std::string> observationType;  // PSEUDORANGE, CARRIER_PHASE, DOPPLER, SIGNAL_STRENGTH
    std::map<std::string, std::string> observationCode;  // GNSS observation descriptors

    std::fstream obsFile;     // Output file stream for RINEX observation file
    std::fstream navFile;     // Output file stream for RINEX navigation data file
    std::fstream sbsFile;     // Output file stream for RINEX SBAS raw data file
    std::fstream navGalFile;  // Output file stream for RINEX Galileo navigation data file
    std::fstream navGloFile;  // Output file stream for RINEX GLONASS navigation data file
    std::fstream navBdsFile;  // Output file stream for RINEX Galileo navigation data file
    std::fstream navMixFile;  // Output file stream for RINEX Mixed navigation data file

    std::string navfilename;                      // Name of RINEX navigation file for GPS L1
    std::string obsfilename;                      // Name of RINEX observation file
    std::string sbsfilename;                      // Name of RINEX SBAS file
    std::string navGalfilename;                   // Name of RINEX navigation file for Galileo
    std::string navGlofilename;                   // Name of RINEX navigation file for Glonass
    std::string navBdsfilename;                   // Name of RINEX navigation file for BeiDou
    std::string navMixfilename;                   // Name of RINEX navigation file for fixed signals
    std::vector<std::string> output_navfilename;  // Name of output RINEX navigation file(s)

    std::string d_stringVersion;  // RINEX version (2.10/2.11 or 3.01/3.02)

    double d_fake_cnav_iode;
    int d_version;                  // RINEX version (2 for 2.10/2.11 and 3 for 3.01)
    int d_numberTypesObservations;  // Number of available types of observable in the system. Should be public?
    bool d_rinex_header_updated;
    bool d_rinex_header_written;
    bool d_pre_2009_file;
};


// Implementation of inline functions (modified versions from GNSSTk https://github.com/SGL-UT/gnsstk)

inline std::string& Rinex_Printer::leftJustify(std::string& s,
    std::string::size_type length,
    char pad) const
{
    if (length < s.length())
        {
            s = s.substr(0, length);
        }
    else
        {
            s.append(length - s.length(), pad);
        }
    return s;
}


// if the string is bigger than length, truncate it from the left.
// otherwise, add pad characters to its left.
inline std::string& Rinex_Printer::rightJustify(std::string& s,
    std::string::size_type length,
    char pad) const
{
    if (length < s.length())
        {
            s = s.substr(s.length() - length, std::string::npos);
        }
    else
        {
            s.insert(static_cast<std::string::size_type>(0), length - s.length(), pad);
        }
    return s;
}


inline std::string Rinex_Printer::doub2for(double d,
    std::string::size_type length,
    std::string::size_type expLen,
    bool checkSwitch) const
{
    int16_t exponentLength = expLen;

    /* Validate the assumptions regarding the input arguments */
    if (exponentLength < 0)
        {
            exponentLength = 1;
        }

    if (exponentLength > 3 && checkSwitch)
        {
            exponentLength = 3;
        }

    std::string toReturn = doub2sci(d, length, exponentLength, true, checkSwitch);
    sci2for(toReturn, 0, length, exponentLength, checkSwitch);

    return toReturn;
}


inline std::string Rinex_Printer::doub2sci(double d,
    std::string::size_type length,
    std::string::size_type expLen,
    bool showSign,
    bool checkSwitch) const
{
    std::string toReturn;
    int16_t exponentLength = expLen;

    /* Validate the assumptions regarding the input arguments */
    if (exponentLength < 0)
        {
            exponentLength = 1;
        }

    if (exponentLength > 3 && checkSwitch)
        {
            exponentLength = 3;
        }

    std::stringstream c;
    c.setf(std::ios::scientific, std::ios::floatfield);

    // length - 3 for special characters ('.', 'e', '+' or '-')
    // - exponentlength (e04)
    // - 1 for the digit before the decimal (2.)
    // and if showSign == true,
    //    an extra -1 for '-' or ' ' if it's positive or negative
    int expSize = 0;
    if (showSign)
        {
            expSize = 1;
        }

    c.precision(length - 3 - exponentLength - 1 - expSize);
    c << d;
    c >> toReturn;
    return toReturn;
}


inline std::string& Rinex_Printer::sci2for(std::string& aStr,
    std::string::size_type startPos,
    std::string::size_type length,
    std::string::size_type expLen,
    bool checkSwitch) const
{
    std::string::size_type idx = aStr.find('.', startPos);
    int expAdd = 0;
    std::string exp;
    int64_t iexp;
    // If checkSwitch is false, always redo the exponential. Otherwise,
    // set it to false.
    bool redoexp = !checkSwitch;

    // Check for decimal place within specified boundaries
    if ((idx <= 0) || (idx >= (startPos + length - expLen - 1)))
        {
            // Error: no decimal point in string
            return aStr;
        }

    // Here, account for the possibility that there are
    // no numbers to the left of the decimal, but do not
    // account for the possibility of non-scientific
    // notation (more than one digit to the left of the
    // decimal)
    if (idx > startPos)
        {
            redoexp = true;
            // Swap digit and decimal.
            aStr[idx] = aStr[idx - 1];
            aStr[idx - 1] = '.';
            // Only add one to the exponent if the number is non-zero
            if (asDouble(aStr.substr(startPos, length)) != 0.0)
                {
                    expAdd = 1;
                }
        }

    idx = aStr.find('e', startPos);
    if (idx == std::string::npos)
        {
            idx = aStr.find('E', startPos);
            if (idx == std::string::npos)
                {
                    // Error: no 'e' or 'E' in string";
                }
        }

    // Change the exponent character to D normally, or E of checkSwitch is false.
    if (checkSwitch)
        {
            aStr[idx] = 'D';
        }
    else
        {
            aStr[idx] = 'E';
        }

    // Change the exponent itself
    if (redoexp)
        {
            exp = aStr.substr(idx + 1, std::string::npos);
            iexp = asInt(exp);
            iexp += expAdd;

            aStr.erase(idx + 1);
            if (iexp < 0)
                {
                    aStr += "-";
                    iexp -= iexp * 2;
                }
            else
                {
                    aStr += "+";
                }

            aStr += Rinex_Printer::rightJustify(asString(iexp), expLen, '0');
        }

    // if the number is positive, append a space
    // (if it's negative, there's a leading '-'
    if (aStr[0] == '.')
        {
            aStr.insert(static_cast<std::string::size_type>(0), 1, ' ');
        }

    // If checkSwitch is false, add on one leading zero to the string
    if (!checkSwitch)
        {
            aStr.insert(static_cast<std::string::size_type>(1), 1, '0');
        }

    return aStr;
}  // end sci2for


inline std::string asString(long double x, std::string::size_type precision)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << x;
    return ss.str();
}


inline std::string Rinex_Printer::asString(double x, std::string::size_type precision) const
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << x;
    return ss.str();
}


inline std::string Rinex_Printer::asFixWidthString(int x, int width, char fill_digit) const
{
    std::ostringstream ss;
    ss << std::setfill(fill_digit) << std::setw(width) << x;
    return ss.str().substr(ss.str().size() - width);
}


inline int64_t asInt(const std::string& s)
{
    return strtol(s.c_str(), nullptr, 10);
}


inline int Rinex_Printer::toInt(const std::string& bitString, int sLength) const
{
    int tempInt;
    int num = 0;
    for (int i = 0; i < sLength; i++)
        {
            tempInt = bitString[i] - '0';
            num |= (1 << (sLength - 1 - i)) * tempInt;
        }
    return num;
}


template <class X>
inline std::string Rinex_Printer::asString(const X x) const
{
    std::ostringstream ss;
    ss << x;
    return ss.str();
}


/** \} */
/** \} */
#endif  // GNSS_SDR_RINEX_PRINTER_H
