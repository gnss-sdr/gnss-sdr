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
 * here: https://igs.org/formats-and-standards/
 * \author Carles Fernandez Prades, 2011-2026. cfernandez(at)cttc.es
 * \author Mathieu Favreau, 2025-2026. favreau.mathieu(at)hotmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_RINEX_PRINTER_H
#define GNSS_SDR_RINEX_PRINTER_H

#include "signal_enabled_flags.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <cstdint>  // for int32_t
#include <cstdlib>  // for strtol, strtod
#include <fstream>  // for fstream
#include <map>      // for map
#include <string>   // for string
#include <vector>   // for vector


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
    explicit Rinex_Printer(uint32_t signal_enabled_flags,
        int version = 3,
        const std::string& base_path = ".",
        const std::string& base_name = "-",
        bool pre_2009_file = false);

    /*!
     * \brief Destructor. Removes created files if empty.
     */
    ~Rinex_Printer();

    /*!
     * \brief Print RINEX annotation. If it is the first annotation, it also
     * prints the RINEX headers for navigation and observation files. If it is
     * not the first annotation, it only annotates the observation, and updates
     * the navigation header if UTC data was not available when writing it for
     * the first time.
     *
     */
    void print_rinex_annotation(const Rtklib_Solver* pvt_solver,
        const std::map<int, Gnss_Synchro>& gnss_observables_map,
        double rx_time,
        bool flag_write_RINEX_obs_output);

    /*!
     * \brief Print RINEX annotation for GPS NAV message
     */
    void log_rinex_nav_gps_nav(const std::map<int32_t, Gps_Ephemeris>& new_eph);

    /*!
     * \brief Print RINEX annotation for GPS CNAV message
     */
    void log_rinex_nav_gps_cnav(const std::map<int32_t, Gps_CNAV_Ephemeris>& new_cnav_eph);

    /*!
     * \brief Print RINEX annotation for Galileo NAV message
     */
    void log_rinex_nav_gal_nav(const std::map<int32_t, Galileo_Ephemeris>& new_gal_eph);

    /*!
     * \brief Print RINEX annotation for Glonass GNAV message
     */
    void log_rinex_nav_glo_gnav(const std::map<int32_t, Glonass_Gnav_Ephemeris>& new_glo_eph);

    /*!
     * \brief Print RINEX annotation for BeiDou DNAV message
     */
    void log_rinex_nav_bds_dnav(const std::map<int32_t, Beidou_Dnav_Ephemeris>& new_bds_eph);

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
    // Not the best, but reorder params to select the correct constructor
    explicit Rinex_Printer(uint32_t signal_enabled_flags,
        const std::string& base_name,
        const std::string& base_rinex_path,
        int version,
        bool pre_2009_file);

    /*
     * Generates the GPS Observation data header
     */
    void rinex_obs_header(std::fstream& out,
        const std::string& time_constellation,
        const boost::posix_time::ptime& system_time,
        double seconds);

    /*
     * Generates the Navigation Data header
     */
    void rinex_nav_header(std::fstream& out,
        const std::vector<std::string>& iono_lines,
        const std::vector<std::string>& time_corr_lines,
        const std::string& leap_second_line) const;

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
        const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model) const;

    /*
     * Generates the Mixed (GPS L1 C/A/GLONASS L1, L2) Navigation Data header
     */
    void rinex_nav_header(std::fstream& out,
        const Gps_Iono& gps_iono,
        const Gps_Utc_Model& gps_utc_model,
        const Gps_Ephemeris& eph,
        const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model);

    /*
     * Generates the Mixed (GPS L2C C/A/GLONASS L1, L2) Navigation Data header
     */
    void rinex_nav_header(std::fstream& out,
        const Gps_CNAV_Iono& gps_iono,
        const Gps_CNAV_Utc_Model& gps_utc_model,
        const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model);

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
     * Writes data from the GLONASS GNAV navigation message into the RINEX file
     */
    void log_rinex_nav(std::fstream& out,
        const std::map<int32_t, Glonass_Gnav_Ephemeris>& eph_map) const;

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
        const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model) const;

    void update_nav_header(std::fstream& out,
        const Gps_Iono& gps_iono,
        const Gps_Utc_Model& gps_utc,
        const Gps_Ephemeris& eph,
        const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model) const;

    void update_nav_header(std::fstream& out,
        const Gps_CNAV_Iono& gps_iono,
        const Gps_CNAV_Utc_Model& gps_utc_model,
        const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model) const;

    void update_nav_header(std::fstream& out,
        const Galileo_Iono& galileo_iono,
        const Galileo_Utc_Model& galileo_utc_model,
        const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model) const;

    void update_nav_header(std::fstream& out,
        const Beidou_Dnav_Utc_Model& utc_model,
        const Beidou_Dnav_Iono& beidou_dnav_iono) const;

    void update_obs_header(std::fstream& out, const std::string& leap_second_line) const;

    const std::map<std::string, std::string> observationType;  // PSEUDORANGE, CARRIER_PHASE, DOPPLER, SIGNAL_STRENGTH
    const std::map<std::string, std::string> observationCode;  // GNSS observation descriptors

    const Signal_Enabled_Flags d_flags;

    const int d_version;                // RINEX version (2 for 2.10/2.11 and 3 for 3.01)
    const std::string d_stringVersion;  // RINEX version (2.10/2.11 or 3.01/3.02)

    double d_fake_cnav_iode;
    bool d_rinex_header_updated;
    bool d_rinex_header_written;
    const bool d_pre_2009_file;

    const std::string navfilename;                // Name of RINEX navigation file
    const std::string obsfilename;                // Name of RINEX observation file
    const std::string navGlofilename;             // Name of RINEX navigation file for Glonass
    std::vector<std::string> output_navfilename;  // Name of output RINEX navigation file(s)

    std::fstream obsFile;     // Output file stream for RINEX observation file
    std::fstream navFile;     // Output file stream for RINEX navigation data file
    std::fstream navGloFile;  // Output file stream for RINEX GLONASS navigation data file
};


/** \} */
/** \} */
#endif  // GNSS_SDR_RINEX_PRINTER_H
