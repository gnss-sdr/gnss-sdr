/*!
 * \file rinex_printer.h
 * \brief Interface of a RINEX 2.11 / 3.01 printer
 * See http://igscb.jpl.nasa.gov/igscb/data/format/rinex301.pdf
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
 * here: http://igscb.jpl.nasa.gov/components/formats.html
 * \author Carles Fernandez Prades, 2011. cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_RINEX_PRINTER_H_
#define GNSS_SDR_RINEX_PRINTER_H_

#include "gps_navigation_message.h"
#include "gps_cnav_navigation_message.h"
#include "galileo_navigation_message.h"
#include "glonass_gnav_navigation_message.h"
#include "GPS_L1_CA.h"
#include "Galileo_E1.h"
#include "GLONASS_L1_L2_CA.h"
#include "gnss_synchro.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <cstdint>
#include <string>
#include <fstream>
#include <sstream>  // for stringstream
#include <iomanip>  // for setprecision
#include <map>

class Sbas_Raw_Msg;

/*!
 * \brief Class that handles the generation of Receiver
 * INdependent EXchange format (RINEX) files
 */
class Rinex_Printer
{
public:
    /*!
     * \brief Default constructor. Creates GPS Navigation and Observables RINEX files and their headers
     */
    Rinex_Printer(int version = 0);

    /*!
     * \brief Default destructor. Closes GPS Navigation and Observables RINEX files
     */
    ~Rinex_Printer();

    std::fstream obsFile;     //<! Output file stream for RINEX observation file
    std::fstream navFile;     //<! Output file stream for RINEX navigation data file
    std::fstream sbsFile;     //<! Output file stream for RINEX SBAS raw data file
    std::fstream navGalFile;  //<! Output file stream for RINEX Galileo navigation data file
    std::fstream navGloFile;  //<! Output file stream for RINEX GLONASS navigation data file
    std::fstream navMixFile;  //<! Output file stream for RINEX Mixed navigation data file

    /*!
     *  \brief Generates the GPS L1 C/A Navigation Data header
     */
    void rinex_nav_header(std::fstream& out, const Gps_Iono& iono, const Gps_Utc_Model& utc_model);

    /*!
     *  \brief Generates the GPS L2C(M) Navigation Data header
     */
    void rinex_nav_header(std::fstream& out, const Gps_CNAV_Iono& iono, const Gps_CNAV_Utc_Model& utc_model);

    /*!
     *  \brief Generates the Galileo Navigation Data header
     */
    void rinex_nav_header(std::fstream& out, const Galileo_Iono& iono, const Galileo_Utc_Model& utc_model);

    /*!
     *  \brief Generates the Mixed (GPS/Galileo) Navigation Data header
     */
    void rinex_nav_header(std::fstream& out, const Gps_Iono& gps_iono, const Gps_Utc_Model& gps_utc_model, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model);

    /*!
     *  \brief Generates the GLONASS L1, L2 C/A Navigation Data header
     */
    void rinex_nav_header(std::fstream& out, const Glonass_Gnav_Utc_Model& utc_model, const Glonass_Gnav_Ephemeris& glonass_gnav_eph);

    /*!
     *  \brief Generates the Mixed (Galileo/GLONASS) Navigation Data header
     */
    void rinex_nav_header(std::fstream& out, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac);

    /*!
     *  \brief Generates the Mixed (GPS L1 C/A/GLONASS L1, L2) Navigation Data header
     */
    void rinex_nav_header(std::fstream& out, const Gps_Iono& gps_iono, const Gps_Utc_Model& gps_utc_model, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac);

    /*!
    *  \brief Generates the Mixed (GPS L2C C/A/GLONASS L1, L2) Navigation Data header
    */
    void rinex_nav_header(std::fstream& out, const Gps_CNAV_Iono& gps_iono, const Gps_CNAV_Utc_Model& gps_utc_model, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac);

    /*!
     *  \brief Generates the GPS Observation data header
     */
    void rinex_obs_header(std::fstream& out, const Gps_Ephemeris& eph, const double d_TOW_first_observation);

    /*!
     *  \brief Generates the GPS L2 Observation data header
     */
    void rinex_obs_header(std::fstream& out, const Gps_CNAV_Ephemeris& eph, const double d_TOW_first_observation);

    /*!
     *  \brief Generates the dual frequency GPS L1 & L2 Observation data header
     */
    void rinex_obs_header(std::fstream& out, const Gps_Ephemeris& eph, const Gps_CNAV_Ephemeris& eph_cnav, const double d_TOW_first_observation);

    /*!
     *  \brief Generates the Galileo Observation data header. Example: bands("1B"), bands("1B 5X"), bands("5X"), ... Default: "1B".
     */
    void rinex_obs_header(std::fstream& out, const Galileo_Ephemeris& eph, const double d_TOW_first_observation, const std::string bands = "1B");

    /*!
     *  \brief Generates the Mixed (GPS/Galileo) Observation data header. Example: galileo_bands("1B"), galileo_bands("1B 5X"), galileo_bands("5X"), ... Default: "1B".
     */
    void rinex_obs_header(std::fstream& out, const Gps_Ephemeris& gps_eph, const Galileo_Ephemeris& galileo_eph, const double d_TOW_first_observation, const std::string galileo_bands = "1B");

    /*!
     *  \brief Generates the GLONASS GNAV Observation data header. Example: bands("1C"), bands("1C 2C"), bands("2C"), ... Default: "1C".
     */
    void rinex_obs_header(std::fstream& out, const Glonass_Gnav_Ephemeris& eph, const double d_TOW_first_observation, const std::string bands = "1G");

    /*!
     *  \brief Generates the Mixed (GPS L1 C/A /GLONASS) Observation data header. Example: galileo_bands("1C"), galileo_bands("1B 5X"), galileo_bands("5X"), ... Default: "1B".
     */
    void rinex_obs_header(std::fstream& out, const Gps_Ephemeris& gps_eph, const Glonass_Gnav_Ephemeris& glonass_gnav_eph, const double d_TOW_first_observation, const std::string glo_bands = "1C");

    /*!
     *  \brief Generates the Mixed (Galileo/GLONASS) Observation data header. Example: galileo_bands("1C"), galileo_bands("1B 5X"), galileo_bands("5X"), ... Default: "1B".
     */
    void rinex_obs_header(std::fstream& out, const Galileo_Ephemeris& galileo_eph, const Glonass_Gnav_Ephemeris& glonass_gnav_eph, const double d_TOW_first_observation, const std::string galileo_bands = "1B", const std::string glo_bands = "1C");

    /*!
     *  \brief Generates the Mixed (GPS L2C/GLONASS) Observation data header. Example: galileo_bands("1G")... Default: "1G".
     */
    void rinex_obs_header(std::fstream& out, const Gps_CNAV_Ephemeris& gps_cnav_eph, const Glonass_Gnav_Ephemeris& glonass_gnav_eph, const double d_TOW_first_observation, const std::string glo_bands = "1G");

    /*!
     *  \brief Generates the SBAS raw data header
     */
    void rinex_sbs_header(std::fstream& out);

    /*!
     *  \brief Computes the UTC time and returns a boost::posix_time::ptime object
     */
    boost::posix_time::ptime compute_UTC_time(const Gps_Navigation_Message& nav_msg);

    /*!
     *  \brief Computes the GPS time and returns a boost::posix_time::ptime object
     */
    boost::posix_time::ptime compute_GPS_time(const Gps_Ephemeris& eph, const double obs_time);

    /*!
     *  \brief Computes the GPS time and returns a boost::posix_time::ptime object
     */
    boost::posix_time::ptime compute_GPS_time(const Gps_CNAV_Ephemeris& eph, const double obs_time);

    /*!
     *  \brief Computes the Galileo time and returns a boost::posix_time::ptime object
     */
    boost::posix_time::ptime compute_Galileo_time(const Galileo_Ephemeris& eph, const double obs_time);

    /*!
     *  \brief Computes the UTC Time and returns a boost::posix_time::ptime object
     *  \details Function used as a method to convert the observation time into UTC time which is used
     *  as the default time for RINEX files
     *  \param eph GLONASS GNAV Ephemeris object
     *  \param obs_time Observation time in GPS seconds of week
     */
    boost::posix_time::ptime compute_UTC_time(const Glonass_Gnav_Ephemeris& eph, const double obs_time);

    /*!
     *  \brief Computes number of leap seconds of GPS relative to UTC
     *  \param eph GLONASS GNAV Ephemeris object
     *  \param gps_obs_time Observation time in GPS seconds of week
     */
    double get_leap_second(const Glonass_Gnav_Ephemeris& eph, const double gps_obs_time);

    /*!
     *  \brief Writes data from the GPS L1 C/A navigation message into the RINEX file
     */
    void log_rinex_nav(std::fstream& out, const std::map<int32_t, Gps_Ephemeris>& eph_map);

    /*!
     *  \brief Writes data from the GPS L2 navigation message into the RINEX file
     */
    void log_rinex_nav(std::fstream& out, const std::map<int32_t, Gps_CNAV_Ephemeris>& eph_map);

    /*!
     *  \brief Writes data from the Galileo navigation message into the RINEX file
     */
    void log_rinex_nav(std::fstream& out, const std::map<int32_t, Galileo_Ephemeris>& eph_map);

    /*!
     *  \brief Writes data from the Mixed (GPS/Galileo) navigation message into the RINEX file
     */
    void log_rinex_nav(std::fstream& out, const std::map<int32_t, Gps_Ephemeris>& gps_eph_map, const std::map<int32_t, Galileo_Ephemeris>& galileo_eph_map);

    /*!
     *  \brief Writes data from the GLONASS GNAV navigation message into the RINEX file
     */
    void log_rinex_nav(std::fstream& out, const std::map<int32_t, Glonass_Gnav_Ephemeris>& eph_map);

    /*!
     *  \brief Writes data from the Mixed (GPS/GLONASS GNAV) navigation message into the RINEX file
     */
    void log_rinex_nav(std::fstream& out, const std::map<int32_t, Gps_Ephemeris>& gps_eph_map, const std::map<int32_t, Glonass_Gnav_Ephemeris>& glonass_gnav_eph_map);

    /*!
     *  \brief Writes data from the Mixed (GPS/GLONASS GNAV) navigation message into the RINEX file
     */
    void log_rinex_nav(std::fstream& out, const std::map<int32_t, Gps_CNAV_Ephemeris>& gps_cnav_eph_map, const std::map<int32_t, Glonass_Gnav_Ephemeris>& glonass_gnav_eph_map);

    /*!
     *  \brief Writes data from the Mixed (Galileo/ GLONASS GNAV) navigation message into the RINEX file
     */
    void log_rinex_nav(std::fstream& out, const std::map<int32_t, Galileo_Ephemeris>& galileo_eph_map, const std::map<int32_t, Glonass_Gnav_Ephemeris>& glonass_gnav_eph_map);

    /*!
     *  \brief Writes GPS L1 observables into the RINEX file
     */
    void log_rinex_obs(std::fstream& out, const Gps_Ephemeris& eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables);

    /*!
     *  \brief Writes GPS L2 observables into the RINEX file
     */
    void log_rinex_obs(std::fstream& out, const Gps_CNAV_Ephemeris& eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables);

    /*!
     *  \brief Writes dual frequency GPS L1 and L2 observables into the RINEX file
     */
    void log_rinex_obs(std::fstream& out, const Gps_Ephemeris& eph, const Gps_CNAV_Ephemeris& eph_cnav, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables);

    /*!
     *  \brief Writes Galileo observables into the RINEX file. Example: galileo_bands("1B"), galileo_bands("1B 5X"), galileo_bands("5X"), ... Default: "1B".
     */
    void log_rinex_obs(std::fstream& out, const Galileo_Ephemeris& eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables, const std::string galileo_bands = "1B");

    /*!
     *  \brief Writes Mixed GPS / Galileo observables into the RINEX file
     */
    void log_rinex_obs(std::fstream& out, const Gps_Ephemeris& gps_eph, const Galileo_Ephemeris& galileo_eph, const double gps_obs_time, const std::map<int32_t, Gnss_Synchro>& observables);

    /*!
     *  \brief Writes GLONASS GNAV observables into the RINEX file. Example: glonass_bands("1C"), galileo_bands("1B 5X"), galileo_bands("5X"), ... Default: "1B".
     */
    void log_rinex_obs(std::fstream& out, const Glonass_Gnav_Ephemeris& eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables, const std::string glonass_bands = "1C");

    /*!
     *  \brief Writes Mixed GPS L1 C/A - GLONASS observables into the RINEX file
     */
    void log_rinex_obs(std::fstream& out, const Gps_Ephemeris& gps_eph, const Glonass_Gnav_Ephemeris& glonass_gnav_eph, const double gps_obs_time, const std::map<int32_t, Gnss_Synchro>& observables);

    /*!
     *  \brief Writes Mixed GPS L2C - GLONASS observables into the RINEX file
     */
    void log_rinex_obs(std::fstream& out, const Gps_CNAV_Ephemeris& gps_cnav_eph, const Glonass_Gnav_Ephemeris& glonass_gnav_eph, const double gps_obs_time, const std::map<int32_t, Gnss_Synchro>& observables);

    /*!
     *  \brief Writes Mixed Galileo/GLONASS observables into the RINEX file
     */
    void log_rinex_obs(std::fstream& out, const Galileo_Ephemeris& galileo_eph, const Glonass_Gnav_Ephemeris& glonass_gnav_eph, const double gps_obs_time, const std::map<int32_t, Gnss_Synchro>& observables);

    /*!
     * \brief Represents GPS time in the date time format. Leap years are considered, but leap seconds are not.
     */
    void to_date_time(int gps_week, int gps_tow, int& year, int& month, int& day, int& hour, int& minute, int& second);

    /*!
     *  \brief Writes raw SBAS messages into the RINEX file
     */
    //void log_rinex_sbs(std::fstream & out, const Sbas_Raw_Msg & sbs_message);

    void update_nav_header(std::fstream& out, const Gps_Utc_Model& gps_utc, const Gps_Iono& gps_iono);

    void update_nav_header(std::fstream& out, const Gps_CNAV_Utc_Model& utc_model, const Gps_CNAV_Iono& iono);

    void update_nav_header(std::fstream& out, const Gps_Iono& gps_iono, const Gps_Utc_Model& gps_utc_model, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model);

    void update_nav_header(std::fstream& out, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& utc_model);

    void update_nav_header(std::fstream& out, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac);

    void update_nav_header(std::fstream& out, const Gps_Iono& gps_iono, const Gps_Utc_Model& gps_utc, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac);

    void update_nav_header(std::fstream& out, const Gps_CNAV_Iono& gps_cnav_iono, const Gps_CNAV_Utc_Model& gps_cnav_utc, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac);

    void update_nav_header(std::fstream& out, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac);

    void update_obs_header(std::fstream& out, const Gps_Utc_Model& utc_model);

    void update_obs_header(std::fstream& out, const Gps_CNAV_Utc_Model& utc_model);

    void update_obs_header(std::fstream& out, const Galileo_Utc_Model& galileo_utc_model);

    void update_obs_header(std::fstream& out, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model);

    std::map<std::string, std::string> satelliteSystem;  //<! GPS, GLONASS, SBAS payload, Galileo or Compass
    std::map<std::string, std::string> observationType;  //<! PSEUDORANGE, CARRIER_PHASE, DOPPLER, SIGNAL_STRENGTH
    std::map<std::string, std::string> observationCode;  //<! GNSS observation descriptors
    std::string stringVersion;                           //<! RINEX version (2.10/2.11 or 3.01/3.02)

    std::string navfilename;
    std::string obsfilename;
    std::string sbsfilename;
    std::string navGalfilename;
    std::string navGlofilename;
    std::string navMixfilename;

private:
    int version;                  // RINEX version (2 for 2.10/2.11 and 3 for 3.01)
    int numberTypesObservations;  // Number of available types of observable in the system. Should be public?
    /*
     * Generation of RINEX signal strength indicators
     */
    int signalStrength(const double snr);

    /* Creates RINEX file names according to the naming convention
     *
     * See http://igscb.jpl.nasa.gov/igscb/data/format/rinex301.pdf
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
    std::string createFilename(std::string type);

    /*
     * Generates the data for the PGM / RUN BY / DATE line
     */
    std::string getLocalTime();

    /*
     *  Checks that the line is 80 characters length
     */
    void lengthCheck(const std::string& line);

    double fake_cnav_iode;

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
        const std::string::size_type length,
        const char pad = ' ');

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
        const std::string::size_type length,
        const char pad = ' ')
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
        const std::string::size_type length,
        const char pad = ' ');

    /*
     * Right-justifies the receiver in a string of the specified
     * length (const version). If the receiver's data is shorter than the
     * requested length (\a length), it is padded on the left with
     * the pad character (\a pad). The default pad
     * character is a blank.*/
    inline std::string rightJustify(const std::string& s,
        const std::string::size_type length,
        const char pad = ' ')
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
    inline std::string doub2sci(const double& d,
        const std::string::size_type length,
        const std::string::size_type expLen,
        const bool showSign = true,
        const bool checkSwitch = true);


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
        const std::string::size_type startPos = 0,
        const std::string::size_type length = std::string::npos,
        const std::string::size_type expLen = 3,
        const bool checkSwitch = true);


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
    inline std::string doub2for(const double& d,
        const std::string::size_type length,
        const std::string::size_type expLen,
        const bool checkSwitch = true);


    /*
     * Convert a string to a double precision floating point number.
     * @param s string containing a number.
     * @return double representation of string.
     */
    inline double asDouble(const std::string& s)
    {
        return strtod(s.c_str(), 0);
    }


    inline int toInt(std::string bitString, int sLength);

    /*
     * Convert a string to an integer.
     * @param s string containing a number.
     * @return int64_t  integer representation of string.
     */
    inline int64_t asInt(const std::string& s)
    {
        return strtol(s.c_str(), 0, 10);
    }


    /*
     * Convert a double to a string in fixed notation.
     * @param x double.
     * @param precision the number of decimal places you want displayed.
     * @return string representation of \a x.
     */
    inline std::string asString(const double x,
        const std::string::size_type precision = 17);


    /*
     * Convert a long double to a string in fixed notation.
     * @param x long double.
     * @param precision the number of decimal places you want displayed.
     * @return string representation of \a x.
     */
    inline std::string asString(const long double x,
        const std::string::size_type precision = 21);


    /*
     * Convert any old object to a string.
     * The class must have stream operators defined.
     * @param x object to turn into a string.
     * @return string representation of \a x.
     */
    template <class X>
    inline std::string asString(const X x);

    inline std::string asFixWidthString(const int x, const int width, char fill_digit);
};


// Implementation of inline functions (modified versions from GPSTk http://www.gpstk.org)

inline std::string& Rinex_Printer::leftJustify(std::string& s,
    const std::string::size_type length,
    const char pad)
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
    const std::string::size_type length,
    const char pad)
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


inline std::string Rinex_Printer::doub2for(const double& d,
    const std::string::size_type length,
    const std::string::size_type expLen,
    const bool checkSwitch)
{
    short exponentLength = expLen;

    /* Validate the assumptions regarding the input arguments */
    if (exponentLength < 0) exponentLength = 1;
    if (exponentLength > 3 && checkSwitch) exponentLength = 3;

    std::string toReturn = doub2sci(d, length, exponentLength, true, checkSwitch);
    sci2for(toReturn, 0, length, exponentLength, checkSwitch);

    return toReturn;
}


inline std::string Rinex_Printer::doub2sci(const double& d,
    const std::string::size_type length,
    const std::string::size_type expLen,
    const bool showSign,
    const bool checkSwitch)
{
    std::string toReturn;
    short exponentLength = expLen;

    /* Validate the assumptions regarding the input arguments */
    if (exponentLength < 0) exponentLength = 1;
    if (exponentLength > 3 && checkSwitch) exponentLength = 3;

    std::stringstream c;
    c.setf(std::ios::scientific, std::ios::floatfield);

    // length - 3 for special characters ('.', 'e', '+' or '-')
    // - exponentlength (e04)
    // - 1 for the digit before the decimal (2.)
    // and if showSign == true,
    //    an extra -1 for '-' or ' ' if it's positive or negative
    int expSize = 0;
    if (showSign)
        expSize = 1;
    c.precision(length - 3 - exponentLength - 1 - expSize);
    c << d;
    c >> toReturn;
    return toReturn;
}


inline std::string& Rinex_Printer::sci2for(std::string& aStr,
    const std::string::size_type startPos,
    const std::string::size_type length,
    const std::string::size_type expLen,
    const bool checkSwitch)
{
    std::string::size_type idx = aStr.find('.', startPos);
    int expAdd = 0;
    std::string exp;
    int64_t iexp;
    //If checkSwitch is false, always redo the exponential. Otherwise,
    //set it to false.
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
                expAdd = 1;
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
        aStr[idx] = 'D';
    else
        aStr[idx] = 'E';

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
                aStr += "+";
            aStr += Rinex_Printer::rightJustify(asString(iexp), expLen, '0');
        }

    // if the number is positive, append a space
    // (if it's negative, there's a leading '-'
    if (aStr[0] == '.')
        {
            aStr.insert(static_cast<std::string::size_type>(0), 1, ' ');
        }

    //If checkSwitch is false, add on one leading zero to the string
    if (!checkSwitch)
        {
            aStr.insert(static_cast<std::string::size_type>(1), 1, '0');
        }

    return aStr;
}  // end sci2for


inline std::string asString(const long double x, const std::string::size_type precision)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << x;
    return ss.str();
}


inline std::string Rinex_Printer::asString(const double x, const std::string::size_type precision)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << x;
    return ss.str();
}


inline std::string Rinex_Printer::asFixWidthString(const int x, const int width, char fill_digit)
{
    std::ostringstream ss;
    ss << std::setfill(fill_digit) << std::setw(width) << x;
    return ss.str().substr(ss.str().size() - width);
}


inline int64_t asInt(const std::string& s)
{
    return strtol(s.c_str(), 0, 10);
}


inline int Rinex_Printer::toInt(std::string bitString, int sLength)
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
inline std::string Rinex_Printer::asString(const X x)
{
    std::ostringstream ss;
    ss << x;
    return ss.str();
}


#endif
