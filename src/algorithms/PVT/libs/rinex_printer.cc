/*!
 * \file rinex_printer.cc
 * \brief Implementation of a RINEX 2.11 / 3.02 printer
 * See ftp://igs.org/pub/data/format/rinex302.pdf
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

#include "rinex_printer.h"
#include "GLONASS_L1_L2_CA.h"
#include "beidou_dnav_ephemeris.h"
#include "beidou_dnav_iono.h"
#include "beidou_dnav_utc_model.h"
#include "galileo_ephemeris.h"
#include "galileo_iono.h"
#include "galileo_utc_model.h"
#include "glonass_gnav_ephemeris.h"
#include "glonass_gnav_utc_model.h"
#include "gnss_sdr_filesystem.h"
#include "gnss_synchro.h"
#include "gps_cnav_ephemeris.h"
#include "gps_cnav_iono.h"
#include "gps_cnav_utc_model.h"
#include "gps_ephemeris.h"
#include "gps_iono.h"
#include "gps_navigation_message.h"
#include "gps_utc_model.h"
#include "rtklib_solver.h"
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/local_time/local_time.hpp>
#include <boost/date_time/time_zone_base.hpp>
#include <algorithm>  // for min and max, swap
#include <array>
#include <cmath>  // for floor, abs
#include <exception>
#include <iostream>  // for cout
#include <ostream>
#include <set>
#include <unistd.h>  // for getlogin_r()
#include <utility>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

namespace
{
using Constellation_Observables_Map = std::map<char, std::map<uint32_t, std::map<signal_flag, Gnss_Synchro>>>;

const std::unordered_map<std::string, char> satelliteSystem = {
    {"GPS", 'G'},
    {"GLONASS", 'R'},
    {"SBAS payload", 'S'},
    {"Galileo", 'E'},
    {"Beidou", 'C'},
    {"Mixed", 'M'}};  // RINEX v3.02 codes


std::string signal_flag_to_string(signal_flag flag)
{
    switch (flag)
        {
        case GPS_1C:
            return "1C";
        case GPS_2S:
            return "2S";
        case GPS_L5:
            return "L5";
        case GAL_1B:
            return "1B";
        case GAL_E5a:
            return "5X";
        case GAL_E5b:
            return "7X";
        case GAL_E6:
            return "E6";
        case GLO_1G:
            return "1G";
        case GLO_2G:
            return "2G";
        case BDS_B1:
            return "B1";
        case BDS_B3:
            return "B3";
        }

    throw std::runtime_error("Invalid signal");
}


std::map<std::string, signal_flag> string_to_signal_flag_map()
{
    std::map<std::string, signal_flag> convertion_map;

    for (const auto flag : {GPS_1C, GPS_2S, GPS_L5, GAL_1B, GAL_E5a, GAL_E5b, GAL_E6, GLO_1G, GLO_2G, BDS_B1, BDS_B3})
        {
            convertion_map[signal_flag_to_string(flag)] = flag;
        }

    return convertion_map;
}


signal_flag string_to_signal_flag(const std::string& signal_str)
{
    static const auto convertion_map = string_to_signal_flag_map();
    return convertion_map.at(signal_str);
}


std::map<char, std::set<signal_flag>> get_constel_signal_flags(const Signal_Enabled_Flags& flags)
{
    std::map<char, std::set<signal_flag>> constel_signal_flags;

    for (const auto& it : std::map<char, std::set<signal_flag>>{{'G', {GPS_1C, GPS_2S, GPS_L5}}, {'E', {GAL_1B, GAL_E5a, GAL_E5b, GAL_E6}}, {'R', {GLO_1G, GLO_2G}}, {'C', {BDS_B1, BDS_B3}}})
        {
            for (const auto flag : it.second)
                {
                    if (flags.check_any_enabled(flag))
                        {
                            constel_signal_flags[it.first].insert(flag);
                        }
                }
        }

    return constel_signal_flags;
}


Constellation_Observables_Map get_constellation_observables_map(
    const std::map<char, std::set<signal_flag>>& constel_signal_flags,
    const std::map<int32_t, Gnss_Synchro>& observables)
{
    Constellation_Observables_Map constel_observables;

    // Create an entry for each constellation
    for (const auto& it : constel_signal_flags)
        {
            constel_observables[it.first] = {};
        }

    for (const auto& observables_iter : observables)
        {
            // Find active signals for observable constellation
            const auto& signal_flags_it = constel_signal_flags.find(observables_iter.second.System);

            if (signal_flags_it != constel_signal_flags.end())
                {
                    // Find if the observable signal is active
                    const auto signal_flag = string_to_signal_flag(observables_iter.second.Signal);
                    if (signal_flags_it->second.find(signal_flag) != signal_flags_it->second.end())
                        {
                            // Create an entry for the Constellation / PRN if it doesn't exist
                            auto& signal_obs_map = constel_observables[observables_iter.second.System][observables_iter.second.PRN];

                            // Check if the signal entry exist
                            const auto& signal_obs_it = signal_obs_map.find(signal_flag);

                            // If it doesn't, it's the first one and we create an entry for each active signal, otherwise just update
                            if (signal_obs_it == signal_obs_map.end())
                                {
                                    for (const auto cosntel_signal_flag : signal_flags_it->second)
                                        {
                                            signal_obs_map[cosntel_signal_flag] = Gnss_Synchro{};
                                        }

                                    signal_obs_map[signal_flag] = observables_iter.second;
                                }
                            else
                                {
                                    signal_obs_it->second = observables_iter.second;
                                }
                        }
                }
        }

    return constel_observables;
}


std::map<std::string, std::string> getObservationCodes()
{
    return {
        {"GPS_L1_CA", "1C"},          // "1C" GPS L1 C/A
        {"GPS_L1_P", "1P"},           // "1P" GPS L1 P
        {"GPS_L1_Z_TRACKING", "1W"},  // "1W" GPS L1 Z-tracking and similar (AS on)
        {"GPS_L1_Y", "1Y"},           // "1Y" GPS L1 Y
        {"GPS_L1_M ", "1M"},          // "1M" GPS L1 M
        {"GPS_L1_CODELESS", "1N"},    // "1N" GPS L1 codeless
        {"GPS_L2_CA", "2C"},          // "2C" GPS L2 C/A
        {"L2_SEMI_CODELESS", "2D"},   // "2D" GPS L2 L1(C/A)+(P2-P1) semi-codeless
        {"GPS_L2_L2CM", "2S"},        // "2S" GPS L2 L2C (M)
        {"GPS_L2_L2CL", "2L"},        // "2L" GPS L2 L2C (L)
        {"GPS_L2_L2CML", "2X"},       // "2X" GPS L2 L2C (M+L)
        {"GPS_L2_P", "2P"},           // "2P" GPS L2 P
        {"GPS_L2_Z_TRACKING", "2W"},  // "2W" GPS L2 Z-tracking and similar (AS on)
        {"GPS_L2_Y", "2Y"},           // "2Y" GPS L2 Y
        {"GPS_L2_M", "2M"},           // "2M" GPS GPS L2 M
        {"GPS_L2_codeless", "2N"},    // "2N" GPS L2 codeless
        {"GPS_L5_I", "5I"},           // "5I" GPS L5 I
        {"GPS_L5_Q", "5Q"},           // "5Q" GPS L5 Q
        {"GPS_L5_IQ", "5X"},          // "5X" GPS L5 I+Q
        {"GLONASS_G1_CA", "1C"},      // "1C" GLONASS G1 C/A
        {"GLONASS_G1_P", "1P"},       // "1P" GLONASS G1 P
        {"GLONASS_G2_CA", "2C"},      // "2C" GLONASS G2 C/A  (Glonass M)
        {"GLONASS_G2_P", "2P"},       // "2P" GLONASS G2 P
        {"GALILEO_E1_A", "1A"},       // "1A" GALILEO E1 A (PRS)
        {"GALILEO_E1_B", "1B"},       // "1B" GALILEO E1 B (I/NAV OS/CS/SoL)
        {"GALILEO_E1_C", "1C"},       // "1C" GALILEO E1 C (no data)
        {"GALILEO_E1_BC", "1X"},      // "1X" GALILEO E1 B+C
        {"GALILEO_E1_ABC", "1Z"},     // "1Z" GALILEO E1 A+B+C
        {"GALILEO_E5a_I", "5I"},      // "5I" GALILEO E5a I (F/NAV OS)
        {"GALILEO_E5a_Q", "5Q"},      // "5Q" GALILEO E5a Q  (no data)
        {"GALILEO_E5a_IQ", "5X"},     // "5X" GALILEO E5a I+Q
        {"GALILEO_E5b_I", "7I"},      // "7I" GALILEO E5b I
        {"GALILEO_E5b_Q", "7Q"},      // "7Q" GALILEO E5b Q
        {"GALILEO_E5b_IQ", "7X"},     // "7X" GALILEO E5b I+Q
        {"GALILEO_E5_I", "8I"},       // "8I" GALILEO E5 I
        {"GALILEO_E5_Q", "8Q"},       // "8Q" GALILEO E5 Q
        {"GALILEO_E5_IQ", "8X"},      // "8X" GALILEO E5 I+Q
        {"GALILEO_E56_A", "6A"},      // "6A" GALILEO E6 A
        {"GALILEO_E56_B", "6B"},      // "6B" GALILEO E6 B
        {"GALILEO_E56_C", "6C"},      // "6C" GALILEO E6 C
        {"GALILEO_E56_BC", "6X"},     // "6X" GALILEO E6 B+C
        {"GALILEO_E56_ABC", "6Z"},    // "6Z" GALILEO E6 A+B+C
        {"SBAS_L1_CA", "1C"},         // "1C" SBAS L1 C/A
        {"SBAS_L5_I", "5I"},          // "5I" SBAS L5 I
        {"SBAS_L5_Q", "5Q"},          // "5Q" SBAS L5 Q
        {"SBAS_L5_IQ", "5X"},         // "5X" SBAS L5 I+Q
        {"COMPASS_E2_I", "2I"},
        {"COMPASS_E2_Q", "2Q"},
        {"COMPASS_E2_IQ", "2X"},
        {"COMPASS_E5b_I", "7I"},
        {"COMPASS_E5b_Q", "7Q"},
        {"COMPASS_E5b_IQ", "7X"},
        {"COMPASS_E6_I", "6I"},
        {"COMPASS_E6_Q", "6Q"},
        {"COMPASS_E6_IQ", "6X"},
        {"BEIDOU_B1_I", "1I"},
        {"BEIDOU_B1_Q", "1Q"},
        {"BEIDOU_B1_IQ", "1X"},
        {"BEIDOU_B3_I", "6I"},
        {"BEIDOU_B3_Q", "6Q"},
        {"BEIDOU_B3_IQ", "6X"},
        {"PULSAR_X1", "X1"},
        {"PULSAR_X5", "X5"},

        // RINEX v2.10 and v2.11 codes
        {"GPS_L1_CA_v2", "1"},
        {"GLONASS_G1_CA_v2", "1"}};
}


std::map<std::string, std::string> getObservationTypes()
{
    return {
        {"PSEUDORANGE", "C"},
        {"CARRIER_PHASE", "L"},
        {"DOPPLER", "D"},
        {"SIGNAL_STRENGTH", "S"},

        // RINEX v2.10 and v2.11 codes
        {"PSEUDORANGE_CA_v2", "C"},
        {"PSEUDORANGE_P_v2", "P"},
        {"CARRIER_PHASE_CA_v2", "L"},
        {"DOPPLER_v2", "D"},
        {"SIGNAL_STRENGTH_v2", "S"},
    };
}


std::string getAndCreateBaseRinexPath(const std::string& base_path)
{
    std::string base_rinex_path = base_path;
    fs::path full_path(fs::current_path());
    const fs::path p(base_rinex_path);
    if (!fs::exists(p))
        {
            std::string new_folder;
            for (const auto& folder : fs::path(base_rinex_path))
                {
                    new_folder += folder.string();
                    errorlib::error_code ec;
                    if (!fs::exists(new_folder))
                        {
                            if (!fs::create_directory(new_folder, ec))
                                {
                                    std::cout << "Could not create the " << new_folder << " folder.\n";
                                    base_rinex_path = full_path.string();
                                }
                        }
                    new_folder += fs::path::preferred_separator;
                }
        }
    else
        {
            base_rinex_path = p.string();
        }
    if (base_rinex_path != ".")
        {
            std::cout << "RINEX files will be stored at " << base_rinex_path << '\n';
        }

    return base_rinex_path;
}


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
std::string createFilename(const std::string& type, const std::string& base_name)
{
    const std::string stationName = "GSDR";  // 4-character station name designator
    const boost::gregorian::date today = boost::gregorian::day_clock::local_day();
    const int32_t dayOfTheYear = today.day_of_year();
    std::stringstream strm0;
    if (dayOfTheYear < 100)
        {
            strm0 << "0";  // three digits for day of the year
        }
    if (dayOfTheYear < 10)
        {
            strm0 << "0";  // three digits for day of the year
        }
    strm0 << dayOfTheYear;
    const std::string dayOfTheYearTag = strm0.str();

    const std::map<std::string, std::string> fileType = {
        {"RINEX_FILE_TYPE_OBS", "O"},        // O - Observation file.
        {"RINEX_FILE_TYPE_GPS_NAV", "N"},    // N - GPS navigation message file.
        {"RINEX_FILE_TYPE_MET", "M"},        // M - Meteorological data file.
        {"RINEX_FILE_TYPE_GLO_NAV", "G"},    // G - GLONASS navigation file.
        {"RINEX_FILE_TYPE_GAL_NAV", "L"},    // L - Galileo navigation message file.
        {"RINEX_FILE_TYPE_MIXED_NAV", "P"},  // P - Mixed GNSS navigation message file.
        {"RINEX_FILE_TYPE_GEO_NAV", "H"},    // H - SBAS Payload navigation message file.
        {"RINEX_FILE_TYPE_SBAS", "B"},       // B - SBAS broadcast data file.
        {"RINEX_FILE_TYPE_CLK", "C"},        // C - Clock file.
        {"RINEX_FILE_TYPE_SUMMARY", "S"},    // S - Summary file (used e.g., by IGS, not a standard!).
        {"RINEX_FILE_TYPE_BDS_NAV", "F"},    // F - BeiDou navigation file.
    };

    const boost::posix_time::ptime pt = boost::posix_time::second_clock::local_time();
    const tm pt_tm = boost::posix_time::to_tm(pt);
    const int32_t local_hour = pt_tm.tm_hour;
    std::stringstream strm;
    strm << local_hour;

    const std::map<std::string, std::string> Hmap = {
        {"0", "a"},
        {"1", "b"},
        {"2", "c"},
        {"3", "d"},
        {"4", "e"},
        {"5", "f"},
        {"6", "g"},
        {"7", "h"},
        {"8", "i"},
        {"9", "j"},
        {"10", "k"},
        {"11", "l"},
        {"12", "m"},
        {"13", "n"},
        {"14", "o"},
        {"15", "p"},
        {"16", "q"},
        {"17", "r"},
        {"18", "s"},
        {"19", "t"},
        {"20", "u"},
        {"21", "v"},
        {"22", "w"},
        {"23", "x"},
    };

    const std::string& hourTag = Hmap.at(strm.str());

    const int32_t local_minute = pt_tm.tm_min;
    std::stringstream strm2;
    if (local_minute < 10)
        {
            strm2 << "0";  // at least two digits for minutes
        }
    strm2 << local_minute;

    const std::string minTag = strm2.str();

    const int32_t local_year = pt_tm.tm_year - 100;  // 2012 is 112
    std::stringstream strm3;
    strm3 << local_year;
    std::string yearTag = strm3.str();

    const std::string& typeOfFile = fileType.at(type);
    std::string filename;
    if (base_name == "-")
        {
            filename = stationName + dayOfTheYearTag + hourTag + minTag + "." + yearTag + typeOfFile;
        }
    else
        {
            filename = base_name + "." + yearTag + typeOfFile;
        }
    return filename;
}


std::string getFilePath(const std::string& type, const std::string& base_name, const std::string& base_rinex_path)
{
    return base_rinex_path + fs::path::preferred_separator + createFilename(type, base_name);
}


std::string getNavFilePath(const Signal_Enabled_Flags& flags, int version, const std::string& base_name, const std::string& base_rinex_path)
{
    std::string type;

    if (flags.only_gps)
        {
            type = "RINEX_FILE_TYPE_GPS_NAV";
        }
    else if (flags.only_galileo)
        {
            type = "RINEX_FILE_TYPE_GAL_NAV";
        }
    else if (flags.only_glonass)
        {
            type = "RINEX_FILE_TYPE_GLO_NAV";
        }
    else if (flags.only_beidou)
        {
            type = "RINEX_FILE_TYPE_BDS_NAV";
        }
    else if (version == 2)
        {
            type = "RINEX_FILE_TYPE_GPS_NAV";  // Will need one file for GPS and one for GlONASS
        }
    else
        {
            type = "RINEX_FILE_TYPE_MIXED_NAV";
        }

    return base_rinex_path + fs::path::preferred_separator + createFilename(type, base_name);
}


/*
 * (modified versions from GNSSTk https://github.com/SGL-UT/gnsstk)
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
std::string& leftJustify(std::string& s,
    std::string::size_type length,
    char pad)
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
std::string leftJustify(const std::string& s,
    std::string::size_type length,
    char pad = ' ')
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
std::string& rightJustify(std::string& s,
    std::string::size_type length,
    char pad)
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


/*
 * Right-justifies the receiver in a string of the specified
 * length (const version). If the receiver's data is shorter than the
 * requested length (\a length), it is padded on the left with
 * the pad character (\a pad). The default pad
 * character is a blank.*/
std::string rightJustify(const std::string& s,
    std::string::size_type length,
    char pad = ' ')
{
    std::string t(s);
    return rightJustify(t, length, pad);
}


/*
 * Convert a string to a double precision floating point number.
 * @param s string containing a number.
 * @return double representation of string.
 */
double asDouble(const std::string& s)
{
    return strtod(s.c_str(), nullptr);
}


int toInt(const std::string& bitString, int sLength)
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


/*
 * Convert a string to an integer.
 * @param s string containing a number.
 * @return int64_t  integer representation of string.
 */
int64_t asInt(const std::string& s)
{
    return strtol(s.c_str(), nullptr, 10);
}


/*
 * Convert a double to a string in fixed notation.
 * @param x double.
 * @param precision the number of decimal places you want displayed.
 * @return string representation of \a x.
 */
std::string asString(double x, std::string::size_type precision = 17)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << x;
    return ss.str();
}


/*
 * Convert any old object to a string.
 * The class must have stream operators defined.
 * @param x object to turn into a string.
 * @return string representation of \a x.
 */
template <class X>
std::string asString(const X x)
{
    std::ostringstream ss;
    ss << x;
    return ss.str();
}


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
std::string& sci2for(std::string& aStr,
    std::string::size_type startPos,
    std::string::size_type length,
    std::string::size_type expLen,
    bool checkSwitch)
{
    const auto dotIdx = aStr.find('.', startPos);
    if (dotIdx == std::string::npos || dotIdx <= startPos || dotIdx >= (startPos + length - expLen - 1))
        {
            // Invalid position for decimal point
            return aStr;
        }

    bool redoExp = !checkSwitch;
    int expAdd = 0;

    // Swap digit before '.' if exists, e.g., "1.234E5" -> ".1234E6"
    if (dotIdx > startPos)
        {
            redoExp = true;
            std::swap(aStr[dotIdx], aStr[dotIdx - 1]);
            if (asDouble(aStr.substr(startPos, length)) != 0.0)
                {
                    expAdd = 1;
                }
        }

    // Find exponent marker ('e' or 'E')
    auto expIdx = aStr.find('e', startPos);
    if (expIdx == std::string::npos)
        {
            expIdx = aStr.find('E', startPos);
            if (expIdx == std::string::npos)
                {
                    // No exponent found; not a scientific notation
                    return aStr;
                }
        }

    // Replace exponent letter: 'D' for Fortran-style, or 'E' if checkSwitch is false
    aStr[expIdx] = checkSwitch ? 'D' : 'E';

    if (redoExp)
        {
            std::string expStr = aStr.substr(expIdx + 1);
            int64_t expVal = asInt(expStr) + expAdd;

            // Replace exponent part
            aStr.erase(expIdx + 1);
            aStr += (expVal < 0) ? "-" : "+";
            aStr += rightJustify(asString(std::abs(expVal)), expLen, '0');
        }

    // Ensure leading space for positive values starting with '.'
    if (aStr[startPos] == '.')
        {
            aStr.insert(startPos, 1, ' ');
        }

    // If checkSwitch is false, always insert a leading 0 before the decimal
    if (!checkSwitch && aStr[startPos + 1] == '.')
        {
            aStr.insert(startPos + 1, 1, '0');
        }

    return aStr;
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
std::string doub2sci(double d,
    std::string::size_type length,
    std::string::size_type expLen,
    bool showSign,
    bool checkSwitch)
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
std::string doub2for(double d,
    std::string::size_type length,
    std::string::size_type expLen,
    bool checkSwitch = true)
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


/*
 *  Checks that the line is 80 characters length
 */
void lengthCheck(const std::string& line)
{
    if (line.length() != 80)
        {
            LOG(ERROR) << "Bad defined RINEX line: "
                       << line.length() << " characters (must be 80)" << '\n'
                       << line << '\n'
                       << "----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|\n";
        }
}


/*
 * Generation of RINEX signal strength indicators
 */
int32_t signal_strength(double snr)
{
    auto ss = static_cast<int32_t>(std::min(std::max(static_cast<int32_t>(floor(snr / 6)), 1), 9));
    return ss;
}


void override_stream_with_new_data(std::fstream& out, const std::string& filename, const std::vector<std::string>& data, int64_t seek_pos)
{
    out.close();
    out.open(filename, std::ios::out | std::ios::trunc);
    out.seekp(0);
    for (const auto& line : data)
        {
            out << line << '\n';
        }
    out.close();
    out.open(filename, std::ios::out | std::ios::app);
    out.seekp(seek_pos);
}


void add_obs_rinex_version_and_type(std::fstream& out, const std::string& type, const std::string& version)
{
    std::string line;
    line += std::string(5, ' ');
    line += version;
    line += std::string(11, ' ');
    line += leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem.at(type);
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    lengthCheck(line);
    out << line << '\n';
}


void add_constellation_legend(std::fstream& out, const std::string& constellation_legend)
{
    std::string line;
    line += leftJustify(constellation_legend, 60);
    line += leftJustify("COMMENT", 20);
    lengthCheck(line);
    out << line << '\n';
}


void add_run_by_line(std::fstream& out, const std::string& local_time)
{
    std::string line;
    line += local_time;
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    lengthCheck(line);
    out << line << '\n';
}


void add_generated_by_gnss_sdr(std::fstream& out, const std::string& constellation, const std::string& file_type)
{
    std::string line;
    line += leftJustify(constellation + " " + file_type + " FILE GENERATED BY GNSS-SDR", 60);
    line += leftJustify("COMMENT", 20);
    lengthCheck(line);
    out << line << '\n';
}


void add_gnss_sdr_version(std::fstream& out)
{
    std::string line;
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += leftJustify(gnss_sdr_version, 43);
    line += leftJustify("COMMENT", 20);
    lengthCheck(line);
    out << line << '\n';
}


void add_gnss_sdr_url(std::fstream& out)
{
    std::string line;
    line += leftJustify("See https://gnss-sdr.org", 60);
    line += leftJustify("COMMENT", 20);
    lengthCheck(line);
    out << line << '\n';
}


void add_marker_name(std::fstream& out)
{
    std::string line;
    line += leftJustify("DEFAULT MARKER NAME", 60);  // put a flag or a property,
    line += leftJustify("MARKER NAME", 20);
    lengthCheck(line);
    out << line << '\n';
}


void add_obs_observer_agency(std::fstream& out)
{
    // -------- Line OBSERVER / AGENCY
    std::string line;
    std::string username;
#if ANDROID
    username = "ANDROID USER";
#else
    std::array<char, 20> c_username{};
    const int32_t nGet = getlogin_r(c_username.data(), c_username.size() - 1);
    if (nGet == 0)
        {
            username = c_username.data();
        }
    else
        {
            username = "UNKNOWN USER";
        }
#endif
    line += leftJustify(username, 20);
    line += leftJustify("CTTC", 40);  // add flag and property
    line += leftJustify("OBSERVER / AGENCY", 20);
    lengthCheck(line);
    out << line << '\n';
}


void add_obs_rec_type(std::fstream& out)
{
    std::string line;
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    // -------- Line  REC / TYPE VERS
    line += leftJustify("GNSS-SDR", 20);           // add flag and property
    line += leftJustify("Software Receiver", 20);  // add flag and property
    // line += leftJustify(google::VersionString(), 20); // add flag and property
    if (gnss_sdr_version.length() > 20)
        {
            gnss_sdr_version.resize(9, ' ');
        }
    line += leftJustify(gnss_sdr_version, 20);
    line += leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << '\n';
}


void add_obs_antenna(std::fstream& out)
{
    std::string line;

    // -------- ANTENNA TYPE
    line.clear();
    line += leftJustify("Antenna number", 20);  // add flag and property
    line += leftJustify("Antenna type", 20);    // add flag and property
    line += std::string(20, ' ');
    line += leftJustify("ANT # / TYPE", 20);
    lengthCheck(line);
    out << line << '\n';

    // -------- APPROX POSITION  (optional for moving platforms)
    // put here real data!
    double antena_x = 0.0;
    double antena_y = 0.0;
    double antena_z = 0.0;
    line.clear();
    line += rightJustify(asString(antena_x, 4), 14);
    line += rightJustify(asString(antena_y, 4), 14);
    line += rightJustify(asString(antena_z, 4), 14);
    line += std::string(18, ' ');
    line += leftJustify("APPROX POSITION XYZ", 20);
    lengthCheck(line);
    out << line << '\n';

    // -------- ANTENNA: DELTA H/E/N
    // put here real data!
    double antena_h = 0.0;
    double antena_e = 0.0;
    double antena_n = 0.0;
    line.clear();
    line += rightJustify(asString(antena_h, 4), 14);
    line += rightJustify(asString(antena_e, 4), 14);
    line += rightJustify(asString(antena_n, 4), 14);
    line += std::string(18, ' ');
    line += leftJustify("ANTENNA: DELTA H/E/N", 20);
    lengthCheck(line);
    out << line << '\n';
}


std::string get_local_time(int version)
{
    std::string line;
    line += std::string("GNSS-SDR");
    line += std::string(12, ' ');
    std::string username;
#if ANDROID
    username = "ANDROID USER";
#else
    std::array<char, 20> c_username{};
    const int32_t nGet = getlogin_r(c_username.data(), c_username.size() - 1);
    if (nGet == 0)
        {
            username = c_username.data();
        }
    else
        {
            username = "UNKNOWN USER";
        }
#endif

    line += leftJustify(username, 20);
    const boost::gregorian::date today = boost::gregorian::day_clock::local_day();

    const boost::local_time::time_zone_ptr zone(new boost::local_time::posix_time_zone("UTC"));
    const boost::local_time::local_date_time pt = boost::local_time::local_sec_clock::local_time(zone);
    const tm pt_tm = boost::local_time::to_tm(pt);

    std::stringstream strmHour;
    const int32_t utc_hour = pt_tm.tm_hour;
    if (utc_hour < 10)
        {
            strmHour << "0";  //  two digits for hours
        }
    strmHour << utc_hour;

    std::stringstream strmMin;
    const int32_t utc_minute = pt_tm.tm_min;
    if (utc_minute < 10)
        {
            strmMin << "0";  //  two digits for minutes
        }
    strmMin << utc_minute;

    if (version == 2)
        {
            const int32_t day = pt_tm.tm_mday;
            line += rightJustify(std::to_string(day), 2);
            line += std::string("-");

            const std::map<int32_t, std::string> months = {
                {0, "JAN"}, {1, "FEB"}, {2, "MAR"}, {3, "APR"}, {4, "MAY"}, {5, "JUN"},
                {6, "JUL"}, {7, "AUG"}, {8, "SEP"}, {9, "OCT"}, {10, "NOV"}, {11, "DEC"}};

            line += months.at(pt_tm.tm_mon);
            line += std::string("-");
            line += std::to_string(pt_tm.tm_year - 100);
            line += std::string(1, ' ');
            line += strmHour.str();
            line += std::string(":");
            line += strmMin.str();
            line += std::string(5, ' ');
        }

    if (version == 3)
        {
            line += boost::gregorian::to_iso_string(today);
            line += std::string(1, ' ');
            line += strmHour.str();
            line += strmMin.str();

            std::stringstream strm2;
            const int32_t utc_seconds = pt_tm.tm_sec;
            if (utc_seconds < 10)
                {
                    strm2 << "0";  //  two digits for seconds
                }
            strm2 << utc_seconds;
            line += strm2.str();
            line += std::string(1, ' ');
            line += std::string("UTC");
            line += std::string(1, ' ');
        }
    return line;
}


void add_navigation_header_start(std::fstream& out, const std::string& type, const std::string& constellation, int version, const std::string& version_str)
{
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += version_str;
    line += std::string(11, ' ');

    if (version == 2)
        {
            if (constellation == "GPS")
                {
                    line += std::string("N: GPS NAV DATA");
                    line += std::string(25, ' ');
                }
            else
                {
                    line += std::string("G: GLONASS NAV DATA");
                    line += std::string(21, ' ');
                }
        }
    else
        {
            line += std::string("N: GNSS NAV DATA");
            line += std::string(4, ' ');
            line += type;
            line += std::string(20 - type.size(), ' ');
        }

    line += std::string("RINEX VERSION / TYPE");
    lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    add_run_by_line(out, get_local_time(version));

    // -------- Line COMMENT
    add_generated_by_gnss_sdr(out, constellation, "NAVIGATION MESSAGE");

    // -------- Line COMMENT
    add_gnss_sdr_version(out);

    // -------- Line COMMENT
    add_gnss_sdr_url(out);
}


void add_observation_header_start(std::fstream& out,
    const std::string& type,
    const std::string& constellation,
    const std::string& local_time,
    const std::string& version,
    const std::string& constellation_legend)
{
    // -------- Line 1
    add_obs_rinex_version_and_type(out, type, version);

    // -------- Line 2
    add_constellation_legend(out, constellation_legend);

    // -------- Line 3
    add_run_by_line(out, local_time);

    // -------- Line COMMENT
    add_generated_by_gnss_sdr(out, constellation, "OBSERVATION DATA");

    // -------- Line COMMENT
    add_gnss_sdr_version(out);

    // -------- Line COMMENT
    add_gnss_sdr_url(out);

    // -------- Line MARKER NAME
    add_marker_name(out);

    // -------- Line OBSERVER / AGENCY
    add_obs_observer_agency(out);

    // -------- Line  REC / TYPE VERS
    add_obs_rec_type(out);

    add_obs_antenna(out);
}


std::string get_iono_line(const std::string& identifier, double value0, double value1, double value2, double value3)
{
    std::string line;

    line += identifier;
    line += std::string(1, ' ');
    line += rightJustify(doub2for(value0, 10, 2), 12);
    line += rightJustify(doub2for(value1, 10, 2), 12);
    line += rightJustify(doub2for(value2, 10, 2), 12);
    line += rightJustify(doub2for(value3, 10, 2), 12);
    line += std::string(7, ' ');
    line += leftJustify("IONOSPHERIC CORR", 20);
    lengthCheck(line);

    return line;
}


std::string get_iono_line_v2(const std::string& identifier, double value0, double value1, double value2, double value3)
{
    std::string line;

    line += std::string(2, ' ');
    line += rightJustify(doub2for(value0, 10, 2), 12);
    line += rightJustify(doub2for(value1, 10, 2), 12);
    line += rightJustify(doub2for(value2, 10, 2), 12);
    line += rightJustify(doub2for(value3, 10, 2), 12);
    line += std::string(10, ' ');
    line += leftJustify(identifier, 20);
    lengthCheck(line);

    return line;
}


std::string get_gps_iono_alpha_line_v2(const Gps_Iono& iono)
{
    return get_iono_line_v2("ION ALPHA", iono.alpha0, iono.alpha1, iono.alpha2, iono.alpha3);
}


std::string get_gps_iono_beta_line_v2(const Gps_Iono& iono)
{
    return get_iono_line_v2("ION BETA", iono.beta0, iono.beta1, iono.beta2, iono.beta3);
}


std::string get_delta_utc_line_v2(const Gps_Utc_Model& utc_model, const Gps_Ephemeris& eph, bool pre_2009_file)
{
    std::string line;

    line += std::string(3, ' ');
    line += rightJustify(doub2for(utc_model.A0, 18, 2), 19);
    line += rightJustify(doub2for(utc_model.A1, 18, 2), 19);
    line += rightJustify(std::to_string(utc_model.tot), 9);

    if (pre_2009_file == false && eph.WN < 512)
        {
            if (utc_model.WN_T == 0)
                {
                    line += rightJustify(std::to_string(eph.WN + 2048), 9);  // valid from 2019 to 2029
                }
            else
                {
                    line += rightJustify(std::to_string(utc_model.WN_T + (eph.WN / 256) * 256 + 1024), 9);  // valid from 2019 to 2029
                }
        }
    else
        {
            if (utc_model.WN_T == 0)
                {
                    line += rightJustify(std::to_string(eph.WN + 1024), 9);  // valid from 2019 to 2029
                }
            else
                {
                    line += rightJustify(std::to_string(utc_model.WN_T + (eph.WN / 256) * 256 + 1024), 9);  // valid from 2009 to 2019
                }
        }

    line += std::string(1, ' ');
    line += leftJustify("DELTA-UTC: A0,A1,T,W", 20);
    lengthCheck(line);

    return line;
}


std::string get_time_corr_line(const std::string& identifier, double a0, double a1, const int32_t* tot = nullptr, const int32_t* wn = nullptr)
{
    std::string line;

    line += identifier;
    line += rightJustify(doub2for(a0, 16, 2), 18);
    line += rightJustify(doub2for(a1, 15, 2), 16);

    if (tot && wn)
        {
            line += rightJustify(std::to_string(*tot), 7);
            line += rightJustify(std::to_string(*wn), 5);
            line += std::string(10, ' ');
        }
    else
        {
            line += std::string(22, ' ');
        }

    line += leftJustify("TIME SYSTEM CORR", 20);
    lengthCheck(line);

    return line;
}


// For leap second information, see https://endruntechnologies.com/support/leap-seconds
std::string get_leap_second_line(int32_t Delta_tLS, int32_t Delta_tLSF, int32_t WN_LSF, int32_t DN)
{
    std::string line;

    line += rightJustify(std::to_string(Delta_tLS), 6);
    line += rightJustify(std::to_string(Delta_tLSF), 6);
    line += rightJustify(std::to_string(WN_LSF), 6);
    line += rightJustify(std::to_string(DN), 6);
    line += std::string(36, ' ');
    line += leftJustify("LEAP SECONDS", 20);
    lengthCheck(line);

    return line;
}


std::string get_end_of_header_line()
{
    std::string line;

    line += std::string(60, ' ');
    line += leftJustify("END OF HEADER", 20);
    lengthCheck(line);

    return line;
}


std::string get_gps_iono_alpha_line(const Gps_Iono& iono)
{
    return get_iono_line("GPSA", iono.alpha0, iono.alpha1, iono.alpha2, iono.alpha3);
}


std::string get_gps_iono_beta_line(const Gps_Iono& iono)
{
    return get_iono_line("GPSB", iono.beta0, iono.beta1, iono.beta2, iono.beta3);
}


std::string get_gps_iono_alpha_line(const Gps_CNAV_Iono& iono)
{
    return get_iono_line("GPSA", iono.alpha0, iono.alpha1, iono.alpha2, iono.alpha3);
}


std::string get_gps_iono_beta_line(const Gps_CNAV_Iono& iono)
{
    return get_iono_line("GPSB", iono.beta0, iono.beta1, iono.beta2, iono.beta3);
}


std::string get_galileo_iono_alpha_line(const Galileo_Iono& iono)
{
    return get_iono_line("GAL ", iono.ai0, iono.ai1, iono.ai2, 0.0);
}


std::string get_beidou_iono_alpha_line(const Beidou_Dnav_Iono& iono)
{
    return get_iono_line("BDSA", iono.alpha0, iono.alpha1, iono.alpha2, iono.alpha3);
}


std::string get_beidou_iono_beta_line(const Beidou_Dnav_Iono& iono)
{
    return get_iono_line("BDSB", iono.beta0, iono.beta1, iono.beta2, iono.beta3);
}


std::string get_gps_time_corr_line(const Gps_Utc_Model& utc_model, const Gps_Ephemeris& gps_eph, bool pre_2009_file)
{
    int32_t WN_T = 0;

    if (!pre_2009_file && gps_eph.WN < 512)
        {
            if (utc_model.WN_T == 0)
                {
                    WN_T = gps_eph.WN + 2048;  // valid from 2019 to 2029
                }
            else
                {
                    WN_T = utc_model.WN_T + (gps_eph.WN / 256) * 256 + 2048;  // valid from 2019 to 2029
                }
        }
    else
        {
            if (utc_model.WN_T == 0)
                {
                    WN_T = gps_eph.WN + 1024;  // valid from 2009 to 2019
                }
            else
                {
                    WN_T = utc_model.WN_T + (gps_eph.WN / 256) * 256 + 1024;  // valid from 2009 to 2019
                }
        }

    return get_time_corr_line("GPUT", utc_model.A0, utc_model.A1, &utc_model.tot, &WN_T);
}


std::string get_gps_time_corr_line(const Gps_CNAV_Utc_Model& utc_model)
{
    return get_time_corr_line("GPUT", utc_model.A0, utc_model.A1, &utc_model.tot, &utc_model.WN_T);
}


std::string get_galileo_time_corr_line(const Galileo_Utc_Model& utc_model)
{
    return get_time_corr_line("GAUT", utc_model.A0, utc_model.A1, &utc_model.tot, &utc_model.WNot);
}


std::string get_beidou_time_corr_line(const Beidou_Dnav_Utc_Model& utc_model)
{
    return get_time_corr_line("BDUT", utc_model.A0_UTC, utc_model.A1_UTC);
}


std::string get_glonass_time_corr_line(const Glonass_Gnav_Utc_Model& utc_model)
{
    return get_time_corr_line("GLUT", utc_model.d_tau_c, 0.0, nullptr, nullptr);
}


std::string get_gps_to_galileo_time_corr_line(const Galileo_Utc_Model& utc_model)
{
    return get_time_corr_line("GPGA", utc_model.A_0G, utc_model.A_1G, &utc_model.t_0G, &utc_model.WN_0G);
}


std::string get_glonass_to_gps_time_corr_line(const Glonass_Gnav_Utc_Model& utc_model)
{
    return get_time_corr_line("GLGP", utc_model.d_tau_gps, 0.0, nullptr, nullptr);
}


std::string get_glonass_time_corr_line_v2(const Glonass_Gnav_Utc_Model& utc_model, const Glonass_Gnav_Ephemeris& eph)
{
    // Set reference time and its clock corrections
    const boost::posix_time::ptime p_utc_ref_time = eph.glot_to_utc(eph.d_t_b, 0.0);
    const std::string timestring = boost::posix_time::to_iso_string(p_utc_ref_time);
    const std::string year(timestring, 0, 4);
    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);

    std::string line;
    line += rightJustify(year, 6);
    line += rightJustify(month, 6);
    line += rightJustify(day, 6);
    line += std::string(3, ' ');
    line += rightJustify(doub2for(utc_model.d_tau_c, 19, 2), 19);
    line += std::string(20, ' ');
    line += leftJustify("CORR TO SYSTEM TIME", 20);
    lengthCheck(line);

    return line;
}


std::string get_leap_second_line_v2(const Gps_Utc_Model& utc_model)
{
    std::string line;
    line += rightJustify(std::to_string(utc_model.DeltaT_LS), 6);
    line += std::string(54, ' ');
    line += leftJustify("LEAP SECONDS", 20);
    lengthCheck(line);
    return line;
}


std::string get_leap_second_line(const Gps_Utc_Model& utc_model)
{
    return get_leap_second_line(utc_model.DeltaT_LS, utc_model.DeltaT_LSF, utc_model.WN_LSF, utc_model.DN);
}


std::string get_leap_second_line(const Gps_CNAV_Utc_Model& utc_model)
{
    return get_leap_second_line(utc_model.DeltaT_LS, utc_model.DeltaT_LSF, utc_model.WN_LSF, utc_model.DN);
}


std::string get_leap_second_line(const Galileo_Utc_Model& utc_model)
{
    return get_leap_second_line(utc_model.Delta_tLS, utc_model.Delta_tLSF, utc_model.WN_LSF, utc_model.DN);
}


std::string get_leap_second_line(const Beidou_Dnav_Utc_Model& utc_model)
{
    return get_leap_second_line(utc_model.DeltaT_LS, utc_model.DeltaT_LSF, utc_model.WN_LSF, utc_model.DN);
}


void write_two_digits_string(const std::string& two_digit_string, bool remove_leading_zero, std::string& line)
{
    if (remove_leading_zero && boost::lexical_cast<int32_t>(two_digit_string) < 10)
        {
            line += std::string(1, ' ');
            line += two_digit_string.substr(1, 1);
        }
    else
        {
            line += two_digit_string;
        }
}


void add_svclk_to_line(const boost::posix_time::ptime& system_time, bool log_seconds, int version, std::string& line)
{
    const std::string timestring = boost::posix_time::to_iso_string(system_time);
    const std::string year(timestring, 0, 4);
    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);
    const std::string seconds(timestring, 13, 2);

    line += std::string(1, ' ');
    line += year;
    line += std::string(1, ' ');
    write_two_digits_string(month, version == 2, line);
    line += std::string(1, ' ');
    write_two_digits_string(day, version == 2, line);
    line += std::string(1, ' ');
    line += hour;
    line += std::string(1, ' ');
    line += minutes;
    line += std::string(1, ' ');

    if (log_seconds)
        {
            line += seconds;
            line += std::string(1, ' ');
        }
}


std::string get_obs_epoch_record_lines(const boost::posix_time::ptime& system_time, double seconds, int version)
{
    // -------- EPOCH record
    std::string line = std::string(1, '>');

    // double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    // const double gps_t = eph.sv_clock_correction(obs_time);

    add_svclk_to_line(system_time, false, version, line);

    // Add extra 0 if seconds are < 10
    if (seconds < 10)
        {
            line += std::string(1, '0');
        }
    line += asString(seconds, 7);
    line += std::string(2, ' ');

    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    return line;
}


void add_obs_sat_record_line(const Gnss_Synchro& synchro, std::string& line, bool padding = true)
{
    const int32_t ssi = signal_strength(synchro.CN0_dB_hz);
    const char lli = synchro.Flag_cycle_slip ? '1' : ' ';

    // PSEUDORANGE
    line += rightJustify(asString(synchro.Pseudorange_m, 3), 14);
    line += std::string(1, lli);                      // Loss of lock indicator (LLI)
    line += rightJustify(asString<int32_t>(ssi), 1);  // Signal Strength Indicator (SSI)

    // PHASE
    line += rightJustify(asString(synchro.Carrier_phase_rads / TWO_PI, 3), 14);
    line += std::string(1, lli);                      // Loss of lock indicator (LLI)
    line += rightJustify(asString<int32_t>(ssi), 1);  // Signal Strength Indicator (SSI)

    // DOPPLER
    line += rightJustify(asString(synchro.Carrier_Doppler_hz, 3), 14);
    line += std::string(1, lli);                      // Loss of lock indicator (LLI)
    line += rightJustify(asString<int32_t>(ssi), 1);  // Signal Strength Indicator (SSI)

    // SIGNAL STRENGTH
    line += rightJustify(asString(synchro.CN0_dB_hz, 3), 14);

    if (padding && line.size() < 80)
        {
            line += std::string(80 - line.size(), ' ');
        }
}


void add_constellation_obs_sat_record_lines(std::fstream& out, const std::string& system, const Constellation_Observables_Map& observables, int version)
{
    const auto system_char = satelliteSystem.at(system);

    for (const auto& it : observables.at(system_char))
        {
            std::string line;

            if (version == 3)
                {
                    const auto prn = it.first;
                    line += system_char;
                    if (static_cast<int32_t>(prn) < 10)
                        {
                            line += std::string(1, '0');
                        }
                    line += std::to_string(static_cast<int32_t>(prn));
                }

            for (const auto& observable_iter : it.second)
                {
                    add_obs_sat_record_line(observable_iter.second, line, false);
                }

            if (line.size() < 80)
                {
                    line += std::string(80 - line.size(), ' ');
                }

            out << line << '\n';
        }
}


void add_constellation_obs_sat_record_lines(std::fstream& out, const Signal_Enabled_Flags& flags, const Constellation_Observables_Map& observables, int version)
{
    if (flags.has_gps)
        {
            add_constellation_obs_sat_record_lines(out, "GPS", observables, version);
        }
    if (flags.has_galileo)
        {
            add_constellation_obs_sat_record_lines(out, "Galileo", observables, version);
        }
    if (flags.has_glonass)
        {
            add_constellation_obs_sat_record_lines(out, "GLONASS", observables, version);
        }
    if (flags.has_beidou)
        {
            add_constellation_obs_sat_record_lines(out, "Beidou", observables, version);
        }
}


std::string get_nav_sv_epoch_svclk_line(const boost::posix_time::ptime& p_utc_time, char sys_char, uint32_t prn, double value0, double value1, double value2)
{
    std::string line;

    line += sys_char;

    if (prn < 10)
        {
            line += std::string("0");
        }

    line += std::to_string(prn);

    add_svclk_to_line(p_utc_time, true, 3, line);

    line += doub2for(value0, 18, 2);
    line += std::string(1, ' ');
    line += doub2for(value1, 18, 2);
    line += std::string(1, ' ');
    line += doub2for(value2, 18, 2);

    lengthCheck(line);

    return line;
}


std::string get_nav_broadcast_orbit(const double* value0, const double* value1, const double* value2, const double* value3, uint32_t version = 3)
{
    std::string line;

    line += std::string(version + 2, ' ');
    line += value0 ? doub2for(*value0, 18, 2) : std::string(18, ' ');
    line += std::string(1, ' ');
    line += value1 ? doub2for(*value1, 18, 2) : std::string(18, ' ');
    line += std::string(1, ' ');
    line += value2 ? doub2for(*value2, 18, 2) : std::string(18, ' ');
    line += std::string(1, ' ');
    line += value3 ? doub2for(*value3, 18, 2) : std::string(18, ' ');

    if (version == 2)
        {
            line += std::string(1, ' ');
        }

    lengthCheck(line);

    return line;
}


void add_obs_signal_strength(std::fstream& out)
{
    std::string line;
    line += leftJustify("DBHZ", 20);
    line += std::string(40, ' ');
    line += leftJustify("SIGNAL STRENGTH UNIT", 20);
    lengthCheck(line);
    out << line << '\n';
}


void add_obs_time_first_obs(std::fstream& out, const std::string& constellation, const boost::posix_time::ptime& p_gps_time, double seconds)
{
    const std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    const std::string year(timestring, 0, 4);
    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);
    std::string line;
    line += rightJustify(year, 6);
    line += rightJustify(month, 6);
    line += rightJustify(day, 6);
    line += rightJustify(hour, 6);
    line += rightJustify(minutes, 6);
    line += rightJustify(asString(seconds, 7), 13);
    line += rightJustify(constellation, 8);
    line += std::string(9, ' ');
    line += leftJustify("TIME OF FIRST OBS", 20);
    lengthCheck(line);
    out << line << '\n';
}


std::string get_datetime_v2(const boost::posix_time::ptime& p_utc_time)
{
    std::string line;

    const std::string timestring = boost::posix_time::to_iso_string(p_utc_time);
    const std::string year(timestring, 2, 2);
    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);
    const std::string seconds(timestring, 13, 2);

    line += year;
    line += std::string(1, ' ');
    write_two_digits_string(month, true, line);
    line += std::string(1, ' ');
    write_two_digits_string(day, true, line);
    line += std::string(1, ' ');
    write_two_digits_string(hour, true, line);
    line += std::string(1, ' ');
    write_two_digits_string(minutes, true, line);
    line += std::string(1, ' ');
    write_two_digits_string(seconds, true, line);
    line += std::string(1, '.');
    std::string decimal = "0";
    if (timestring.size() > 16)
        {
            decimal = std::string(timestring, 16, 1);
        }
    line += decimal;

    return line;
}


void add_obs_sys_obs_type(std::fstream& out,
    char sys_char,
    int numberTypesObservations,
    const std::map<std::string, std::string>& observationType,
    const std::vector<std::string>& obsCodes)
{
    // one line per available system
    std::string line;
    line += sys_char;
    line += std::string(2, ' ');
    std::stringstream strm;
    strm << numberTypesObservations;
    line += rightJustify(strm.str(), 3);

    for (const auto& obsCode : obsCodes)
        {
            line += std::string(1, ' ');
            line += observationType.at("PSEUDORANGE");
            line += obsCode;
            line += std::string(1, ' ');
            line += observationType.at("CARRIER_PHASE");
            line += obsCode;
            line += std::string(1, ' ');
            line += observationType.at("DOPPLER");
            line += obsCode;
            line += std::string(1, ' ');
            line += observationType.at("SIGNAL_STRENGTH");
            line += obsCode;
        }

    line += std::string(60 - line.size(), ' ');
    line += leftJustify("SYS / # / OBS TYPES", 20);
    lengthCheck(line);
    out << line << '\n';
}


void add_wavelength(std::fstream& out)
{
    // put here real data!
    std::string line;
    line += rightJustify("1", 6);
    line += rightJustify("1", 6);
    line += std::string(48, ' ');
    line += leftJustify("WAVELENGTH FACT L1/2", 20);
    lengthCheck(line);
    out << line << '\n';
}


void add_obs_sys_obs_type(std::fstream& out,
    const std::string& constellation,
    const Signal_Enabled_Flags& flags,
    const std::map<std::string, std::string>& observationType,
    const std::map<std::string, std::string>& observationCode,
    const std::map<uint32_t, std::string>& signal_to_code_map)
{
    std::vector<std::string> obsCodes;
    uint32_t number_of_observations = 0;

    for (const auto& it : signal_to_code_map)
        {
            if (flags.check_any_enabled(it.first))
                {
                    obsCodes.emplace_back(observationCode.at(it.second));
                    number_of_observations += 4;
                }
        }

    const auto& sys_char = satelliteSystem.at(constellation);
    add_obs_sys_obs_type(out, sys_char, number_of_observations, observationType, obsCodes);
}


void add_obs_sys_obs_type_gps(std::fstream& out,
    const Signal_Enabled_Flags& flags,
    const std::map<std::string, std::string>& observationType,
    const std::map<std::string, std::string>& observationCode)
{
    const std::map<uint32_t, std::string> signal_to_code_map = {
        {GPS_1C, "GPS_L1_CA"},
        {GPS_2S, "GPS_L2_L2CM"},
        {GPS_L5, "GPS_L5_Q"},
    };

    add_obs_sys_obs_type(out, "GPS", flags, observationType, observationCode, signal_to_code_map);
}


void add_obs_sys_obs_type_galileo(std::fstream& out,
    const Signal_Enabled_Flags& flags,
    const std::map<std::string, std::string>& observationType,
    const std::map<std::string, std::string>& observationCode)
{
    const std::map<uint32_t, std::string> signal_to_code_map = {
        {GAL_1B, "GALILEO_E1_B"},
        {GAL_E5a, "GALILEO_E5a_IQ"},
        {GAL_E5b, "GALILEO_E5b_IQ"},
        {GAL_E6, "GALILEO_E56_B"},
    };

    add_obs_sys_obs_type(out, "Galileo", flags, observationType, observationCode, signal_to_code_map);
}


void add_obs_sys_obs_type_glonass(std::fstream& out,
    const Signal_Enabled_Flags& flags,
    const std::map<std::string, std::string>& observationType,
    const std::map<std::string, std::string>& observationCode)
{
    const std::map<uint32_t, std::string> signal_to_code_map = {
        {GLO_1G, "GLONASS_G1_CA"},
        {GLO_2G, "GLONASS_G2_CA"},
    };

    add_obs_sys_obs_type(out, "GLONASS", flags, observationType, observationCode, signal_to_code_map);
}


void add_obs_sys_obs_type_beidou(std::fstream& out,
    const Signal_Enabled_Flags& flags,
    const std::map<std::string, std::string>& observationType,
    const std::map<std::string, std::string>& observationCode)
{
    const std::map<uint32_t, std::string> signal_to_code_map = {
        {BDS_B1, "BEIDOU_B1_I"},
        {BDS_B3, "BEIDOU_B3_I"},
    };

    add_obs_sys_obs_type(out, "Beidou", flags, observationType, observationCode, signal_to_code_map);
}


void add_obs_sys_obs_type_v2(std::fstream& out,
    const std::string& codeKey,
    const std::map<std::string, std::string>& observationType,
    const std::map<std::string, std::string>& observationCode)
{
    std::string line;
    std::stringstream strm;
    strm << 4;
    line += rightJustify(strm.str(), 6);
    // per type of observation
    line += rightJustify(observationType.at("PSEUDORANGE_CA_v2"), 5);
    line += observationCode.at(codeKey);
    line += rightJustify(observationType.at("CARRIER_PHASE_CA_v2"), 5);
    line += observationCode.at(codeKey);
    line += rightJustify(observationType.at("DOPPLER_v2"), 5);
    line += observationCode.at(codeKey);
    line += rightJustify(observationType.at("SIGNAL_STRENGTH_v2"), 5);
    line += observationCode.at(codeKey);
    line += std::string(60 - line.size(), ' ');
    line += leftJustify("# / TYPES OF OBSERV", 20);
    lengthCheck(line);
    out << line << '\n';
}


void add_obs_glonass_slot_freq(std::fstream& out)
{
    // TODO Need to provide system with list of all satellites and update this accordingly
    std::string line;
    line += rightJustify(std::to_string(0), 3);  // Number of satellites in list
    line += std::string(1, ' ');
    line += satelliteSystem.at("GLONASS");
    line += rightJustify(std::to_string(0), 2);  // Slot Number
    line += std::string(1, ' ');
    line += rightJustify(std::to_string(0), 2);  // Frequency Number
    line += std::string(1, ' ');
    line += std::string(60 - line.size(), ' ');
    line += leftJustify("GLONASS SLOT / FRQ #", 20);
    lengthCheck(line);
    out << line << '\n';
}


void add_obs_glonass_code_phase_bias(std::fstream& out,
    const std::map<std::string, std::string>& observationType,
    const std::map<std::string, std::string>& observationCode)
{
    // No GLONASS Phase bias correction used to align code and phase observations.
    std::string line;
    line += std::string(1, ' ');
    line += observationType.at("PSEUDORANGE");
    line += observationCode.at("GLONASS_G1_CA");
    line += std::string(1, ' ');
    line += rightJustify(asString(0.0, 3), 8);
    line += std::string(1, ' ');
    line += observationType.at("PSEUDORANGE");
    line += observationCode.at("GLONASS_G1_P");
    line += std::string(1, ' ');
    line += rightJustify(asString(0.0, 3), 8);
    line += std::string(1, ' ');
    line += observationType.at("PSEUDORANGE");
    line += observationCode.at("GLONASS_G2_CA");
    line += std::string(1, ' ');
    line += rightJustify(asString(0.0, 3), 8);
    line += std::string(1, ' ');
    line += observationType.at("PSEUDORANGE");
    line += observationCode.at("GLONASS_G2_P");
    line += std::string(1, ' ');
    line += rightJustify(asString(0.0, 3), 8);
    line += std::string(60 - line.size(), ' ');
    line += leftJustify("GLONASS COD/PHS/BIS", 20);
    lengthCheck(line);
    out << line << '\n';
}


void add_obs_epoch_record(std::fstream& out, const boost::posix_time::ptime& system_time, double seconds, int version, const Constellation_Observables_Map& constel_observables)
{
    std::string line = get_obs_epoch_record_lines(system_time, seconds, version);

    int32_t number_satellites = 0;

    for (const auto& it : constel_observables)
        {
            number_satellites += it.second.size();
        }

    line += rightJustify(std::to_string(number_satellites), 3);

    // Receiver clock offset (optional)
    // line += rightJustify(asString(clockOffset, 12), 15);

    if (version == 2)
        {
            for (const auto& it : constel_observables)
                {
                    for (const auto& observables_iter : it.second)
                        {
                            line += it.first;
                            if (static_cast<int32_t>(observables_iter.first) < 10)
                                {
                                    line += std::string(1, '0');
                                }
                            line += std::to_string(static_cast<int32_t>(observables_iter.first));
                        }
                }
        }

    line += std::string(80 - line.size(), ' ');
    lengthCheck(line);
    out << line << '\n';
}


struct NavHeaderInfo
{
    std::string prefix;
    std::string suffix;
    std::string new_line;

    NavHeaderInfo() = default;

    NavHeaderInfo(std::string p, std::string s, std::string nl)
        : prefix(std::move(p)), suffix(std::move(s)), new_line(std::move(nl))
    {
    }
};


void update_nav_header_from_info(std::fstream& out, const std::string& filename, const std::vector<NavHeaderInfo>& infos)
{
    std::vector<std::string> data;

    const int64_t pos = out.tellp();
    out.seekp(0);
    data.clear();

    bool no_more_finds = false;
    std::string line_str;

    while (!out.eof())
        {
            std::getline(out, line_str);

            if (!no_more_finds)
                {
                    bool updated = false;

                    for (const auto& info : infos)
                        {
                            if ((info.prefix.empty() || line_str.find(info.prefix, 0) != std::string::npos) &&
                                (info.suffix.empty() || line_str.find(info.suffix, 59) != std::string::npos))
                                {
                                    data.push_back(info.new_line);
                                    updated = true;
                                    break;
                                }
                        }

                    if (!updated)
                        {
                            data.push_back(line_str);

                            if (line_str.find("END OF HEADER", 59) != std::string::npos)
                                {
                                    no_more_finds = true;
                                }
                        }
                }
            else
                {
                    data.push_back(line_str);
                }
        }

    override_stream_with_new_data(out, filename, data, pos);
}


int get_version(const Signal_Enabled_Flags& flags, int version)
{
    if (version == 2)
        {
            if (flags.check_only_enabled(GPS_1C) ||
                flags.check_only_enabled(GLO_1G) ||
                flags.check_only_enabled(GPS_1C, GLO_1G))
                {
                    return 2;
                }

            std::cout << "Changing RINEX version to 3.02, because version 2.11 is incompatible with signal selection.\n";
        }

    return 3;  // Only support version 2.11 and 3.02
}

}  // namespace


Rinex_Printer::Rinex_Printer(uint32_t signal_enabled_flags,
    int version,
    const std::string& base_path,
    const std::string& base_name,
    bool pre_2009_file) : Rinex_Printer(signal_enabled_flags, base_name, getAndCreateBaseRinexPath(base_path), version, pre_2009_file)
{
}


Rinex_Printer::Rinex_Printer(uint32_t signal_enabled_flags,
    const std::string& base_name,
    const std::string& base_rinex_path,
    int version,
    bool pre_2009_file) : observationType(getObservationTypes()),
                          observationCode(getObservationCodes()),
                          d_flags(signal_enabled_flags),
                          d_version(get_version(d_flags, version)),
                          d_stringVersion(d_version == 2 ? "2.11" : "3.02"),  // Only version 2.11 and 3.02
                          d_fake_cnav_iode(1),
                          d_rinex_header_updated(false),
                          d_rinex_header_gps_updated(!d_flags.has_gps),
                          d_rinex_header_galileo_updated(!d_flags.has_galileo),
                          d_rinex_header_glonass_updated(!d_flags.has_glonass),
                          d_rinex_header_beidou_updated(!d_flags.has_beidou),
                          d_rinex_header_written(false),
                          d_pre_2009_file(pre_2009_file),
                          navfilename(getNavFilePath(d_flags, d_version, base_name, base_rinex_path)),
                          obsfilename(getFilePath("RINEX_FILE_TYPE_OBS", base_name, base_rinex_path)),
                          navGlofilename(getFilePath("RINEX_FILE_TYPE_GLO_NAV", base_name, base_rinex_path)),
                          output_navfilename({navfilename})
{
    std::map<std::string, std::fstream&> fileMap = {
        {navfilename, navFile},
        {obsfilename, obsFile}};

    if (d_version == 2 && navfilename != navGlofilename)
        {
            fileMap.emplace(navGlofilename, navGloFile);
        }

    bool all_open = true;

    for (const auto& it : fileMap)
        {
            it.second.open(it.first, std::ios::out | std::ios::in | std::ios::app);
            all_open = all_open && it.second.is_open();
        }

    if (!all_open)
        {
            std::cout << "RINEX files cannot be saved. Wrong permissions?\n";
        }
}


Rinex_Printer::~Rinex_Printer()
{
    DLOG(INFO) << "RINEX printer destructor called.";

    std::map<std::string, std::fstream&> fileMap = {
        {navfilename, navFile},
        {obsfilename, obsFile}};

    if (d_version == 2 && navfilename != navGlofilename)
        {
            fileMap.emplace(navGlofilename, navGloFile);
        }

    std::map<std::string, decltype(navFile.tellp())> filePosMap;

    for (const auto& it : fileMap)
        {
            filePosMap[it.first] = it.second.tellp();
        }

    try
        {
            for (const auto& it : fileMap)
                {
                    it.second.close();
                }
        }
    catch (const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

    for (const auto& it : filePosMap)
        {
            if (it.second == 0)
                {
                    errorlib::error_code ec;
                    if (!fs::remove(fs::path(it.first), ec))
                        {
                            LOG(INFO) << "Error deleting temporary file";
                        }
                }
        }
}


void Rinex_Printer::print_rinex_annotation(const Rtklib_Solver* pvt_solver,
    const std::map<int, Gnss_Synchro>& gnss_observables_map,
    double rx_time,
    bool flag_write_RINEX_obs_output)
{
    const auto galileo_ephemeris_iter = pvt_solver->galileo_ephemeris_map.cbegin();
    const auto gps_ephemeris_iter = pvt_solver->gps_ephemeris_map.cbegin();
    const auto gps_cnav_ephemeris_iter = pvt_solver->gps_cnav_ephemeris_map.cbegin();
    const auto glonass_gnav_ephemeris_iter = pvt_solver->glonass_gnav_ephemeris_map.cbegin();
    const auto beidou_dnav_ephemeris_iter = pvt_solver->beidou_dnav_ephemeris_map.cbegin();

    const bool has_gps_lnav_eph = !pvt_solver->gps_ephemeris_map.empty();
    const bool has_gps_cnav_eph = !pvt_solver->gps_cnav_ephemeris_map.empty();
    const bool has_galileo_eph = !pvt_solver->galileo_ephemeris_map.empty();
    const bool has_glonass_eph = !pvt_solver->glonass_gnav_ephemeris_map.empty();
    const bool has_beidou_dnav_eph = !pvt_solver->beidou_dnav_ephemeris_map.empty();

    // We require at least one ephemeris for an active signal of a constellation
    // Note: this is currently required because ephemeris are used when creating the headers,
    //       but we already have code to update the headers later, so this should not be needed,
    //       we should instead write blank statements in the header and update them when we receive
    //       the ephemeris of the different constellations
    if (d_flags.check_any_enabled(GPS_1C) && !has_gps_lnav_eph)
        {
            return;
        }
    if (d_flags.check_any_enabled(GPS_2S, GPS_L5) && !has_gps_cnav_eph)
        {
            return;
        }
    if (d_flags.has_galileo && !has_galileo_eph)
        {
            return;
        }
    if (d_flags.has_glonass && !has_glonass_eph)
        {
            return;
        }
    if (d_flags.has_beidou && !has_beidou_dnav_eph)
        {
            return;
        }

    double seconds{};
    boost::posix_time::ptime system_time;
    std::string system_time_str;

    // Order is important, it defines which time is used with multiple constellations
    if (d_flags.check_any_enabled(GPS_1C))
        {
            system_time = Rinex_Printer::compute_GPS_time(gps_ephemeris_iter->second, rx_time);
            seconds = fmod(rx_time, 60);
            system_time_str = "GPS";
        }
    else if (d_flags.has_gps)
        {
            system_time = Rinex_Printer::compute_GPS_time(gps_cnav_ephemeris_iter->second, rx_time);
            seconds = fmod(rx_time, 60);
            system_time_str = "GPS";
        }
    else if (d_flags.has_galileo)
        {
            system_time = Rinex_Printer::compute_Galileo_time(galileo_ephemeris_iter->second, rx_time);
            seconds = fmod(rx_time, 60);
            system_time_str = "GAL";
        }
    else if (d_flags.has_glonass)
        {
            double int_sec = 0;
            system_time = Rinex_Printer::compute_UTC_time(glonass_gnav_ephemeris_iter->second, rx_time);
            seconds = modf(rx_time, &int_sec) + system_time.time_of_day().seconds();
            system_time_str = "GLO";
        }
    else if (d_flags.has_beidou)
        {
            system_time = Rinex_Printer::compute_BDS_time(beidou_dnav_ephemeris_iter->second, rx_time);
            seconds = fmod(rx_time, 60);
            system_time_str = "BDS";
        }

    if (system_time.is_not_a_date_time())
        {
            return;
        }

    if (!d_rinex_header_written)  // & we have utc data in nav message!
        {
            std::vector<std::string> iono_lines;
            std::vector<std::string> time_corr_lines;
            std::string leap_second_line;

            if (d_flags.check_any_enabled(GPS_1C))
                {
                    if (d_version == 2)
                        {
                            iono_lines.emplace_back(get_gps_iono_alpha_line_v2(pvt_solver->gps_iono));
                            iono_lines.emplace_back(get_gps_iono_beta_line_v2(pvt_solver->gps_iono));
                            time_corr_lines.emplace_back(get_delta_utc_line_v2(pvt_solver->gps_utc_model, gps_ephemeris_iter->second, d_pre_2009_file));
                            leap_second_line = get_leap_second_line_v2(pvt_solver->gps_utc_model);
                        }
                    else
                        {
                            iono_lines.emplace_back(get_gps_iono_alpha_line(pvt_solver->gps_iono));
                            iono_lines.emplace_back(get_gps_iono_beta_line(pvt_solver->gps_iono));
                            time_corr_lines.emplace_back(get_gps_time_corr_line(pvt_solver->gps_utc_model, gps_ephemeris_iter->second, d_pre_2009_file));
                            leap_second_line = get_leap_second_line(pvt_solver->gps_utc_model);
                        }
                }
            else if (d_flags.check_any_enabled(GPS_2S, GPS_L5))
                {
                    iono_lines.emplace_back(get_gps_iono_alpha_line(pvt_solver->gps_cnav_iono));
                    iono_lines.emplace_back(get_gps_iono_beta_line(pvt_solver->gps_cnav_iono));
                    time_corr_lines.emplace_back(get_gps_time_corr_line(pvt_solver->gps_cnav_utc_model));
                    leap_second_line = get_leap_second_line(pvt_solver->gps_cnav_utc_model);
                }

            if (d_flags.has_galileo)
                {
                    iono_lines.emplace_back(get_galileo_iono_alpha_line(pvt_solver->galileo_iono));
                    time_corr_lines.emplace_back(get_galileo_time_corr_line(pvt_solver->galileo_utc_model));
                    time_corr_lines.emplace_back(get_gps_to_galileo_time_corr_line(pvt_solver->galileo_utc_model));

                    if (system_time_str == "GAL")
                        {
                            leap_second_line = get_leap_second_line(pvt_solver->galileo_utc_model);
                        }
                }
            if (d_flags.has_glonass)
                {
                    if (d_version == 2)
                        {
                            auto time_corr_line = get_glonass_time_corr_line_v2(pvt_solver->glonass_gnav_utc_model, glonass_gnav_ephemeris_iter->second);

                            if (navfilename != navGlofilename)
                                {
                                    rinex_nav_header(navGloFile, {}, {std::move(time_corr_line)}, {});
                                    output_navfilename.push_back(navGlofilename);
                                }
                            else
                                {
                                    time_corr_lines.emplace_back(std::move(time_corr_line));
                                }
                        }
                    else
                        {
                            time_corr_lines.emplace_back(get_glonass_time_corr_line(pvt_solver->glonass_gnav_utc_model));
                            time_corr_lines.emplace_back(get_glonass_to_gps_time_corr_line(pvt_solver->glonass_gnav_utc_model));
                        }
                }
            if (d_flags.has_beidou)
                {
                    iono_lines.emplace_back(get_beidou_iono_alpha_line(pvt_solver->beidou_dnav_iono));
                    iono_lines.emplace_back(get_beidou_iono_beta_line(pvt_solver->beidou_dnav_iono));
                    time_corr_lines.emplace_back(get_beidou_time_corr_line(pvt_solver->beidou_dnav_utc_model));

                    if (system_time_str == "BDS")
                        {
                            leap_second_line = get_leap_second_line(pvt_solver->beidou_dnav_utc_model);
                        }
                }

            rinex_obs_header(obsFile, system_time_str, system_time, seconds);
            rinex_nav_header(navFile, iono_lines, time_corr_lines, leap_second_line);

            if (has_gps_lnav_eph && !d_flags.check_any_enabled(GPS_L5))  // That's how it used to be, not sure why
                {
                    log_rinex_nav_gps_nav(pvt_solver->gps_ephemeris_map);
                }
            else if (has_gps_cnav_eph)
                {
                    log_rinex_nav_gps_cnav(pvt_solver->gps_cnav_ephemeris_map);
                }

            if (has_galileo_eph)
                {
                    log_rinex_nav_gal_nav(pvt_solver->galileo_ephemeris_map);
                }
            if (has_glonass_eph)
                {
                    log_rinex_nav_glo_gnav(pvt_solver->glonass_gnav_ephemeris_map);
                }
            if (has_beidou_dnav_eph)
                {
                    log_rinex_nav_bds_dnav(pvt_solver->beidou_dnav_ephemeris_map);
                }

            d_rinex_header_written = true;
        }

    if (d_rinex_header_written && flag_write_RINEX_obs_output)  // The header is already written, we can now log the navigation message data
        {
            const auto constel_signal_flags = get_constel_signal_flags(d_flags);
            const auto constel_observables = get_constellation_observables_map(constel_signal_flags, gnss_observables_map);
            add_obs_epoch_record(obsFile, system_time, seconds, d_version, constel_observables);
            add_constellation_obs_sat_record_lines(obsFile, d_flags, constel_observables, d_version);

            if (!d_rinex_header_updated)
                {
                    std::string leap_second_line;
                    std::vector<NavHeaderInfo> nav_header_info;

                    if (!d_rinex_header_gps_updated)
                        {
                            if (d_flags.check_any_enabled(GPS_1C) && pvt_solver->gps_utc_model.A0 != 0)
                                {
                                    if (d_version == 2)
                                        {
                                            nav_header_info.emplace_back("", "ION ALPHA", get_gps_iono_alpha_line_v2(pvt_solver->gps_iono));
                                            nav_header_info.emplace_back("", "ION BETA", get_gps_iono_beta_line_v2(pvt_solver->gps_iono));
                                            nav_header_info.emplace_back("", "DELTA-UTC", get_delta_utc_line_v2(pvt_solver->gps_utc_model, gps_ephemeris_iter->second, d_pre_2009_file));
                                            leap_second_line = get_leap_second_line_v2(pvt_solver->gps_utc_model);
                                        }
                                    else
                                        {
                                            nav_header_info.emplace_back("GPSA", "IONOSPHERIC CORR", get_gps_iono_alpha_line(pvt_solver->gps_iono));
                                            nav_header_info.emplace_back("GPSB", "IONOSPHERIC CORR", get_gps_iono_beta_line(pvt_solver->gps_iono));
                                            nav_header_info.emplace_back("GPUT", "TIME SYSTEM CORR", get_gps_time_corr_line(pvt_solver->gps_utc_model, gps_ephemeris_iter->second, d_pre_2009_file));
                                            leap_second_line = get_leap_second_line(pvt_solver->gps_utc_model);
                                        }
                                    d_rinex_header_gps_updated = true;
                                }
                            else if (d_flags.check_any_enabled(GPS_2S, GPS_L5) && pvt_solver->gps_cnav_utc_model.A0 != 0)
                                {
                                    nav_header_info.emplace_back("GPSA", "IONOSPHERIC CORR", get_gps_iono_alpha_line(pvt_solver->gps_cnav_iono));
                                    nav_header_info.emplace_back("GPSB", "IONOSPHERIC CORR", get_gps_iono_beta_line(pvt_solver->gps_cnav_iono));
                                    nav_header_info.emplace_back("GPUT", "TIME SYSTEM CORR", get_gps_time_corr_line(pvt_solver->gps_cnav_utc_model));
                                    leap_second_line = get_leap_second_line(pvt_solver->gps_cnav_utc_model);
                                    d_rinex_header_gps_updated = true;
                                }
                        }
                    if (!d_rinex_header_galileo_updated && pvt_solver->galileo_utc_model.A0 != 0)
                        {
                            nav_header_info.emplace_back("GAL", "IONOSPHERIC CORR", get_galileo_iono_alpha_line(pvt_solver->galileo_iono));
                            nav_header_info.emplace_back("GAUT", "TIME SYSTEM CORR", get_galileo_time_corr_line(pvt_solver->galileo_utc_model));
                            nav_header_info.emplace_back("GPGA", "TIME SYSTEM CORR", get_gps_to_galileo_time_corr_line(pvt_solver->galileo_utc_model));
                            d_rinex_header_galileo_updated = true;

                            if (system_time_str == "GAL")
                                {
                                    leap_second_line = get_leap_second_line(pvt_solver->galileo_utc_model);
                                }
                        }
                    if (!d_rinex_header_glonass_updated && pvt_solver->glonass_gnav_utc_model.d_tau_c != 0)
                        {
                            if (d_version == 3)  // TODO handle version 2
                                {
                                    nav_header_info.emplace_back("GLUT", "TIME SYSTEM CORR", get_glonass_time_corr_line(pvt_solver->glonass_gnav_utc_model));
                                    nav_header_info.emplace_back("GLGP", "TIME SYSTEM CORR", get_glonass_to_gps_time_corr_line(pvt_solver->glonass_gnav_utc_model));
                                }
                            d_rinex_header_glonass_updated = true;
                        }
                    if (!d_rinex_header_beidou_updated && pvt_solver->beidou_dnav_utc_model.A0_UTC != 0)
                        {
                            nav_header_info.emplace_back("BDSA", "IONOSPHERIC CORR", get_beidou_iono_alpha_line(pvt_solver->beidou_dnav_iono));
                            nav_header_info.emplace_back("BDSB", "IONOSPHERIC CORR", get_beidou_iono_beta_line(pvt_solver->beidou_dnav_iono));
                            nav_header_info.emplace_back("BDUT", "TIME SYSTEM CORR", get_beidou_time_corr_line(pvt_solver->beidou_dnav_utc_model));
                            d_rinex_header_beidou_updated = true;

                            if (system_time_str == "BDS")
                                {
                                    leap_second_line = get_leap_second_line(pvt_solver->beidou_dnav_utc_model);
                                }
                        }

                    if (nav_header_info.empty())
                        {
                            return;
                        }

                    if (!leap_second_line.empty())
                        {
                            update_obs_header(obsFile, leap_second_line);
                            nav_header_info.emplace_back("", "LEAP SECONDS", leap_second_line);
                        }

                    update_nav_header_from_info(navFile, navfilename, nav_header_info);
                    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info.\n";
                    d_rinex_header_updated = d_rinex_header_gps_updated && d_rinex_header_galileo_updated && d_rinex_header_glonass_updated && d_rinex_header_beidou_updated;
                }
        }
}


void Rinex_Printer::log_rinex_nav_gps_nav(const std::map<int32_t, Gps_Ephemeris>& new_eph)
{
    std::string line;
    auto& out = navFile;
    const auto& sys_char = satelliteSystem.at("GPS");

    for (const auto& gps_ephemeris_iter : new_eph)
        {
            const auto& eph = gps_ephemeris_iter.second;

            // -------- SV / EPOCH / SV CLK
            const boost::posix_time::ptime p_utc_time = Rinex_Printer::compute_GPS_time(eph, eph.toc);

            if (d_version == 2)
                {
                    line += rightJustify(std::to_string(eph.PRN), 2);
                    line += std::string(1, ' ');
                    line += get_datetime_v2(p_utc_time);
                    line += std::string(1, ' ');
                    line += doub2for(eph.af0, 18, 2);
                    line += std::string(1, ' ');
                    line += doub2for(eph.af1, 18, 2);
                    line += std::string(1, ' ');
                    line += doub2for(eph.af2, 18, 2);
                    line += std::string(1, ' ');
                    lengthCheck(line);
                    out << line << '\n';
                    line.clear();
                }
            if (d_version == 3)
                {
                    out << get_nav_sv_epoch_svclk_line(p_utc_time, sys_char, eph.PRN, eph.af0, eph.af1, eph.af2) << '\n';
                }

            // -------- BROADCAST ORBIT - 1
            const bool discontinued_reception = eph.IODE_SF2 != eph.IODE_SF3;
            const double* p_value0 = nullptr;
            double iode_d = 0.0;
            if (!discontinued_reception)
                {
                    iode_d = static_cast<double>(eph.IODE_SF2);
                    p_value0 = &iode_d;
                }
            out << get_nav_broadcast_orbit(p_value0, &eph.Crs, &eph.delta_n, &eph.M_0, d_version) << '\n';

            // If there is a discontinued reception the ephemeris is not validated
            if (discontinued_reception)
                {
                    LOG(WARNING) << "Discontinued reception of Frame 2 and 3";
                }

            // -------- BROADCAST ORBIT - 2
            out << get_nav_broadcast_orbit(&eph.Cuc, &eph.ecc, &eph.Cus, &eph.sqrtA, d_version) << '\n';

            // -------- BROADCAST ORBIT - 3
            const auto toe_d = static_cast<double>(eph.toe);
            out << get_nav_broadcast_orbit(&toe_d, &eph.Cic, &eph.OMEGA_0, &eph.Cis, d_version) << '\n';

            // -------- BROADCAST ORBIT - 4
            out << get_nav_broadcast_orbit(&eph.i_0, &eph.Crc, &eph.omega, &eph.OMEGAdot, d_version) << '\n';

            // -------- BROADCAST ORBIT - 5

            double GPS_week_continuous_number;
            if (d_pre_2009_file == false)
                {
                    if (eph.WN < 512)
                        {
                            GPS_week_continuous_number = static_cast<double>(eph.WN + 2048);  // valid until 2029
                        }
                    else
                        {
                            GPS_week_continuous_number = static_cast<double>(eph.WN + 1024);  // valid until April 7, 2019
                        }
                }
            else
                {
                    GPS_week_continuous_number = static_cast<double>(eph.WN + 1024);
                }
            // This week goes with Toe. This is different from the GPS week in the original satellite message!
            if (eph.toe < 7200.0)
                {
                    GPS_week_continuous_number += 1.0;
                }

            auto aux1 = static_cast<double>(eph.code_on_L2);

            out << get_nav_broadcast_orbit(&eph.idot, &aux1, &GPS_week_continuous_number, &aux1, d_version) << '\n';

            // -------- BROADCAST ORBIT - 6
            aux1 = static_cast<double>(eph.SV_accuracy);
            const auto aux2 = static_cast<double>(eph.SV_health);
            const auto aux3 = static_cast<double>(eph.IODC);
            out << get_nav_broadcast_orbit(&aux1, &aux2, &eph.TGD, &aux3, d_version) << '\n';

            // -------- BROADCAST ORBIT - 7
            double tx_time_of_message = eph.tow;
            if (eph.toe < 7200.0)
                {
                    tx_time_of_message -= 604800.0;  // see RINEX 3.03 section 6.13
                }

            int curve_fit_interval = 4;

            if (eph.satelliteBlock.at(eph.PRN) == "IIA")
                {
                    // Block II/IIA (Table 20-XI IS-GPS-200M)
                    if ((eph.IODC > 239) && (eph.IODC < 248))
                        {
                            curve_fit_interval = 8;
                        }
                    if (((eph.IODC > 247) && (eph.IODC < 256)) || (eph.IODC == 496))
                        {
                            curve_fit_interval = 14;
                        }
                    if ((eph.IODC > 496) && (eph.IODC < 504))
                        {
                            curve_fit_interval = 26;
                        }
                    if ((eph.IODC > 503) && (eph.IODC < 511))
                        {
                            curve_fit_interval = 50;
                        }
                    if (((eph.IODC > 751) && (eph.IODC < 757)) || (eph.IODC == 511))
                        {
                            curve_fit_interval = 74;
                        }
                    if (eph.IODC == 757)
                        {
                            curve_fit_interval = 98;
                        }
                }

            if ((eph.satelliteBlock.at(eph.PRN) == "IIR") ||
                (eph.satelliteBlock.at(eph.PRN) == "IIR-M") ||
                (eph.satelliteBlock.at(eph.PRN) == "IIF") ||
                (eph.satelliteBlock.at(eph.PRN) == "III"))
                {
                    // Block IIR/IIR-M/IIF/III/IIIF (Table 20-XII IS-GPS-200M)
                    if ((eph.IODC > 239) && (eph.IODC < 248))
                        {
                            curve_fit_interval = 8;
                        }
                    if (((eph.IODC > 247) && (eph.IODC < 256)) || (eph.IODC == 496))
                        {
                            curve_fit_interval = 14;
                        }
                    if (((eph.IODC > 496) && (eph.IODC < 504)) || ((eph.IODC > 1020) && (eph.IODC < 1024)))
                        {
                            curve_fit_interval = 26;
                        }
                }
            const double fit_d = (curve_fit_interval == 4 ? 0.0 : 1.0);
            out << get_nav_broadcast_orbit(&tx_time_of_message, &fit_d, nullptr, nullptr, d_version) << '\n';
        }
}


void Rinex_Printer::log_rinex_nav_gps_cnav(const std::map<int32_t, Gps_CNAV_Ephemeris>& new_cnav_eph)
{
    auto& out = navFile;
    const auto& sys_char = satelliteSystem.at("GPS");

    for (const auto& gps_ephemeris_iter : new_cnav_eph)
        {
            const auto& eph = gps_ephemeris_iter.second;

            // -------- SV / EPOCH / SV CLK
            const boost::posix_time::ptime p_utc_time = Rinex_Printer::compute_GPS_time(eph, eph.toc);
            out << get_nav_sv_epoch_svclk_line(p_utc_time, sys_char, eph.PRN, eph.af0, eph.af1, eph.af2) << '\n';

            // -------- BROADCAST ORBIT - 1

            // If there is no IODE in CNAV, so we check if Toe in message Type 10, Toe in Message type 11 and Toc in message types 30-37.
            // Whenever these three terms do not match, a data set cutover has occurred and new data must be collected.
            // See IS-GPS-200M, paragraph 20.3.3.4.1
            if (!((eph.toe1 == eph.toe2) && (eph.toe1 == eph.toc)))  // Toe1: Toe in message type 10,  Toe2: Toe in message type 11
                {
                    // Toe1: Toe in message type 10,  Toe2: Toe in message type 11,
                    d_fake_cnav_iode = d_fake_cnav_iode + 1;
                    if (d_fake_cnav_iode == 240)
                        {
                            d_fake_cnav_iode = 1;
                        }
                }

            out << get_nav_broadcast_orbit(&d_fake_cnav_iode, &eph.Crs, &eph.delta_n, &eph.M_0) << '\n';

            // -------- BROADCAST ORBIT - 2
            out << get_nav_broadcast_orbit(&eph.Cuc, &eph.ecc, &eph.Cus, &eph.sqrtA) << '\n';

            // -------- BROADCAST ORBIT - 3
            double max_d = std::max(eph.toe1, eph.toe2);
            out << get_nav_broadcast_orbit(&max_d, &eph.Cic, &eph.OMEGA_0, &eph.Cis) << '\n';

            // -------- BROADCAST ORBIT - 4
            out << get_nav_broadcast_orbit(&eph.i_0, &eph.Crc, &eph.omega, &eph.OMEGAdot) << '\n';

            // -------- BROADCAST ORBIT - 5

            auto GPS_week_continuous_number = static_cast<double>(eph.WN);
            // This week goes with Toe. This is different from the GPS week in the original satellite message!
            if (eph.toe1 < 7200.0)
                {
                    GPS_week_continuous_number += 1.0;
                }
            const double zero = 0.0;
            out << get_nav_broadcast_orbit(&eph.idot, &zero, &GPS_week_continuous_number, &zero) << '\n';  // No data flag for L2 P code

            // -------- BROADCAST ORBIT - 6
            const auto ura_d = static_cast<double>(eph.URA);
            const auto health_d = static_cast<double>(eph.signal_health);
            out << get_nav_broadcast_orbit(&ura_d, &health_d, &eph.TGD, &d_fake_cnav_iode) << '\n';  // no IODC in CNAV, so we fake it (see above)

            // -------- BROADCAST ORBIT - 7
            const auto tow_d = static_cast<double>(eph.tow);
            out << get_nav_broadcast_orbit(&tow_d, &zero, nullptr, nullptr) << '\n';  // ?? Curve fit interval not defined in CNAV
        }
}


void Rinex_Printer::log_rinex_nav_gal_nav(const std::map<int32_t, Galileo_Ephemeris>& new_gal_eph)
{
    auto& out = navFile;
    const auto& sys_char = satelliteSystem.at("Galileo");

    for (const auto& galileo_ephemeris_iter : new_gal_eph)
        {
            const auto& eph = galileo_ephemeris_iter.second;

            // -------- SV / EPOCH / SV CLK
            const boost::posix_time::ptime p_utc_time = Rinex_Printer::compute_Galileo_time(eph, eph.toe);
            out << get_nav_sv_epoch_svclk_line(p_utc_time, sys_char, eph.PRN, eph.af0, eph.af1, eph.af2) << '\n';

            // -------- BROADCAST ORBIT - 1
            const auto iod_d = static_cast<double>(eph.IOD_ephemeris);
            out << get_nav_broadcast_orbit(&iod_d, &eph.Crs, &eph.delta_n, &eph.M_0) << '\n';

            // -------- BROADCAST ORBIT - 2
            out << get_nav_broadcast_orbit(&eph.Cuc, &eph.ecc, &eph.Cus, &eph.sqrtA) << '\n';

            // -------- BROADCAST ORBIT - 3
            const auto toe_d = static_cast<double>(eph.toe);
            out << get_nav_broadcast_orbit(&toe_d, &eph.Cic, &eph.OMEGA_0, &eph.Cis) << '\n';

            // -------- BROADCAST ORBIT - 4
            out << get_nav_broadcast_orbit(&eph.i_0, &eph.Crc, &eph.omega, &eph.OMEGAdot) << '\n';

            // -------- BROADCAST ORBIT - 5
            const std::string iNAVE1B("1000000001");
            const auto data_source_INAV = static_cast<double>(toInt(iNAVE1B, 10));

            const auto GST_week = static_cast<double>(eph.WN);
            const double num_GST_rollovers = floor((GST_week + 1024.0) / 4096.0);
            const double Galileo_week_continuous_number = GST_week + 1024.0 + num_GST_rollovers * 4096.0;
            const double zero = 0.0;
            out << get_nav_broadcast_orbit(&eph.idot, &data_source_INAV, &Galileo_week_continuous_number, &zero) << '\n';

            // -------- BROADCAST ORBIT - 6
            std::string E1B_HS = std::bitset<2>(eph.E1B_HS).to_string();
            std::string E5B_HS = std::bitset<2>(eph.E5b_HS).to_string();

            std::string E1B_DVS = std::to_string(eph.E1B_DVS);
            const std::string SVhealth_str = std::move(E5B_HS) + std::to_string(eph.E5b_DVS) + "11" + "1" + std::move(E1B_DVS) + std::move(E1B_HS) + std::to_string(eph.E1B_DVS);
            const auto SVhealth = static_cast<double>(toInt(SVhealth_str, 9));
            const auto SISA_d = static_cast<double>(eph.SISA);
            out << get_nav_broadcast_orbit(&SISA_d, &SVhealth, &eph.BGD_E1E5a, &eph.BGD_E1E5b) << '\n';

            // -------- BROADCAST ORBIT - 7
            const auto tow_d = static_cast<double>(eph.tow);
            out << get_nav_broadcast_orbit(&tow_d, &zero, nullptr, nullptr) << '\n';
        }
}


void Rinex_Printer::log_rinex_nav_glo_gnav(const std::map<int32_t, Glonass_Gnav_Ephemeris>& new_glo_eph)
{
    std::string line;
    auto& out = (d_version == 2 && navfilename != navGlofilename) ? navGloFile : navFile;
    const auto& sys_char = satelliteSystem.at("GLONASS");

    for (const auto& glonass_gnav_ephemeris_iter : new_glo_eph)
        {
            const auto& eph = glonass_gnav_ephemeris_iter.second;

            // -------- SV / EPOCH / SV CLK
            const boost::posix_time::ptime p_utc_time = eph.glot_to_utc(eph.d_t_b, 0.0);

            if (d_version == 2)
                {
                    line += rightJustify(std::to_string(eph.PRN), 2);
                    line += std::string(1, ' ');
                    line += get_datetime_v2(p_utc_time);
                    line += std::string(1, ' ');
                    line += doub2for(-eph.d_tau_c, 18, 2);
                    line += std::string(1, ' ');
                    line += doub2for(eph.d_gamma_n, 18, 2);
                    line += std::string(1, ' ');
                    line += doub2for(eph.d_t_k, 18, 2);
                    line += std::string(1, ' ');
                    lengthCheck(line);
                    out << line << '\n';
                }
            if (d_version == 3)
                {
                    out << get_nav_sv_epoch_svclk_line(p_utc_time, sys_char, eph.PRN, -eph.d_tau_n, +eph.d_gamma_n, eph.d_t_k + p_utc_time.date().day_of_week() * 86400) << '\n';
                }
            line.clear();

            // -------- BROADCAST ORBIT - 1
            out << get_nav_broadcast_orbit(&eph.d_Xn, &eph.d_VXn, &eph.d_AXn, &eph.d_B_n, d_version) << '\n';

            // -------- BROADCAST ORBIT - 2
            const auto freq_channel_d = static_cast<double>(eph.i_satellite_freq_channel);
            out << get_nav_broadcast_orbit(&eph.d_Yn, &eph.d_VYn, &eph.d_AYn, &freq_channel_d, d_version) << '\n';

            // -------- BROADCAST ORBIT - 3
            out << get_nav_broadcast_orbit(&eph.d_Zn, &eph.d_VZn, &eph.d_AZn, &eph.d_E_n, d_version) << '\n';
        }
}


void Rinex_Printer::log_rinex_nav_bds_dnav(const std::map<int32_t, Beidou_Dnav_Ephemeris>& new_bds_eph)
{
    auto& out = navFile;
    const auto& sys_char = satelliteSystem.at("Beidou");

    for (const auto& bds_ephemeris_iter : new_bds_eph)
        {
            const auto& eph = bds_ephemeris_iter.second;

            // -------- SV / EPOCH / SV CLK
            const boost::posix_time::ptime p_utc_time = Rinex_Printer::compute_BDS_time(eph, eph.toc);
            out << get_nav_sv_epoch_svclk_line(p_utc_time, sys_char, eph.PRN, eph.af0, eph.af1, eph.af2) << '\n';

            // -------- BROADCAST ORBIT - 1
            out << get_nav_broadcast_orbit(&eph.AODE, &eph.Crs, &eph.delta_n, &eph.M_0) << '\n';

            // -------- BROADCAST ORBIT - 2
            out << get_nav_broadcast_orbit(&eph.Cuc, &eph.ecc, &eph.Cus, &eph.sqrtA) << '\n';

            // -------- BROADCAST ORBIT - 3
            const auto toe_d = static_cast<double>(eph.toe);
            out << get_nav_broadcast_orbit(&toe_d, &eph.Cic, &eph.OMEGA_0, &eph.Cis) << '\n';

            // -------- BROADCAST ORBIT - 4
            out << get_nav_broadcast_orbit(&eph.i_0, &eph.Crc, &eph.omega, &eph.OMEGAdot) << '\n';

            // -------- BROADCAST ORBIT - 5
            const auto wn_d = static_cast<double>(eph.WN);
            out << get_nav_broadcast_orbit(&eph.idot, nullptr, &wn_d, nullptr) << '\n';

            // -------- BROADCAST ORBIT - 6
            const auto acc_d = static_cast<double>(eph.SV_accuracy);
            const auto health_d = static_cast<double>(eph.SV_health);
            out << get_nav_broadcast_orbit(&acc_d, &health_d, &eph.TGD1, &eph.TGD2) << '\n';

            // -------- BROADCAST ORBIT - 7
            const auto tow_d = static_cast<double>(eph.tow);
            out << get_nav_broadcast_orbit(&tow_d, &eph.AODC, nullptr, nullptr) << '\n';
        }
}


void Rinex_Printer::rinex_nav_header(std::fstream& out,
    const std::vector<std::string>& iono_lines,
    const std::vector<std::string>& time_corr_lines,
    const std::string& leap_second_line) const
{
    std::string type;
    std::string constellation;

    if (d_flags.only_gps)
        {
            type = "G: GPS";
            constellation = "GPS";
        }
    else if (d_flags.only_galileo)
        {
            type = "E: GALILEO";
            constellation = "GALILEO";
        }
    else if (d_flags.only_glonass)
        {
            type = "R: GLONASS";
            constellation = "GLONASS";
        }
    else if (d_flags.only_beidou)
        {
            type = "C: BEIDOU";
            constellation = "BEIDOU";
        }
    else
        {
            type = "M: MIXED";
            constellation = "GNSS";
        }

    add_navigation_header_start(out, type, constellation, d_version, d_stringVersion);

    for (const auto& iono_line : iono_lines)
        {
            out << iono_line << '\n';
        }

    for (const auto& time_corr_line : time_corr_lines)
        {
            out << time_corr_line << '\n';
        }

    if (!leap_second_line.empty())
        {
            out << leap_second_line << '\n';
        }

    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out,
    const std::string& time_constellation,
    const boost::posix_time::ptime& system_time,
    double seconds)
{
    std::string constellation_legend;

    if (d_version == 2)
        {
            constellation_legend = "BLANK OR G = GPS,  R = GLONASS,  E = GALILEO,  M = MIXED";
        }
    else
        {
            constellation_legend = "G = GPS  R = GLONASS  E = GALILEO  C = BEIDOU  M = MIXED";
        }

    std::string type;
    std::string header_constellation;

    if (d_flags.only_gps)
        {
            type = "GPS";
            header_constellation = "GPS";
        }
    else if (d_flags.only_galileo)
        {
            type = "Galileo";
            header_constellation = "GALILEO";
        }
    else if (d_flags.only_glonass)
        {
            type = "GLONASS";
            header_constellation = "GLONASS";
        }
    else if (d_flags.only_beidou)
        {
            type = "Beidou";
            header_constellation = "BEIDOU";
        }
    else
        {
            type = "Mixed";
            header_constellation = "MIXED";

            std::vector<std::string> constellations;

            if (d_flags.has_gps)
                {
                    constellations.emplace_back("GPS");
                }
            if (d_flags.has_galileo)
                {
                    constellations.emplace_back("GAL");
                }
            if (d_flags.has_glonass)
                {
                    constellations.emplace_back("GLO");
                }
            if (d_flags.has_beidou)
                {
                    constellations.emplace_back("BDS");
                }

            header_constellation += " (";

            for (size_t i = 0; i < constellations.size(); ++i)
                {
                    if (i > 0)
                        {
                            header_constellation += "/";
                        }

                    header_constellation += constellations[i];
                }

            header_constellation += ")";
        }

    add_observation_header_start(out, type, header_constellation, get_local_time(d_version), d_stringVersion, constellation_legend);

    if (d_version == 2)
        {
            // --------- WAVELENGTH FACTOR
            add_wavelength(out);

            // -------- SYS / OBS TYPES
            // Note: These two are basically the same, and we don't want to add two lines
            if (d_flags.has_gps)
                {
                    add_obs_sys_obs_type_v2(out, "GPS_L1_CA_v2", observationType, observationCode);
                }
            else if (d_flags.has_glonass)
                {
                    add_obs_sys_obs_type_v2(out, "GLONASS_G1_CA_v2", observationType, observationCode);
                }
        }
    else
        {
            // -------- SYS / OBS TYPES
            if (d_flags.has_gps)
                {
                    add_obs_sys_obs_type_gps(out, d_flags, observationType, observationCode);
                }
            if (d_flags.has_galileo)
                {
                    add_obs_sys_obs_type_galileo(out, d_flags, observationType, observationCode);
                }
            if (d_flags.has_glonass)
                {
                    add_obs_sys_obs_type_glonass(out, d_flags, observationType, observationCode);
                }
            if (d_flags.has_beidou)
                {
                    add_obs_sys_obs_type_beidou(out, d_flags, observationType, observationCode);
                }

            // -------- Signal Strength units
            add_obs_signal_strength(out);
        }

    // -------- TIME OF FIRST OBS
    add_obs_time_first_obs(out, time_constellation, system_time, seconds);

    // -------- SYS /PHASE SHIFTS

    if (d_version == 3 && d_flags.has_glonass)
        {
            // -------- GLONASS SLOT / FRQ #
            add_obs_glonass_slot_freq(out);

            // -------- GLONASS CODE/PHS/BIS
            add_obs_glonass_code_phase_bias(out, observationType, observationCode);
        }

    // -------- end of header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::update_obs_header(std::fstream& out, const std::string& leap_second_line) const
{
    out.seekp(0);

    std::vector<std::string> data;
    bool no_more_finds = false;
    std::string line_str;

    while (!out.eof())
        {
            std::getline(out, line_str);

            if (!no_more_finds && line_str.find("END OF HEADER", 59) != std::string::npos)
                {
                    data.push_back(leap_second_line);
                    no_more_finds = true;
                }

            data.push_back(line_str);
        }

    override_stream_with_new_data(out, obsfilename, data, 0);
}


void Rinex_Printer::to_date_time(int32_t gps_week, int32_t gps_tow, int& year, int& month, int& day, int& hour, int& minute, int& second) const
{
    // represents GPS time (week, TOW) in the date time format of the Gregorian calendar.
    // -> Leap years are considered, but leap seconds are not.
    const std::array<int32_t, 12> days_per_month{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    // seconds in a not leap year
    const int32_t secs_per_day = 24 * 60 * 60;
    const int32_t secs_per_week = 7 * secs_per_day;
    const int32_t secs_per_normal_year = 365 * secs_per_day;
    const int32_t secs_per_leap_year = secs_per_normal_year + secs_per_day;

    // the GPS epoch is 06.01.1980 00:00, i.e. midnight 5. / 6. January 1980
    // -> seconds since then
    const int32_t secs_since_gps_epoch = gps_week * secs_per_week + gps_tow;

    // find year, consider leap years
    bool is_leap_year;
    int32_t remaining_secs = secs_since_gps_epoch + 5 * secs_per_day;
    for (int32_t y = 1980; true; y++)
        {
            is_leap_year = y % 4 == 0 && (y % 100 != 0 || y % 400 == 0);
            int32_t secs_in_year_y = is_leap_year ? secs_per_leap_year : secs_per_normal_year;

            if (secs_in_year_y <= remaining_secs)
                {
                    remaining_secs -= secs_in_year_y;
                }
            else
                {
                    year = y;
                    // std::cout << "year: year=" << year << " secs_in_year_y="<< secs_in_year_y << " remaining_secs="<< remaining_secs << '\n';
                    break;
                }
            // std::cout << "year: y=" << y << " secs_in_year_y="<< secs_in_year_y << " remaining_secs="<< remaining_secs << '\n';
        }

    // find month
    for (int32_t m = 1; true; m++)
        {
            int32_t secs_in_month_m = days_per_month[m - 1] * secs_per_day;
            if (is_leap_year && m == 2)  // consider February of leap year
                {
                    secs_in_month_m += secs_per_day;
                }

            if (secs_in_month_m <= remaining_secs)
                {
                    remaining_secs -= secs_in_month_m;
                }
            else
                {
                    month = m;
                    // std::cout << "month: month=" << month << " secs_in_month_m="<< secs_in_month_m << " remaining_secs="<< remaining_secs << '\n';
                    break;
                }
            // std::cout << "month: m=" << m << " secs_in_month_m="<< secs_in_month_m << " remaining_secs="<< remaining_secs << '\n';
        }

    day = remaining_secs / secs_per_day + 1;
    remaining_secs = remaining_secs % secs_per_day;

    hour = remaining_secs / (60 * 60);
    remaining_secs = remaining_secs % (60 * 60);

    minute = remaining_secs / 60;
    second = remaining_secs % 60;
}


boost::posix_time::ptime Rinex_Printer::compute_UTC_time(const Gps_Navigation_Message& nav_msg) const
{
    // if we are processing a file -> wait to leap second to resolve the ambiguity else take the week from the local system time
    // idea: resolve the ambiguity with the leap second
    const double utc_t = nav_msg.utc_time(nav_msg.get_TOW());
    const boost::posix_time::time_duration t = boost::posix_time::milliseconds(static_cast<int64_t>((utc_t + 604800 * static_cast<double>(nav_msg.get_GPS_week())) * 1000));
    // Handle week rollover
    if (d_pre_2009_file == false)
        {
            // Handle week rollover (valid from 2009 to 2029)
            if (nav_msg.get_GPS_week() < 512)
                {
                    boost::posix_time::ptime p_time(boost::gregorian::date(2019, 4, 7), t);
                    return p_time;
                }
            boost::posix_time::ptime p_time(boost::gregorian::date(1999, 8, 22), t);
            return p_time;
        }
    else
        {
            // assume receiver operating in between 1999 to 2008
            boost::posix_time::ptime p_time(boost::gregorian::date(1999, 8, 22), t);
            return p_time;
        }
}


boost::posix_time::ptime Rinex_Printer::compute_BDS_time(const Beidou_Dnav_Ephemeris& eph, double obs_time) const
{
    // The RINEX v2.11 v3.00 format uses GPS time for the observations epoch, not UTC time, thus, no leap seconds needed here.
    // (see Section 3 in ftp://igs.org/pub/data/format/rinex211.txt)
    // (see Pag. 17 in ftp://igs.org/pub/data/format/rinex300.pdf)
    // --??? No time correction here, since it will be done in the RINEX processor
    const double bds_t = obs_time;
    const boost::posix_time::time_duration t = boost::posix_time::milliseconds(static_cast<int64_t>((bds_t + 604800 * static_cast<double>(eph.WN % 8192)) * 1000));
    boost::posix_time::ptime p_time(boost::gregorian::date(2006, 1, 1), t);
    return p_time;
}


boost::posix_time::ptime Rinex_Printer::compute_GPS_time(const Gps_Ephemeris& eph, double obs_time) const
{
    // The RINEX v2.11 v3.00 format uses GPS time for the observations epoch, not UTC time, thus, no leap seconds needed here.
    // (see Section 3 in ftp://igs.org/pub/data/format/rinex211.txt)
    // (see Pag. 17 in ftp://igs.org/pub/data/format/rinex300.pdf)
    // No time correction here, since it will be done in the PVT processor
    boost::posix_time::time_duration t = boost::posix_time::milliseconds(static_cast<int64_t>((obs_time + 604800 * static_cast<double>(eph.WN % 1024)) * 1000));
    // Handle TOW rollover
    if (obs_time < 18.0)
        {
            t += boost::posix_time::seconds(604800);
        }

    // Handle week rollover
    if (d_pre_2009_file == false)
        {
            // Handle week rollover (valid from 2009 to 2029)
            if (eph.WN < 512)
                {
                    boost::posix_time::ptime p_time(boost::gregorian::date(2019, 4, 7), t);
                    return p_time;
                }
            boost::posix_time::ptime p_time(boost::gregorian::date(1999, 8, 22), t);
            return p_time;
        }
    else
        {
            // assume receiver operating in between 1999 to 2008
            boost::posix_time::ptime p_time(boost::gregorian::date(1999, 8, 22), t);
            return p_time;
        }
}


boost::posix_time::ptime Rinex_Printer::compute_GPS_time(const Gps_CNAV_Ephemeris& eph, double obs_time) const
{
    // The RINEX v2.11 v3.00 format uses GPS time for the observations epoch, not UTC time, thus, no leap seconds needed here.
    // (see Section 3 in ftp://igs.org/pub/data/format/rinex211.txt)
    // (see Pag. 17 in ftp://igs.org/pub/data/format/rinex300.pdf)
    // --??? No time correction here, since it will be done in the RINEX processor
    const boost::posix_time::time_duration t = boost::posix_time::milliseconds(static_cast<int64_t>((obs_time + 604800 * static_cast<double>(eph.WN % 8192)) * 1000));
    boost::posix_time::ptime p_time(boost::gregorian::date(1980, 1, 6), t);
    return p_time;
}


boost::posix_time::ptime Rinex_Printer::compute_Galileo_time(const Galileo_Ephemeris& eph, double obs_time) const
{
    // The RINEX v2.11 v3.00 format uses Galileo time for the observations epoch, not UTC time, thus, no leap seconds needed here.
    // (see Pag. 17 in ftp://igs.org/pub/data/format/rinex301.pdf)
    // --??? No time correction here, since it will be done in the RINEX processor
    const double galileo_t = obs_time;
    const boost::posix_time::time_duration t = boost::posix_time::milliseconds(static_cast<int64_t>((galileo_t + 604800 * static_cast<double>(eph.WN)) * 1000));  //
    boost::posix_time::ptime p_time(boost::gregorian::date(1999, 8, 22), t);
    return p_time;
}


boost::posix_time::ptime Rinex_Printer::compute_UTC_time(const Glonass_Gnav_Ephemeris& eph, double obs_time) const
{
    double tod = 0.0;
    const double glot2utc = 3 * 3600;
    double obs_time_glot = 0.0;
    int32_t i = 0;
    int J = 0;
    if (eph.d_N_T >= 1 && eph.d_N_T <= 366)
        {
            J = 1;
        }
    else if (eph.d_N_T >= 367 && eph.d_N_T <= 731)
        {
            J = 2;
        }
    else if (eph.d_N_T >= 732 && eph.d_N_T <= 1096)
        {
            J = 3;
        }
    else if (eph.d_N_T >= 1097 && eph.d_N_T <= 1461)
        {
            J = 4;
        }

    // Get observation time in nearly GLONASS time. Correction for leap seconds done at the end
    obs_time_glot = obs_time + glot2utc;

    // Get seconds of day in glonass time
    tod = fmod(obs_time_glot, 86400);

    // Form date and time duration types
    const boost::posix_time::time_duration t1(0, 0, tod);
    const boost::gregorian::date d1(eph.d_yr - J + 1.0, 1, 1);
    const boost::gregorian::days d2(eph.d_N_T - 1);
    const boost::posix_time::ptime glo_time(d1 + d2, t1);

    // Convert to utc
    const boost::posix_time::time_duration t2(0, 0, glot2utc);
    boost::posix_time::ptime utc_time = glo_time - t2;

    // Adjust for leap second correction
    for (i = 0; GLONASS_LEAP_SECONDS[i][0] > 0; i++)
        {
            const boost::posix_time::time_duration t3(GLONASS_LEAP_SECONDS[i][3], GLONASS_LEAP_SECONDS[i][4], GLONASS_LEAP_SECONDS[i][5]);
            const boost::gregorian::date d3(GLONASS_LEAP_SECONDS[i][0], GLONASS_LEAP_SECONDS[i][1], GLONASS_LEAP_SECONDS[i][2]);
            const boost::posix_time::ptime ls_time(d3, t3);
            if (utc_time >= ls_time)
                {
                    // We subtract the leap second when going from gpst to utc, values store as negatives
                    utc_time = utc_time + boost::posix_time::time_duration(0, 0, GLONASS_LEAP_SECONDS[i][6]);
                    break;
                }
        }

    return utc_time;
}


/*

enum RINEX_enumMarkerType {
    GEODETIC,      //!< GEODETIC Earth-fixed, high-precision monumentation
    NON_GEODETIC,  //!< NON_GEODETIC Earth-fixed, low-precision monumentation
    SPACEBORNE,    //!< SPACEBORNE Orbiting space vehicle
    AIRBORNE ,     //!< AIRBORNE Aircraft, balloon, etc.
    WATER_CRAFT,   //!< WATER_CRAFT Mobile water craft
    GROUND_CRAFT,  //!< GROUND_CRAFT Mobile terrestrial vehicle
    FIXED_BUOY,    //!< FIXED_BUOY "Fixed" on water surface
    FLOATING_BUOY, //!< FLOATING_BUOY Floating on water surface
    FLOATING_ICE,  //!< FLOATING_ICE Floating ice sheet, etc.
    GLACIER,       //!< GLACIER "Fixed" on a glacier
    BALLISTIC,     //!< BALLISTIC Rockets, shells, etc
    ANIMAL,        //!< ANIMAL Animal carrying a receiver
    HUMAN          //!< HUMAN Human being
};

 */
