/*!
 * \file rinex_printer.cc
 * \brief Implementation of a RINEX 2.11 / 3.02 printer
 * See ftp://igs.org/pub/data/format/rinex302.pdf
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
#include "signal_enabled_flags.h"
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
#include <vector>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

namespace
{
const std::unordered_map<std::string, std::string> satelliteSystem = {
    {"GPS", "G"},
    {"GLONASS", "R"},
    {"SBAS payload", "S"},
    {"Galileo", "E"},
    {"Beidou", "C"},
    {"Mixed", "M"}};  // RINEX v3.02 codes


std::string enabled_signal_flags_to_string(const Signal_Enabled_Flags& flags)
{
    std::vector<std::string> signal_str_vector;

    if (flags.check_any_enabled(GPS_1C))
        {
            signal_str_vector.emplace_back("1C");
        }
    if (flags.check_any_enabled(GPS_2S))
        {
            signal_str_vector.emplace_back("2S");
        }
    if (flags.check_any_enabled(GPS_L5))
        {
            signal_str_vector.emplace_back("L5");
        }
    if (flags.check_any_enabled(GAL_1B))
        {
            signal_str_vector.emplace_back("1B");
        }
    if (flags.check_any_enabled(GAL_E5a))
        {
            signal_str_vector.emplace_back("5X");
        }
    if (flags.check_any_enabled(GAL_E5b))
        {
            signal_str_vector.emplace_back("7X");
        }
    if (flags.check_any_enabled(GAL_E6))
        {
            signal_str_vector.emplace_back("E6");
        }
    if (flags.check_any_enabled(GLO_1G))
        {
            signal_str_vector.emplace_back("1G");
        }
    if (flags.check_any_enabled(GLO_2G))
        {
            signal_str_vector.emplace_back("2G");
        }
    if (flags.check_any_enabled(BDS_B1))
        {
            signal_str_vector.emplace_back("B1");
        }
    if (flags.check_any_enabled(BDS_B3))
        {
            signal_str_vector.emplace_back("B3");
        }

    std::ostringstream oss;

    for (size_t i = 0; i < signal_str_vector.size(); ++i)
        {
            oss << signal_str_vector[i];
            if (i != signal_str_vector.size() - 1)
                {
                    oss << ' ';
                }
        }

    return oss.str();
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


void add_marker_type(std::fstream& out, const std::string& marker_type, const std::string& marker_type_id)
{
    // -------- Line MARKER TYPE
    std::string line;
    line += leftJustify(marker_type, 60);  // put a flag or a property
    line += leftJustify("MARKER " + marker_type_id, 20);
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


void add_navigation_header_start(std::fstream& out, const std::string& type, const std::string& constellation, const std::string& local_time, const std::string& version)
{
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += version;
    line += std::string(11, ' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4, ' ');
    line += type;
    line += std::string(20 - type.size(), ' ');
    line += std::string("RINEX VERSION / TYPE");
    lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    add_run_by_line(out, local_time);

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
    const std::string& constellation_legend,
    const std::string& marker_type = "",
    const std::string& marker_type_id = "")
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

    // -------- Line MARKER TYPE
    if (!marker_type.empty())
        {
            add_marker_type(out, marker_type, marker_type_id);
        }

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


std::string get_leap_second_line_v2(const Gps_Utc_Model& utc_model)
{
    std::string line;
    line += rightJustify(std::to_string(utc_model.DeltaT_LS), 6);
    line += std::string(54, ' ');
    line += leftJustify("LEAP SECONDS", 20);
    return line;
}


std::string get_leap_second_line(const Gps_Utc_Model& utc_model)
{
    return get_leap_second_line(utc_model.DeltaT_LS, utc_model.DeltaT_LSF, utc_model.WN_LSF, utc_model.DN);
}

std::string get_leap_second_line(const Gps_Utc_Model& utc_model, int32_t version)
{
    if (version == 2)
        {
            return get_leap_second_line_v2(utc_model);
        }
    return get_leap_second_line(utc_model);
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

void add_svclk_to_line(const boost::posix_time::ptime& p_utc_time, bool log_seconds, std::string& line)
{
    const std::string timestring = boost::posix_time::to_iso_string(p_utc_time);
    const std::string year(timestring, 0, 4);
    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);
    const std::string seconds(timestring, 13, 2);

    line += std::string(1, ' ');
    line += year;
    line += std::string(1, ' ');
    line += month;
    line += std::string(1, ' ');
    line += day;
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

void add_seconds_to_line(double seconds, std::string& line)
{
    // Add extra 0 if seconds are < 10
    if (seconds < 10)
        {
            line += std::string(1, '0');
        }
    line += asString(seconds, 7);
    line += std::string(2, ' ');
}

std::string get_nav_sv_epoch_svclk_line(const boost::posix_time::ptime& p_utc_time, const std::string& sys_char, uint32_t prn, double value0, double value1, double value2)
{
    std::string line;

    if (!sys_char.empty())
        {
            line += sys_char;
        }
    if (prn < 10)
        {
            line += std::string("0");
        }

    line += std::to_string(prn);

    add_svclk_to_line(p_utc_time, true, line);

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


void add_obs_time_first_obs(std::fstream& out, const std::string& constellation, const boost::posix_time::ptime& p_gps_time, double tow)
{
    const std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    const std::string year(timestring, 0, 4);
    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);
    const double seconds = fmod(tow, 60);
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


void add_obs_sys_obs_type(std::fstream& out,
    const std::string& sys_char,
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


void add_obs_sys_obs_type(std::fstream& out,
    const std::string& constellation,
    const std::string& bands,
    const std::map<std::string, std::string>& observationType,
    const std::map<std::string, std::string>& observationCode,
    const std::map<std::string, std::string>& band_to_code_map)
{
    std::vector<std::string> obsCodes;
    uint32_t number_of_observations = 0;

    for (const auto& it : band_to_code_map)
        {
            if (bands.find(it.first) != std::string::npos)
                {
                    obsCodes.emplace_back(observationCode.at(it.second));
                    number_of_observations += 4;
                }
        }

    const auto& sys_char = satelliteSystem.at(constellation);
    add_obs_sys_obs_type(out, sys_char, number_of_observations, observationType, obsCodes);
}


void add_obs_sys_obs_type_gps(std::fstream& out,
    const std::string& bands,
    const std::map<std::string, std::string>& observationType,
    const std::map<std::string, std::string>& observationCode)
{
    const std::map<std::string, std::string> band_to_code_map = {
        {"1C", "GPS_L1_CA"},
        {"2S", "GPS_L2_L2CM"},
        {"L5", "GPS_L5_Q"},
    };

    add_obs_sys_obs_type(out, "GPS", bands, observationType, observationCode, band_to_code_map);
}


void add_obs_sys_obs_type_galileo(std::fstream& out,
    const std::string& bands,
    const std::map<std::string, std::string>& observationType,
    const std::map<std::string, std::string>& observationCode)
{
    const std::map<std::string, std::string> band_to_code_map = {
        {"1B", "GALILEO_E1_B"},
        {"5X", "GALILEO_E5a_IQ"},
        {"7X", "GALILEO_E5b_IQ"},
        {"E6", "GALILEO_E56_B"},
    };

    add_obs_sys_obs_type(out, "Galileo", bands, observationType, observationCode, band_to_code_map);
}


void add_obs_sys_obs_type_glonass(std::fstream& out,
    const std::string& bands,
    const std::map<std::string, std::string>& observationType,
    const std::map<std::string, std::string>& observationCode)
{
    const std::map<std::string, std::string> band_to_code_map = {
        {"1G", "GLONASS_G1_CA"},
        {"2G", "GLONASS_G2_CA"},
    };

    add_obs_sys_obs_type(out, "GLONASS", bands, observationType, observationCode, band_to_code_map);
}


void add_obs_sys_obs_type_beidou(std::fstream& out,
    const std::string& bands,
    const std::map<std::string, std::string>& observationType,
    const std::map<std::string, std::string>& observationCode)
{
    const std::map<std::string, std::string> band_to_code_map = {
        {"B1", "BEIDOU_B1_I"},
        {"B3", "BEIDOU_B3_I"},
    };

    add_obs_sys_obs_type(out, "Beidou", bands, observationType, observationCode, band_to_code_map);
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

}  // namespace


Rinex_Printer::Rinex_Printer(int version,
    const std::string& base_path,
    const std::string& base_name) : Rinex_Printer(base_name, getAndCreateBaseRinexPath(base_path), version)
{
}


Rinex_Printer::Rinex_Printer(const std::string& base_name,
    const std::string& base_rinex_path,
    int version) : observationType(getObservationTypes()),
                   observationCode(getObservationCodes()),
                   navfilename(getFilePath("RINEX_FILE_TYPE_GPS_NAV", base_name, base_rinex_path)),
                   obsfilename(getFilePath("RINEX_FILE_TYPE_OBS", base_name, base_rinex_path)),
                   sbsfilename(getFilePath("RINEX_FILE_TYPE_SBAS", base_name, base_rinex_path)),
                   navGalfilename(getFilePath("RINEX_FILE_TYPE_GAL_NAV", base_name, base_rinex_path)),
                   navGlofilename(getFilePath("RINEX_FILE_TYPE_GLO_NAV", base_name, base_rinex_path)),
                   navBdsfilename(getFilePath("RINEX_FILE_TYPE_BDS_NAV", base_name, base_rinex_path)),
                   navMixfilename(getFilePath("RINEX_FILE_TYPE_MIXED_NAV", base_name, base_rinex_path)),
                   d_fake_cnav_iode(1),
                   d_rinex_header_updated(false),
                   d_rinex_header_written(false),
                   d_pre_2009_file(false)

{
    std::map<std::string, std::fstream&> fileMap = {
        {navfilename, navFile},
        {obsfilename, obsFile},
        {sbsfilename, sbsFile},
        {navGalfilename, navGalFile},
        {navMixfilename, navMixFile},
        {navGlofilename, navGloFile},
        {navBdsfilename, navBdsFile},
    };

    bool all_open = true;

    for (auto& it : fileMap)
        {
            it.second.open(it.first, std::ios::out | std::ios::in | std::ios::app);
            all_open = all_open && it.second.is_open();
        }

    if (!all_open)
        {
            std::cout << "RINEX files cannot be saved. Wrong permissions?\n";
        }

    if (version == 2)
        {
            d_version = 2;
            d_stringVersion = "2.11";
        }
    else
        {
            d_version = 3;
            d_stringVersion = "3.02";
        }
}


Rinex_Printer::~Rinex_Printer()
{
    DLOG(INFO) << "RINEX printer destructor called.";

    std::map<std::string, std::fstream&> fileMap = {
        {navfilename, navFile},
        {obsfilename, obsFile},
        {sbsfilename, sbsFile},
        {navGalfilename, navGalFile},
        {navMixfilename, navMixFile},
        {navGlofilename, navGloFile},
        {navBdsfilename, navBdsFile},
    };

    std::map<std::string, decltype(navFile.tellp())> filePosMap;

    for (const auto& it : fileMap)
        {
            filePosMap[it.first] = it.second.tellp();
        }

    try
        {
            for (auto& it : fileMap)
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
    uint32_t signal_enabled_flags,
    bool flag_write_RINEX_obs_output)
{
    const auto galileo_ephemeris_iter = pvt_solver->galileo_ephemeris_map.cbegin();
    const auto gps_ephemeris_iter = pvt_solver->gps_ephemeris_map.cbegin();
    const auto gps_cnav_ephemeris_iter = pvt_solver->gps_cnav_ephemeris_map.cbegin();
    const auto glonass_gnav_ephemeris_iter = pvt_solver->glonass_gnav_ephemeris_map.cbegin();
    const auto beidou_dnav_ephemeris_iter = pvt_solver->beidou_dnav_ephemeris_map.cbegin();

    const Signal_Enabled_Flags flags(signal_enabled_flags);
    const auto signal = enabled_signal_flags_to_string(flags);
    const auto has_gps = flags.check_any_enabled(GPS_1C, GPS_2S, GPS_L5);
    const auto has_galileo = flags.check_any_enabled(GAL_1B, GAL_E5a, GAL_E5b, GAL_E6);
    const auto has_glonass = flags.check_any_enabled(GLO_1G, GLO_2G);
    const auto has_beidou = flags.check_any_enabled(BDS_B1, BDS_B3);
    const auto only_galileo = has_galileo && !(has_gps || has_glonass || has_beidou);
    const auto only_glonass = has_glonass && !(has_gps || has_galileo || has_beidou);
    const auto only_beidou = has_beidou && !(has_gps || has_galileo || has_glonass);

    const bool has_gps_lnav_eph = !pvt_solver->gps_ephemeris_map.empty();
    const bool has_gps_cnav_eph = !pvt_solver->gps_cnav_ephemeris_map.empty();
    const bool has_galileo_eph = !pvt_solver->galileo_ephemeris_map.empty();
    const bool has_glonass_eph = !pvt_solver->glonass_gnav_ephemeris_map.empty();
    const bool has_beidou_dnav_eph = !pvt_solver->beidou_dnav_ephemeris_map.empty();

    if (!d_rinex_header_written)  // & we have utc data in nav message!
        {
            bool rinex_header_written = true;

            if (flags.check_only_enabled(GPS_1C) && has_gps_lnav_eph)
                {
                    rinex_obs_header(obsFile, gps_ephemeris_iter->second, rx_time);
                    rinex_nav_header(navFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second);
                    output_navfilename.push_back(navfilename);
                    log_rinex_nav(navFile, pvt_solver->gps_ephemeris_map);
                }
            else if ((flags.check_only_enabled(GPS_2S) || flags.check_only_enabled(GPS_L5)) && has_gps_cnav_eph)
                {
                    rinex_obs_header(obsFile, gps_cnav_ephemeris_iter->second, rx_time, signal);
                    rinex_nav_header(navFile, pvt_solver->gps_cnav_iono, pvt_solver->gps_cnav_utc_model);
                    output_navfilename.push_back(navfilename);
                    log_rinex_nav(navFile, pvt_solver->gps_cnav_ephemeris_map);
                }
            else if (only_galileo && has_galileo_eph)
                {
                    rinex_obs_header(obsFile, galileo_ephemeris_iter->second, rx_time, signal);
                    rinex_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                    output_navfilename.push_back(navGalfilename);
                    log_rinex_nav(navGalFile, pvt_solver->galileo_ephemeris_map);
                }
            else if (only_glonass && has_glonass_eph)
                {
                    rinex_obs_header(obsFile, glonass_gnav_ephemeris_iter->second, rx_time, signal);
                    rinex_nav_header(navGloFile, pvt_solver->glonass_gnav_utc_model, glonass_gnav_ephemeris_iter->second);
                    output_navfilename.push_back(navGlofilename);
                    log_rinex_nav(navGloFile, pvt_solver->glonass_gnav_ephemeris_map);
                }
            else if (only_beidou && has_beidou_dnav_eph)
                {
                    rinex_obs_header(obsFile, beidou_dnav_ephemeris_iter->second, rx_time, signal);
                    rinex_nav_header(navBdsFile, pvt_solver->beidou_dnav_iono, pvt_solver->beidou_dnav_utc_model);
                    output_navfilename.push_back(navBdsfilename);
                    log_rinex_nav(navBdsFile, pvt_solver->beidou_dnav_ephemeris_map);
                }
            else if ((flags.check_only_enabled(GPS_1C, GPS_2S) ||
                         flags.check_only_enabled(GPS_1C, GPS_L5) ||
                         flags.check_only_enabled(GPS_1C, GPS_2S, GPS_L5)) &&
                     has_gps_lnav_eph && has_gps_cnav_eph)
                {
                    rinex_obs_header(obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, rx_time, signal);
                    rinex_nav_header(navFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second);
                    output_navfilename.push_back(navfilename);

                    if (flags.check_any_enabled(GPS_L5))
                        {
                            log_rinex_nav(navFile, pvt_solver->gps_ephemeris_map);
                        }
                    else
                        {
                            log_rinex_nav(navFile, pvt_solver->gps_cnav_ephemeris_map);
                        }
                }
            else if ((flags.check_only_enabled(GPS_1C, GAL_1B) || flags.check_only_enabled(GPS_1C, GAL_E5a) || flags.check_only_enabled(GPS_1C, GAL_E5b)) &&
                     has_gps_lnav_eph && has_galileo_eph)
                {
                    rinex_obs_header(obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, signal);
                    rinex_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                    output_navfilename.push_back(navMixfilename);
                    log_rinex_nav(navMixFile, pvt_solver->gps_ephemeris_map, pvt_solver->galileo_ephemeris_map);
                }
            else if (flags.check_only_enabled(GPS_L5, GAL_E5a) &&
                     has_gps_cnav_eph && has_galileo_eph)
                {
                    rinex_obs_header(obsFile, gps_cnav_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, signal, signal);
                    rinex_nav_header(navMixFile, pvt_solver->gps_cnav_iono, pvt_solver->gps_cnav_utc_model, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                    output_navfilename.push_back(navMixfilename);
                    log_rinex_nav(navMixFile, pvt_solver->gps_cnav_ephemeris_map, pvt_solver->galileo_ephemeris_map);
                }
            else if ((flags.check_only_enabled(GPS_1C, GLO_1G) || flags.check_only_enabled(GPS_1C, GLO_2G)) &&
                     has_gps_lnav_eph && has_glonass_eph)
                {
                    rinex_obs_header(obsFile, gps_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, rx_time, signal);
                    if (d_version == 3)
                        {
                            rinex_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->glonass_gnav_utc_model);
                            output_navfilename.push_back(navMixfilename);
                            log_rinex_nav(navMixFile, pvt_solver->gps_ephemeris_map, pvt_solver->glonass_gnav_ephemeris_map);
                        }
                    if (d_version == 2)
                        {
                            rinex_nav_header(navFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second);
                            rinex_nav_header(navGloFile, pvt_solver->glonass_gnav_utc_model, glonass_gnav_ephemeris_iter->second);
                            output_navfilename.push_back(navfilename);
                            output_navfilename.push_back(navGlofilename);
                            log_rinex_nav(navFile, pvt_solver->gps_ephemeris_map);
                            log_rinex_nav(navGloFile, pvt_solver->glonass_gnav_ephemeris_map);
                        }
                }
            else if ((flags.check_only_enabled(GLO_1G, GPS_2S) || flags.check_only_enabled(GLO_2G, GPS_2S)) &&
                     has_gps_cnav_eph && has_glonass_eph)
                {
                    rinex_obs_header(obsFile, gps_cnav_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, rx_time, signal);
                    rinex_nav_header(navMixFile, pvt_solver->gps_cnav_iono, pvt_solver->gps_cnav_utc_model, pvt_solver->glonass_gnav_utc_model);
                    output_navfilename.push_back(navfilename);
                    log_rinex_nav(navMixFile, pvt_solver->gps_cnav_ephemeris_map, pvt_solver->glonass_gnav_ephemeris_map);
                }
            else if ((flags.check_only_enabled(GAL_1B, GLO_1G) || flags.check_only_enabled(GAL_1B, GLO_2G)) &&
                     has_galileo_eph && has_glonass_eph)
                {
                    rinex_obs_header(obsFile, galileo_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, rx_time, signal, signal);
                    rinex_nav_header(navMixFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model, pvt_solver->glonass_gnav_utc_model);
                    output_navfilename.push_back(navMixfilename);
                    log_rinex_nav(navMixFile, pvt_solver->galileo_ephemeris_map, pvt_solver->glonass_gnav_ephemeris_map);
                }
            else if ((flags.check_only_enabled(GPS_1C, GAL_1B, GPS_L5, GAL_E5a) ||
                         flags.check_only_enabled(GPS_1C, GAL_1B, GPS_L5, GAL_E5a, GAL_E6) ||
                         flags.check_only_enabled(GPS_1C, GAL_1B, GPS_2S, GPS_L5, GAL_E5a)) &&
                     has_gps_lnav_eph && has_gps_cnav_eph && has_galileo_eph)
                {
                    rinex_obs_header(obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, signal, signal);
                    rinex_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                    output_navfilename.push_back(navMixfilename);
                    log_rinex_nav(navMixFile, pvt_solver->gps_ephemeris_map, pvt_solver->galileo_ephemeris_map);
                }
            else if ((flags.check_only_enabled(GPS_1C, GAL_1B, GAL_E5a) || flags.check_only_enabled(GPS_1C, GAL_1B, GAL_E5b)) && has_gps_lnav_eph && has_galileo_eph)
                {
                    rinex_obs_header(obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, signal);
                    rinex_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                    output_navfilename.push_back(navMixfilename);
                    log_rinex_nav(navMixFile, pvt_solver->gps_ephemeris_map, pvt_solver->galileo_ephemeris_map);
                }
            else if (flags.check_only_enabled(GPS_1C, GAL_E6) && has_gps_lnav_eph)
                {
                    if (has_galileo_eph)
                        {
                            // we have Galileo ephemeris, maybe from assistance
                            rinex_obs_header(obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, signal);
                            rinex_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navMixfilename);
                            log_rinex_nav(navMixFile, pvt_solver->gps_ephemeris_map, pvt_solver->galileo_ephemeris_map);
                        }
                    else
                        {
                            // we do not have galileo ephemeris, print only GPS data
                            rinex_obs_header(obsFile, gps_ephemeris_iter->second, rx_time);
                            rinex_nav_header(navFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second);
                            output_navfilename.push_back(navfilename);
                            log_rinex_nav(navFile, pvt_solver->gps_ephemeris_map);
                        }
                }
            else if (has_beidou && has_beidou_dnav_eph)
                {
                    rinex_obs_header(obsFile, beidou_dnav_ephemeris_iter->second, rx_time, signal);
                    // Not implemented for beidou
                }
            else
                {
                    rinex_header_written = false;
                }

            d_rinex_header_written = rinex_header_written;
        }

    if (d_rinex_header_written && flag_write_RINEX_obs_output)  // The header is already written, we can now log the navigation message data
        {
            if (flags.check_only_enabled(GPS_1C) && has_gps_lnav_eph)
                {
                    log_rinex_obs(obsFile, gps_ephemeris_iter->second, rx_time, gnss_observables_map);
                    if (!d_rinex_header_updated && (pvt_solver->gps_utc_model.A0 != 0))
                        {
                            update_obs_header(obsFile, get_leap_second_line(pvt_solver->gps_utc_model, d_version));
                            update_nav_header(navFile, pvt_solver->gps_utc_model, pvt_solver->gps_iono, gps_ephemeris_iter->second);
                            d_rinex_header_updated = true;
                        }
                }
            else if (flags.check_only_enabled(GPS_1C, GPS_2S) && has_gps_lnav_eph && has_gps_cnav_eph)
                {
                    log_rinex_obs(obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, rx_time, gnss_observables_map);
                    if (!d_rinex_header_updated && (pvt_solver->gps_utc_model.A0 != 0))
                        {
                            update_obs_header(obsFile, get_leap_second_line(pvt_solver->gps_utc_model, d_version));
                            update_nav_header(navFile, pvt_solver->gps_utc_model, pvt_solver->gps_iono, gps_ephemeris_iter->second);
                            d_rinex_header_updated = true;
                        }
                }
            else if (flags.check_only_enabled(GPS_1C, GPS_L5) && has_gps_lnav_eph && has_gps_cnav_eph)
                {
                    log_rinex_obs(obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, rx_time, gnss_observables_map);
                    if (!d_rinex_header_updated && ((pvt_solver->gps_cnav_utc_model.A0 != 0) || (pvt_solver->gps_utc_model.A0 != 0)))
                        {
                            if (pvt_solver->gps_cnav_utc_model.A0 != 0)
                                {
                                    update_obs_header(obsFile, get_leap_second_line(pvt_solver->gps_cnav_utc_model));
                                    update_nav_header(navFile, pvt_solver->gps_cnav_utc_model, pvt_solver->gps_cnav_iono);
                                }
                            else
                                {
                                    update_obs_header(obsFile, get_leap_second_line(pvt_solver->gps_utc_model, d_version));
                                    update_nav_header(navFile, pvt_solver->gps_utc_model, pvt_solver->gps_iono, gps_ephemeris_iter->second);
                                }
                            d_rinex_header_updated = true;
                        }
                }
            else if (flags.check_only_enabled(GPS_2S) || flags.check_only_enabled(GPS_L5) || flags.check_only_enabled(GPS_2S, GPS_L5))
                {
                    if (has_gps_cnav_eph)
                        {
                            log_rinex_obs(obsFile, gps_cnav_ephemeris_iter->second, rx_time, gnss_observables_map);
                        }
                    if (!d_rinex_header_updated && (pvt_solver->gps_cnav_utc_model.A0 != 0))
                        {
                            update_obs_header(obsFile, get_leap_second_line(pvt_solver->gps_cnav_utc_model));
                            update_nav_header(navFile, pvt_solver->gps_cnav_utc_model, pvt_solver->gps_cnav_iono);
                            d_rinex_header_updated = true;
                        }
                }
            else if (flags.check_only_enabled(GPS_1C, GPS_2S, GPS_L5))
                {
                    if (has_gps_lnav_eph && has_gps_cnav_eph)
                        {
                            log_rinex_obs(obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, rx_time, gnss_observables_map, true);
                        }
                    if (!d_rinex_header_updated && (pvt_solver->gps_utc_model.A0 != 0) && (has_gps_lnav_eph))
                        {
                            update_obs_header(obsFile, get_leap_second_line(pvt_solver->gps_utc_model, d_version));
                            update_nav_header(navFile, pvt_solver->gps_utc_model, pvt_solver->gps_iono, gps_ephemeris_iter->second);
                            d_rinex_header_updated = true;
                        }
                }
            else if (only_galileo)
                {
                    if (has_galileo_eph)
                        {
                            log_rinex_obs(obsFile, galileo_ephemeris_iter->second, rx_time, gnss_observables_map, signal);
                        }
                    if (!d_rinex_header_updated && (pvt_solver->galileo_utc_model.A0 != 0))
                        {
                            update_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            update_obs_header(obsFile, get_leap_second_line(pvt_solver->galileo_utc_model));
                            d_rinex_header_updated = true;
                        }
                }
            else if (only_glonass)
                {
                    if (has_glonass_eph)
                        {
                            log_rinex_obs(obsFile, glonass_gnav_ephemeris_iter->second, rx_time, gnss_observables_map);
                        }
                    if (!d_rinex_header_updated && (pvt_solver->glonass_gnav_utc_model.d_tau_c != 0))
                        {
                            update_nav_header(navGloFile, pvt_solver->glonass_gnav_utc_model);
                            d_rinex_header_updated = true;
                        }
                }
            else if (only_beidou)
                {
                    if (has_beidou_dnav_eph)
                        {
                            log_rinex_obs(obsFile, beidou_dnav_ephemeris_iter->second, rx_time, gnss_observables_map, signal);
                        }
                    if (!d_rinex_header_updated && (pvt_solver->beidou_dnav_utc_model.A0_UTC != 0))
                        {
                            update_obs_header(obsFile, get_leap_second_line(pvt_solver->beidou_dnav_utc_model));
                            update_nav_header(navBdsFile, pvt_solver->beidou_dnav_utc_model, pvt_solver->beidou_dnav_iono);
                            d_rinex_header_updated = true;
                        }
                }
            else if ((flags.check_only_enabled(GPS_1C, GAL_1B) || flags.check_only_enabled(GPS_1C, GAL_1B, GAL_E6)) &&
                     has_gps_lnav_eph && has_galileo_eph)
                {
                    log_rinex_obs(obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gnss_observables_map);
                    if (!d_rinex_header_updated && (pvt_solver->gps_utc_model.A0 != 0))
                        {
                            update_obs_header(obsFile, get_leap_second_line(pvt_solver->gps_utc_model, d_version));
                            update_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            d_rinex_header_updated = true;
                        }
                }
            else if ((flags.check_only_enabled(GPS_1C, GLO_1G) || flags.check_only_enabled(GPS_1C, GLO_2G)) &&
                     has_gps_lnav_eph && has_glonass_eph)
                {
                    log_rinex_obs(obsFile, gps_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, rx_time, gnss_observables_map);
                    if (!d_rinex_header_updated && (pvt_solver->gps_utc_model.A0 != 0))
                        {
                            update_obs_header(obsFile, get_leap_second_line(pvt_solver->gps_utc_model, d_version));
                            update_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->glonass_gnav_utc_model);
                            d_rinex_header_updated = true;  // do not write header anymore
                        }
                }
            else if (flags.check_only_enabled(GPS_L5, GAL_E5a))
                {
                    if ((has_gps_cnav_eph) && (has_galileo_eph))
                        {
                            log_rinex_obs(obsFile, gps_cnav_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gnss_observables_map);
                        }
                    if (!d_rinex_header_updated && (pvt_solver->gps_cnav_utc_model.A0 != 0) && (pvt_solver->galileo_utc_model.A0 != 0))
                        {
                            update_obs_header(obsFile, get_leap_second_line(pvt_solver->gps_cnav_utc_model));
                            update_nav_header(navMixFile, pvt_solver->gps_cnav_utc_model, pvt_solver->gps_cnav_iono, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            d_rinex_header_updated = true;  // do not write header anymore
                        }
                }
            else if (flags.check_only_enabled(GAL_1B, GLO_1G) || flags.check_only_enabled(GAL_1B, GLO_2G))
                {
                    if ((has_glonass_eph) && (has_galileo_eph))
                        {
                            log_rinex_obs(obsFile, galileo_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, rx_time, gnss_observables_map);
                        }
                    if (!d_rinex_header_updated && (pvt_solver->galileo_utc_model.A0 != 0))
                        {
                            update_obs_header(obsFile, get_leap_second_line(pvt_solver->galileo_utc_model));
                            update_nav_header(navMixFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model, pvt_solver->glonass_gnav_utc_model);
                            d_rinex_header_updated = true;  // do not write header anymore
                        }
                }
            else if (flags.check_only_enabled(GPS_2S, GLO_1G) || flags.check_only_enabled(GPS_2S, GLO_2G))
                {
                    if ((has_glonass_eph) && (has_gps_cnav_eph))
                        {
                            log_rinex_obs(obsFile, gps_cnav_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, rx_time, gnss_observables_map);
                        }
                    if (!d_rinex_header_updated && (pvt_solver->gps_cnav_utc_model.A0 != 0))
                        {
                            update_obs_header(obsFile, get_leap_second_line(pvt_solver->gps_cnav_utc_model));
                            update_nav_header(navMixFile, pvt_solver->gps_cnav_iono, pvt_solver->gps_cnav_utc_model, pvt_solver->glonass_gnav_utc_model);
                            d_rinex_header_updated = true;  // do not write header anymore
                        }
                }
            else if (flags.check_only_enabled(GPS_1C, GAL_1B, GPS_L5, GAL_E5a))
                {
                    if (has_gps_lnav_eph && has_gps_cnav_eph && has_galileo_eph)
                        {
                            log_rinex_obs(obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gnss_observables_map);
                            if (!d_rinex_header_updated && ((pvt_solver->gps_cnav_utc_model.A0 != 0) || (pvt_solver->gps_utc_model.A0 != 0)) && (pvt_solver->galileo_utc_model.A0 != 0))
                                {
                                    if (pvt_solver->gps_cnav_utc_model.A0 != 0)
                                        {
                                            update_obs_header(obsFile, get_leap_second_line(pvt_solver->gps_cnav_utc_model));
                                            update_nav_header(navMixFile, pvt_solver->gps_cnav_utc_model, pvt_solver->gps_cnav_iono, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                        }
                                    else
                                        {
                                            update_obs_header(obsFile, get_leap_second_line(pvt_solver->gps_utc_model, d_version));
                                            update_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                        }
                                    d_rinex_header_updated = true;  // do not write header anymore
                                }
                        }
                }
            else if (flags.check_only_enabled(GPS_1C, GAL_1B, GAL_E5a))
                {
                    if (has_gps_lnav_eph && has_galileo_eph)
                        {
                            log_rinex_obs(obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gnss_observables_map);
                            if (!d_rinex_header_updated && (pvt_solver->gps_utc_model.A0 != 0) && (pvt_solver->galileo_utc_model.A0 != 0))
                                {
                                    update_obs_header(obsFile, get_leap_second_line(pvt_solver->gps_utc_model, d_version));
                                    update_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                    d_rinex_header_updated = true;  // do not write header anymore
                                }
                        }
                }
            else if (flags.check_only_enabled(GPS_1C, GAL_1B, GPS_L5, GAL_E5a, GAL_E6) || flags.check_only_enabled(GPS_1C, GAL_1B, GPS_2S, GPS_L5, GAL_E5a))
                {
                    if (has_galileo_eph && has_gps_lnav_eph && has_gps_cnav_eph)
                        {
                            log_rinex_obs(obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gnss_observables_map, true);
                        }
                    if (!d_rinex_header_updated && (pvt_solver->gps_utc_model.A0 != 0) && (pvt_solver->galileo_utc_model.A0 != 0) && (has_gps_lnav_eph))
                        {
                            update_obs_header(obsFile, get_leap_second_line(pvt_solver->gps_utc_model, d_version));
                            update_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            d_rinex_header_updated = true;
                        }
                }
            else if (flags.check_only_enabled(GPS_1C, GAL_E6) && has_gps_lnav_eph)
                {
                    if (has_galileo_eph)
                        {
                            // we have Galileo ephemeris, maybe from assistance
                            log_rinex_obs(obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gnss_observables_map);
                            if (!d_rinex_header_updated && (pvt_solver->gps_utc_model.A0 != 0))
                                {
                                    update_obs_header(obsFile, get_leap_second_line(pvt_solver->gps_utc_model, d_version));
                                    update_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                    d_rinex_header_updated = true;
                                }
                        }
                    else
                        {
                            // we do not have galileo ephemeris, print only GPS data
                            log_rinex_obs(obsFile, gps_ephemeris_iter->second, rx_time, gnss_observables_map);
                            if (!d_rinex_header_updated && (pvt_solver->gps_utc_model.A0 != 0))
                                {
                                    update_obs_header(obsFile, get_leap_second_line(pvt_solver->gps_utc_model, d_version));
                                    update_nav_header(navFile, pvt_solver->gps_utc_model, pvt_solver->gps_iono, gps_ephemeris_iter->second);
                                    d_rinex_header_updated = true;
                                }
                        }
                }
        }
}


void Rinex_Printer::log_rinex_nav_gps_nav(uint32_t signal_enabled_flags, const std::map<int32_t, Gps_Ephemeris>& new_eph)
{
    std::map<int32_t, Galileo_Ephemeris> new_gal_eph;
    std::map<int32_t, Glonass_Gnav_Ephemeris> new_glo_eph;
    const Signal_Enabled_Flags flags(signal_enabled_flags);

    if (!flags.check_any_enabled(GPS_1C))
        {
            return;
        }

    if (flags.check_any_enabled(GAL_1B, GAL_E5a, GAL_E5b, GAL_E6))
        {
            if (flags.check_any_enabled(GAL_E6) && navMixFile.tellp() == 0)
                {
                    log_rinex_nav(navFile, new_eph);
                }
            else
                {
                    log_rinex_nav(navMixFile, new_eph, new_gal_eph);
                }
        }
    else if (flags.check_any_enabled(GLO_1G, GLO_2G))
        {
            if (d_version == 3)
                {
                    log_rinex_nav(navMixFile, new_eph, new_glo_eph);
                }
            else if (d_version == 2)
                {
                    if (flags.check_any_enabled(GLO_1G))
                        {
                            log_rinex_nav(navFile, new_glo_eph);
                        }
                    else
                        {
                            log_rinex_nav(navFile, new_eph);
                        }
                }
        }
    else
        {
            log_rinex_nav(navFile, new_eph);
        }
}


void Rinex_Printer::log_rinex_nav_gps_cnav(uint32_t signal_enabled_flags, const std::map<int32_t, Gps_CNAV_Ephemeris>& new_cnav_eph)
{
    std::map<int32_t, Galileo_Ephemeris> new_gal_eph;
    std::map<int32_t, Glonass_Gnav_Ephemeris> new_glo_eph;
    const Signal_Enabled_Flags flags(signal_enabled_flags);

    if (!flags.check_any_enabled(GPS_2S, GPS_L5))
        {
            return;
        }

    if (flags.check_any_enabled(GAL_1B, GAL_E5a, GAL_E5b, GAL_E6))
        {
            log_rinex_nav(navMixFile, new_cnav_eph, new_gal_eph);
        }
    else if (flags.check_any_enabled(GLO_1G, GLO_2G))
        {
            log_rinex_nav(navMixFile, new_cnav_eph, new_glo_eph);
        }
    else
        {
            log_rinex_nav(navFile, new_cnav_eph);
        }
}


void Rinex_Printer::log_rinex_nav_gal_nav(uint32_t signal_enabled_flags, const std::map<int32_t, Galileo_Ephemeris>& new_gal_eph)
{
    std::map<int32_t, Glonass_Gnav_Ephemeris> new_glo_eph;
    std::map<int32_t, Gps_CNAV_Ephemeris> new_cnav_eph;
    std::map<int32_t, Gps_Ephemeris> new_eph;
    const Signal_Enabled_Flags flags(signal_enabled_flags);

    if (!flags.check_any_enabled(GAL_1B, GAL_E5a, GAL_E5b, GAL_E6))
        {
            return;
        }

    if (flags.check_any_enabled(GPS_1C, GPS_2S, GPS_L5))
        {
            if (flags.check_any_enabled(GAL_E6) && navMixFile.tellp() == 0)
                {
                    return;
                }

            if (flags.check_any_enabled(GPS_1C))
                {
                    log_rinex_nav(navMixFile, new_eph, new_gal_eph);
                }
            else
                {
                    log_rinex_nav(navMixFile, new_cnav_eph, new_gal_eph);
                }
        }
    else if (flags.check_any_enabled(GLO_1G, GLO_2G))
        {
            log_rinex_nav(navMixFile, new_gal_eph, new_glo_eph);
        }
    else
        {
            log_rinex_nav(navGalFile, new_gal_eph);
        }
}


void Rinex_Printer::log_rinex_nav_glo_gnav(uint32_t signal_enabled_flags, const std::map<int32_t, Glonass_Gnav_Ephemeris>& new_glo_eph)
{
    std::map<int32_t, Galileo_Ephemeris> new_gal_eph;
    std::map<int32_t, Gps_CNAV_Ephemeris> new_cnav_eph;
    std::map<int32_t, Gps_Ephemeris> new_eph;
    const Signal_Enabled_Flags flags(signal_enabled_flags);

    if (!flags.check_any_enabled(GLO_1G, GLO_2G))
        {
            return;
        }

    if (flags.check_any_enabled(GPS_1C, GPS_2S, GPS_L5))
        {
            if (flags.check_any_enabled(GPS_1C))
                {
                    if (d_version == 3)
                        {
                            log_rinex_nav(navMixFile, new_eph, new_glo_eph);
                        }
                    else if (d_version == 2)
                        {
                            log_rinex_nav(navGloFile, new_glo_eph);
                        }
                }
            else
                {
                    log_rinex_nav(navMixFile, new_cnav_eph, new_glo_eph);
                }
        }

    else if (flags.check_any_enabled(GAL_1B, GAL_E5a, GAL_E5b, GAL_E6))
        {
            log_rinex_nav(navMixFile, new_gal_eph, new_glo_eph);
        }
    else
        {
            log_rinex_nav(navGloFile, new_glo_eph);
        }
}


void Rinex_Printer::log_rinex_nav_bds_dnav(uint32_t signal_enabled_flags, const std::map<int32_t, Beidou_Dnav_Ephemeris>& new_bds_eph)
{
    const Signal_Enabled_Flags flags(signal_enabled_flags);

    if (flags.check_any_enabled(BDS_B1) || flags.check_any_enabled(BDS_B3))
        {
            log_rinex_nav(navBdsFile, new_bds_eph);
        }
}


std::string Rinex_Printer::getLocalTime() const
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

    if (d_version == 2)
        {
            const int32_t day = pt_tm.tm_mday;
            line += rightJustify(std::to_string(day), 2);
            line += std::string("-");

            std::map<int32_t, std::string> months;
            months[0] = "JAN";
            months[1] = "FEB";
            months[2] = "MAR";
            months[3] = "APR";
            months[4] = "MAY";
            months[5] = "JUN";
            months[6] = "JUL";
            months[7] = "AUG";
            months[8] = "SEP";
            months[9] = "OCT";
            months[10] = "NOV";
            months[11] = "DEC";

            line += months[pt_tm.tm_mon];
            line += std::string("-");
            line += std::to_string(pt_tm.tm_year - 100);
            line += std::string(1, ' ');
            line += strmHour.str();
            line += std::string(":");
            line += strmMin.str();
            line += std::string(5, ' ');
        }

    if (d_version == 3)
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


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Ephemeris& glonass_gnav_eph)
{
    add_navigation_header_start(out, "R: GLONASS", "GLONASS", Rinex_Printer::getLocalTime(), d_stringVersion);

    std::string line;

    // -------- Line system time correction
    if (d_version == 3)
        {
            out << get_glonass_time_corr_line(glonass_gnav_utc_model) << '\n';

            // -------- Line system time correction 2
            out << get_glonass_to_gps_time_corr_line(glonass_gnav_utc_model) << '\n';
        }
    if (d_version == 2)
        {
            // Set reference time and its clock corrections
            const boost::posix_time::ptime p_utc_ref_time = glonass_gnav_eph.glot_to_utc(glonass_gnav_eph.d_t_b, 0.0);
            const std::string timestring = boost::posix_time::to_iso_string(p_utc_ref_time);
            const std::string year(timestring, 0, 4);
            const std::string month(timestring, 4, 2);
            const std::string day(timestring, 6, 2);

            line.clear();
            line += rightJustify(year, 6);
            line += rightJustify(month, 6);
            line += rightJustify(day, 6);
            line += std::string(3, ' ');
            line += rightJustify(doub2for(glonass_gnav_utc_model.d_tau_c, 19, 2), 19);
            line += std::string(20, ' ');
            line += leftJustify("CORR TO SYSTEM TIME", 20);
            lengthCheck(line);
            out << line << '\n';
        }

    // -------- End of Header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Gps_Iono& gps_iono, const Gps_Utc_Model& gps_utc_model, const Gps_Ephemeris& eph, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model)
{
    d_stringVersion = "3.02";
    d_version = 3;

    add_navigation_header_start(out, "M: MIXED", "GNSS", Rinex_Printer::getLocalTime(), d_stringVersion);

    // -------- Line ionospheric info 1
    out << get_gps_iono_alpha_line(gps_iono) << '\n';

    // -------- Line system time correction 1
    out << get_glonass_time_corr_line(glonass_gnav_utc_model) << '\n';

    // -------- Line system time correction 2
    out << get_glonass_to_gps_time_corr_line(glonass_gnav_utc_model) << '\n';

    // -------- Line system time correction 3
    out << get_gps_time_corr_line(gps_utc_model, eph, d_pre_2009_file) << '\n';

    // -------- Line 6 leap seconds
    out << get_leap_second_line(gps_utc_model) << '\n';

    // -------- End of Header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Gps_CNAV_Iono& gps_iono, const Gps_CNAV_Utc_Model& gps_utc_model, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model)
{
    d_stringVersion = "3.02";
    d_version = 3;

    add_navigation_header_start(out, "M: MIXED", "GNSS", Rinex_Printer::getLocalTime(), d_stringVersion);

    // -------- Line ionospheric info 1
    out << get_gps_iono_alpha_line(gps_iono) << '\n';

    // -------- Line system time correction 1
    out << get_glonass_time_corr_line(glonass_gnav_utc_model) << '\n';

    // -------- Line system time correction 2
    out << get_glonass_to_gps_time_corr_line(glonass_gnav_utc_model) << '\n';

    // -------- Line system time correction 3
    out << get_gps_time_corr_line(gps_utc_model) << '\n';

    // -------- Line 6 leap seconds
    out << get_leap_second_line(gps_utc_model) << '\n';

    // -------- End of Header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model) const
{
    add_navigation_header_start(out, "M: MIXED", "GNSS", Rinex_Printer::getLocalTime(), d_stringVersion);

    // -------- Line ionospheric info 1
    out << get_galileo_iono_alpha_line(galileo_iono) << '\n';

    // -------- Line system time correction
    out << get_galileo_time_corr_line(galileo_utc_model) << '\n';

    // -------- Line system time correction 1
    out << get_glonass_time_corr_line(glonass_gnav_utc_model) << '\n';

    // -------- Line 6 leap seconds
    out << get_leap_second_line(galileo_utc_model) << '\n';

    // -------- End of Header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Galileo_Iono& iono, const Galileo_Utc_Model& utc_model) const
{
    add_navigation_header_start(out, "E: GALILEO", "GALILEO", Rinex_Printer::getLocalTime(), d_stringVersion);

    // -------- Line ionospheric info 1
    out << get_galileo_iono_alpha_line(iono) << '\n';

    // -------- Line system time correction
    out << get_galileo_time_corr_line(utc_model) << '\n';

    // -------- Line system time correction 2
    out << get_gps_to_galileo_time_corr_line(utc_model) << '\n';

    // -------- Line 6 leap seconds
    out << get_leap_second_line(utc_model) << '\n';

    // -------- End of Header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Gps_CNAV_Iono& iono, const Gps_CNAV_Utc_Model& utc_model) const
{
    add_navigation_header_start(out, "G: GPS", "GPS", Rinex_Printer::getLocalTime(), d_stringVersion);

    // -------- Line ionospheric info 1
    out << get_gps_iono_alpha_line(iono) << '\n';

    // -------- Line ionospheric info 2
    out << get_gps_iono_beta_line(iono) << '\n';

    // -------- Line 5 system time correction
    out << get_gps_time_corr_line(utc_model) << '\n';

    // -------- Line 6 leap seconds
    out << get_leap_second_line(utc_model) << '\n';

    // -------- End of Header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Gps_CNAV_Iono& iono, const Gps_CNAV_Utc_Model& utc_model, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model) const
{
    add_navigation_header_start(out, "M: MIXED", "GNSS", Rinex_Printer::getLocalTime(), d_stringVersion);

    // -------- Line ionospheric info 1
    out << get_galileo_iono_alpha_line(galileo_iono) << '\n';

    // -------- Line ionospheric info 2
    out << get_gps_iono_alpha_line(iono) << '\n';

    // -------- Line ionospheric info 3
    out << get_gps_iono_beta_line(iono) << '\n';

    // -------- Line system time correction
    out << get_galileo_time_corr_line(galileo_utc_model) << '\n';

    // -------- Line system time correction 2
    out << get_gps_time_corr_line(utc_model) << '\n';

    // -------- Line 6 leap seconds
    out << get_leap_second_line(utc_model) << '\n';

    // -------- End of Header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Gps_Iono& iono, const Gps_Utc_Model& utc_model, const Gps_Ephemeris& eph) const
{
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += d_stringVersion;
    line += std::string(11, ' ');

    if (d_version == 2)
        {
            line += std::string("N: GPS NAV DATA");
            line += std::string(25, ' ');
        }

    if (d_version == 3)
        {
            line += std::string("N: GNSS NAV DATA");
            line += std::string(4, ' ');
            // todo Add here other systems...
            line += std::string("G: GPS");
            line += std::string(14, ' ');
            // ...
        }

    line += std::string("RINEX VERSION / TYPE");
    lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    add_run_by_line(out, Rinex_Printer::getLocalTime());

    // -------- Line 3
    add_generated_by_gnss_sdr(out, "GPS", "NAVIGATION MESSAGE");

    // -------- Line COMMENT
    add_gnss_sdr_version(out);

    // -------- Line COMMENT
    add_gnss_sdr_url(out);

    // -------- Line ionospheric info 1
    line.clear();
    if (d_version == 2)
        {
            line += std::string(2, ' ');
            line += rightJustify(doub2for(iono.alpha0, 10, 2), 12);
            line += rightJustify(doub2for(iono.alpha1, 10, 2), 12);
            line += rightJustify(doub2for(iono.alpha2, 10, 2), 12);
            line += rightJustify(doub2for(iono.alpha3, 10, 2), 12);
            line += std::string(10, ' ');
            line += leftJustify("ION ALPHA", 20);
            lengthCheck(line);
            out << line << '\n';
        }
    if (d_version == 3)
        {
            out << get_gps_iono_alpha_line(iono) << '\n';
        }

    // -------- Line ionospheric info 2
    line.clear();
    if (d_version == 2)
        {
            line += std::string(2, ' ');
            line += rightJustify(doub2for(iono.beta0, 10, 2), 12);
            line += rightJustify(doub2for(iono.beta1, 10, 2), 12);
            line += rightJustify(doub2for(iono.beta2, 10, 2), 12);
            line += rightJustify(doub2for(iono.beta3, 10, 2), 12);
            line += std::string(10, ' ');
            line += leftJustify("ION BETA", 20);
            lengthCheck(line);
            out << line << '\n';
        }

    if (d_version == 3)
        {
            out << get_gps_iono_beta_line(iono) << '\n';
        }

    // -------- Line 5 system time correction
    line.clear();
    if (d_version == 2)
        {
            line += std::string(3, ' ');
            line += rightJustify(doub2for(utc_model.A0, 18, 2), 19);
            line += rightJustify(doub2for(utc_model.A1, 18, 2), 19);
            line += rightJustify(std::to_string(utc_model.tot), 9);
            if (d_pre_2009_file == false)
                {
                    if (eph.WN < 512)
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
            out << line << '\n';
        }

    if (d_version == 3)
        {
            out << get_gps_time_corr_line(utc_model, eph, d_pre_2009_file) << '\n';
        }

    // -------- Line 6 leap seconds
    // For leap second information, see https://endruntechnologies.com/support/leap-seconds
    line.clear();
    line += rightJustify(std::to_string(utc_model.DeltaT_LS), 6);
    if (d_version == 2)
        {
            line += std::string(54, ' ');
        }
    if (d_version == 3)
        {
            line += rightJustify(std::to_string(utc_model.DeltaT_LSF), 6);
            line += rightJustify(std::to_string(utc_model.WN_LSF), 6);
            line += rightJustify(std::to_string(utc_model.DN), 6);
            line += std::string(36, ' ');
        }
    line += leftJustify("LEAP SECONDS", 20);
    lengthCheck(line);
    out << line << '\n';

    // -------- End of Header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Gps_Iono& gps_iono, const Gps_Utc_Model& gps_utc_model, const Gps_Ephemeris& eph, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model) const
{
    add_navigation_header_start(out, "M: MIXED", "GNSS", Rinex_Printer::getLocalTime(), d_stringVersion);

    // -------- Line ionospheric info 1
    out << get_galileo_iono_alpha_line(galileo_iono) << '\n';

    // -------- Line ionospheric info 2
    out << get_gps_iono_alpha_line(gps_iono) << '\n';

    // -------- Line system time correction
    out << get_galileo_time_corr_line(galileo_utc_model) << '\n';

    // -------- Line system time correction 2
    out << get_gps_to_galileo_time_corr_line(galileo_utc_model) << '\n';

    // -------- Line system time correction 3
    out << get_gps_time_corr_line(gps_utc_model, eph, d_pre_2009_file) << '\n';

    // -------- Line 6 leap seconds
    out << get_leap_second_line(gps_utc_model) << '\n';

    // -------- End of Header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Beidou_Dnav_Iono& iono, const Beidou_Dnav_Utc_Model& utc_model) const
{
    add_navigation_header_start(out, "F: BDS", "BDS", Rinex_Printer::getLocalTime(), d_stringVersion);  // TODO version handling

    // -------- Line ionospheric info 1, only version 3 supported
    out << get_beidou_iono_alpha_line(iono) << '\n';

    // -------- Line ionospheric info 2
    out << get_beidou_iono_beta_line(iono) << '\n';

    // -------- Line 5 system time correction
    out << get_beidou_time_corr_line(utc_model) << '\n';

    // -------- Line 6 leap seconds
    out << get_leap_second_line(utc_model) << '\n';

    // -------- End of Header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Gps_Iono& gps_iono, const Gps_Utc_Model& gps_utc_model, const Gps_Ephemeris& gps_eph, const Beidou_Dnav_Iono& bds_dnav_iono, const Beidou_Dnav_Utc_Model& bds_dnav_utc_model) const
{
    add_navigation_header_start(out, "M: MIXED", "GNSS", Rinex_Printer::getLocalTime(), d_stringVersion);

    // -------- Line ionospheric info 1, only version 3 supported
    out << get_beidou_iono_alpha_line(bds_dnav_iono) << '\n';

    // -------- Line ionospheric info 2
    out << get_beidou_iono_beta_line(bds_dnav_iono) << '\n';

    // -------- Line ionospheric info 2
    out << get_gps_iono_alpha_line(gps_iono) << '\n';

    // -------- Line 5 system time correction
    out << get_beidou_time_corr_line(bds_dnav_utc_model) << '\n';

    // -------- Line system time correction 3
    out << get_gps_time_corr_line(gps_utc_model, gps_eph, d_pre_2009_file) << '\n';

    // -------- Line 6 leap seconds
    out << get_leap_second_line(gps_utc_model) << '\n';

    // -------- End of Header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Gps_CNAV_Iono& gps_cnav_iono, const Gps_CNAV_Utc_Model& gps_cnav_utc_model, const Beidou_Dnav_Iono& bds_dnav_iono, const Beidou_Dnav_Utc_Model& bds_dnav_utc_model)
{
    d_stringVersion = "3.02";
    d_version = 3;

    add_navigation_header_start(out, "M: MIXED", "GNSS", Rinex_Printer::getLocalTime(), d_stringVersion);

    // -------- Line ionospheric info 1, only version 3 supported
    out << get_beidou_iono_alpha_line(bds_dnav_iono) << '\n';

    // -------- Line ionospheric info 2
    out << get_beidou_iono_beta_line(bds_dnav_iono) << '\n';

    // -------- Line ionospheric info 1
    out << get_gps_iono_alpha_line(gps_cnav_iono) << '\n';

    // -------- Line 5 system time correction
    out << get_beidou_time_corr_line(bds_dnav_utc_model) << '\n';

    // -------- Line system time correction 3
    out << get_gps_time_corr_line(gps_cnav_utc_model) << '\n';

    // -------- Line 6 leap seconds
    out << get_leap_second_line(gps_cnav_utc_model) << '\n';

    // -------- End of Header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Glonass_Gnav_Utc_Model& glo_gnav_utc_model, const Beidou_Dnav_Iono& bds_dnav_iono, const Beidou_Dnav_Utc_Model& bds_dnav_utc_model) const
{
    add_navigation_header_start(out, "M: MIXED", "GNSS", Rinex_Printer::getLocalTime(), d_stringVersion);

    // -------- Line ionospheric info 1, only version 3 supported
    out << get_beidou_iono_alpha_line(bds_dnav_iono) << '\n';

    // -------- Line ionospheric info 2
    out << get_beidou_iono_beta_line(bds_dnav_iono) << '\n';

    // -------- Line 5 system time correction
    out << get_beidou_time_corr_line(bds_dnav_utc_model) << '\n';

    // -------- Line system time correction 1
    out << get_glonass_time_corr_line(glo_gnav_utc_model) << '\n';

    // -------- Line 6 leap seconds
    out << get_leap_second_line(bds_dnav_utc_model) << '\n';

    // -------- End of Header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model, const Beidou_Dnav_Iono& bds_dnav_iono, const Beidou_Dnav_Utc_Model& bds_dnav_utc_model) const
{
    add_navigation_header_start(out, "M: MIXED", "GNSS", Rinex_Printer::getLocalTime(), d_stringVersion);

    // -------- Line ionospheric info 1
    out << get_galileo_iono_alpha_line(galileo_iono) << '\n';

    // -------- Line ionospheric info 1, only version 3 supported
    out << get_beidou_iono_alpha_line(bds_dnav_iono) << '\n';

    // -------- Line ionospheric info 2
    out << get_beidou_iono_beta_line(bds_dnav_iono) << '\n';

    // -------- Line system time correction
    out << get_galileo_time_corr_line(galileo_utc_model) << '\n';

    // -------- Line system time correction 1
    // -------- Line 5 system time correction
    out << get_beidou_time_corr_line(bds_dnav_utc_model) << '\n';

    // -------- Line 6 leap seconds
    out << get_leap_second_line(galileo_utc_model) << '\n';

    // -------- End of Header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_sbs_header(std::fstream& out) const
{
    std::string line;

    // -------- Line 1
    line.clear();
    line = std::string(5, ' ');
    line += std::string("2.10");
    line += std::string(11, ' ');
    line += leftJustify("B SBAS DATA", 20);
    line += std::string(20, ' ');
    line += std::string("RINEX VERSION / TYPE");

    lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += leftJustify("GNSS-SDR", 20);
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
    // Date of file creation (dd-mmm-yy hhmm)
    const boost::local_time::time_zone_ptr zone(new boost::local_time::posix_time_zone("UTC"));
    const boost::local_time::local_date_time pt = boost::local_time::local_sec_clock::local_time(zone);
    const tm pt_tm = boost::local_time::to_tm(pt);
    std::stringstream strYear;
    int32_t utc_year = pt.date().year();
    utc_year -= 2000;  //  two digits for year
    strYear << utc_year;
    std::stringstream strMonth;
    int32_t utc_month = pt.date().month().as_number();
    if (utc_month < 10)
        {
            strMonth << "0";  //  two digits for months
        }
    strMonth << utc_month;
    std::stringstream strmDay;
    int32_t utc_day = pt.date().day().as_number();
    if (utc_day < 10)
        {
            strmDay << "0";  //  two digits for days
        }
    strmDay << utc_day;
    std::stringstream strmHour;
    int32_t utc_hour = pt_tm.tm_hour;
    if (utc_hour < 10)
        {
            strmHour << "0";  //  two digits for hours
        }
    strmHour << utc_hour;
    std::stringstream strmMin;
    int32_t utc_minute = pt_tm.tm_min;
    if (utc_minute < 10)
        {
            strmMin << "0";  //  two digits for minutes
        }
    strmMin << utc_minute;
    std::string time_str;
    time_str += strmDay.str();
    time_str += "-";
    time_str += strMonth.str();
    time_str += "-";
    time_str += strYear.str();
    time_str += " ";
    time_str += strmHour.str();
    time_str += strmMin.str();
    line += leftJustify(time_str, 20);
    line += leftJustify("PGM / RUN BY / DATE", 20);
    lengthCheck(line);
    out << line << '\n';

    // -------- Line 3
    line.clear();
    line += std::string(60, ' ');
    line += leftJustify("REC INDEX/TYPE/VERS", 20);
    lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT 1
    line.clear();
    line += leftJustify("BROADCAST DATA FILE FOR GEO SV, GENERATED BY GNSS-SDR", 60);
    line += leftJustify("COMMENT", 20);
    lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    add_gnss_sdr_version(out);

    // -------- Line COMMENT 2
    add_gnss_sdr_url(out);

    // -------- End of Header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model) const
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
                    if ((line_str.find("GLUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_glonass_time_corr_line(glonass_gnav_utc_model));
                        }
                    else if ((line_str.find("GLGP", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_glonass_to_gps_time_corr_line(glonass_gnav_utc_model));
                        }
                    else if (line_str.find("END OF HEADER", 59) != std::string::npos)
                        {
                            data.push_back(line_str);
                            no_more_finds = true;
                        }
                    else
                        {
                            data.push_back(line_str);
                        }
                }
            else
                {
                    data.push_back(line_str);
                }
        }

    override_stream_with_new_data(out, navGlofilename, data, pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC info.\n";
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& utc_model) const
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
                    if ((line_str.find("GAL", 0) != std::string::npos) && (line_str.find("IONOSPHERIC CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_galileo_iono_alpha_line(galileo_iono));
                        }
                    else if ((line_str.find("GAUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_galileo_time_corr_line(utc_model));
                        }
                    else if ((line_str.find("GPGA", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_gps_to_galileo_time_corr_line(utc_model));
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            data.push_back(get_leap_second_line(utc_model));
                        }
                    else if (line_str.find("END OF HEADER", 59) != std::string::npos)
                        {
                            data.push_back(line_str);
                            no_more_finds = true;
                        }
                    else
                        {
                            data.push_back(line_str);
                        }
                }
            else
                {
                    data.push_back(line_str);
                }
        }

    override_stream_with_new_data(out, navGalfilename, data, pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info.\n";
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Gps_Utc_Model& utc_model, const Gps_Iono& iono, const Gps_Ephemeris& eph) const
{
    std::vector<std::string> data;
    std::string line_aux;

    int64_t pos = out.tellp();
    out.seekp(0);
    data.clear();

    bool no_more_finds = false;
    std::string line_str;

    while (!out.eof())
        {
            std::getline(out, line_str);

            if (!no_more_finds)
                {
                    line_aux.clear();

                    if (d_version == 2)
                        {
                            if (line_str.find("ION ALPHA", 59) != std::string::npos)
                                {
                                    line_aux += std::string(2, ' ');
                                    line_aux += rightJustify(doub2for(iono.alpha0, 10, 2), 12);
                                    line_aux += rightJustify(doub2for(iono.alpha1, 10, 2), 12);
                                    line_aux += rightJustify(doub2for(iono.alpha2, 10, 2), 12);
                                    line_aux += rightJustify(doub2for(iono.alpha3, 10, 2), 12);
                                    line_aux += std::string(10, ' ');
                                    line_aux += leftJustify("ION ALPHA", 20);
                                    data.push_back(line_aux);
                                }
                            else if (line_str.find("ION BETA", 59) != std::string::npos)
                                {
                                    line_aux += std::string(2, ' ');
                                    line_aux += rightJustify(doub2for(iono.beta0, 10, 2), 12);
                                    line_aux += rightJustify(doub2for(iono.beta1, 10, 2), 12);
                                    line_aux += rightJustify(doub2for(iono.beta2, 10, 2), 12);
                                    line_aux += rightJustify(doub2for(iono.beta3, 10, 2), 12);
                                    line_aux += std::string(10, ' ');
                                    line_aux += leftJustify("ION BETA", 20);
                                    data.push_back(line_aux);
                                }
                            else if (line_str.find("DELTA-UTC", 59) != std::string::npos)
                                {
                                    line_aux += std::string(3, ' ');
                                    line_aux += rightJustify(doub2for(utc_model.A0, 18, 2), 19);
                                    line_aux += rightJustify(doub2for(utc_model.A1, 18, 2), 19);
                                    line_aux += rightJustify(std::to_string(utc_model.tot), 9);
                                    if (d_pre_2009_file == false)
                                        {
                                            if (eph.WN < 512)
                                                {
                                                    line_aux += rightJustify(std::to_string(utc_model.WN_T + (eph.WN / 256) * 256 + 2048), 9);  // valid from 2019 to 2029
                                                }
                                            else
                                                {
                                                    line_aux += rightJustify(std::to_string(utc_model.WN_T + (eph.WN / 256) * 256 + 1024), 9);  // valid from 2009 to 2019
                                                }
                                        }
                                    else
                                        {
                                            line_aux += rightJustify(std::to_string(utc_model.WN_T + (eph.WN / 256) * 256), 9);
                                        }
                                    line_aux += std::string(1, ' ');
                                    line_aux += leftJustify("DELTA-UTC: A0,A1,T,W", 20);
                                    data.push_back(line_aux);
                                }
                            else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                                {
                                    line_aux += rightJustify(std::to_string(utc_model.DeltaT_LS), 6);
                                    line_aux += std::string(54, ' ');
                                    line_aux += leftJustify("LEAP SECONDS", 20);
                                    data.push_back(line_aux);
                                }
                            else if (line_str.find("END OF HEADER", 59) != std::string::npos)
                                {
                                    data.push_back(line_str);
                                    no_more_finds = true;
                                }
                            else
                                {
                                    data.push_back(line_str);
                                }
                        }

                    if (d_version == 3)
                        {
                            if (line_str.find("GPSA", 0) != std::string::npos)
                                {
                                    data.push_back(get_gps_iono_alpha_line(iono));
                                }
                            else if (line_str.find("GPSB", 0) != std::string::npos)
                                {
                                    data.push_back(get_gps_iono_beta_line(iono));
                                }
                            else if (line_str.find("GPUT", 0) != std::string::npos)
                                {
                                    data.push_back(get_gps_time_corr_line(utc_model, eph, d_pre_2009_file));
                                }
                            else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                                {
                                    data.push_back(get_leap_second_line(utc_model));
                                }
                            else if (line_str.find("END OF HEADER", 59) != std::string::npos)
                                {
                                    data.push_back(line_str);
                                    no_more_finds = true;
                                }
                            else
                                {
                                    data.push_back(line_str);
                                }
                        }
                }
            else
                {
                    data.push_back(line_str);
                }
        }

    override_stream_with_new_data(out, navfilename, data, pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info.\n";
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Gps_CNAV_Utc_Model& utc_model, const Gps_CNAV_Iono& iono) const
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
                    if (line_str.find("GPSA", 0) != std::string::npos)
                        {
                            data.push_back(get_gps_iono_alpha_line(iono));
                        }
                    else if (line_str.find("GPSB", 0) != std::string::npos)
                        {
                            data.push_back(get_gps_iono_beta_line(iono));
                        }
                    else if (line_str.find("GPUT", 0) != std::string::npos)
                        {
                            data.push_back(get_gps_time_corr_line(utc_model));
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            data.push_back(get_leap_second_line(utc_model));
                        }
                    else if (line_str.find("END OF HEADER", 59) != std::string::npos)
                        {
                            data.push_back(line_str);
                            no_more_finds = true;
                        }
                    else
                        {
                            data.push_back(line_str);
                        }
                }
            else
                {
                    data.push_back(line_str);
                }
        }

    override_stream_with_new_data(out, navfilename, data, pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info.\n";
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Gps_CNAV_Utc_Model& utc_model, const Gps_CNAV_Iono& iono, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model) const
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
                    if ((line_str.find("GAL", 0) != std::string::npos) && (line_str.find("IONOSPHERIC CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_galileo_iono_alpha_line(galileo_iono));
                        }
                    else if ((line_str.find("GPSA", 0) != std::string::npos) && (line_str.find("IONOSPHERIC CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_gps_iono_alpha_line(iono));
                        }
                    else if ((line_str.find("GPSB", 0) != std::string::npos) && (line_str.find("IONOSPHERIC CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_gps_iono_beta_line(iono));
                        }
                    else if ((line_str.find("GAUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_galileo_time_corr_line(galileo_utc_model));
                        }
                    else if ((line_str.find("GPGA", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_gps_to_galileo_time_corr_line(galileo_utc_model));
                        }
                    else if (line_str.find("GPUT", 0) != std::string::npos)
                        {
                            data.push_back(get_gps_time_corr_line(utc_model));
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            data.push_back(get_leap_second_line(utc_model));
                        }
                    else if (line_str.find("END OF HEADER", 59) != std::string::npos)
                        {
                            data.push_back(line_str);
                            no_more_finds = true;
                        }
                    else
                        {
                            data.push_back(line_str);
                        }
                }
            else
                {
                    data.push_back(line_str);
                }
        }
    override_stream_with_new_data(out, navfilename, data, pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info.\n";
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Gps_Iono& gps_iono, const Gps_Utc_Model& gps_utc_model, const Gps_Ephemeris& eph, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model) const
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
                    if (line_str.find("GPSA", 0) != std::string::npos)
                        {
                            data.push_back(get_gps_iono_alpha_line(gps_iono));
                        }
                    else if ((line_str.find("GAL", 0) != std::string::npos) && (line_str.find("IONOSPHERIC CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_galileo_iono_alpha_line(galileo_iono));
                        }
                    else if ((line_str.find("GPSB", 0) != std::string::npos) && (line_str.find("IONOSPHERIC CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_gps_iono_beta_line(gps_iono));
                        }
                    else if ((line_str.find("GPUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_gps_time_corr_line(gps_utc_model, eph, d_pre_2009_file));
                        }
                    else if ((line_str.find("GAUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_galileo_time_corr_line(galileo_utc_model));
                        }
                    else if ((line_str.find("GPGA", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_gps_to_galileo_time_corr_line(galileo_utc_model));
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            data.push_back(get_leap_second_line(galileo_utc_model));
                        }
                    else if (line_str.find("END OF HEADER", 59) != std::string::npos)
                        {
                            data.push_back(line_str);
                            no_more_finds = true;
                        }
                    else
                        {
                            data.push_back(line_str);
                        }
                }
            else
                {
                    data.push_back(line_str);
                }
        }

    override_stream_with_new_data(out, navMixfilename, data, pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info.\n";
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Gps_Iono& gps_iono, const Gps_Utc_Model& gps_utc_model, const Gps_Ephemeris& eph, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model) const
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
                    if (line_str.find("GPSA", 0) != std::string::npos)
                        {
                            data.push_back(get_gps_iono_alpha_line(gps_iono));
                        }
                    else if ((line_str.find("GPUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_gps_time_corr_line(gps_utc_model, eph, d_pre_2009_file));
                        }
                    else if ((line_str.find("GLUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_glonass_time_corr_line(glonass_gnav_utc_model));
                        }
                    else if ((line_str.find("GLGP", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_glonass_to_gps_time_corr_line(glonass_gnav_utc_model));
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            data.push_back(get_leap_second_line(gps_utc_model));
                        }
                    else if (line_str.find("END OF HEADER", 59) != std::string::npos)
                        {
                            data.push_back(line_str);
                            no_more_finds = true;
                        }
                    else
                        {
                            data.push_back(line_str);
                        }
                }
            else
                {
                    data.push_back(line_str);
                }
        }

    override_stream_with_new_data(out, navMixfilename, data, pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info.\n";
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Gps_CNAV_Iono& gps_iono, const Gps_CNAV_Utc_Model& gps_utc_model, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model) const
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
                    if (line_str.find("GPSA", 0) != std::string::npos)
                        {
                            data.push_back(get_gps_iono_alpha_line(gps_iono));
                        }
                    else if ((line_str.find("GPUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_gps_time_corr_line(gps_utc_model));
                        }
                    else if ((line_str.find("GLUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_glonass_time_corr_line(glonass_gnav_utc_model));
                        }
                    else if ((line_str.find("GLGP", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_glonass_to_gps_time_corr_line(glonass_gnav_utc_model));
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            data.push_back(get_leap_second_line(gps_utc_model));
                        }
                    else if (line_str.find("END OF HEADER", 59) != std::string::npos)
                        {
                            data.push_back(line_str);
                            no_more_finds = true;
                        }
                    else
                        {
                            data.push_back(line_str);
                        }
                }
            else
                {
                    data.push_back(line_str);
                }
        }

    override_stream_with_new_data(out, navMixfilename, data, pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info.\n";
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model) const
{
    // There is not time system correction between Galileo and GLONASS
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
                    if ((line_str.find("GAL", 0) != std::string::npos) && (line_str.find("IONOSPHERIC CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_galileo_iono_alpha_line(galileo_iono));
                        }
                    else if ((line_str.find("GAUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_galileo_time_corr_line(galileo_utc_model));
                        }
                    else if ((line_str.find("GLUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            data.push_back(get_glonass_time_corr_line(glonass_gnav_utc_model));
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            data.push_back(get_leap_second_line(galileo_utc_model));
                        }
                    else if (line_str.find("END OF HEADER", 59) != std::string::npos)
                        {
                            data.push_back(line_str);
                            no_more_finds = true;
                        }
                    else
                        {
                            data.push_back(line_str);
                        }
                }
            else
                {
                    data.push_back(line_str);
                }
        }

    override_stream_with_new_data(out, navMixfilename, data, pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info.\n";
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Beidou_Dnav_Utc_Model& utc_model, const Beidou_Dnav_Iono& iono) const
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
                    if (line_str.find("BDSA", 0) != std::string::npos)
                        {
                            data.push_back(get_beidou_iono_alpha_line(iono));
                        }
                    else if (line_str.find("BDSB", 0) != std::string::npos)
                        {
                            data.push_back(get_beidou_iono_beta_line(iono));
                        }
                    else if (line_str.find("BDUT", 0) != std::string::npos)
                        {
                            data.push_back(get_beidou_time_corr_line(utc_model));
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            data.push_back(get_leap_second_line(utc_model));
                        }
                    else if (line_str.find("END OF HEADER", 59) != std::string::npos)
                        {
                            data.push_back(line_str);
                            no_more_finds = true;
                        }
                    else
                        {
                            data.push_back(line_str);
                        }
                }
            else
                {
                    data.push_back(line_str);
                }
        }

    override_stream_with_new_data(out, navfilename, data, pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info.\n";
}


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int32_t, Gps_Ephemeris>& eph_map) const
{
    std::string line;
    const auto& sys_char = satelliteSystem.at("GPS");

    for (const auto& gps_ephemeris_iter : eph_map)
        {
            const auto& eph = gps_ephemeris_iter.second;

            // -------- SV / EPOCH / SV CLK
            const boost::posix_time::ptime p_utc_time = Rinex_Printer::compute_GPS_time(eph, eph.toc);

            if (d_version == 2)
                {
                    const std::string timestring = boost::posix_time::to_iso_string(p_utc_time);
                    const std::string month(timestring, 4, 2);
                    const std::string day(timestring, 6, 2);
                    const std::string hour(timestring, 9, 2);
                    const std::string minutes(timestring, 11, 2);
                    const std::string seconds(timestring, 13, 2);
                    line += rightJustify(std::to_string(eph.PRN), 2);
                    line += std::string(1, ' ');
                    const std::string year(timestring, 2, 2);
                    line += year;
                    line += std::string(1, ' ');
                    if (boost::lexical_cast<int32_t>(month) < 10)
                        {
                            line += std::string(1, ' ');
                            line += std::string(month, 1, 1);
                        }
                    else
                        {
                            line += month;
                        }
                    line += std::string(1, ' ');
                    if (boost::lexical_cast<int32_t>(day) < 10)
                        {
                            line += std::string(1, ' ');
                            line += std::string(day, 1, 1);
                        }
                    else
                        {
                            line += day;
                        }
                    line += std::string(1, ' ');
                    if (boost::lexical_cast<int32_t>(hour) < 10)
                        {
                            line += std::string(1, ' ');
                            line += std::string(hour, 1, 1);
                        }
                    else
                        {
                            line += hour;
                        }
                    line += std::string(1, ' ');
                    if (boost::lexical_cast<int32_t>(minutes) < 10)
                        {
                            line += std::string(1, ' ');
                            line += std::string(minutes, 1, 1);
                        }
                    else
                        {
                            line += minutes;
                        }
                    line += std::string(1, ' ');
                    if (boost::lexical_cast<int32_t>(seconds) < 10)
                        {
                            line += std::string(1, ' ');
                            line += std::string(seconds, 1, 1);
                        }
                    else
                        {
                            line += seconds;
                        }
                    line += std::string(1, '.');
                    std::string decimal = std::string("0");
                    if (timestring.size() > 16)
                        {
                            decimal = std::string(timestring, 16, 1);
                        }
                    line += decimal;
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


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int32_t, Gps_CNAV_Ephemeris>& eph_map)
{
    const auto& sys_char = satelliteSystem.at("GPS");

    for (const auto& gps_ephemeris_iter : eph_map)
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


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int32_t, Galileo_Ephemeris>& eph_map) const
{
    const auto& sys_char = satelliteSystem.at("Galileo");

    for (const auto& galileo_ephemeris_iter : eph_map)
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
            std::string E1B_HS;
            std::string E5B_HS;
            if (eph.E1B_HS == 0)
                {
                    E1B_HS = "00";
                }
            if (eph.E1B_HS == 1)
                {
                    E1B_HS = "01";
                }
            if (eph.E1B_HS == 2)
                {
                    E1B_HS = "10";
                }
            if (eph.E1B_HS == 3)
                {
                    E1B_HS = "11";
                }
            if (eph.E5b_HS == 0)
                {
                    E5B_HS = "00";
                }
            if (eph.E5b_HS == 1)
                {
                    E5B_HS = "01";
                }
            if (eph.E5b_HS == 2)
                {
                    E5B_HS = "10";
                }
            if (eph.E5b_HS == 3)
                {
                    E5B_HS = "11";
                }

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


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int32_t, Glonass_Gnav_Ephemeris>& eph_map) const
{
    std::string line;
    const auto& sys_char = satelliteSystem.at("GLONASS");

    for (const auto& glonass_gnav_ephemeris_iter : eph_map)
        {
            const auto& eph = glonass_gnav_ephemeris_iter.second;

            // -------- SV / EPOCH / SV CLK
            const boost::posix_time::ptime p_utc_time = eph.glot_to_utc(eph.d_t_b, 0.0);
            const std::string timestring = boost::posix_time::to_iso_string(p_utc_time);
            const std::string month(timestring, 4, 2);
            const std::string day(timestring, 6, 2);
            const std::string hour(timestring, 9, 2);
            const std::string minutes(timestring, 11, 2);
            const std::string seconds(timestring, 13, 2);
            if (d_version == 2)
                {
                    line += rightJustify(std::to_string(eph.PRN), 2);
                    line += std::string(1, ' ');
                    const std::string year(timestring, 2, 2);
                    line += year;
                    line += std::string(1, ' ');
                    if (boost::lexical_cast<int32_t>(month) < 10)
                        {
                            line += std::string(1, ' ');
                            line += std::string(month, 1, 1);
                        }
                    else
                        {
                            line += month;
                        }
                    line += std::string(1, ' ');
                    if (boost::lexical_cast<int32_t>(day) < 10)
                        {
                            line += std::string(1, ' ');
                            line += std::string(day, 1, 1);
                        }
                    else
                        {
                            line += day;
                        }
                    line += std::string(1, ' ');
                    if (boost::lexical_cast<int32_t>(hour) < 10)
                        {
                            line += std::string(1, ' ');
                            line += std::string(hour, 1, 1);
                        }
                    else
                        {
                            line += hour;
                        }
                    line += std::string(1, ' ');
                    if (boost::lexical_cast<int32_t>(minutes) < 10)
                        {
                            line += std::string(1, ' ');
                            line += std::string(minutes, 1, 1);
                        }
                    else
                        {
                            line += minutes;
                        }
                    line += std::string(1, ' ');
                    if (boost::lexical_cast<int32_t>(seconds) < 10)
                        {
                            line += std::string(1, ' ');
                            line += std::string(seconds, 1, 1);
                        }
                    else
                        {
                            line += seconds;
                        }
                    line += std::string(1, '.');
                    std::string decimal = std::string("0");
                    if (timestring.size() > 16)
                        {
                            decimal = std::string(timestring, 16, 1);
                        }
                    line += decimal;
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


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int32_t, Gps_Ephemeris>& gps_eph_map, const std::map<int32_t, Galileo_Ephemeris>& galileo_eph_map)
{
    d_version = 3;
    d_stringVersion = "3.02";
    Rinex_Printer::log_rinex_nav(out, gps_eph_map);
    Rinex_Printer::log_rinex_nav(out, galileo_eph_map);
}


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int32_t, Gps_CNAV_Ephemeris>& gps_cnav_eph_map, const std::map<int32_t, Galileo_Ephemeris>& galileo_eph_map)
{
    d_version = 3;
    d_stringVersion = "3.02";
    Rinex_Printer::log_rinex_nav(out, gps_cnav_eph_map);
    Rinex_Printer::log_rinex_nav(out, galileo_eph_map);
}


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int32_t, Gps_Ephemeris>& gps_eph_map, const std::map<int32_t, Glonass_Gnav_Ephemeris>& glonass_gnav_eph_map) const
{
    Rinex_Printer::log_rinex_nav(out, gps_eph_map);
    Rinex_Printer::log_rinex_nav(out, glonass_gnav_eph_map);
}


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int32_t, Gps_CNAV_Ephemeris>& gps_cnav_eph_map, const std::map<int32_t, Glonass_Gnav_Ephemeris>& glonass_gnav_eph_map)
{
    Rinex_Printer::log_rinex_nav(out, gps_cnav_eph_map);
    Rinex_Printer::log_rinex_nav(out, glonass_gnav_eph_map);
}


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int32_t, Galileo_Ephemeris>& galileo_eph_map, const std::map<int32_t, Glonass_Gnav_Ephemeris>& glonass_gnav_eph_map)
{
    d_version = 3;
    d_stringVersion = "3.02";
    Rinex_Printer::log_rinex_nav(out, galileo_eph_map);
    Rinex_Printer::log_rinex_nav(out, glonass_gnav_eph_map);
}


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int32_t, Beidou_Dnav_Ephemeris>& eph_map) const
{
    const auto& sys_char = satelliteSystem.at("Beidou");

    for (const auto& bds_ephemeris_iter : eph_map)
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


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Glonass_Gnav_Ephemeris& eph, double d_TOW_first_observation, const std::string& glonass_bands)
{
    std::map<int32_t, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_ephemeris_iter;

    std::string constellation_legend;
    std::string marker_type_id;

    if (d_version == 2)
        {
            constellation_legend = "BLANK OR G = GPS,  R = GLONASS,  E = GALILEO,  M = MIXED";
            marker_type_id = "NUMBER";
        }
    if (d_version == 3)
        {
            constellation_legend = "G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED";
            marker_type_id = "TYPE";
        }

    add_observation_header_start(out, "GLONASS", "GLONASS", Rinex_Printer::getLocalTime(), d_stringVersion, constellation_legend, "GROUND_CRAFT", marker_type_id);

    // -------- SYS / OBS TYPES
    if (d_version == 3)
        {
            // -------- SYS / OBS TYPES
            add_obs_sys_obs_type_glonass(out, glonass_bands, observationType, observationCode);
        }
    if (d_version == 2)
        {
            // -------- SYS / OBS TYPES
            add_obs_sys_obs_type_v2(out, "GLONASS_G1_CA_v2", observationType, observationCode);
        }

    // -------- Signal Strength units (Only version 3)
    if (d_version == 3)
        {
            add_obs_signal_strength(out);
        }

    // -------- TIME OF FIRST OBS
    double intpart = 0;
    const boost::posix_time::ptime p_utc_time = Rinex_Printer::compute_UTC_time(eph, d_TOW_first_observation);
    const double seconds = p_utc_time.time_of_day().seconds() + modf(d_TOW_first_observation, &intpart);
    add_obs_time_first_obs(out, "GLO", p_utc_time, seconds);

    // -------- GLONASS SLOT / FRQ # (On;y d_version 3)
    if (d_version == 3)
        {
            // -------- GLONASS SLOT / FRQ #
            add_obs_glonass_slot_freq(out);

            // -------- GLONASS CODE/PHS/BIS
            add_obs_glonass_code_phase_bias(out, observationType, observationCode);
        }

    // -------- END OF HEADER
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Gps_Ephemeris& gps_eph, const Glonass_Gnav_Ephemeris& /*glonass_gnav_eph*/, double d_TOW_first_observation, const std::string& glonass_bands)
{
    std::string marker_type_id;

    if (d_version == 2)
        {
            marker_type_id = "NUMBER";
        }
    else if (d_version == 3)
        {
            marker_type_id = "TYPE";
        }

    const std::string constellation_legend = "G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED";
    add_observation_header_start(out, "Mixed", "MIXED (GPS/GLO)", Rinex_Printer::getLocalTime(), d_stringVersion, constellation_legend, "GROUND_CRAFT", marker_type_id);

    // -------- SYS / OBS TYPES
    if (d_version == 3)
        {
            // GPS line
            add_obs_sys_obs_type_gps(out, "1C", observationType, observationCode);

            // Glonass line
            add_obs_sys_obs_type_glonass(out, glonass_bands, observationType, observationCode);
        }
    if (d_version == 2)
        {
            // -------- SYS / OBS TYPES
            add_obs_sys_obs_type_v2(out, "GLONASS_G1_CA_v2", observationType, observationCode);
        }

    // -------- Signal Strength units (only version 3)
    if (d_version == 3)
        {
            add_obs_signal_strength(out);
        }

    // -------- TIME OF FIRST OBS
    add_obs_time_first_obs(out, "GPS", Rinex_Printer::compute_GPS_time(gps_eph, d_TOW_first_observation), d_TOW_first_observation);

    // -------- GLONASS SLOT / FRQ # (On;y version 3)
    if (d_version == 3)
        {
            // -------- GLONASS SLOT / FRQ #
            add_obs_glonass_slot_freq(out);

            // -------- GLONASS CODE/PHS/BIS
            add_obs_glonass_code_phase_bias(out, observationType, observationCode);
        }

    // -------- end of header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Gps_CNAV_Ephemeris& gps_cnav_eph, const Glonass_Gnav_Ephemeris& /*glonass_gnav_eph*/, double d_TOW_first_observation, const std::string& glonass_bands)
{
    std::string marker_type_id;

    if (d_version == 2)
        {
            marker_type_id = "NUMBER";
        }
    else if (d_version == 3)
        {
            marker_type_id = "TYPE";
        }

    const std::string constellation_legend = "G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED";
    add_observation_header_start(out, "Mixed", "MIXED (GPS/GLO)", Rinex_Printer::getLocalTime(), d_stringVersion, constellation_legend, "GROUND_CRAFT", marker_type_id);

    // -------- SYS / OBS TYPES

    // GPS line
    add_obs_sys_obs_type_gps(out, "2S", observationType, observationCode);

    // Glonass line
    add_obs_sys_obs_type_glonass(out, glonass_bands, observationType, observationCode);

    // -------- Signal Strength units (only version 3)
    add_obs_signal_strength(out);

    // -------- TIME OF FIRST OBS
    add_obs_time_first_obs(out, "GPS", Rinex_Printer::compute_GPS_time(gps_cnav_eph, d_TOW_first_observation), d_TOW_first_observation);

    // -------- GLONASS SLOT / FRQ #
    add_obs_glonass_slot_freq(out);

    // -------- GLONASS CODE/PHS/BIS
    add_obs_glonass_code_phase_bias(out, observationType, observationCode);

    // -------- end of header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Galileo_Ephemeris& galileo_eph, const Glonass_Gnav_Ephemeris& /*glonass_gnav_eph*/, double d_TOW_first_observation, const std::string& galileo_bands, const std::string& glonass_bands)
{
    d_version = 3;

    const std::string constellation_legend = "G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED";
    add_observation_header_start(out, "Mixed", "MIXED (GALILEO/GLONASS)", Rinex_Printer::getLocalTime(), "3.02", constellation_legend, "NON_GEODETIC", "TYPE");

    // -------- SYS / OBS TYPES

    // Galileo line
    add_obs_sys_obs_type_galileo(out, galileo_bands, observationType, observationCode);

    // Glonass line
    add_obs_sys_obs_type_glonass(out, glonass_bands, observationType, observationCode);

    // -------- Signal Strength units
    add_obs_signal_strength(out);

    // -------- TIME OF FIRST OBS
    add_obs_time_first_obs(out, "Galileo", Rinex_Printer::compute_Galileo_time(galileo_eph, d_TOW_first_observation), d_TOW_first_observation);

    // -------- end of header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Gps_Ephemeris& eph, double d_TOW_first_observation)
{
    std::string line;
    std::string constellation_legend;

    if (d_version == 2)
        {
            constellation_legend = "BLANK OR G = GPS,  R = GLONASS,  E = GALILEO,  M = MIXED";
        }
    if (d_version == 3)
        {
            constellation_legend = "G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED";
        }

    add_observation_header_start(out, "GPS", "GPS", Rinex_Printer::getLocalTime(), "3.02", constellation_legend, "NON_GEODETIC", "TYPE");

    if (d_version == 2)
        {
            // --------- WAVELENGTH FACTOR
            // put here real data!
            line.clear();
            line += rightJustify("1", 6);
            line += rightJustify("1", 6);
            line += std::string(48, ' ');
            line += leftJustify("WAVELENGTH FACT L1/2", 20);
            lengthCheck(line);
            out << line << '\n';
        }

    if (d_version == 3)
        {
            // -------- SYS / OBS TYPES
            add_obs_sys_obs_type_gps(out, "1C", observationType, observationCode);
        }

    if (d_version == 2)
        {
            // -------- SYS / OBS TYPES
            add_obs_sys_obs_type_v2(out, "GPS_L1_CA_v2", observationType, observationCode);
        }

    if (d_version == 3)
        {
            // -------- Signal Strength units
            add_obs_signal_strength(out);
        }

    // -------- TIME OF FIRST OBS
    add_obs_time_first_obs(out, "GPS", Rinex_Printer::compute_GPS_time(eph, d_TOW_first_observation), d_TOW_first_observation);

    // -------- SYS /PHASE SHIFTS

    // -------- end of header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Gps_CNAV_Ephemeris& eph, double d_TOW_first_observation, const std::string& gps_bands)
{
    const std::string constellation_legend = "G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED";
    add_observation_header_start(out, "GPS", "GPS", Rinex_Printer::getLocalTime(), d_stringVersion, constellation_legend);

    // -------- SYS / OBS TYPES
    add_obs_sys_obs_type_gps(out, gps_bands, observationType, observationCode);

    // -------- Signal Strength units
    add_obs_signal_strength(out);

    // -------- TIME OF FIRST OBS
    add_obs_time_first_obs(out, "GPS", Rinex_Printer::compute_GPS_time(eph, d_TOW_first_observation), d_TOW_first_observation);

    // -------- SYS /PHASE SHIFTS

    // -------- end of header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Gps_Ephemeris& eph, const Gps_CNAV_Ephemeris& /*eph_cnav*/, double d_TOW_first_observation, const std::string& gps_bands)
{
    const std::string constellation_legend = "G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED";
    add_observation_header_start(out, "GPS", "GPS", Rinex_Printer::getLocalTime(), d_stringVersion, constellation_legend);

    // -------- SYS / OBS TYPES
    add_obs_sys_obs_type_gps(out, gps_bands, observationType, observationCode);

    // -------- Signal Strength units
    add_obs_signal_strength(out);

    // -------- TIME OF FIRST OBS
    add_obs_time_first_obs(out, "GPS", Rinex_Printer::compute_GPS_time(eph, d_TOW_first_observation), d_TOW_first_observation);

    // -------- SYS /PHASE SHIFTS

    // -------- end of header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Gps_Ephemeris& gps_eph, const Gps_CNAV_Ephemeris& /*eph_cnav*/, const Galileo_Ephemeris& /*galileo_eph*/, double d_TOW_first_observation, const std::string& gps_bands, const std::string& galileo_bands)
{
    d_version = 3;

    const std::string constellation_legend = "G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED";
    add_observation_header_start(out, "Mixed", "MIXED (GPS/GALILEO)", Rinex_Printer::getLocalTime(), "3.02", constellation_legend);

    // -------- SYS / OBS TYPES

    // GPS line
    add_obs_sys_obs_type_gps(out, gps_bands, observationType, observationCode);

    // Galileo line
    add_obs_sys_obs_type_galileo(out, galileo_bands, observationType, observationCode);

    // -------- Signal Strength units
    add_obs_signal_strength(out);

    // -------- TIME OF FIRST OBS
    add_obs_time_first_obs(out, "GPS", Rinex_Printer::compute_GPS_time(gps_eph, d_TOW_first_observation), d_TOW_first_observation);

    // -------- end of header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Gps_CNAV_Ephemeris& eph_cnav, const Galileo_Ephemeris& /*galileo_eph*/, double d_TOW_first_observation, const std::string& gps_bands, const std::string& galileo_bands)
{
    d_version = 3;

    const std::string constellation_legend = "G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED";
    add_observation_header_start(out, "Mixed", "MIXED (GPS/GALILEO)", Rinex_Printer::getLocalTime(), "3.02", constellation_legend);

    // -------- SYS / OBS TYPES

    // GPS line
    add_obs_sys_obs_type_gps(out, gps_bands, observationType, observationCode);

    // Galileo line
    add_obs_sys_obs_type_galileo(out, galileo_bands, observationType, observationCode);

    // -------- Signal Strength units
    add_obs_signal_strength(out);

    // -------- TIME OF FIRST OBS
    add_obs_time_first_obs(out, "GPS", Rinex_Printer::compute_GPS_time(eph_cnav, d_TOW_first_observation), d_TOW_first_observation);

    // -------- end of header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Galileo_Ephemeris& eph, double d_TOW_first_observation, const std::string& bands)
{
    d_version = 3;

    const std::string version = bands.find("E6") != std::string::npos ? "3.05" : "3.02";
    const std::string constellation_legend = "G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED";
    add_observation_header_start(out, "Galileo", "GALILEO", Rinex_Printer::getLocalTime(), version, constellation_legend);

    // -------- SYS / OBS TYPES

    add_obs_sys_obs_type_galileo(out, bands, observationType, observationCode);

    // -------- Signal Strength units
    add_obs_signal_strength(out);

    // -------- TIME OF FIRST OBS
    add_obs_time_first_obs(out, "GAL", Rinex_Printer::compute_Galileo_time(eph, d_TOW_first_observation), d_TOW_first_observation);

    // -------- SYS /PHASE SHIFTS

    // -------- end of header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Gps_Ephemeris& gps_eph, const Galileo_Ephemeris& /*galileo_eph*/, double d_TOW_first_observation, const std::string& galileo_bands)
{
    d_version = 3;

    const std::string constellation_legend = "G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED";
    add_observation_header_start(out, "Mixed", "MIXED (GPS/GALILEO)", Rinex_Printer::getLocalTime(), "3.02", constellation_legend);

    // -------- SYS / OBS TYPES

    // GPS line
    add_obs_sys_obs_type_gps(out, "1C", observationType, observationCode);

    // Galileo line
    add_obs_sys_obs_type_galileo(out, galileo_bands, observationType, observationCode);

    // -------- Signal Strength units
    add_obs_signal_strength(out);

    // -------- TIME OF FIRST OBS
    add_obs_time_first_obs(out, "GPS", Rinex_Printer::compute_GPS_time(gps_eph, d_TOW_first_observation), d_TOW_first_observation);

    // -------- end of header
    out << get_end_of_header_line() << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Beidou_Dnav_Ephemeris& eph, double d_TOW_first_observation, const std::string& bands)
{
    d_version = 3;

    const std::string constellation_legend = "G = GPS  R = GLONASS  E = GALILEO  C = BEIDOU  M = MIXED";
    add_observation_header_start(out, "Beidou", "BEIDOU", Rinex_Printer::getLocalTime(), "3.02", constellation_legend);

    // -------- SYS / OBS TYPES
    add_obs_sys_obs_type_beidou(out, bands, observationType, observationCode);

    // -------- Signal Strength units
    add_obs_signal_strength(out);

    // -------- TIME OF FIRST OBS
    add_obs_time_first_obs(out, "BDT", Rinex_Printer::compute_BDS_time(eph, d_TOW_first_observation), d_TOW_first_observation);

    // -------- SYS /PHASE SHIFTS

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
            data.push_back(line_str);

            if (!no_more_finds)
                {
                    if (line_str.find("TIME OF FIRST OBS", 59) != std::string::npos)  // TIME OF FIRST OBS last header annotation might change in the future
                        {
                            data.push_back(leap_second_line);
                        }
                    else if (line_str.find("END OF HEADER", 59) != std::string::npos)
                        {
                            no_more_finds = true;
                        }
                }
        }

    override_stream_with_new_data(out, obsfilename, data, 0);
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Glonass_Gnav_Ephemeris& eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables) const
{
    // RINEX observations timestamps are GPS timestamps.
    std::string line;
    double int_sec = 0;

    const boost::posix_time::ptime p_glonass_time = Rinex_Printer::compute_UTC_time(eph, obs_time);
    const std::string timestring = boost::posix_time::to_iso_string(p_glonass_time);
    // double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    // const double gps_t = eph.sv_clock_correction(obs_time);

    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);
    const double utc_sec = modf(obs_time, &int_sec) + p_glonass_time.time_of_day().seconds();

    if (d_version == 2)
        {
            line.clear();
            const std::string year(timestring, 2, 2);
            line += std::string(1, ' ');
            line += year;
            line += std::string(1, ' ');
            if (month.compare(0, 1, "0") == 0)
                {
                    line += std::string(1, ' ');
                    line += month.substr(1, 1);
                }
            else
                {
                    line += month;
                }
            line += std::string(1, ' ');
            if (day.compare(0, 1, "0") == 0)
                {
                    line += std::string(1, ' ');
                    line += day.substr(1, 1);
                }
            else
                {
                    line += day;
                }
            line += std::string(1, ' ');
            line += hour;
            line += std::string(1, ' ');
            line += minutes;
            line += std::string(1, ' ');
            if (utc_sec < 10)
                {
                    line += std::string(1, ' ');
                }
            line += asString(utc_sec, 7);
            line += std::string(2, ' ');
            // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
            line += std::string(1, '0');
            // Number of satellites observed in current epoch
            const int32_t numSatellitesObserved = observables.size();
            line += rightJustify(std::to_string(numSatellitesObserved), 3);
            for (const auto& observables_iter : observables)
                {
                    line += satelliteSystem.at("GLONASS");
                    if (static_cast<int32_t>(observables_iter.second.PRN) < 10)
                        {
                            line += std::string(1, '0');
                        }
                    line += std::to_string(static_cast<int32_t>(observables_iter.second.PRN));
                }
            // Receiver clock offset (optional)
            // line += rightJustify(asString(clockOffset, 12), 15);
            line += std::string(80 - line.size(), ' ');
            lengthCheck(line);
            out << line << '\n';

            for (const auto& observables_iter : observables)
                {
                    std::string lineObs;
                    lineObs.clear();
                    line.clear();
                    // GLONASS L1 PSEUDORANGE
                    line += std::string(2, ' ');
                    lineObs += rightJustify(asString(observables_iter.second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(observables_iter.second.CN0_dB_hz);
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);
                    // GLONASS L1 CA PHASE
                    lineObs += rightJustify(asString(observables_iter.second.Carrier_phase_rads / TWO_PI, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);
                    // GLONASS L1 CA DOPPLER
                    lineObs += rightJustify(asString(observables_iter.second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);
                    // GLONASS L1 SIGNAL STRENGTH
                    lineObs += rightJustify(asString(observables_iter.second.CN0_dB_hz, 3), 14);
                    if (lineObs.size() < 80)
                        {
                            lineObs += std::string(80 - lineObs.size(), ' ');
                        }
                    out << lineObs << '\n';
                }
        }

    if (d_version == 3)
        {
            const std::string year(timestring, 0, 4);
            line += std::string(1, '>');
            line += std::string(1, ' ');
            line += year;
            line += std::string(1, ' ');
            line += month;
            line += std::string(1, ' ');
            line += day;
            line += std::string(1, ' ');
            line += hour;
            line += std::string(1, ' ');
            line += minutes;

            line += std::string(1, ' ');
            // Add extra 0 if seconds are < 10
            if (utc_sec < 10)
                {
                    line += std::string(1, '0');
                }
            line += asString(utc_sec, 7);
            line += std::string(2, ' ');
            // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
            line += std::string(1, '0');

            // Number of satellites observed in current epoch
            const int32_t numSatellitesObserved = observables.size();
            line += rightJustify(std::to_string(numSatellitesObserved), 3);

            // Receiver clock offset (optional)
            // line += rightJustify(asString(clockOffset, 12), 15);

            line += std::string(80 - line.size(), ' ');
            lengthCheck(line);
            out << line << '\n';

            for (const auto& observables_iter : observables)
                {
                    std::string lineObs;
                    lineObs.clear();
                    lineObs += satelliteSystem.at("GLONASS");
                    if (static_cast<int32_t>(observables_iter.second.PRN) < 10)
                        {
                            lineObs += std::string(1, '0');
                        }
                    lineObs += std::to_string(static_cast<int32_t>(observables_iter.second.PRN));
                    // lineObs += std::string(2, ' ');
                    lineObs += rightJustify(asString(observables_iter.second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(observables_iter.second.CN0_dB_hz);
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // GLONASS L1 CA PHASE
                    lineObs += rightJustify(asString(observables_iter.second.Carrier_phase_rads / TWO_PI, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // GLONASS L1 CA DOPPLER
                    lineObs += rightJustify(asString(observables_iter.second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }

                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // GLONASS L1 SIGNAL STRENGTH
                    lineObs += rightJustify(asString(observables_iter.second.CN0_dB_hz, 3), 14);

                    if (lineObs.size() < 80)
                        {
                            lineObs += std::string(80 - lineObs.size(), ' ');
                        }
                    out << lineObs << '\n';
                }
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Gps_Ephemeris& gps_eph, const Glonass_Gnav_Ephemeris& /*glonass_gnav_eph*/, double gps_obs_time, const std::map<int32_t, Gnss_Synchro>& observables) const
{
    std::string line;

    // -------- EPOCH record
    const boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(gps_eph, gps_obs_time);
    const std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    // double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    // const double gps_t = eph.sv_clock_correction(obs_time);
    const double gps_t = gps_obs_time;

    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);

    if (d_version == 2)
        {
            line.clear();
            const std::string year(timestring, 2, 2);
            line += std::string(1, ' ');
            line += year;
            line += std::string(1, ' ');
            if (month.compare(0, 1, "0") == 0)
                {
                    line += std::string(1, ' ');
                    line += month.substr(1, 1);
                }
            else
                {
                    line += month;
                }
            line += std::string(1, ' ');
            if (day.compare(0, 1, "0") == 0)
                {
                    line += std::string(1, ' ');
                    line += day.substr(1, 1);
                }
            else
                {
                    line += day;
                }
            line += std::string(1, ' ');
            line += hour;
            line += std::string(1, ' ');
            line += minutes;
            line += std::string(1, ' ');
            const double second_ = fmod(gps_t, 60);
            if (second_ < 10)
                {
                    line += std::string(1, ' ');
                }
            line += asString(second_, 7);
            line += std::string(2, ' ');
            // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
            line += std::string(1, '0');
        }
    if (d_version == 3)
        {
            const std::string year(timestring, 0, 4);
            line += std::string(1, '>');
            line += std::string(1, ' ');
            line += year;
            line += std::string(1, ' ');
            line += month;
            line += std::string(1, ' ');
            line += day;
            line += std::string(1, ' ');
            line += hour;
            line += std::string(1, ' ');
            line += minutes;

            line += std::string(1, ' ');
            const double seconds = fmod(gps_t, 60);
            // Add extra 0 if seconds are < 10
            if (seconds < 10)
                {
                    line += std::string(1, '0');
                }
            line += asString(seconds, 7);
            line += std::string(2, ' ');
            // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
            line += std::string(1, '0');
        }

    // Number of satellites observed in current epoch
    // Get maps with observations
    std::map<int32_t, Gnss_Synchro> observablesG1C;
    std::map<int32_t, Gnss_Synchro> observablesR1C;
    std::map<int32_t, Gnss_Synchro> observablesR2C;

    for (const auto& observables_iter : observables)
        {
            const std::string system_(&observables_iter.second.System, 1);
            const std::string sig_(observables_iter.second.Signal);
            if ((system_ == "R") && (sig_ == "1G"))
                {
                    observablesR1C.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "R") && (sig_ == "2G"))
                {
                    observablesR2C.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "G") && (sig_ == "1C"))
                {
                    observablesG1C.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
        }

    std::multimap<uint32_t, Gnss_Synchro> total_glo_map;
    std::set<uint32_t> available_glo_prns;
    std::set<uint32_t>::iterator it;
    for (const auto& observables_iter : observablesR1C)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_glo_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_glo_prns.find(prn_);
            if (it == available_glo_prns.end())
                {
                    available_glo_prns.insert(prn_);
                }
        }

    for (const auto& observables_iter : observablesR2C)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_glo_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_glo_prns.find(prn_);
            if (it == available_glo_prns.end())
                {
                    available_glo_prns.insert(prn_);
                }
        }

    const int32_t numGloSatellitesObserved = available_glo_prns.size();
    const int32_t numGpsSatellitesObserved = observablesG1C.size();
    const int32_t numSatellitesObserved = numGloSatellitesObserved + numGpsSatellitesObserved;
    line += rightJustify(std::to_string(numSatellitesObserved), 3);
    if (d_version == 2)
        {
            // Add list of GPS satellites
            for (const auto& observables_iter : observablesG1C)
                {
                    line += satelliteSystem.at("GPS");
                    if (static_cast<int32_t>(observables_iter.second.PRN) < 10)
                        {
                            line += std::string(1, '0');
                        }
                    line += std::to_string(static_cast<int32_t>(observables_iter.second.PRN));
                }
            // Add list of GLONASS L1 satellites
            for (const auto& observables_iter : observablesR1C)
                {
                    line += satelliteSystem.at("GLONASS");
                    if (static_cast<int32_t>(observables_iter.second.PRN) < 10)
                        {
                            line += std::string(1, '0');
                        }
                    line += std::to_string(static_cast<int32_t>(observables_iter.second.PRN));
                }
            // Add list of GLONASS L2 satellites
            for (const auto& observables_iter : observablesR2C)
                {
                    line += satelliteSystem.at("GLONASS");
                    if (static_cast<int32_t>(observables_iter.second.PRN) < 10)
                        {
                            line += std::string(1, '0');
                        }
                    line += std::to_string(static_cast<int32_t>(observables_iter.second.PRN));
                }
        }
    line += std::string(80 - line.size(), ' ');
    lengthCheck(line);
    out << line << '\n';

    // -------- OBSERVATION record
    std::string s;
    std::string lineObs;
    for (const auto& observables_iter : observablesG1C)
        {
            lineObs.clear();

            s.assign(1, observables_iter.second.System);
            if (d_version == 3)
                {
                    // Specify system only if in version 3
                    if (s == "G")
                        {
                            line += satelliteSystem.at("GPS");
                        }
                    if (s == "R")
                        {
                            line += satelliteSystem.at("GLONASS");
                        }
                    if (static_cast<int32_t>(observables_iter.second.PRN) < 10)
                        {
                            lineObs += std::string(1, '0');
                        }
                    lineObs += std::to_string(static_cast<int32_t>(observables_iter.second.PRN));
                }

            // Pseudorange Measurements
            lineObs += rightJustify(asString(observables_iter.second.Pseudorange_m, 3), 14);

            // Loss of lock indicator (LLI)
            int32_t lli = 0;  // Include in the observation!!
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += rightJustify(asString<int16_t>(lli), 1);
            //    }

            // Signal Strength Indicator (SSI)
            const int32_t ssi = Rinex_Printer::signalStrength(observables_iter.second.CN0_dB_hz);
            lineObs += rightJustify(asString<int32_t>(ssi), 1);

            // PHASE
            lineObs += rightJustify(asString(observables_iter.second.Carrier_phase_rads / TWO_PI, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += rightJustify(asString<int16_t>(lli), 1);
            //    }
            lineObs += rightJustify(asString<int32_t>(ssi), 1);

            // DOPPLER
            lineObs += rightJustify(asString(observables_iter.second.Carrier_Doppler_hz, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += rightJustify(asString<int16_t>(lli), 1);
            //    }
            lineObs += rightJustify(asString<int32_t>(ssi), 1);

            // SIGNAL STRENGTH
            lineObs += rightJustify(asString(observables_iter.second.CN0_dB_hz, 3), 14);

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }

    for (const auto& available_glo_prn : available_glo_prns)
        {
            lineObs.clear();
            if (d_version == 3)
                {
                    lineObs += satelliteSystem.at("GLONASS");
                    if (static_cast<int32_t>(available_glo_prn) < 10)
                        {
                            lineObs += std::string(1, '0');
                        }
                    lineObs += std::to_string(static_cast<int32_t>(available_glo_prn));
                }
            const auto ret = total_glo_map.equal_range(available_glo_prn);
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    /// \todo Need to account for pseudorange correction for glonass
                    // double leap_seconds = Rinex_Printer::get_leap_second(glonass_gnav_eph, gps_obs_time);
                    lineObs += rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // GLONASS CARRIER PHASE
                    lineObs += rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // GLONASS  DOPPLER
                    lineObs += rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // GLONASS SIGNAL STRENGTH
                    lineObs += rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Gps_CNAV_Ephemeris& gps_eph, const Glonass_Gnav_Ephemeris& /*glonass_gnav_eph*/, double gps_obs_time, const std::map<int32_t, Gnss_Synchro>& observables) const
{
    std::string line;

    // -------- EPOCH record

    // double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    // const double gps_t = eph.sv_clock_correction(obs_time);
    line += std::string(1, '>');

    add_svclk_to_line(Rinex_Printer::compute_GPS_time(gps_eph, gps_obs_time), false, line);
    add_seconds_to_line(fmod(gps_obs_time, 60), line);

    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    // Number of satellites observed in current epoch
    // Get maps with observations
    std::map<int32_t, Gnss_Synchro> observablesG2S;
    std::map<int32_t, Gnss_Synchro> observablesR1C;
    std::map<int32_t, Gnss_Synchro> observablesR2C;

    for (const auto& observables_iter : observables)
        {
            const std::string system_(&observables_iter.second.System, 1);
            const std::string sig_(observables_iter.second.Signal);
            if ((system_ == "R") && (sig_ == "1G"))
                {
                    observablesR1C.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "R") && (sig_ == "2G"))
                {
                    observablesR2C.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "G") && (sig_ == "2S"))
                {
                    observablesG2S.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
        }

    std::multimap<uint32_t, Gnss_Synchro> total_glo_map;
    std::set<uint32_t> available_glo_prns;
    std::set<uint32_t>::iterator it;
    for (const auto& observables_iter : observablesR1C)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_glo_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_glo_prns.find(prn_);
            if (it == available_glo_prns.end())
                {
                    available_glo_prns.insert(prn_);
                }
        }

    for (const auto& observables_iter : observablesR2C)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_glo_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_glo_prns.find(prn_);
            if (it == available_glo_prns.end())
                {
                    available_glo_prns.insert(prn_);
                }
        }

    const int32_t numGloSatellitesObserved = available_glo_prns.size();
    const int32_t numGpsSatellitesObserved = observablesG2S.size();
    const int32_t numSatellitesObserved = numGloSatellitesObserved + numGpsSatellitesObserved;
    line += rightJustify(std::to_string(numSatellitesObserved), 3);

    line += std::string(80 - line.size(), ' ');
    lengthCheck(line);
    out << line << '\n';

    // -------- OBSERVATION record
    std::string s;
    std::string lineObs;
    for (const auto& observables_iter : observablesG2S)
        {
            lineObs.clear();

            s.assign(1, observables_iter.second.System);
            // Specify system only if in version 3
            if (s == "G")
                {
                    line += satelliteSystem.at("GPS");
                }
            if (s == "R")
                {
                    line += satelliteSystem.at("GLONASS");
                }
            if (static_cast<int32_t>(observables_iter.second.PRN) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(observables_iter.second.PRN));

            // Pseudorange Measurements
            lineObs += rightJustify(asString(observables_iter.second.Pseudorange_m, 3), 14);

            // Loss of lock indicator (LLI)
            int32_t lli = 0;  // Include in the observation!!
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += rightJustify(asString<int16_t>(lli), 1);
            //    }

            // Signal Strength Indicator (SSI)
            const int32_t ssi = Rinex_Printer::signalStrength(observables_iter.second.CN0_dB_hz);
            lineObs += rightJustify(asString<int32_t>(ssi), 1);

            // PHASE
            lineObs += rightJustify(asString(observables_iter.second.Carrier_phase_rads / TWO_PI, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += rightJustify(asString<int16_t>(lli), 1);
            //    }
            lineObs += rightJustify(asString<int32_t>(ssi), 1);

            // DOPPLER
            lineObs += rightJustify(asString(observables_iter.second.Carrier_Doppler_hz, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += rightJustify(asString<int16_t>(lli), 1);
            //    }
            lineObs += rightJustify(asString<int32_t>(ssi), 1);

            // SIGNAL STRENGTH
            lineObs += rightJustify(asString(observables_iter.second.CN0_dB_hz, 3), 14);

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }

    std::pair<std::multimap<uint32_t, Gnss_Synchro>::iterator, std::multimap<uint32_t, Gnss_Synchro>::iterator> ret;
    for (const auto& prn : available_glo_prns)
        {
            lineObs.clear();
            lineObs += satelliteSystem.at("GLONASS");
            if (static_cast<int32_t>(prn) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(prn));

            ret = total_glo_map.equal_range(prn);
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    /// \todo Need to account for pseudorange correction for glonass
                    // double leap_seconds = Rinex_Printer::get_leap_second(glonass_gnav_eph, gps_obs_time);
                    lineObs += rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // GLONASS CARRIER PHASE
                    lineObs += rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // GLONASS  DOPPLER
                    lineObs += rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // GLONASS SIGNAL STRENGTH
                    lineObs += rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Galileo_Ephemeris& galileo_eph, const Glonass_Gnav_Ephemeris& /*glonass_gnav_eph*/, double galileo_obs_time, const std::map<int32_t, Gnss_Synchro>& observables) const
{
    std::string line;

    // double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    // const double gps_t = eph.sv_clock_correction(obs_time);

    line += std::string(1, '>');

    add_svclk_to_line(Rinex_Printer::compute_Galileo_time(galileo_eph, galileo_obs_time), false, line);
    add_seconds_to_line(fmod(galileo_obs_time, 60), line);

    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    // Number of satellites observed in current epoch

    // Get maps with observations
    std::map<int32_t, Gnss_Synchro> observablesE1B;
    std::map<int32_t, Gnss_Synchro> observablesR1C;
    std::map<int32_t, Gnss_Synchro> observablesR2C;

    for (const auto& observables_iter : observables)
        {
            const std::string system_(&observables_iter.second.System, 1);
            const std::string sig_(observables_iter.second.Signal);
            if ((system_ == "R") && (sig_ == "1G"))
                {
                    observablesR1C.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "R") && (sig_ == "2G"))
                {
                    observablesR2C.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "E") && (sig_ == "1B"))
                {
                    observablesE1B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
        }

    std::multimap<uint32_t, Gnss_Synchro> total_glo_map;
    std::set<uint32_t> available_glo_prns;
    std::set<uint32_t>::iterator it;
    for (const auto& observables_iter : observablesR1C)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_glo_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_glo_prns.find(prn_);
            if (it == available_glo_prns.end())
                {
                    available_glo_prns.insert(prn_);
                }
        }

    for (const auto& observables_iter : observablesR2C)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_glo_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_glo_prns.find(prn_);
            if (it == available_glo_prns.end())
                {
                    available_glo_prns.insert(prn_);
                }
        }

    const int32_t numGloSatellitesObserved = available_glo_prns.size();
    const int32_t numGalSatellitesObserved = observablesE1B.size();
    const int32_t numSatellitesObserved = numGalSatellitesObserved + numGloSatellitesObserved;
    line += rightJustify(std::to_string(numSatellitesObserved), 3);

    // Receiver clock offset (optional)
    // line += rightJustify(asString(clockOffset, 12), 15);

    line += std::string(80 - line.size(), ' ');
    lengthCheck(line);
    out << line << '\n';

    std::string s;
    std::string lineObs;
    for (const auto& observables_iter : observablesE1B)
        {
            lineObs.clear();

            s.assign(1, observables_iter.second.System);
            if (s == "E")
                {
                    lineObs += satelliteSystem.at("Galileo");
                }
            if (s == "R")
                {
                    line += satelliteSystem.at("GLONASS");
                }
            if (static_cast<int32_t>(observables_iter.second.PRN) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(observables_iter.second.PRN));
            lineObs += rightJustify(asString(observables_iter.second.Pseudorange_m, 3), 14);

            // Loss of lock indicator (LLI)
            int32_t lli = 0;  // Include in the observation!!
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += rightJustify(asString<int16_t>(lli), 1);
            //    }

            // Signal Strength Indicator (SSI)
            const int32_t ssi = Rinex_Printer::signalStrength(observables_iter.second.CN0_dB_hz);
            lineObs += rightJustify(asString<int32_t>(ssi), 1);

            // PHASE
            lineObs += rightJustify(asString(observables_iter.second.Carrier_phase_rads / TWO_PI, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += rightJustify(asString<int16_t>(lli), 1);
            //    }
            lineObs += rightJustify(asString<int32_t>(ssi), 1);

            // DOPPLER
            lineObs += rightJustify(asString(observables_iter.second.Carrier_Doppler_hz, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += rightJustify(asString<int16_t>(lli), 1);
            //    }
            lineObs += rightJustify(asString<int32_t>(ssi), 1);

            // SIGNAL STRENGTH
            lineObs += rightJustify(asString(observables_iter.second.CN0_dB_hz, 3), 14);

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }

    std::pair<std::multimap<uint32_t, Gnss_Synchro>::iterator, std::multimap<uint32_t, Gnss_Synchro>::iterator> ret;
    for (const auto& prn : available_glo_prns)
        {
            lineObs.clear();
            lineObs += satelliteSystem.at("Galileo");
            if (static_cast<int32_t>(prn) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(prn));
            ret = total_glo_map.equal_range(prn);
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    lineObs += rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // GLONASS CARRIER PHASE
                    lineObs += rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // GLONASS  DOPPLER
                    lineObs += rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //   }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // GLONASS SIGNAL STRENGTH
                    lineObs += rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Gps_Ephemeris& eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables) const
{
    // RINEX observations timestamps are GPS timestamps.
    std::string line;

    const boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(eph, obs_time);
    const std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    // double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    // const double gps_t = eph.sv_clock_correction(obs_time);
    const double gps_t = obs_time;

    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);

    if (d_version == 2)
        {
            line.clear();
            const std::string year(timestring, 2, 2);
            line += std::string(1, ' ');
            line += year;
            line += std::string(1, ' ');
            if (month.compare(0, 1, "0") == 0)
                {
                    line += std::string(1, ' ');
                    line += month.substr(1, 1);
                }
            else
                {
                    line += month;
                }
            line += std::string(1, ' ');
            if (day.compare(0, 1, "0") == 0)
                {
                    line += std::string(1, ' ');
                    line += day.substr(1, 1);
                }
            else
                {
                    line += day;
                }
            line += std::string(1, ' ');
            line += hour;
            line += std::string(1, ' ');
            line += minutes;
            line += std::string(1, ' ');
            const double second_ = fmod(gps_t, 60);
            if (second_ < 10)
                {
                    line += std::string(1, ' ');
                }
            line += asString(second_, 7);
            line += std::string(2, ' ');
            // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
            line += std::string(1, '0');
            // Number of satellites observed in current epoch
            const int32_t numSatellitesObserved = observables.size();
            line += rightJustify(std::to_string(numSatellitesObserved), 3);
            for (const auto& observables_iter : observables)
                {
                    line += satelliteSystem.at("GPS");
                    if (static_cast<int32_t>(observables_iter.second.PRN) < 10)
                        {
                            line += std::string(1, '0');
                        }
                    line += std::to_string(static_cast<int32_t>(observables_iter.second.PRN));
                }
            // Receiver clock offset (optional)
            // line += rightJustify(asString(clockOffset, 12), 15);
            line += std::string(80 - line.size(), ' ');
            lengthCheck(line);
            out << line << '\n';

            for (const auto& observables_iter : observables)
                {
                    std::string lineObs;
                    lineObs.clear();
                    line.clear();
                    // GPS L1 PSEUDORANGE
                    line += std::string(2, ' ');
                    lineObs += rightJustify(asString(observables_iter.second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(observables_iter.second.CN0_dB_hz);
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);
                    // GPS L1 CA PHASE
                    lineObs += rightJustify(asString(observables_iter.second.Carrier_phase_rads / TWO_PI, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);
                    // GPS L1 CA DOPPLER
                    lineObs += rightJustify(asString(observables_iter.second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //       lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //   }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);
                    // GPS L1 SIGNAL STRENGTH
                    lineObs += rightJustify(asString(observables_iter.second.CN0_dB_hz, 3), 14);
                    if (lineObs.size() < 80)
                        {
                            lineObs += std::string(80 - lineObs.size(), ' ');
                        }
                    out << lineObs << '\n';
                }
        }

    if (d_version == 3)
        {
            const std::string year(timestring, 0, 4);
            line += std::string(1, '>');
            line += std::string(1, ' ');
            line += year;
            line += std::string(1, ' ');
            line += month;
            line += std::string(1, ' ');
            line += day;
            line += std::string(1, ' ');
            line += hour;
            line += std::string(1, ' ');
            line += minutes;

            line += std::string(1, ' ');
            const double seconds = fmod(gps_t, 60);
            // Add extra 0 if seconds are < 10
            if (seconds < 10)
                {
                    line += std::string(1, '0');
                }
            line += asString(seconds, 7);
            line += std::string(2, ' ');
            // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
            line += std::string(1, '0');

            // Number of satellites observed in current epoch
            const int32_t numSatellitesObserved = observables.size();
            line += rightJustify(std::to_string(numSatellitesObserved), 3);

            // Receiver clock offset (optional)
            // line += rightJustify(asString(clockOffset, 12), 15);

            line += std::string(80 - line.size(), ' ');
            lengthCheck(line);
            out << line << '\n';

            for (const auto& observables_iter : observables)
                {
                    std::string lineObs;
                    lineObs.clear();
                    lineObs += satelliteSystem.at("GPS");
                    if (static_cast<int32_t>(observables_iter.second.PRN) < 10)
                        {
                            lineObs += std::string(1, '0');
                        }
                    lineObs += std::to_string(static_cast<int32_t>(observables_iter.second.PRN));
                    // lineObs += std::string(2, ' ');
                    lineObs += rightJustify(asString(observables_iter.second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(observables_iter.second.CN0_dB_hz);
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // GPS L1 CA PHASE
                    lineObs += rightJustify(asString(observables_iter.second.Carrier_phase_rads / TWO_PI, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // GPS L1 CA DOPPLER
                    lineObs += rightJustify(asString(observables_iter.second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }

                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // GPS L1 SIGNAL STRENGTH
                    lineObs += rightJustify(asString(observables_iter.second.CN0_dB_hz, 3), 14);

                    if (lineObs.size() < 80)
                        {
                            lineObs += std::string(80 - lineObs.size(), ' ');
                        }
                    out << lineObs << '\n';
                }
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Gps_CNAV_Ephemeris& eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables) const
{
    // RINEX observations timestamps are GPS timestamps.
    std::string line;

    // double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    // const double gps_t = eph.sv_clock_correction(obs_time);

    line += std::string(1, '>');

    add_svclk_to_line(Rinex_Printer::compute_GPS_time(eph, obs_time), false, line);
    add_seconds_to_line(fmod(obs_time, 60), line);

    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    // Number of satellites observed in current epoch
    const int32_t numSatellitesObserved = observables.size();
    line += rightJustify(std::to_string(numSatellitesObserved), 3);

    // Receiver clock offset (optional)
    // line += rightJustify(asString(clockOffset, 12), 15);

    line += std::string(80 - line.size(), ' ');
    lengthCheck(line);
    out << line << '\n';

    for (const auto& observables_iter : observables)
        {
            std::string lineObs;
            lineObs.clear();
            lineObs += satelliteSystem.at("GPS");
            if (static_cast<int32_t>(observables_iter.second.PRN) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(observables_iter.second.PRN));
            // lineObs += std::string(2, ' ');
            // GPS L2 PSEUDORANGE
            lineObs += rightJustify(asString(observables_iter.second.Pseudorange_m, 3), 14);

            // Loss of lock indicator (LLI)
            int32_t lli = 0;  // Include in the observation!!
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //       lineObs += rightJustify(asString<int16_t>(lli), 1);
            //   }

            // Signal Strength Indicator (SSI)
            const int32_t ssi = Rinex_Printer::signalStrength(observables_iter.second.CN0_dB_hz);
            lineObs += rightJustify(asString<int32_t>(ssi), 1);

            // GPS L2 PHASE
            lineObs += rightJustify(asString(observables_iter.second.Carrier_phase_rads / TWO_PI, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += rightJustify(asString<int16_t>(lli), 1);
            //    }
            lineObs += rightJustify(asString<int32_t>(ssi), 1);

            // GPS L2 DOPPLER
            lineObs += rightJustify(asString(observables_iter.second.Carrier_Doppler_hz, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += rightJustify(asString<int16_t>(lli), 1);
            //   }

            lineObs += rightJustify(asString<int32_t>(ssi), 1);

            // GPS L2 SIGNAL STRENGTH
            lineObs += rightJustify(asString(observables_iter.second.CN0_dB_hz, 3), 14);

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Gps_Ephemeris& eph, const Gps_CNAV_Ephemeris& /*eph_cnav*/, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables, bool triple_band) const
{
    // RINEX observations timestamps are GPS timestamps.
    std::string line;

    // double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    // const double gps_t = eph.sv_clock_correction(obs_time);
    line += std::string(1, '>');

    add_svclk_to_line(Rinex_Printer::compute_GPS_time(eph, obs_time), false, line);
    add_seconds_to_line(fmod(obs_time, 60), line);

    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    // Number of satellites observed in current epoch

    // Get maps with GPS L1 and L2 observations
    std::map<int32_t, Gnss_Synchro> observablesL1;
    std::map<int32_t, Gnss_Synchro> observablesL2;
    std::map<int32_t, Gnss_Synchro> observablesL5;

    std::multimap<uint32_t, Gnss_Synchro> total_mmap;
    std::multimap<uint32_t, Gnss_Synchro>::iterator mmap_iter;
    for (const auto& observables_iter : observables)
        {
            const std::string system_(&observables_iter.second.System, 1);
            const std::string sig_(observables_iter.second.Signal);
            if ((system_ == "G") && (sig_ == "1C"))
                {
                    observablesL1.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                    total_mmap.insert(std::pair<uint32_t, Gnss_Synchro>(observables_iter.second.PRN, observables_iter.second));
                }
            if ((system_ == "G") && (sig_ == "2S"))
                {
                    observablesL2.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                    mmap_iter = total_mmap.find(observables_iter.second.PRN);
                    if (mmap_iter == total_mmap.end())
                        {
                            Gnss_Synchro gs = Gnss_Synchro();
                            total_mmap.insert(std::pair<uint32_t, Gnss_Synchro>(observables_iter.second.PRN, gs));
                        }
                    total_mmap.insert(std::pair<uint32_t, Gnss_Synchro>(observables_iter.second.PRN, observables_iter.second));
                }

            if ((system_ == "G") && (sig_ == "L5"))
                {
                    observablesL5.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                    mmap_iter = total_mmap.find(observables_iter.second.PRN);
                    if (mmap_iter == total_mmap.end())
                        {
                            Gnss_Synchro gs = Gnss_Synchro();
                            total_mmap.insert(std::pair<uint32_t, Gnss_Synchro>(observables_iter.second.PRN, gs));
                        }
                    total_mmap.insert(std::pair<uint32_t, Gnss_Synchro>(observables_iter.second.PRN, observables_iter.second));
                }
        }

    // Fill with zeros satellites with L1 obs but not L2
    std::multimap<uint32_t, Gnss_Synchro> mmap_aux;
    mmap_aux = total_mmap;
    for (const auto& mmap_iter : mmap_aux)
        {
            if ((total_mmap.count(mmap_iter.second.PRN)) == 1 && (mmap_iter.second.PRN != 0))
                {
                    Gnss_Synchro gs = Gnss_Synchro();
                    gs.System = 'G';
                    gs.Signal[0] = '2';
                    gs.Signal[1] = 'S';
                    gs.Signal[2] = '\0';
                    gs.PRN = mmap_iter.second.PRN;
                    total_mmap.insert(std::pair<uint32_t, Gnss_Synchro>(mmap_iter.second.PRN, gs));
                }
        }

    std::set<uint32_t> available_prns;
    std::set<uint32_t>::iterator it;
    for (const auto& observables_iter : observablesL1)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            it = available_prns.find(prn_);
            if (it == available_prns.end())
                {
                    available_prns.insert(prn_);
                }
        }

    for (const auto& observables_iter : observablesL2)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            it = available_prns.find(prn_);
            if (it == available_prns.end())
                {
                    available_prns.insert(prn_);
                }
        }

    for (const auto& observables_iter : observablesL5)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            it = available_prns.find(prn_);
            if (it == available_prns.end())
                {
                    available_prns.insert(prn_);
                }
        }

    const int32_t numSatellitesObserved = available_prns.size();
    line += rightJustify(std::to_string(numSatellitesObserved), 3);
    // Receiver clock offset (optional)
    // line += rightJustify(asString(clockOffset, 12), 15);
    line += std::string(80 - line.size(), ' ');
    lengthCheck(line);
    out << line << '\n';

    std::string lineObs;
    for (const auto& available_prn : available_prns)
        {
            lineObs.clear();
            lineObs += satelliteSystem.at("GPS");
            if (static_cast<int32_t>(available_prn) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(available_prn));
            const auto ret = total_mmap.equal_range(available_prn);
            bool have_l2 = false;
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    const std::string sig_(iter->second.Signal);
                    if (sig_ == "2S")
                        {
                            have_l2 = true;
                        }
                    if (triple_band && sig_ == "L5" && have_l2 == false)
                        {
                            lineObs += std::string(62, ' ');
                        }

                    lineObs += rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //   {
                    //       lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //   }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // GPS CARRIER PHASE
                    lineObs += rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // GPS  DOPPLER
                    lineObs += rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // GPS SIGNAL STRENGTH
                    lineObs += rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Galileo_Ephemeris& eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables, const std::string& galileo_bands) const
{
    // RINEX observations timestamps are Galileo timestamps.
    // See https://gage.upc.edu/sites/default/files/gLAB/HTML/Observation_Rinex_v3.01.html
    std::string line;

    // double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    // const double gps_t = eph.sv_clock_correction(obs_time);
    line += std::string(1, '>');

    add_svclk_to_line(Rinex_Printer::compute_Galileo_time(eph, obs_time), false, line);
    add_seconds_to_line(fmod(obs_time, 60), line);

    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    // Number of satellites observed in current epoch

    // Get maps with Galileo observations
    std::map<int32_t, Gnss_Synchro> observablesE1B;
    std::map<int32_t, Gnss_Synchro> observablesE5A;
    std::map<int32_t, Gnss_Synchro> observablesE5B;
    std::map<int32_t, Gnss_Synchro> observablesE6B;

    for (const auto& observables_iter : observables)
        {
            const std::string system_(&observables_iter.second.System, 1);
            const std::string sig_(observables_iter.second.Signal);
            if ((system_ == "E") && (sig_ == "1B"))
                {
                    observablesE1B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "E") && (sig_ == "5X"))
                {
                    observablesE5A.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "E") && (sig_ == "7X"))
                {
                    observablesE5B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "E") && (sig_ == "E6"))
                {
                    observablesE6B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
        }
    const std::size_t found_1B = galileo_bands.find("1B");
    const std::size_t found_E5a = galileo_bands.find("5X");
    const std::size_t found_E5b = galileo_bands.find("7X");
    const std::size_t found_E6b = galileo_bands.find("E6");

    std::multimap<uint32_t, Gnss_Synchro> total_map;
    std::set<uint32_t> available_prns;
    std::set<uint32_t>::iterator it;
    if (found_1B != std::string::npos)
        {
            for (const auto& observables_iter : observablesE1B)
                {
                    const uint32_t prn_ = observables_iter.second.PRN;
                    total_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
                    it = available_prns.find(prn_);
                    if (it == available_prns.end())
                        {
                            available_prns.insert(prn_);
                        }
                }
        }
    if (found_E5a != std::string::npos)
        {
            for (const auto& observables_iter : observablesE5A)
                {
                    const uint32_t prn_ = observables_iter.second.PRN;
                    it = available_prns.find(prn_);
                    if (it == available_prns.end())
                        {
                            available_prns.insert(prn_);
                            if (found_1B != std::string::npos)
                                {
                                    Gnss_Synchro gs = Gnss_Synchro();
                                    gs.System = 'E';
                                    gs.Signal[0] = '1';
                                    gs.Signal[1] = 'B';
                                    gs.Signal[2] = '\0';
                                    gs.PRN = prn_;
                                    total_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, gs));
                                }
                        }
                    total_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
                }
        }
    if (found_E5b != std::string::npos)
        {
            for (const auto& observables_iter : observablesE5B)
                {
                    const uint32_t prn_ = observables_iter.second.PRN;
                    it = available_prns.find(prn_);
                    if (it == available_prns.end())
                        {
                            available_prns.insert(prn_);
                            if (found_1B != std::string::npos)
                                {
                                    Gnss_Synchro gs = Gnss_Synchro();
                                    gs.System = 'E';
                                    gs.Signal[0] = '1';
                                    gs.Signal[1] = 'B';
                                    gs.Signal[2] = '\0';
                                    gs.PRN = prn_;
                                    total_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, gs));
                                }
                        }
                    total_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
                }
        }

    if (found_E6b != std::string::npos)
        {
            for (const auto& observables_iter : observablesE6B)
                {
                    const uint32_t prn_ = observables_iter.second.PRN;
                    total_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
                    it = available_prns.find(prn_);
                    if (it == available_prns.end())
                        {
                            available_prns.insert(prn_);
                        }
                }
        }

    const int32_t numSatellitesObserved = available_prns.size();
    line += rightJustify(std::to_string(numSatellitesObserved), 3);
    // Receiver clock offset (optional)
    // line += rightJustify(asString(clockOffset, 12), 15);
    line += std::string(80 - line.size(), ' ');
    lengthCheck(line);
    out << line << '\n';

    std::string lineObs;
    for (const auto& available_prn : available_prns)
        {
            lineObs.clear();
            lineObs += satelliteSystem.at("Galileo");
            if (static_cast<int32_t>(available_prn) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(available_prn));
            const auto ret = total_map.equal_range(available_prn);
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    lineObs += rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //   }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // Galileo CARRIER PHASE
                    lineObs += rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // Galileo  DOPPLER
                    lineObs += rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //       lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // Galileo SIGNAL STRENGTH
                    lineObs += rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Gps_Ephemeris& gps_eph, const Galileo_Ephemeris& /*galileo_eph*/, double gps_obs_time, const std::map<int32_t, Gnss_Synchro>& observables) const
{
    std::string line;

    // double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    // double gps_t = eph.sv_clock_correction(obs_time);

    line += std::string(1, '>');

    add_svclk_to_line(Rinex_Printer::compute_GPS_time(gps_eph, gps_obs_time), false, line);
    add_seconds_to_line(fmod(gps_obs_time, 60), line);

    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    // Number of satellites observed in current epoch

    // Get maps with observations
    std::map<int32_t, Gnss_Synchro> observablesG1C;
    std::map<int32_t, Gnss_Synchro> observablesE1B;
    std::map<int32_t, Gnss_Synchro> observablesE5A;
    std::map<int32_t, Gnss_Synchro> observablesE5B;
    std::map<int32_t, Gnss_Synchro> observablesE6B;

    for (const auto& observables_iter : observables)
        {
            const std::string system_(&observables_iter.second.System, 1);
            const std::string sig_(observables_iter.second.Signal);
            if ((system_ == "E") && (sig_ == "1B"))
                {
                    observablesE1B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "E") && (sig_ == "5X"))
                {
                    observablesE5A.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "E") && (sig_ == "7X"))
                {
                    observablesE5B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "E") && (sig_ == "E6"))
                {
                    observablesE6B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "G") && (sig_ == "1C"))
                {
                    observablesG1C.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
        }

    std::multimap<uint32_t, Gnss_Synchro> total_gal_map;
    std::set<uint32_t> available_gal_prns;
    std::set<uint32_t>::iterator it;
    for (const auto& observables_iter : observablesE1B)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (const auto& observables_iter : observablesE5A)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (const auto& observables_iter : observablesE5B)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (const auto& observables_iter : observablesE6B)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    const int32_t numGalSatellitesObserved = available_gal_prns.size();
    const int32_t numGpsSatellitesObserved = observablesG1C.size();
    const int32_t numSatellitesObserved = numGalSatellitesObserved + numGpsSatellitesObserved;
    line += rightJustify(std::to_string(numSatellitesObserved), 3);

    // Receiver clock offset (optional)
    // line += rightJustify(asString(clockOffset, 12), 15);

    line += std::string(80 - line.size(), ' ');
    lengthCheck(line);
    out << line << '\n';

    std::string s;
    std::string lineObs;

    for (const auto& observables_iter : observablesG1C)
        {
            lineObs.clear();

            s.assign(1, observables_iter.second.System);
            if (s == "G")
                {
                    lineObs += satelliteSystem.at("GPS");
                }
            if (s == "E")
                {
                    lineObs += satelliteSystem.at("Galileo");
                }
            if (static_cast<int32_t>(observables_iter.second.PRN) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(observables_iter.second.PRN));
            lineObs += rightJustify(asString(observables_iter.second.Pseudorange_m, 3), 14);

            // Loss of lock indicator (LLI)
            int32_t lli = 0;  // Include in the observation!!
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //       lineObs += rightJustify(asString<int16_t>(lli), 1);
            //    }

            // Signal Strength Indicator (SSI)
            const int32_t ssi = Rinex_Printer::signalStrength(observables_iter.second.CN0_dB_hz);
            lineObs += rightJustify(asString<int32_t>(ssi), 1);

            // PHASE
            lineObs += rightJustify(asString(observables_iter.second.Carrier_phase_rads / TWO_PI, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += rightJustify(asString<int16_t>(lli), 1);
            //   }
            lineObs += rightJustify(asString<int32_t>(ssi), 1);

            // DOPPLER
            lineObs += rightJustify(asString(observables_iter.second.Carrier_Doppler_hz, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += rightJustify(asString<int16_t>(lli), 1);
            //    }
            lineObs += rightJustify(asString<int32_t>(ssi), 1);

            // SIGNAL STRENGTH
            lineObs += rightJustify(asString(observables_iter.second.CN0_dB_hz, 3), 14);

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }

    std::pair<std::multimap<uint32_t, Gnss_Synchro>::iterator, std::multimap<uint32_t, Gnss_Synchro>::iterator> ret;
    for (const auto& prn : available_gal_prns)
        {
            lineObs.clear();
            lineObs += satelliteSystem.at("Galileo");
            if (static_cast<int32_t>(prn) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(prn));
            ret = total_gal_map.equal_range(prn);
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    lineObs += rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // Galileo CARRIER PHASE
                    lineObs += rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // Galileo  DOPPLER
                    lineObs += rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // Galileo SIGNAL STRENGTH
                    lineObs += rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Gps_CNAV_Ephemeris& eph, const Galileo_Ephemeris& /*galileo_eph*/, double gps_obs_time, const std::map<int32_t, Gnss_Synchro>& observables) const
{
    std::string line;

    // double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    // double gps_t = eph.sv_clock_correction(obs_time);

    line += std::string(1, '>');

    add_svclk_to_line(Rinex_Printer::compute_GPS_time(eph, gps_obs_time), false, line);
    add_seconds_to_line(fmod(gps_obs_time, 60), line);

    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    // Number of satellites observed in current epoch

    // Get maps with observations
    std::map<int32_t, Gnss_Synchro> observablesG2S;
    std::map<int32_t, Gnss_Synchro> observablesGL5;
    std::map<int32_t, Gnss_Synchro> observablesE1B;
    std::map<int32_t, Gnss_Synchro> observablesE5A;
    std::map<int32_t, Gnss_Synchro> observablesE5B;
    std::map<int32_t, Gnss_Synchro> observablesE6B;

    for (const auto& observables_iter : observables)
        {
            const std::string system_(&observables_iter.second.System, 1);
            const std::string sig_(observables_iter.second.Signal);
            if ((system_ == "E") && (sig_ == "1B"))
                {
                    observablesE1B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "E") && (sig_ == "5X"))
                {
                    observablesE5A.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "E") && (sig_ == "7X"))
                {
                    observablesE5B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "E") && (sig_ == "E6"))
                {
                    observablesE6B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "G") && (sig_ == "2S"))
                {
                    observablesG2S.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "G") && (sig_ == "L5"))
                {
                    observablesGL5.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
        }

    std::multimap<uint32_t, Gnss_Synchro> total_gps_map;
    std::multimap<uint32_t, Gnss_Synchro> total_gal_map;
    std::set<uint32_t> available_gal_prns;
    std::set<uint32_t> available_gps_prns;
    std::set<uint32_t>::iterator it;
    for (const auto& observables_iter : observablesE1B)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (const auto& observables_iter : observablesE5A)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (const auto& observables_iter : observablesE5B)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (const auto& observables_iter : observablesE6B)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (const auto& observables_iter : observablesG2S)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_gps_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_gps_prns.find(prn_);
            if (it == available_gps_prns.end())
                {
                    available_gps_prns.insert(prn_);
                }
        }

    for (const auto& observables_iter : observablesGL5)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_gps_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_gps_prns.find(prn_);
            if (it == available_gps_prns.end())
                {
                    available_gps_prns.insert(prn_);
                }
        }

    const int32_t numGalSatellitesObserved = available_gal_prns.size();
    const int32_t numGpsSatellitesObserved = available_gps_prns.size();
    const int32_t numSatellitesObserved = numGalSatellitesObserved + numGpsSatellitesObserved;
    line += rightJustify(std::to_string(numSatellitesObserved), 3);

    // Receiver clock offset (optional)
    // line += rightJustify(asString(clockOffset, 12), 15);

    line += std::string(80 - line.size(), ' ');
    lengthCheck(line);
    out << line << '\n';

    std::string s;
    std::string lineObs;

    std::pair<std::multimap<uint32_t, Gnss_Synchro>::iterator, std::multimap<uint32_t, Gnss_Synchro>::iterator> ret;
    for (const auto& prn : available_gps_prns)
        {
            lineObs.clear();
            lineObs += satelliteSystem.at("GPS");
            if (static_cast<int32_t>(prn) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(prn));
            ret = total_gps_map.equal_range(prn);
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    lineObs += rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // CARRIER PHASE
                    lineObs += rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    //  DOPPLER
                    lineObs += rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // SIGNAL STRENGTH
                    lineObs += rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            out << lineObs << '\n';
        }

    for (const auto& prn : available_gal_prns)
        {
            lineObs.clear();
            lineObs += satelliteSystem.at("Galileo");
            if (static_cast<int32_t>(prn) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(prn));
            ret = total_gal_map.equal_range(prn);
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    lineObs += rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // Galileo CARRIER PHASE
                    lineObs += rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // Galileo  DOPPLER
                    lineObs += rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // Galileo SIGNAL STRENGTH
                    lineObs += rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            // if (lineObs.size() < 80) lineObs += std::string(80 - lineObs.size(), ' ');
            out << lineObs << '\n';
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Gps_Ephemeris& gps_eph, const Gps_CNAV_Ephemeris& /*gps_cnav_eph*/, const Galileo_Ephemeris& /*galileo_eph*/, double gps_obs_time, const std::map<int32_t, Gnss_Synchro>& observables, bool triple_band) const
{
    std::string line;

    // double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    // double gps_t = eph.sv_clock_correction(obs_time);

    line += std::string(1, '>');

    add_svclk_to_line(Rinex_Printer::compute_GPS_time(gps_eph, gps_obs_time), false, line);
    add_seconds_to_line(fmod(gps_obs_time, 60), line);

    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    // Number of satellites observed in current epoch

    // Get maps with observations
    std::map<int32_t, Gnss_Synchro> observablesG2S;
    std::map<int32_t, Gnss_Synchro> observablesGL5;
    std::map<int32_t, Gnss_Synchro> observablesG1C;
    std::map<int32_t, Gnss_Synchro> observablesE1B;
    std::map<int32_t, Gnss_Synchro> observablesE5A;
    std::map<int32_t, Gnss_Synchro> observablesE5B;
    std::map<int32_t, Gnss_Synchro> observablesE6B;

    for (const auto& observables_iter : observables)
        {
            const std::string system_(&observables_iter.second.System, 1);
            const std::string sig_(observables_iter.second.Signal);
            if ((system_ == "E") && (sig_ == "1B"))
                {
                    observablesE1B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "E") && (sig_ == "5X"))
                {
                    observablesE5A.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "E") && (sig_ == "7X"))
                {
                    observablesE5B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "E") && (sig_ == "E6"))
                {
                    observablesE6B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "G") && (sig_ == "2S"))
                {
                    observablesG2S.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "G") && (sig_ == "L5"))
                {
                    observablesGL5.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "G") && (sig_ == "1C"))
                {
                    observablesG1C.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
        }

    std::multimap<uint32_t, Gnss_Synchro> total_gps_map;
    std::multimap<uint32_t, Gnss_Synchro> total_gal_map;
    std::set<uint32_t> available_gal_prns;
    std::set<uint32_t> available_gps_prns;
    std::set<uint32_t>::iterator it;
    for (const auto& observables_iter : observablesE1B)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (const auto& observables_iter : observablesE5A)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (const auto& observables_iter : observablesE5B)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (const auto& observables_iter : observablesE6B)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (const auto& observables_iter : observablesG1C)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_gps_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_gps_prns.find(prn_);
            if (it == available_gps_prns.end())
                {
                    available_gps_prns.insert(prn_);
                }
        }

    for (const auto& observables_iter : observablesG2S)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_gps_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_gps_prns.find(prn_);
            if (it == available_gps_prns.end())
                {
                    available_gps_prns.insert(prn_);
                }
        }

    for (const auto& observables_iter : observablesGL5)
        {
            const uint32_t prn_ = observables_iter.second.PRN;
            total_gps_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
            it = available_gps_prns.find(prn_);
            if (it == available_gps_prns.end())
                {
                    available_gps_prns.insert(prn_);
                }
        }

    const int32_t numGalSatellitesObserved = available_gal_prns.size();
    const int32_t numGpsSatellitesObserved = available_gps_prns.size();
    const int32_t numSatellitesObserved = numGalSatellitesObserved + numGpsSatellitesObserved;
    line += rightJustify(std::to_string(numSatellitesObserved), 3);

    // Receiver clock offset (optional)
    // line += rightJustify(asString(clockOffset, 12), 15);

    line += std::string(80 - line.size(), ' ');
    lengthCheck(line);
    out << line << '\n';

    std::string s;
    std::string lineObs;

    std::pair<std::multimap<uint32_t, Gnss_Synchro>::iterator, std::multimap<uint32_t, Gnss_Synchro>::iterator> ret;
    for (const auto& prn : available_gps_prns)
        {
            lineObs.clear();
            lineObs += satelliteSystem.at("GPS");
            if (static_cast<int32_t>(prn) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(prn));
            ret = total_gps_map.equal_range(prn);
            bool have_l2 = false;
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    const std::string sig_(iter->second.Signal);
                    if (sig_ == "2S")
                        {
                            have_l2 = true;
                        }
                    if (triple_band && sig_ == "L5" && have_l2 == false)
                        {
                            lineObs += std::string(62, ' ');
                        }

                    lineObs += rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // CARRIER PHASE
                    lineObs += rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    //  DOPPLER
                    lineObs += rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // SIGNAL STRENGTH
                    lineObs += rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            out << lineObs << '\n';
        }

    for (const auto& prn : available_gal_prns)
        {
            lineObs.clear();
            lineObs += satelliteSystem.at("Galileo");
            if (static_cast<int32_t>(prn) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(prn));
            ret = total_gal_map.equal_range(prn);
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    lineObs += rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // Galileo CARRIER PHASE
                    lineObs += rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // Galileo  DOPPLER
                    lineObs += rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += rightJustify(asString<int16_t>(lli), 1);
                    //    }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // Galileo SIGNAL STRENGTH
                    lineObs += rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            // if (lineObs.size() < 80) lineObs += std::string(80 - lineObs.size(), ' ');
            out << lineObs << '\n';
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Beidou_Dnav_Ephemeris& eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables, const std::string& bds_bands) const
{
    std::string line;

    // double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    // double gps_t = eph.sv_clock_correction(obs_time);

    line += std::string(1, '>');

    add_svclk_to_line(Rinex_Printer::compute_BDS_time(eph, obs_time), false, line);
    add_seconds_to_line(fmod(obs_time, 60), line);

    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    // Number of satellites observed in current epoch

    // Get maps with BeiDou observations
    std::map<int32_t, Gnss_Synchro> observablesB1I;
    std::map<int32_t, Gnss_Synchro> observablesB3I;


    for (const auto& observables_iter : observables)
        {
            const std::string system_(&observables_iter.second.System, 1);
            const std::string sig_(observables_iter.second.Signal);
            if ((system_ == "C") && (sig_ == "B1"))
                {
                    observablesB1I.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
            if ((system_ == "C") && (sig_ == "B3"))
                {
                    observablesB3I.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter.first, observables_iter.second));
                }
        }
    const std::size_t found_B1 = bds_bands.find("B1");
    const std::size_t found_B3 = bds_bands.find("B3");

    std::multimap<uint32_t, Gnss_Synchro> total_map;
    std::set<uint32_t> available_prns;
    std::set<uint32_t>::iterator it;
    if (found_B1 != std::string::npos)
        {
            for (const auto& observables_iter : observablesB1I)
                {
                    const uint32_t prn_ = observables_iter.second.PRN;
                    total_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
                    it = available_prns.find(prn_);
                    if (it == available_prns.end())
                        {
                            available_prns.insert(prn_);
                        }
                }
        }
    if (found_B3 != std::string::npos)
        {
            for (const auto& observables_iter : observablesB3I)
                {
                    const uint32_t prn_ = observables_iter.second.PRN;
                    it = available_prns.find(prn_);
                    if (it == available_prns.end())
                        {
                            available_prns.insert(prn_);
                            if (found_B1 != std::string::npos)
                                {
                                    Gnss_Synchro gs = Gnss_Synchro();
                                    gs.System = 'C';
                                    gs.Signal[0] = 'B';
                                    gs.Signal[1] = '1';
                                    gs.Signal[2] = '\0';
                                    gs.PRN = prn_;
                                    total_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, gs));
                                }
                        }
                    total_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter.second));
                }
        }

    const int32_t numSatellitesObserved = available_prns.size();
    line += rightJustify(std::to_string(numSatellitesObserved), 3);
    // Receiver clock offset (optional)
    // line += rightJustify(asString(clockOffset, 12), 15);
    line += std::string(80 - line.size(), ' ');
    lengthCheck(line);
    out << line << '\n';

    std::string lineObs;
    for (const auto& available_prn : available_prns)
        {
            lineObs.clear();
            lineObs += satelliteSystem.at("Beidou");
            if (static_cast<int32_t>(available_prn) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(available_prn));
            const auto ret = total_map.equal_range(available_prn);
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    lineObs += rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    // CARRIER PHASE
                    lineObs += rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    //  DOPPLER
                    lineObs += rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    lineObs += rightJustify(asString<int32_t>(ssi), 1);

                    //  SIGNAL STRENGTH
                    lineObs += rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }
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


// void Rinex_Printer::log_rinex_sbs(std::fstream& out, const Sbas_Raw_Msg& sbs_message)
// {
//    // line 1: PRN / EPOCH / RCVR
//    std::stringstream line1;
//
//    // SBAS PRN
//    line1 << sbs_message.get_prn();
//    line1 << " ";
//
//    // gps time of reception
//    int32_t gps_week;
//    double gps_sec;
//    if(sbs_message.get_rx_time_obj().get_gps_time(gps_week, gps_sec))
//        {
//            int32_t year;
//            int32_t month;
//            int32_t day;
//            int32_t hour;
//            int32_t minute;
//            int32_t second;
//
//            double gps_sec_one_digit_precicion = round(gps_sec *10)/10; // to prevent rounding towards 60.0sec in the stream output
//            int32_t gps_tow = trunc(gps_sec_one_digit_precicion);
//            double sub_sec = gps_sec_one_digit_precicion - double(gps_tow);
//
//            to_date_time(gps_week, gps_tow, year, month, day, hour, minute, second);
//            line1 << asFixWidthString(year, 2, '0') <<  " " << asFixWidthString(month, 2, '0') <<  " " << asFixWidthString(day, 2, '0') <<  " " << asFixWidthString(hour, 2, '0') <<  " " << asFixWidthString(minute, 2, '0') <<  " " << rightJustify(asString(double(second)+sub_sec,1),4,' ');
//        }
//    else
//        {
//            line1 << std::string(19, ' ');
//        }
//    line1 << "  ";
//
//    // band
//    line1 << "L1";
//    line1 << "   ";
//
//    // Length of data message (bytes)
//    line1 <<  asFixWidthString(sbs_message.get_msg().size(), 3, ' ');
//    line1 << "   ";
//    // File-internal receiver index
//    line1 << "  0";
//    line1 << "   ";
//    // Transmission System Identifier
//    line1 << "SBA";
//    line1 << std::string(35, ' ');
//    lengthCheck(line1.str());
//    out << line1.str() << '\n';
//
//    // DATA RECORD - 1
//    std::stringstream line2;
//    line2 << " ";
//    // Message frame identifier
//    if (sbs_message.get_msg_type() < 10) line2 << " ";
//    line2 << sbs_message.get_msg_type();
//    line2 << std::string(4, ' ');
//    // First 18 bytes of message (hex)
//    std::vector<unsigned char> msg = sbs_message.get_msg();
//    for (size_t i = 0; i < 18 && i <  msg.size(); ++i)
//        {
//            line2 << std::hex << std::setfill('0') << std::setw(2);
//            line2 << int(msg[i]) << " ";
//        }
//    line2 << std::string(19, ' ');
//    lengthCheck(line2.str());
//    out << line2.str() << '\n';
//
//    // DATA RECORD - 2
//    std::stringstream line3;
//    line3 << std::string(7, ' ');
//    // Remaining bytes of message (hex)
//    for (size_t i = 18; i < 36 && i <  msg.size(); ++i)
//        {
//            line3 << std::hex << std::setfill('0') << std::setw(2);
//            line3 << int(msg[i]) << " ";
//        }
//    line3 << std::string(31, ' ');
//    lengthCheck(line3.str());
//    out << line3.str() << '\n';
// }


int32_t Rinex_Printer::signalStrength(double snr) const
{
    auto ss = static_cast<int32_t>(std::min(std::max(static_cast<int32_t>(floor(snr / 6)), 1), 9));
    return ss;
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
    const boost::posix_time::time_duration t = boost::posix_time::milliseconds(static_cast<int64_t>((obs_time + 604800 * static_cast<double>(eph.WN % 1024)) * 1000));
    boost::posix_time::ptime p_time(boost::gregorian::date(1999, 8, 22), t);
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


double Rinex_Printer::get_leap_second(const Glonass_Gnav_Ephemeris& eph, double gps_obs_time) const
{
    double tod = 0.0;
    const double glot2utc = 3 * 3600;
    double obs_time_glot = 0.0;
    int32_t i = 0;
    double leap_second = 0;
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
    obs_time_glot = gps_obs_time + glot2utc;

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
                    // We subtract the leap second when going from gpst to utc
                    leap_second = fabs(GLONASS_LEAP_SECONDS[i][6]);
                    break;
                }
        }

    return leap_second;
}


void Rinex_Printer::set_pre_2009_file(bool pre_2009_file)
{
    d_pre_2009_file = pre_2009_file;
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
