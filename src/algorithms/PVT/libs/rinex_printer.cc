/*!
 * \file rinex_printer.cc
 * \brief Implementation of a RINEX 2.11 / 3.02 printer
 * See http://igscb.jpl.nasa.gov/igscb/data/format/rinex302.pdf
 * \author Carles Fernandez Prades, 2011. cfernandez(at)cttc.es
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

#include "rinex_printer.h"
#include <unistd.h>  // for getlogin_r()
#include <algorithm> // for min and max
#include <cmath>     // for floor
#include <cstdlib>   // for getenv()
#include <iterator>
#include <ostream>
#include <set>
#include <utility>
#include <vector>
#include <boost/date_time/time_zone_base.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/local_time/local_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <glog/logging.h>
#include "gnss_sdr_flags.h"


using google::LogMessage;


Rinex_Printer::Rinex_Printer(int conf_version)
{
    navfilename = Rinex_Printer::createFilename("RINEX_FILE_TYPE_GPS_NAV");
    obsfilename = Rinex_Printer::createFilename("RINEX_FILE_TYPE_OBS");
    sbsfilename = Rinex_Printer::createFilename("RINEX_FILE_TYPE_SBAS");
    navGalfilename = Rinex_Printer::createFilename("RINEX_FILE_TYPE_GAL_NAV");
    navMixfilename = Rinex_Printer::createFilename("RINEX_FILE_TYPE_MIXED_NAV");
    navGlofilename = Rinex_Printer::createFilename("RINEX_FILE_TYPE_GLO_NAV");

    Rinex_Printer::navFile.open(navfilename, std::ios::out | std::ios::in | std::ios::app);
    Rinex_Printer::obsFile.open(obsfilename, std::ios::out | std::ios::in | std::ios::app);
    Rinex_Printer::sbsFile.open(sbsfilename, std::ios::out | std::ios::app);
    Rinex_Printer::navGalFile.open(navGalfilename, std::ios::out | std::ios::in | std::ios::app);
    Rinex_Printer::navMixFile.open(navMixfilename, std::ios::out | std::ios::in | std::ios::app);
    Rinex_Printer::navGloFile.open(navGlofilename, std::ios::out | std::ios::in | std::ios::app);

    // RINEX v3.02 codes
    satelliteSystem["GPS"] = "G";
    satelliteSystem["GLONASS"] = "R";
    satelliteSystem["SBAS payload"] = "S";
    satelliteSystem["Galileo"] = "E";
    satelliteSystem["Beidou"] = "C";
    satelliteSystem["Mixed"] = "M";

    observationCode["GPS_L1_CA"] = "1C";              // "1C" GPS L1 C/A
    observationCode["GPS_L1_P"] = "1P";               // "1P" GPS L1 P
    observationCode["GPS_L1_Z_TRACKING"] = "1W";      // "1W" GPS L1 Z-tracking and similar (AS on)
    observationCode["GPS_L1_Y"] = "1Y";               // "1Y" GPS L1 Y
    observationCode["GPS_L1_M "] = "1M";              // "1M" GPS L1 M
    observationCode["GPS_L1_CODELESS"] = "1N";        // "1N" GPS L1 codeless
    observationCode["GPS_L2_CA"] = "2C";              // "2C" GPS L2 C/A
    observationCode["L2_SEMI_CODELESS"] = "2D";       // "2D" GPS L2 L1(C/A)+(P2-P1) semi-codeless
    observationCode["GPS_L2_L2CM"] = "2S";            // "2S" GPS L2 L2C (M)
    observationCode["GPS_L2_L2CL"] = "2L";            // "2L" GPS L2 L2C (L)
    observationCode["GPS_L2_L2CML"] = "2X";           // "2X" GPS L2 L2C (M+L)
    observationCode["GPS_L2_P"] = "2P";               // "2P" GPS L2 P
    observationCode["GPS_L2_Z_TRACKING"] = "2W";      // "2W" GPS L2 Z-tracking and similar (AS on)
    observationCode["GPS_L2_Y"] = "2Y";               // "2Y" GPS L2 Y
    observationCode["GPS_L2_M"] = "2M";               // "2M" GPS GPS L2 M
    observationCode["GPS_L2_codeless"] = "2N";        // "2N" GPS L2 codeless
    observationCode["GPS_L5_I"] = "5I";               // "5I" GPS L5 I
    observationCode["GPS_L5_Q"] = "5Q";               // "5Q" GPS L5 Q
    observationCode["GPS_L5_IQ"] = "5X";              // "5X" GPS L5 I+Q
    observationCode["GLONASS_G1_CA"] = "1C";          // "1C" GLONASS G1 C/A
    observationCode["GLONASS_G1_P"] = "1P";           // "1P" GLONASS G1 P
    observationCode["GLONASS_G2_CA"] = "2C";          // "2C" GLONASS G2 C/A  (Glonass M)
    observationCode["GLONASS_G2_P"] = "2P";           // "2P" GLONASS G2 P
    observationCode["GALILEO_E1_A"] = "1A";           // "1A" GALILEO E1 A (PRS)
    observationCode["GALILEO_E1_B"] = "1B";           // "1B" GALILEO E1 B (I/NAV OS/CS/SoL)
    observationCode["GALILEO_E1_C"] = "1C";           // "1C" GALILEO E1 C (no data)
    observationCode["GALILEO_E1_BC"] = "1X";          // "1X" GALILEO E1 B+C
    observationCode["GALILEO_E1_ABC"] = "1Z";         // "1Z" GALILEO E1 A+B+C
    observationCode["GALILEO_E5a_I"] = "5I";          // "5I" GALILEO E5a I (F/NAV OS)
    observationCode["GALILEO_E5a_Q"] = "5Q";          // "5Q" GALILEO E5a Q  (no data)
    observationCode["GALILEO_E5a_IQ"] = "5X";          // "5X" GALILEO E5a I+Q
    observationCode["GALILEO_E5b_I"] = "7I";          // "7I" GALILEO E5b I
    observationCode["GALILEO_E5b_Q"] = "7Q";          // "7Q" GALILEO E5b Q
    observationCode["GALILEO_E5b_IQ"] = "7X";         // "7X" GALILEO E5b I+Q
    observationCode["GALILEO_E5_I"] = "8I";           // "8I" GALILEO E5 I
    observationCode["GALILEO_E5_Q"] = "8Q";           // "8Q" GALILEO E5 Q
    observationCode["GALILEO_E5_IQ"] = "8X";          // "8X" GALILEO E5 I+Q
    observationCode["GALILEO_E56_A"] = "6A";          // "6A" GALILEO E6 A
    observationCode["GALILEO_E56_B"] = "6B";          // "6B" GALILEO E6 B
    observationCode["GALILEO_E56_B"] = "6C";          // "6C" GALILEO E6 C
    observationCode["GALILEO_E56_BC"] = "6X";         // "6X" GALILEO E6 B+C
    observationCode["GALILEO_E56_ABC"] = "6Z";        // "6Z" GALILEO E6 A+B+C
    observationCode["SBAS_L1_CA"] = "1C";             // "1C" SBAS L1 C/A
    observationCode["SBAS_L5_I"] = "5I";              // "5I" SBAS L5 I
    observationCode["SBAS_L5_Q"] = "5Q";              // "5Q" SBAS L5 Q
    observationCode["SBAS_L5_IQ"] = "5X";             // "5X" SBAS L5 I+Q
    observationCode["COMPASS_E2_I"] = "2I";
    observationCode["COMPASS_E2_Q"] = "2Q";
    observationCode["COMPASS_E2_IQ"] = "2X";
    observationCode["COMPASS_E5b_I"] = "7I";
    observationCode["COMPASS_E5b_Q"] = "7Q";
    observationCode["COMPASS_E5b_IQ"] = "7X";
    observationCode["COMPASS_E6_I"] = "6I";
    observationCode["COMPASS_E6_Q"] = "6Q";
    observationCode["COMPASS_E6_IQ"] = "6X";

    observationType["PSEUDORANGE"] = "C";
    observationType["CARRIER_PHASE"] = "L";
    observationType["DOPPLER"] = "D";
    observationType["SIGNAL_STRENGTH"] = "S";

    // RINEX v2.10 and v2.11 codes
    observationType["PSEUDORANGE_CA_v2"] = "C";
    observationType["PSEUDORANGE_P_v2"] = "P";
    observationType["CARRIER_PHASE_CA_v2"] = "L";
    observationType["DOPPLER_v2"] = "D";
    observationType["SIGNAL_STRENGTH_v2"] = "S";
    observationCode["GPS_L1_CA_v2"] = "1";
    observationCode["GLONASS_G1_CA_v2"] = "1";

    if ( FLAGS_RINEX_version.compare("3.01") == 0 )
        {
            version = 3;
            stringVersion = "3.01";
        }
    else if ( FLAGS_RINEX_version.compare("3.02") == 0 )
        {
            version = 3;
            stringVersion = "3.02";
        }
    else if ( FLAGS_RINEX_version.compare("3") == 0 )
        {
            version = 3;
            stringVersion = "3.02";
        }
    else if ( FLAGS_RINEX_version.compare("2.11") == 0 )
        {
            version = 2;
            stringVersion = "2.11";
        }
    else if ( FLAGS_RINEX_version.compare("2.10") == 0 )
        {
            version = 2;
            stringVersion = "2.10";
        }
    else if ( FLAGS_RINEX_version.compare("2") == 0 )
        {
            version = 2;
            stringVersion = "2.11";
        }
    else
        {
            LOG(WARNING) << "Unknown RINEX version " << FLAGS_RINEX_version << " (must be 2.11 or 3.02). Using 3.02";
            version = 3;
            stringVersion = "3.02";
        }

    if(conf_version != 0)
        {
            if(conf_version == 2)
                {
                    version = 2;
                    stringVersion = "2.11";
                }
            if(conf_version == 3)
                {
                    version = 3;
                    stringVersion = "3.02";
                }
        }

    numberTypesObservations = 4; // Number of available types of observable in the system
    fake_cnav_iode = 1;
}


Rinex_Printer::~Rinex_Printer()
{
    // close RINEX files
    long posn, poso, poss, posng, posmn, posnr;
    posn = navFile.tellp();
    poso = obsFile.tellp();
    poss = sbsFile.tellp();
    posng = navGalFile.tellp();
    posmn = navMixFile.tellp();
    posnr = navGloFile.tellp();

    Rinex_Printer::navFile.close();
    Rinex_Printer::obsFile.close();
    Rinex_Printer::sbsFile.close();
    Rinex_Printer::navGalFile.close();
    Rinex_Printer::navGloFile.close();
    // If nothing written, erase the files.
    if (posn == 0)
        {
            if(remove(navfilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
        }
    if (poso == 0)
        {
            if(remove(obsfilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
        }
    if (poss == 0)
        {
            if(remove(sbsfilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
        }
    if (posng == 0)
        {
            if(remove(navGalfilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
        }
    if (posmn == 0)
        {
            if(remove(navMixfilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
        }
    if (posnr == 0)
        {
            if(remove(navGlofilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
        }
}


void Rinex_Printer::lengthCheck(const std::string& line)
{
    if (line.length() != 80)
        {
            LOG(ERROR) << "Bad defined RINEX line: "
                    << line.length() << " characters (must be 80)" << std::endl
                    << line << std::endl
                    << "----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|" << std::endl;
        }
}


std::string Rinex_Printer::createFilename(std::string type)
{
    const std::string stationName = "GSDR"; // 4-character station name designator
    boost::gregorian::date today = boost::gregorian::day_clock::local_day();
    const int dayOfTheYear = today.day_of_year();
    std::stringstream strm0;
    if (dayOfTheYear < 100) strm0 << "0"; // three digits for day of the year
    if (dayOfTheYear < 10) strm0 << "0"; // three digits for day of the year
    strm0 << dayOfTheYear;
    std::string dayOfTheYearTag=strm0.str();

    std::map<std::string, std::string> fileType;
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_OBS", "O"));        // O - Observation file.
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_GPS_NAV", "N"));    // N - GPS navigation message file.
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_MET", "M"));        // M - Meteorological data file.
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_GLO_NAV", "G"));    // G - GLONASS navigation file.
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_GAL_NAV", "L"));    // L - Galileo navigation message file.
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_MIXED_NAV", "P"));  // P - Mixed GNSS navigation message file.
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_GEO_NAV", "H"));    // H - SBAS Payload navigation message file.
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_SBAS", "B"));       // B - SBAS broadcast data file.
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_CLK", "C"));        // C - Clock file.
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_SUMMARY", "S"));    // S - Summary file (used e.g., by IGS, not a standard!).

    boost::posix_time::ptime pt = boost::posix_time::second_clock::local_time();
    tm pt_tm = boost::posix_time::to_tm(pt);
    int local_hour = pt_tm.tm_hour;
    std::stringstream strm;
    strm << local_hour;

    std::map<std::string, std::string> Hmap;
    Hmap.insert(std::pair<std::string, std::string>("0", "a"));
    Hmap.insert(std::pair<std::string, std::string>("1", "b"));
    Hmap.insert(std::pair<std::string, std::string>("2", "c"));
    Hmap.insert(std::pair<std::string, std::string>("3", "d"));
    Hmap.insert(std::pair<std::string, std::string>("4", "e"));
    Hmap.insert(std::pair<std::string, std::string>("5", "f"));
    Hmap.insert(std::pair<std::string, std::string>("6", "g"));
    Hmap.insert(std::pair<std::string, std::string>("7", "h"));
    Hmap.insert(std::pair<std::string, std::string>("8", "i"));
    Hmap.insert(std::pair<std::string, std::string>("9", "j"));
    Hmap.insert(std::pair<std::string, std::string>("10", "k"));
    Hmap.insert(std::pair<std::string, std::string>("11", "l"));
    Hmap.insert(std::pair<std::string, std::string>("12", "m"));
    Hmap.insert(std::pair<std::string, std::string>("13", "n"));
    Hmap.insert(std::pair<std::string, std::string>("14", "o"));
    Hmap.insert(std::pair<std::string, std::string>("15", "p"));
    Hmap.insert(std::pair<std::string, std::string>("16", "q"));
    Hmap.insert(std::pair<std::string, std::string>("17", "r"));
    Hmap.insert(std::pair<std::string, std::string>("18", "s"));
    Hmap.insert(std::pair<std::string, std::string>("19", "t"));
    Hmap.insert(std::pair<std::string, std::string>("20", "u"));
    Hmap.insert(std::pair<std::string, std::string>("21", "v"));
    Hmap.insert(std::pair<std::string, std::string>("22", "w"));
    Hmap.insert(std::pair<std::string, std::string>("23", "x"));

    std::string hourTag = Hmap[strm.str()];

    int local_minute = pt_tm.tm_min;
    std::stringstream strm2;
    if (local_minute<10) strm2 << "0"; // at least two digits for minutes
    strm2 << local_minute;

    std::string minTag = strm2.str();

    int local_year = pt_tm.tm_year - 100; // 2012 is 112
    std::stringstream strm3;
    strm3 << local_year;
    std::string yearTag = strm3.str();

    std::string typeOfFile = fileType[type];

    std::string filename(stationName + dayOfTheYearTag + hourTag + minTag + "." + yearTag + typeOfFile);
    return filename;
}


std::string Rinex_Printer::getLocalTime()
{
    std::string line;
    line += std::string("GNSS-SDR");
    line += std::string(12, ' ');
    std::string username;
    char c_username[20] = {0};
    int nGet = getlogin_r(c_username, sizeof(c_username) - 1);
    if (nGet == 0)
        {
            username = c_username;
        }
    else
        {
            username = "UNKNOWN USER";
        }
    line += Rinex_Printer::leftJustify(username, 20);
    boost::gregorian::date today = boost::gregorian::day_clock::local_day();

    boost::local_time::time_zone_ptr zone(new boost::local_time::posix_time_zone("UTC"));
    boost::local_time::local_date_time pt = boost::local_time::local_sec_clock::local_time(zone);
    tm pt_tm = boost::local_time::to_tm(pt);

    std::stringstream strmHour;
    int utc_hour = pt_tm.tm_hour;
    if (utc_hour < 10) strmHour << "0"; //  two digits for hours
    strmHour << utc_hour;

    std::stringstream strmMin;
    int utc_minute = pt_tm.tm_min;
    if (utc_minute < 10) strmMin << "0"; //  two digits for minutes
    strmMin << utc_minute;

    if (version == 2)
        {
            int day = pt_tm.tm_mday;
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(day), 2);
            line += std::string("-");

            std::map<int, std::string> months;
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
            line += boost::lexical_cast<std::string>(pt_tm.tm_year - 100);
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
            int utc_seconds = pt_tm.tm_sec;
            if (utc_seconds < 10) strm2 << "0"; //  two digits for seconds
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
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += stringVersion;
    line += std::string(11, ' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4, ' ');
    line += std::string("R: GLONASS");
    line += std::string(10, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GLONASS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See http://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line system time correction
    if (version == 3)
        {
            line.clear();
            line += std::string("GLUT");
            line += std::string(1, ' ');
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_c, 16, 2), 17);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 7);
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 5);
            line += std::string(10, ' ');
            line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- Line system time correction 2
            line.clear();
            line += std::string("GLGP");
            line += std::string(1, ' ');
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_gps, 16, 2), 17);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 7);
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 5);
            line += std::string(10, ' ');
            line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
        }
    if (version == 2)
        {
            // Set reference time and its clock corrections
            boost::posix_time::ptime p_utc_ref_time = glonass_gnav_eph.glot_to_utc(glonass_gnav_eph.d_t_b, 0.0);
            std::string timestring = boost::posix_time::to_iso_string(p_utc_ref_time);
            std::string year (timestring, 0, 4);
            std::string month (timestring, 4, 2);
            std::string day (timestring, 6, 2);

            line.clear();
            line += Rinex_Printer::rightJustify(year, 6);
            line += Rinex_Printer::rightJustify(month, 6);
            line += Rinex_Printer::rightJustify(day, 6);
            line += std::string(3, ' ');
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_c, 19, 2), 19);
            line += std::string(20, ' ');
            line += Rinex_Printer::leftJustify("CORR TO SYSTEM TIME", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
        }

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;
}

void Rinex_Printer::rinex_nav_header(std::fstream& out, const Gps_Iono& gps_iono, const Gps_Utc_Model& gps_utc_model, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac)
{
    if(glonass_gnav_almanac.i_satellite_freq_channel){} //Avoid compiler warning
    std::string line;
    stringVersion = "3.02";
    version = 3;

    // -------- Line 1
    line = std::string(5, ' ');
    line += stringVersion;
    line += std::string(11, ' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4, ' ');
    line += std::string("M: MIXED");
    line += std::string(12, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See http://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line ionospheric info 1
    line.clear();
    line += std::string("GPSA");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line system time correction 1
    line.clear();
    line += std::string("GLUT");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_c, 16, 2), 17);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 7);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line system time correction 2
    line.clear();
    line += std::string("GLGP");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_gps, 16, 2), 17);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 7);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line system time correction 3
    line.clear();
    line += std::string("GPUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.d_A0, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.d_A1, 15, 2), 16);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.d_t_OT), 7);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.i_WN_T + 1024), 5);  // valid until 2019
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 6 leap seconds
    // For leap second information, see http://www.endruntechnologies.com/leap.htm
    line.clear();
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.d_DeltaT_LS), 6);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.d_DeltaT_LSF), 6);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.i_WN_LSF), 6);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.i_DN), 6);
    line += std::string(36, ' ');
    line += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Gps_CNAV_Iono& gps_iono, const Gps_CNAV_Utc_Model& gps_utc_model, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac)
{
    if(glonass_gnav_almanac.i_satellite_freq_channel){} //Avoid compiler warning
    std::string line;
    stringVersion = "3.02";
    version = 3;

    // -------- Line 1
    line = std::string(5, ' ');
    line += stringVersion;
    line += std::string(11, ' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4, ' ');
    line += std::string("M: MIXED");
    line += std::string(12, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See http://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line ionospheric info 1
    line.clear();
    line += std::string("GPSA");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line system time correction 1
    line.clear();
    line += std::string("GLUT");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_c, 16, 2), 17);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 7);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line system time correction 2
    line.clear();
    line += std::string("GLGP");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_gps, 16, 2), 17);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 7);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line system time correction 3
    line.clear();
    line += std::string("GPUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.d_A0, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.d_A1, 15, 2), 16);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.d_t_OT), 7);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.i_WN_T + 1024), 5);  // valid until 2019
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 6 leap seconds
    // For leap second information, see http://www.endruntechnologies.com/leap.htm
    line.clear();
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.d_DeltaT_LS), 6);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.d_DeltaT_LSF), 6);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.i_WN_LSF), 6);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.i_DN), 6);
    line += std::string(36, ' ');
    line += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model, const Galileo_Almanac& galileo_almanac, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac)
{
    if(glonass_gnav_almanac.i_satellite_freq_channel){} //Avoid compiler warning
    //Avoid compiler warning, there is not time system correction between Galileo and GLONASS
    if(galileo_almanac.A_0G_10){}
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += stringVersion;
    line += std::string(11, ' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4, ' ');
    line += std::string("M: MIXED");
    line += std::string(12, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See http://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line ionospheric info 1
    line.clear();
    line += std::string("GAL ");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai0_5, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai1_5, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai2_5, 10, 2), 12);
    double zero = 0.0;
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(zero, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line system time correction
    line.clear();
    line += std::string("GAUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A0_6, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A1_6, 15, 2), 16);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_utc_model.t0t_6), 7);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_utc_model.WNot_6), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line system time correction 1
    line.clear();
    line += std::string("GLUT");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_c, 16, 2), 17);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 7);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 6 leap seconds
    // For leap second information, see http://www.endruntechnologies.com/leap.htm
    line.clear();
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_utc_model.Delta_tLS_6), 6);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_utc_model.Delta_tLSF_6), 6);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_utc_model.WN_LSF_6), 6);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_utc_model.DN_6), 6);
    line += std::string(36, ' ');
    line += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Galileo_Iono& iono, const Galileo_Utc_Model& utc_model, const Galileo_Almanac& galileo_almanac)
{
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += stringVersion;
    line += std::string(11, ' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4, ' ');
    line += std::string("E: GALILEO");
    line += std::string(10, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GALILEO NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See http://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line ionospheric info 1
    line.clear();
    line += std::string("GAL ");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.ai0_5, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.ai1_5, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.ai2_5, 10, 2), 12);
    double zero = 0.0;
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(zero, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line system time correction
    line.clear();
    line += std::string("GAUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A0_6, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A1_6, 15, 2), 16);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.t0t_6), 7);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.WNot_6), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line system time correction 2
    line.clear();
    line += std::string("GPGA");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_almanac.A_0G_10, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_almanac.A_1G_10, 15, 2), 16);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_almanac.t_0G_10), 7);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_almanac.WN_0G_10), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 6 leap seconds
    // For leap second information, see http://www.endruntechnologies.com/leap.htm
    line.clear();
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.Delta_tLS_6), 6);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.Delta_tLSF_6), 6);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.WN_LSF_6), 6);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.DN_6), 6);
    line += std::string(36, ' ');
    line += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Gps_CNAV_Iono & iono, const Gps_CNAV_Utc_Model & utc_model)
{
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += stringVersion;
    line += std::string(11, ' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4, ' ');
    line += std::string("G: GPS");
    line += std::string(14, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::leftJustify("GPS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See http://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line ionospheric info 1
    line.clear();
    line += std::string("GPSA");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line ionospheric info 2
    line.clear();
    line += std::string("GPSB");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 5 system time correction
    line.clear();
    line += std::string("GPUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.d_A0, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.d_A1, 15, 2), 16);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.d_t_OT), 7);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.i_WN_T + 1024), 5);  // valid until 2019
    /*  if ( SBAS )
    {
      line += string(1, ' ');
      line += leftJustify(asString(d_t_OT_SBAS),5);
      line += string(1, ' ');
      line += leftJustify(asString(d_WN_T_SBAS),2);
      line += string(1, ' ');
           }
    else
     */
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 6 leap seconds
    // For leap second information, see http://www.endruntechnologies.com/leap.htm
    line.clear();
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.d_DeltaT_LS), 6);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.d_DeltaT_LSF), 6);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.i_WN_LSF), 6);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.i_DN), 6);
    line += std::string(36, ' ');
    line += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Gps_Iono& iono, const Gps_Utc_Model& utc_model)
{
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += stringVersion;
    line += std::string(11, ' ');

    if (version == 2)
        {
            line += std::string("N: GPS NAV DATA");
            line += std::string(25, ' ');
        }

    if (version == 3 )
        {
            line += std::string("N: GNSS NAV DATA");
            line += std::string(4, ' ');
            //! \todo Add here other systems...
            line += std::string("G: GPS");
            line += std::string(14, ' ');
            // ...
        }

    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::leftJustify("GPS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See http://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line ionospheric info 1
    line.clear();
    if (version == 2)
        {
            line += std::string(2, ' ');
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha0, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha1, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha2, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha3, 10, 2), 12);
            line += std::string(10, ' ');
            line += Rinex_Printer::leftJustify("ION ALPHA", 20);
        }
    if (version == 3)
        {
            line += std::string("GPSA");
            line += std::string(1, ' ');
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha0, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha1, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha2, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha3, 10, 2), 12);
            line += std::string(7, ' ');
            line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
        }
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line ionospheric info 2
    line.clear();
    if (version == 2)
        {
            line += std::string(2, ' ');
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta0, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta1, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta2, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta3, 10, 2), 12);
            line += std::string(10, ' ');
            line += Rinex_Printer::leftJustify("ION BETA", 20);
        }

    if (version == 3)
        {
            line += std::string("GPSB");
            line += std::string(1, ' ');
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta0, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta1, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta2, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta3, 10, 2), 12);
            line += std::string(7, ' ');
            line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
        }
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 5 system time correction
    line.clear();
    if (version == 2)
        {
            line += std::string(3, ' ');
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.d_A0, 18, 2), 19);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.d_A1, 18, 2), 19);
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.d_t_OT), 9);
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.i_WN_T + 1024), 9); // valid until 2019
            line += std::string(1, ' ');
            line += Rinex_Printer::leftJustify("DELTA-UTC: A0,A1,T,W", 20);
        }

    if (version == 3)
        {
            line += std::string("GPUT");
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.d_A0, 16, 2), 18);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.d_A1, 15, 2), 16);
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.d_t_OT), 7);
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.i_WN_T + 1024), 5);  // valid until 2019
            /*  if ( SBAS )
        {
          line += string(1, ' ');
          line += leftJustify(asString(d_t_OT_SBAS),5);
          line += string(1, ' ');
          line += leftJustify(asString(d_WN_T_SBAS),2);
          line += string(1, ' ');
               }
          else
             */
            line += std::string(10, ' ');
            line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
        }
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 6 leap seconds
    // For leap second information, see http://www.endruntechnologies.com/leap.htm
    line.clear();
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.d_DeltaT_LS), 6);
    if (version == 2)
        {
            line += std::string(54, ' ');
        }
    if (version == 3)
        {
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.d_DeltaT_LSF), 6);
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.i_WN_LSF), 6);
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.i_DN), 6);
            line += std::string(36, ' ');
        }
    line += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Gps_Iono& gps_iono, const Gps_Utc_Model& gps_utc_model, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model, const Galileo_Almanac& galileo_almanac)
{
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += stringVersion;
    line += std::string(11, ' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4, ' ');
    line += std::string("M: MIXED");
    line += std::string(12, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See http://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line ionospheric info 1
    line.clear();
    line += std::string("GAL ");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai0_5, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai1_5, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai2_5, 10, 2), 12);
    double zero = 0.0;
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(zero, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line ionospheric info 2
    line.clear();
    line += std::string("GPSA");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line system time correction
    line.clear();
    line += std::string("GAUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A0_6, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A1_6, 15, 2), 16);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_utc_model.t0t_6), 7);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_utc_model.WNot_6), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line system time correction 2
    line.clear();
    line += std::string("GPGA");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_almanac.A_0G_10, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_almanac.A_1G_10, 15, 2), 16);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_almanac.t_0G_10), 7);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_almanac.WN_0G_10), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line system time correction 3
    line.clear();
    line += std::string("GPUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.d_A0, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.d_A1, 15, 2), 16);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.d_t_OT), 7);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.i_WN_T + 1024), 5);  // valid until 2019
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 6 leap seconds
    // For leap second information, see http://www.endruntechnologies.com/leap.htm
    line.clear();
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.d_DeltaT_LS), 6);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.d_DeltaT_LSF), 6);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.i_WN_LSF), 6);
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.i_DN), 6);
    line += std::string(36, ' ');
    line += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;
}


void Rinex_Printer::rinex_sbs_header(std::fstream& out)
{
    std::string line;

    // -------- Line 1
    line.clear();
    line = std::string(5, ' ');
    line += std::string("2.10");
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("B SBAS DATA",20);
    line += std::string(20, ' ');
    line += std::string("RINEX VERSION / TYPE");

    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20);
    std::string username;
    char c_username[20] = {0};
    int nGet = getlogin_r(c_username, sizeof(c_username) - 1);
    if (nGet == 0)
        {
            username = c_username;
        }
    else
        {
            username = "UNKNOWN USER";
        }
    line += Rinex_Printer::leftJustify(username, 20);
    // Date of file creation (dd-mmm-yy hhmm)
    boost::local_time::time_zone_ptr zone(new boost::local_time::posix_time_zone("UTC"));
    boost::local_time::local_date_time pt = boost::local_time::local_sec_clock::local_time(zone);
    tm pt_tm = boost::local_time::to_tm(pt);
    std::stringstream strYear;
    int utc_year = pt.date().year();
    utc_year -= 2000; //  two digits for year
    strYear << utc_year;
    std::stringstream strMonth;
    int utc_month = pt.date().month().as_number();
    if (utc_month < 10) strMonth << "0"; //  two digits for months
    strMonth << utc_month;
    std::stringstream strmDay;
    int utc_day = pt.date().day().as_number();
    if (utc_day < 10) strmDay << "0"; //  two digits for days
    strmDay << utc_day;
    std::stringstream strmHour;
    int utc_hour = pt_tm.tm_hour;
    if (utc_hour < 10) strmHour << "0"; //  two digits for hours
    strmHour << utc_hour;
    std::stringstream strmMin;
    int utc_minute = pt_tm.tm_min;
    if (utc_minute < 10) strmMin << "0"; //  two digits for minutes
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
    line += Rinex_Printer::leftJustify(time_str, 20);
    line += Rinex_Printer::leftJustify("PGM / RUN BY / DATE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 3
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("REC INDEX/TYPE/VERS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT 1
    line.clear();
    line += Rinex_Printer::leftJustify("BROADCAST DATA FILE FOR GEO SV, GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT 2
    line.clear();
    line += Rinex_Printer::leftJustify("See http://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac)
{
    if(glonass_gnav_almanac.i_satellite_freq_channel){} //Avoid compiler warning
    std::vector<std::string> data;
    std::string line_aux;

    long pos = out.tellp();
    out.seekp(0);
    data.clear();

    bool no_more_finds = false;
    std::string line_str;

    while(!out.eof())
        {
            std::getline(out, line_str);

            if(!no_more_finds)
                {
                    line_aux.clear();

                    if ((line_str.find("GLUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GLUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_c, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 7);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GLGP", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GLGP");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_gps, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 7);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
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
            else
                {
                    data.push_back(line_str);
                }
        }

    out.close();
    out.open(navGlofilename, std::ios::out | std::ios::trunc);
    out.seekp(0);
    for (int i = 0; i < static_cast<int>(data.size()) - 1; i++)
        {
            out << data[i] << std::endl;
        }
    out.close();
    out.open(navGlofilename, std::ios::out | std::ios::app);
    out.seekp(pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC info." << std::endl;
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& utc_model, const Galileo_Almanac& galileo_almanac)
{
    std::vector<std::string> data;
    std::string line_aux;

    long pos = out.tellp();
    out.seekp(0);
    data.clear();

    bool no_more_finds = false;
    std::string line_str;

    while(!out.eof())
        {
            std::getline(out, line_str);

            if(!no_more_finds)
                {
                    line_aux.clear();

                    if ((line_str.find("GAL", 0) != std::string::npos) && (line_str.find("IONOSPHERIC CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GAL ");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai0_5, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai1_5, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai2_5, 10, 2), 12);
                            double zero = 0.0;
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(zero, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GAUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GAUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A0_6, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A1_6, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.t0t_6), 7);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.WNot_6), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GPGA", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GPGA");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_almanac.A_0G_10, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_almanac.A_1G_10, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_almanac.t_0G_10), 7);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_almanac.WN_0G_10), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.Delta_tLS_6), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.Delta_tLSF_6), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.WN_LSF_6), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.DN_6), 6);
                            line_aux += std::string(36, ' ');
                            line_aux += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
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
            else
                {
                    data.push_back(line_str);
                }
        }

    out.close();
    out.open(navGalfilename, std::ios::out | std::ios::trunc);
    out.seekp(0);
    for (int i = 0; i < (int) data.size() - 1; i++)
        {
            out << data[i] << std::endl;
        }
    out.close();
    out.open(navGalfilename, std::ios::out | std::ios::app);
    out.seekp(pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info." << std::endl;
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Gps_Utc_Model& utc_model, const Gps_Iono& iono)
{
    std::vector<std::string> data;
    std::string line_aux;

    long pos = out.tellp();
    out.seekp(0);
    data.clear();

    bool no_more_finds = false;
    std::string line_str;

    while(!out.eof())
        {
            std::getline(out, line_str);

            if(!no_more_finds)
                {
                    line_aux.clear();

                    if (version == 2)
                        {
                            if (line_str.find("ION ALPHA", 59) != std::string::npos)
                                {
                                    line_aux += std::string(2, ' ');
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha0, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha1, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha2, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha3, 10, 2), 12);
                                    line_aux += std::string(10, ' ');
                                    line_aux += Rinex_Printer::leftJustify("ION ALPHA", 20);
                                    data.push_back(line_aux);
                                }
                            else if (line_str.find("ION BETA", 59) != std::string::npos)
                                {
                                    line_aux += std::string(2, ' ');
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta0, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta1, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta2, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta3, 10, 2), 12);
                                    line_aux += std::string(10, ' ');
                                    line_aux += Rinex_Printer::leftJustify("ION BETA", 20);
                                    data.push_back(line_aux);
                                }
                            else if (line_str.find("DELTA-UTC", 59) != std::string::npos)
                                {
                                    line_aux += std::string(3, ' ');
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.d_A0, 18, 2), 19);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.d_A1, 18, 2), 19);
                                    line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.d_t_OT), 9);
                                    line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.i_WN_T + 1024), 9); // valid until 2019
                                    line_aux += std::string(1, ' ');
                                    line_aux += Rinex_Printer::leftJustify("DELTA-UTC: A0,A1,T,W", 20);
                                    data.push_back(line_aux);
                                }
                            else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                                {
                                    line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.d_DeltaT_LS), 6);
                                    line_aux += std::string(54, ' ');
                                    line_aux += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
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

                    if (version == 3)
                        {
                            if (line_str.find("GPSA", 0) != std::string::npos)
                                {
                                    line_aux += std::string("GPSA");
                                    line_aux += std::string(1, ' ');
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha0, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha1, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha2, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha3, 10, 2), 12);
                                    line_aux += std::string(7, ' ');
                                    line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                                    data.push_back(line_aux);
                                }
                            else if (line_str.find("GPSB", 0) != std::string::npos)
                                {
                                    line_aux += std::string("GPSB");
                                    line_aux += std::string(1, ' ');
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta0, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta1, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta2, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta3, 10, 2), 12);
                                    line_aux += std::string(7, ' ');
                                    line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                                    data.push_back(line_aux);
                                }
                            else if (line_str.find("GPUT", 0) != std::string::npos)
                                {
                                    line_aux += std::string("GPUT");
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.d_A0, 16, 2), 18);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.d_A1, 15, 2), 16);
                                    line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.d_t_OT), 7);
                                    line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.i_WN_T + 1024), 5);  // valid until 2019
                                    line_aux += std::string(10, ' ');
                                    line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                                    data.push_back(line_aux);
                                }
                            else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                                {
                                    line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.d_DeltaT_LS), 6);
                                    line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.d_DeltaT_LSF), 6);
                                    line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.i_WN_LSF), 6);
                                    line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.i_DN), 6);
                                    line_aux += std::string(36, ' ');
                                    line_aux += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
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
                }
            else
                {
                    data.push_back(line_str);
                }
        }

    out.close();
    out.open(navfilename, std::ios::out | std::ios::trunc);
    out.seekp(0);
    for (int i = 0; i < static_cast<int>(data.size()) - 1; i++)
        {
            out << data[i] << std::endl;
        }
    out.close();
    out.open(navfilename, std::ios::out | std::ios::app);
    out.seekp(pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info." << std::endl;
}


void Rinex_Printer::update_nav_header(std::fstream & out, const Gps_CNAV_Utc_Model & utc_model, const Gps_CNAV_Iono & iono )
{
    std::vector<std::string> data;
    std::string line_aux;

    long pos = out.tellp();
    out.seekp(0);
    data.clear();

    bool no_more_finds = false;
    std::string line_str;

    while(!out.eof())
        {
            std::getline(out, line_str);

            if(!no_more_finds)
                {
                    line_aux.clear();

                    if (line_str.find("GPSA", 0) != std::string::npos)
                        {
                            line_aux += std::string("GPSA");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha0, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha1, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha2, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_alpha3, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("GPSB", 0) != std::string::npos)
                        {
                            line_aux += std::string("GPSB");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta0, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta1, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta2, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.d_beta3, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("GPUT", 0) != std::string::npos)
                        {
                            line_aux += std::string("GPUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.d_A0, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.d_A1, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.d_t_OT), 7);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.i_WN_T + 1024), 5);  // valid until 2019
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.d_DeltaT_LS), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.d_DeltaT_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.i_WN_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.i_DN), 6);
                            line_aux += std::string(36, ' ');
                            line_aux += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
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
            else
                {
                    data.push_back(line_str);
                }
        }

    out.close();
    out.open(navfilename, std::ios::out | std::ios::trunc);
    out.seekp(0);
    for (int i = 0; i < static_cast<int>(data.size()) - 1; i++)
        {
            out << data[i] << std::endl;
        }
    out.close();
    out.open(navfilename, std::ios::out | std::ios::app);
    out.seekp(pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info." << std::endl;
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Gps_Iono& gps_iono, const Gps_Utc_Model& gps_utc_model, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model, const Galileo_Almanac& galileo_almanac)
{
    std::vector<std::string> data;
    std::string line_aux;

    long pos = out.tellp();
    out.seekp(0);
    data.clear();

    bool no_more_finds = false;
    std::string line_str;

    while(!out.eof())
        {
            std::getline(out, line_str);

            if(!no_more_finds)
                {
                    line_aux.clear();

                    if (line_str.find("GPSA", 0) != std::string::npos)
                        {
                            line_aux += std::string("GPSA");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha0, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha1, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha2, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha3, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GAL", 0) != std::string::npos) && (line_str.find("IONOSPHERIC CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GAL ");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai0_5, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai1_5, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai2_5, 10, 2), 12);
                            double zero = 0.0;
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(zero, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GPSB", 0) != std::string::npos) && (line_str.find("IONOSPHERIC CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GPSB");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_beta0, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_beta1, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_beta2, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_beta3, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GPUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GPUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.d_A0, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.d_A1, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.d_t_OT), 7);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.i_WN_T + 1024), 5);  // valid until 2019
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GAUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GAUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A0_6, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A1_6, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_utc_model.t0t_6), 7);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_utc_model.WNot_6), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GPGA", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GPGA");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_almanac.A_0G_10, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_almanac.A_1G_10, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_almanac.t_0G_10), 7);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_almanac.WN_0G_10), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.d_DeltaT_LS), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.d_DeltaT_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.i_WN_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.i_DN), 6);
                            line_aux += std::string(36, ' ');
                            line_aux += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
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
            else
                {
                    data.push_back(line_str);
                }
        }

    out.close();
    out.open(navMixfilename, std::ios::out | std::ios::trunc);
    out.seekp(0);
    for (int i = 0; i < static_cast<int>(data.size()) - 1; i++)
        {
            out << data[i] << std::endl;
        }
    out.close();
    out.open(navMixfilename, std::ios::out | std::ios::app);
    out.seekp(pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info." << std::endl;
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Gps_Iono& gps_iono, const Gps_Utc_Model& gps_utc_model, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac)
{
    if(glonass_gnav_almanac.i_satellite_freq_channel){} //Avoid compiler warning
    std::vector<std::string> data;
    std::string line_aux;

    long pos = out.tellp();
    out.seekp(0);
    data.clear();

    bool no_more_finds = false;
    std::string line_str;

    while(!out.eof())
        {
            std::getline(out, line_str);

            if(!no_more_finds)
                {
                    line_aux.clear();

                    if (line_str.find("GPSA", 0) != std::string::npos)
                        {
                            line_aux += std::string("GPSA");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha0, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha1, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha2, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha3, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GPUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GPUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.d_A0, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.d_A1, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.d_t_OT), 7);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.i_WN_T + 1024), 5);  // valid until 2019
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GLUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GLUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_c, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 7);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GLGP", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GLGP");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_gps, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 7);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.d_DeltaT_LS), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.d_DeltaT_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.i_WN_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.i_DN), 6);
                            line_aux += std::string(36, ' ');
                            line_aux += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
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
            else
                {
                    data.push_back(line_str);
                }
        }

    out.close();
    out.open(navMixfilename, std::ios::out | std::ios::trunc);
    out.seekp(0);
    for (int i = 0; i < (int) data.size() - 1; i++)
        {
            out << data[i] << std::endl;
        }
    out.close();
    out.open(navMixfilename, std::ios::out | std::ios::app);
    out.seekp(pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info." << std::endl;
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Gps_CNAV_Iono& gps_iono, const Gps_CNAV_Utc_Model& gps_utc_model, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac)
{
    if(glonass_gnav_almanac.i_satellite_freq_channel){} //Avoid compiler warning
    std::vector<std::string> data;
    std::string line_aux;

    long pos = out.tellp();
    out.seekp(0);
    data.clear();

    bool no_more_finds = false;
    std::string line_str;

    while(!out.eof())
        {
            std::getline(out, line_str);

            if(!no_more_finds)
                {
                    line_aux.clear();

                    if (line_str.find("GPSA", 0) != std::string::npos)
                        {
                            line_aux += std::string("GPSA");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha0, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha1, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha2, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.d_alpha3, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GPUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GPUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.d_A0, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.d_A1, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.d_t_OT), 7);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.i_WN_T + 1024), 5);  // valid until 2019
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GLUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GLUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_c, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 7);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GLGP", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GLGP");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_gps, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 7);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.d_DeltaT_LS), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.d_DeltaT_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.i_WN_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_utc_model.i_DN), 6);
                            line_aux += std::string(36, ' ');
                            line_aux += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
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
            else
                {
                    data.push_back(line_str);
                }
        }

    out.close();
    out.open(navMixfilename, std::ios::out | std::ios::trunc);
    out.seekp(0);
    for (int i = 0; i < (int) data.size() - 1; i++)
        {
            out << data[i] << std::endl;
        }
    out.close();
    out.open(navMixfilename, std::ios::out | std::ios::app);
    out.seekp(pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info." << std::endl;
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model, const Galileo_Almanac& galileo_almanac, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac)
{
    if(glonass_gnav_almanac.i_satellite_freq_channel){} //Avoid compiler warning
    //Avoid compiler warning, there is not time system correction between Galileo and GLONASS
    if(galileo_almanac.A_0G_10){}
    std::vector<std::string> data;
    std::string line_aux;

    long pos = out.tellp();
    out.seekp(0);
    data.clear();

    bool no_more_finds = false;
    std::string line_str;

    while(!out.eof())
        {
            std::getline(out, line_str);

            if(!no_more_finds)
                {
                    line_aux.clear();

                    if ((line_str.find("GAL", 0) != std::string::npos) && (line_str.find("IONOSPHERIC CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GAL ");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai0_5, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai1_5, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai2_5, 10, 2), 12);
                            double zero = 0.0;
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(zero, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GAUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GAUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A0_6, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A1_6, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_utc_model.t0t_6), 7);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_utc_model.WNot_6), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GLUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GLUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_c, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 7);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0.0), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_utc_model.Delta_tLS_6), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_utc_model.Delta_tLSF_6), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_utc_model.WN_LSF_6), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_utc_model.DN_6), 6);
                            line_aux += std::string(36, ' ');
                            line_aux += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
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
            else
                {
                    data.push_back(line_str);
                }
        }

    out.close();
    out.open(navMixfilename, std::ios::out | std::ios::trunc);
    out.seekp(0);
    for (int i = 0; i < (int) data.size() - 1; i++)
        {
            out << data[i] << std::endl;
        }
    out.close();
    out.open(navMixfilename, std::ios::out | std::ios::app);
    out.seekp(pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info." << std::endl;
}


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int,Gps_Ephemeris>& eph_map)
{
    std::string line;
    std::map<int,Gps_Ephemeris>::const_iterator gps_ephemeris_iter;

    for(gps_ephemeris_iter = eph_map.cbegin();
            gps_ephemeris_iter != eph_map.cend();
            gps_ephemeris_iter++)
        {
            // -------- SV / EPOCH / SV CLK
            boost::posix_time::ptime p_utc_time = Rinex_Printer::compute_GPS_time(gps_ephemeris_iter->second, gps_ephemeris_iter->second.d_Toc);
            std::string timestring = boost::posix_time::to_iso_string(p_utc_time);
            std::string month (timestring, 4, 2);
            std::string day (timestring, 6, 2);
            std::string hour (timestring, 9, 2);
            std::string minutes (timestring, 11, 2);
            std::string seconds (timestring, 13, 2);
            if (version == 2)
                {
                    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(gps_ephemeris_iter->second.i_satellite_PRN), 2);
                    line += std::string(1, ' ');
                    std::string year (timestring, 2, 2);
                    line += year;
                    line += std::string(1, ' ');
                    if(boost::lexical_cast<int>(month) < 10)
                        {
                            line += std::string(1, ' ');
                            line += std::string(month, 1, 1);
                        }
                    else
                        {
                            line += month;
                        }
                    line += std::string(1, ' ');
                    if(boost::lexical_cast<int>(day) < 10)
                        {
                            line += std::string(1, ' ');
                            line += std::string(day, 1, 1);
                        }
                    else
                        {
                            line += day;
                        }
                    line += std::string(1, ' ');
                    if(boost::lexical_cast<int>(hour) < 10)
                        {
                            line += std::string(1, ' ');
                            line += std::string(hour, 1, 1);
                        }
                    else
                        {
                            line += hour;
                        }
                    line += std::string(1, ' ');
                    if(boost::lexical_cast<int>(minutes) < 10)
                        {
                            line += std::string(1, ' ');
                            line += std::string(minutes, 1, 1);
                        }
                    else
                        {
                            line += minutes;
                        }
                    line += std::string(1, ' ');
                    if(boost::lexical_cast<int>(seconds) < 10)
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
                            std::string decimal (timestring, 16, 1);
                        }
                    line += decimal;
                    line += std::string(1, ' ');
                    line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_A_f0, 18, 2);
                    line += std::string(1, ' ');
                    line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_A_f1, 18, 2);
                    line += std::string(1, ' ');
                    line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_A_f2, 18, 2);
                    line += std::string(1, ' ');
                }
            if (version == 3)
                {
                    line += satelliteSystem["GPS"];
                    if (gps_ephemeris_iter->second.i_satellite_PRN < 10)  line += std::string("0");
                    line += boost::lexical_cast<std::string>(gps_ephemeris_iter->second.i_satellite_PRN);
                    std::string year (timestring, 0, 4);
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
                    line += seconds;
                    line += std::string(1, ' ');
                    line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_A_f0, 18, 2);
                    line += std::string(1, ' ');
                    line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_A_f1, 18, 2);
                    line += std::string(1, ' ');
                    line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_A_f2, 18, 2);
                }
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 1
            line.clear();

            if (version == 2)
                {
                    line += std::string(4, ' ');
                }
            if (version == 3)
                {
                    line += std::string(5, ' ');
                }
            // If there is a discontinued reception the ephemeris is not validated
            if (gps_ephemeris_iter->second.d_IODE_SF2 == gps_ephemeris_iter->second.d_IODE_SF3)
                {
                    line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_IODE_SF2, 18, 2);
                }
            else
                {
                    LOG(WARNING) << "Discontinued reception of Frame 2 and 3";
                }
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_Crs, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_Delta_n, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_M_0, 18, 2);
            if (version == 2)
                {
                    line += std::string(1, ' ');
                }
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 2
            line.clear();
            if (version == 2)
                {
                    line += std::string(4, ' ');
                }
            if (version == 3)
                {
                    line += std::string(5, ' ');
                }
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_Cuc, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_e_eccentricity, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_Cus, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_sqrt_A, 18, 2);
            if (version == 2)
                {
                    line += std::string(1, ' ');
                }
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 3
            line.clear();
            if (version == 2)
                {
                    line += std::string(4, ' ');
                }
            if (version == 3)
                {
                    line += std::string(5, ' ');
                }
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_Toe, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_Cic, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_OMEGA0, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_Cis, 18, 2);
            if (version == 2)
                {
                    line += std::string(1, ' ');
                }
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 4
            line.clear();
            if (version == 2)
                {
                    line += std::string(4, ' ');
                }
            if (version == 3)
                {
                    line += std::string(5, ' ');
                }
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_i_0, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_Crc, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_OMEGA, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_OMEGA_DOT, 18, 2);
            if (version == 2)
                {
                    line += std::string(1, ' ');
                }
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 5
            line.clear();
            if (version == 2)
                {
                    line += std::string(4, ' ');
                }
            if (version == 3)
                {
                    line += std::string(5, ' ');
                }
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_IDOT, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(static_cast<double>(gps_ephemeris_iter->second.i_code_on_L2), 18, 2);
            line += std::string(1, ' ');
            double GPS_week_continuous_number = static_cast<double>(gps_ephemeris_iter->second.i_GPS_week + 1024); // valid until April 7, 2019 (check http://www.colorado.edu/geography/gcraft/notes/gps/gpseow.htm)
            line += Rinex_Printer::doub2for(GPS_week_continuous_number, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(static_cast<double>(gps_ephemeris_iter->second.i_code_on_L2), 18, 2);
            if (version == 2)
                {
                    line += std::string(1, ' ');
                }
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;


            // -------- BROADCAST ORBIT - 6
            line.clear();
            if (version == 2)
                {
                    line += std::string(4, ' ');
                }
            if (version == 3)
                {
                    line += std::string(5, ' ');
                }
            line += Rinex_Printer::doub2for(static_cast<double>(gps_ephemeris_iter->second.i_SV_accuracy), 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(static_cast<double>(gps_ephemeris_iter->second.i_SV_health), 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_TGD, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_IODC, 18, 2);
            if (version == 2)
                {
                    line += std::string(1, ' ');
                }
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 7
            line.clear();
            if (version == 2)
                {
                    line += std::string(4, ' ');
                }
            if (version == 3)
                {
                    line += std::string(5, ' ');
                }
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_TOW, 18, 2);
            line += std::string(1, ' ');
            double curve_fit_interval = 4;

            if (gps_ephemeris_iter->second.satelliteBlock.at(gps_ephemeris_iter->second.i_satellite_PRN).compare("IIA"))
                {
                    // Block II/IIA (Table 20-XI IS-GPS-200E )
                    if ( (gps_ephemeris_iter->second.d_IODC > 239) && (gps_ephemeris_iter->second.d_IODC < 248) )  curve_fit_interval = 8;
                    if ( ( (gps_ephemeris_iter->second.d_IODC > 247) && (gps_ephemeris_iter->second.d_IODC < 256) ) || (gps_ephemeris_iter->second.d_IODC == 496) ) curve_fit_interval = 14;
                    if ( (gps_ephemeris_iter->second.d_IODC > 496) && (gps_ephemeris_iter->second.d_IODC < 504) ) curve_fit_interval = 26;
                    if ( (gps_ephemeris_iter->second.d_IODC > 503) && (gps_ephemeris_iter->second.d_IODC < 511) )  curve_fit_interval = 50;
                    if ( ( (gps_ephemeris_iter->second.d_IODC > 751) && (gps_ephemeris_iter->second.d_IODC < 757) ) || (gps_ephemeris_iter->second.d_IODC == 511) ) curve_fit_interval = 74;
                    if ( gps_ephemeris_iter->second.d_IODC == 757 ) curve_fit_interval = 98;
                }

            if ((gps_ephemeris_iter->second.satelliteBlock.at(gps_ephemeris_iter->second.i_satellite_PRN).compare("IIR") == 0) ||
                    (gps_ephemeris_iter->second.satelliteBlock.at(gps_ephemeris_iter->second.i_satellite_PRN).compare("IIR-M") == 0) ||
                    (gps_ephemeris_iter->second.satelliteBlock.at(gps_ephemeris_iter->second.i_satellite_PRN).compare("IIF") == 0) ||
                    (gps_ephemeris_iter->second.satelliteBlock.at(gps_ephemeris_iter->second.i_satellite_PRN).compare("IIIA") == 0) )
                {
                    // Block IIR/IIR-M/IIF/IIIA (Table 20-XII IS-GPS-200E )
                    if ( (gps_ephemeris_iter->second.d_IODC > 239) && (gps_ephemeris_iter->second.d_IODC < 248))  curve_fit_interval = 8;
                    if ( ( (gps_ephemeris_iter->second.d_IODC > 247) && (gps_ephemeris_iter->second.d_IODC < 256)) || (gps_ephemeris_iter->second.d_IODC == 496) ) curve_fit_interval = 14;
                    if ( ( (gps_ephemeris_iter->second.d_IODC > 496) && (gps_ephemeris_iter->second.d_IODC < 504)) || ( (gps_ephemeris_iter->second.d_IODC > 1020) && (gps_ephemeris_iter->second.d_IODC < 1024) ) ) curve_fit_interval = 26;
                }
            line += Rinex_Printer::doub2for(curve_fit_interval, 18, 2);
            line += std::string(1, ' ');
            line += std::string(18, ' '); // spare
            line += std::string(1, ' ');
            line += std::string(18, ' '); // spare
            if (version == 2)
                {
                    line += std::string(1, ' ');
                }
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
            line.clear();
        }
}


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int,Gps_CNAV_Ephemeris>& eph_map)
{
    std::string line;
    std::map<int,Gps_CNAV_Ephemeris>::const_iterator gps_ephemeris_iter;

    for(gps_ephemeris_iter = eph_map.cbegin();
            gps_ephemeris_iter != eph_map.cend();
            gps_ephemeris_iter++)
        {
            // -------- SV / EPOCH / SV CLK
            boost::posix_time::ptime p_utc_time = Rinex_Printer::compute_GPS_time(gps_ephemeris_iter->second, gps_ephemeris_iter->second.d_Toc);
            std::string timestring = boost::posix_time::to_iso_string(p_utc_time);
            std::string month (timestring, 4, 2);
            std::string day (timestring, 6, 2);
            std::string hour (timestring, 9, 2);
            std::string minutes (timestring, 11, 2);
            std::string seconds (timestring, 13, 2);
            line += satelliteSystem["GPS"];
            if (gps_ephemeris_iter->second.i_satellite_PRN < 10)  line += std::string("0");
            line += boost::lexical_cast<std::string>(gps_ephemeris_iter->second.i_satellite_PRN);
            std::string year (timestring, 0, 4);
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
            line += seconds;
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_A_f0, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_A_f1, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_A_f2, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 1
            line.clear();
            line += std::string(5, ' ');
            // If there is no IODE in CNAV, so we check if Toe in message Type 10, Toe in Message type 11 and Toc in message types 30-37.
            // Whenever these three terms do not match, a data set cutover has occurred and new data must be collected.
            // See IS-GPS-200H, p. 155
            if ( !((gps_ephemeris_iter->second.d_Toe1 == gps_ephemeris_iter->second.d_Toe2) && (gps_ephemeris_iter->second.d_Toe1 == gps_ephemeris_iter->second.d_Toc)) ) // Toe1: Toe in message type 10,  Toe2: Toe in message type 11
                {
                    // Toe1: Toe in message type 10,  Toe2: Toe in message type 11,
                    fake_cnav_iode = fake_cnav_iode + 1;
                    if(fake_cnav_iode == 240) fake_cnav_iode = 1;
                }

            line += Rinex_Printer::doub2for(fake_cnav_iode, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_Crs, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_Delta_n, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_M_0, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 2
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_Cuc, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_e_eccentricity, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_Cus, 18, 2);
            line += std::string(1, ' ');
            const double A_REF = 26559710.0; // See IS-GPS-200H,  pp. 163
            double sqrt_A = sqrt(A_REF + gps_ephemeris_iter->second.d_DELTA_A);
            line += Rinex_Printer::doub2for(sqrt_A, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 3
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(std::max(gps_ephemeris_iter->second.d_Toe1, gps_ephemeris_iter->second.d_Toe2), 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_Cic, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_OMEGA0, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_Cis, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 4
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_i_0, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_Crc, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_OMEGA, 18, 2);
            line += std::string(1, ' ');
            const double OMEGA_DOT_REF = -2.6e-9; // semicircles / s, see IS-GPS-200H pp. 164
            double OMEGA_DOT = OMEGA_DOT_REF + gps_ephemeris_iter->second.d_DELTA_OMEGA_DOT;
            line += Rinex_Printer::doub2for(OMEGA_DOT, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 5
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_IDOT, 18, 2);
            line += std::string(1, ' ');
            // No data flag for L2 P code
            double my_zero = 0.0;
            line += Rinex_Printer::doub2for(my_zero, 18, 2);
            line += std::string(1, ' ');
            double GPS_week_continuous_number = static_cast<double>(gps_ephemeris_iter->second.i_GPS_week + 1024); // valid until April 7, 2019 (check http://www.colorado.edu/geography/gcraft/notes/gps/gpseow.htm)
            line += Rinex_Printer::doub2for(GPS_week_continuous_number, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(my_zero, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 6
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(static_cast<double>(gps_ephemeris_iter->second.i_URA), 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(static_cast<double>(gps_ephemeris_iter->second.i_signal_health), 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_TGD, 18, 2);
            line += std::string(1, ' ');
            // no IODC in CNAV, so we fake it (see above)
            line += Rinex_Printer::doub2for(fake_cnav_iode, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 7
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.d_TOW, 18, 2);
            line += std::string(1, ' ');
            double curve_fit_interval = 3; /// ?? Not defined in CNAV
            line += Rinex_Printer::doub2for(curve_fit_interval, 18, 2);
            line += std::string(1, ' ');
            line += std::string(18, ' '); // spare
            line += std::string(1, ' ');
            line += std::string(18, ' '); // spare
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
            line.clear();
        }
}


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int, Galileo_Ephemeris>& eph_map)
{
    std::string line;
    std::map<int,Galileo_Ephemeris>::const_iterator galileo_ephemeris_iter;
    line.clear();
    for(galileo_ephemeris_iter = eph_map.cbegin();
            galileo_ephemeris_iter != eph_map.cend();
            galileo_ephemeris_iter++)
        {
            // -------- SV / EPOCH / SV CLK
            boost::posix_time::ptime p_utc_time = Rinex_Printer::compute_Galileo_time(galileo_ephemeris_iter->second, galileo_ephemeris_iter->second.t0e_1);
            std::string timestring = boost::posix_time::to_iso_string(p_utc_time);
            std::string month (timestring, 4, 2);
            std::string day (timestring, 6, 2);
            std::string hour (timestring, 9, 2);
            std::string minutes (timestring, 11, 2);
            std::string seconds (timestring, 13, 2);

            line += satelliteSystem["Galileo"];
            if (galileo_ephemeris_iter->second.i_satellite_PRN < 10)  line += std::string("0");
            line += boost::lexical_cast<std::string>(galileo_ephemeris_iter->second.i_satellite_PRN);
            std::string year (timestring, 0, 4);
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
            line += seconds;
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.af0_4, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.af1_4, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.af2_4, 18, 2);

            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 1
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(static_cast<double>(galileo_ephemeris_iter->second.IOD_ephemeris), 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.C_rs_3, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.delta_n_3, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.M0_1, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 2
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.C_uc_3, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.e_1, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.C_us_3, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.A_1, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 3
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.t0e_1, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.C_ic_4, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.OMEGA_0_2, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.C_is_4, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 4
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.i_0_2, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.C_rc_3, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.omega_2, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.OMEGA_dot_3, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 5
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.iDot_2, 18, 2);
            line += std::string(1, ' ');
            //double one = 1.0; // INAV E1-B
            std::string iNAVE1B("1000000001");
            int data_source_INAV = Rinex_Printer::toInt(iNAVE1B, 10);
            line += Rinex_Printer::doub2for(static_cast<double>(data_source_INAV), 18, 2);
            line += std::string(1, ' ');
            double GST_week = static_cast<double>(galileo_ephemeris_iter->second.WN_5);
            double num_GST_rollovers = floor((GST_week + 1024.0) / 4096.0 );
            double Galileo_week_continuous_number = GST_week + 1024.0 + num_GST_rollovers * 4096.0;
            line += Rinex_Printer::doub2for(Galileo_week_continuous_number, 18, 2);
            line += std::string(1, ' ');
            double zero = 0.0;
            line += Rinex_Printer::doub2for(zero, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 6
            line.clear();
            line += std::string(5, ' ');
            //line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.SISA_3, 18, 2);
            line += Rinex_Printer::doub2for(zero, 18, 2);  // *************** CHANGE THIS WHEN GALILEO SIGNAL IS VALID
            line += std::string(1, ' ');
            std::string E1B_HS;
            std::string E5B_HS;
            if(galileo_ephemeris_iter->second.E1B_HS_5 == 0) E1B_HS = "00";
            if(galileo_ephemeris_iter->second.E1B_HS_5 == 1) E1B_HS = "01";
            if(galileo_ephemeris_iter->second.E1B_HS_5 == 2) E1B_HS = "10";
            if(galileo_ephemeris_iter->second.E1B_HS_5 == 3) E1B_HS = "11";
            if(galileo_ephemeris_iter->second.E5b_HS_5 == 0) E5B_HS = "00";
            if(galileo_ephemeris_iter->second.E5b_HS_5 == 1) E5B_HS = "01";
            if(galileo_ephemeris_iter->second.E5b_HS_5 == 2) E5B_HS = "10";
            if(galileo_ephemeris_iter->second.E5b_HS_5 == 3) E5B_HS = "11";

            if(E1B_HS == "11") LOG(WARNING) << "Signal Component currently in Test";
            if(E1B_HS == "10") LOG(WARNING) << "Signal will be out of service";
            if(E1B_HS == "01") LOG(WARNING) << "Signal out of service";
            E1B_HS = "00";  // *************** CHANGE THIS WHEN GALILEO SIGNAL IS VALID

            std::string E1B_DVS = boost::lexical_cast<std::string>(galileo_ephemeris_iter->second.E1B_DVS_5);
            if(E1B_DVS == "1") LOG(WARNING) << "Navigation data without guarantee";
            E1B_DVS = "0"; // *************** CHANGE THIS WHEN GALILEO SIGNAL IS VALID

            std::string SVhealth_str = E5B_HS + boost::lexical_cast<std::string>(galileo_ephemeris_iter->second.E5b_DVS_5)
                            + "11" + "1" + E1B_DVS +  E1B_HS
                            + boost::lexical_cast<std::string>(galileo_ephemeris_iter->second.E1B_DVS_5);
            SVhealth_str = "000000000"; // *************** CHANGE THIS WHEN GALILEO SIGNAL IS VALID
            int SVhealth = Rinex_Printer::toInt(SVhealth_str, 9);
            line += Rinex_Printer::doub2for(static_cast<double>(SVhealth), 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.BGD_E1E5a_5, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.BGD_E1E5b_5, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 7
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.TOW_5, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(zero, 18, 2);
            line += std::string(1, ' ');
            line += std::string(18, ' '); // spare
            line += std::string(1, ' ');
            line += std::string(18, ' '); // spare
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
            line.clear();
        }
}


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int,Glonass_Gnav_Ephemeris>& eph_map)
{
    std::string line;
    std::map<int,Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_ephemeris_iter;

    for(glonass_gnav_ephemeris_iter = eph_map.begin();
            glonass_gnav_ephemeris_iter != eph_map.end();
            glonass_gnav_ephemeris_iter++)
        {
            // -------- SV / EPOCH / SV CLK
            boost::posix_time::ptime p_utc_time = glonass_gnav_ephemeris_iter->second.glot_to_utc(glonass_gnav_ephemeris_iter->second.d_t_b, 0.0);
            std::string timestring = boost::posix_time::to_iso_string(p_utc_time);
            std::string month (timestring, 4, 2);
            std::string day (timestring, 6, 2);
            std::string hour (timestring, 9, 2);
            std::string minutes (timestring, 11, 2);
            std::string seconds (timestring, 13, 2);
            if (version == 2)
                {
                    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(glonass_gnav_ephemeris_iter->second.i_satellite_PRN), 2);
                    line += std::string(1, ' ');
                    std::string year (timestring, 2, 2);
                    line += year;
                    line += std::string(1, ' ');
                    if(boost::lexical_cast<int>(month) < 10)
                        {
                            line += std::string(1, ' ');
                            line += std::string(month, 1, 1);
                        }
                    else
                        {
                            line += month;
                        }
                    line += std::string(1, ' ');
                    if(boost::lexical_cast<int>(day) < 10)
                        {
                            line += std::string(1, ' ');
                            line += std::string(day, 1, 1);
                        }
                    else
                        {
                            line += day;
                        }
                    line += std::string(1, ' ');
                    if(boost::lexical_cast<int>(hour) < 10)
                        {
                            line += std::string(1, ' ');
                            line += std::string(hour, 1, 1);
                        }
                    else
                        {
                            line += hour;
                        }
                    line += std::string(1, ' ');
                    if(boost::lexical_cast<int>(minutes) < 10)
                        {
                            line += std::string(1, ' ');
                            line += std::string(minutes, 1, 1);
                        }
                    else
                        {
                            line += minutes;
                        }
                    line += std::string(1, ' ');
                    if(boost::lexical_cast<int>(seconds) < 10)
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
                            std::string decimal (timestring, 16, 1);
                        }
                    line += decimal;
                    line += std::string(1, ' ');
                    line += Rinex_Printer::doub2for(-glonass_gnav_ephemeris_iter->second.d_tau_c, 18, 2);
                    line += std::string(1, ' ');
                    line += Rinex_Printer::doub2for(glonass_gnav_ephemeris_iter->second.d_gamma_n, 18, 2);
                    line += std::string(1, ' ');
                    line += Rinex_Printer::doub2for(glonass_gnav_ephemeris_iter->second.d_t_k, 18, 2);
                    line += std::string(1, ' ');
                }
            if (version == 3)
                {
                    line += satelliteSystem["GLONASS"];
                    if (glonass_gnav_ephemeris_iter->second.i_satellite_PRN < 10)  line += std::string("0");
                    line += boost::lexical_cast<std::string>(glonass_gnav_ephemeris_iter->second.i_satellite_PRN);
                    std::string year (timestring, 0, 4);
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
                    line += seconds;
                    line += std::string(1, ' ');
                    line += Rinex_Printer::doub2for(-glonass_gnav_ephemeris_iter->second.d_tau_n, 18, 2);
                    line += std::string(1, ' ');
                    line += Rinex_Printer::doub2for(+glonass_gnav_ephemeris_iter->second.d_gamma_n, 18, 2);
                    line += std::string(1, ' ');
                    //TODO need to define this here. what is nd
                    line += Rinex_Printer::doub2for(glonass_gnav_ephemeris_iter->second.d_t_k + p_utc_time.date().day_of_week()*86400, 18, 2);
                }
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 1
            line.clear();
            // TODO Why is this happening here?. The extra space maybe is intended to help with readability
            if (version == 2)
                {
                    line += std::string(3, ' ');
                }
            if (version == 3)
                {
                    line += std::string(4, ' ');
                }
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(glonass_gnav_ephemeris_iter->second.d_Xn, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(glonass_gnav_ephemeris_iter->second.d_VXn, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(glonass_gnav_ephemeris_iter->second.d_AXn, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(glonass_gnav_ephemeris_iter->second.d_B_n, 18, 2);
            if (version == 2)
                {
                    line += std::string(1, ' ');
                }
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 2
            line.clear();
            if (version == 2)
                {
                    line += std::string(3, ' ');
                }
            if (version == 3)
                {
                    line += std::string(4, ' ');
                }
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(glonass_gnav_ephemeris_iter->second.d_Yn, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(glonass_gnav_ephemeris_iter->second.d_VYn, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(glonass_gnav_ephemeris_iter->second.d_AYn, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(glonass_gnav_ephemeris_iter->second.i_satellite_freq_channel, 18, 2);
            if (version == 2)
                {
                    line += std::string(1, ' ');
                }
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- BROADCAST ORBIT - 3
            line.clear();
            if (version == 2)
                {
                    line += std::string(3, ' ');
                }
            if (version == 3)
                {
                    line += std::string(4, ' ');
                }
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(glonass_gnav_ephemeris_iter->second.d_Zn, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(glonass_gnav_ephemeris_iter->second.d_VZn, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(glonass_gnav_ephemeris_iter->second.d_AZn, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(glonass_gnav_ephemeris_iter->second.d_E_n, 18, 2);
            if (version == 2)
                {
                    line += std::string(1, ' ');
                }
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
            line.clear();
        }
}


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int, Gps_Ephemeris>& gps_eph_map, const std::map<int, Galileo_Ephemeris>& galileo_eph_map)
{
    version = 3;
    stringVersion = "3.02";
    Rinex_Printer::log_rinex_nav(out, gps_eph_map);
    Rinex_Printer::log_rinex_nav(out, galileo_eph_map);
}


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int, Gps_Ephemeris>& gps_eph_map, const std::map<int, Glonass_Gnav_Ephemeris>& glonass_gnav_eph_map)
{
    Rinex_Printer::log_rinex_nav(out, gps_eph_map);
    Rinex_Printer::log_rinex_nav(out, glonass_gnav_eph_map);
}


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int, Gps_CNAV_Ephemeris>& gps_eph_map, const std::map<int, Glonass_Gnav_Ephemeris>& glonass_gnav_eph_map)
{
    Rinex_Printer::log_rinex_nav(out, gps_eph_map);
    Rinex_Printer::log_rinex_nav(out, glonass_gnav_eph_map);
}


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int, Galileo_Ephemeris>& galileo_eph_map, const std::map<int, Glonass_Gnav_Ephemeris>& glonass_gnav_eph_map)
{
    version = 3;
    stringVersion = "3.02";
    Rinex_Printer::log_rinex_nav(out, galileo_eph_map);
    Rinex_Printer::log_rinex_nav(out, glonass_gnav_eph_map);
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Glonass_Gnav_Ephemeris& eph, const double d_TOW_first_observation, const std::string glonass_bands)
{
    if(eph.d_m){} //Avoid compiler warning
    std::string line;
    std::map<int,Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_ephemeris_iter;

    // -------- Line 1
    line = std::string(5, ' ');
    line += stringVersion;
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem["GLONASS"];
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 2
    line.clear();
    if (version == 2)
        {
            line += Rinex_Printer::leftJustify("BLANK OR G = GPS,  R = GLONASS,  E = GALILEO,  M = MIXED", 60);
        }
    if (version == 3)
        {
            line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED", 60);
        }
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GLONASS OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See http://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line MARKER NAME
    line.clear();
    line += Rinex_Printer::leftJustify("DEFAULT MARKER NAME", 60); // put a flag or a property,
    line += Rinex_Printer::leftJustify("MARKER NAME", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line MARKER TYPE
    if (version == 2)
        {
            line.clear();
            line += Rinex_Printer::leftJustify("GROUND_CRAFT", 20); // put a flag or a property
            line += std::string(40, ' ');
            line += Rinex_Printer::leftJustify("MARKER NUMBER", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
        }
    if (version == 3)
        {
            line.clear();
            line += Rinex_Printer::leftJustify("GROUND_CRAFT", 20); // put a flag or a property
            line += std::string(40, ' ');
            line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
        }

    // -------- Line OBSERVER / AGENCY
    line.clear();
    std::string username;
    char c_username[20] = {0};
    int nGet = getlogin_r(c_username, sizeof(c_username) - 1);
    if (nGet == 0)
        {
            username = c_username;
        }
    else
        {
            username = "UNKNOWN USER";
        }
    line += leftJustify(username, 20);
    line += Rinex_Printer::leftJustify("CTTC", 40); // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20); // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20); // add flag and property
    //line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if(gnss_sdr_version.length() > 20) gnss_sdr_version.resize(9, ' ');
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << std::endl;

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);  // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- APPROX POSITION  (optional for moving platforms)
    // put here real data!
    double antena_x = 0.0;
    double antena_y = 0.0;
    double antena_z = 0.0;
    line.clear();
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_x, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_y, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_z, 4), 14);
    line += std::string(18, ' ');
    line += Rinex_Printer::leftJustify("APPROX POSITION XYZ", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- ANTENNA: DELTA H/E/N
    // put here real data!
    double antena_h = 0.0;
    double antena_e = 0.0;
    double antena_n = 0.0;
    line.clear();
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_h, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_e, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_n, 4), 14);
    line += std::string(18, ' ');
    line += Rinex_Printer::leftJustify("ANTENNA: DELTA H/E/N", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- SYS / OBS TYPES
    if (version == 3)
        {
            // -------- SYS / OBS TYPES
            // one line per available system
            line.clear();
            line += satelliteSystem["GLONASS"];
            line += std::string(2, ' ');
            std::stringstream strm;
            numberTypesObservations = 4;
            strm << numberTypesObservations;
            line += Rinex_Printer::rightJustify(strm.str(), 3);

            std::string signal_ = "1G";
            std::size_t found_1G = glonass_bands.find(signal_);
            signal_ = "2G";
            std::size_t found_2G = glonass_bands.find(signal_);

            if(found_1G != std::string::npos)
                {
                    line += std::string(1, ' ');
                    line += observationType["PSEUDORANGE"];
                    line += observationCode["GLONASS_G1_CA"];
                    line += std::string(1, ' ');
                    line += observationType["CARRIER_PHASE"];
                    line += observationCode["GLONASS_G1_CA"];
                    line += std::string(1, ' ');
                    line += observationType["DOPPLER"];
                    line += observationCode["GLONASS_G1_CA"];
                    line += std::string(1, ' ');
                    line += observationType["SIGNAL_STRENGTH"];
                    line += observationCode["GLONASS_G1_CA"];
                }

            if(found_2G != std::string::npos)
                {
                    line += std::string(1, ' ');
                    line += observationType["PSEUDORANGE"];
                    line += observationCode["GLONASS_G2_CA"];
                    line += std::string(1, ' ');
                    line += observationType["CARRIER_PHASE"];
                    line += observationCode["GLONASS_G2_CA"];
                    line += std::string(1, ' ');
                    line += observationType["DOPPLER"];
                    line += observationCode["GLONASS_G2_CA"];
                    line += std::string(1, ' ');
                    line += observationType["SIGNAL_STRENGTH"];
                    line += observationCode["GLONASS_G2_CA"];
                }

            line += std::string(60-line.size(), ' ');
            line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
        }
    if (version == 2)
        {
            // -------- SYS / OBS TYPES
            line.clear();
            std::stringstream strm;
            strm << numberTypesObservations;
            line += Rinex_Printer::rightJustify(strm.str(), 6);
            // per type of observation
            // GLONASS L1 C/A PSEUDORANGE
            line += Rinex_Printer::rightJustify(observationType["PSEUDORANGE_CA_v2"], 5);
            line += observationCode["GLONASS_G1_CA_v2"];
            // GLONASS L1 PHASE
            line += Rinex_Printer::rightJustify(observationType["CARRIER_PHASE_CA_v2"], 5);
            line += observationCode["GLONASS_G1_CA_v2"];
            // GLONASS DOPPLER L1
            line += Rinex_Printer::rightJustify(observationType["DOPPLER_v2"], 5);
            line += observationCode["GLONASS_G1_CA_v2"];
            // GLONASS L1 SIGNAL STRENGTH
            line += Rinex_Printer::rightJustify(observationType["SIGNAL_STRENGTH_v2"], 5);
            line += observationCode["GLONASS_G1_CA_v2"];
            line += std::string(60-line.size(), ' ');
            line += Rinex_Printer::leftJustify("# / TYPES OF OBSERV", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
        }

    // -------- Signal Strength units (Only version 3)
    if (version == 3)
        {
            // -------- Signal Strength units
            line.clear();
            line += Rinex_Printer::leftJustify("DBHZ", 20);
            line += std::string(40, ' ');
            line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
        }

    // -------- TIME OF FIRST OBS
    boost::posix_time::ptime p_utc_time = Rinex_Printer::compute_UTC_time(eph,d_TOW_first_observation);
    std::string timestring = boost::posix_time::to_iso_string(p_utc_time);
    std::string year (timestring, 0, 4);
    std::string month (timestring, 4, 2);
    std::string day (timestring, 6, 2);
    std::string hour (timestring, 9, 2);
    std::string minutes (timestring, 11, 2);
    double intpart = 0;
    double seconds = p_utc_time.time_of_day().seconds() + modf (d_TOW_first_observation , &intpart);
    line.clear();
    line += Rinex_Printer::rightJustify(year, 6);
    line += Rinex_Printer::rightJustify(month, 6);
    line += Rinex_Printer::rightJustify(day, 6);
    line += Rinex_Printer::rightJustify(hour, 6);
    line += Rinex_Printer::rightJustify(minutes, 6);
    line += Rinex_Printer::rightJustify(asString(seconds, 7), 13);
    line += Rinex_Printer::rightJustify(std::string("GLO"), 8);
    line += std::string(9, ' ');
    line += Rinex_Printer::leftJustify("TIME OF FIRST OBS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- GLONASS SLOT / FRQ # (On;y version 3)
    if (version == 3)
        {
            // -------- GLONASS SLOT / FRQ #
            // TODO Need to provide system with list of all satellites and update this accordingly
            line.clear();
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0), 3); // Number of satellites in list
            line += std::string(1, ' ');
            line += satelliteSystem["GLONASS"];
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0), 2); // Slot Number
            line += std::string(1, ' ');
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0), 2); // Frequency Number
            line += std::string(1, ' ');
            line += std::string(60-line.size(), ' ');
            line += Rinex_Printer::leftJustify("GLONASS SLOT / FRQ #", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- GLONASS CODE/PHS/BIS
            // No GLONASS Phase bias correction used to align code and phase observations.
            line.clear();
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GLONASS_G1_CA"];
            line += std::string(1, ' ');
            line += Rinex_Printer::rightJustify(asString(0.0, 3), 8);
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GLONASS_G1_P"];
            line += std::string(1, ' ');
            line += Rinex_Printer::rightJustify(asString(0.0, 3), 8);
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GLONASS_G2_CA"];
            line += std::string(1, ' ');
            line += Rinex_Printer::rightJustify(asString(0.0, 3), 8);
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GLONASS_G2_P"];
            line += std::string(1, ' ');
            line += Rinex_Printer::rightJustify(asString(0.0, 3), 8);
            line += std::string(60-line.size(), ' ');
            line += Rinex_Printer::leftJustify("GLONASS COD/PHS/BIS", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
        }

    // -------- END OF HEADER
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Gps_Ephemeris& gps_eph, const Glonass_Gnav_Ephemeris& glonass_gnav_eph, const double d_TOW_first_observation, const std::string glonass_bands)
{
    if(glonass_gnav_eph.d_m){} // avoid warning, not needed
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += stringVersion;
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem["Mixed"];
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("MIXED (GPS/GLO) OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See http://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line MARKER NAME / TYPE
    if (version == 2)
        {
            line.clear();
            line += Rinex_Printer::leftJustify("GROUND_CRAFT", 20); // put a flag or a property
            line += std::string(40, ' ');
            line += Rinex_Printer::leftJustify("MARKER NUMBER", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
        }
    if (version == 3)
        {
            line.clear();
            line += Rinex_Printer::leftJustify("GROUND_CRAFT", 20); // put a flag or a property
            line += std::string(40, ' ');
            line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
        }

    // -------- Line MARKER TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("NON_GEODETIC", 20); // put a flag or a property
    line += std::string(40, ' ');
    line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line OBSERVER / AGENCY
    line.clear();
    std::string username;
    char c_username[20] = {0};
    int nGet = getlogin_r(c_username, sizeof(c_username) - 1);
    if (nGet == 0)
        {
            username = c_username;
        }
    else
        {
            username = "UNKNOWN USER";
        }
    line += leftJustify(username, 20);
    line += Rinex_Printer::leftJustify("CTTC", 40); // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20); // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20); // add flag and property
    //line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if(gnss_sdr_version.length() > 20) gnss_sdr_version.resize(9, ' ');
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << std::endl;

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);  // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- APPROX POSITION  (optional for moving platforms)
    // put here real data!
    double antena_x = 0.0;
    double antena_y = 0.0;
    double antena_z = 0.0;
    line.clear();
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_x, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_y, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_z, 4), 14);
    line += std::string(18, ' ');
    line += Rinex_Printer::leftJustify("APPROX POSITION XYZ", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- ANTENNA: DELTA H/E/N
    // put here real data!
    double antena_h = 0.0;
    double antena_e = 0.0;
    double antena_n = 0.0;
    line.clear();
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_h, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_e, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_n, 4), 14);
    line += std::string(18, ' ');
    line += Rinex_Printer::leftJustify("ANTENNA: DELTA H/E/N", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- SYS / OBS TYPES
    if(version == 3)
        {
            // one line per available system
            line.clear();
            line += satelliteSystem["GPS"];
            line += std::string(2, ' ');
            std::stringstream strm;
            numberTypesObservations = 4;
            strm << numberTypesObservations;
            line += Rinex_Printer::rightJustify(strm.str(), 3);
            // per type of observation
            // GPS L1 PSEUDORANGE
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GPS_L1_CA"];
            // GPS L1 PHASE
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GPS_L1_CA"];
            // GPS DOPPLER L1
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GPS_L1_CA"];
            // GPS L1 CA SIGNAL STRENGTH
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GPS_L1_CA"];
            line += std::string(60-line.size(), ' ');
            line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // Find GLONASS Signal in Mixed file
            unsigned int number_of_observations_glo = 0;
            std::string signal_("1G");
            std::size_t found_1G = glonass_bands.find(signal_);
            if(found_1G != std::string::npos)
                {
                    number_of_observations_glo = number_of_observations_glo + 4;
                }
            signal_ = "2G";
            std::size_t found_2G = glonass_bands.find(signal_);
            if(found_2G != std::string::npos)
                {
                    number_of_observations_glo = number_of_observations_glo + 4;
                }
            line.clear();
            line += satelliteSystem["GLONASS"];
            line += std::string(2, ' ');
            line += Rinex_Printer::rightJustify(std::to_string(number_of_observations_glo), 3);
            if(found_1G != std::string::npos)
                {
                    line += std::string(1, ' ');
                    line += observationType["PSEUDORANGE"];
                    line += observationCode["GLONASS_G1_CA"];
                    line += std::string(1, ' ');
                    line += observationType["CARRIER_PHASE"];
                    line += observationCode["GLONASS_G1_CA"];
                    line += std::string(1, ' ');
                    line += observationType["DOPPLER"];
                    line += observationCode["GLONASS_G1_CA"];
                    line += std::string(1, ' ');
                    line += observationType["SIGNAL_STRENGTH"];
                    line += observationCode["GLONASS_G1_CA"];
                }
            if(found_2G != std::string::npos)
                {
                    line += std::string(1, ' ');
                    line += observationType["PSEUDORANGE"];
                    line += observationCode["GLONASS_G2_CA"];
                    line += std::string(1, ' ');
                    line += observationType["CARRIER_PHASE"];
                    line += observationCode["GLONASS_G2_CA"];
                    line += std::string(1, ' ');
                    line += observationType["DOPPLER"];
                    line += observationCode["GLONASS_G2_CA"];
                    line += std::string(1, ' ');
                    line += observationType["SIGNAL_STRENGTH"];
                    line += observationCode["GLONASS_G2_CA"];
                }
            line += std::string(60-line.size(), ' ');
            line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
        }
    if(version == 2)
        {
            // -------- SYS / OBS TYPES
            line.clear();
            std::stringstream strm;
            strm << numberTypesObservations;
            line += Rinex_Printer::rightJustify(strm.str(), 6);
            // per type of observation
            // GLONASS L1 C/A PSEUDORANGE
            line += Rinex_Printer::rightJustify(observationType["PSEUDORANGE_CA_v2"], 5);
            line += observationCode["GLONASS_G1_CA_v2"];
            // GLONASS L1 PHASE
            line += Rinex_Printer::rightJustify(observationType["CARRIER_PHASE_CA_v2"], 5);
            line += observationCode["GLONASS_G1_CA_v2"];
            // GLONASS DOPPLER L1
            line += Rinex_Printer::rightJustify(observationType["DOPPLER_v2"], 5);
            line += observationCode["GLONASS_G1_CA_v2"];
            // GLONASS L1 SIGNAL STRENGTH
            line += Rinex_Printer::rightJustify(observationType["SIGNAL_STRENGTH_v2"], 5);
            line += observationCode["GLONASS_G1_CA_v2"];
            line += std::string(60-line.size(), ' ');
            line += Rinex_Printer::leftJustify("# / TYPES OF OBSERV", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
        }

    // -------- Signal Strength units (only version 3)
    if(version == 3)
        {
            line.clear();
            line += Rinex_Printer::leftJustify("DBHZ", 20);
            line += std::string(40, ' ');
            line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
        }

    // -------- TIME OF FIRST OBS
    line.clear();
    boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(gps_eph, d_TOW_first_observation);
    std::string timestring=boost::posix_time::to_iso_string(p_gps_time);
    std::string year (timestring, 0, 4);
    std::string month (timestring, 4, 2);
    std::string day (timestring, 6, 2);
    std::string hour (timestring, 9, 2);
    std::string minutes (timestring, 11, 2);
    double gps_t = d_TOW_first_observation;
    double seconds = fmod(gps_t, 60);
    line += Rinex_Printer::rightJustify(year, 6);
    line += Rinex_Printer::rightJustify(month, 6);
    line += Rinex_Printer::rightJustify(day, 6);
    line += Rinex_Printer::rightJustify(hour, 6);
    line += Rinex_Printer::rightJustify(minutes, 6);
    line += Rinex_Printer::rightJustify(asString(seconds, 7), 13);
    line += Rinex_Printer::rightJustify(std::string("GPS"), 8);
    line += std::string(9, ' ');
    line += Rinex_Printer::leftJustify("TIME OF FIRST OBS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- GLONASS SLOT / FRQ # (On;y version 3)
    if (version == 3)
        {
            // -------- GLONASS SLOT / FRQ #
            // TODO Need to provide system with list of all satellites and update this accordingly
            line.clear();
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0), 3); // Number of satellites in list
            line += std::string(1, ' ');
            line += satelliteSystem["GLONASS"];
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0), 2); // Slot Number
            line += std::string(1, ' ');
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0), 2); // Frequency Number
            line += std::string(1, ' ');
            line += std::string(60-line.size(), ' ');
            line += Rinex_Printer::leftJustify("GLONASS SLOT / FRQ #", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            // -------- GLONASS CODE/PHS/BIS
            // No GLONASS Phase bias correction used to align code and phase observations.
            line.clear();
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GLONASS_G1_CA"];
            line += std::string(1, ' ');
            line += Rinex_Printer::rightJustify(asString(0.0, 3), 8);
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GLONASS_G1_P"];
            line += std::string(1, ' ');
            line += Rinex_Printer::rightJustify(asString(0.0, 3), 8);
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GLONASS_G2_CA"];
            line += std::string(1, ' ');
            line += Rinex_Printer::rightJustify(asString(0.0, 3), 8);
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GLONASS_G2_P"];
            line += std::string(1, ' ');
            line += Rinex_Printer::rightJustify(asString(0.0, 3), 8);
            line += std::string(60-line.size(), ' ');
            line += Rinex_Printer::leftJustify("GLONASS COD/PHS/BIS", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
        }

    // -------- end of header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Gps_CNAV_Ephemeris& gps_eph, const Glonass_Gnav_Ephemeris& glonass_gnav_eph, const double d_TOW_first_observation, const std::string glonass_bands)
{
    if(glonass_gnav_eph.d_m){} // avoid warning, not needed
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += stringVersion;
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem["Mixed"];
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("MIXED (GPS/GLO) OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See http://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line MARKER NAME
    if (version == 2)
        {
            line.clear();
            line += Rinex_Printer::leftJustify("GROUND_CRAFT", 20); // put a flag or a property
            line += std::string(40, ' ');
            line += Rinex_Printer::leftJustify("MARKER NUMBER", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
        }
    if (version == 3)
        {
            line.clear();
            line += Rinex_Printer::leftJustify("GROUND_CRAFT", 20); // put a flag or a property
            line += std::string(40, ' ');
            line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
        }

    // -------- Line MARKER TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("NON_GEODETIC", 20); // put a flag or a property
    line += std::string(40, ' ');
    line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line OBSERVER / AGENCY
    line.clear();
    std::string username;
    char c_username[20] = {0};
    int nGet = getlogin_r(c_username, sizeof(c_username) - 1);
    if (nGet == 0)
        {
            username = c_username;
        }
    else
        {
            username = "UNKNOWN USER";
        }
    line += leftJustify(username, 20);
    line += Rinex_Printer::leftJustify("CTTC", 40); // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20); // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20); // add flag and property
    //line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if(gnss_sdr_version.length() > 20) gnss_sdr_version.resize(9, ' ');
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << std::endl;

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);  // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- APPROX POSITION  (optional for moving platforms)
    // put here real data!
    double antena_x = 0.0;
    double antena_y = 0.0;
    double antena_z = 0.0;
    line.clear();
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_x, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_y, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_z, 4), 14);
    line += std::string(18, ' ');
    line += Rinex_Printer::leftJustify("APPROX POSITION XYZ", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- ANTENNA: DELTA H/E/N
    // put here real data!
    double antena_h = 0.0;
    double antena_e = 0.0;
    double antena_n = 0.0;
    line.clear();
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_h, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_e, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_n, 4), 14);
    line += std::string(18, ' ');
    line += Rinex_Printer::leftJustify("ANTENNA: DELTA H/E/N", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- SYS / OBS TYPES
    // one line per available system
    line.clear();
    line += satelliteSystem["GPS"];
    line += std::string(2, ' ');
    std::stringstream strm;
    numberTypesObservations = 4;
    strm << numberTypesObservations;
    line += Rinex_Printer::rightJustify(strm.str(), 3);
    // per type of observation
    // GPS L1 PSEUDORANGE
    line += std::string(1, ' ');
    line += observationType["PSEUDORANGE"];
    line += observationCode["GPS_L2_L2CM"];
    // GPS L1 PHASE
    line += std::string(1, ' ');
    line += observationType["CARRIER_PHASE"];
    line += observationCode["GPS_L2_L2CM"];
    // GPS DOPPLER L1
    line += std::string(1, ' ');
    line += observationType["DOPPLER"];
    line += observationCode["GPS_L2_L2CM"];
    // GPS L1 CA SIGNAL STRENGTH
    line += std::string(1, ' ');
    line += observationType["SIGNAL_STRENGTH"];
    line += observationCode["GPS_L2_L2CM"];
    line += std::string(60-line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // Find GLONASS Signal in Mixed file
    unsigned int number_of_observations_glo = 0;
    std::string signal_("1G");
    std::size_t found_1G = glonass_bands.find(signal_);
    if(found_1G != std::string::npos)
        {
            number_of_observations_glo = number_of_observations_glo + 4;
        }
    signal_ = "2G";
    std::size_t found_2G = glonass_bands.find(signal_);
    if(found_2G != std::string::npos)
        {
            number_of_observations_glo = number_of_observations_glo + 4;
        }
    line.clear();
    line += satelliteSystem["GLONASS"];
    line += std::string(2, ' ');
    line += Rinex_Printer::rightJustify(std::to_string(number_of_observations_glo), 3);
    if(found_1G != std::string::npos)
        {
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GLONASS_G1_CA"];
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GLONASS_G1_CA"];
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GLONASS_G1_CA"];
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GLONASS_G1_CA"];
        }
    if(found_2G != std::string::npos)
        {
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GLONASS_G2_CA"];
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GLONASS_G2_CA"];
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GLONASS_G2_CA"];
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GLONASS_G2_CA"];
        }
    line += std::string(60-line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Signal Strength units (only version 3)
    line.clear();
    line += Rinex_Printer::leftJustify("DBHZ", 20);
    line += std::string(40, ' ');
    line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- TIME OF FIRST OBS
    line.clear();
    boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(gps_eph, d_TOW_first_observation);
    std::string timestring=boost::posix_time::to_iso_string(p_gps_time);
    std::string year (timestring, 0, 4);
    std::string month (timestring, 4, 2);
    std::string day (timestring, 6, 2);
    std::string hour (timestring, 9, 2);
    std::string minutes (timestring, 11, 2);
    double gps_t = d_TOW_first_observation;
    double seconds = fmod(gps_t, 60);
    line += Rinex_Printer::rightJustify(year, 6);
    line += Rinex_Printer::rightJustify(month, 6);
    line += Rinex_Printer::rightJustify(day, 6);
    line += Rinex_Printer::rightJustify(hour, 6);
    line += Rinex_Printer::rightJustify(minutes, 6);
    line += Rinex_Printer::rightJustify(asString(seconds, 7), 13);
    line += Rinex_Printer::rightJustify(std::string("GPS"), 8);
    line += std::string(9, ' ');
    line += Rinex_Printer::leftJustify("TIME OF FIRST OBS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- GLONASS SLOT / FRQ #
    // TODO Need to provide system with list of all satellites and update this accordingly
    line.clear();
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0), 3); // Number of satellites in list
    line += std::string(1, ' ');
    line += satelliteSystem["GLONASS"];
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0), 2); // Slot Number
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(0), 2); // Frequency Number
    line += std::string(1, ' ');
    line += std::string(60-line.size(), ' ');
    line += Rinex_Printer::leftJustify("GLONASS SLOT / FRQ #", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- GLONASS CODE/PHS/BIS
    // No GLONASS Phase bias correction used to align code and phase observations.
    line.clear();
    line += std::string(1, ' ');
    line += observationType["PSEUDORANGE"];
    line += observationCode["GLONASS_G1_CA"];
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(asString(0.0, 3), 8);
    line += std::string(1, ' ');
    line += observationType["PSEUDORANGE"];
    line += observationCode["GLONASS_G1_P"];
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(asString(0.0, 3), 8);
    line += std::string(1, ' ');
    line += observationType["PSEUDORANGE"];
    line += observationCode["GLONASS_G2_CA"];
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(asString(0.0, 3), 8);
    line += std::string(1, ' ');
    line += observationType["PSEUDORANGE"];
    line += observationCode["GLONASS_G2_P"];
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(asString(0.0, 3), 8);
    line += std::string(60-line.size(), ' ');
    line += Rinex_Printer::leftJustify("GLONASS COD/PHS/BIS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- end of header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Galileo_Ephemeris& galileo_eph, const Glonass_Gnav_Ephemeris& glonass_gnav_eph, const double d_TOW_first_observation, const std::string galileo_bands, const std::string glonass_bands)
{
    if(glonass_gnav_eph.d_m){} // avoid warning, not needed
    std::string line;
    version = 3;

    // -------- Line 1
    line = std::string(5, ' ');
    line += "3.02";
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem["Mixed"];
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("MIXED (GALILEO/GLONASS) OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See http://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line MARKER NAME
    line.clear();
    line += Rinex_Printer::leftJustify("DEFAULT MARKER NAME", 60); // put a flag or a property,
    line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line MARKER TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("NON_GEODETIC", 20); // put a flag or a property
    line += std::string(40, ' ');
    line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line OBSERVER / AGENCY
    line.clear();
    std::string username;
    char c_username[20] = {0};
    int nGet = getlogin_r(c_username, sizeof(c_username) - 1);
    if (nGet == 0)
        {
            username = c_username;
        }
    else
        {
            username = "UNKNOWN USER";
        }
    line += leftJustify(username, 20);
    line += Rinex_Printer::leftJustify("CTTC", 40); // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20); // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20); // add flag and property
    //line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if(gnss_sdr_version.length() > 20) gnss_sdr_version.resize(9, ' ');
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << std::endl;

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);  // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- APPROX POSITION  (optional for moving platforms)
    // put here real data!
    double antena_x = 0.0;
    double antena_y = 0.0;
    double antena_z = 0.0;
    line.clear();
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_x, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_y, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_z, 4), 14);
    line += std::string(18, ' ');
    line += Rinex_Printer::leftJustify("APPROX POSITION XYZ", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- ANTENNA: DELTA H/E/N
    // put here real data!
    double antena_h = 0.0;
    double antena_e = 0.0;
    double antena_n = 0.0;
    line.clear();
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_h, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_e, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_n, 4), 14);
    line += std::string(18, ' ');
    line += Rinex_Printer::leftJustify("ANTENNA: DELTA H/E/N", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- SYS / OBS TYPES
    line.clear();
    unsigned int number_of_observations_gal = 0;
    std::string signal_("1B");
    std::size_t found_1B = galileo_bands.find(signal_);
    if(found_1B != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }
    signal_ = "5X";
    std::size_t found_5X = galileo_bands.find(signal_);
    if(found_5X != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }

    line.clear();
    signal_ = "7X";
    std::size_t found_7X = galileo_bands.find(signal_);
    if(found_7X != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }


    line += satelliteSystem["Galileo"];
    line += std::string(2, ' ');
    line += Rinex_Printer::rightJustify(std::to_string(number_of_observations_gal), 3);

    if(found_1B != std::string::npos)
        {
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GALILEO_E1_B"];
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GALILEO_E1_B"];
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GALILEO_E1_B"];
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GALILEO_E1_B"];
        }

    if(found_5X != std::string::npos)
        {
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GALILEO_E5a_IQ"];
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GALILEO_E5a_IQ"];
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GALILEO_E5a_IQ"];
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GALILEO_E5a_IQ"];
        }

    if(found_7X != std::string::npos)
        {
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GALILEO_E5b_IQ"];
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GALILEO_E5b_IQ"];
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GALILEO_E5b_IQ"];
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GALILEO_E5b_IQ"];
        }

    line += std::string(60-line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    line.clear();
    unsigned int number_of_observations_glo = 0;
    signal_ = "1G";
    std::size_t found_1G = glonass_bands.find(signal_);
    if(found_1G != std::string::npos)
        {
            number_of_observations_glo = number_of_observations_glo + 4;
        }
    signal_ = "2G";
    std::size_t found_2G = glonass_bands.find(signal_);
    if(found_2G != std::string::npos)
        {
            number_of_observations_glo = number_of_observations_glo + 4;
        }

    line += satelliteSystem["GLONASS"];
    line += std::string(2, ' ');
    line += Rinex_Printer::rightJustify(std::to_string(number_of_observations_glo), 3);

    if(found_1G != std::string::npos)
        {
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GLONASS_L1_CA"];
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GLONASS_L1_CA"];
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GLONASS_L1_CA"];
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GLONASS_L1_CA"];
        }

    if(found_2G != std::string::npos)
        {
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GLONASS_L2_CA"];
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GLONASS_L2_CA"];
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GLONASS_L2_CA"];
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GLONASS_L2_CA"];
        }

    line += std::string(60-line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Signal Strength units
    line.clear();
    line += Rinex_Printer::leftJustify("DBHZ", 20);
    line += std::string(40, ' ');
    line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- TIME OF FIRST OBS
    line.clear();
    boost::posix_time::ptime p_galileo_time = Rinex_Printer::compute_Galileo_time(galileo_eph, d_TOW_first_observation);
    std::string timestring=boost::posix_time::to_iso_string(p_galileo_time);
    std::string year (timestring, 0, 4);
    std::string month (timestring, 4, 2);
    std::string day (timestring, 6, 2);
    std::string hour (timestring, 9, 2);
    std::string minutes (timestring, 11, 2);
    double galileo_t = d_TOW_first_observation;
    double seconds = fmod(galileo_t, 60);
    line += Rinex_Printer::rightJustify(year, 6);
    line += Rinex_Printer::rightJustify(month, 6);
    line += Rinex_Printer::rightJustify(day, 6);
    line += Rinex_Printer::rightJustify(hour, 6);
    line += Rinex_Printer::rightJustify(minutes, 6);
    line += Rinex_Printer::rightJustify(asString(seconds, 7), 13);
    line += Rinex_Printer::rightJustify(std::string("Galileo"), 8);
    line += std::string(9, ' ');
    line += Rinex_Printer::leftJustify("TIME OF FIRST OBS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- end of header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Gps_Ephemeris& eph, const double d_TOW_first_observation)
{
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += stringVersion;
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem["GPS"];
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 2
    line.clear();
    if (version == 2)
        {
            line += Rinex_Printer::leftJustify("BLANK OR G = GPS,  R = GLONASS,  E = GALILEO,  M = MIXED", 60);
        }
    if (version == 3)
        {
            line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED", 60);
        }
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GPS OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See http://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line MARKER NAME
    line.clear();
    line += Rinex_Printer::leftJustify("DEFAULT MARKER NAME", 60); // put a flag or a property,
    line += Rinex_Printer::leftJustify("MARKER NAME", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line MARKER TYPE
    //line.clear();
    //line += Rinex_Printer::leftJustify("GROUND_CRAFT", 20); // put a flag or a property
    //line += std::string(40, ' ');
    //line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
    //Rinex_Printer::lengthCheck(line);
    //out << line << std::endl;

    // -------- Line OBSERVER / AGENCY
    line.clear();
    std::string username;
    char c_username[20] = {0};
    int nGet = getlogin_r(c_username, sizeof(c_username) - 1);
    if (nGet == 0)
        {
            username = c_username;
        }
    else
        {
            username = "UNKNOWN USER";
        }
    line += leftJustify(username, 20);
    line += Rinex_Printer::leftJustify("CTTC", 40); // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20); // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20); // add flag and property
    //line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if(gnss_sdr_version.length() > 20) gnss_sdr_version.resize(9, ' ');
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << std::endl;

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);  // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- APPROX POSITION  (optional for moving platforms)
    // put here real data!
    double antena_x = 0.0;
    double antena_y = 0.0;
    double antena_z = 0.0;
    line.clear();
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_x, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_y, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_z, 4), 14);
    line += std::string(18, ' ');
    line += Rinex_Printer::leftJustify("APPROX POSITION XYZ", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- ANTENNA: DELTA H/E/N
    // put here real data!
    double antena_h = 0.0;
    double antena_e = 0.0;
    double antena_n = 0.0;
    line.clear();
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_h, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_e, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_n, 4), 14);
    line += std::string(18, ' ');
    line += Rinex_Printer::leftJustify("ANTENNA: DELTA H/E/N", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    if (version == 2)
        {
            // --------- WAVELENGHT FACTOR
            // put here real data!
            line.clear();
            line +=Rinex_Printer::rightJustify("1",6);
            line +=Rinex_Printer::rightJustify("1",6);
            line += std::string(48, ' ');
            line += Rinex_Printer::leftJustify("WAVELENGTH FACT L1/2", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
        }

    if (version == 3)
        {
            // -------- SYS / OBS TYPES
            // one line per available system
            line.clear();
            line += satelliteSystem["GPS"];
            line += std::string(2, ' ');
            std::stringstream strm;
            numberTypesObservations = 4;
            strm << numberTypesObservations;
            line += Rinex_Printer::rightJustify(strm.str(), 3);
            // per type of observation
            // GPS L1 PSEUDORANGE
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GPS_L1_CA"];
            // GPS L1 PHASE
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GPS_L1_CA"];
            // GPS DOPPLER L1
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GPS_L1_CA"];
            // GPS L1 CA SIGNAL STRENGTH
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GPS_L1_CA"];

            line += std::string(60-line.size(), ' ');
            line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
        }

    if (version == 2)
        {
            // -------- SYS / OBS TYPES
            line.clear();
            std::stringstream strm;
            strm << numberTypesObservations;
            line += Rinex_Printer::rightJustify(strm.str(), 6);
            // per type of observation
            // GPS L1 PSEUDORANGE
            line += Rinex_Printer::rightJustify(observationType["PSEUDORANGE_CA_v2"], 5);
            line += observationCode["GPS_L1_CA_v2"];
            // GPS L1 PHASE
            line += Rinex_Printer::rightJustify(observationType["CARRIER_PHASE_CA_v2"], 5);
            line += observationCode["GPS_L1_CA_v2"];
            // GPS DOPPLER L1
            line += Rinex_Printer::rightJustify(observationType["DOPPLER_v2"], 5);
            line += observationCode["GPS_L1_CA_v2"];
            // GPS L1 SIGNAL STRENGTH
            line += Rinex_Printer::rightJustify(observationType["SIGNAL_STRENGTH_v2"], 5);
            line += observationCode["GPS_L1_CA_v2"];
            line += std::string(60-line.size(), ' ');
            line += Rinex_Printer::leftJustify("# / TYPES OF OBSERV", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
        }

    if (version == 3)
        {
            // -------- Signal Strength units
            line.clear();
            line += Rinex_Printer::leftJustify("DBHZ", 20);
            line += std::string(40, ' ');
            line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;
        }

    // -------- TIME OF FIRST OBS
    line.clear();
    boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(eph,d_TOW_first_observation);
    std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    std::string year (timestring, 0, 4);
    std::string month (timestring, 4, 2);
    std::string day (timestring, 6, 2);
    std::string hour (timestring, 9, 2);
    std::string minutes (timestring, 11, 2);
    double gps_t = d_TOW_first_observation;
    double seconds = fmod(gps_t, 60);
    line += Rinex_Printer::rightJustify(year, 6);
    line += Rinex_Printer::rightJustify(month, 6);
    line += Rinex_Printer::rightJustify(day, 6);
    line += Rinex_Printer::rightJustify(hour, 6);
    line += Rinex_Printer::rightJustify(minutes, 6);
    line += Rinex_Printer::rightJustify(asString(seconds, 7), 13);
    line += Rinex_Printer::rightJustify(std::string("GPS"), 8);
    line += std::string(9, ' ');
    line += Rinex_Printer::leftJustify("TIME OF FIRST OBS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- SYS /PHASE SHIFTS

    // -------- end of header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;
}


void Rinex_Printer::rinex_obs_header(std::fstream & out, const Gps_CNAV_Ephemeris & eph, const double d_TOW_first_observation)
{
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += stringVersion;
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem["GPS"];
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GPS OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See http://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line MARKER NAME
    line.clear();
    line += Rinex_Printer::leftJustify("DEFAULT MARKER NAME", 60); // put a flag or a property,
    line += Rinex_Printer::leftJustify("MARKER NAME", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line MARKER TYPE
    //line.clear();
    //line += Rinex_Printer::leftJustify("GROUND_CRAFT", 20); // put a flag or a property
    //line += std::string(40, ' ');
    //line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
    //Rinex_Printer::lengthCheck(line);
    //out << line << std::endl;

    // -------- Line OBSERVER / AGENCY
    line.clear();
    std::string username;
    char c_username[20] = {0};
    int nGet = getlogin_r(c_username, sizeof(c_username) - 1);
    if (nGet == 0)
        {
            username = c_username;
        }
    else
        {
            username = "UNKNOWN USER";
        }
    line += leftJustify(username, 20);
    line += Rinex_Printer::leftJustify("CTTC", 40); // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20); // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20); // add flag and property
    //line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if(gnss_sdr_version.length() > 20) gnss_sdr_version.resize(9, ' ');
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << std::endl;

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);  // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- APPROX POSITION  (optional for moving platforms)
    // put here real data!
    double antena_x = 0.0;
    double antena_y = 0.0;
    double antena_z = 0.0;
    line.clear();
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_x, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_y, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_z, 4), 14);
    line += std::string(18, ' ');
    line += Rinex_Printer::leftJustify("APPROX POSITION XYZ", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- ANTENNA: DELTA H/E/N
    // put here real data!
    double antena_h = 0.0;
    double antena_e = 0.0;
    double antena_n = 0.0;
    line.clear();
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_h, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_e, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_n, 4), 14);
    line += std::string(18, ' ');
    line += Rinex_Printer::leftJustify("ANTENNA: DELTA H/E/N", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- SYS / OBS TYPES
    // one line per available system
    line.clear();
    line += satelliteSystem["GPS"];
    line += std::string(2, ' ');
    std::stringstream strm;
    numberTypesObservations = 4;
    strm << numberTypesObservations;
    line += Rinex_Printer::rightJustify(strm.str(), 3);
    // per type of observation
    // GPS L2 PSEUDORANGE
    line += std::string(1, ' ');
    line += observationType["PSEUDORANGE"];
    line += observationCode["GPS_L2_L2CM"];
    // GPS L2 PHASE
    line += std::string(1, ' ');
    line += observationType["CARRIER_PHASE"];
    line += observationCode["GPS_L2_L2CM"];
    // GPS DOPPLER L2
    line += std::string(1, ' ');
    line += observationType["DOPPLER"];
    line += observationCode["GPS_L2_L2CM"];
    // GPS L2 SIGNAL STRENGTH
    line += std::string(1, ' ');
    line += observationType["SIGNAL_STRENGTH"];
    line += observationCode["GPS_L2_L2CM"];

    line += std::string(60-line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Signal Strength units
    line.clear();
    line += Rinex_Printer::leftJustify("DBHZ", 20);
    line += std::string(40, ' ');
    line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- TIME OF FIRST OBS
    line.clear();
    boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(eph,d_TOW_first_observation);
    std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    std::string year (timestring, 0, 4);
    std::string month (timestring, 4, 2);
    std::string day (timestring, 6, 2);
    std::string hour (timestring, 9, 2);
    std::string minutes (timestring, 11, 2);
    double gps_t = d_TOW_first_observation;
    double seconds = fmod(gps_t, 60);
    line += Rinex_Printer::rightJustify(year, 6);
    line += Rinex_Printer::rightJustify(month, 6);
    line += Rinex_Printer::rightJustify(day, 6);
    line += Rinex_Printer::rightJustify(hour, 6);
    line += Rinex_Printer::rightJustify(minutes, 6);
    line += Rinex_Printer::rightJustify(asString(seconds, 7), 13);
    line += Rinex_Printer::rightJustify(std::string("GPS"), 8);
    line += std::string(9, ' ');
    line += Rinex_Printer::leftJustify("TIME OF FIRST OBS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- SYS /PHASE SHIFTS

    // -------- end of header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;
}


void Rinex_Printer::rinex_obs_header(std::fstream & out, const Gps_Ephemeris & eph, const Gps_CNAV_Ephemeris & eph_cnav, const double d_TOW_first_observation)
{
    if(eph_cnav.d_i_0){} // avoid warning, not needed
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += stringVersion;
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem["GPS"];
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GPS OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See http://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line MARKER NAME
    line.clear();
    line += Rinex_Printer::leftJustify("DEFAULT MARKER NAME", 60); // put a flag or a property,
    line += Rinex_Printer::leftJustify("MARKER NAME", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line MARKER TYPE
    //line.clear();
    //line += Rinex_Printer::leftJustify("GROUND_CRAFT", 20); // put a flag or a property
    //line += std::string(40, ' ');
    //line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
    //Rinex_Printer::lengthCheck(line);
    //out << line << std::endl;

    // -------- Line OBSERVER / AGENCY
    line.clear();
    std::string username;
    char c_username[20] = {0};
    int nGet = getlogin_r(c_username, sizeof(c_username) - 1);
    if (nGet == 0)
        {
            username = c_username;
        }
    else
        {
            username = "UNKNOWN USER";
        }
    line += leftJustify(username, 20);
    line += Rinex_Printer::leftJustify("CTTC", 40); // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20); // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20); // add flag and property
    //line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if(gnss_sdr_version.length() > 20) gnss_sdr_version.resize(9, ' ');
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << std::endl;

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);  // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- APPROX POSITION  (optional for moving platforms)
    // put here real data!
    double antena_x = 0.0;
    double antena_y = 0.0;
    double antena_z = 0.0;
    line.clear();
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_x, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_y, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_z, 4), 14);
    line += std::string(18, ' ');
    line += Rinex_Printer::leftJustify("APPROX POSITION XYZ", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- ANTENNA: DELTA H/E/N
    // put here real data!
    double antena_h = 0.0;
    double antena_e = 0.0;
    double antena_n = 0.0;
    line.clear();
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_h, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_e, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_n, 4), 14);
    line += std::string(18, ' ');
    line += Rinex_Printer::leftJustify("ANTENNA: DELTA H/E/N", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- SYS / OBS TYPES
    // one line per available system
    line.clear();
    line += satelliteSystem["GPS"];
    line += std::string(2, ' ');
    std::stringstream strm;
    numberTypesObservations = 8;
    strm << numberTypesObservations;
    line += Rinex_Printer::rightJustify(strm.str(), 3);
    // per type of observation
    // GPS L1 PSEUDORANGE
    line += std::string(1, ' ');
    line += observationType["PSEUDORANGE"];
    line += observationCode["GPS_L1_CA"];
    // GPS L1 PHASE
    line += std::string(1, ' ');
    line += observationType["CARRIER_PHASE"];
    line += observationCode["GPS_L1_CA"];
    // GPS DOPPLER L1
    line += std::string(1, ' ');
    line += observationType["DOPPLER"];
    line += observationCode["GPS_L1_CA"];
    // GPS L1 CA SIGNAL STRENGTH
    line += std::string(1, ' ');
    line += observationType["SIGNAL_STRENGTH"];
    line += observationCode["GPS_L1_CA"];
    // GPS L2 PSEUDORANGE
    line += std::string(1, ' ');
    line += observationType["PSEUDORANGE"];
    line += observationCode["GPS_L2_L2CM"];
    // GPS L2 PHASE
    line += std::string(1, ' ');
    line += observationType["CARRIER_PHASE"];
    line += observationCode["GPS_L2_L2CM"];
    // GPS DOPPLER L2
    line += std::string(1, ' ');
    line += observationType["DOPPLER"];
    line += observationCode["GPS_L2_L2CM"];
    // GPS L2 SIGNAL STRENGTH
    line += std::string(1, ' ');
    line += observationType["SIGNAL_STRENGTH"];
    line += observationCode["GPS_L2_L2CM"];

    line += std::string(60-line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Signal Strength units
    line.clear();
    line += Rinex_Printer::leftJustify("DBHZ", 20);
    line += std::string(40, ' ');
    line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- TIME OF FIRST OBS
    line.clear();
    boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(eph,d_TOW_first_observation);
    std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    std::string year (timestring, 0, 4);
    std::string month (timestring, 4, 2);
    std::string day (timestring, 6, 2);
    std::string hour (timestring, 9, 2);
    std::string minutes (timestring, 11, 2);
    double gps_t = d_TOW_first_observation;
    double seconds = fmod(gps_t, 60);
    line += Rinex_Printer::rightJustify(year, 6);
    line += Rinex_Printer::rightJustify(month, 6);
    line += Rinex_Printer::rightJustify(day, 6);
    line += Rinex_Printer::rightJustify(hour, 6);
    line += Rinex_Printer::rightJustify(minutes, 6);
    line += Rinex_Printer::rightJustify(asString(seconds, 7), 13);
    line += Rinex_Printer::rightJustify(std::string("GPS"), 8);
    line += std::string(9, ' ');
    line += Rinex_Printer::leftJustify("TIME OF FIRST OBS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- SYS /PHASE SHIFTS

    // -------- end of header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Galileo_Ephemeris& eph, const double d_TOW_first_observation, const std::string bands)
{
    std::string line;
    version = 3;

    // -------- Line 1
    line = std::string(5, ' ');
    line += "3.02";
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem["Galileo"];
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GALILEO OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See http://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line MARKER NAME
    line.clear();
    line += Rinex_Printer::leftJustify("DEFAULT MARKER NAME", 60); // put a flag or a property,
    line += Rinex_Printer::leftJustify("MARKER NAME", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line MARKER TYPE
    //line.clear();
    //line += Rinex_Printer::leftJustify("NON_GEODETIC", 20); // put a flag or a property
    //line += std::string(40, ' ');
    //line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
    //Rinex_Printer::lengthCheck(line);
    //out << line << std::endl;

    // -------- Line OBSERVER / AGENCY
    line.clear();
    std::string username;
    char c_username[20] = {0};
    int nGet = getlogin_r(c_username, sizeof(c_username) - 1);
    if (nGet == 0)
        {
            username = c_username;
        }
    else
        {
            username = "UNKNOWN USER";
        }
    line += leftJustify(username, 20);
    line += Rinex_Printer::leftJustify("CTTC", 40); // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20); // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20); // add flag and property
    //line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if(gnss_sdr_version.length() > 20) gnss_sdr_version.resize(9, ' ');
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << std::endl;

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);  // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- APPROX POSITION  (optional for moving platforms)
    // put here real data!
    double antena_x = 0.0;
    double antena_y = 0.0;
    double antena_z = 0.0;
    line.clear();
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_x, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_y, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_z, 4), 14);
    line += std::string(18, ' ');
    line += Rinex_Printer::leftJustify("APPROX POSITION XYZ", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- ANTENNA: DELTA H/E/N
    // put here real data!
    double antena_h = 0.0;
    double antena_e = 0.0;
    double antena_n = 0.0;
    line.clear();
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_h, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_e, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_n, 4), 14);
    line += std::string(18, ' ');
    line += Rinex_Printer::leftJustify("ANTENNA: DELTA H/E/N", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- SYS / OBS TYPES
    // one line per available system
    unsigned int number_of_observations = 0;
    std::string signal_("1B");
    std::size_t found_1B = bands.find(signal_);
    if(found_1B != std::string::npos)
        {
            number_of_observations = number_of_observations + 4;
        }
    signal_ = "5X";
    std::size_t found_5X = bands.find(signal_);
    if(found_5X != std::string::npos)
        {
            number_of_observations = number_of_observations + 4;
        }

    line.clear();
    signal_ = "7X";
    std::size_t found_7X = bands.find(signal_);
    if(found_7X != std::string::npos)
        {
            number_of_observations = number_of_observations + 4;
        }

    line.clear();

    line += satelliteSystem["Galileo"];
    line += std::string(2, ' ');
    line += Rinex_Printer::rightJustify(std::to_string(number_of_observations), 3);

    if(found_1B != std::string::npos)
        {
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GALILEO_E1_B"];
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GALILEO_E1_B"];
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GALILEO_E1_B"];
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GALILEO_E1_B"];
        }

    if(found_5X != std::string::npos)
        {
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GALILEO_E5a_IQ"];
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GALILEO_E5a_IQ"];
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GALILEO_E5a_IQ"];
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GALILEO_E5a_IQ"];
        }

    if(found_7X != std::string::npos)
        {
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GALILEO_E5b_IQ"];
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GALILEO_E5b_IQ"];
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GALILEO_E5b_IQ"];
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GALILEO_E5b_IQ"];
        }

    line += std::string(60-line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Signal Strength units
    line.clear();
    line += Rinex_Printer::leftJustify("DBHZ", 20);
    line += std::string(40, ' ');
    line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- TIME OF FIRST OBS
    line.clear();
    boost::posix_time::ptime p_galileo_time = Rinex_Printer::compute_Galileo_time(eph, d_TOW_first_observation);
    std::string timestring=boost::posix_time::to_iso_string(p_galileo_time);
    std::string year (timestring, 0, 4);
    std::string month (timestring, 4, 2);
    std::string day (timestring, 6, 2);
    std::string hour (timestring, 9, 2);
    std::string minutes (timestring, 11, 2);
    double galileo_t = d_TOW_first_observation;
    double seconds = fmod(galileo_t, 60);
    line += Rinex_Printer::rightJustify(year, 6);
    line += Rinex_Printer::rightJustify(month, 6);
    line += Rinex_Printer::rightJustify(day, 6);
    line += Rinex_Printer::rightJustify(hour, 6);
    line += Rinex_Printer::rightJustify(minutes, 6);
    line += Rinex_Printer::rightJustify(asString(seconds, 7), 13);
    line += Rinex_Printer::rightJustify(std::string("GAL"), 8);
    line += std::string(9, ' ');
    line += Rinex_Printer::leftJustify("TIME OF FIRST OBS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- SYS /PHASE SHIFTS

    // -------- end of header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Gps_Ephemeris& gps_eph, const Galileo_Ephemeris& galileo_eph, const double d_TOW_first_observation, const std::string galileo_bands)
{
    if(galileo_eph.e_1){} // avoid warning, not needed
    std::string line;
    version = 3;

    // -------- Line 1
    line = std::string(5, ' ');
    line += "3.02";
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem["Mixed"];
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("MIXED (GPS/GALILEO) OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See http://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line MARKER NAME
    line.clear();
    line += Rinex_Printer::leftJustify("DEFAULT MARKER NAME", 60); // put a flag or a property,
    line += Rinex_Printer::leftJustify("MARKER NAME", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line MARKER TYPE
    //line.clear();
    //line += Rinex_Printer::leftJustify("NON_GEODETIC", 20); // put a flag or a property
    //line += std::string(40, ' ');
    //line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
    //Rinex_Printer::lengthCheck(line);
    //out << line << std::endl;

    // -------- Line OBSERVER / AGENCY
    line.clear();
    std::string username;
    char c_username[20] = {0};
    int nGet = getlogin_r(c_username, sizeof(c_username) - 1);
    if (nGet == 0)
        {
            username = c_username;
        }
    else
        {
            username = "UNKNOWN USER";
        }
    line += leftJustify(username, 20);
    line += Rinex_Printer::leftJustify("CTTC", 40); // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20); // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20); // add flag and property
    //line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if(gnss_sdr_version.length() > 20) gnss_sdr_version.resize(9, ' ');
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << std::endl;

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);  // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- APPROX POSITION  (optional for moving platforms)
    // put here real data!
    double antena_x = 0.0;
    double antena_y = 0.0;
    double antena_z = 0.0;
    line.clear();
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_x, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_y, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_z, 4), 14);
    line += std::string(18, ' ');
    line += Rinex_Printer::leftJustify("APPROX POSITION XYZ", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- ANTENNA: DELTA H/E/N
    // put here real data!
    double antena_h = 0.0;
    double antena_e = 0.0;
    double antena_n = 0.0;
    line.clear();
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_h, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_e, 4), 14);
    line += Rinex_Printer::rightJustify(Rinex_Printer::asString(antena_n, 4), 14);
    line += std::string(18, ' ');
    line += Rinex_Printer::leftJustify("ANTENNA: DELTA H/E/N", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- SYS / OBS TYPES
    // one line per available system
    line.clear();
    line += satelliteSystem["GPS"];
    line += std::string(2, ' ');
    std::stringstream strm;
    numberTypesObservations = 4;
    strm << numberTypesObservations;
    line += Rinex_Printer::rightJustify(strm.str(), 3);
    // per type of observation
    // GPS L1 PSEUDORANGE
    line += std::string(1, ' ');
    line += observationType["PSEUDORANGE"];
    line += observationCode["GPS_L1_CA"];
    // GPS L1 PHASE
    line += std::string(1, ' ');
    line += observationType["CARRIER_PHASE"];
    line += observationCode["GPS_L1_CA"];
    // GPS DOPPLER L1
    line += std::string(1, ' ');
    line += observationType["DOPPLER"];
    line += observationCode["GPS_L1_CA"];
    // GPS L1 CA SIGNAL STRENGTH
    line += std::string(1, ' ');
    line += observationType["SIGNAL_STRENGTH"];
    line += observationCode["GPS_L1_CA"];
    line += std::string(60-line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    line.clear();
    unsigned int number_of_observations_gal = 0;
    std::string signal_("1B");
    std::size_t found_1B = galileo_bands.find(signal_);
    if(found_1B != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }
    signal_ = "5X";
    std::size_t found_5X = galileo_bands.find(signal_);
    if(found_5X != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }

    line.clear();
    signal_ = "7X";
    std::size_t found_7X = galileo_bands.find(signal_);
    if(found_7X != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }

    line += satelliteSystem["Galileo"];
    line += std::string(2, ' ');
    line += Rinex_Printer::rightJustify(std::to_string(number_of_observations_gal), 3);

    if(found_1B != std::string::npos)
        {
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GALILEO_E1_B"];
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GALILEO_E1_B"];
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GALILEO_E1_B"];
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GALILEO_E1_B"];
        }

    if(found_5X != std::string::npos)
        {
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GALILEO_E5a_IQ"];
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GALILEO_E5a_IQ"];
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GALILEO_E5a_IQ"];
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GALILEO_E5a_IQ"];
        }

    if(found_7X != std::string::npos)
        {
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GALILEO_E5b_IQ"];
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GALILEO_E5b_IQ"];
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GALILEO_E5b_IQ"];
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GALILEO_E5b_IQ"];
        }

    line += std::string(60-line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- Signal Strength units
    line.clear();
    line += Rinex_Printer::leftJustify("DBHZ", 20);
    line += std::string(40, ' ');
    line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- TIME OF FIRST OBS
    line.clear();
    boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(gps_eph, d_TOW_first_observation);
    std::string timestring=boost::posix_time::to_iso_string(p_gps_time);
    std::string year (timestring, 0, 4);
    std::string month (timestring, 4, 2);
    std::string day (timestring, 6, 2);
    std::string hour (timestring, 9, 2);
    std::string minutes (timestring, 11, 2);
    double gps_t = d_TOW_first_observation;
    double seconds = fmod(gps_t, 60);
    line += Rinex_Printer::rightJustify(year, 6);
    line += Rinex_Printer::rightJustify(month, 6);
    line += Rinex_Printer::rightJustify(day, 6);
    line += Rinex_Printer::rightJustify(hour, 6);
    line += Rinex_Printer::rightJustify(minutes, 6);
    line += Rinex_Printer::rightJustify(asString(seconds, 7), 13);
    line += Rinex_Printer::rightJustify(std::string("GPS"), 8);
    line += std::string(9, ' ');
    line += Rinex_Printer::leftJustify("TIME OF FIRST OBS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- end of header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;
}


void Rinex_Printer::update_obs_header(std::fstream& out __attribute__((unused)), const Glonass_Gnav_Utc_Model& utc_model)
{
    if(utc_model.d_N_4)
        {
            // do nothing
        }
}


void Rinex_Printer::update_obs_header(std::fstream& out, const Gps_Utc_Model& utc_model)
{
    std::vector<std::string> data;
    std::string line_aux;

    out.seekp(0);
    data.clear();

    bool no_more_finds = false;
    std::string line_str;

    while(!out.eof())
        {
            std::getline(out, line_str);

            if(!no_more_finds)
                {
                    line_aux.clear();

                    if (version == 2)
                        {
                            if (line_str.find("TIME OF FIRST OBS", 59) != std::string::npos) // TIME OF FIRST OBS last header annotation might change in the future
                                {
                                    data.push_back(line_str);
                                    line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.d_DeltaT_LS), 6);
                                    line_aux += std::string(54, ' ');
                                    line_aux += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
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

                    if (version == 3)
                        {
                            if (line_str.find("TIME OF FIRST OBS", 59) != std::string::npos)
                                {
                                    data.push_back(line_str);
                                    line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.d_DeltaT_LS), 6);
                                    line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.d_DeltaT_LSF), 6);
                                    line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.i_WN_LSF), 6);
                                    line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.i_DN), 6);
                                    line_aux += std::string(36, ' ');
                                    line_aux += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
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
                }
            else
                {
                    data.push_back(line_str);
                }
        }

    out.close();
    out.open(obsfilename, std::ios::out | std::ios::trunc);
    out.seekp(0);
    for (int i = 0; i < static_cast<int>( data.size()) - 1; i++)
        {
            out << data[i] << std::endl;
        }
    out.close();
    out.open(obsfilename, std::ios::out | std::ios::in | std::ios::app);
    out.seekp(0, std::ios_base::end);
}


void Rinex_Printer::update_obs_header(std::fstream& out, const Gps_CNAV_Utc_Model& utc_model)
{
    std::vector<std::string> data;
    std::string line_aux;

    out.seekp(0);
    data.clear();

    bool no_more_finds = false;
    std::string line_str;

    while(!out.eof())
        {
            std::getline(out, line_str);

            if(!no_more_finds)
                {
                    line_aux.clear();
                    if (line_str.find("TIME OF FIRST OBS", 59) != std::string::npos)
                        {
                            data.push_back(line_str);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.d_DeltaT_LS), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.d_DeltaT_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.i_WN_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(utc_model.i_DN), 6);
                            line_aux += std::string(36, ' ');
                            line_aux += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
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
            else
                {
                    data.push_back(line_str);
                }
        }

    out.close();
    out.open(obsfilename, std::ios::out | std::ios::trunc);
    out.seekp(0);
    for (int i = 0; i < static_cast<int>(data.size()) - 1; i++)
        {
            out << data[i] << std::endl;
        }
    out.close();
    out.open(obsfilename, std::ios::out | std::ios::in | std::ios::app);
    out.seekp(0, std::ios_base::end);
}


void Rinex_Printer::update_obs_header(std::fstream& out, const Galileo_Utc_Model& galileo_utc_model)
{
    std::vector<std::string> data;
    std::string line_aux;

    out.seekp(0);
    data.clear();

    bool no_more_finds = false;
    std::string line_str;

    while(!out.eof())
        {
            std::getline(out, line_str);

            if(!no_more_finds)
                {
                    line_aux.clear();

                    if (line_str.find("TIME OF FIRST OBS", 59) != std::string::npos)
                        {
                            data.push_back(line_str);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_utc_model.Delta_tLS_6), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_utc_model.Delta_tLSF_6), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_utc_model.WN_LSF_6), 6);
                            line_aux += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(galileo_utc_model.DN_6), 6);
                            line_aux += std::string(36, ' ');
                            line_aux += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
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
            else
                {
                    data.push_back(line_str);
                }
        }

    out.close();
    out.open(obsfilename, std::ios::out | std::ios::trunc);
    out.seekp(0);
    for (int i = 0; i < static_cast<int>(data.size()) - 1; i++)
        {
            out << data[i] << std::endl;
        }
    out.close();
    out.open(obsfilename, std::ios::out | std::ios::in |std::ios::app);
    out.seekp(0, std::ios_base::end);
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Glonass_Gnav_Ephemeris& eph, const double obs_time, const std::map<int,Gnss_Synchro>& observables, const std::string glonass_band)
{
    // RINEX observations timestamps are GPS timestamps.
    std::string line;

    // Avoid compiler warning
    if(glonass_band.size()){}

    boost::posix_time::ptime p_glonass_time = Rinex_Printer::compute_UTC_time(eph, obs_time);
    std::string timestring = boost::posix_time::to_iso_string(p_glonass_time);
    //double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    //double gps_t = eph.sv_clock_correction(obs_time);
    double glonass_t = obs_time;

    std::string month (timestring, 4, 2);
    std::string day (timestring, 6, 2);
    std::string hour (timestring, 9, 2);
    std::string minutes (timestring, 11, 2);

    if (version == 2)
        {
            line.clear();
            std::string year (timestring, 2, 2);
            line += std::string(1, ' ');
            line += year;
            line += std::string(1, ' ');
            if (month.compare(0, 1 , "0") == 0)
                {
                    line += std::string(1, ' ');
                    line += month.substr(1, 1);
                }
            else
                {
                    line += month;
                }
            line += std::string(1, ' ');
            if (day.compare(0, 1 , "0") == 0)
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
            double second_ = fmod(glonass_t, 60);
            if (second_ < 10)
                {
                    line += std::string(1, ' ');
                }
            line += Rinex_Printer::asString(second_, 7);
            line += std::string(2, ' ');
            // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
            line += std::string(1, '0');
            //Number of satellites observed in current epoch
            int numSatellitesObserved = 0;
            std::map<int, Gnss_Synchro>::const_iterator observables_iter;
            for(observables_iter = observables.begin();
                    observables_iter != observables.end();
                    observables_iter++)
                {
                    numSatellitesObserved++;
                }
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(numSatellitesObserved), 3);
            for(observables_iter = observables.begin();
                    observables_iter != observables.end();
                    observables_iter++)
                {
                    line += satelliteSystem["GLONASS"];
                    if (static_cast<int>(observables_iter->second.PRN) < 10) line += std::string(1, '0');
                    line += boost::lexical_cast<std::string>(static_cast<int>(observables_iter->second.PRN));
                }
            // Receiver clock offset (optional)
            //line += rightJustify(asString(clockOffset, 12), 15);
            line += std::string(80 - line.size(), ' ');
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            for(observables_iter = observables.begin();
                    observables_iter != observables.end();
                    observables_iter++)
                {
                    std::string lineObs;
                    lineObs.clear();
                    line.clear();
                    // GLONASS L1 PSEUDORANGE
                    line += std::string(2, ' ');
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Pseudorange_m, 3), 14);

                    //Loss of lock indicator (LLI)
                    int lli = 0; // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    int ssi = Rinex_Printer::signalStrength(observables_iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);
                    // GLONASS L1 CA PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_phase_rads/GLONASS_TWO_PI, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);
                    // GLONASS L1 CA DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);
                    //GLONASS L1 SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.CN0_dB_hz, 3), 14);
                    if (lineObs.size() < 80) lineObs += std::string(80 - lineObs.size(), ' ');
                    out << lineObs << std::endl;
                }
        }

    if (version == 3)
        {
            std::string year (timestring, 0, 4);
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
            double seconds = fmod(glonass_t, 60);
            // Add extra 0 if seconds are < 10
            if (seconds < 10)
                {
                    line += std::string(1, '0');
                }
            line += Rinex_Printer::asString(seconds, 7);
            line += std::string(2, ' ');
            // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
            line += std::string(1, '0');

            //Number of satellites observed in current epoch
            int numSatellitesObserved = 0;
            std::map<int, Gnss_Synchro>::const_iterator observables_iter;
            for(observables_iter = observables.begin();
                    observables_iter != observables.end();
                    observables_iter++)
                {
                    numSatellitesObserved++;
                }
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(numSatellitesObserved), 3);

            // Receiver clock offset (optional)
            //line += rightJustify(asString(clockOffset, 12), 15);

            line += std::string(80 - line.size(), ' ');
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            for(observables_iter = observables.begin();
                    observables_iter != observables.end();
                    observables_iter++)
                {
                    std::string lineObs;
                    lineObs.clear();
                    lineObs += satelliteSystem["GLONASS"];
                    if (static_cast<int>(observables_iter->second.PRN) < 10) lineObs += std::string(1, '0');
                    lineObs += boost::lexical_cast<std::string>(static_cast<int>(observables_iter->second.PRN));
                    //lineObs += std::string(2, ' ');
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Pseudorange_m, 3), 14);

                    //Loss of lock indicator (LLI)
                    int lli = 0; // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    int ssi = Rinex_Printer::signalStrength(observables_iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // GLONASS L1 CA PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_phase_rads/GLONASS_TWO_PI, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // GLONASS L1 CA DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }

                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    //GLONASS L1 SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.CN0_dB_hz, 3), 14);

                    if (lineObs.size() < 80) lineObs += std::string(80 - lineObs.size(), ' ');
                    out << lineObs << std::endl;
                }
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Gps_Ephemeris& gps_eph, const Glonass_Gnav_Ephemeris& glonass_gnav_eph,  double gps_obs_time, const std::map<int,Gnss_Synchro>& observables)
{
    if(glonass_gnav_eph.d_m){} // avoid warning, not needed
    std::string line;

    // -------- EPOCH record
    boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(gps_eph, gps_obs_time);
    std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    //double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    //double gps_t = eph.sv_clock_correction(obs_time);
    double gps_t = gps_obs_time;

    std::string month (timestring, 4, 2);
    std::string day (timestring, 6, 2);
    std::string hour (timestring, 9, 2);
    std::string minutes (timestring, 11, 2);

    if (version == 2)
        {
            line.clear();
            std::string year (timestring, 2, 2);
            line += std::string(1, ' ');
            line += year;
            line += std::string(1, ' ');
            if (month.compare(0, 1 , "0") == 0)
                {
                    line += std::string(1, ' ');
                    line += month.substr(1, 1);
                }
            else
                {
                    line += month;
                }
            line += std::string(1, ' ');
            if (day.compare(0, 1 , "0") == 0)
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
            double second_ = fmod(gps_t, 60);
            if (second_ < 10)
                {
                    line += std::string(1, ' ');
                }
            line += Rinex_Printer::asString(second_, 7);
            line += std::string(2, ' ');
            // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
            line += std::string(1, '0');
        }
    if (version == 3)
        {
            std::string year (timestring, 0, 4);
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
            double seconds = fmod(gps_t, 60);
            // Add extra 0 if seconds are < 10
            if (seconds < 10)
                {
                    line +=std::string(1, '0');
                }
            line += Rinex_Printer::asString(seconds, 7);
            line += std::string(2, ' ');
            // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
            line += std::string(1, '0');
        }

    //Number of satellites observed in current epoch
    //Get maps with observations
    std::map<int, Gnss_Synchro> observablesG1C;
    std::map<int, Gnss_Synchro> observablesR1C;
    std::map<int, Gnss_Synchro> observablesR2C;
    std::map<int, Gnss_Synchro>::const_iterator observables_iter;

    for(observables_iter = observables.begin();
            observables_iter != observables.end();
            observables_iter++)
        {
            std::string system_(&observables_iter->second.System, 1);
            std::string sig_(observables_iter->second.Signal);
            if((system_.compare("R") == 0) && (sig_.compare("1G") == 0))
                {
                    observablesR1C.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if((system_.compare("R") == 0) && (sig_.compare("2G") == 0))
                {
                    observablesR2C.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if((system_.compare("G") == 0) && (sig_.compare("1C") == 0))
                {
                    observablesG1C.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
        }

    std::multimap<unsigned int, Gnss_Synchro> total_glo_map;
    std::set<unsigned int> available_glo_prns;
    std::set<unsigned int>::iterator it;
    for(observables_iter = observablesR1C.begin();
            observables_iter != observablesR1C.end();
            observables_iter++)
        {
            unsigned int prn_ = observables_iter->second.PRN;
            total_glo_map.insert(std::pair<unsigned int, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_glo_prns.find(prn_);
            if(it == available_glo_prns.end())
                {
                    available_glo_prns.insert(prn_);
                }
        }

    for(observables_iter = observablesR2C.begin();
            observables_iter != observablesR2C.end();
            observables_iter++)
        {
            unsigned int prn_ = observables_iter->second.PRN;
            total_glo_map.insert(std::pair<unsigned int, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_glo_prns.find(prn_);
            if(it == available_glo_prns.end())
                {
                    available_glo_prns.insert(prn_);
                }
        }

    int numGloSatellitesObserved = available_glo_prns.size();
    int numGpsSatellitesObserved = observablesG1C.size();
    int numSatellitesObserved = numGloSatellitesObserved + numGpsSatellitesObserved;
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(numSatellitesObserved), 3);
    if(version == 2)
        {
            // Add list of GPS satellites
            for(observables_iter = observablesG1C.begin();
                    observables_iter != observablesG1C.end();
                    observables_iter++)
                {
                    line += satelliteSystem["GPS"];
                    if (static_cast<int>(observables_iter->second.PRN) < 10) line += std::string(1, '0');
                    line += boost::lexical_cast<std::string>(static_cast<int>(observables_iter->second.PRN));
                }
            // Add list of GLONASS L1 satellites
            for(observables_iter = observablesR1C.begin();
                    observables_iter != observablesR1C.end();
                    observables_iter++)
                {
                    line += satelliteSystem["GLONASS"];
                    if (static_cast<int>(observables_iter->second.PRN) < 10) line += std::string(1, '0');
                    line += boost::lexical_cast<std::string>(static_cast<int>(observables_iter->second.PRN));
                }
            // Add list of GLONASS L2 satellites
            for(observables_iter = observablesR2C.begin();
                    observables_iter != observablesR2C.end();
                    observables_iter++)
                {
                    line += satelliteSystem["GLONASS"];
                    if (static_cast<int>(observables_iter->second.PRN) < 10) line += std::string(1, '0');
                    line += boost::lexical_cast<std::string>(static_cast<int>(observables_iter->second.PRN));
                }
        }
    line += std::string(80 - line.size(), ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- OBSERVATION record
    std::string s;
    std::string lineObs;
    for(observables_iter = observablesG1C.begin();
            observables_iter != observablesG1C.end();
            observables_iter++)
        {
            lineObs.clear();

            s.assign(1, observables_iter->second.System);
            if(version == 3)
                {
                    // Specify system only if in version 3
                    if(s.compare("G") == 0) lineObs += satelliteSystem["GPS"];
                    if(s.compare("R") == 0) lineObs += satelliteSystem["GLONASS"]; // should not happen
                    if (static_cast<int>(observables_iter->second.PRN) < 10) lineObs += std::string(1, '0');
                    lineObs += boost::lexical_cast<std::string>(static_cast<int>(observables_iter->second.PRN));
                }

            // Pseudorange Measurements
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Pseudorange_m, 3), 14);

            //Loss of lock indicator (LLI)
            int lli = 0; // Include in the observation!!
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            //else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
            //    }

            // Signal Strength Indicator (SSI)
            int ssi = Rinex_Printer::signalStrength(observables_iter->second.CN0_dB_hz);
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

            // PHASE
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_phase_rads/GPS_TWO_PI, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            //else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
            //    }
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

            // DOPPLER
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_Doppler_hz, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            //else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
            //    }
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

            // SIGNAL STRENGTH
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.CN0_dB_hz, 3), 14);

            if (lineObs.size() < 80) lineObs += std::string(80 - lineObs.size(), ' ');
            out << lineObs << std::endl;
        }

    std::pair <std::multimap<unsigned int, Gnss_Synchro>::iterator, std::multimap<unsigned int, Gnss_Synchro>::iterator> ret;
    for(it = available_glo_prns.begin();
            it != available_glo_prns.end();
            it++)
        {
            lineObs.clear();
            if(version == 3)
                {
                    lineObs += satelliteSystem["GLONASS"];
                    if (static_cast<int>(*it) < 10) lineObs += std::string(1, '0');
                    lineObs += boost::lexical_cast<std::string>(static_cast<int>(*it));
                }
            ret = total_glo_map.equal_range(*it);
            for (std::multimap<unsigned int, Gnss_Synchro>::iterator iter = ret.first; iter != ret.second; ++iter)
                {
                    /// \todo Need to account for pseudorange correction for glonass
                    //double leap_seconds = Rinex_Printer::get_leap_second(glonass_gnav_eph, gps_obs_time);
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    //Loss of lock indicator (LLI)
                    int lli = 0; // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    int ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // GLONASS CARRIER PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_phase_rads / (GLONASS_TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // GLONASS  DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // GLONASS SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            if (lineObs.size() < 80) lineObs += std::string(80 - lineObs.size(), ' ');
            out << lineObs << std::endl;
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Gps_CNAV_Ephemeris& gps_eph, const Glonass_Gnav_Ephemeris& glonass_gnav_eph,  double gps_obs_time, const std::map<int,Gnss_Synchro>& observables)
{
    if(glonass_gnav_eph.d_m){} // avoid warning, not needed
    std::string line;

    // -------- EPOCH record
    boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(gps_eph, gps_obs_time);
    std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    //double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    //double gps_t = eph.sv_clock_correction(obs_time);
    double gps_t = gps_obs_time;

    std::string month (timestring, 4, 2);
    std::string day (timestring, 6, 2);
    std::string hour (timestring, 9, 2);
    std::string minutes (timestring, 11, 2);

    std::string year (timestring, 0, 4);
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
    double seconds = fmod(gps_t, 60);
    // Add extra 0 if seconds are < 10
    if (seconds < 10)
        {
            line +=std::string(1, '0');
        }
    line += Rinex_Printer::asString(seconds, 7);
    line += std::string(2, ' ');
    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    //Number of satellites observed in current epoch
    //Get maps with observations
    std::map<int, Gnss_Synchro> observablesG2S;
    std::map<int, Gnss_Synchro> observablesR1C;
    std::map<int, Gnss_Synchro> observablesR2C;
    std::map<int, Gnss_Synchro>::const_iterator observables_iter;

    for(observables_iter = observables.begin();
            observables_iter != observables.end();
            observables_iter++)
        {
            std::string system_(&observables_iter->second.System, 1);
            std::string sig_(observables_iter->second.Signal);
            if((system_.compare("R") == 0) && (sig_.compare("1G") == 0))
                {
                    observablesR1C.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if((system_.compare("R") == 0) && (sig_.compare("2G") == 0))
                {
                    observablesR2C.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if((system_.compare("G") == 0) && (sig_.compare("2S") == 0))
                {
                    observablesG2S.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
        }

    std::multimap<unsigned int, Gnss_Synchro> total_glo_map;
    std::set<unsigned int> available_glo_prns;
    std::set<unsigned int>::iterator it;
    for(observables_iter = observablesR1C.begin();
            observables_iter != observablesR1C.end();
            observables_iter++)
        {
            unsigned int prn_ = observables_iter->second.PRN;
            total_glo_map.insert(std::pair<unsigned int, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_glo_prns.find(prn_);
            if(it == available_glo_prns.end())
                {
                    available_glo_prns.insert(prn_);
                }
        }

    for(observables_iter = observablesR2C.begin();
            observables_iter != observablesR2C.end();
            observables_iter++)
        {
            unsigned int prn_ = observables_iter->second.PRN;
            total_glo_map.insert(std::pair<unsigned int, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_glo_prns.find(prn_);
            if(it == available_glo_prns.end())
                {
                    available_glo_prns.insert(prn_);
                }
        }

    int numGloSatellitesObserved = available_glo_prns.size();
    int numGpsSatellitesObserved = observablesG2S.size();
    int numSatellitesObserved = numGloSatellitesObserved + numGpsSatellitesObserved;
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(numSatellitesObserved), 3);

    line += std::string(80 - line.size(), ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    // -------- OBSERVATION record
    std::string s;
    std::string lineObs;
    for(observables_iter = observablesG2S.begin();
            observables_iter != observablesG2S.end();
            observables_iter++)
        {
            lineObs.clear();

            s.assign(1, observables_iter->second.System);
            // Specify system only if in version 3
            if(s.compare("G") == 0) lineObs += satelliteSystem["GPS"];
            if(s.compare("R") == 0) lineObs += satelliteSystem["GLONASS"]; // should not happen
            if (static_cast<int>(observables_iter->second.PRN) < 10) lineObs += std::string(1, '0');
            lineObs += boost::lexical_cast<std::string>(static_cast<int>(observables_iter->second.PRN));

            // Pseudorange Measurements
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Pseudorange_m, 3), 14);

            //Loss of lock indicator (LLI)
            int lli = 0; // Include in the observation!!
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            //else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
            //    }

            // Signal Strength Indicator (SSI)
            int ssi = Rinex_Printer::signalStrength(observables_iter->second.CN0_dB_hz);
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

            // PHASE
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_phase_rads/GPS_TWO_PI, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            //else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
            //    }
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

            // DOPPLER
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_Doppler_hz, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            //else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
            //    }
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

            // SIGNAL STRENGTH
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.CN0_dB_hz, 3), 14);

            if (lineObs.size() < 80) lineObs += std::string(80 - lineObs.size(), ' ');
            out << lineObs << std::endl;
        }

    std::pair <std::multimap<unsigned int, Gnss_Synchro>::iterator, std::multimap<unsigned int, Gnss_Synchro>::iterator> ret;
    for(it = available_glo_prns.begin();
            it != available_glo_prns.end();
            it++)
        {
            lineObs.clear();
            lineObs += satelliteSystem["GLONASS"];
            if (static_cast<int>(*it) < 10) lineObs += std::string(1, '0');
            lineObs += boost::lexical_cast<std::string>(static_cast<int>(*it));

            ret = total_glo_map.equal_range(*it);
            for (std::multimap<unsigned int, Gnss_Synchro>::iterator iter = ret.first; iter != ret.second; ++iter)
                {
                    /// \todo Need to account for pseudorange correction for glonass
                    //double leap_seconds = Rinex_Printer::get_leap_second(glonass_gnav_eph, gps_obs_time);
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    //Loss of lock indicator (LLI)
                    int lli = 0; // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    int ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // GLONASS CARRIER PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_phase_rads / (GLONASS_TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // GLONASS  DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // GLONASS SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            if (lineObs.size() < 80) lineObs += std::string(80 - lineObs.size(), ' ');
            out << lineObs << std::endl;
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Galileo_Ephemeris& galileo_eph, const Glonass_Gnav_Ephemeris& glonass_gnav_eph, double galileo_obs_time, const std::map<int,Gnss_Synchro>& observables)
{
    if(glonass_gnav_eph.d_m){} // avoid warning, not needed
    std::string line;

    boost::posix_time::ptime p_galileo_time = Rinex_Printer::compute_Galileo_time(galileo_eph, galileo_obs_time);
    std::string timestring = boost::posix_time::to_iso_string(p_galileo_time);
    //double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    //double gps_t = eph.sv_clock_correction(obs_time);
    double galileo_t = galileo_obs_time;

    std::string month (timestring, 4, 2);
    std::string day (timestring, 6, 2);
    std::string hour (timestring, 9, 2);
    std::string minutes (timestring, 11, 2);

    std::string year (timestring, 0, 4);
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
    double seconds = fmod(galileo_t, 60);
    // Add extra 0 if seconds are < 10
    if (seconds < 10)
        {
            line +=std::string(1, '0');
        }
    line += Rinex_Printer::asString(seconds, 7);
    line += std::string(2, ' ');
    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    //Number of satellites observed in current epoch

    //Get maps with observations
    std::map<int, Gnss_Synchro> observablesE1B;
    std::map<int, Gnss_Synchro> observablesR1C;
    std::map<int, Gnss_Synchro> observablesR2C;
    std::map<int, Gnss_Synchro>::const_iterator observables_iter;

    for(observables_iter = observables.begin();
            observables_iter != observables.end();
            observables_iter++)
        {
            std::string system_(&observables_iter->second.System, 1);
            std::string sig_(observables_iter->second.Signal);
            if((system_.compare("R") == 0) && (sig_.compare("1G") == 0))
                {
                    observablesR1C.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if((system_.compare("R") == 0) && (sig_.compare("2G") == 0))
                {
                    observablesR2C.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if((system_.compare("E") == 0) && (sig_.compare("1B") == 0))
                {
                    observablesE1B.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
        }

    std::multimap<unsigned int, Gnss_Synchro> total_glo_map;
    std::set<unsigned int> available_glo_prns;
    std::set<unsigned int>::iterator it;
    for(observables_iter = observablesR1C.begin();
            observables_iter != observablesR1C.end();
            observables_iter++)
        {
            unsigned int prn_ = observables_iter->second.PRN;
            total_glo_map.insert(std::pair<unsigned int, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_glo_prns.find(prn_);
            if(it == available_glo_prns.end())
                {
                    available_glo_prns.insert(prn_);
                }
        }
    for(observables_iter = observablesR2C.begin();
            observables_iter != observablesR2C.end();
            observables_iter++)
        {
            unsigned int prn_ = observables_iter->second.PRN;
            total_glo_map.insert(std::pair<unsigned int, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_glo_prns.find(prn_);
            if(it == available_glo_prns.end())
                {
                    available_glo_prns.insert(prn_);
                }
        }

    int numGloSatellitesObserved = available_glo_prns.size();
    int numGalSatellitesObserved = observablesE1B.size();
    int numSatellitesObserved = numGalSatellitesObserved + numGloSatellitesObserved;
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(numSatellitesObserved), 3);

    // Receiver clock offset (optional)
    //line += rightJustify(asString(clockOffset, 12), 15);

    line += std::string(80 - line.size(), ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    std::string s;
    std::string lineObs;
    for(observables_iter = observablesE1B.begin();
            observables_iter != observablesE1B.end();
            observables_iter++)
        {
            lineObs.clear();

            s.assign(1, observables_iter->second.System);
            if(s.compare("E") == 0) lineObs += satelliteSystem["Galileo"];
            if(s.compare("R") == 0) lineObs += satelliteSystem["GLONASS"]; // should not happen
            if (static_cast<int>(observables_iter->second.PRN) < 10) lineObs += std::string(1, '0');
            lineObs += boost::lexical_cast<std::string>(static_cast<int>(observables_iter->second.PRN));
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Pseudorange_m, 3), 14);

            //Loss of lock indicator (LLI)
            int lli = 0; // Include in the observation!!
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            //else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
            //    }

            // Signal Strength Indicator (SSI)
            int ssi = Rinex_Printer::signalStrength(observables_iter->second.CN0_dB_hz);
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

            // PHASE
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_phase_rads/GPS_TWO_PI, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            //else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
            //    }
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

            // DOPPLER
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_Doppler_hz, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            //else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
            //    }
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

            // SIGNAL STRENGTH
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.CN0_dB_hz, 3), 14);

            if (lineObs.size() < 80) lineObs += std::string(80 - lineObs.size(), ' ');
            out << lineObs << std::endl;
        }

    std::pair <std::multimap<unsigned int, Gnss_Synchro>::iterator, std::multimap<unsigned int, Gnss_Synchro>::iterator> ret;
    for(it = available_glo_prns.begin();
            it != available_glo_prns.end();
            it++)
        {
            lineObs.clear();
            lineObs += satelliteSystem["Galileo"];
            if (static_cast<int>(*it) < 10) lineObs += std::string(1, '0');
            lineObs += boost::lexical_cast<std::string>(static_cast<int>(*it));
            ret = total_glo_map.equal_range(*it);
            for (std::multimap<unsigned int, Gnss_Synchro>::iterator iter = ret.first; iter != ret.second; ++iter)
                {
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    //Loss of lock indicator (LLI)
                    int lli = 0; // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    int ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // GLONASS CARRIER PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_phase_rads / (GLONASS_TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // GLONASS  DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //   }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // GLONASS SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            if (lineObs.size() < 80) lineObs += std::string(80 - lineObs.size(), ' ');
            out << lineObs << std::endl;
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Gps_Ephemeris& eph, const double obs_time, const std::map<int,Gnss_Synchro>& observables)
{
    // RINEX observations timestamps are GPS timestamps.
    std::string line;

    boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(eph, obs_time);
    std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    //double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    //double gps_t = eph.sv_clock_correction(obs_time);
    double gps_t = obs_time;

    std::string month (timestring, 4, 2);
    std::string day (timestring, 6, 2);
    std::string hour (timestring, 9, 2);
    std::string minutes (timestring, 11, 2);

    if (version == 2)
        {
            line.clear();
            std::string year (timestring, 2, 2);
            line += std::string(1, ' ');
            line += year;
            line += std::string(1, ' ');
            if (month.compare(0, 1 , "0") == 0)
                {
                    line += std::string(1, ' ');
                    line += month.substr(1, 1);
                }
            else
                {
                    line += month;
                }
            line += std::string(1, ' ');
            if (day.compare(0, 1 , "0") == 0)
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
            double second_ = fmod(gps_t, 60);
            if (second_ < 10)
                {
                    line += std::string(1, ' ');
                }
            line += Rinex_Printer::asString(second_, 7);
            line += std::string(2, ' ');
            // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
            line += std::string(1, '0');
            //Number of satellites observed in current epoch
            int numSatellitesObserved = 0;
            std::map<int, Gnss_Synchro>::const_iterator observables_iter;
            for(observables_iter = observables.cbegin();
                    observables_iter != observables.cend();
                    observables_iter++)
                {
                    numSatellitesObserved++;
                }
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(numSatellitesObserved), 3);
            for(observables_iter = observables.cbegin();
                    observables_iter != observables.cend();
                    observables_iter++)
                {
                    line += satelliteSystem["GPS"];
                    if (static_cast<int>(observables_iter->second.PRN) < 10) line += std::string(1, '0');
                    line += boost::lexical_cast<std::string>(static_cast<int>(observables_iter->second.PRN));
                }
            // Receiver clock offset (optional)
            //line += rightJustify(asString(clockOffset, 12), 15);
            line += std::string(80 - line.size(), ' ');
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            for(observables_iter = observables.cbegin();
                    observables_iter != observables.cend();
                    observables_iter++)
                {
                    std::string lineObs;
                    lineObs.clear();
                    line.clear();
                    // GPS L1 PSEUDORANGE
                    line += std::string(2, ' ');
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Pseudorange_m, 3), 14);

                    //Loss of lock indicator (LLI)
                    int lli = 0; // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    int ssi = Rinex_Printer::signalStrength(observables_iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);
                    // GPS L1 CA PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_phase_rads/GPS_TWO_PI, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);
                    // GPS L1 CA DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //       lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //   }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);
                    //GPS L1 SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.CN0_dB_hz, 3), 14);
                    if (lineObs.size() < 80) lineObs += std::string(80 - lineObs.size(), ' ');
                    out << lineObs << std::endl;
                }
        }

    if (version == 3)
        {
            std::string year (timestring, 0, 4);
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
            double seconds = fmod(gps_t, 60);
            // Add extra 0 if seconds are < 10
            if (seconds < 10)
                {
                    line += std::string(1, '0');
                }
            line += Rinex_Printer::asString(seconds, 7);
            line += std::string(2, ' ');
            // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
            line += std::string(1, '0');

            //Number of satellites observed in current epoch
            int numSatellitesObserved = 0;
            std::map<int, Gnss_Synchro>::const_iterator observables_iter;
            for(observables_iter = observables.cbegin();
                    observables_iter != observables.cend();
                    observables_iter++)
                {
                    numSatellitesObserved++;
                }
            line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(numSatellitesObserved), 3);

            // Receiver clock offset (optional)
            //line += rightJustify(asString(clockOffset, 12), 15);

            line += std::string(80 - line.size(), ' ');
            Rinex_Printer::lengthCheck(line);
            out << line << std::endl;

            for(observables_iter = observables.cbegin();
                    observables_iter != observables.cend();
                    observables_iter++)
                {
                    std::string lineObs;
                    lineObs.clear();
                    lineObs += satelliteSystem["GPS"];
                    if (static_cast<int>(observables_iter->second.PRN) < 10) lineObs += std::string(1, '0');
                    lineObs += boost::lexical_cast<std::string>(static_cast<int>(observables_iter->second.PRN));
                    //lineObs += std::string(2, ' ');
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Pseudorange_m, 3), 14);

                    //Loss of lock indicator (LLI)
                    int lli = 0; // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    int ssi = Rinex_Printer::signalStrength(observables_iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // GPS L1 CA PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_phase_rads/GPS_TWO_PI, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // GPS L1 CA DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }

                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    //GPS L1 SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.CN0_dB_hz, 3), 14);

                    if (lineObs.size() < 80) lineObs += std::string(80 - lineObs.size(), ' ');
                    out << lineObs << std::endl;
                }
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream & out, const Gps_CNAV_Ephemeris & eph, double obs_time, const std::map<int, Gnss_Synchro> & observables)
{
    // RINEX observations timestamps are GPS timestamps.
    std::string line;

    boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(eph, obs_time);
    std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    //double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    //double gps_t = eph.sv_clock_correction(obs_time);
    double gps_t = obs_time;

    std::string month (timestring, 4, 2);
    std::string day (timestring, 6, 2);
    std::string hour (timestring, 9, 2);
    std::string minutes (timestring, 11, 2);

    std::string year (timestring, 0, 4);
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
    double seconds = fmod(gps_t, 60);
    // Add extra 0 if seconds are < 10
    if (seconds < 10)
        {
            line += std::string(1, '0');
        }
    line += Rinex_Printer::asString(seconds, 7);
    line += std::string(2, ' ');
    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    //Number of satellites observed in current epoch
    int numSatellitesObserved = 0;
    std::map<int, Gnss_Synchro>::const_iterator observables_iter;
    for(observables_iter = observables.cbegin();
            observables_iter != observables.cend();
            observables_iter++)
        {
            numSatellitesObserved++;
        }
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(numSatellitesObserved), 3);

    // Receiver clock offset (optional)
    //line += rightJustify(asString(clockOffset, 12), 15);

    line += std::string(80 - line.size(), ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    for(observables_iter = observables.cbegin();
            observables_iter != observables.cend();
            observables_iter++)
        {
            std::string lineObs;
            lineObs.clear();
            lineObs += satelliteSystem["GPS"];
            if (static_cast<int>(observables_iter->second.PRN) < 10) lineObs += std::string(1, '0');
            lineObs += boost::lexical_cast<std::string>(static_cast<int>(observables_iter->second.PRN));
            //lineObs += std::string(2, ' ');
            //GPS L2 PSEUDORANGE
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Pseudorange_m, 3), 14);

            //Loss of lock indicator (LLI)
            int lli = 0; // Include in the observation!!
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            //else
            //    {
            //       lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
            //   }

            // Signal Strength Indicator (SSI)
            int ssi = Rinex_Printer::signalStrength(observables_iter->second.CN0_dB_hz);
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

            // GPS L2 PHASE
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_phase_rads/GPS_TWO_PI, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            //else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
            //    }
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

            // GPS L2 DOPPLER
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_Doppler_hz, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            //else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
            //   }

            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

            //GPS L2 SIGNAL STRENGTH
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.CN0_dB_hz, 3), 14);

            if (lineObs.size() < 80) lineObs += std::string(80 - lineObs.size(), ' ');
            out << lineObs << std::endl;
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream & out, const Gps_Ephemeris & eph, const Gps_CNAV_Ephemeris & eph_cnav, double obs_time, const std::map<int, Gnss_Synchro> & observables)
{
    if(eph_cnav.d_i_0){} // avoid warning, not needed
    // RINEX observations timestamps are GPS timestamps.
    std::string line;

    boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(eph, obs_time);
    std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    //double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    //double gps_t = eph.sv_clock_correction(obs_time);
    double gps_t = obs_time;

    std::string month (timestring, 4, 2);
    std::string day (timestring, 6, 2);
    std::string hour (timestring, 9, 2);
    std::string minutes (timestring, 11, 2);

    std::string year (timestring, 0, 4);
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
    double seconds = fmod(gps_t, 60);
    // Add extra 0 if seconds are < 10
    if (seconds < 10)
        {
            line += std::string(1, '0');
        }
    line += Rinex_Printer::asString(seconds, 7);
    line += std::string(2, ' ');
    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    //Number of satellites observed in current epoch

    //Get maps with GPS L1 and L2 observations
    std::map<int, Gnss_Synchro> observablesL1;
    std::map<int, Gnss_Synchro> observablesL2;
    std::map<int, Gnss_Synchro>::const_iterator observables_iter;

    std::multimap<unsigned int, Gnss_Synchro> total_mmap;
    std::multimap<unsigned int, Gnss_Synchro>::iterator mmap_iter;
    for(observables_iter = observables.cbegin();
            observables_iter != observables.cend();
            observables_iter++)
        {
            std::string system_(&observables_iter->second.System, 1);
            std::string sig_(observables_iter->second.Signal);
            if((system_.compare("G") == 0) && (sig_.compare("1C") == 0))
                {
                    observablesL1.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                    total_mmap.insert(std::pair<unsigned int, Gnss_Synchro>(observables_iter->second.PRN, observables_iter->second));
                }
            if((system_.compare("G") == 0) && (sig_.compare("2S") == 0))
                {
                    observablesL2.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                    mmap_iter = total_mmap.find(observables_iter->second.PRN);
                    if(mmap_iter == total_mmap.end())
                        {
                            Gnss_Synchro gs = Gnss_Synchro();
                            total_mmap.insert(std::pair<unsigned int, Gnss_Synchro>(observables_iter->second.PRN, gs));
                        }
                    total_mmap.insert(std::pair<unsigned int, Gnss_Synchro>(observables_iter->second.PRN, observables_iter->second));
                }
        }

    // Fill with zeros satellites with L1 obs but not L2
    std::multimap<unsigned int, Gnss_Synchro> mmap_aux;
    mmap_aux = total_mmap;
    for(mmap_iter = mmap_aux.begin();
            mmap_iter != mmap_aux.end();
            mmap_iter++)
        {
            if((total_mmap.count(mmap_iter->second.PRN)) == 1 && (mmap_iter->second.PRN != 0))
                {
                    Gnss_Synchro gs = Gnss_Synchro();
                    std::string sys = "G";
                    gs.System = *sys.c_str();
                    std::string sig = "2S";
                    std::memcpy(static_cast<void*>(gs.Signal), sig.c_str(), 3);
                    gs.PRN = mmap_iter->second.PRN;
                    total_mmap.insert(std::pair<unsigned int, Gnss_Synchro>(mmap_iter->second.PRN, gs));
                }
        }

    std::set<unsigned int> available_prns;
    std::set<unsigned int>::iterator it;
    for(observables_iter = observablesL1.cbegin();
            observables_iter != observablesL1.cend();
            observables_iter++)
        {
            unsigned int prn_ = observables_iter->second.PRN;
            it = available_prns.find(prn_);
            if(it == available_prns.end())
                {
                    available_prns.insert(prn_);
                }
        }

    for(observables_iter = observablesL2.cbegin();
            observables_iter != observablesL2.cend();
            observables_iter++)
        {
            unsigned int prn_ = observables_iter->second.PRN;
            it = available_prns.find(prn_);
            if(it == available_prns.end())
                {
                    available_prns.insert(prn_);
                }
        }

    int numSatellitesObserved = available_prns.size();
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(numSatellitesObserved), 3);
    // Receiver clock offset (optional)
    //line += rightJustify(asString(clockOffset, 12), 15);
    line += std::string(80 - line.size(), ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    std::string lineObs;
    std::pair <std::multimap<unsigned int, Gnss_Synchro>::iterator, std::multimap<unsigned int, Gnss_Synchro>::iterator> ret;
    for(it = available_prns.begin();
            it != available_prns.end();
            it++)
        {
            lineObs.clear();
            lineObs += satelliteSystem["GPS"];
            if (static_cast<int>(*it) < 10) lineObs += std::string(1, '0');
            lineObs += boost::lexical_cast<std::string>(static_cast<int>(*it));
            ret = total_mmap.equal_range(*it);
            for (std::multimap<unsigned int, Gnss_Synchro>::iterator iter = ret.first; iter != ret.second; ++iter)
                {
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    //Loss of lock indicator (LLI)
                    int lli = 0; // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //   {
                    //       lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //   }

                    // Signal Strength Indicator (SSI)
                    int ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // GPS CARRIER PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_phase_rads / (GALILEO_TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // GPS  DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // GPS SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            if (lineObs.size() < 80) lineObs += std::string(80 - lineObs.size(), ' ');
            out << lineObs << std::endl;
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Galileo_Ephemeris& eph, double obs_time, const std::map<int,Gnss_Synchro>& observables, const std::string galileo_bands)
{
    // RINEX observations timestamps are Galileo timestamps.
    // See http://gage14.upc.es/gLAB/HTML/Observation_Rinex_v3.01.html
    std::string line;

    boost::posix_time::ptime p_galileo_time = Rinex_Printer::compute_Galileo_time(eph, obs_time);
    std::string timestring = boost::posix_time::to_iso_string(p_galileo_time);
    //double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    //double gps_t = eph.sv_clock_correction(obs_time);
    double galileo_t = obs_time;

    std::string month (timestring, 4, 2);
    std::string day (timestring, 6, 2);
    std::string hour (timestring, 9, 2);
    std::string minutes (timestring, 11, 2);

    std::string year (timestring, 0, 4);
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
    double seconds = fmod(galileo_t, 60);
    // Add extra 0 if seconds are < 10
    if (seconds < 10)
        {
            line += std::string(1, '0');
        }
    line += Rinex_Printer::asString(seconds, 7);
    line += std::string(2, ' ');
    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    //Number of satellites observed in current epoch

    //Get maps with Galileo observations
    std::map<int, Gnss_Synchro> observablesE1B;
    std::map<int, Gnss_Synchro> observablesE5A;
    std::map<int, Gnss_Synchro> observablesE5B;
    std::map<int, Gnss_Synchro>::const_iterator observables_iter;

    for(observables_iter = observables.cbegin();
            observables_iter != observables.cend();
            observables_iter++)
        {
            std::string system_(&observables_iter->second.System, 1);
            std::string sig_(observables_iter->second.Signal);
            if((system_.compare("E") == 0) && (sig_.compare("1B") == 0))
                {
                    observablesE1B.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if((system_.compare("E") == 0) && (sig_.compare("5X") == 0))
                {
                    observablesE5A.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if((system_.compare("E") == 0) && (sig_.compare("7X") == 0))
                {
                    observablesE5B.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
        }
    std::size_t found_1B = galileo_bands.find("1B");
    std::size_t found_E5a = galileo_bands.find("5X");
    std::size_t found_E5b = galileo_bands.find("7X");

    std::multimap<unsigned int, Gnss_Synchro> total_map;
    std::set<unsigned int> available_prns;
    std::set<unsigned int>::iterator it;
    if(found_1B != std::string::npos)
        {
            for(observables_iter = observablesE1B.begin();
                    observables_iter != observablesE1B.end();
                    observables_iter++)
                {
                    unsigned int prn_ = observables_iter->second.PRN;
                    total_map.insert(std::pair<unsigned int, Gnss_Synchro>(prn_, observables_iter->second));
                    it = available_prns.find(prn_);
                    if(it == available_prns.end())
                        {
                            available_prns.insert(prn_);
                        }
                }
        }
    if(found_E5a != std::string::npos)
        {
            for(observables_iter = observablesE5A.cbegin();
                    observables_iter != observablesE5A.cend();
                    observables_iter++)
                {
                    unsigned int prn_ = observables_iter->second.PRN;
                    it = available_prns.find(prn_);
                    if(it == available_prns.end())
                        {
                            available_prns.insert(prn_);
                            if(found_1B != std::string::npos)
                                {
                                    Gnss_Synchro gs = Gnss_Synchro();
                                    std::string sys = "E";
                                    gs.System = *sys.c_str();
                                    std::string sig = "1B";
                                    std::memcpy(static_cast<void*>(gs.Signal), sig.c_str(), 3);
                                    gs.PRN = prn_;
                                    total_map.insert(std::pair<unsigned int, Gnss_Synchro>(prn_, gs));
                                }
                        }
                    total_map.insert(std::pair<unsigned int, Gnss_Synchro>(prn_, observables_iter->second));
                }
        }
    if(found_E5b != std::string::npos)
        {
            for(observables_iter = observablesE5B.cbegin();
                    observables_iter != observablesE5B.cend();
                    observables_iter++)
                {
                    unsigned int prn_ = observables_iter->second.PRN;
                    it = available_prns.find(prn_);
                    if(it == available_prns.end())
                        {
                            available_prns.insert(prn_);
                            if(found_1B != std::string::npos)
                                {
                                    Gnss_Synchro gs = Gnss_Synchro();
                                    std::string sys = "E";
                                    gs.System = *sys.c_str();
                                    std::string sig = "1B";
                                    std::memcpy(static_cast<void*>(gs.Signal), sig.c_str(), 3);
                                    gs.PRN = prn_;
                                    total_map.insert(std::pair<unsigned int, Gnss_Synchro>(prn_, gs));
                                }
                            if(found_E5a != std::string::npos)
                                {
                                    Gnss_Synchro gs = Gnss_Synchro();
                                    std::string sys = "E";
                                    gs.System = *sys.c_str();
                                    std::string sig = "5X";
                                    std::memcpy(static_cast<void*>(gs.Signal), sig.c_str(), 3);
                                    gs.PRN = prn_;
                                    total_map.insert(std::pair<unsigned int, Gnss_Synchro>(prn_, gs));
                                }
                        }
                    else
                        {
                            // if 5X is listed but empty
                            if(found_E5a != std::string::npos)
                                {
                                    if( (total_map.count(prn_)) == 1)
                                        {
                                            Gnss_Synchro gs = Gnss_Synchro();
                                            std::string sys = "E";
                                            gs.System = *sys.c_str();
                                            std::string sig = "5X";
                                            std::memcpy(static_cast<void*>(gs.Signal), sig.c_str(), 3);
                                            gs.PRN = prn_;
                                            total_map.insert(std::pair<unsigned int, Gnss_Synchro>(prn_, gs));
                                        }
                                }
                        }
                    total_map.insert(std::pair<unsigned int, Gnss_Synchro>(prn_, observables_iter->second));
                }
        }
    int numSatellitesObserved = available_prns.size();
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(numSatellitesObserved), 3);
    // Receiver clock offset (optional)
    //line += rightJustify(asString(clockOffset, 12), 15);
    line += std::string(80 - line.size(), ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    std::string lineObs;
    std::pair <std::multimap<unsigned int, Gnss_Synchro>::iterator, std::multimap<unsigned int, Gnss_Synchro>::iterator> ret;
    for(it = available_prns.begin();
            it != available_prns.end();
            it++)
        {
            lineObs.clear();
            lineObs += satelliteSystem["Galileo"];
            if (static_cast<int>(*it) < 10) lineObs += std::string(1, '0');
            lineObs += boost::lexical_cast<std::string>(static_cast<int>(*it));
            ret = total_map.equal_range(*it);
            for (std::multimap<unsigned int, Gnss_Synchro>::iterator iter = ret.first; iter != ret.second; ++iter)
                {
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    //Loss of lock indicator (LLI)
                    int lli = 0; // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //   }

                    // Signal Strength Indicator (SSI)
                    int ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // Galileo CARRIER PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_phase_rads / (GALILEO_TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // Galileo  DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //       lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // Galileo SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            if (lineObs.size() < 80) lineObs += std::string(80 - lineObs.size(), ' ');
            out << lineObs << std::endl;
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Gps_Ephemeris& gps_eph, const Galileo_Ephemeris& galileo_eph,  double gps_obs_time, const std::map<int,Gnss_Synchro>& observables)
{
    if(galileo_eph.e_1){} // avoid warning, not needed
    std::string line;

    boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(gps_eph, gps_obs_time);
    std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    //double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    //double gps_t = eph.sv_clock_correction(obs_time);
    double gps_t = gps_obs_time;

    std::string month (timestring, 4, 2);
    std::string day (timestring, 6, 2);
    std::string hour (timestring, 9, 2);
    std::string minutes (timestring, 11, 2);

    std::string year (timestring, 0, 4);
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
    double seconds = fmod(gps_t, 60);
    // Add extra 0 if seconds are < 10
    if (seconds < 10)
        {
            line +=std::string(1, '0');
        }
    line += Rinex_Printer::asString(seconds, 7);
    line += std::string(2, ' ');
    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    //Number of satellites observed in current epoch

    //Get maps with observations
    std::map<int, Gnss_Synchro> observablesG1C;
    std::map<int, Gnss_Synchro> observablesE1B;
    std::map<int, Gnss_Synchro> observablesE5A;
    std::map<int, Gnss_Synchro> observablesE5B;
    std::map<int, Gnss_Synchro>::const_iterator observables_iter;

    for(observables_iter = observables.cbegin();
            observables_iter != observables.cend();
            observables_iter++)
        {
            std::string system_(&observables_iter->second.System, 1);
            std::string sig_(observables_iter->second.Signal);
            if((system_.compare("E") == 0) && (sig_.compare("1B") == 0))
                {
                    observablesE1B.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if((system_.compare("E") == 0) && (sig_.compare("5X") == 0))
                {
                    observablesE5A.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if((system_.compare("E") == 0) && (sig_.compare("7X") == 0))
                {
                    observablesE5B.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if((system_.compare("G") == 0) && (sig_.compare("1C") == 0))
                {
                    observablesG1C.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
        }

    std::multimap<unsigned int, Gnss_Synchro> total_gal_map;
    std::set<unsigned int> available_gal_prns;
    std::set<unsigned int>::iterator it;
    for(observables_iter = observablesE1B.cbegin();
            observables_iter != observablesE1B.cend();
            observables_iter++)
        {
            unsigned int prn_ = observables_iter->second.PRN;
            total_gal_map.insert(std::pair<unsigned int, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_gal_prns.find(prn_);
            if(it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for(observables_iter = observablesE5A.cbegin();
            observables_iter != observablesE5A.cend();
            observables_iter++)
        {
            unsigned int prn_ = observables_iter->second.PRN;
            total_gal_map.insert(std::pair<unsigned int, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_gal_prns.find(prn_);
            if(it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for(observables_iter = observablesE5B.cbegin();
            observables_iter != observablesE5B.cend();
            observables_iter++)
        {
            unsigned int prn_ = observables_iter->second.PRN;
            total_gal_map.insert(std::pair<unsigned int, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_gal_prns.find(prn_);
            if(it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    int numGalSatellitesObserved = available_gal_prns.size();
    int numGpsSatellitesObserved = observablesG1C.size();
    int numSatellitesObserved = numGalSatellitesObserved + numGpsSatellitesObserved;
    line += Rinex_Printer::rightJustify(boost::lexical_cast<std::string>(numSatellitesObserved), 3);

    // Receiver clock offset (optional)
    //line += rightJustify(asString(clockOffset, 12), 15);

    line += std::string(80 - line.size(), ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << std::endl;

    std::string s;
    std::string lineObs;
    for(observables_iter = observablesG1C.cbegin();
            observables_iter != observablesG1C.cend();
            observables_iter++)
        {
            lineObs.clear();

            s.assign(1, observables_iter->second.System);
            if(s.compare("G") == 0) lineObs += satelliteSystem["GPS"];
            if(s.compare("E") == 0) lineObs += satelliteSystem["Galileo"]; // should not happen
            if (static_cast<int>(observables_iter->second.PRN) < 10) lineObs += std::string(1, '0');
            lineObs += boost::lexical_cast<std::string>(static_cast<int>(observables_iter->second.PRN));
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Pseudorange_m, 3), 14);

            //Loss of lock indicator (LLI)
            int lli = 0; // Include in the observation!!
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            //else
            //    {
            //       lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
            //    }

            // Signal Strength Indicator (SSI)
            int ssi = Rinex_Printer::signalStrength(observables_iter->second.CN0_dB_hz);
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

            // PHASE
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_phase_rads/GPS_TWO_PI, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            //else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
            //   }
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

            // DOPPLER
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_Doppler_hz, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            //else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
            //    }
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

            // SIGNAL STRENGTH
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.CN0_dB_hz, 3), 14);

            if (lineObs.size() < 80) lineObs += std::string(80 - lineObs.size(), ' ');
            out << lineObs << std::endl;
        }

    std::pair <std::multimap<unsigned int, Gnss_Synchro>::iterator, std::multimap<unsigned int, Gnss_Synchro>::iterator> ret;
    for(it = available_gal_prns.begin();
            it != available_gal_prns.end();
            it++)
        {
            lineObs.clear();
            lineObs += satelliteSystem["Galileo"];
            if (static_cast<int>(*it) < 10) lineObs += std::string(1, '0');
            lineObs += boost::lexical_cast<std::string>(static_cast<int>(*it));
            ret = total_gal_map.equal_range(*it);
            for (std::multimap<unsigned int, Gnss_Synchro>::iterator iter = ret.first; iter != ret.second; ++iter)
                {
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    //Loss of lock indicator (LLI)
                    int lli = 0; // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    int ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // Galileo CARRIER PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_phase_rads / (GALILEO_TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // Galileo  DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    //else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<short>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int>(ssi), 1);

                    // Galileo SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            if (lineObs.size() < 80) lineObs += std::string(80 - lineObs.size(), ' ');
            out << lineObs << std::endl;
        }
}



void Rinex_Printer::to_date_time(int gps_week, int gps_tow, int &year, int &month, int &day, int &hour, int &minute, int &second)
{
    // represents GPS time (week, TOW) in the date time format of the Gregorian calendar.
    // -> Leap years are considered, but leap seconds are not.
    int days_per_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    // seconds in a not leap year
    const int secs_per_day = 24*60*60;
    const int secs_per_week = 7*secs_per_day;
    const int secs_per_normal_year = 365*secs_per_day;
    const int secs_per_leap_year = secs_per_normal_year + secs_per_day;

    // the GPS epoch is 06.01.1980 00:00, i.e. midnight 5. / 6. January 1980
    // -> seconds since then
    int secs_since_gps_epoch = gps_week*secs_per_week + gps_tow;

    // find year, consider leap years
    bool is_leap_year;
    int remaining_secs = secs_since_gps_epoch + 5*secs_per_day;
    for (int y = 1980; true ; y++)
        {
            is_leap_year = y%4 == 0 && (y%100 != 0 || y%400 == 0);
            int secs_in_year_y = is_leap_year ? secs_per_leap_year : secs_per_normal_year;

            if (secs_in_year_y <= remaining_secs)
                {
                    remaining_secs -= secs_in_year_y;
                }
            else
                {
                    year = y;
                    //std::cout << "year: year=" << year << " secs_in_year_y="<< secs_in_year_y << " remaining_secs="<< remaining_secs << std::endl;
                    break;
                }
            //std::cout << "year: y=" << y << " secs_in_year_y="<< secs_in_year_y << " remaining_secs="<< remaining_secs << std::endl;
        }

    // find month
    for (int m = 1; true ; m++)
        {
            int secs_in_month_m = days_per_month[m-1]*secs_per_day;
            if (is_leap_year && m == 2 ) // consider February of leap year
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
                    //std::cout << "month: month=" << month << " secs_in_month_m="<< secs_in_month_m << " remaining_secs="<< remaining_secs << std::endl;
                    break;
                }
            //std::cout << "month: m=" << m << " secs_in_month_m="<< secs_in_month_m << " remaining_secs="<< remaining_secs << std::endl;
        }

    day = remaining_secs/secs_per_day+1;
    remaining_secs = remaining_secs%secs_per_day;

    hour = remaining_secs/(60*60);
    remaining_secs = remaining_secs%(60*60);

    minute = remaining_secs/60;
    second = remaining_secs%60;
}


//void Rinex_Printer::log_rinex_sbs(std::fstream& out, const Sbas_Raw_Msg& sbs_message)
//{
//    // line 1: PRN / EPOCH / RCVR
//    std::stringstream line1;
//
//    // SBAS PRN
//    line1 << sbs_message.get_prn();
//    line1 << " ";
//
//    // gps time of reception
//    int gps_week;
//    double gps_sec;
//    if(sbs_message.get_rx_time_obj().get_gps_time(gps_week, gps_sec))
//        {
//            int year;
//            int month;
//            int day;
//            int hour;
//            int minute;
//            int second;
//
//            double gps_sec_one_digit_precicion = round(gps_sec *10)/10; // to prevent rounding towards 60.0sec in the stream output
//            int gps_tow = trunc(gps_sec_one_digit_precicion);
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
//    out << line1.str() << std::endl;
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
//    out << line2.str() << std::endl;
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
//    out << line3.str() << std::endl;
//}


int Rinex_Printer::signalStrength(const double snr)
{
    int ss;
    ss = int ( std::min( std::max( int (floor(snr/6)) , 1), 9) );
    return ss;
}


boost::posix_time::ptime Rinex_Printer::compute_UTC_time(const Gps_Navigation_Message& nav_msg)
{
    // if we are processing a file -> wait to leap second to resolve the ambiguity else take the week from the local system time
    //: idea resolve the ambiguity with the leap second  http://www.colorado.edu/geography/gcraft/notes/gps/gpseow.htm
    const double utc_t = nav_msg.utc_time(nav_msg.d_TOW);
    boost::posix_time::time_duration t = boost::posix_time::millisec((utc_t + 604800 * static_cast<double>(nav_msg.i_GPS_week)) * 1000);
    boost::posix_time::ptime p_time(boost::gregorian::date(1999, 8, 22), t);
    return p_time;
}


boost::posix_time::ptime Rinex_Printer::compute_GPS_time(const Gps_Ephemeris& eph, const double obs_time)
{
    // The RINEX v2.11 v3.00 format uses GPS time for the observations epoch, not UTC time, thus, no leap seconds needed here.
    // (see Section 3 in http://igscb.jpl.nasa.gov/igscb/data/format/rinex211.txt)
    // (see Pag. 17 in http://igscb.jpl.nasa.gov/igscb/data/format/rinex300.pdf)
    // --??? No time correction here, since it will be done in the RINEX processor
    const double gps_t = obs_time;
    boost::posix_time::time_duration t = boost::posix_time::millisec((gps_t + 604800 * static_cast<double>(eph.i_GPS_week % 1024)) * 1000);
    boost::posix_time::ptime p_time(boost::gregorian::date(1999, 8, 22), t);
    return p_time;
}


boost::posix_time::ptime Rinex_Printer::compute_GPS_time(const Gps_CNAV_Ephemeris & eph, const double obs_time)
{
    // The RINEX v2.11 v3.00 format uses GPS time for the observations epoch, not UTC time, thus, no leap seconds needed here.
    // (see Section 3 in http://igscb.jpl.nasa.gov/igscb/data/format/rinex211.txt)
    // (see Pag. 17 in http://igscb.jpl.nasa.gov/igscb/data/format/rinex300.pdf)
    // --??? No time correction here, since it will be done in the RINEX processor
    const double gps_t = obs_time;
    boost::posix_time::time_duration t = boost::posix_time::millisec((gps_t + 604800 * static_cast<double>(eph.i_GPS_week % 1024)) * 1000);
    boost::posix_time::ptime p_time(boost::gregorian::date(1999, 8, 22), t);
    return p_time;
}


boost::posix_time::ptime Rinex_Printer::compute_Galileo_time(const Galileo_Ephemeris& eph, const double obs_time)
{
    // The RINEX v2.11 v3.00 format uses Galileo time for the observations epoch, not UTC time, thus, no leap seconds needed here.
    // (see Pag. 17 in http://igscb.jpl.nasa.gov/igscb/data/format/rinex301.pdf)
    // --??? No time correction here, since it will be done in the RINEX processor
    double galileo_t = obs_time;
    boost::posix_time::time_duration t = boost::posix_time::millisec((galileo_t + 604800 * static_cast<double>(eph.WN_5)) * 1000); //
    boost::posix_time::ptime p_time(boost::gregorian::date(1999, 8, 22), t);
    return p_time;
}


boost::posix_time::ptime Rinex_Printer::compute_UTC_time(const Glonass_Gnav_Ephemeris& eph, const double obs_time)
{
    double tod = 0.0;
    double glot2utc = 3*3600;
    double obs_time_glot = 0.0;
    int i = 0;

    // Get observation time in nearly GLONASS time. Correction for leap seconds done at the end
    obs_time_glot = obs_time + glot2utc;

    // Get seconds of day in glonass time
    tod = fmod (obs_time_glot, 86400);

    // Form date and time duration types
    boost::posix_time::time_duration t1(0, 0, tod);
    boost::gregorian::date d1(eph.d_yr, 1, 1);
    boost::gregorian::days d2(eph.d_N_T-1);
    boost::posix_time::ptime glo_time(d1 + d2, t1);

    // Convert to utc
    boost::posix_time::time_duration t2(0, 0, glot2utc);
    boost::posix_time::ptime utc_time = glo_time - t2;

    // Adjust for leap second correction
    for (i = 0; GLONASS_LEAP_SECONDS[i][0]>0; i++)
        {
            boost::posix_time::time_duration t3(GLONASS_LEAP_SECONDS[i][3], GLONASS_LEAP_SECONDS[i][4], GLONASS_LEAP_SECONDS[i][5]);
            boost::gregorian::date d3(GLONASS_LEAP_SECONDS[i][0], GLONASS_LEAP_SECONDS[i][1], GLONASS_LEAP_SECONDS[i][2]);
            boost::posix_time::ptime ls_time(d3, t3);
            if (utc_time >= ls_time)
                {
                    // We subtract the leap second when going from gpst to utc
                    utc_time = utc_time - boost::posix_time::time_duration(0,0,fabs(GLONASS_LEAP_SECONDS[i][6]));
                    break;
                }
        }

    return utc_time;
}


double Rinex_Printer::get_leap_second(const Glonass_Gnav_Ephemeris& eph, const double gps_obs_time)
{
    double tod = 0.0;
    double glot2utc = 3*3600;
    double obs_time_glot = 0.0;
    int i = 0;
    double leap_second = 0;

    // Get observation time in nearly GLONASS time. Correction for leap seconds done at the end
    obs_time_glot = gps_obs_time + glot2utc;

    // Get seconds of day in glonass time
    tod = fmod (obs_time_glot, 86400);

    // Form date and time duration types
    boost::posix_time::time_duration t1(0, 0, tod);
    boost::gregorian::date d1(eph.d_yr, 1, 1);
    boost::gregorian::days d2(eph.d_N_T-1);
    boost::posix_time::ptime glo_time(d1 + d2, t1);

    // Convert to utc
    boost::posix_time::time_duration t2(0, 0, glot2utc);
    boost::posix_time::ptime utc_time = glo_time - t2;

    // Adjust for leap second correction
    for (i = 0; GLONASS_LEAP_SECONDS[i][0]>0; i++)
        {
            boost::posix_time::time_duration t3(GLONASS_LEAP_SECONDS[i][3], GLONASS_LEAP_SECONDS[i][4], GLONASS_LEAP_SECONDS[i][5]);
            boost::gregorian::date d3(GLONASS_LEAP_SECONDS[i][0], GLONASS_LEAP_SECONDS[i][1], GLONASS_LEAP_SECONDS[i][2]);
            boost::posix_time::ptime ls_time(d3, t3);
            if (utc_time >= ls_time)
                {
                    // We subtract the leap second when going from gpst to utc
                    leap_second = fabs(GLONASS_LEAP_SECONDS[i][6]);
                    break;
                }
        }

    return leap_second;
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
