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
#include "Beidou_DNAV.h"
#include "GLONASS_L1_L2_CA.h"
#include "GPS_L1_CA.h"
#include "Galileo_E1.h"
#include "beidou_dnav_ephemeris.h"
#include "beidou_dnav_iono.h"
#include "beidou_dnav_utc_model.h"
#include "galileo_ephemeris.h"
#include "galileo_iono.h"
#include "galileo_utc_model.h"
#include "glonass_gnav_almanac.h"
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
#include <glog/logging.h>
#include <algorithm>  // for min and max
#include <array>
#include <cmath>  // for floor
#include <exception>
#include <iostream>  // for cout
#include <iterator>
#include <ostream>
#include <set>
#include <unistd.h>  // for getlogin_r()
#include <utility>
#include <vector>


Rinex_Printer::Rinex_Printer(int32_t conf_version,
    const std::string& base_path,
    const std::string& base_name) : d_fake_cnav_iode(1),
                                    d_numberTypesObservations(4),
                                    d_rinex_header_updated(false),
                                    d_rinex_header_written(false),
                                    d_pre_2009_file(false)

{
    // RINEX v3.02 codes
    satelliteSystem["GPS"] = "G";
    satelliteSystem["GLONASS"] = "R";
    satelliteSystem["SBAS payload"] = "S";
    satelliteSystem["Galileo"] = "E";
    satelliteSystem["Beidou"] = "C";
    satelliteSystem["Mixed"] = "M";

    observationCode["GPS_L1_CA"] = "1C";          // "1C" GPS L1 C/A
    observationCode["GPS_L1_P"] = "1P";           // "1P" GPS L1 P
    observationCode["GPS_L1_Z_TRACKING"] = "1W";  // "1W" GPS L1 Z-tracking and similar (AS on)
    observationCode["GPS_L1_Y"] = "1Y";           // "1Y" GPS L1 Y
    observationCode["GPS_L1_M "] = "1M";          // "1M" GPS L1 M
    observationCode["GPS_L1_CODELESS"] = "1N";    // "1N" GPS L1 codeless
    observationCode["GPS_L2_CA"] = "2C";          // "2C" GPS L2 C/A
    observationCode["L2_SEMI_CODELESS"] = "2D";   // "2D" GPS L2 L1(C/A)+(P2-P1) semi-codeless
    observationCode["GPS_L2_L2CM"] = "2S";        // "2S" GPS L2 L2C (M)
    observationCode["GPS_L2_L2CL"] = "2L";        // "2L" GPS L2 L2C (L)
    observationCode["GPS_L2_L2CML"] = "2X";       // "2X" GPS L2 L2C (M+L)
    observationCode["GPS_L2_P"] = "2P";           // "2P" GPS L2 P
    observationCode["GPS_L2_Z_TRACKING"] = "2W";  // "2W" GPS L2 Z-tracking and similar (AS on)
    observationCode["GPS_L2_Y"] = "2Y";           // "2Y" GPS L2 Y
    observationCode["GPS_L2_M"] = "2M";           // "2M" GPS GPS L2 M
    observationCode["GPS_L2_codeless"] = "2N";    // "2N" GPS L2 codeless
    observationCode["GPS_L5_I"] = "5I";           // "5I" GPS L5 I
    observationCode["GPS_L5_Q"] = "5Q";           // "5Q" GPS L5 Q
    observationCode["GPS_L5_IQ"] = "5X";          // "5X" GPS L5 I+Q
    observationCode["GLONASS_G1_CA"] = "1C";      // "1C" GLONASS G1 C/A
    observationCode["GLONASS_G1_P"] = "1P";       // "1P" GLONASS G1 P
    observationCode["GLONASS_G2_CA"] = "2C";      // "2C" GLONASS G2 C/A  (Glonass M)
    observationCode["GLONASS_G2_P"] = "2P";       // "2P" GLONASS G2 P
    observationCode["GALILEO_E1_A"] = "1A";       // "1A" GALILEO E1 A (PRS)
    observationCode["GALILEO_E1_B"] = "1B";       // "1B" GALILEO E1 B (I/NAV OS/CS/SoL)
    observationCode["GALILEO_E1_C"] = "1C";       // "1C" GALILEO E1 C (no data)
    observationCode["GALILEO_E1_BC"] = "1X";      // "1X" GALILEO E1 B+C
    observationCode["GALILEO_E1_ABC"] = "1Z";     // "1Z" GALILEO E1 A+B+C
    observationCode["GALILEO_E5a_I"] = "5I";      // "5I" GALILEO E5a I (F/NAV OS)
    observationCode["GALILEO_E5a_Q"] = "5Q";      // "5Q" GALILEO E5a Q  (no data)
    observationCode["GALILEO_E5a_IQ"] = "5X";     // "5X" GALILEO E5a I+Q
    observationCode["GALILEO_E5b_I"] = "7I";      // "7I" GALILEO E5b I
    observationCode["GALILEO_E5b_Q"] = "7Q";      // "7Q" GALILEO E5b Q
    observationCode["GALILEO_E5b_IQ"] = "7X";     // "7X" GALILEO E5b I+Q
    observationCode["GALILEO_E5_I"] = "8I";       // "8I" GALILEO E5 I
    observationCode["GALILEO_E5_Q"] = "8Q";       // "8Q" GALILEO E5 Q
    observationCode["GALILEO_E5_IQ"] = "8X";      // "8X" GALILEO E5 I+Q
    observationCode["GALILEO_E56_A"] = "6A";      // "6A" GALILEO E6 A
    observationCode["GALILEO_E56_B"] = "6B";      // "6B" GALILEO E6 B
    observationCode["GALILEO_E56_C"] = "6C";      // "6C" GALILEO E6 C
    observationCode["GALILEO_E56_BC"] = "6X";     // "6X" GALILEO E6 B+C
    observationCode["GALILEO_E56_ABC"] = "6Z";    // "6Z" GALILEO E6 A+B+C
    observationCode["SBAS_L1_CA"] = "1C";         // "1C" SBAS L1 C/A
    observationCode["SBAS_L5_I"] = "5I";          // "5I" SBAS L5 I
    observationCode["SBAS_L5_Q"] = "5Q";          // "5Q" SBAS L5 Q
    observationCode["SBAS_L5_IQ"] = "5X";         // "5X" SBAS L5 I+Q
    observationCode["COMPASS_E2_I"] = "2I";
    observationCode["COMPASS_E2_Q"] = "2Q";
    observationCode["COMPASS_E2_IQ"] = "2X";
    observationCode["COMPASS_E5b_I"] = "7I";
    observationCode["COMPASS_E5b_Q"] = "7Q";
    observationCode["COMPASS_E5b_IQ"] = "7X";
    observationCode["COMPASS_E6_I"] = "6I";
    observationCode["COMPASS_E6_Q"] = "6Q";
    observationCode["COMPASS_E6_IQ"] = "6X";
    observationCode["BEIDOU_B1_I"] = "1I";
    observationCode["BEIDOU_B1_Q"] = "1Q";
    observationCode["BEIDOU_B1_IQ"] = "1X";
    observationCode["BEIDOU_B3_I"] = "6I";
    observationCode["BEIDOU_B3_Q"] = "6Q";
    observationCode["BEIDOU_B3_IQ"] = "6X";

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

    navfilename = base_rinex_path + fs::path::preferred_separator + Rinex_Printer::createFilename("RINEX_FILE_TYPE_GPS_NAV", base_name);
    obsfilename = base_rinex_path + fs::path::preferred_separator + Rinex_Printer::createFilename("RINEX_FILE_TYPE_OBS", base_name);
    sbsfilename = base_rinex_path + fs::path::preferred_separator + Rinex_Printer::createFilename("RINEX_FILE_TYPE_SBAS", base_name);
    navGalfilename = base_rinex_path + fs::path::preferred_separator + Rinex_Printer::createFilename("RINEX_FILE_TYPE_GAL_NAV", base_name);
    navMixfilename = base_rinex_path + fs::path::preferred_separator + Rinex_Printer::createFilename("RINEX_FILE_TYPE_MIXED_NAV", base_name);
    navGlofilename = base_rinex_path + fs::path::preferred_separator + Rinex_Printer::createFilename("RINEX_FILE_TYPE_GLO_NAV", base_name);
    navBdsfilename = base_rinex_path + fs::path::preferred_separator + Rinex_Printer::createFilename("RINEX_FILE_TYPE_BDS_NAV", base_name);

    Rinex_Printer::navFile.open(navfilename, std::ios::out | std::ios::in | std::ios::app);
    Rinex_Printer::obsFile.open(obsfilename, std::ios::out | std::ios::in | std::ios::app);
    Rinex_Printer::sbsFile.open(sbsfilename, std::ios::out | std::ios::app);
    Rinex_Printer::navGalFile.open(navGalfilename, std::ios::out | std::ios::in | std::ios::app);
    Rinex_Printer::navMixFile.open(navMixfilename, std::ios::out | std::ios::in | std::ios::app);
    Rinex_Printer::navGloFile.open(navGlofilename, std::ios::out | std::ios::in | std::ios::app);
    Rinex_Printer::navBdsFile.open(navBdsfilename, std::ios::out | std::ios::in | std::ios::app);

    if (!Rinex_Printer::navFile.is_open() or !Rinex_Printer::obsFile.is_open() or
        !Rinex_Printer::sbsFile.is_open() or !Rinex_Printer::navGalFile.is_open() or
        !Rinex_Printer::navMixFile.is_open() or !Rinex_Printer::navGloFile.is_open())
        {
            std::cout << "RINEX files cannot be saved. Wrong permissions?\n";
        }

    if (conf_version == 2)
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
    // close RINEX files
    const auto posn = navFile.tellp();
    const auto poso = obsFile.tellp();
    const auto poss = sbsFile.tellp();
    const auto posng = navGalFile.tellp();
    const auto posmn = navMixFile.tellp();
    const auto posnr = navGloFile.tellp();
    const auto posnc = navBdsFile.tellp();

    try
        {
            Rinex_Printer::navFile.close();
            Rinex_Printer::obsFile.close();
            Rinex_Printer::sbsFile.close();
            Rinex_Printer::navGalFile.close();
            Rinex_Printer::navGloFile.close();
            Rinex_Printer::navBdsFile.close();
        }
    catch (const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

    // If nothing written, erase the files.
    if (posn == 0)
        {
            errorlib::error_code ec;
            if (!fs::remove(fs::path(navfilename), ec))
                {
                    LOG(INFO) << "Error deleting temporary file";
                }
        }
    if (poso == 0)
        {
            errorlib::error_code ec;
            if (!fs::remove(fs::path(obsfilename), ec))
                {
                    LOG(INFO) << "Error deleting temporary file";
                }
        }
    if (poss == 0)
        {
            errorlib::error_code ec;
            if (!fs::remove(fs::path(sbsfilename), ec))
                {
                    LOG(INFO) << "Error deleting temporary file";
                }
        }
    if (posng == 0)
        {
            errorlib::error_code ec;
            if (!fs::remove(fs::path(navGalfilename), ec))
                {
                    LOG(INFO) << "Error deleting temporary file";
                }
        }
    if (posmn == 0)
        {
            errorlib::error_code ec;
            if (!fs::remove(fs::path(navMixfilename), ec))
                {
                    LOG(INFO) << "Error deleting temporary file";
                }
        }
    if (posnr == 0)
        {
            errorlib::error_code ec;
            if (!fs::remove(fs::path(navGlofilename), ec))
                {
                    LOG(INFO) << "Error deleting temporary file";
                }
        }
    if (posnc == 0)
        {
            errorlib::error_code ec;
            if (!fs::remove(fs::path(navBdsfilename), ec))
                {
                    LOG(INFO) << "Error deleting temporary file";
                }
        }
}


void Rinex_Printer::print_rinex_annotation(const Rtklib_Solver* pvt_solver, const std::map<int, Gnss_Synchro>& gnss_observables_map, double rx_time, int type_of_rx, bool flag_write_RINEX_obs_output)
{
    std::map<int, Galileo_Ephemeris>::const_iterator galileo_ephemeris_iter;
    std::map<int, Gps_Ephemeris>::const_iterator gps_ephemeris_iter;
    std::map<int, Gps_CNAV_Ephemeris>::const_iterator gps_cnav_ephemeris_iter;
    std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_ephemeris_iter;
    std::map<int, Beidou_Dnav_Ephemeris>::const_iterator beidou_dnav_ephemeris_iter;
    if (!d_rinex_header_written)  // & we have utc data in nav message!
        {
            galileo_ephemeris_iter = pvt_solver->galileo_ephemeris_map.cbegin();
            gps_ephemeris_iter = pvt_solver->gps_ephemeris_map.cbegin();
            gps_cnav_ephemeris_iter = pvt_solver->gps_cnav_ephemeris_map.cbegin();
            glonass_gnav_ephemeris_iter = pvt_solver->glonass_gnav_ephemeris_map.cbegin();
            beidou_dnav_ephemeris_iter = pvt_solver->beidou_dnav_ephemeris_map.cbegin();
            switch (type_of_rx)
                {
                case 1:  // GPS L1 C/A only
                    if (gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend())
                        {
                            rinex_obs_header(obsFile, gps_ephemeris_iter->second, rx_time);
                            rinex_nav_header(navFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second);
                            output_navfilename.push_back(navfilename);
                            log_rinex_nav(navFile, pvt_solver->gps_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 2:  // GPS L2C only
                    if (gps_cnav_ephemeris_iter != pvt_solver->gps_cnav_ephemeris_map.cend())
                        {
                            const std::string signal("2S");
                            rinex_obs_header(obsFile, gps_cnav_ephemeris_iter->second, rx_time, signal);
                            rinex_nav_header(navFile, pvt_solver->gps_cnav_iono, pvt_solver->gps_cnav_utc_model);
                            output_navfilename.push_back(navfilename);
                            log_rinex_nav(navFile, pvt_solver->gps_cnav_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 3:  // GPS L5 only
                    if (gps_cnav_ephemeris_iter != pvt_solver->gps_cnav_ephemeris_map.cend())
                        {
                            const std::string signal("L5");
                            rinex_obs_header(obsFile, gps_cnav_ephemeris_iter->second, rx_time, signal);
                            rinex_nav_header(navFile, pvt_solver->gps_cnav_iono, pvt_solver->gps_cnav_utc_model);
                            output_navfilename.push_back(navfilename);
                            log_rinex_nav(navFile, pvt_solver->gps_cnav_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 4:  // Galileo E1B only
                    if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                        {
                            rinex_obs_header(obsFile, galileo_ephemeris_iter->second, rx_time);
                            rinex_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navGalfilename);
                            log_rinex_nav(navGalFile, pvt_solver->galileo_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 5:  // Galileo E5a only
                    if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                        {
                            const std::string signal("5X");
                            rinex_obs_header(obsFile, galileo_ephemeris_iter->second, rx_time, signal);
                            rinex_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navGalfilename);
                            log_rinex_nav(navGalFile, pvt_solver->galileo_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 6:  // Galileo E5b only
                    if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                        {
                            const std::string signal("7X");
                            rinex_obs_header(obsFile, galileo_ephemeris_iter->second, rx_time, signal);
                            rinex_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navGalfilename);
                            log_rinex_nav(navGalFile, pvt_solver->galileo_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 7:  // GPS L1 C/A + GPS L2C
                    if ((gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != pvt_solver->gps_cnav_ephemeris_map.cend()))
                        {
                            const std::string signal("1C 2S");
                            rinex_obs_header(obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, rx_time, signal);
                            rinex_nav_header(navFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second);
                            output_navfilename.push_back(navfilename);
                            log_rinex_nav(navFile, pvt_solver->gps_cnav_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 8:  // GPS L1 + GPS L5
                    if ((gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != pvt_solver->gps_cnav_ephemeris_map.cend()))
                        {
                            const std::string signal("1C L5");
                            rinex_obs_header(obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, rx_time, signal);
                            rinex_nav_header(navFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second);
                            output_navfilename.push_back(navfilename);
                            log_rinex_nav(navFile, pvt_solver->gps_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 9:  // GPS L1 C/A + Galileo E1B
                    if ((galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()) and (gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()))
                        {
                            const std::string gal_signal("1B");
                            rinex_obs_header(obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gal_signal);
                            rinex_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navMixfilename);
                            log_rinex_nav(navMixFile, pvt_solver->gps_ephemeris_map, pvt_solver->galileo_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 10:  // GPS L1 C/A + Galileo E5a
                    if ((galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()) and (gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()))
                        {
                            const std::string gal_signal("5X");
                            rinex_obs_header(obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gal_signal);
                            rinex_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navMixfilename);
                            log_rinex_nav(navMixFile, pvt_solver->gps_ephemeris_map, pvt_solver->galileo_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 11:  // GPS L1 C/A + Galileo E5b
                    if ((galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()) and (gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()))
                        {
                            const std::string gal_signal("7X");
                            rinex_obs_header(obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gal_signal);
                            rinex_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navMixfilename);
                            log_rinex_nav(navMixFile, pvt_solver->gps_ephemeris_map, pvt_solver->galileo_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 13:  // L5+E5a
                    if ((galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != pvt_solver->gps_cnav_ephemeris_map.cend()))
                        {
                            const std::string gal_signal("5X");
                            const std::string gps_signal("L5");
                            rinex_obs_header(obsFile, gps_cnav_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gps_signal, gal_signal);
                            rinex_nav_header(navMixFile, pvt_solver->gps_cnav_iono, pvt_solver->gps_cnav_utc_model, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navMixfilename);
                            log_rinex_nav(navMixFile, pvt_solver->gps_cnav_ephemeris_map, pvt_solver->galileo_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 14:  // Galileo E1B + Galileo E5a
                    if ((galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()))
                        {
                            const std::string gal_signal("1B 5X");
                            rinex_obs_header(obsFile, galileo_ephemeris_iter->second, rx_time, gal_signal);
                            rinex_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navGalfilename);
                            log_rinex_nav(navGalFile, pvt_solver->galileo_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 15:  // Galileo E1B + Galileo E5b
                    if ((galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()))
                        {
                            const std::string gal_signal("1B 7X");
                            rinex_obs_header(obsFile, galileo_ephemeris_iter->second, rx_time, gal_signal);
                            rinex_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navGalfilename);
                            log_rinex_nav(navGalFile, pvt_solver->galileo_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 19:  // Galileo E5a + Galileo E5b
                    if ((galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()))
                        {
                            const std::string gal_signal("5X 7X");
                            rinex_obs_header(obsFile, galileo_ephemeris_iter->second, rx_time, gal_signal);
                            rinex_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navGalfilename);
                            log_rinex_nav(navGalFile, pvt_solver->galileo_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 23:  // GLONASS L1 C/A only
                    if (glonass_gnav_ephemeris_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                        {
                            const std::string signal("1G");
                            rinex_obs_header(obsFile, glonass_gnav_ephemeris_iter->second, rx_time, signal);
                            rinex_nav_header(navGloFile, pvt_solver->glonass_gnav_utc_model, glonass_gnav_ephemeris_iter->second);
                            output_navfilename.push_back(navGlofilename);
                            log_rinex_nav(navGloFile, pvt_solver->glonass_gnav_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 24:  // GLONASS L2 C/A only
                    if (glonass_gnav_ephemeris_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                        {
                            const std::string signal("2G");
                            rinex_obs_header(obsFile, glonass_gnav_ephemeris_iter->second, rx_time, signal);
                            rinex_nav_header(navGloFile, pvt_solver->glonass_gnav_utc_model, glonass_gnav_ephemeris_iter->second);
                            output_navfilename.push_back(navGlofilename);
                            log_rinex_nav(navGloFile, pvt_solver->glonass_gnav_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 25:  // GLONASS L1 C/A + GLONASS L2 C/A
                    if (glonass_gnav_ephemeris_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                        {
                            const std::string signal("1G 2G");
                            rinex_obs_header(obsFile, glonass_gnav_ephemeris_iter->second, rx_time, signal);
                            rinex_nav_header(navGloFile, pvt_solver->glonass_gnav_utc_model, glonass_gnav_ephemeris_iter->second);
                            output_navfilename.push_back(navGlofilename);
                            log_rinex_nav(navGloFile, pvt_solver->glonass_gnav_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 26:  // GPS L1 C/A + GLONASS L1 C/A
                    if ((glonass_gnav_ephemeris_iter != pvt_solver->glonass_gnav_ephemeris_map.cend()) and (gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()))
                        {
                            const std::string glo_signal("1G");
                            rinex_obs_header(obsFile, gps_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, rx_time, glo_signal);
                            if (d_version == 3)
                                {
                                    rinex_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->glonass_gnav_utc_model, pvt_solver->glonass_gnav_almanac);
                                    log_rinex_nav(navMixFile, pvt_solver->gps_ephemeris_map, pvt_solver->glonass_gnav_ephemeris_map);
                                    output_navfilename.push_back(navMixfilename);
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
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 27:  // Galileo E1B + GLONASS L1 C/A
                    if ((glonass_gnav_ephemeris_iter != pvt_solver->glonass_gnav_ephemeris_map.cend()) and (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()))
                        {
                            const std::string glo_signal("1G");
                            const std::string gal_signal("1B");
                            rinex_obs_header(obsFile, galileo_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, rx_time, glo_signal, gal_signal);
                            rinex_nav_header(navMixFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model, pvt_solver->glonass_gnav_utc_model, pvt_solver->glonass_gnav_almanac);
                            output_navfilename.push_back(navMixfilename);
                            log_rinex_nav(navMixFile, pvt_solver->galileo_ephemeris_map, pvt_solver->glonass_gnav_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 28:  // GPS L2C + GLONASS L1 C/A
                    if ((glonass_gnav_ephemeris_iter != pvt_solver->glonass_gnav_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != pvt_solver->gps_cnav_ephemeris_map.cend()))
                        {
                            const std::string glo_signal("1G");
                            rinex_obs_header(obsFile, gps_cnav_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, rx_time, glo_signal);
                            rinex_nav_header(navMixFile, pvt_solver->gps_cnav_iono, pvt_solver->gps_cnav_utc_model, pvt_solver->glonass_gnav_utc_model, pvt_solver->glonass_gnav_almanac);
                            output_navfilename.push_back(navfilename);
                            log_rinex_nav(navMixFile, pvt_solver->gps_cnav_ephemeris_map, pvt_solver->glonass_gnav_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 29:  // GPS L1 C/A + GLONASS L2 C/A
                    if ((glonass_gnav_ephemeris_iter != pvt_solver->glonass_gnav_ephemeris_map.cend()) and (gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()))
                        {
                            const std::string glo_signal("2G");
                            rinex_obs_header(obsFile, gps_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, rx_time, glo_signal);
                            if (d_version == 3)
                                {
                                    rinex_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->glonass_gnav_utc_model, pvt_solver->glonass_gnav_almanac);
                                    output_navfilename.push_back(navfilename);
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
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 30:  // Galileo E1B + GLONASS L2 C/A
                    if ((glonass_gnav_ephemeris_iter != pvt_solver->glonass_gnav_ephemeris_map.cend()) and (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()))
                        {
                            const std::string glo_signal("2G");
                            const std::string gal_signal("1B");
                            rinex_obs_header(obsFile, galileo_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, rx_time, glo_signal, gal_signal);
                            rinex_nav_header(navMixFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model, pvt_solver->glonass_gnav_utc_model, pvt_solver->glonass_gnav_almanac);
                            output_navfilename.push_back(navMixfilename);
                            log_rinex_nav(navMixFile, pvt_solver->galileo_ephemeris_map, pvt_solver->glonass_gnav_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 31:  // GPS L2C + GLONASS L2 C/A
                    if ((glonass_gnav_ephemeris_iter != pvt_solver->glonass_gnav_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != pvt_solver->gps_cnav_ephemeris_map.cend()))
                        {
                            const std::string glo_signal("2G");
                            rinex_obs_header(obsFile, gps_cnav_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, rx_time, glo_signal);
                            rinex_nav_header(navMixFile, pvt_solver->gps_cnav_iono, pvt_solver->gps_cnav_utc_model, pvt_solver->glonass_gnav_utc_model, pvt_solver->glonass_gnav_almanac);
                            output_navfilename.push_back(navfilename);
                            log_rinex_nav(navMixFile, pvt_solver->gps_cnav_ephemeris_map, pvt_solver->glonass_gnav_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 32:  // L1+E1+L5+E5a
                    if ((gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()) and
                        (gps_cnav_ephemeris_iter != pvt_solver->gps_cnav_ephemeris_map.cend()) and
                        (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()))
                        {
                            const std::string gal_signal("1B 5X");
                            const std::string gps_signal("1C L5");
                            rinex_obs_header(obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gps_signal, gal_signal);
                            rinex_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navMixfilename);
                            log_rinex_nav(navMixFile, pvt_solver->gps_ephemeris_map, pvt_solver->galileo_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 33:  // L1+E1+E5a
                    if ((gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()) and
                        (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()))
                        {
                            const std::string gal_signal("1B 5X");
                            rinex_obs_header(obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gal_signal);
                            rinex_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navMixfilename);
                            log_rinex_nav(navMixFile, pvt_solver->gps_ephemeris_map, pvt_solver->galileo_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 100:  // Galileo E6B
                    if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                        {
                            const std::string gal_signal("E6");
                            rinex_obs_header(obsFile, galileo_ephemeris_iter->second, rx_time, gal_signal);
                            rinex_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navGalfilename);
                            log_rinex_nav(navGalFile, pvt_solver->galileo_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 101:  // Galileo E1B + Galileo E6B
                    if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                        {
                            const std::string gal_signal("1B E6");
                            rinex_obs_header(obsFile, galileo_ephemeris_iter->second, rx_time, gal_signal);
                            rinex_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navGalfilename);
                            log_rinex_nav(navGalFile, pvt_solver->galileo_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 102:  // Galileo E5a + Galileo E6B
                    if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                        {
                            const std::string signal("5X E6");
                            rinex_obs_header(obsFile, galileo_ephemeris_iter->second, rx_time, signal);
                            rinex_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navGalfilename);
                            log_rinex_nav(navGalFile, pvt_solver->galileo_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 103:  // Galileo E5b + Galileo E6B
                    if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                        {
                            const std::string signal("7X E6");
                            rinex_obs_header(obsFile, galileo_ephemeris_iter->second, rx_time, signal);
                            rinex_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navGalfilename);
                            log_rinex_nav(navGalFile, pvt_solver->galileo_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 104:  // Galileo E1B + Galileo E5a + Galileo E6B
                    if ((galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()))
                        {
                            const std::string gal_signal("1B 5X E6");
                            rinex_obs_header(obsFile, galileo_ephemeris_iter->second, rx_time, gal_signal);
                            rinex_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navGalfilename);
                            log_rinex_nav(navGalFile, pvt_solver->galileo_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 105:  // Galileo E1B + Galileo E5b + Galileo E6B
                    if ((galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()))
                        {
                            const std::string gal_signal("1B 7X E6");
                            rinex_obs_header(obsFile, galileo_ephemeris_iter->second, rx_time, gal_signal);
                            rinex_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navGalfilename);
                            log_rinex_nav(navGalFile, pvt_solver->galileo_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 106:  // GPS L1 C/A + Galileo E1B + Galileo E6B
                    if ((galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()) and (gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()))
                        {
                            const std::string gal_signal("1B E6");
                            rinex_obs_header(obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gal_signal);
                            rinex_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navMixfilename);
                            log_rinex_nav(navMixFile, pvt_solver->gps_ephemeris_map, pvt_solver->galileo_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 107:  // GPS L1 C/A + Galileo E6B
                    if (gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend())
                        {
                            if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                                {
                                    // we have Galileo ephemeris, maybe from assistance
                                    const std::string gal_signal("E6");
                                    rinex_obs_header(obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gal_signal);
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
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 500:  // BDS B1I only
                    if (beidou_dnav_ephemeris_iter != pvt_solver->beidou_dnav_ephemeris_map.cend())
                        {
                            rinex_obs_header(obsFile, beidou_dnav_ephemeris_iter->second, rx_time, "B1");
                            rinex_nav_header(navFile, pvt_solver->beidou_dnav_iono, pvt_solver->beidou_dnav_utc_model);
                            output_navfilename.push_back(navfilename);
                            log_rinex_nav(navFile, pvt_solver->beidou_dnav_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }

                    break;
                case 501:  // BeiDou B1I + GPS L1 C/A
                    if ((gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()) and (beidou_dnav_ephemeris_iter != pvt_solver->beidou_dnav_ephemeris_map.cend()))
                        {
                            const std::string bds_signal("B1");
                            // rinex_obs_header(obsFile, gps_ephemeris_iter->second, beidou_dnav_ephemeris_iter->second, rx_time, bds_signal);
                            // rinex_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->beidou_dnav_iono, pvt_solver->beidou_dnav_utc_model);
                            d_rinex_header_written = true;  // do not write header anymore
                        }

                    break;
                case 502:  // BeiDou B1I + Galileo E1B
                    if ((galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()) and (beidou_dnav_ephemeris_iter != pvt_solver->beidou_dnav_ephemeris_map.cend()))
                        {
                            const std::string bds_signal("B1");
                            const std::string gal_signal("1B");
                            // rinex_obs_header(obsFile, galileo_ephemeris_iter->second, beidou_dnav_ephemeris_iter->second, rx_time, gal_signal, bds_signal);
                            // rinex_nav_header(navMixFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model, pvt_solver->beidou_dnav_iono, pvt_solver->beidou_dnav_utc_model);
                            d_rinex_header_written = true;  // do not write header anymore
                        }

                    break;
                case 503:  // BeiDou B1I + GLONASS L1 C/A
                case 504:  // BeiDou B1I + GPS L1 C/A + Galileo E1B
                case 505:  // BeiDou B1I + GPS L1 C/A + GLONASS L1 C/A + Galileo E1B
                case 506:  // BeiDou B1I + Beidou B3I
                    if (beidou_dnav_ephemeris_iter != pvt_solver->beidou_dnav_ephemeris_map.cend())
                        {
                            // rinex_obs_header(obsFile, beidou_dnav_ephemeris_iter->second, rx_time, "B1");
                            // rinex_nav_header(navFile, pvt_solver->beidou_dnav_iono, pvt_solver->beidou_dnav_utc_model);
                            // log_rinex_nav(navFile, pvt_solver->beidou_dnav_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }

                    break;
                case 600:  // BDS B3I only
                    if (beidou_dnav_ephemeris_iter != pvt_solver->beidou_dnav_ephemeris_map.cend())
                        {
                            rinex_obs_header(obsFile, beidou_dnav_ephemeris_iter->second, rx_time, "B3");
                            rinex_nav_header(navFile, pvt_solver->beidou_dnav_iono, pvt_solver->beidou_dnav_utc_model);
                            output_navfilename.push_back(navfilename);
                            log_rinex_nav(navFile, pvt_solver->beidou_dnav_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }

                    break;
                case 601:  // BeiDou B3I + GPS L2C
                case 602:  // BeiDou B3I + GLONASS L2 C/A
                case 603:  // BeiDou B3I + GPS L2C + GLONASS L2 C/A
                    if (beidou_dnav_ephemeris_iter != pvt_solver->beidou_dnav_ephemeris_map.cend())
                        {
                            rinex_obs_header(obsFile, beidou_dnav_ephemeris_iter->second, rx_time, "B3");
                            // rinex_nav_header(navFile, pvt_solver->beidou_dnav_iono, pvt_solver->beidou_dnav_utc_model);
                            d_rinex_header_written = true;  // do not write header anymore
                        }

                    break;
                case 1000:  // GPS L1 C/A + GPS L2C + GPS L5
                    if ((gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()) and
                        (gps_cnav_ephemeris_iter != pvt_solver->gps_cnav_ephemeris_map.cend()))
                        {
                            const std::string gps_signal("1C 2S L5");
                            rinex_obs_header(obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, rx_time, gps_signal);
                            rinex_nav_header(navFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second);
                            output_navfilename.push_back(navfilename);
                            log_rinex_nav(navFile, pvt_solver->gps_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                case 1001:  // GPS L1 C/A + Galileo E1B + GPS L2C + GPS L5 + Galileo E5a
                    if ((galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()) and
                        (gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()) and
                        (gps_cnav_ephemeris_iter != pvt_solver->gps_cnav_ephemeris_map.cend()))
                        {
                            const std::string gal_signal("1B 5X");
                            const std::string gps_signal("1C 2S L5");
                            rinex_obs_header(obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gps_signal, gal_signal);
                            rinex_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                            output_navfilename.push_back(navMixfilename);
                            log_rinex_nav(navMixFile, pvt_solver->gps_ephemeris_map, pvt_solver->galileo_ephemeris_map);
                            d_rinex_header_written = true;  // do not write header anymore
                        }
                    break;
                default:
                    break;
                }
        }
    if (d_rinex_header_written)  // The header is already written, we can now log the navigation message data
        {
            galileo_ephemeris_iter = pvt_solver->galileo_ephemeris_map.cbegin();
            gps_ephemeris_iter = pvt_solver->gps_ephemeris_map.cbegin();
            gps_cnav_ephemeris_iter = pvt_solver->gps_cnav_ephemeris_map.cbegin();
            glonass_gnav_ephemeris_iter = pvt_solver->glonass_gnav_ephemeris_map.cbegin();
            beidou_dnav_ephemeris_iter = pvt_solver->beidou_dnav_ephemeris_map.cbegin();

            // Log observables into the RINEX file
            if (flag_write_RINEX_obs_output)
                {
                    switch (type_of_rx)
                        {
                        case 1:  // GPS L1 C/A only
                            if (gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend())
                                {
                                    log_rinex_obs(obsFile, gps_ephemeris_iter->second, rx_time, gnss_observables_map);
                                    if (!d_rinex_header_updated and (pvt_solver->gps_utc_model.A0 != 0))
                                        {
                                            update_obs_header(obsFile, pvt_solver->gps_utc_model);
                                            update_nav_header(navFile, pvt_solver->gps_utc_model, pvt_solver->gps_iono, gps_ephemeris_iter->second);
                                            d_rinex_header_updated = true;
                                        }
                                }
                            break;
                        case 2:  // GPS L2C only
                        case 3:  // GPS L5
                            if (gps_cnav_ephemeris_iter != pvt_solver->gps_cnav_ephemeris_map.cend())
                                {
                                    log_rinex_obs(obsFile, gps_cnav_ephemeris_iter->second, rx_time, gnss_observables_map);
                                }
                            if (!d_rinex_header_updated and (pvt_solver->gps_cnav_utc_model.A0 != 0))
                                {
                                    update_obs_header(obsFile, pvt_solver->gps_cnav_utc_model);
                                    update_nav_header(navFile, pvt_solver->gps_cnav_utc_model, pvt_solver->gps_cnav_iono);
                                    d_rinex_header_updated = true;
                                }
                            break;
                        case 4:  // Galileo E1B only
                            if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                                {
                                    log_rinex_obs(obsFile, galileo_ephemeris_iter->second, rx_time, gnss_observables_map, "1B");
                                }
                            if (!d_rinex_header_updated and (pvt_solver->galileo_utc_model.A0 != 0))
                                {
                                    update_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                    update_obs_header(obsFile, pvt_solver->galileo_utc_model);
                                    d_rinex_header_updated = true;
                                }
                            break;
                        case 5:  // Galileo E5a only
                            if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                                {
                                    log_rinex_obs(obsFile, galileo_ephemeris_iter->second, rx_time, gnss_observables_map, "5X");
                                }
                            if (!d_rinex_header_updated and (pvt_solver->galileo_utc_model.A0 != 0))
                                {
                                    update_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                    update_obs_header(obsFile, pvt_solver->galileo_utc_model);
                                    d_rinex_header_updated = true;
                                }
                            break;
                        case 6:  // Galileo E5b only
                            if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                                {
                                    log_rinex_obs(obsFile, galileo_ephemeris_iter->second, rx_time, gnss_observables_map, "7X");
                                }
                            if (!d_rinex_header_updated and (pvt_solver->galileo_utc_model.A0 != 0))
                                {
                                    update_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                    update_obs_header(obsFile, pvt_solver->galileo_utc_model);
                                    d_rinex_header_updated = true;
                                }
                            break;
                        case 7:  // GPS L1 C/A + GPS L2C
                            if ((gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != pvt_solver->gps_cnav_ephemeris_map.cend()))
                                {
                                    log_rinex_obs(obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, rx_time, gnss_observables_map);
                                    if (!d_rinex_header_updated and (pvt_solver->gps_utc_model.A0 != 0))
                                        {
                                            update_obs_header(obsFile, pvt_solver->gps_utc_model);
                                            update_nav_header(navFile, pvt_solver->gps_utc_model, pvt_solver->gps_iono, gps_ephemeris_iter->second);
                                            d_rinex_header_updated = true;
                                        }
                                }
                            break;
                        case 8:  // L1+L5
                            if ((gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != pvt_solver->gps_cnav_ephemeris_map.cend()))
                                {
                                    log_rinex_obs(obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, rx_time, gnss_observables_map);
                                    if (!d_rinex_header_updated and ((pvt_solver->gps_cnav_utc_model.A0 != 0) or (pvt_solver->gps_utc_model.A0 != 0)))
                                        {
                                            if (pvt_solver->gps_cnav_utc_model.A0 != 0)
                                                {
                                                    update_obs_header(obsFile, pvt_solver->gps_cnav_utc_model);
                                                    update_nav_header(navFile, pvt_solver->gps_cnav_utc_model, pvt_solver->gps_cnav_iono);
                                                }
                                            else
                                                {
                                                    update_obs_header(obsFile, pvt_solver->gps_utc_model);
                                                    update_nav_header(navFile, pvt_solver->gps_utc_model, pvt_solver->gps_iono, gps_ephemeris_iter->second);
                                                }
                                            d_rinex_header_updated = true;
                                        }
                                }
                            break;
                        case 9:  // GPS L1 C/A + Galileo E1B
                            if ((galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()) and (gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()))
                                {
                                    log_rinex_obs(obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gnss_observables_map);
                                    if (!d_rinex_header_updated and (pvt_solver->gps_utc_model.A0 != 0))
                                        {
                                            update_obs_header(obsFile, pvt_solver->gps_utc_model);
                                            update_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                            d_rinex_header_updated = true;
                                        }
                                }
                            break;
                        case 13:  // L5+E5a
                            if ((gps_cnav_ephemeris_iter != pvt_solver->gps_cnav_ephemeris_map.cend()) and (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()))
                                {
                                    log_rinex_obs(obsFile, gps_cnav_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gnss_observables_map);
                                }
                            if (!d_rinex_header_updated and (pvt_solver->gps_cnav_utc_model.A0 != 0) and (pvt_solver->galileo_utc_model.A0 != 0))
                                {
                                    update_obs_header(obsFile, pvt_solver->gps_cnav_utc_model);
                                    update_nav_header(navMixFile, pvt_solver->gps_cnav_utc_model, pvt_solver->gps_cnav_iono, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                    d_rinex_header_updated = true;  // do not write header anymore
                                }
                            break;
                        case 14:  // Galileo E1B + Galileo E5a
                            if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                                {
                                    log_rinex_obs(obsFile, galileo_ephemeris_iter->second, rx_time, gnss_observables_map, "1B 5X");
                                }
                            if (!d_rinex_header_updated and (pvt_solver->galileo_utc_model.A0 != 0))
                                {
                                    update_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                    update_obs_header(obsFile, pvt_solver->galileo_utc_model);
                                    d_rinex_header_updated = true;
                                }
                            break;
                        case 15:  // Galileo E1B + Galileo E5b
                            if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                                {
                                    log_rinex_obs(obsFile, galileo_ephemeris_iter->second, rx_time, gnss_observables_map, "1B 7X");
                                }
                            if (!d_rinex_header_updated and (pvt_solver->galileo_utc_model.A0 != 0))
                                {
                                    update_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                    update_obs_header(obsFile, pvt_solver->galileo_utc_model);
                                    d_rinex_header_updated = true;
                                }
                            break;
                        case 19:  // Galileo E5a + Galileo E5b
                            if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                                {
                                    log_rinex_obs(obsFile, galileo_ephemeris_iter->second, rx_time, gnss_observables_map, "5X 7X");
                                }
                            if (!d_rinex_header_updated and (pvt_solver->galileo_utc_model.A0 != 0))
                                {
                                    update_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                    update_obs_header(obsFile, pvt_solver->galileo_utc_model);
                                    d_rinex_header_updated = true;
                                }
                            break;
                        case 23:  // GLONASS L1 C/A only
                            if (glonass_gnav_ephemeris_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                {
                                    log_rinex_obs(obsFile, glonass_gnav_ephemeris_iter->second, rx_time, gnss_observables_map, "1C");
                                }
                            if (!d_rinex_header_updated and (pvt_solver->glonass_gnav_utc_model.d_tau_c != 0))
                                {
                                    update_nav_header(navGloFile, pvt_solver->glonass_gnav_utc_model, pvt_solver->glonass_gnav_almanac);
                                    update_obs_header(obsFile, pvt_solver->glonass_gnav_utc_model);
                                    d_rinex_header_updated = true;
                                }
                            break;
                        case 24:  // GLONASS L2 C/A only
                            if (glonass_gnav_ephemeris_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                {
                                    log_rinex_obs(obsFile, glonass_gnav_ephemeris_iter->second, rx_time, gnss_observables_map, "2C");
                                }
                            if (!d_rinex_header_updated and (pvt_solver->glonass_gnav_utc_model.d_tau_c != 0))
                                {
                                    update_nav_header(navGloFile, pvt_solver->glonass_gnav_utc_model, pvt_solver->glonass_gnav_almanac);
                                    update_obs_header(obsFile, pvt_solver->glonass_gnav_utc_model);
                                    d_rinex_header_updated = true;
                                }
                            break;
                        case 25:  // GLONASS L1 C/A + GLONASS L2 C/A
                            if (glonass_gnav_ephemeris_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                {
                                    log_rinex_obs(obsFile, glonass_gnav_ephemeris_iter->second, rx_time, gnss_observables_map, "1C 2C");
                                }
                            if (!d_rinex_header_updated and (pvt_solver->glonass_gnav_utc_model.d_tau_c != 0))
                                {
                                    update_nav_header(navMixFile, pvt_solver->glonass_gnav_utc_model, pvt_solver->glonass_gnav_almanac);
                                    update_obs_header(obsFile, pvt_solver->glonass_gnav_utc_model);
                                    d_rinex_header_updated = true;
                                }
                            break;
                        case 26:  // GPS L1 C/A + GLONASS L1 C/A
                            if ((glonass_gnav_ephemeris_iter != pvt_solver->glonass_gnav_ephemeris_map.cend()) and (gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()))
                                {
                                    log_rinex_obs(obsFile, gps_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, rx_time, gnss_observables_map);
                                    if (!d_rinex_header_updated and (pvt_solver->gps_utc_model.A0 != 0))
                                        {
                                            update_obs_header(obsFile, pvt_solver->gps_utc_model);
                                            update_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->glonass_gnav_utc_model, pvt_solver->glonass_gnav_almanac);
                                            d_rinex_header_updated = true;  // do not write header anymore
                                        }
                                }
                            break;
                        case 27:  // Galileo E1B + GLONASS L1 C/A
                            if ((glonass_gnav_ephemeris_iter != pvt_solver->glonass_gnav_ephemeris_map.cend()) and (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()))
                                {
                                    log_rinex_obs(obsFile, galileo_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, rx_time, gnss_observables_map);
                                }
                            if (!d_rinex_header_updated and (pvt_solver->galileo_utc_model.A0 != 0))
                                {
                                    update_obs_header(obsFile, pvt_solver->galileo_utc_model);
                                    update_nav_header(navMixFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model, pvt_solver->glonass_gnav_utc_model, pvt_solver->glonass_gnav_almanac);
                                    d_rinex_header_updated = true;  // do not write header anymore
                                }
                            break;
                        case 28:  // GPS L2C + GLONASS L1 C/A
                            if ((glonass_gnav_ephemeris_iter != pvt_solver->glonass_gnav_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != pvt_solver->gps_cnav_ephemeris_map.cend()))
                                {
                                    log_rinex_obs(obsFile, gps_cnav_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, rx_time, gnss_observables_map);
                                }
                            if (!d_rinex_header_updated and (pvt_solver->gps_cnav_utc_model.A0 != 0))
                                {
                                    update_obs_header(obsFile, pvt_solver->gps_cnav_utc_model);
                                    update_nav_header(navMixFile, pvt_solver->gps_cnav_iono, pvt_solver->gps_cnav_utc_model, pvt_solver->glonass_gnav_utc_model, pvt_solver->glonass_gnav_almanac);
                                    d_rinex_header_updated = true;  // do not write header anymore
                                }
                            break;
                        case 29:  // GPS L1 C/A + GLONASS L2 C/A
                            if ((glonass_gnav_ephemeris_iter != pvt_solver->glonass_gnav_ephemeris_map.cend()) and (gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()))
                                {
                                    log_rinex_obs(obsFile, gps_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, rx_time, gnss_observables_map);
                                    if (!d_rinex_header_updated and (pvt_solver->gps_utc_model.A0 != 0))
                                        {
                                            update_obs_header(obsFile, pvt_solver->gps_utc_model);
                                            update_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->glonass_gnav_utc_model, pvt_solver->glonass_gnav_almanac);
                                            d_rinex_header_updated = true;  // do not write header anymore
                                        }
                                }
                            break;
                        case 30:  // Galileo E1B + GLONASS L2 C/A
                            if ((glonass_gnav_ephemeris_iter != pvt_solver->glonass_gnav_ephemeris_map.cend()) and (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()))
                                {
                                    log_rinex_obs(obsFile, galileo_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, rx_time, gnss_observables_map);
                                }
                            if (!d_rinex_header_updated and (pvt_solver->galileo_utc_model.A0 != 0))
                                {
                                    update_obs_header(obsFile, pvt_solver->galileo_utc_model);
                                    update_nav_header(navMixFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model, pvt_solver->glonass_gnav_utc_model, pvt_solver->glonass_gnav_almanac);
                                    d_rinex_header_updated = true;  // do not write header anymore
                                }
                            break;
                        case 31:  // GPS L2C + GLONASS L2 C/A
                            if ((glonass_gnav_ephemeris_iter != pvt_solver->glonass_gnav_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != pvt_solver->gps_cnav_ephemeris_map.cend()))
                                {
                                    log_rinex_obs(obsFile, gps_cnav_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, rx_time, gnss_observables_map);
                                }
                            if (!d_rinex_header_updated and (pvt_solver->gps_cnav_utc_model.A0 != 0))
                                {
                                    update_obs_header(obsFile, pvt_solver->gps_cnav_utc_model);
                                    update_nav_header(navMixFile, pvt_solver->gps_cnav_iono, pvt_solver->gps_cnav_utc_model, pvt_solver->glonass_gnav_utc_model, pvt_solver->glonass_gnav_almanac);
                                    d_rinex_header_updated = true;  // do not write header anymore
                                }
                            break;
                        case 32:  // L1+E1+L5+E5a
                            if ((gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != pvt_solver->gps_cnav_ephemeris_map.cend()) and (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()))
                                {
                                    log_rinex_obs(obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gnss_observables_map);
                                    if (!d_rinex_header_updated and ((pvt_solver->gps_cnav_utc_model.A0 != 0) or (pvt_solver->gps_utc_model.A0 != 0)) and (pvt_solver->galileo_utc_model.A0 != 0))
                                        {
                                            if (pvt_solver->gps_cnav_utc_model.A0 != 0)
                                                {
                                                    update_obs_header(obsFile, pvt_solver->gps_cnav_utc_model);
                                                    update_nav_header(navMixFile, pvt_solver->gps_cnav_utc_model, pvt_solver->gps_cnav_iono, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                                }
                                            else
                                                {
                                                    update_obs_header(obsFile, pvt_solver->gps_utc_model);
                                                    update_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                                }
                                            d_rinex_header_updated = true;  // do not write header anymore
                                        }
                                }
                            break;
                        case 33:  // L1+E1+E5a
                            if ((gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()) and (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()))
                                {
                                    log_rinex_obs(obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gnss_observables_map);
                                    if (!d_rinex_header_updated and (pvt_solver->gps_utc_model.A0 != 0) and (pvt_solver->galileo_utc_model.A0 != 0))
                                        {
                                            update_obs_header(obsFile, pvt_solver->gps_utc_model);
                                            update_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                            d_rinex_header_updated = true;  // do not write header anymore
                                        }
                                }
                            break;
                        case 100:  // Galileo E6B
                            if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                                {
                                    log_rinex_obs(obsFile, galileo_ephemeris_iter->second, rx_time, gnss_observables_map, "E6");
                                }
                            if (!d_rinex_header_updated and (pvt_solver->galileo_utc_model.A0 != 0))
                                {
                                    update_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                    update_obs_header(obsFile, pvt_solver->galileo_utc_model);
                                    d_rinex_header_updated = true;
                                }
                            break;
                        case 101:  // Galileo E1B + Galileo E6B
                            if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                                {
                                    log_rinex_obs(obsFile, galileo_ephemeris_iter->second, rx_time, gnss_observables_map, "1B E6");
                                }
                            if (!d_rinex_header_updated and (pvt_solver->galileo_utc_model.A0 != 0))
                                {
                                    update_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                    update_obs_header(obsFile, pvt_solver->galileo_utc_model);
                                    d_rinex_header_updated = true;
                                }
                            break;
                        case 102:  // Galileo E5a + Galileo E6B
                            if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                                {
                                    log_rinex_obs(obsFile, galileo_ephemeris_iter->second, rx_time, gnss_observables_map, "5X E6");
                                }
                            if (!d_rinex_header_updated and (pvt_solver->galileo_utc_model.A0 != 0))
                                {
                                    update_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                    update_obs_header(obsFile, pvt_solver->galileo_utc_model);
                                    d_rinex_header_updated = true;
                                }
                            break;
                        case 103:  // Galileo E5b + Galileo E6B
                            if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                                {
                                    log_rinex_obs(obsFile, galileo_ephemeris_iter->second, rx_time, gnss_observables_map, "7X E6");
                                }
                            if (!d_rinex_header_updated and (pvt_solver->galileo_utc_model.A0 != 0))
                                {
                                    update_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                    update_obs_header(obsFile, pvt_solver->galileo_utc_model);
                                    d_rinex_header_updated = true;
                                }
                            break;
                        case 104:  // Galileo E1B + Galileo E5a + Galileo E6B
                            if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                                {
                                    log_rinex_obs(obsFile, galileo_ephemeris_iter->second, rx_time, gnss_observables_map, "1B 5X E6");
                                }
                            if (!d_rinex_header_updated and (pvt_solver->galileo_utc_model.A0 != 0))
                                {
                                    update_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                    update_obs_header(obsFile, pvt_solver->galileo_utc_model);
                                    d_rinex_header_updated = true;
                                }
                            break;
                        case 105:  // Galileo E1B + Galileo E5b + Galileo E6B
                            if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                                {
                                    log_rinex_obs(obsFile, galileo_ephemeris_iter->second, rx_time, gnss_observables_map, "1B 7X E6");
                                }
                            if (!d_rinex_header_updated and (pvt_solver->galileo_utc_model.A0 != 0))
                                {
                                    update_nav_header(navGalFile, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                    update_obs_header(obsFile, pvt_solver->galileo_utc_model);
                                    d_rinex_header_updated = true;
                                }
                            break;
                        case 106:  // GPS L1 C/A + Galileo E1B + Galileo E6B
                            if ((galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()) and (gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()))
                                {
                                    log_rinex_obs(obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gnss_observables_map);
                                    if (!d_rinex_header_updated and (pvt_solver->gps_utc_model.A0 != 0))
                                        {
                                            update_obs_header(obsFile, pvt_solver->gps_utc_model);
                                            update_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                            d_rinex_header_updated = true;
                                        }
                                }
                            break;
                        case 107:
                            if (gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend())
                                {
                                    if (galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend())
                                        {
                                            // we have Galileo ephemeris, maybe from assistance
                                            log_rinex_obs(obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gnss_observables_map);
                                            if (!d_rinex_header_updated and (pvt_solver->gps_utc_model.A0 != 0))
                                                {
                                                    update_obs_header(obsFile, pvt_solver->gps_utc_model);
                                                    update_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                                    d_rinex_header_updated = true;
                                                }
                                        }
                                    else
                                        {
                                            // we do not have galileo ephemeris, print only GPS data
                                            log_rinex_obs(obsFile, gps_ephemeris_iter->second, rx_time, gnss_observables_map);
                                            if (!d_rinex_header_updated and (pvt_solver->gps_utc_model.A0 != 0))
                                                {
                                                    update_obs_header(obsFile, pvt_solver->gps_utc_model);
                                                    update_nav_header(navFile, pvt_solver->gps_utc_model, pvt_solver->gps_iono, gps_ephemeris_iter->second);
                                                    d_rinex_header_updated = true;
                                                }
                                        }
                                }

                            break;
                        case 500:  // BDS B1I only
                            if (beidou_dnav_ephemeris_iter != pvt_solver->beidou_dnav_ephemeris_map.cend())
                                {
                                    log_rinex_obs(obsFile, beidou_dnav_ephemeris_iter->second, rx_time, gnss_observables_map, "B1");
                                }
                            if (!d_rinex_header_updated and (pvt_solver->beidou_dnav_utc_model.A0_UTC != 0))
                                {
                                    update_obs_header(obsFile, pvt_solver->beidou_dnav_utc_model);
                                    update_nav_header(navFile, pvt_solver->beidou_dnav_utc_model, pvt_solver->beidou_dnav_iono);
                                    d_rinex_header_updated = true;
                                }
                            break;
                        case 600:  // BDS B3I only
                            if (beidou_dnav_ephemeris_iter != pvt_solver->beidou_dnav_ephemeris_map.cend())
                                {
                                    log_rinex_obs(obsFile, beidou_dnav_ephemeris_iter->second, rx_time, gnss_observables_map, "B3");
                                }
                            if (!d_rinex_header_updated and (pvt_solver->beidou_dnav_utc_model.A0_UTC != 0))
                                {
                                    update_obs_header(obsFile, pvt_solver->beidou_dnav_utc_model);
                                    update_nav_header(navFile, pvt_solver->beidou_dnav_utc_model, pvt_solver->beidou_dnav_iono);
                                    d_rinex_header_updated = true;
                                }
                            break;
                        case 1000:  // GPS L1 C/A + GPS L2C + GPS L5
                            if ((gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()) and
                                (gps_cnav_ephemeris_iter != pvt_solver->gps_cnav_ephemeris_map.cend()))
                                {
                                    log_rinex_obs(obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, rx_time, gnss_observables_map, true);
                                }
                            if (!d_rinex_header_updated and (pvt_solver->gps_utc_model.A0 != 0))
                                {
                                    update_obs_header(obsFile, pvt_solver->gps_utc_model);
                                    update_nav_header(navFile, pvt_solver->gps_utc_model, pvt_solver->gps_iono, gps_ephemeris_iter->second);
                                    d_rinex_header_updated = true;
                                }
                            break;
                        case 1001:  // GPS L1 C/A + Galileo E1B + GPS L2C + GPS L5 + Galileo E5a
                            if ((galileo_ephemeris_iter != pvt_solver->galileo_ephemeris_map.cend()) and
                                (gps_ephemeris_iter != pvt_solver->gps_ephemeris_map.cend()) and
                                (gps_cnav_ephemeris_iter != pvt_solver->gps_cnav_ephemeris_map.cend()))
                                {
                                    log_rinex_obs(obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, galileo_ephemeris_iter->second, rx_time, gnss_observables_map, true);
                                }
                            if (!d_rinex_header_updated and (pvt_solver->gps_utc_model.A0 != 0) and (pvt_solver->galileo_utc_model.A0 != 0))
                                {
                                    update_obs_header(obsFile, pvt_solver->gps_utc_model);
                                    update_nav_header(navMixFile, pvt_solver->gps_iono, pvt_solver->gps_utc_model, gps_ephemeris_iter->second, pvt_solver->galileo_iono, pvt_solver->galileo_utc_model);
                                    d_rinex_header_updated = true;
                                }
                            break;
                        default:
                            break;
                        }
                }
        }
}


void Rinex_Printer::log_rinex_nav_gps_cnav(int type_of_rx, const std::map<int32_t, Gps_CNAV_Ephemeris>& new_cnav_eph)
{
    std::map<int32_t, Galileo_Ephemeris> new_gal_eph;
    std::map<int32_t, Glonass_Gnav_Ephemeris> new_glo_eph;
    switch (type_of_rx)
        {
        case 2:  // GPS L2C only
        case 3:  // GPS L5 only
        case 7:  // GPS L1 C/A + GPS L2C
            log_rinex_nav(navFile, new_cnav_eph);
            break;
        case 13:  // L5+E5a
            log_rinex_nav(navMixFile, new_cnav_eph, new_gal_eph);
            break;
        case 28:  // GPS L2C + GLONASS L1 C/A
        case 31:  // GPS L2C + GLONASS L2 C/A
            log_rinex_nav(navMixFile, new_cnav_eph, new_glo_eph);
            break;
        default:
            break;
        }
}


void Rinex_Printer::log_rinex_nav_gps_nav(int type_of_rx, const std::map<int32_t, Gps_Ephemeris>& new_eph)
{
    std::map<int32_t, Galileo_Ephemeris> new_gal_eph;
    std::map<int32_t, Glonass_Gnav_Ephemeris> new_glo_eph;
    switch (type_of_rx)
        {
        case 1:  // GPS L1 C/A only
        case 8:  // L1+L5
            log_rinex_nav(navFile, new_eph);
            break;
        case 9:   // GPS L1 C/A + Galileo E1B
        case 10:  // GPS L1 C/A + Galileo E5a
        case 11:  // GPS L1 C/A + Galileo E5b
            log_rinex_nav(navMixFile, new_eph, new_gal_eph);
            break;
        case 26:  // GPS L1 C/A + GLONASS L1 C/A
            if (d_version == 3)
                {
                    log_rinex_nav(navMixFile, new_eph, new_glo_eph);
                }
            if (d_version == 2)
                {
                    log_rinex_nav(navFile, new_glo_eph);
                }
            break;
        case 29:  // GPS L1 C/A + GLONASS L2 C/A
            if (d_version == 3)
                {
                    log_rinex_nav(navMixFile, new_eph, new_glo_eph);
                }
            if (d_version == 2)
                {
                    log_rinex_nav(navFile, new_eph);
                }
            break;
        case 32:  // L1+E1+L5+E5a
        case 33:  // L1+E1+E5a
            log_rinex_nav(navMixFile, new_eph, new_gal_eph);
            break;
        case 106:  // GPS L1 C/A + Galileo E1B + Galileo E6B
            log_rinex_nav(navMixFile, new_eph, new_gal_eph);
            break;
        case 107:  // GPS L1 C/A + Galileo E6B
            if (navMixFile.tellp() != 0)
                {
                    log_rinex_nav(navMixFile, new_eph, new_gal_eph);
                }
            else
                {
                    log_rinex_nav(navFile, new_eph);
                }
            break;
        case 1000:  // L1+L2+L5
            log_rinex_nav(navFile, new_eph);
            break;
        case 1001:  // L1+E1+L2+L5+E5a
            log_rinex_nav(navMixFile, new_eph, new_gal_eph);
            break;
        default:
            break;
        }
}


void Rinex_Printer::log_rinex_nav_gal_nav(int type_of_rx, const std::map<int32_t, Galileo_Ephemeris>& new_gal_eph)
{
    std::map<int32_t, Glonass_Gnav_Ephemeris> new_glo_eph;
    std::map<int32_t, Gps_CNAV_Ephemeris> new_cnav_eph;
    std::map<int32_t, Gps_Ephemeris> new_eph;
    switch (type_of_rx)
        {
        case 4:  // Galileo E1B only
        case 5:  // Galileo E5a only
        case 6:  // Galileo E5b only
            log_rinex_nav(navGalFile, new_gal_eph);
            break;
        case 9:   // GPS L1 C/A + Galileo E1B
        case 10:  // GPS L1 C/A + Galileo E5a
        case 11:  // GPS L1 C/A + Galileo E5b
            log_rinex_nav(navMixFile, new_eph, new_gal_eph);
            break;
        case 13:  // L5+E5a
            log_rinex_nav(navMixFile, new_cnav_eph, new_gal_eph);
            break;
        case 15:  // Galileo E1B + Galileo E5b
            log_rinex_nav(navGalFile, new_gal_eph);
            break;
        case 19:  // Galileo E1B + Galileo E5b
            log_rinex_nav(navGalFile, new_gal_eph);
            break;
        case 27:  // Galileo E1B + GLONASS L1 C/A
        case 30:  // Galileo E1B + GLONASS L2 C/A
            log_rinex_nav(navMixFile, new_gal_eph, new_glo_eph);
            break;
        case 32:  // L1+E1+L5+E5a
        case 33:  // L1+E1+E5a
            log_rinex_nav(navMixFile, new_eph, new_gal_eph);
            break;
        case 100:  // E6B
        case 101:  // E1B + E6B
        case 102:  // Galileo E5a + Galileo E6B
        case 103:  // Galileo E5b + Galileo E6B
        case 104:  // Galileo E1B + Galileo E5a + Galileo E6B
        case 105:  // Galileo E1B + Galileo E5b + Galileo E6B
            log_rinex_nav(navGalFile, new_gal_eph);
            break;
        case 106:  // GPS L1 C/A + Galileo E1B + Galileo E6B
            log_rinex_nav(navMixFile, new_eph, new_gal_eph);
            break;
        case 107:  // GPS L1 C/A + Galileo E6B
            if (navMixFile.tellp() != 0)
                {
                    log_rinex_nav(navMixFile, new_eph, new_gal_eph);
                }
            break;
        case 1001:  // L1+E1+L2+L5+E5a
            log_rinex_nav(navMixFile, new_eph, new_gal_eph);
            break;
        default:
            break;
        }
}


void Rinex_Printer::log_rinex_nav_glo_gnav(int type_of_rx, const std::map<int32_t, Glonass_Gnav_Ephemeris>& new_glo_eph)
{
    std::map<int32_t, Galileo_Ephemeris> new_gal_eph;
    std::map<int32_t, Gps_CNAV_Ephemeris> new_cnav_eph;
    std::map<int32_t, Gps_Ephemeris> new_eph;
    switch (type_of_rx)
        {
        case 23:  // GLONASS L1 C/A
        case 24:  // GLONASS L2 C/A
        case 25:  // GLONASS L1 C/A + GLONASS L2 C/A
            log_rinex_nav(navGloFile, new_glo_eph);
            break;
        case 26:  // GPS L1 C/A + GLONASS L1 C/A
            if (d_version == 3)
                {
                    log_rinex_nav(navMixFile, new_eph, new_glo_eph);
                }
            if (d_version == 2)
                {
                    log_rinex_nav(navGloFile, new_glo_eph);
                }
            break;
        case 27:  // Galileo E1B + GLONASS L1 C/A
            log_rinex_nav(navMixFile, new_gal_eph, new_glo_eph);
            break;
        case 28:  // GPS L2C + GLONASS L1 C/A
            log_rinex_nav(navMixFile, new_cnav_eph, new_glo_eph);
            break;
        case 29:  // GPS L1 C/A + GLONASS L2 C/A
            if (d_version == 3)
                {
                    log_rinex_nav(navMixFile, new_eph, new_glo_eph);
                }
            if (d_version == 2)
                {
                    log_rinex_nav(navGloFile, new_glo_eph);
                }
            break;
        case 30:  // Galileo E1B + GLONASS L2 C/A
            log_rinex_nav(navMixFile, new_gal_eph, new_glo_eph);
            break;
        case 31:  // GPS L2C + GLONASS L2 C/A
            log_rinex_nav(navMixFile, new_cnav_eph, new_glo_eph);
            break;
        default:
            break;
        }
}


void Rinex_Printer::log_rinex_nav_bds_dnav(int type_of_rx, const std::map<int32_t, Beidou_Dnav_Ephemeris>& new_bds_eph)
{
    switch (type_of_rx)
        {
        case 500:  // BDS B1I only
        case 600:  // BDS B3I only
            log_rinex_nav(navFile, new_bds_eph);
            break;
        default:
            break;
        }
}


void Rinex_Printer::lengthCheck(const std::string& line) const
{
    if (line.length() != 80)
        {
            LOG(ERROR) << "Bad defined RINEX line: "
                       << line.length() << " characters (must be 80)" << '\n'
                       << line << '\n'
                       << "----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|\n";
        }
}


std::string Rinex_Printer::createFilename(const std::string& type, const std::string& base_name) const
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
    fileType.insert(std::pair<std::string, std::string>("RINEX_FILE_TYPE_BDS_NAV", "F"));    // G - GLONASS navigation file.

    const boost::posix_time::ptime pt = boost::posix_time::second_clock::local_time();
    const tm pt_tm = boost::posix_time::to_tm(pt);
    const int32_t local_hour = pt_tm.tm_hour;
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

    const std::string hourTag = Hmap[strm.str()];

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

    const std::string typeOfFile = fileType[type];
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

    line += Rinex_Printer::leftJustify(username, 20);
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
            line += Rinex_Printer::rightJustify(std::to_string(day), 2);
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
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += d_stringVersion;
    line += std::string(11, ' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4, ' ');
    line += std::string("R: GLONASS");
    line += std::string(10, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GLONASS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction
    if (d_version == 3)
        {
            line.clear();
            line += std::string("GLUT");
            line += std::string(1, ' ');
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_c, 16, 2), 17);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
            line += Rinex_Printer::rightJustify(std::to_string(0.0), 7);
            line += Rinex_Printer::rightJustify(std::to_string(0.0), 5);
            line += std::string(10, ' ');
            line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- Line system time correction 2
            line.clear();
            line += std::string("GLGP");
            line += std::string(1, ' ');
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_gps, 16, 2), 17);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
            line += Rinex_Printer::rightJustify(std::to_string(0.0), 7);
            line += Rinex_Printer::rightJustify(std::to_string(0.0), 5);
            line += std::string(10, ' ');
            line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
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
            line += Rinex_Printer::rightJustify(year, 6);
            line += Rinex_Printer::rightJustify(month, 6);
            line += Rinex_Printer::rightJustify(day, 6);
            line += std::string(3, ' ');
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_c, 19, 2), 19);
            line += std::string(20, ' ');
            line += Rinex_Printer::leftJustify("CORR TO SYSTEM TIME", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
        }

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Gps_Iono& gps_iono, const Gps_Utc_Model& gps_utc_model, const Gps_Ephemeris& eph, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac)
{
    if (glonass_gnav_almanac.i_satellite_freq_channel)
        {
        }  // Avoid compiler warning
    std::string line;
    d_stringVersion = "3.02";
    d_version = 3;

    // -------- Line 1
    line = std::string(5, ' ');
    line += d_stringVersion;
    line += std::string(11, ' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4, ' ');
    line += std::string("M: MIXED");
    line += std::string(12, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 1
    line.clear();
    line += std::string("GPSA");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction 1
    line.clear();
    line += std::string("GLUT");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_c, 16, 2), 17);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
    line += Rinex_Printer::rightJustify(std::to_string(0.0), 7);
    line += Rinex_Printer::rightJustify(std::to_string(0.0), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction 2
    line.clear();
    line += std::string("GLGP");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_gps, 16, 2), 17);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
    line += Rinex_Printer::rightJustify(std::to_string(0.0), 7);
    line += Rinex_Printer::rightJustify(std::to_string(0.0), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction 3
    line.clear();
    line += std::string("GPUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.A0, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.A1, 15, 2), 16);
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.tot), 7);

    if (d_pre_2009_file == false)
        {
            if (eph.WN < 512)
                {
                    if (gps_utc_model.WN_T == 0)
                        {
                            line += Rinex_Printer::rightJustify(std::to_string(eph.WN + 2048), 5);  // valid from 2019 to 2029
                        }
                    else
                        {
                            line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_T + (eph.WN / 256) * 256 + 2048), 5);  // valid from 2019 to 2029
                        }
                }
            else
                {
                    if (gps_utc_model.WN_T == 0)
                        {
                            line += Rinex_Printer::rightJustify(std::to_string(eph.WN + 1024), 5);  // valid from 2009 to 2019
                        }
                    else
                        {
                            line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_T + (eph.WN / 256) * 256 + 1024), 5);  // valid from 2009 to 2019
                        }
                }
        }
    else
        {
            if (gps_utc_model.WN_T == 0)
                {
                    line += Rinex_Printer::rightJustify(std::to_string(eph.WN + 1024), 5);  // valid from 2009 to 2019
                }
            else
                {
                    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_T + (eph.WN / 256) * 256 + 1024), 5);  // valid from 2009 to 2019
                }
        }
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 6 leap seconds
    // For leap second information, see https://endruntechnologies.com/support/leap-seconds
    line.clear();
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DeltaT_LS), 6);
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DeltaT_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DN), 6);
    line += std::string(36, ' ');
    line += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Gps_CNAV_Iono& gps_iono, const Gps_CNAV_Utc_Model& gps_utc_model, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac)
{
    if (glonass_gnav_almanac.i_satellite_freq_channel)
        {
        }  // Avoid compiler warning
    std::string line;
    d_stringVersion = "3.02";
    d_version = 3;

    // -------- Line 1
    line = std::string(5, ' ');
    line += d_stringVersion;
    line += std::string(11, ' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4, ' ');
    line += std::string("M: MIXED");
    line += std::string(12, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 1
    line.clear();
    line += std::string("GPSA");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction 1
    line.clear();
    line += std::string("GLUT");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_c, 16, 2), 17);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
    line += Rinex_Printer::rightJustify(std::to_string(0.0), 7);
    line += Rinex_Printer::rightJustify(std::to_string(0.0), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction 2
    line.clear();
    line += std::string("GLGP");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_gps, 16, 2), 17);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
    line += Rinex_Printer::rightJustify(std::to_string(0.0), 7);
    line += Rinex_Printer::rightJustify(std::to_string(0.0), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction 3
    line.clear();
    line += std::string("GPUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.A0, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.A1, 15, 2), 16);
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.tot), 7);
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_T), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 6 leap seconds
    // For leap second information, see https://endruntechnologies.com/support/leap-seconds
    line.clear();
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DeltaT_LS), 6);
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DeltaT_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DN), 6);
    line += std::string(36, ' ');
    line += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac) const
{
    if (glonass_gnav_almanac.i_satellite_freq_channel)
        {
        }  // Avoid compiler warning
    // Avoid compiler warning, there is not time system correction between Galileo and GLONASS
    if (galileo_utc_model.A_0G > 0.0)
        {
        }
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += d_stringVersion;
    line += std::string(11, ' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4, ' ');
    line += std::string("M: MIXED");
    line += std::string(12, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 1
    line.clear();
    line += std::string("GAL ");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai2, 10, 2), 12);
    const double zero = 0.0;
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(zero, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction
    line.clear();
    line += std::string("GAUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A0, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A1, 15, 2), 16);
    line += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.tot), 7);
    line += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.WNot), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction 1
    line.clear();
    line += std::string("GLUT");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_c, 16, 2), 17);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
    line += Rinex_Printer::rightJustify(std::to_string(0.0), 7);
    line += Rinex_Printer::rightJustify(std::to_string(0.0), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 6 leap seconds
    // For leap second information, see https://endruntechnologies.com/support/leap-seconds
    line.clear();
    line += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.Delta_tLS), 6);
    line += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.Delta_tLSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.WN_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.DN), 6);
    line += std::string(36, ' ');
    line += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Galileo_Iono& iono, const Galileo_Utc_Model& utc_model) const
{
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += d_stringVersion;
    line += std::string(11, ' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4, ' ');
    line += std::string("E: GALILEO");
    line += std::string(10, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GALILEO NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 1
    line.clear();
    line += std::string("GAL ");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.ai0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.ai1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.ai2, 10, 2), 12);
    const double zero = 0.0;
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(zero, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction
    line.clear();
    line += std::string("GAUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A0, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A1, 15, 2), 16);
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.tot), 7);
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.WNot), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction 2
    line.clear();
    line += std::string("GPGA");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A_0G, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A_1G, 15, 2), 16);
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.t_0G), 7);
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_0G), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 6 leap seconds
    // For leap second information, see https://endruntechnologies.com/support/leap-seconds
    line.clear();
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.Delta_tLS), 6);
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.Delta_tLSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.DN), 6);
    line += std::string(36, ' ');
    line += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Gps_CNAV_Iono& iono, const Gps_CNAV_Utc_Model& utc_model) const
{
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += d_stringVersion;
    line += std::string(11, ' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4, ' ');
    line += std::string("G: GPS");
    line += std::string(14, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::leftJustify("GPS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 1
    line.clear();
    line += std::string("GPSA");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 2
    line.clear();
    line += std::string("GPSB");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 5 system time correction
    line.clear();
    line += std::string("GPUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A0, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A1, 15, 2), 16);
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.tot), 7);
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_T), 5);
    /*  if ( SBAS )
    {
      line += string(1, ' ');
      line += leftJustify(asString(tot_SBAS),5);
      line += string(1, ' ');
      line += leftJustify(asString(d_WN_T_SBAS),2);
      line += string(1, ' ');
           }
    else
     */
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 6 leap seconds
    // For leap second information, see https://endruntechnologies.com/support/leap-seconds
    line.clear();
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LS), 6);
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.DN), 6);
    line += std::string(36, ' ');
    line += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Gps_CNAV_Iono& iono, const Gps_CNAV_Utc_Model& utc_model, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model) const
{
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += d_stringVersion;
    line += std::string(11, ' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4, ' ');
    line += std::string("M: MIXED");
    line += std::string(12, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 1
    line.clear();
    line += std::string("GAL ");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai2, 10, 2), 12);
    const double zero = 0.0;
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(zero, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 2
    line.clear();
    line += std::string("GPSA");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 3
    line.clear();
    line += std::string("GPSB");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction
    line.clear();
    line += std::string("GAUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A0, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A1, 15, 2), 16);
    line += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.tot), 7);
    line += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.WNot), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction 2
    line.clear();
    line += std::string("GPUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A0, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A1, 15, 2), 16);
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.tot), 7);
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_T), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 6 leap seconds
    // For leap second information, see https://endruntechnologies.com/support/leap-seconds
    line.clear();
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LS), 6);
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.DN), 6);
    line += std::string(36, ' ');
    line += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
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
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::leftJustify("GPS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 1
    line.clear();
    if (d_version == 2)
        {
            line += std::string(2, ' ');
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha0, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha1, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha2, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha3, 10, 2), 12);
            line += std::string(10, ' ');
            line += Rinex_Printer::leftJustify("ION ALPHA", 20);
        }
    if (d_version == 3)
        {
            line += std::string("GPSA");
            line += std::string(1, ' ');
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha0, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha1, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha2, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha3, 10, 2), 12);
            line += std::string(7, ' ');
            line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
        }
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 2
    line.clear();
    if (d_version == 2)
        {
            line += std::string(2, ' ');
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta0, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta1, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta2, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta3, 10, 2), 12);
            line += std::string(10, ' ');
            line += Rinex_Printer::leftJustify("ION BETA", 20);
        }

    if (d_version == 3)
        {
            line += std::string("GPSB");
            line += std::string(1, ' ');
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta0, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta1, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta2, 10, 2), 12);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta3, 10, 2), 12);
            line += std::string(7, ' ');
            line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
        }
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 5 system time correction
    line.clear();
    if (d_version == 2)
        {
            line += std::string(3, ' ');
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A0, 18, 2), 19);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A1, 18, 2), 19);
            line += Rinex_Printer::rightJustify(std::to_string(utc_model.tot), 9);
            if (d_pre_2009_file == false)
                {
                    if (eph.WN < 512)
                        {
                            if (utc_model.WN_T == 0)
                                {
                                    line += Rinex_Printer::rightJustify(std::to_string(eph.WN + 2048), 9);  // valid from 2019 to 2029
                                }
                            else
                                {
                                    line += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_T + (eph.WN / 256) * 256 + 1024), 9);  // valid from 2019 to 2029
                                }
                        }
                    else
                        {
                            if (utc_model.WN_T == 0)
                                {
                                    line += Rinex_Printer::rightJustify(std::to_string(eph.WN + 1024), 9);  // valid from 2019 to 2029
                                }
                            else
                                {
                                    line += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_T + (eph.WN / 256) * 256 + 1024), 9);  // valid from 2009 to 2019
                                }
                        }
                }
            else
                {
                    if (utc_model.WN_T == 0)
                        {
                            line += Rinex_Printer::rightJustify(std::to_string(eph.WN + 1024), 9);  // valid from 2019 to 2029
                        }
                    else
                        {
                            line += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_T + (eph.WN / 256) * 256 + 1024), 9);  // valid from 2009 to 2019
                        }
                }
            line += std::string(1, ' ');
            line += Rinex_Printer::leftJustify("DELTA-UTC: A0,A1,T,W", 20);
        }

    if (d_version == 3)
        {
            line += std::string("GPUT");
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A0, 16, 2), 18);
            line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A1, 15, 2), 16);
            line += Rinex_Printer::rightJustify(std::to_string(utc_model.tot), 7);
            if (d_pre_2009_file == false)
                {
                    if (eph.WN < 512)
                        {
                            if (utc_model.WN_T == 0)
                                {
                                    line += Rinex_Printer::rightJustify(std::to_string(eph.WN + 2048), 5);  // valid from 2019 to 2029
                                }
                            else
                                {
                                    line += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_T + (eph.WN / 256) * 256 + 2048), 5);  // valid from 2019 to 2029
                                }
                        }
                    else
                        {
                            if (utc_model.WN_T == 0)
                                {
                                    line += Rinex_Printer::rightJustify(std::to_string(eph.WN + 1024), 5);  // valid from 2009 to 2019
                                }
                            else
                                {
                                    line += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_T + (eph.WN / 256) * 256 + 1024), 5);  // valid from 2009 to 2019
                                }
                        }
                }
            else
                {
                    if (utc_model.WN_T == 0)
                        {
                            line += Rinex_Printer::rightJustify(std::to_string(eph.WN + 1024), 5);  // valid from 2009 to 2019
                        }
                    else
                        {
                            line += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_T + (eph.WN / 256) * 256 + 1024), 5);  // valid from 2009 to 2019
                        }
                }
            /*  if ( SBAS )
        {
          line += string(1, ' ');
          line += leftJustify(asString(tot_SBAS),5);
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
    out << line << '\n';

    // -------- Line 6 leap seconds
    // For leap second information, see https://endruntechnologies.com/support/leap-seconds
    line.clear();
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LS), 6);
    if (d_version == 2)
        {
            line += std::string(54, ' ');
        }
    if (d_version == 3)
        {
            line += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LSF), 6);
            line += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_LSF), 6);
            line += Rinex_Printer::rightJustify(std::to_string(utc_model.DN), 6);
            line += std::string(36, ' ');
        }
    line += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Gps_Iono& gps_iono, const Gps_Utc_Model& gps_utc_model, const Gps_Ephemeris& eph, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model) const
{
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += d_stringVersion;
    line += std::string(11, ' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4, ' ');
    line += std::string("M: MIXED");
    line += std::string(12, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 1
    line.clear();
    line += std::string("GAL ");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai2, 10, 2), 12);
    const double zero = 0.0;
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(zero, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 2
    line.clear();
    line += std::string("GPSA");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction
    line.clear();
    line += std::string("GAUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A0, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A1, 15, 2), 16);
    line += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.tot), 7);
    line += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.WNot), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction 2
    line.clear();
    line += std::string("GPGA");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A_0G, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A_1G, 15, 2), 16);
    line += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.t_0G), 7);
    line += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.WN_0G), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction 3
    line.clear();
    line += std::string("GPUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.A0, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.A1, 15, 2), 16);
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.tot), 7);
    if (d_pre_2009_file == false)
        {
            if (eph.WN < 512)
                {
                    if (gps_utc_model.WN_T == 0)
                        {
                            line += Rinex_Printer::rightJustify(std::to_string(eph.WN + 2048), 5);  // valid from 2019 to 2029
                        }
                    else
                        {
                            line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_T + (eph.WN / 256) * 256 + 2048), 5);  // valid from 2019 to 2029
                        }
                }
            else
                {
                    if (gps_utc_model.WN_T == 0)
                        {
                            line += Rinex_Printer::rightJustify(std::to_string(eph.WN + 1024), 5);  // valid from 2009 to 2019
                        }
                    else
                        {
                            line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_T + (eph.WN / 256) * 256 + 1024), 5);  // valid from 2009 to 2019
                        }
                }
        }
    else
        {
            if (gps_utc_model.WN_T == 0)
                {
                    line += Rinex_Printer::rightJustify(std::to_string(eph.WN + 1024), 5);  // valid from 2009 to 2019
                }
            else
                {
                    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_T + (eph.WN / 256) * 256 + 1024), 5);  // valid from 2009 to 2019
                }
        }
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 6 leap seconds
    // For leap second information, see https://endruntechnologies.com/support/leap-seconds
    line.clear();
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DeltaT_LS), 6);
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DeltaT_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DN), 6);
    line += std::string(36, ' ');
    line += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Beidou_Dnav_Iono& iono, const Beidou_Dnav_Utc_Model& utc_model) const
{
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += d_stringVersion;
    line += std::string(11, ' ');

    if (d_version == 3)
        {
            line += std::string("N: GNSS NAV DATA");
            line += std::string(4, ' ');
            // todo: Add here other systems...
            line += std::string("F: BDS");
            line += std::string(14, ' ');
            // ...
        }

    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::leftJustify("BDS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 1, only version 3 supported
    line.clear();
    line += std::string("BDSA");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 2
    line.clear();
    line += std::string("BDSB");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 5 system time correction
    line.clear();
    line += std::string("BDUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A0_UTC, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A1_UTC, 15, 2), 16);
    line += std::string(22, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 6 leap seconds
    // For leap second information, see https://endruntechnologies.com/support/leap-seconds
    line.clear();
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LS), 6);
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(utc_model.DN), 6);
    line += std::string(36, ' ');
    line += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Gps_Iono& gps_iono, const Gps_Utc_Model& gps_utc_model, const Gps_Ephemeris& gps_eph, const Beidou_Dnav_Iono& bds_dnav_iono, const Beidou_Dnav_Utc_Model& bds_dnav_utc_model) const
{
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += d_stringVersion;
    line += std::string(11, ' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4, ' ');
    line += std::string("M: MIXED");
    line += std::string(12, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 1, only version 3 supported
    line.clear();
    line += std::string("BDSA");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.alpha0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.alpha1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.alpha2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.alpha3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 2
    line.clear();
    line += std::string("BDSB");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.beta0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.beta1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.beta2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.beta3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 2
    line.clear();
    line += std::string("GPSA");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 5 system time correction
    line.clear();
    line += std::string("BDUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_utc_model.A0_UTC, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_utc_model.A1_UTC, 15, 2), 16);
    line += std::string(22, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);

    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction 3
    line.clear();
    line += std::string("GPUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.A0, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.A1, 15, 2), 16);
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.tot), 7);
    if (gps_eph.WN < 512)
        {
            if (gps_utc_model.WN_T == 0)
                {
                    line += Rinex_Printer::rightJustify(std::to_string(gps_eph.WN + 2048), 5);  // valid from 2019 to 2029
                }
            else
                {
                    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_T + (gps_eph.WN / 256) * 256 + 2048), 5);  // valid from 2019 to 2029
                }
        }
    else
        {
            if (gps_utc_model.WN_T == 0)
                {
                    line += Rinex_Printer::rightJustify(std::to_string(gps_eph.WN + 1024), 5);  // valid from 2009 to 2019
                }
            else
                {
                    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_T + (gps_eph.WN / 256) * 256 + 1024), 5);  // valid from 2009 to 2019
                }
        }
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 6 leap seconds
    // For leap second information, see https://endruntechnologies.com/support/leap-seconds
    line.clear();
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DeltaT_LS), 6);
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DeltaT_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DN), 6);
    line += std::string(36, ' ');
    line += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Gps_CNAV_Iono& gps_cnav_iono, const Gps_CNAV_Utc_Model& gps_cnav_utc_model, const Beidou_Dnav_Iono& bds_dnav_iono, const Beidou_Dnav_Utc_Model& bds_dnav_utc_model)
{
    std::string line;
    d_stringVersion = "3.02";
    d_version = 3;

    // -------- Line 1
    line = std::string(5, ' ');
    line += d_stringVersion;
    line += std::string(11, ' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4, ' ');
    line += std::string("M: MIXED");
    line += std::string(12, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 1, only version 3 supported
    line.clear();
    line += std::string("BDSA");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.alpha0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.alpha1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.alpha2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.alpha3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 2
    line.clear();
    line += std::string("BDSB");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.beta0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.beta1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.beta2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.beta3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 1
    line.clear();
    line += std::string("GPSA");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_cnav_iono.alpha0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_cnav_iono.alpha1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_cnav_iono.alpha2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_cnav_iono.alpha3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 5 system time correction
    line.clear();
    line += std::string("BDUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_utc_model.A0_UTC, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_utc_model.A1_UTC, 15, 2), 16);
    line += std::string(22, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction 3
    line.clear();
    line += std::string("GPUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_cnav_utc_model.A0, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_cnav_utc_model.A1, 15, 2), 16);
    line += Rinex_Printer::rightJustify(std::to_string(gps_cnav_utc_model.tot), 7);
    line += Rinex_Printer::rightJustify(std::to_string(gps_cnav_utc_model.WN_T), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 6 leap seconds
    // For leap second information, see https://endruntechnologies.com/support/leap-seconds
    line.clear();
    line += Rinex_Printer::rightJustify(std::to_string(gps_cnav_utc_model.DeltaT_LS), 6);
    line += Rinex_Printer::rightJustify(std::to_string(gps_cnav_utc_model.DeltaT_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(gps_cnav_utc_model.WN_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(gps_cnav_utc_model.DN), 6);
    line += std::string(36, ' ');
    line += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Glonass_Gnav_Utc_Model& glo_gnav_utc_model, const Beidou_Dnav_Iono& bds_dnav_iono, const Beidou_Dnav_Utc_Model& bds_dnav_utc_model) const
{
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += d_stringVersion;
    line += std::string(11, ' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4, ' ');
    line += std::string("M: MIXED");
    line += std::string(12, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 1, only version 3 supported
    line.clear();
    line += std::string("BDSA");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.alpha0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.alpha1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.alpha2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.alpha3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 2
    line.clear();
    line += std::string("BDSB");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.beta0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.beta1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.beta2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.beta3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 5 system time correction
    line.clear();
    line += std::string("BDUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_utc_model.A0_UTC, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_utc_model.A1_UTC, 15, 2), 16);
    line += std::string(22, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction 1
    line.clear();
    line += std::string("GLUT");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glo_gnav_utc_model.d_tau_c, 16, 2), 17);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
    line += Rinex_Printer::rightJustify(std::to_string(0.0), 7);
    line += Rinex_Printer::rightJustify(std::to_string(0.0), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 6 leap seconds
    // For leap second information, see https://endruntechnologies.com/support/leap-seconds
    line.clear();
    line += Rinex_Printer::rightJustify(std::to_string(bds_dnav_utc_model.DeltaT_LS), 6);
    line += Rinex_Printer::rightJustify(std::to_string(bds_dnav_utc_model.DeltaT_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(bds_dnav_utc_model.WN_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(bds_dnav_utc_model.DN), 6);
    line += std::string(36, ' ');
    line += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_nav_header(std::fstream& out, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model, const Beidou_Dnav_Iono& bds_dnav_iono, const Beidou_Dnav_Utc_Model& bds_dnav_utc_model) const
{
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += d_stringVersion;
    line += std::string(11, ' ');
    line += std::string("N: GNSS NAV DATA");
    line += std::string(4, ' ');
    line += std::string("M: MIXED");
    line += std::string(12, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS NAVIGATION MESSAGE FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 1
    line.clear();
    line += std::string("GAL ");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai2, 10, 2), 12);
    const double zero = 0.0;
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(zero, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 1, only version 3 supported
    line.clear();
    line += std::string("BDSA");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.alpha0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.alpha1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.alpha2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.alpha3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line ionospheric info 2
    line.clear();
    line += std::string("BDSB");
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.beta0, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.beta1, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.beta2, 10, 2), 12);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_iono.beta3, 10, 2), 12);
    line += std::string(7, ' ');
    line += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction
    line.clear();
    line += std::string("GAUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A0, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A1, 15, 2), 16);
    line += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.tot), 7);
    line += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.WNot), 5);
    line += std::string(10, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line system time correction 1
    // -------- Line 5 system time correction
    line.clear();
    line += std::string("BDUT");
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_utc_model.A0_UTC, 16, 2), 18);
    line += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(bds_dnav_utc_model.A1_UTC, 15, 2), 16);
    line += std::string(22, ' ');
    line += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 6 leap seconds
    // For leap second information, see https://endruntechnologies.com/support/leap-seconds
    line.clear();
    line += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.Delta_tLS), 6);
    line += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.Delta_tLSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.WN_LSF), 6);
    line += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.DN), 6);
    line += std::string(36, ' ');
    line += Rinex_Printer::leftJustify("LEAP SECONDS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_sbs_header(std::fstream& out) const
{
    std::string line;

    // -------- Line 1
    line.clear();
    line = std::string(5, ' ');
    line += std::string("2.10");
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("B SBAS DATA", 20);
    line += std::string(20, ' ');
    line += std::string("RINEX VERSION / TYPE");

    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20);
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
    line += Rinex_Printer::leftJustify(username, 20);
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
    line += Rinex_Printer::leftJustify(time_str, 20);
    line += Rinex_Printer::leftJustify("PGM / RUN BY / DATE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 3
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("REC INDEX/TYPE/VERS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT 1
    line.clear();
    line += Rinex_Printer::leftJustify("BROADCAST DATA FILE FOR GEO SV, GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT 2
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- End of Header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac) const
{
    if (glonass_gnav_almanac.i_satellite_freq_channel)
        {
        }  // Avoid compiler warning
    std::vector<std::string> data;
    std::string line_aux;

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
                    line_aux.clear();

                    if ((line_str.find("GLUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GLUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_c, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(0.0), 7);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(0.0), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GLGP", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GLGP");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_gps, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(0.0), 7);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(0.0), 5);
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
    for (int32_t i = 0; i < static_cast<int32_t>(data.size()) - 1; i++)
        {
            out << data[i] << '\n';
        }
    out.close();
    out.open(navGlofilename, std::ios::out | std::ios::app);
    out.seekp(pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC info.\n";
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& utc_model) const
{
    std::vector<std::string> data;
    std::string line_aux;

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
                    line_aux.clear();

                    if ((line_str.find("GAL", 0) != std::string::npos) && (line_str.find("IONOSPHERIC CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GAL ");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai0, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai1, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai2, 10, 2), 12);
                            const double zero = 0.0;
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(zero, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GAUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GAUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A0, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A1, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.tot), 7);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.WNot), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GPGA", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GPGA");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A_0G, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A_1G, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.t_0G), 7);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_0G), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.Delta_tLS), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.Delta_tLSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DN), 6);
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
    for (int32_t i = 0; i < static_cast<int32_t>(data.size()) - 1; i++)
        {
            out << data[i] << '\n';
        }
    out.close();
    out.open(navGalfilename, std::ios::out | std::ios::app);
    out.seekp(pos);
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
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha0, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha1, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha2, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha3, 10, 2), 12);
                                    line_aux += std::string(10, ' ');
                                    line_aux += Rinex_Printer::leftJustify("ION ALPHA", 20);
                                    data.push_back(line_aux);
                                }
                            else if (line_str.find("ION BETA", 59) != std::string::npos)
                                {
                                    line_aux += std::string(2, ' ');
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta0, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta1, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta2, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta3, 10, 2), 12);
                                    line_aux += std::string(10, ' ');
                                    line_aux += Rinex_Printer::leftJustify("ION BETA", 20);
                                    data.push_back(line_aux);
                                }
                            else if (line_str.find("DELTA-UTC", 59) != std::string::npos)
                                {
                                    line_aux += std::string(3, ' ');
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A0, 18, 2), 19);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A1, 18, 2), 19);
                                    line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.tot), 9);
                                    if (d_pre_2009_file == false)
                                        {
                                            if (eph.WN < 512)
                                                {
                                                    line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_T + (eph.WN / 256) * 256 + 2048), 9);  // valid from 2019 to 2029
                                                }
                                            else
                                                {
                                                    line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_T + (eph.WN / 256) * 256 + 1024), 9);  // valid from 2009 to 2019
                                                }
                                        }
                                    else
                                        {
                                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_T + (eph.WN / 256) * 256), 9);
                                        }
                                    line_aux += std::string(1, ' ');
                                    line_aux += Rinex_Printer::leftJustify("DELTA-UTC: A0,A1,T,W", 20);
                                    data.push_back(line_aux);
                                }
                            else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                                {
                                    line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LS), 6);
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

                    if (d_version == 3)
                        {
                            if (line_str.find("GPSA", 0) != std::string::npos)
                                {
                                    line_aux += std::string("GPSA");
                                    line_aux += std::string(1, ' ');
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha0, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha1, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha2, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha3, 10, 2), 12);
                                    line_aux += std::string(7, ' ');
                                    line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                                    data.push_back(line_aux);
                                }
                            else if (line_str.find("GPSB", 0) != std::string::npos)
                                {
                                    line_aux += std::string("GPSB");
                                    line_aux += std::string(1, ' ');
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta0, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta1, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta2, 10, 2), 12);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta3, 10, 2), 12);
                                    line_aux += std::string(7, ' ');
                                    line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                                    data.push_back(line_aux);
                                }
                            else if (line_str.find("GPUT", 0) != std::string::npos)
                                {
                                    line_aux += std::string("GPUT");
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A0, 16, 2), 18);
                                    line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A1, 15, 2), 16);
                                    line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.tot), 7);
                                    if (d_pre_2009_file == false)
                                        {
                                            if (eph.WN < 512)
                                                {
                                                    line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_T + (eph.WN / 256) * 256 + 2048), 5);  // valid from 2019 to 2029
                                                }
                                            else
                                                {
                                                    line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_T + (eph.WN / 256) * 256 + 1024), 5);  // valid from 2009 to 2019
                                                }
                                        }
                                    else
                                        {
                                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_T + (eph.WN / 256) * 256 + 1024), 5);  // valid from 1999 to 2008
                                        }
                                    line_aux += std::string(10, ' ');
                                    line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                                    data.push_back(line_aux);
                                }
                            else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                                {
                                    line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LS), 6);
                                    line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LSF), 6);
                                    line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_LSF), 6);
                                    line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DN), 6);
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
    for (int32_t i = 0; i < static_cast<int32_t>(data.size()) - 1; i++)
        {
            out << data[i] << '\n';
        }
    out.close();
    out.open(navfilename, std::ios::out | std::ios::app);
    out.seekp(pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info.\n";
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Gps_CNAV_Utc_Model& utc_model, const Gps_CNAV_Iono& iono) const
{
    std::vector<std::string> data;
    std::string line_aux;

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
                    line_aux.clear();

                    if (line_str.find("GPSA", 0) != std::string::npos)
                        {
                            line_aux += std::string("GPSA");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha0, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha1, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha2, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha3, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("GPSB", 0) != std::string::npos)
                        {
                            line_aux += std::string("GPSB");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta0, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta1, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta2, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta3, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("GPUT", 0) != std::string::npos)
                        {
                            line_aux += std::string("GPUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A0, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A1, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.tot), 7);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_T), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LS), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DN), 6);
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
    for (int32_t i = 0; i < static_cast<int32_t>(data.size()) - 1; i++)
        {
            out << data[i] << '\n';
        }
    out.close();
    out.open(navfilename, std::ios::out | std::ios::app);
    out.seekp(pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info.\n";
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Gps_CNAV_Utc_Model& utc_model, const Gps_CNAV_Iono& iono, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model) const
{
    std::vector<std::string> data;
    std::string line_aux;

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
                    line_aux.clear();
                    if ((line_str.find("GAL", 0) != std::string::npos) && (line_str.find("IONOSPHERIC CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GAL ");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai0, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai1, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai2, 10, 2), 12);
                            const double zero = 0.0;
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(zero, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GPSA", 0) != std::string::npos) && (line_str.find("IONOSPHERIC CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GPSA");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha0, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha1, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha2, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha3, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GPSB", 0) != std::string::npos) && (line_str.find("IONOSPHERIC CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GPSB");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta0, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta1, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta2, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta3, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }

                    else if ((line_str.find("GAUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GAUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A0, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A1, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.tot), 7);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.WNot), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GPGA", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GPGA");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A_0G, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A_1G, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.t_0G), 7);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.WN_0G), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("GPUT", 0) != std::string::npos)
                        {
                            line_aux += std::string("GPUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A0, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A1, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.tot), 7);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_T), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LS), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DN), 6);
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
    for (int32_t i = 0; i < static_cast<int32_t>(data.size()) - 1; i++)
        {
            out << data[i] << '\n';
        }
    out.close();
    out.open(navfilename, std::ios::out | std::ios::app);
    out.seekp(pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info.\n";
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Gps_Iono& gps_iono, const Gps_Utc_Model& gps_utc_model, const Gps_Ephemeris& eph, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model) const
{
    std::vector<std::string> data;
    std::string line_aux;

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
                    line_aux.clear();

                    if (line_str.find("GPSA", 0) != std::string::npos)
                        {
                            line_aux += std::string("GPSA");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha0, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha1, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha2, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha3, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GAL", 0) != std::string::npos) && (line_str.find("IONOSPHERIC CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GAL ");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai0, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai1, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai2, 10, 2), 12);
                            const double zero = 0.0;
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(zero, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GPSB", 0) != std::string::npos) && (line_str.find("IONOSPHERIC CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GPSB");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.beta0, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.beta1, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.beta2, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.beta3, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GPUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GPUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.A0, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.A1, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.tot), 7);
                            if (d_pre_2009_file == false)
                                {
                                    if (eph.WN < 512)
                                        {
                                            line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_T + (eph.WN / 256) * 256 + 2048), 5);  // valid from 2019 to 2029
                                        }
                                    else
                                        {
                                            line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_T + (eph.WN / 256) * 256 + 1024), 5);  // valid from 2009 to 2019
                                        }
                                }
                            else
                                {
                                    line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_T + (eph.WN / 256) * 256 + 1024), 5);  // valid from 1999 to 2008
                                }
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GAUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GAUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A0, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A1, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.tot), 7);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.WNot), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GPGA", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GPGA");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A_0G, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A_1G, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.t_0G), 7);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.WN_0G), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DeltaT_LS), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DeltaT_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DN), 6);
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
    for (int32_t i = 0; i < static_cast<int32_t>(data.size()) - 1; i++)
        {
            out << data[i] << '\n';
        }
    out.close();
    out.open(navMixfilename, std::ios::out | std::ios::app);
    out.seekp(pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info.\n";
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Gps_Iono& gps_iono, const Gps_Utc_Model& gps_utc_model, const Gps_Ephemeris& eph, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac) const
{
    if (glonass_gnav_almanac.i_satellite_freq_channel)
        {
        }  // Avoid compiler warning
    std::vector<std::string> data;
    std::string line_aux;

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
                    line_aux.clear();

                    if (line_str.find("GPSA", 0) != std::string::npos)
                        {
                            line_aux += std::string("GPSA");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha0, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha1, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha2, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha3, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GPUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GPUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.A0, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.A1, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.tot), 7);
                            if (d_pre_2009_file == false)
                                {
                                    if (eph.WN < 512)
                                        {
                                            line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_T + (eph.WN / 256) * 256 + 2048), 5);  // valid from 2019 to 2029
                                        }
                                    else
                                        {
                                            line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_T + (eph.WN / 256) * 256 + 1024), 5);  // valid from 2009 to 2019
                                        }
                                }
                            else
                                {
                                    line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_T + (eph.WN / 256) * 256), 5);
                                }
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GLUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GLUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_c, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(0.0), 7);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(0.0), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GLGP", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GLGP");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_gps, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(0.0), 7);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(0.0), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DeltaT_LS), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DeltaT_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DN), 6);
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
    for (int32_t i = 0; i < static_cast<int32_t>(data.size()) - 1; i++)
        {
            out << data[i] << '\n';
        }
    out.close();
    out.open(navMixfilename, std::ios::out | std::ios::app);
    out.seekp(pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info.\n";
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Gps_CNAV_Iono& gps_iono, const Gps_CNAV_Utc_Model& gps_utc_model, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac) const
{
    if (glonass_gnav_almanac.i_satellite_freq_channel)
        {
        }  // Avoid compiler warning
    std::vector<std::string> data;
    std::string line_aux;

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
                    line_aux.clear();

                    if (line_str.find("GPSA", 0) != std::string::npos)
                        {
                            line_aux += std::string("GPSA");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha0, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha1, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha2, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_iono.alpha3, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GPUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GPUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.A0, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(gps_utc_model.A1, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.tot), 7);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_T), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GLUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GLUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_c, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(0.0), 7);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(0.0), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GLGP", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GLGP");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_gps, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(0.0), 7);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(0.0), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DeltaT_LS), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DeltaT_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.WN_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(gps_utc_model.DN), 6);
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
    for (int32_t i = 0; i < static_cast<int32_t>(data.size()) - 1; i++)
        {
            out << data[i] << '\n';
        }
    out.close();
    out.open(navMixfilename, std::ios::out | std::ios::app);
    out.seekp(pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info.\n";
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Galileo_Iono& galileo_iono, const Galileo_Utc_Model& galileo_utc_model, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model, const Glonass_Gnav_Almanac& glonass_gnav_almanac) const
{
    if (glonass_gnav_almanac.i_satellite_freq_channel)
        {
        }  // Avoid compiler warning
    // Avoid compiler warning, there is not time system correction between Galileo and GLONASS
    if (galileo_utc_model.A_0G > 0.0)
        {
        }
    std::vector<std::string> data;
    std::string line_aux;

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
                    line_aux.clear();

                    if ((line_str.find("GAL", 0) != std::string::npos) && (line_str.find("IONOSPHERIC CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GAL ");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai0, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai1, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_iono.ai2, 10, 2), 12);
                            const double zero = 0.0;
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(zero, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GAUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GAUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A0, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(galileo_utc_model.A1, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.tot), 7);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.WNot), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if ((line_str.find("GLUT", 0) != std::string::npos) && (line_str.find("TIME SYSTEM CORR", 59) != std::string::npos))
                        {
                            line_aux += std::string("GLUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(glonass_gnav_utc_model.d_tau_c, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(0.0, 15, 2), 16);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(0.0), 7);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(0.0), 5);
                            line_aux += std::string(10, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            line_aux += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.Delta_tLS), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.Delta_tLSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.WN_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.DN), 6);
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
    for (int32_t i = 0; i < static_cast<int32_t>(data.size()) - 1; i++)
        {
            out << data[i] << '\n';
        }
    out.close();
    out.open(navMixfilename, std::ios::out | std::ios::app);
    out.seekp(pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info.\n";
}


void Rinex_Printer::update_nav_header(std::fstream& out, const Beidou_Dnav_Utc_Model& utc_model, const Beidou_Dnav_Iono& iono) const
{
    std::vector<std::string> data;
    std::string line_aux;

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
                    line_aux.clear();

                    if (line_str.find("BDSA", 0) != std::string::npos)
                        {
                            line_aux += std::string("BDSA");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha0, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha1, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha2, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.alpha3, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("BDSB", 0) != std::string::npos)
                        {
                            line_aux += std::string("BDSB");
                            line_aux += std::string(1, ' ');
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta0, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta1, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta2, 10, 2), 12);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(iono.beta3, 10, 2), 12);
                            line_aux += std::string(7, ' ');
                            line_aux += Rinex_Printer::leftJustify("IONOSPHERIC CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("BDUT", 0) != std::string::npos)
                        {
                            line_aux += std::string("BDUT");
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A0_UTC, 16, 2), 18);
                            line_aux += Rinex_Printer::rightJustify(Rinex_Printer::doub2for(utc_model.A1_UTC, 15, 2), 16);
                            line_aux += std::string(22, ' ');
                            line_aux += Rinex_Printer::leftJustify("TIME SYSTEM CORR", 20);
                            data.push_back(line_aux);
                        }
                    else if (line_str.find("LEAP SECONDS", 59) != std::string::npos)
                        {
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LS), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DN), 6);
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
    for (int32_t i = 0; i < static_cast<int32_t>(data.size()) - 1; i++)
        {
            out << data[i] << '\n';
        }
    out.close();
    out.open(navfilename, std::ios::out | std::ios::app);
    out.seekp(pos);
    std::cout << "The RINEX Navigation file header has been updated with UTC and IONO info.\n";
}


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int32_t, Gps_Ephemeris>& eph_map) const
{
    std::string line;
    std::map<int32_t, Gps_Ephemeris>::const_iterator gps_ephemeris_iter;

    for (gps_ephemeris_iter = eph_map.cbegin();
         gps_ephemeris_iter != eph_map.cend();
         gps_ephemeris_iter++)
        {
            // -------- SV / EPOCH / SV CLK
            const boost::posix_time::ptime p_utc_time = Rinex_Printer::compute_GPS_time(gps_ephemeris_iter->second, gps_ephemeris_iter->second.toc);
            const std::string timestring = boost::posix_time::to_iso_string(p_utc_time);
            const std::string month(timestring, 4, 2);
            const std::string day(timestring, 6, 2);
            const std::string hour(timestring, 9, 2);
            const std::string minutes(timestring, 11, 2);
            const std::string seconds(timestring, 13, 2);
            if (d_version == 2)
                {
                    line += Rinex_Printer::rightJustify(std::to_string(gps_ephemeris_iter->second.PRN), 2);
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
                    line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.af0, 18, 2);
                    line += std::string(1, ' ');
                    line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.af1, 18, 2);
                    line += std::string(1, ' ');
                    line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.af2, 18, 2);
                    line += std::string(1, ' ');
                }
            if (d_version == 3)
                {
                    line += satelliteSystem.find("GPS")->second;
                    if (gps_ephemeris_iter->second.PRN < 10)
                        {
                            line += std::string("0");
                        }
                    line += std::to_string(gps_ephemeris_iter->second.PRN);
                    const std::string year(timestring, 0, 4);
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
                    line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.af0, 18, 2);
                    line += std::string(1, ' ');
                    line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.af1, 18, 2);
                    line += std::string(1, ' ');
                    line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.af2, 18, 2);
                }
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 1
            line.clear();

            if (d_version == 2)
                {
                    line += std::string(4, ' ');
                }
            if (d_version == 3)
                {
                    line += std::string(5, ' ');
                }
            // If there is a discontinued reception the ephemeris is not validated
            if (gps_ephemeris_iter->second.IODE_SF2 == gps_ephemeris_iter->second.IODE_SF3)
                {
                    line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.IODE_SF2, 18, 2);
                }
            else
                {
                    LOG(WARNING) << "Discontinued reception of Frame 2 and 3";
                }
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.Crs, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.delta_n, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.M_0, 18, 2);
            if (d_version == 2)
                {
                    line += std::string(1, ' ');
                }
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 2
            line.clear();
            if (d_version == 2)
                {
                    line += std::string(4, ' ');
                }
            if (d_version == 3)
                {
                    line += std::string(5, ' ');
                }
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.Cuc, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.ecc, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.Cus, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.sqrtA, 18, 2);
            if (d_version == 2)
                {
                    line += std::string(1, ' ');
                }
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 3
            line.clear();
            if (d_version == 2)
                {
                    line += std::string(4, ' ');
                }
            if (d_version == 3)
                {
                    line += std::string(5, ' ');
                }
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.toe, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.Cic, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.OMEGA_0, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.Cis, 18, 2);
            if (d_version == 2)
                {
                    line += std::string(1, ' ');
                }
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 4
            line.clear();
            if (d_version == 2)
                {
                    line += std::string(4, ' ');
                }
            if (d_version == 3)
                {
                    line += std::string(5, ' ');
                }
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.i_0, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.Crc, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.omega, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.OMEGAdot, 18, 2);
            if (d_version == 2)
                {
                    line += std::string(1, ' ');
                }
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 5
            line.clear();
            if (d_version == 2)
                {
                    line += std::string(4, ' ');
                }
            if (d_version == 3)
                {
                    line += std::string(5, ' ');
                }
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.idot, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(static_cast<double>(gps_ephemeris_iter->second.code_on_L2), 18, 2);
            line += std::string(1, ' ');
            double GPS_week_continuous_number;
            if (d_pre_2009_file == false)
                {
                    if (gps_ephemeris_iter->second.WN < 512)
                        {
                            GPS_week_continuous_number = static_cast<double>(gps_ephemeris_iter->second.WN + 2048);  // valid until 2029
                        }
                    else
                        {
                            GPS_week_continuous_number = static_cast<double>(gps_ephemeris_iter->second.WN + 1024);  // valid until April 7, 2019
                        }
                }
            else
                {
                    GPS_week_continuous_number = static_cast<double>(gps_ephemeris_iter->second.WN + 1024);
                }
            // This week goes with Toe. This is different from the GPS week in the original satellite message!
            if (gps_ephemeris_iter->second.toe < 7200.0)
                {
                    GPS_week_continuous_number += 1.0;
                }
            line += Rinex_Printer::doub2for(GPS_week_continuous_number, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(static_cast<double>(gps_ephemeris_iter->second.code_on_L2), 18, 2);
            if (d_version == 2)
                {
                    line += std::string(1, ' ');
                }
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 6
            line.clear();
            if (d_version == 2)
                {
                    line += std::string(4, ' ');
                }
            if (d_version == 3)
                {
                    line += std::string(5, ' ');
                }
            line += Rinex_Printer::doub2for(static_cast<double>(gps_ephemeris_iter->second.SV_accuracy), 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(static_cast<double>(gps_ephemeris_iter->second.SV_health), 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.TGD, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.IODC, 18, 2);
            if (d_version == 2)
                {
                    line += std::string(1, ' ');
                }
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 7
            line.clear();
            if (d_version == 2)
                {
                    line += std::string(4, ' ');
                }
            if (d_version == 3)
                {
                    line += std::string(5, ' ');
                }
            double tx_time_of_message = gps_ephemeris_iter->second.tow;
            if (gps_ephemeris_iter->second.toe < 7200.0)
                {
                    tx_time_of_message -= 604800.0;  // see RINEX 3.03 section 6.13
                }
            line += Rinex_Printer::doub2for(tx_time_of_message, 18, 2);
            line += std::string(1, ' ');
            int curve_fit_interval = 4;

            if (gps_ephemeris_iter->second.satelliteBlock.at(gps_ephemeris_iter->second.PRN) == "IIA")
                {
                    // Block II/IIA (Table 20-XI IS-GPS-200M)
                    if ((gps_ephemeris_iter->second.IODC > 239) && (gps_ephemeris_iter->second.IODC < 248))
                        {
                            curve_fit_interval = 8;
                        }
                    if (((gps_ephemeris_iter->second.IODC > 247) && (gps_ephemeris_iter->second.IODC < 256)) || (gps_ephemeris_iter->second.IODC == 496))
                        {
                            curve_fit_interval = 14;
                        }
                    if ((gps_ephemeris_iter->second.IODC > 496) && (gps_ephemeris_iter->second.IODC < 504))
                        {
                            curve_fit_interval = 26;
                        }
                    if ((gps_ephemeris_iter->second.IODC > 503) && (gps_ephemeris_iter->second.IODC < 511))
                        {
                            curve_fit_interval = 50;
                        }
                    if (((gps_ephemeris_iter->second.IODC > 751) && (gps_ephemeris_iter->second.IODC < 757)) || (gps_ephemeris_iter->second.IODC == 511))
                        {
                            curve_fit_interval = 74;
                        }
                    if (gps_ephemeris_iter->second.IODC == 757)
                        {
                            curve_fit_interval = 98;
                        }
                }

            if ((gps_ephemeris_iter->second.satelliteBlock.at(gps_ephemeris_iter->second.PRN) == "IIR") ||
                (gps_ephemeris_iter->second.satelliteBlock.at(gps_ephemeris_iter->second.PRN) == "IIR-M") ||
                (gps_ephemeris_iter->second.satelliteBlock.at(gps_ephemeris_iter->second.PRN) == "IIF") ||
                (gps_ephemeris_iter->second.satelliteBlock.at(gps_ephemeris_iter->second.PRN) == "III"))
                {
                    // Block IIR/IIR-M/IIF/III/IIIF (Table 20-XII IS-GPS-200M)
                    if ((gps_ephemeris_iter->second.IODC > 239) && (gps_ephemeris_iter->second.IODC < 248))
                        {
                            curve_fit_interval = 8;
                        }
                    if (((gps_ephemeris_iter->second.IODC > 247) && (gps_ephemeris_iter->second.IODC < 256)) || (gps_ephemeris_iter->second.IODC == 496))
                        {
                            curve_fit_interval = 14;
                        }
                    if (((gps_ephemeris_iter->second.IODC > 496) && (gps_ephemeris_iter->second.IODC < 504)) || ((gps_ephemeris_iter->second.IODC > 1020) && (gps_ephemeris_iter->second.IODC < 1024)))
                        {
                            curve_fit_interval = 26;
                        }
                }
            if (curve_fit_interval == 4)
                {
                    line += Rinex_Printer::doub2for(0.0, 18, 2);
                }
            else
                {
                    line += Rinex_Printer::doub2for(1.0, 18, 2);
                }
            line += std::string(1, ' ');
            line += std::string(18, ' ');  // spare
            line += std::string(1, ' ');
            line += std::string(18, ' ');  // spare
            if (d_version == 2)
                {
                    line += std::string(1, ' ');
                }
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
            line.clear();
        }
}


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int32_t, Gps_CNAV_Ephemeris>& eph_map)
{
    std::string line;
    std::map<int32_t, Gps_CNAV_Ephemeris>::const_iterator gps_ephemeris_iter;

    for (gps_ephemeris_iter = eph_map.cbegin();
         gps_ephemeris_iter != eph_map.cend();
         gps_ephemeris_iter++)
        {
            // -------- SV / EPOCH / SV CLK
            const boost::posix_time::ptime p_utc_time = Rinex_Printer::compute_GPS_time(gps_ephemeris_iter->second, gps_ephemeris_iter->second.toc);
            const std::string timestring = boost::posix_time::to_iso_string(p_utc_time);
            const std::string month(timestring, 4, 2);
            const std::string day(timestring, 6, 2);
            const std::string hour(timestring, 9, 2);
            const std::string minutes(timestring, 11, 2);
            const std::string seconds(timestring, 13, 2);
            line += satelliteSystem.find("GPS")->second;
            if (gps_ephemeris_iter->second.PRN < 10)
                {
                    line += std::string("0");
                }
            line += std::to_string(gps_ephemeris_iter->second.PRN);
            const std::string year(timestring, 0, 4);
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
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.af0, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.af1, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.af2, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 1
            line.clear();
            line += std::string(5, ' ');
            // If there is no IODE in CNAV, so we check if Toe in message Type 10, Toe in Message type 11 and Toc in message types 30-37.
            // Whenever these three terms do not match, a data set cutover has occurred and new data must be collected.
            // See IS-GPS-200M, paragraph 20.3.3.4.1
            if (!((gps_ephemeris_iter->second.toe1 == gps_ephemeris_iter->second.toe2) && (gps_ephemeris_iter->second.toe1 == gps_ephemeris_iter->second.toc)))  // Toe1: Toe in message type 10,  Toe2: Toe in message type 11
                {
                    // Toe1: Toe in message type 10,  Toe2: Toe in message type 11,
                    d_fake_cnav_iode = d_fake_cnav_iode + 1;
                    if (d_fake_cnav_iode == 240)
                        {
                            d_fake_cnav_iode = 1;
                        }
                }

            line += Rinex_Printer::doub2for(d_fake_cnav_iode, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.Crs, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.delta_n, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.M_0, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 2
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.Cuc, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.ecc, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.Cus, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.sqrtA, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 3
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(std::max(gps_ephemeris_iter->second.toe1, gps_ephemeris_iter->second.toe2), 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.Cic, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.OMEGA_0, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.Cis, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 4
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.i_0, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.Crc, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.omega, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.OMEGAdot, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 5
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.idot, 18, 2);
            line += std::string(1, ' ');
            // No data flag for L2 P code
            double my_zero = 0.0;
            line += Rinex_Printer::doub2for(my_zero, 18, 2);
            line += std::string(1, ' ');
            auto GPS_week_continuous_number = static_cast<double>(gps_ephemeris_iter->second.WN);
            // This week goes with Toe. This is different from the GPS week in the original satellite message!
            if (gps_ephemeris_iter->second.toe1 < 7200.0)
                {
                    GPS_week_continuous_number += 1.0;
                }
            line += Rinex_Printer::doub2for(GPS_week_continuous_number, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(my_zero, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 6
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(static_cast<double>(gps_ephemeris_iter->second.URA), 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(static_cast<double>(gps_ephemeris_iter->second.signal_health), 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.TGD, 18, 2);
            line += std::string(1, ' ');
            // no IODC in CNAV, so we fake it (see above)
            line += Rinex_Printer::doub2for(d_fake_cnav_iode, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 7
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(gps_ephemeris_iter->second.tow, 18, 2);
            line += std::string(1, ' ');
            double curve_fit_interval = 0.0;  /// ?? Not defined in CNAV
            line += Rinex_Printer::doub2for(curve_fit_interval, 18, 2);
            line += std::string(1, ' ');
            line += std::string(18, ' ');  // spare
            line += std::string(1, ' ');
            line += std::string(18, ' ');  // spare
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
            line.clear();
        }
}


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int32_t, Galileo_Ephemeris>& eph_map) const
{
    std::string line;
    std::map<int32_t, Galileo_Ephemeris>::const_iterator galileo_ephemeris_iter;
    line.clear();
    for (galileo_ephemeris_iter = eph_map.cbegin();
         galileo_ephemeris_iter != eph_map.cend();
         galileo_ephemeris_iter++)
        {
            // -------- SV / EPOCH / SV CLK
            const boost::posix_time::ptime p_utc_time = Rinex_Printer::compute_Galileo_time(galileo_ephemeris_iter->second, galileo_ephemeris_iter->second.toe);
            const std::string timestring = boost::posix_time::to_iso_string(p_utc_time);
            const std::string month(timestring, 4, 2);
            const std::string day(timestring, 6, 2);
            const std::string hour(timestring, 9, 2);
            const std::string minutes(timestring, 11, 2);
            const std::string seconds(timestring, 13, 2);

            line += satelliteSystem.find("Galileo")->second;
            if (galileo_ephemeris_iter->second.PRN < 10)
                {
                    line += std::string("0");
                }
            line += std::to_string(galileo_ephemeris_iter->second.PRN);
            const std::string year(timestring, 0, 4);
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
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.af0, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.af1, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.af2, 18, 2);

            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 1
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(static_cast<double>(galileo_ephemeris_iter->second.IOD_ephemeris), 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.Crs, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.delta_n, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.M_0, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 2
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.Cuc, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.ecc, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.Cus, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.sqrtA, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 3
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.toe, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.Cic, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.OMEGA_0, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.Cis, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 4
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.i_0, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.Crc, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.omega, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.OMEGAdot, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 5
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.idot, 18, 2);
            line += std::string(1, ' ');
            // double one = 1.0; // INAV E1-B
            std::string iNAVE1B("1000000001");
            int32_t data_source_INAV = Rinex_Printer::toInt(iNAVE1B, 10);
            line += Rinex_Printer::doub2for(static_cast<double>(data_source_INAV), 18, 2);
            line += std::string(1, ' ');
            auto GST_week = static_cast<double>(galileo_ephemeris_iter->second.WN);
            double num_GST_rollovers = floor((GST_week + 1024.0) / 4096.0);
            double Galileo_week_continuous_number = GST_week + 1024.0 + num_GST_rollovers * 4096.0;
            line += Rinex_Printer::doub2for(Galileo_week_continuous_number, 18, 2);
            line += std::string(1, ' ');
            const double zero = 0.0;
            line += Rinex_Printer::doub2for(zero, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 6
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.SISA, 18, 2);
            line += std::string(1, ' ');
            std::string E1B_HS;
            std::string E5B_HS;
            if (galileo_ephemeris_iter->second.E1B_HS == 0)
                {
                    E1B_HS = "00";
                }
            if (galileo_ephemeris_iter->second.E1B_HS == 1)
                {
                    E1B_HS = "01";
                }
            if (galileo_ephemeris_iter->second.E1B_HS == 2)
                {
                    E1B_HS = "10";
                }
            if (galileo_ephemeris_iter->second.E1B_HS == 3)
                {
                    E1B_HS = "11";
                }
            if (galileo_ephemeris_iter->second.E5b_HS == 0)
                {
                    E5B_HS = "00";
                }
            if (galileo_ephemeris_iter->second.E5b_HS == 1)
                {
                    E5B_HS = "01";
                }
            if (galileo_ephemeris_iter->second.E5b_HS == 2)
                {
                    E5B_HS = "10";
                }
            if (galileo_ephemeris_iter->second.E5b_HS == 3)
                {
                    E5B_HS = "11";
                }

            std::string E1B_DVS = std::to_string(galileo_ephemeris_iter->second.E1B_DVS);

            std::string SVhealth_str = E5B_HS + std::to_string(galileo_ephemeris_iter->second.E5b_DVS) + "11" + "1" + std::string(E1B_DVS) + std::string(E1B_HS) + std::to_string(galileo_ephemeris_iter->second.E1B_DVS);
            int32_t SVhealth = Rinex_Printer::toInt(SVhealth_str, 9);
            line += Rinex_Printer::doub2for(static_cast<double>(SVhealth), 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.BGD_E1E5a, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.BGD_E1E5b, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 7
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(galileo_ephemeris_iter->second.tow, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(zero, 18, 2);
            line += std::string(1, ' ');
            line += std::string(18, ' ');  // spare
            line += std::string(1, ' ');
            line += std::string(18, ' ');  // spare
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
            line.clear();
        }
}


void Rinex_Printer::log_rinex_nav(std::fstream& out, const std::map<int32_t, Glonass_Gnav_Ephemeris>& eph_map) const
{
    std::string line;
    std::map<int32_t, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_ephemeris_iter;

    for (glonass_gnav_ephemeris_iter = eph_map.cbegin();
         glonass_gnav_ephemeris_iter != eph_map.cend();
         glonass_gnav_ephemeris_iter++)
        {
            // -------- SV / EPOCH / SV CLK
            const boost::posix_time::ptime p_utc_time = glonass_gnav_ephemeris_iter->second.glot_to_utc(glonass_gnav_ephemeris_iter->second.d_t_b, 0.0);
            const std::string timestring = boost::posix_time::to_iso_string(p_utc_time);
            const std::string month(timestring, 4, 2);
            const std::string day(timestring, 6, 2);
            const std::string hour(timestring, 9, 2);
            const std::string minutes(timestring, 11, 2);
            const std::string seconds(timestring, 13, 2);
            if (d_version == 2)
                {
                    line += Rinex_Printer::rightJustify(std::to_string(glonass_gnav_ephemeris_iter->second.PRN), 2);
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
                    line += Rinex_Printer::doub2for(-glonass_gnav_ephemeris_iter->second.d_tau_c, 18, 2);
                    line += std::string(1, ' ');
                    line += Rinex_Printer::doub2for(glonass_gnav_ephemeris_iter->second.d_gamma_n, 18, 2);
                    line += std::string(1, ' ');
                    line += Rinex_Printer::doub2for(glonass_gnav_ephemeris_iter->second.d_t_k, 18, 2);
                    line += std::string(1, ' ');
                }
            if (d_version == 3)
                {
                    line += satelliteSystem.find("GLONASS")->second;
                    if (glonass_gnav_ephemeris_iter->second.PRN < 10)
                        {
                            line += std::string("0");
                        }
                    line += std::to_string(glonass_gnav_ephemeris_iter->second.PRN);
                    const std::string year(timestring, 0, 4);
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
                    // TODO need to define this here. what is nd
                    line += Rinex_Printer::doub2for(glonass_gnav_ephemeris_iter->second.d_t_k + p_utc_time.date().day_of_week() * 86400, 18, 2);
                }
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 1
            line.clear();
            // TODO Why is this happening here?. The extra space maybe is intended to help with readability
            if (d_version == 2)
                {
                    line += std::string(3, ' ');
                }
            if (d_version == 3)
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
            if (d_version == 2)
                {
                    line += std::string(1, ' ');
                }
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 2
            line.clear();
            if (d_version == 2)
                {
                    line += std::string(3, ' ');
                }
            if (d_version == 3)
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
            if (d_version == 2)
                {
                    line += std::string(1, ' ');
                }
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 3
            line.clear();
            if (d_version == 2)
                {
                    line += std::string(3, ' ');
                }
            if (d_version == 3)
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
            if (d_version == 2)
                {
                    line += std::string(1, ' ');
                }
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
            line.clear();
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
    std::string line;
    std::map<int32_t, Beidou_Dnav_Ephemeris>::const_iterator bds_ephemeris_iter;

    for (bds_ephemeris_iter = eph_map.cbegin();
         bds_ephemeris_iter != eph_map.cend();
         bds_ephemeris_iter++)
        {
            // -------- SV / EPOCH / SV CLK
            const boost::posix_time::ptime p_utc_time = Rinex_Printer::compute_BDS_time(bds_ephemeris_iter->second, bds_ephemeris_iter->second.toc);
            const std::string timestring = boost::posix_time::to_iso_string(p_utc_time);
            const std::string month(timestring, 4, 2);
            const std::string day(timestring, 6, 2);
            const std::string hour(timestring, 9, 2);
            const std::string minutes(timestring, 11, 2);
            const std::string seconds(timestring, 13, 2);

            line += satelliteSystem.find("Beidou")->second;
            if (bds_ephemeris_iter->second.PRN < 10)
                {
                    line += std::string("0");
                }
            line += std::to_string(bds_ephemeris_iter->second.PRN);
            const std::string year(timestring, 0, 4);
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
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.af0, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.af1, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.af2, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 1
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.AODE, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.Crs, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.delta_n, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.M_0, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 2
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.Cuc, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.ecc, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.Cus, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.sqrtA, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 3
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.toe, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.Cic, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.OMEGA_0, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.Cis, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 4
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.i_0, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.Crc, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.omega, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.OMEGAdot, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 5
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.idot, 18, 2);
            line += std::string(1, ' ');
            line += std::string(18, ' ');  // spare
            line += std::string(1, ' ');
            auto BDS_week_continuous_number = static_cast<double>(bds_ephemeris_iter->second.WN);
            line += Rinex_Printer::doub2for(BDS_week_continuous_number, 18, 2);
            line += std::string(1, ' ');
            line += std::string(18, ' ');  // spare
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';


            // -------- BROADCAST ORBIT - 6
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(static_cast<double>(bds_ephemeris_iter->second.SV_accuracy), 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(static_cast<double>(bds_ephemeris_iter->second.SV_health), 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.TGD1, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.TGD2, 18, 2);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // -------- BROADCAST ORBIT - 7
            line.clear();
            line += std::string(5, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.tow, 18, 2);
            line += std::string(1, ' ');
            line += Rinex_Printer::doub2for(bds_ephemeris_iter->second.AODC, 18, 2);
            line += std::string(1, ' ');
            line += std::string(18, ' ');  // spare
            line += std::string(1, ' ');
            line += std::string(18, ' ');  // spare
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
            line.clear();
        }
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Glonass_Gnav_Ephemeris& eph, double d_TOW_first_observation, const std::string& glonass_bands)
{
    if (eph.d_m > 0.0)
        {
        }  // Avoid compiler warning
    std::string line;
    std::map<int32_t, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_ephemeris_iter;

    // -------- Line 1
    line = std::string(5, ' ');
    line += d_stringVersion;
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem.find("GLONASS")->second;
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    if (d_version == 2)
        {
            line += Rinex_Printer::leftJustify("BLANK OR G = GPS,  R = GLONASS,  E = GALILEO,  M = MIXED", 60);
        }
    if (d_version == 3)
        {
            line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED", 60);
        }
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GLONASS OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER NAME
    line.clear();
    line += Rinex_Printer::leftJustify("DEFAULT MARKER NAME", 60);  // put a flag or a property,
    line += Rinex_Printer::leftJustify("MARKER NAME", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER TYPE
    if (d_version == 2)
        {
            line.clear();
            line += Rinex_Printer::leftJustify("GROUND_CRAFT", 20);  // put a flag or a property
            line += std::string(40, ' ');
            line += Rinex_Printer::leftJustify("MARKER NUMBER", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
        }
    if (d_version == 3)
        {
            line.clear();
            line += Rinex_Printer::leftJustify("GROUND_CRAFT", 20);  // put a flag or a property
            line += std::string(40, ' ');
            line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
        }

    // -------- Line OBSERVER / AGENCY
    line.clear();
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
    line += Rinex_Printer::leftJustify("CTTC", 40);  // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20);           // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20);  // add flag and property
    // line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if (gnss_sdr_version.length() > 20)
        {
            gnss_sdr_version.resize(9, ' ');
        }
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << '\n';

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);    // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

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
    out << line << '\n';

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
    out << line << '\n';

    // -------- SYS / OBS TYPES
    if (d_version == 3)
        {
            // -------- SYS / OBS TYPES
            // one line per available system
            line.clear();
            line += satelliteSystem.find("GLONASS")->second;
            line += std::string(2, ' ');
            std::stringstream strm;
            d_numberTypesObservations = 4;
            strm << d_numberTypesObservations;
            line += Rinex_Printer::rightJustify(strm.str(), 3);

            std::string signal_ = "1G";
            const std::size_t found_1G = glonass_bands.find(signal_);
            signal_ = "2G";
            const std::size_t found_2G = glonass_bands.find(signal_);

            if (found_1G != std::string::npos)
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

            if (found_2G != std::string::npos)
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

            line += std::string(60 - line.size(), ' ');
            line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
        }
    if (d_version == 2)
        {
            // -------- SYS / OBS TYPES
            line.clear();
            std::stringstream strm;
            strm << d_numberTypesObservations;
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
            line += std::string(60 - line.size(), ' ');
            line += Rinex_Printer::leftJustify("# / TYPES OF OBSERV", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
        }

    // -------- Signal Strength units (Only version 3)
    if (d_version == 3)
        {
            // -------- Signal Strength units
            line.clear();
            line += Rinex_Printer::leftJustify("DBHZ", 20);
            line += std::string(40, ' ');
            line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
        }

    // -------- TIME OF FIRST OBS
    const boost::posix_time::ptime p_utc_time = Rinex_Printer::compute_UTC_time(eph, d_TOW_first_observation);
    const std::string timestring = boost::posix_time::to_iso_string(p_utc_time);
    const std::string year(timestring, 0, 4);
    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);
    double intpart = 0;
    const double seconds = p_utc_time.time_of_day().seconds() + modf(d_TOW_first_observation, &intpart);
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
    out << line << '\n';

    // -------- GLONASS SLOT / FRQ # (On;y d_version 3)
    if (d_version == 3)
        {
            // -------- GLONASS SLOT / FRQ #
            // TODO Need to provide system with list of all satellites and update this accordingly
            line.clear();
            line += Rinex_Printer::rightJustify(std::to_string(0), 3);  // Number of satellites in list
            line += std::string(1, ' ');
            line += satelliteSystem.find("GLONASS")->second;
            line += Rinex_Printer::rightJustify(std::to_string(0), 2);  // Slot Number
            line += std::string(1, ' ');
            line += Rinex_Printer::rightJustify(std::to_string(0), 2);  // Frequency Number
            line += std::string(1, ' ');
            line += std::string(60 - line.size(), ' ');
            line += Rinex_Printer::leftJustify("GLONASS SLOT / FRQ #", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

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
            line += std::string(60 - line.size(), ' ');
            line += Rinex_Printer::leftJustify("GLONASS COD/PHS/BIS", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
        }

    // -------- END OF HEADER
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Gps_Ephemeris& gps_eph, const Glonass_Gnav_Ephemeris& glonass_gnav_eph, double d_TOW_first_observation, const std::string& glonass_bands)
{
    if (glonass_gnav_eph.d_m > 0.0)
        {
        }  // avoid warning, not needed
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += d_stringVersion;
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem["Mixed"];
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("MIXED (GPS/GLO) OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER NAME / TYPE
    if (d_version == 2)
        {
            line.clear();
            line += Rinex_Printer::leftJustify("GROUND_CRAFT", 20);  // put a flag or a property
            line += std::string(40, ' ');
            line += Rinex_Printer::leftJustify("MARKER NUMBER", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
        }
    if (d_version == 3)
        {
            line.clear();
            line += Rinex_Printer::leftJustify("GROUND_CRAFT", 20);  // put a flag or a property
            line += std::string(40, ' ');
            line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
        }

    // -------- Line MARKER TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("NON_GEODETIC", 20);  // put a flag or a property
    line += std::string(40, ' ');
    line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line OBSERVER / AGENCY
    line.clear();
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
    line += Rinex_Printer::leftJustify("CTTC", 40);  // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20);           // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20);  // add flag and property
    // line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if (gnss_sdr_version.length() > 20)
        {
            gnss_sdr_version.resize(9, ' ');
        }
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << '\n';

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);    // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

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
    out << line << '\n';

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
    out << line << '\n';

    // -------- SYS / OBS TYPES
    if (d_version == 3)
        {
            // one line per available system
            line.clear();
            line += satelliteSystem.find("GPS")->second;
            line += std::string(2, ' ');
            std::stringstream strm;
            d_numberTypesObservations = 4;
            strm << d_numberTypesObservations;
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
            line += std::string(60 - line.size(), ' ');
            line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            // Find GLONASS Signal in Mixed file
            uint32_t number_of_observations_glo = 0;
            std::string signal_("1G");
            const std::size_t found_1G = glonass_bands.find(signal_);
            if (found_1G != std::string::npos)
                {
                    number_of_observations_glo = number_of_observations_glo + 4;
                }
            signal_ = "2G";
            const std::size_t found_2G = glonass_bands.find(signal_);
            if (found_2G != std::string::npos)
                {
                    number_of_observations_glo = number_of_observations_glo + 4;
                }
            line.clear();
            line += satelliteSystem.find("GLONASS")->second;
            line += std::string(2, ' ');
            line += Rinex_Printer::rightJustify(std::to_string(number_of_observations_glo), 3);
            if (found_1G != std::string::npos)
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
            if (found_2G != std::string::npos)
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
            line += std::string(60 - line.size(), ' ');
            line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
        }
    if (d_version == 2)
        {
            // -------- SYS / OBS TYPES
            line.clear();
            std::stringstream strm;
            strm << d_numberTypesObservations;
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
            line += std::string(60 - line.size(), ' ');
            line += Rinex_Printer::leftJustify("# / TYPES OF OBSERV", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
        }

    // -------- Signal Strength units (only version 3)
    if (d_version == 3)
        {
            line.clear();
            line += Rinex_Printer::leftJustify("DBHZ", 20);
            line += std::string(40, ' ');
            line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
        }

    // -------- TIME OF FIRST OBS
    line.clear();
    const boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(gps_eph, d_TOW_first_observation);
    const std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    const std::string year(timestring, 0, 4);
    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);
    const double gps_t = d_TOW_first_observation;
    const double seconds = fmod(gps_t, 60);
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
    out << line << '\n';

    // -------- GLONASS SLOT / FRQ # (On;y version 3)
    if (d_version == 3)
        {
            // -------- GLONASS SLOT / FRQ #
            // TODO Need to provide system with list of all satellites and update this accordingly
            line.clear();
            line += Rinex_Printer::rightJustify(std::to_string(0), 3);  // Number of satellites in list
            line += std::string(1, ' ');
            line += satelliteSystem.find("GLONASS")->second;
            line += Rinex_Printer::rightJustify(std::to_string(0), 2);  // Slot Number
            line += std::string(1, ' ');
            line += Rinex_Printer::rightJustify(std::to_string(0), 2);  // Frequency Number
            line += std::string(1, ' ');
            line += std::string(60 - line.size(), ' ');
            line += Rinex_Printer::leftJustify("GLONASS SLOT / FRQ #", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

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
            line += std::string(60 - line.size(), ' ');
            line += Rinex_Printer::leftJustify("GLONASS COD/PHS/BIS", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
        }

    // -------- end of header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Gps_CNAV_Ephemeris& gps_cnav_eph, const Glonass_Gnav_Ephemeris& glonass_gnav_eph, double d_TOW_first_observation, const std::string& glonass_bands)
{
    if (glonass_gnav_eph.d_m > 0.0)
        {
        }  // avoid warning, not needed
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += d_stringVersion;
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem["Mixed"];
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("MIXED (GPS/GLO) OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER NAME
    line.clear();
    line += Rinex_Printer::leftJustify("DEFAULT MARKER NAME", 60);  // put a flag or a property,
    line += Rinex_Printer::leftJustify("MARKER NAME", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER NUMBER / TYPE
    if (d_version == 2)
        {
            line.clear();
            line += Rinex_Printer::leftJustify("GROUND_CRAFT", 20);  // put a flag or a property
            line += std::string(40, ' ');
            line += Rinex_Printer::leftJustify("MARKER NUMBER", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
        }
    if (d_version == 3)
        {
            line.clear();
            line += Rinex_Printer::leftJustify("GROUND_CRAFT", 20);  // put a flag or a property
            line += std::string(40, ' ');
            line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
        }

    // -------- Line OBSERVER / AGENCY
    line.clear();
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
    line += Rinex_Printer::leftJustify("CTTC", 40);  // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20);           // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20);  // add flag and property
    // line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if (gnss_sdr_version.length() > 20)
        {
            gnss_sdr_version.resize(9, ' ');
        }
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << '\n';

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);    // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

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
    out << line << '\n';

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
    out << line << '\n';

    // -------- SYS / OBS TYPES
    // one line per available system
    line.clear();
    line += satelliteSystem.find("GPS")->second;
    line += std::string(2, ' ');
    std::stringstream strm;
    d_numberTypesObservations = 4;
    strm << d_numberTypesObservations;
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
    line += std::string(60 - line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // Find GLONASS Signal in Mixed file
    uint32_t number_of_observations_glo = 0;
    std::string signal_("1G");
    const std::size_t found_1G = glonass_bands.find(signal_);
    if (found_1G != std::string::npos)
        {
            number_of_observations_glo = number_of_observations_glo + 4;
        }
    signal_ = "2G";
    const std::size_t found_2G = glonass_bands.find(signal_);
    if (found_2G != std::string::npos)
        {
            number_of_observations_glo = number_of_observations_glo + 4;
        }
    line.clear();
    line += satelliteSystem.find("GLONASS")->second;
    line += std::string(2, ' ');
    line += Rinex_Printer::rightJustify(std::to_string(number_of_observations_glo), 3);
    if (found_1G != std::string::npos)
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
    if (found_2G != std::string::npos)
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
    line += std::string(60 - line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Signal Strength units (only version 3)
    line.clear();
    line += Rinex_Printer::leftJustify("DBHZ", 20);
    line += std::string(40, ' ');
    line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- TIME OF FIRST OBS
    line.clear();
    const boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(gps_cnav_eph, d_TOW_first_observation);
    const std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    const std::string year(timestring, 0, 4);
    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);
    const double gps_t = d_TOW_first_observation;
    const double seconds = fmod(gps_t, 60);
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
    out << line << '\n';

    // -------- GLONASS SLOT / FRQ #
    // TODO Need to provide system with list of all satellites and update this accordingly
    line.clear();
    line += Rinex_Printer::rightJustify(std::to_string(0), 3);  // Number of satellites in list
    line += std::string(1, ' ');
    line += satelliteSystem.find("GLONASS")->second;
    line += Rinex_Printer::rightJustify(std::to_string(0), 2);  // Slot Number
    line += std::string(1, ' ');
    line += Rinex_Printer::rightJustify(std::to_string(0), 2);  // Frequency Number
    line += std::string(1, ' ');
    line += std::string(60 - line.size(), ' ');
    line += Rinex_Printer::leftJustify("GLONASS SLOT / FRQ #", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

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
    line += std::string(60 - line.size(), ' ');
    line += Rinex_Printer::leftJustify("GLONASS COD/PHS/BIS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- end of header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Galileo_Ephemeris& galileo_eph, const Glonass_Gnav_Ephemeris& glonass_gnav_eph, double d_TOW_first_observation, const std::string& galileo_bands, const std::string& glonass_bands)
{
    if (glonass_gnav_eph.d_m > 0.0)
        {
        }  // avoid warning, not needed
    std::string line;
    d_version = 3;

    // -------- Line 1
    line = std::string(5, ' ');
    line += "3.02";
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem["Mixed"];
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("MIXED (GALILEO/GLONASS) OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER NAME
    line.clear();
    line += Rinex_Printer::leftJustify("DEFAULT MARKER NAME", 60);  // put a flag or a property,
    line += Rinex_Printer::leftJustify("MARKER NAME", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("NON_GEODETIC", 20);  // put a flag or a property
    line += std::string(40, ' ');
    line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line OBSERVER / AGENCY
    line.clear();
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
    line += Rinex_Printer::leftJustify("CTTC", 40);  // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20);           // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20);  // add flag and property
    // line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if (gnss_sdr_version.length() > 20)
        {
            gnss_sdr_version.resize(9, ' ');
        }
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << '\n';

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);    // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

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
    out << line << '\n';

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
    out << line << '\n';

    // -------- SYS / OBS TYPES
    line.clear();
    uint32_t number_of_observations_gal = 0;
    std::string signal_("1B");
    const std::size_t found_1B = galileo_bands.find(signal_);
    if (found_1B != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }
    signal_ = "5X";
    const std::size_t found_5X = galileo_bands.find(signal_);
    if (found_5X != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }

    line.clear();
    signal_ = "7X";
    const std::size_t found_7X = galileo_bands.find(signal_);
    if (found_7X != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }

    signal_ = "E6";
    const std::size_t found_E6 = galileo_bands.find(signal_);
    if (found_E6 != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }

    line += satelliteSystem.find("Galileo")->second;
    line += std::string(2, ' ');
    line += Rinex_Printer::rightJustify(std::to_string(number_of_observations_gal), 3);

    if (found_1B != std::string::npos)
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

    if (found_5X != std::string::npos)
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

    if (found_7X != std::string::npos)
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

    if (found_E6 != std::string::npos)
        {
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GALILEO_E56_B"];
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GALILEO_E56_B"];
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GALILEO_E56_B"];
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GALILEO_E56_B"];
        }

    line += std::string(60 - line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    line.clear();
    uint32_t number_of_observations_glo = 0;
    signal_ = "1G";
    const std::size_t found_1G = glonass_bands.find(signal_);
    if (found_1G != std::string::npos)
        {
            number_of_observations_glo = number_of_observations_glo + 4;
        }
    signal_ = "2G";
    const std::size_t found_2G = glonass_bands.find(signal_);
    if (found_2G != std::string::npos)
        {
            number_of_observations_glo = number_of_observations_glo + 4;
        }

    line += satelliteSystem.find("GLONASS")->second;
    line += std::string(2, ' ');
    line += Rinex_Printer::rightJustify(std::to_string(number_of_observations_glo), 3);

    if (found_1G != std::string::npos)
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

    if (found_2G != std::string::npos)
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

    line += std::string(60 - line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Signal Strength units
    line.clear();
    line += Rinex_Printer::leftJustify("DBHZ", 20);
    line += std::string(40, ' ');
    line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- TIME OF FIRST OBS
    line.clear();
    const boost::posix_time::ptime p_galileo_time = Rinex_Printer::compute_Galileo_time(galileo_eph, d_TOW_first_observation);
    const std::string timestring = boost::posix_time::to_iso_string(p_galileo_time);
    const std::string year(timestring, 0, 4);
    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);
    const double galileo_t = d_TOW_first_observation;
    const double seconds = fmod(galileo_t, 60);
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
    out << line << '\n';

    // -------- end of header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Gps_Ephemeris& eph, double d_TOW_first_observation)
{
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += d_stringVersion;
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem.find("GPS")->second;
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    if (d_version == 2)
        {
            line += Rinex_Printer::leftJustify("BLANK OR G = GPS,  R = GLONASS,  E = GALILEO,  M = MIXED", 60);
        }
    if (d_version == 3)
        {
            line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED", 60);
        }
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GPS OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER NAME
    line.clear();
    line += Rinex_Printer::leftJustify("DEFAULT MARKER NAME", 60);  // put a flag or a property,
    line += Rinex_Printer::leftJustify("MARKER NAME", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER TYPE
    // line.clear();
    // line += Rinex_Printer::leftJustify("GROUND_CRAFT", 20); // put a flag or a property
    // line += std::string(40, ' ');
    // line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
    // Rinex_Printer::lengthCheck(line);
    // out << line << '\n';

    // -------- Line OBSERVER / AGENCY
    line.clear();
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
    line += Rinex_Printer::leftJustify("CTTC", 40);  // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20);           // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20);  // add flag and property
    // line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if (gnss_sdr_version.length() > 20)
        {
            gnss_sdr_version.resize(9, ' ');
        }
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << '\n';

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);    // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

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
    out << line << '\n';

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
    out << line << '\n';

    if (d_version == 2)
        {
            // --------- WAVELENGTH FACTOR
            // put here real data!
            line.clear();
            line += Rinex_Printer::rightJustify("1", 6);
            line += Rinex_Printer::rightJustify("1", 6);
            line += std::string(48, ' ');
            line += Rinex_Printer::leftJustify("WAVELENGTH FACT L1/2", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
        }

    if (d_version == 3)
        {
            // -------- SYS / OBS TYPES
            // one line per available system
            line.clear();
            line += satelliteSystem.find("GPS")->second;
            line += std::string(2, ' ');
            std::stringstream strm;
            d_numberTypesObservations = 4;
            strm << d_numberTypesObservations;
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

            line += std::string(60 - line.size(), ' ');
            line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
        }

    if (d_version == 2)
        {
            // -------- SYS / OBS TYPES
            line.clear();
            std::stringstream strm;
            strm << d_numberTypesObservations;
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
            line += std::string(60 - line.size(), ' ');
            line += Rinex_Printer::leftJustify("# / TYPES OF OBSERV", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
        }

    if (d_version == 3)
        {
            // -------- Signal Strength units
            line.clear();
            line += Rinex_Printer::leftJustify("DBHZ", 20);
            line += std::string(40, ' ');
            line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';
        }

    // -------- TIME OF FIRST OBS
    line.clear();
    const boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(eph, d_TOW_first_observation);
    const std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    const std::string year(timestring, 0, 4);
    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);
    const double gps_t = d_TOW_first_observation;
    const double seconds = fmod(gps_t, 60);
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
    out << line << '\n';

    // -------- SYS /PHASE SHIFTS

    // -------- end of header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Gps_CNAV_Ephemeris& eph, double d_TOW_first_observation, const std::string& gps_bands)
{
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += d_stringVersion;
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem.find("GPS")->second;
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GPS OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER NAME
    line.clear();
    line += Rinex_Printer::leftJustify("DEFAULT MARKER NAME", 60);  // put a flag or a property,
    line += Rinex_Printer::leftJustify("MARKER NAME", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER TYPE
    // line.clear();
    // line += Rinex_Printer::leftJustify("GROUND_CRAFT", 20); // put a flag or a property
    // line += std::string(40, ' ');
    // line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
    // Rinex_Printer::lengthCheck(line);
    // out << line << '\n';

    // -------- Line OBSERVER / AGENCY
    line.clear();
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
    line += Rinex_Printer::leftJustify("CTTC", 40);  // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20);           // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20);  // add flag and property
    // line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if (gnss_sdr_version.length() > 20)
        {
            gnss_sdr_version.resize(9, ' ');
        }
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << '\n';

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);    // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

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
    out << line << '\n';

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
    out << line << '\n';

    // -------- SYS / OBS TYPES
    // one line per available system
    line.clear();
    uint32_t number_of_observations_gps = 0;
    std::string signal_("2S");
    const std::size_t found_2S = gps_bands.find(signal_);
    if (found_2S != std::string::npos)
        {
            number_of_observations_gps = number_of_observations_gps + 4;
        }
    signal_ = "L5";
    const std::size_t found_L5 = gps_bands.find(signal_);
    if (found_L5 != std::string::npos)
        {
            number_of_observations_gps = number_of_observations_gps + 4;
        }
    line += satelliteSystem.find("GPS")->second;
    line += std::string(2, ' ');
    std::stringstream strm;
    d_numberTypesObservations = number_of_observations_gps;
    strm << d_numberTypesObservations;
    line += Rinex_Printer::rightJustify(strm.str(), 3);
    // per type of observation

    if (found_2S != std::string::npos)
        {
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
        }

    if (found_L5 != std::string::npos)
        {
            // GPS L5 PSEUDORANGE
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GPS_L5_Q"];
            // GPS L5 PHASE
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GPS_L5_Q"];
            // GPS DOPPLER L5
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GPS_L5_Q"];
            // GPS L5 SIGNAL STRENGTH
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GPS_L5_Q"];
        }
    line += std::string(60 - line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Signal Strength units
    line.clear();
    line += Rinex_Printer::leftJustify("DBHZ", 20);
    line += std::string(40, ' ');
    line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- TIME OF FIRST OBS
    line.clear();
    const boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(eph, d_TOW_first_observation);
    const std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    const std::string year(timestring, 0, 4);
    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);
    const double gps_t = d_TOW_first_observation;
    const double seconds = fmod(gps_t, 60);
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
    out << line << '\n';

    // -------- SYS /PHASE SHIFTS

    // -------- end of header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Gps_Ephemeris& eph, const Gps_CNAV_Ephemeris& eph_cnav, double d_TOW_first_observation, const std::string& gps_bands)
{
    if (eph_cnav.i_0 > 0.0)
        {
        }  // avoid warning, not needed
    std::string line;

    // -------- Line 1
    line = std::string(5, ' ');
    line += d_stringVersion;
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem.find("GPS")->second;
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GPS OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER NAME
    line.clear();
    line += Rinex_Printer::leftJustify("DEFAULT MARKER NAME", 60);  // put a flag or a property,
    line += Rinex_Printer::leftJustify("MARKER NAME", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER TYPE
    // line.clear();
    // line += Rinex_Printer::leftJustify("GROUND_CRAFT", 20); // put a flag or a property
    // line += std::string(40, ' ');
    // line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
    // Rinex_Printer::lengthCheck(line);
    // out << line << '\n';

    // -------- Line OBSERVER / AGENCY
    line.clear();
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
    line += Rinex_Printer::leftJustify("CTTC", 40);  // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20);           // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20);  // add flag and property
    // line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if (gnss_sdr_version.length() > 20)
        {
            gnss_sdr_version.resize(9, ' ');
        }
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << '\n';

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);    // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

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
    out << line << '\n';

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
    out << line << '\n';

    // -------- SYS / OBS TYPES
    // one line per available system
    line.clear();
    uint32_t number_of_observations_gps = 0;
    std::string signal_("1C");
    const std::size_t found_1C = gps_bands.find(signal_);
    if (found_1C != std::string::npos)
        {
            number_of_observations_gps = number_of_observations_gps + 4;
        }
    signal_ = "2S";
    const std::size_t found_2S = gps_bands.find(signal_);
    if (found_2S != std::string::npos)
        {
            number_of_observations_gps = number_of_observations_gps + 4;
        }
    signal_ = "L5";
    const std::size_t found_L5 = gps_bands.find(signal_);
    if (found_L5 != std::string::npos)
        {
            number_of_observations_gps = number_of_observations_gps + 4;
        }
    line += satelliteSystem.find("GPS")->second;
    line += std::string(2, ' ');
    std::stringstream strm;
    d_numberTypesObservations = number_of_observations_gps;
    strm << d_numberTypesObservations;
    line += Rinex_Printer::rightJustify(strm.str(), 3);
    // per type of observation
    if (found_1C != std::string::npos)
        {
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
        }
    if (found_2S != std::string::npos)
        {
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
        }

    if (found_L5 != std::string::npos)
        {
            // GPS L5 PSEUDORANGE
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GPS_L5_Q"];
            // GPS L5 PHASE
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GPS_L5_Q"];
            // GPS DOPPLER L5
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GPS_L5_Q"];
            // GPS L5 SIGNAL STRENGTH
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GPS_L5_Q"];
        }
    line += std::string(60 - line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Signal Strength units
    line.clear();
    line += Rinex_Printer::leftJustify("DBHZ", 20);
    line += std::string(40, ' ');
    line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- TIME OF FIRST OBS
    line.clear();
    const boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(eph, d_TOW_first_observation);
    const std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    const std::string year(timestring, 0, 4);
    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);
    const double gps_t = d_TOW_first_observation;
    const double seconds = fmod(gps_t, 60);
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
    out << line << '\n';

    // -------- SYS /PHASE SHIFTS

    // -------- end of header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Gps_Ephemeris& gps_eph, const Gps_CNAV_Ephemeris& eph_cnav, const Galileo_Ephemeris& galileo_eph, double d_TOW_first_observation, const std::string& gps_bands, const std::string& galileo_bands)
{
    std::string line;
    d_version = 3;
    if (eph_cnav.ecc == 0)
        {
            // avoid warning
        }
    if (galileo_eph.ecc == 0)
        {
            // avoid warning
        }

    // -------- Line 1
    line = std::string(5, ' ');
    line += "3.02";
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem["Mixed"];
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("MIXED (GPS/GALILEO) OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER NAME
    line.clear();
    line += Rinex_Printer::leftJustify("DEFAULT MARKER NAME", 60);  // put a flag or a property,
    line += Rinex_Printer::leftJustify("MARKER NAME", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER TYPE
    // line.clear();
    // line += Rinex_Printer::leftJustify("NON_GEODETIC", 20); // put a flag or a property
    // line += std::string(40, ' ');
    // line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
    // Rinex_Printer::lengthCheck(line);
    // out << line << '\n';

    // -------- Line OBSERVER / AGENCY
    line.clear();
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
    line += Rinex_Printer::leftJustify("CTTC", 40);  // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20);           // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20);  // add flag and property
    // line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if (gnss_sdr_version.length() > 20)
        {
            gnss_sdr_version.resize(9, ' ');
        }
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << '\n';

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);    // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

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
    out << line << '\n';

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
    out << line << '\n';

    // -------- SYS / OBS TYPES
    // one line per available system
    line.clear();
    uint32_t number_of_observations_gps = 0;
    std::string signal_("1C");
    const std::size_t found_1C = gps_bands.find(signal_);
    if (found_1C != std::string::npos)
        {
            number_of_observations_gps = number_of_observations_gps + 4;
        }
    signal_ = "2S";
    const std::size_t found_2S = gps_bands.find(signal_);
    if (found_2S != std::string::npos)
        {
            number_of_observations_gps = number_of_observations_gps + 4;
        }
    signal_ = "L5";
    const std::size_t found_L5 = gps_bands.find(signal_);
    if (found_L5 != std::string::npos)
        {
            number_of_observations_gps = number_of_observations_gps + 4;
        }
    line += satelliteSystem.find("GPS")->second;
    line += std::string(2, ' ');
    std::stringstream strm;
    d_numberTypesObservations = number_of_observations_gps;
    strm << d_numberTypesObservations;
    line += Rinex_Printer::rightJustify(strm.str(), 3);
    // per type of observation
    if (found_1C != std::string::npos)
        {
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
        }
    if (found_2S != std::string::npos)
        {
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
        }
    if (found_L5 != std::string::npos)
        {
            // GPS L5 PSEUDORANGE
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GPS_L5_Q"];
            // GPS L5 PHASE
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GPS_L5_Q"];
            // GPS DOPPLER L5
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GPS_L5_Q"];
            // GPS L5 SIGNAL STRENGTH
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GPS_L5_Q"];
        }
    line += std::string(60 - line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    line.clear();
    uint32_t number_of_observations_gal = 0;
    signal_ = "1B";
    const std::size_t found_1B = galileo_bands.find(signal_);
    if (found_1B != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }
    signal_ = "5X";
    const std::size_t found_5X = galileo_bands.find(signal_);
    if (found_5X != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }
    signal_ = "7X";
    const std::size_t found_7X = galileo_bands.find(signal_);
    if (found_7X != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }

    signal_ = "E6";
    const std::size_t found_E6 = galileo_bands.find(signal_);
    if (found_E6 != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }
    line += satelliteSystem.find("Galileo")->second;
    line += std::string(2, ' ');
    line += Rinex_Printer::rightJustify(std::to_string(number_of_observations_gal), 3);
    if (found_1B != std::string::npos)
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
    if (found_5X != std::string::npos)
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
    if (found_7X != std::string::npos)
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
    if (found_E6 != std::string::npos)
        {
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GALILEO_E56_B"];
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GALILEO_E56_B"];
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GALILEO_E56_B"];
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GALILEO_E56_B"];
        }
    line += std::string(60 - line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Signal Strength units
    line.clear();
    line += Rinex_Printer::leftJustify("DBHZ", 20);
    line += std::string(40, ' ');
    line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- TIME OF FIRST OBS
    line.clear();
    const boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(gps_eph, d_TOW_first_observation);
    const std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    const std::string year(timestring, 0, 4);
    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);
    const double gps_t = d_TOW_first_observation;
    const double seconds = fmod(gps_t, 60);
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
    out << line << '\n';

    // -------- end of header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Gps_CNAV_Ephemeris& eph_cnav, const Galileo_Ephemeris& galileo_eph, double d_TOW_first_observation, const std::string& gps_bands, const std::string& galileo_bands)
{
    std::string line;
    d_version = 3;
    if (galileo_eph.ecc == 0)
        {
            // avoid warning
        }
    // -------- Line 1
    line = std::string(5, ' ');
    line += "3.02";
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem["Mixed"];
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("MIXED (GPS/GALILEO) OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER NAME
    line.clear();
    line += Rinex_Printer::leftJustify("DEFAULT MARKER NAME", 60);  // put a flag or a property,
    line += Rinex_Printer::leftJustify("MARKER NAME", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER TYPE
    // line.clear();
    // line += Rinex_Printer::leftJustify("NON_GEODETIC", 20); // put a flag or a property
    // line += std::string(40, ' ');
    // line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
    // Rinex_Printer::lengthCheck(line);
    // out << line << '\n';

    // -------- Line OBSERVER / AGENCY
    line.clear();
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
    line += Rinex_Printer::leftJustify("CTTC", 40);  // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20);           // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20);  // add flag and property
    // line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if (gnss_sdr_version.length() > 20)
        {
            gnss_sdr_version.resize(9, ' ');
        }
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << '\n';

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);    // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

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
    out << line << '\n';

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
    out << line << '\n';

    // -------- SYS / OBS TYPES
    // one line per available system
    line.clear();
    uint32_t number_of_observations_gps = 0;
    std::string signal_("2S");
    const std::size_t found_2S = gps_bands.find(signal_);
    if (found_2S != std::string::npos)
        {
            number_of_observations_gps = number_of_observations_gps + 4;
        }
    signal_ = "L5";
    const std::size_t found_L5 = gps_bands.find(signal_);
    if (found_L5 != std::string::npos)
        {
            number_of_observations_gps = number_of_observations_gps + 4;
        }
    line += satelliteSystem.find("GPS")->second;
    line += std::string(2, ' ');
    std::stringstream strm;
    d_numberTypesObservations = number_of_observations_gps;
    strm << d_numberTypesObservations;
    line += Rinex_Printer::rightJustify(strm.str(), 3);
    // per type of observation
    if (found_2S != std::string::npos)
        {
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
        }
    if (found_L5 != std::string::npos)
        {
            // GPS L5 PSEUDORANGE
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GPS_L5_Q"];
            // GPS L5 PHASE
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GPS_L5_Q"];
            // GPS DOPPLER L5
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GPS_L5_Q"];
            // GPS L5 SIGNAL STRENGTH
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GPS_L5_Q"];
        }
    line += std::string(60 - line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    line.clear();
    uint32_t number_of_observations_gal = 0;
    signal_ = "1B";
    const std::size_t found_1B = galileo_bands.find(signal_);
    if (found_1B != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }
    signal_ = "5X";
    const std::size_t found_5X = galileo_bands.find(signal_);
    if (found_5X != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }
    signal_ = "7X";
    const std::size_t found_7X = galileo_bands.find(signal_);
    if (found_7X != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }
    signal_ = "E6";
    const std::size_t found_E6 = galileo_bands.find(signal_);
    if (found_E6 != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }
    line += satelliteSystem.find("Galileo")->second;
    line += std::string(2, ' ');
    line += Rinex_Printer::rightJustify(std::to_string(number_of_observations_gal), 3);
    if (found_1B != std::string::npos)
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
    if (found_5X != std::string::npos)
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
    if (found_7X != std::string::npos)
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
    if (found_E6 != std::string::npos)
        {
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GALILEO_E56_B"];
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GALILEO_E56_B"];
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GALILEO_E56_B"];
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GALILEO_E56_B"];
        }

    line += std::string(60 - line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Signal Strength units
    line.clear();
    line += Rinex_Printer::leftJustify("DBHZ", 20);
    line += std::string(40, ' ');
    line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- TIME OF FIRST OBS
    line.clear();
    const boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(eph_cnav, d_TOW_first_observation);
    const std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    const std::string year(timestring, 0, 4);
    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);
    const double gps_t = d_TOW_first_observation;
    const double seconds = fmod(gps_t, 60);
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
    out << line << '\n';

    // -------- end of header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Galileo_Ephemeris& eph, double d_TOW_first_observation, const std::string& bands)
{
    std::string line;
    d_version = 3;

    // -------- Line 1
    line = std::string(5, ' ');
    if (bands.find("E6") != std::string::npos)
        {
            line += "3.05";
        }
    else
        {
            line += "3.02";
        }
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem.find("Galileo")->second;
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("GALILEO OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER NAME
    line.clear();
    line += Rinex_Printer::leftJustify("DEFAULT MARKER NAME", 60);  // put a flag or a property,
    line += Rinex_Printer::leftJustify("MARKER NAME", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER TYPE
    // line.clear();
    // line += Rinex_Printer::leftJustify("NON_GEODETIC", 20); // put a flag or a property
    // line += std::string(40, ' ');
    // line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
    // Rinex_Printer::lengthCheck(line);
    // out << line << '\n';

    // -------- Line OBSERVER / AGENCY
    line.clear();
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
    line += Rinex_Printer::leftJustify("CTTC", 40);  // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20);           // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20);  // add flag and property
    // line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if (gnss_sdr_version.length() > 20)
        {
            gnss_sdr_version.resize(9, ' ');
        }
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << '\n';

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);    // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

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
    out << line << '\n';

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
    out << line << '\n';

    // -------- SYS / OBS TYPES
    // one line per available system
    uint32_t number_of_observations = 0;
    std::string signal_("1B");
    const std::size_t found_1B = bands.find(signal_);
    if (found_1B != std::string::npos)
        {
            number_of_observations = number_of_observations + 4;
        }
    signal_ = "5X";
    const std::size_t found_5X = bands.find(signal_);
    if (found_5X != std::string::npos)
        {
            number_of_observations = number_of_observations + 4;
        }

    signal_ = "7X";
    const std::size_t found_7X = bands.find(signal_);
    if (found_7X != std::string::npos)
        {
            number_of_observations = number_of_observations + 4;
        }

    signal_ = "E6";
    const std::size_t found_E6 = bands.find(signal_);
    if (found_E6 != std::string::npos)
        {
            number_of_observations = number_of_observations + 4;
        }

    line.clear();

    line += satelliteSystem.find("Galileo")->second;
    line += std::string(2, ' ');
    line += Rinex_Printer::rightJustify(std::to_string(number_of_observations), 3);

    if (found_1B != std::string::npos)
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

    if (found_5X != std::string::npos)
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

    if (found_7X != std::string::npos)
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

    if (found_E6 != std::string::npos)
        {
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GALILEO_E56_B"];
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GALILEO_E56_B"];
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GALILEO_E56_B"];
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GALILEO_E56_B"];
        }

    line += std::string(60 - line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Signal Strength units
    line.clear();
    line += Rinex_Printer::leftJustify("DBHZ", 20);
    line += std::string(40, ' ');
    line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- TIME OF FIRST OBS
    line.clear();
    const boost::posix_time::ptime p_galileo_time = Rinex_Printer::compute_Galileo_time(eph, d_TOW_first_observation);
    const std::string timestring = boost::posix_time::to_iso_string(p_galileo_time);
    const std::string year(timestring, 0, 4);
    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);
    const double galileo_t = d_TOW_first_observation;
    const double seconds = fmod(galileo_t, 60);
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
    out << line << '\n';

    // -------- SYS /PHASE SHIFTS

    // -------- end of header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Gps_Ephemeris& gps_eph, const Galileo_Ephemeris& galileo_eph, double d_TOW_first_observation, const std::string& galileo_bands)
{
    if (galileo_eph.ecc > 0.0)
        {
        }  // avoid warning, not needed
    std::string line;
    d_version = 3;

    // -------- Line 1
    line = std::string(5, ' ');
    line += "3.02";
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem["Mixed"];
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("MIXED (GPS/GALILEO) OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER NAME
    line.clear();
    line += Rinex_Printer::leftJustify("DEFAULT MARKER NAME", 60);  // put a flag or a property,
    line += Rinex_Printer::leftJustify("MARKER NAME", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER TYPE
    // line.clear();
    // line += Rinex_Printer::leftJustify("NON_GEODETIC", 20); // put a flag or a property
    // line += std::string(40, ' ');
    // line += Rinex_Printer::leftJustify("MARKER TYPE", 20);
    // Rinex_Printer::lengthCheck(line);
    // out << line << '\n';

    // -------- Line OBSERVER / AGENCY
    line.clear();
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
    line += Rinex_Printer::leftJustify("CTTC", 40);  // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20);           // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20);  // add flag and property
    // line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if (gnss_sdr_version.length() > 20)
        {
            gnss_sdr_version.resize(9, ' ');
        }
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << '\n';

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);    // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

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
    out << line << '\n';

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
    out << line << '\n';

    // -------- SYS / OBS TYPES
    // one line per available system
    line.clear();
    line += satelliteSystem.find("GPS")->second;
    line += std::string(2, ' ');
    std::stringstream strm;
    d_numberTypesObservations = 4;
    strm << d_numberTypesObservations;
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
    line += std::string(60 - line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    line.clear();
    uint32_t number_of_observations_gal = 0;
    std::string signal_("1B");
    const std::size_t found_1B = galileo_bands.find(signal_);
    if (found_1B != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }
    signal_ = "5X";
    const std::size_t found_5X = galileo_bands.find(signal_);
    if (found_5X != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }

    signal_ = "7X";
    const std::size_t found_7X = galileo_bands.find(signal_);
    if (found_7X != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }

    signal_ = "E6";
    const std::size_t found_E6 = galileo_bands.find(signal_);
    if (found_E6 != std::string::npos)
        {
            number_of_observations_gal = number_of_observations_gal + 4;
        }

    line.clear();
    line += satelliteSystem.find("Galileo")->second;
    line += std::string(2, ' ');
    line += Rinex_Printer::rightJustify(std::to_string(number_of_observations_gal), 3);

    if (found_1B != std::string::npos)
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

    if (found_5X != std::string::npos)
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

    if (found_7X != std::string::npos)
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

    if (found_E6 != std::string::npos)
        {
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["GALILEO_E56_B"];
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["GALILEO_E56_B"];
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["GALILEO_E56_B"];
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["GALILEO_E56_B"];
        }

    line += std::string(60 - line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Signal Strength units
    line.clear();
    line += Rinex_Printer::leftJustify("DBHZ", 20);
    line += std::string(40, ' ');
    line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- TIME OF FIRST OBS
    line.clear();
    const boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(gps_eph, d_TOW_first_observation);
    const std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    const std::string year(timestring, 0, 4);
    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);
    const double gps_t = d_TOW_first_observation;
    const double seconds = fmod(gps_t, 60);
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
    out << line << '\n';

    // -------- end of header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::rinex_obs_header(std::fstream& out, const Beidou_Dnav_Ephemeris& eph, double d_TOW_first_observation, const std::string& bands)
{
    std::string line;
    d_version = 3;

    // -------- Line 1
    line = std::string(5, ' ');
    line += "3.02";
    line += std::string(11, ' ');
    line += Rinex_Printer::leftJustify("OBSERVATION DATA", 20);
    line += satelliteSystem.find("Beidou")->second;
    line += std::string(19, ' ');
    line += std::string("RINEX VERSION / TYPE");
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 2
    line.clear();
    line += Rinex_Printer::leftJustify("G = GPS  R = GLONASS  E = GALILEO  C = BEIDOU S = GEO  M = MIXED", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line 3
    line.clear();
    line += Rinex_Printer::getLocalTime();
    line += std::string("PGM / RUN BY / DATE");
    line += std::string(1, ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("BEIDOU OBSERVATION DATA FILE GENERATED BY GNSS-SDR", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    std::string gnss_sdr_version(GNSS_SDR_VERSION);
    line += "GNSS-SDR VERSION ";
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 43);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line COMMENT
    line.clear();
    line += Rinex_Printer::leftJustify("See https://gnss-sdr.org", 60);
    line += Rinex_Printer::leftJustify("COMMENT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line MARKER NAME
    line.clear();
    line += Rinex_Printer::leftJustify("DEFAULT MARKER NAME", 60);  // put a flag or a property,
    line += Rinex_Printer::leftJustify("MARKER NAME", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line OBSERVER / AGENCY
    line.clear();
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
    line += Rinex_Printer::leftJustify("CTTC", 40);  // add flag and property
    line += Rinex_Printer::leftJustify("OBSERVER / AGENCY", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Line  REC / TYPE VERS
    line.clear();
    line += Rinex_Printer::leftJustify("GNSS-SDR", 20);           // add flag and property
    line += Rinex_Printer::leftJustify("Software Receiver", 20);  // add flag and property
    // line += Rinex_Printer::leftJustify(google::VersionString(), 20); // add flag and property
    if (gnss_sdr_version.length() > 20)
        {
            gnss_sdr_version.resize(9, ' ');
        }
    line += Rinex_Printer::leftJustify(gnss_sdr_version, 20);
    line += Rinex_Printer::leftJustify("REC # / TYPE / VERS", 20);
    lengthCheck(line);
    out << line << '\n';

    // -------- ANTENNA TYPE
    line.clear();
    line += Rinex_Printer::leftJustify("Antenna number", 20);  // add flag and property
    line += Rinex_Printer::leftJustify("Antenna type", 20);    // add flag and property
    line += std::string(20, ' ');
    line += Rinex_Printer::leftJustify("ANT # / TYPE", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

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
    out << line << '\n';

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
    out << line << '\n';

    // -------- SYS / OBS TYPES
    // one line per available system
    uint32_t number_of_observations = 0;
    std::string signal_("B1");
    const std::size_t found_B1 = bands.find(signal_);
    if (found_B1 != std::string::npos)
        {
            number_of_observations = number_of_observations + 4;
        }
    signal_ = "B3";
    const std::size_t found_B3 = bands.find(signal_);
    if (found_B3 != std::string::npos)
        {
            number_of_observations = number_of_observations + 4;
        }

    line.clear();

    line += satelliteSystem.find("Beidou")->second;
    line += std::string(2, ' ');
    line += Rinex_Printer::rightJustify(std::to_string(number_of_observations), 3);

    if (found_B1 != std::string::npos)
        {
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["BEIDOU_B1_I"];
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["BEIDOU_B1_I"];
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["BEIDOU_B1_I"];
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["BEIDOU_B1_I"];
        }

    if (found_B3 != std::string::npos)
        {
            line += std::string(1, ' ');
            line += observationType["PSEUDORANGE"];
            line += observationCode["BEIDOU_B3_I"];
            line += std::string(1, ' ');
            line += observationType["CARRIER_PHASE"];
            line += observationCode["BEIDOU_B3_I"];
            line += std::string(1, ' ');
            line += observationType["DOPPLER"];
            line += observationCode["BEIDOU_B3_I"];
            line += std::string(1, ' ');
            line += observationType["SIGNAL_STRENGTH"];
            line += observationCode["BEIDOU_B3_I"];
        }

    line += std::string(60 - line.size(), ' ');
    line += Rinex_Printer::leftJustify("SYS / # / OBS TYPES", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- Signal Strength units
    line.clear();
    line += Rinex_Printer::leftJustify("DBHZ", 20);
    line += std::string(40, ' ');
    line += Rinex_Printer::leftJustify("SIGNAL STRENGTH UNIT", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- TIME OF FIRST OBS
    line.clear();
    const boost::posix_time::ptime p_bds_time = Rinex_Printer::compute_BDS_time(eph, d_TOW_first_observation);
    const std::string timestring = boost::posix_time::to_iso_string(p_bds_time);
    const std::string year(timestring, 0, 4);
    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);
    double beidou_t = d_TOW_first_observation;
    const double seconds = fmod(beidou_t, 60);
    line += Rinex_Printer::rightJustify(year, 6);
    line += Rinex_Printer::rightJustify(month, 6);
    line += Rinex_Printer::rightJustify(day, 6);
    line += Rinex_Printer::rightJustify(hour, 6);
    line += Rinex_Printer::rightJustify(minutes, 6);
    line += Rinex_Printer::rightJustify(asString(seconds, 7), 13);
    line += Rinex_Printer::rightJustify(std::string("BDT"), 8);
    line += std::string(9, ' ');
    line += Rinex_Printer::leftJustify("TIME OF FIRST OBS", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- SYS /PHASE SHIFTS

    // -------- end of header
    line.clear();
    line += std::string(60, ' ');
    line += Rinex_Printer::leftJustify("END OF HEADER", 20);
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';
}


void Rinex_Printer::update_obs_header(std::fstream& out __attribute__((unused)), const Glonass_Gnav_Utc_Model& utc_model) const
{
    if (utc_model.d_N_4 > 0.0)
        {
            // do nothing
        }
}


void Rinex_Printer::update_obs_header(std::fstream& out, const Gps_Utc_Model& utc_model) const
{
    std::vector<std::string> data;
    std::string line_aux;

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
                            if (line_str.find("TIME OF FIRST OBS", 59) != std::string::npos)  // TIME OF FIRST OBS last header annotation might change in the future
                                {
                                    data.push_back(line_str);
                                    line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LS), 6);
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

                    if (d_version == 3)
                        {
                            if (line_str.find("TIME OF FIRST OBS", 59) != std::string::npos)
                                {
                                    data.push_back(line_str);
                                    line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LS), 6);
                                    line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LSF), 6);
                                    line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_LSF), 6);
                                    line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DN), 6);
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
    for (int32_t i = 0; i < static_cast<int32_t>(data.size()) - 1; i++)
        {
            out << data[i] << '\n';
        }
    out.close();
    out.open(obsfilename, std::ios::out | std::ios::in | std::ios::app);
    out.seekp(0, std::ios_base::end);
}


void Rinex_Printer::update_obs_header(std::fstream& out, const Gps_CNAV_Utc_Model& utc_model) const
{
    std::vector<std::string> data;
    std::string line_aux;

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
                    if (line_str.find("TIME OF FIRST OBS", 59) != std::string::npos)
                        {
                            data.push_back(line_str);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LS), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DN), 6);
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
    for (int32_t i = 0; i < static_cast<int32_t>(data.size()) - 1; i++)
        {
            out << data[i] << '\n';
        }
    out.close();
    out.open(obsfilename, std::ios::out | std::ios::in | std::ios::app);
    out.seekp(0, std::ios_base::end);
}


void Rinex_Printer::update_obs_header(std::fstream& out, const Galileo_Utc_Model& galileo_utc_model) const
{
    std::vector<std::string> data;
    std::string line_aux;

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

                    if (line_str.find("TIME OF FIRST OBS", 59) != std::string::npos)
                        {
                            data.push_back(line_str);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.Delta_tLS), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.Delta_tLSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.WN_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(galileo_utc_model.DN), 6);
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
    for (int32_t i = 0; i < static_cast<int32_t>(data.size()) - 1; i++)
        {
            out << data[i] << '\n';
        }
    out.close();
    out.open(obsfilename, std::ios::out | std::ios::in | std::ios::app);
    out.seekp(0, std::ios_base::end);
}


void Rinex_Printer::update_obs_header(std::fstream& out, const Beidou_Dnav_Utc_Model& utc_model) const
{
    std::vector<std::string> data;
    std::string line_aux;

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

                    if (line_str.find("TIME OF FIRST OBS", 59) != std::string::npos)
                        {
                            data.push_back(line_str);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LS), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DeltaT_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.WN_LSF), 6);
                            line_aux += Rinex_Printer::rightJustify(std::to_string(utc_model.DN), 6);
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
    for (int32_t i = 0; i < static_cast<int32_t>(data.size()) - 1; i++)
        {
            out << data[i] << '\n';
        }
    out.close();
    out.open(obsfilename, std::ios::out | std::ios::in | std::ios::app);
    out.seekp(0, std::ios_base::end);
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Glonass_Gnav_Ephemeris& eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables, const std::string& glonass_bands) const
{
    // RINEX observations timestamps are GPS timestamps.
    std::string line;
    double int_sec = 0;

    // Avoid compiler warning
    if (!glonass_bands.empty())
        {
        }

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
            line += Rinex_Printer::asString(utc_sec, 7);
            line += std::string(2, ' ');
            // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
            line += std::string(1, '0');
            // Number of satellites observed in current epoch
            int32_t numSatellitesObserved = 0;
            std::map<int32_t, Gnss_Synchro>::const_iterator observables_iter;
            for (observables_iter = observables.cbegin();
                 observables_iter != observables.cend();
                 observables_iter++)
                {
                    numSatellitesObserved++;
                }
            line += Rinex_Printer::rightJustify(std::to_string(numSatellitesObserved), 3);
            for (observables_iter = observables.cbegin();
                 observables_iter != observables.cend();
                 observables_iter++)
                {
                    line += satelliteSystem.find("GLONASS")->second;
                    if (static_cast<int32_t>(observables_iter->second.PRN) < 10)
                        {
                            line += std::string(1, '0');
                        }
                    line += std::to_string(static_cast<int32_t>(observables_iter->second.PRN));
                }
            // Receiver clock offset (optional)
            // line += rightJustify(asString(clockOffset, 12), 15);
            line += std::string(80 - line.size(), ' ');
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            for (observables_iter = observables.cbegin();
                 observables_iter != observables.cend();
                 observables_iter++)
                {
                    std::string lineObs;
                    lineObs.clear();
                    line.clear();
                    // GLONASS L1 PSEUDORANGE
                    line += std::string(2, ' ');
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(observables_iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);
                    // GLONASS L1 CA PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_phase_rads / TWO_PI, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);
                    // GLONASS L1 CA DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);
                    // GLONASS L1 SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.CN0_dB_hz, 3), 14);
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
            line += Rinex_Printer::asString(utc_sec, 7);
            line += std::string(2, ' ');
            // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
            line += std::string(1, '0');

            // Number of satellites observed in current epoch
            int32_t numSatellitesObserved = 0;
            std::map<int32_t, Gnss_Synchro>::const_iterator observables_iter;
            for (observables_iter = observables.cbegin();
                 observables_iter != observables.cend();
                 observables_iter++)
                {
                    numSatellitesObserved++;
                }
            line += Rinex_Printer::rightJustify(std::to_string(numSatellitesObserved), 3);

            // Receiver clock offset (optional)
            // line += rightJustify(asString(clockOffset, 12), 15);

            line += std::string(80 - line.size(), ' ');
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            for (observables_iter = observables.cbegin();
                 observables_iter != observables.cend();
                 observables_iter++)
                {
                    std::string lineObs;
                    lineObs.clear();
                    lineObs += satelliteSystem.find("GLONASS")->second;
                    if (static_cast<int32_t>(observables_iter->second.PRN) < 10)
                        {
                            lineObs += std::string(1, '0');
                        }
                    lineObs += std::to_string(static_cast<int32_t>(observables_iter->second.PRN));
                    // lineObs += std::string(2, ' ');
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(observables_iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // GLONASS L1 CA PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_phase_rads / TWO_PI, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // GLONASS L1 CA DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }

                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // GLONASS L1 SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.CN0_dB_hz, 3), 14);

                    if (lineObs.size() < 80)
                        {
                            lineObs += std::string(80 - lineObs.size(), ' ');
                        }
                    out << lineObs << '\n';
                }
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Gps_Ephemeris& gps_eph, const Glonass_Gnav_Ephemeris& glonass_gnav_eph, double gps_obs_time, const std::map<int32_t, Gnss_Synchro>& observables) const
{
    if (glonass_gnav_eph.d_m > 0.0)
        {
        }  // avoid warning, not needed
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
            line += Rinex_Printer::asString(second_, 7);
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
            line += Rinex_Printer::asString(seconds, 7);
            line += std::string(2, ' ');
            // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
            line += std::string(1, '0');
        }

    // Number of satellites observed in current epoch
    // Get maps with observations
    std::map<int32_t, Gnss_Synchro> observablesG1C;
    std::map<int32_t, Gnss_Synchro> observablesR1C;
    std::map<int32_t, Gnss_Synchro> observablesR2C;
    std::map<int32_t, Gnss_Synchro>::const_iterator observables_iter;

    for (observables_iter = observables.cbegin();
         observables_iter != observables.cend();
         observables_iter++)
        {
            const std::string system_(&observables_iter->second.System, 1);
            const std::string sig_(observables_iter->second.Signal);
            if ((system_ == "R") && (sig_ == "1G"))
                {
                    observablesR1C.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "R") && (sig_ == "2G"))
                {
                    observablesR2C.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "G") && (sig_ == "1C"))
                {
                    observablesG1C.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
        }

    std::multimap<uint32_t, Gnss_Synchro> total_glo_map;
    std::set<uint32_t> available_glo_prns;
    std::set<uint32_t>::iterator it;
    for (observables_iter = observablesR1C.cbegin();
         observables_iter != observablesR1C.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_glo_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_glo_prns.find(prn_);
            if (it == available_glo_prns.end())
                {
                    available_glo_prns.insert(prn_);
                }
        }

    for (observables_iter = observablesR2C.cbegin();
         observables_iter != observablesR2C.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_glo_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_glo_prns.find(prn_);
            if (it == available_glo_prns.end())
                {
                    available_glo_prns.insert(prn_);
                }
        }

    const int32_t numGloSatellitesObserved = available_glo_prns.size();
    const int32_t numGpsSatellitesObserved = observablesG1C.size();
    const int32_t numSatellitesObserved = numGloSatellitesObserved + numGpsSatellitesObserved;
    line += Rinex_Printer::rightJustify(std::to_string(numSatellitesObserved), 3);
    if (d_version == 2)
        {
            // Add list of GPS satellites
            for (observables_iter = observablesG1C.cbegin();
                 observables_iter != observablesG1C.cend();
                 observables_iter++)
                {
                    line += satelliteSystem.find("GPS")->second;
                    if (static_cast<int32_t>(observables_iter->second.PRN) < 10)
                        {
                            line += std::string(1, '0');
                        }
                    line += std::to_string(static_cast<int32_t>(observables_iter->second.PRN));
                }
            // Add list of GLONASS L1 satellites
            for (observables_iter = observablesR1C.cbegin();
                 observables_iter != observablesR1C.cend();
                 observables_iter++)
                {
                    line += satelliteSystem.find("GLONASS")->second;
                    if (static_cast<int32_t>(observables_iter->second.PRN) < 10)
                        {
                            line += std::string(1, '0');
                        }
                    line += std::to_string(static_cast<int32_t>(observables_iter->second.PRN));
                }
            // Add list of GLONASS L2 satellites
            for (observables_iter = observablesR2C.cbegin();
                 observables_iter != observablesR2C.cend();
                 observables_iter++)
                {
                    line += satelliteSystem.find("GLONASS")->second;
                    if (static_cast<int32_t>(observables_iter->second.PRN) < 10)
                        {
                            line += std::string(1, '0');
                        }
                    line += std::to_string(static_cast<int32_t>(observables_iter->second.PRN));
                }
        }
    line += std::string(80 - line.size(), ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- OBSERVATION record
    std::string s;
    std::string lineObs;
    for (observables_iter = observablesG1C.cbegin();
         observables_iter != observablesG1C.cend();
         observables_iter++)
        {
            lineObs.clear();

            s.assign(1, observables_iter->second.System);
            if (d_version == 3)
                {
                    // Specify system only if in version 3
                    if (s == "G")
                        {
                            lineObs += satelliteSystem.find("GPS")->second;
                        }
                    if (s == "R")
                        {
                            lineObs += satelliteSystem.find("GLONASS")->second;  // should not happen
                        }
                    if (static_cast<int32_t>(observables_iter->second.PRN) < 10)
                        {
                            lineObs += std::string(1, '0');
                        }
                    lineObs += std::to_string(static_cast<int32_t>(observables_iter->second.PRN));
                }

            // Pseudorange Measurements
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Pseudorange_m, 3), 14);

            // Loss of lock indicator (LLI)
            int32_t lli = 0;  // Include in the observation!!
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
            //    }

            // Signal Strength Indicator (SSI)
            const int32_t ssi = Rinex_Printer::signalStrength(observables_iter->second.CN0_dB_hz);
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

            // PHASE
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_phase_rads / TWO_PI, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
            //    }
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

            // DOPPLER
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_Doppler_hz, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
            //    }
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

            // SIGNAL STRENGTH
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.CN0_dB_hz, 3), 14);

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }

    std::pair<std::multimap<uint32_t, Gnss_Synchro>::iterator, std::multimap<uint32_t, Gnss_Synchro>::iterator> ret;
    for (it = available_glo_prns.cbegin();
         it != available_glo_prns.cend();
         it++)
        {
            lineObs.clear();
            if (d_version == 3)
                {
                    lineObs += satelliteSystem.find("GLONASS")->second;
                    if (static_cast<int32_t>(*it) < 10)
                        {
                            lineObs += std::string(1, '0');
                        }
                    lineObs += std::to_string(static_cast<int32_t>(*it));
                }
            ret = total_glo_map.equal_range(*it);
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    /// \todo Need to account for pseudorange correction for glonass
                    // double leap_seconds = Rinex_Printer::get_leap_second(glonass_gnav_eph, gps_obs_time);
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // GLONASS CARRIER PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // GLONASS  DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // GLONASS SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Gps_CNAV_Ephemeris& gps_eph, const Glonass_Gnav_Ephemeris& glonass_gnav_eph, double gps_obs_time, const std::map<int32_t, Gnss_Synchro>& observables) const
{
    if (glonass_gnav_eph.d_m > 0.0)
        {
        }  // avoid warning, not needed
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

    // Number of satellites observed in current epoch
    // Get maps with observations
    std::map<int32_t, Gnss_Synchro> observablesG2S;
    std::map<int32_t, Gnss_Synchro> observablesR1C;
    std::map<int32_t, Gnss_Synchro> observablesR2C;
    std::map<int32_t, Gnss_Synchro>::const_iterator observables_iter;

    for (observables_iter = observables.cbegin();
         observables_iter != observables.cend();
         observables_iter++)
        {
            const std::string system_(&observables_iter->second.System, 1);
            const std::string sig_(observables_iter->second.Signal);
            if ((system_ == "R") && (sig_ == "1G"))
                {
                    observablesR1C.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "R") && (sig_ == "2G"))
                {
                    observablesR2C.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "G") && (sig_ == "2S"))
                {
                    observablesG2S.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
        }

    std::multimap<uint32_t, Gnss_Synchro> total_glo_map;
    std::set<uint32_t> available_glo_prns;
    std::set<uint32_t>::iterator it;
    for (observables_iter = observablesR1C.cbegin();
         observables_iter != observablesR1C.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_glo_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_glo_prns.find(prn_);
            if (it == available_glo_prns.end())
                {
                    available_glo_prns.insert(prn_);
                }
        }

    for (observables_iter = observablesR2C.cbegin();
         observables_iter != observablesR2C.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_glo_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_glo_prns.find(prn_);
            if (it == available_glo_prns.end())
                {
                    available_glo_prns.insert(prn_);
                }
        }

    const int32_t numGloSatellitesObserved = available_glo_prns.size();
    const int32_t numGpsSatellitesObserved = observablesG2S.size();
    const int32_t numSatellitesObserved = numGloSatellitesObserved + numGpsSatellitesObserved;
    line += Rinex_Printer::rightJustify(std::to_string(numSatellitesObserved), 3);

    line += std::string(80 - line.size(), ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    // -------- OBSERVATION record
    std::string s;
    std::string lineObs;
    for (observables_iter = observablesG2S.cbegin();
         observables_iter != observablesG2S.cend();
         observables_iter++)
        {
            lineObs.clear();

            s.assign(1, observables_iter->second.System);
            // Specify system only if in version 3
            if (s == "G")
                {
                    lineObs += satelliteSystem.find("GPS")->second;
                }
            if (s == "R")
                {
                    lineObs += satelliteSystem.find("GLONASS")->second;  // should not happen
                }
            if (static_cast<int32_t>(observables_iter->second.PRN) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(observables_iter->second.PRN));

            // Pseudorange Measurements
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Pseudorange_m, 3), 14);

            // Loss of lock indicator (LLI)
            int32_t lli = 0;  // Include in the observation!!
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
            //    }

            // Signal Strength Indicator (SSI)
            const int32_t ssi = Rinex_Printer::signalStrength(observables_iter->second.CN0_dB_hz);
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

            // PHASE
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_phase_rads / TWO_PI, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
            //    }
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

            // DOPPLER
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_Doppler_hz, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
            //    }
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

            // SIGNAL STRENGTH
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.CN0_dB_hz, 3), 14);

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }

    std::pair<std::multimap<uint32_t, Gnss_Synchro>::iterator, std::multimap<uint32_t, Gnss_Synchro>::iterator> ret;
    for (it = available_glo_prns.cbegin();
         it != available_glo_prns.cend();
         it++)
        {
            lineObs.clear();
            lineObs += satelliteSystem.find("GLONASS")->second;
            if (static_cast<int32_t>(*it) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(*it));

            ret = total_glo_map.equal_range(*it);
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    /// \todo Need to account for pseudorange correction for glonass
                    // double leap_seconds = Rinex_Printer::get_leap_second(glonass_gnav_eph, gps_obs_time);
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // GLONASS CARRIER PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // GLONASS  DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // GLONASS SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Galileo_Ephemeris& galileo_eph, const Glonass_Gnav_Ephemeris& glonass_gnav_eph, double galileo_obs_time, const std::map<int32_t, Gnss_Synchro>& observables) const
{
    if (glonass_gnav_eph.d_m > 0.0)
        {
        }  // avoid warning, not needed
    std::string line;

    const boost::posix_time::ptime p_galileo_time = Rinex_Printer::compute_Galileo_time(galileo_eph, galileo_obs_time);
    const std::string timestring = boost::posix_time::to_iso_string(p_galileo_time);
    // double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    // const double gps_t = eph.sv_clock_correction(obs_time);
    const double galileo_t = galileo_obs_time;

    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);

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
    const double seconds = fmod(galileo_t, 60);
    // Add extra 0 if seconds are < 10
    if (seconds < 10)
        {
            line += std::string(1, '0');
        }
    line += Rinex_Printer::asString(seconds, 7);
    line += std::string(2, ' ');
    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    // Number of satellites observed in current epoch

    // Get maps with observations
    std::map<int32_t, Gnss_Synchro> observablesE1B;
    std::map<int32_t, Gnss_Synchro> observablesR1C;
    std::map<int32_t, Gnss_Synchro> observablesR2C;
    std::map<int32_t, Gnss_Synchro>::const_iterator observables_iter;

    for (observables_iter = observables.cbegin();
         observables_iter != observables.cend();
         observables_iter++)
        {
            const std::string system_(&observables_iter->second.System, 1);
            const std::string sig_(observables_iter->second.Signal);
            if ((system_ == "R") && (sig_ == "1G"))
                {
                    observablesR1C.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "R") && (sig_ == "2G"))
                {
                    observablesR2C.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "E") && (sig_ == "1B"))
                {
                    observablesE1B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
        }

    std::multimap<uint32_t, Gnss_Synchro> total_glo_map;
    std::set<uint32_t> available_glo_prns;
    std::set<uint32_t>::iterator it;
    for (observables_iter = observablesR1C.cbegin();
         observables_iter != observablesR1C.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_glo_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_glo_prns.find(prn_);
            if (it == available_glo_prns.end())
                {
                    available_glo_prns.insert(prn_);
                }
        }
    for (observables_iter = observablesR2C.cbegin();
         observables_iter != observablesR2C.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_glo_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_glo_prns.find(prn_);
            if (it == available_glo_prns.end())
                {
                    available_glo_prns.insert(prn_);
                }
        }

    const int32_t numGloSatellitesObserved = available_glo_prns.size();
    const int32_t numGalSatellitesObserved = observablesE1B.size();
    const int32_t numSatellitesObserved = numGalSatellitesObserved + numGloSatellitesObserved;
    line += Rinex_Printer::rightJustify(std::to_string(numSatellitesObserved), 3);

    // Receiver clock offset (optional)
    // line += rightJustify(asString(clockOffset, 12), 15);

    line += std::string(80 - line.size(), ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    std::string s;
    std::string lineObs;
    for (observables_iter = observablesE1B.cbegin();
         observables_iter != observablesE1B.cend();
         observables_iter++)
        {
            lineObs.clear();

            s.assign(1, observables_iter->second.System);
            if (s == "E")
                {
                    lineObs += satelliteSystem.find("Galileo")->second;
                }
            if (s == "R")
                {
                    lineObs += satelliteSystem.find("GLONASS")->second;  // should not happen
                }
            if (static_cast<int32_t>(observables_iter->second.PRN) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(observables_iter->second.PRN));
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Pseudorange_m, 3), 14);

            // Loss of lock indicator (LLI)
            int32_t lli = 0;  // Include in the observation!!
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
            //    }

            // Signal Strength Indicator (SSI)
            const int32_t ssi = Rinex_Printer::signalStrength(observables_iter->second.CN0_dB_hz);
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

            // PHASE
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_phase_rads / TWO_PI, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
            //    }
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

            // DOPPLER
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_Doppler_hz, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
            //    }
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

            // SIGNAL STRENGTH
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.CN0_dB_hz, 3), 14);

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }

    std::pair<std::multimap<uint32_t, Gnss_Synchro>::iterator, std::multimap<uint32_t, Gnss_Synchro>::iterator> ret;
    for (it = available_glo_prns.cbegin();
         it != available_glo_prns.cend();
         it++)
        {
            lineObs.clear();
            lineObs += satelliteSystem.find("Galileo")->second;
            if (static_cast<int32_t>(*it) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(*it));
            ret = total_glo_map.equal_range(*it);
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // GLONASS CARRIER PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // GLONASS  DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //   }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // GLONASS SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
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
            line += Rinex_Printer::asString(second_, 7);
            line += std::string(2, ' ');
            // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
            line += std::string(1, '0');
            // Number of satellites observed in current epoch
            int32_t numSatellitesObserved = 0;
            std::map<int32_t, Gnss_Synchro>::const_iterator observables_iter;
            for (observables_iter = observables.cbegin();
                 observables_iter != observables.cend();
                 observables_iter++)
                {
                    numSatellitesObserved++;
                }
            line += Rinex_Printer::rightJustify(std::to_string(numSatellitesObserved), 3);
            for (observables_iter = observables.cbegin();
                 observables_iter != observables.cend();
                 observables_iter++)
                {
                    line += satelliteSystem.find("GPS")->second;
                    if (static_cast<int32_t>(observables_iter->second.PRN) < 10)
                        {
                            line += std::string(1, '0');
                        }
                    line += std::to_string(static_cast<int32_t>(observables_iter->second.PRN));
                }
            // Receiver clock offset (optional)
            // line += rightJustify(asString(clockOffset, 12), 15);
            line += std::string(80 - line.size(), ' ');
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            for (observables_iter = observables.cbegin();
                 observables_iter != observables.cend();
                 observables_iter++)
                {
                    std::string lineObs;
                    lineObs.clear();
                    line.clear();
                    // GPS L1 PSEUDORANGE
                    line += std::string(2, ' ');
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(observables_iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);
                    // GPS L1 CA PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_phase_rads / TWO_PI, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);
                    // GPS L1 CA DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //       lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //   }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);
                    // GPS L1 SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.CN0_dB_hz, 3), 14);
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
            line += Rinex_Printer::asString(seconds, 7);
            line += std::string(2, ' ');
            // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
            line += std::string(1, '0');

            // Number of satellites observed in current epoch
            int32_t numSatellitesObserved = 0;
            std::map<int32_t, Gnss_Synchro>::const_iterator observables_iter;
            for (observables_iter = observables.cbegin();
                 observables_iter != observables.cend();
                 observables_iter++)
                {
                    numSatellitesObserved++;
                }
            line += Rinex_Printer::rightJustify(std::to_string(numSatellitesObserved), 3);

            // Receiver clock offset (optional)
            // line += rightJustify(asString(clockOffset, 12), 15);

            line += std::string(80 - line.size(), ' ');
            Rinex_Printer::lengthCheck(line);
            out << line << '\n';

            for (observables_iter = observables.cbegin();
                 observables_iter != observables.cend();
                 observables_iter++)
                {
                    std::string lineObs;
                    lineObs.clear();
                    lineObs += satelliteSystem.find("GPS")->second;
                    if (static_cast<int32_t>(observables_iter->second.PRN) < 10)
                        {
                            lineObs += std::string(1, '0');
                        }
                    lineObs += std::to_string(static_cast<int32_t>(observables_iter->second.PRN));
                    // lineObs += std::string(2, ' ');
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(observables_iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // GPS L1 CA PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_phase_rads / TWO_PI, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // GPS L1 CA DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }

                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // GPS L1 SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.CN0_dB_hz, 3), 14);

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

    const boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(eph, obs_time);
    const std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    // double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    // const double gps_t = eph.sv_clock_correction(obs_time);
    const double gps_t = obs_time;

    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);

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
    line += Rinex_Printer::asString(seconds, 7);
    line += std::string(2, ' ');
    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    // Number of satellites observed in current epoch
    int32_t numSatellitesObserved = 0;
    std::map<int32_t, Gnss_Synchro>::const_iterator observables_iter;
    for (observables_iter = observables.cbegin();
         observables_iter != observables.cend();
         observables_iter++)
        {
            numSatellitesObserved++;
        }
    line += Rinex_Printer::rightJustify(std::to_string(numSatellitesObserved), 3);

    // Receiver clock offset (optional)
    // line += rightJustify(asString(clockOffset, 12), 15);

    line += std::string(80 - line.size(), ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    for (observables_iter = observables.cbegin();
         observables_iter != observables.cend();
         observables_iter++)
        {
            std::string lineObs;
            lineObs.clear();
            lineObs += satelliteSystem.find("GPS")->second;
            if (static_cast<int32_t>(observables_iter->second.PRN) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(observables_iter->second.PRN));
            // lineObs += std::string(2, ' ');
            // GPS L2 PSEUDORANGE
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Pseudorange_m, 3), 14);

            // Loss of lock indicator (LLI)
            int32_t lli = 0;  // Include in the observation!!
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //       lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
            //   }

            // Signal Strength Indicator (SSI)
            const int32_t ssi = Rinex_Printer::signalStrength(observables_iter->second.CN0_dB_hz);
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

            // GPS L2 PHASE
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_phase_rads / TWO_PI, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
            //    }
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

            // GPS L2 DOPPLER
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_Doppler_hz, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
            //   }

            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

            // GPS L2 SIGNAL STRENGTH
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.CN0_dB_hz, 3), 14);

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Gps_Ephemeris& eph, const Gps_CNAV_Ephemeris& eph_cnav, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables, bool triple_band) const
{
    if (eph_cnav.i_0 > 0.0)
        {
        }  // avoid warning, not needed
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
    line += Rinex_Printer::asString(seconds, 7);
    line += std::string(2, ' ');
    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    // Number of satellites observed in current epoch

    // Get maps with GPS L1 and L2 observations
    std::map<int32_t, Gnss_Synchro> observablesL1;
    std::map<int32_t, Gnss_Synchro> observablesL2;
    std::map<int32_t, Gnss_Synchro> observablesL5;
    std::map<int32_t, Gnss_Synchro>::const_iterator observables_iter;

    std::multimap<uint32_t, Gnss_Synchro> total_mmap;
    std::multimap<uint32_t, Gnss_Synchro>::iterator mmap_iter;
    for (observables_iter = observables.cbegin();
         observables_iter != observables.cend();
         observables_iter++)
        {
            const std::string system_(&observables_iter->second.System, 1);
            const std::string sig_(observables_iter->second.Signal);
            if ((system_ == "G") && (sig_ == "1C"))
                {
                    observablesL1.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                    total_mmap.insert(std::pair<uint32_t, Gnss_Synchro>(observables_iter->second.PRN, observables_iter->second));
                }
            if ((system_ == "G") && (sig_ == "2S"))
                {
                    observablesL2.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                    mmap_iter = total_mmap.find(observables_iter->second.PRN);
                    if (mmap_iter == total_mmap.end())
                        {
                            Gnss_Synchro gs = Gnss_Synchro();
                            total_mmap.insert(std::pair<uint32_t, Gnss_Synchro>(observables_iter->second.PRN, gs));
                        }
                    total_mmap.insert(std::pair<uint32_t, Gnss_Synchro>(observables_iter->second.PRN, observables_iter->second));
                }

            if ((system_ == "G") && (sig_ == "L5"))
                {
                    observablesL5.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                    mmap_iter = total_mmap.find(observables_iter->second.PRN);
                    if (mmap_iter == total_mmap.end())
                        {
                            Gnss_Synchro gs = Gnss_Synchro();
                            total_mmap.insert(std::pair<uint32_t, Gnss_Synchro>(observables_iter->second.PRN, gs));
                        }
                    total_mmap.insert(std::pair<uint32_t, Gnss_Synchro>(observables_iter->second.PRN, observables_iter->second));
                }
        }

    // Fill with zeros satellites with L1 obs but not L2
    std::multimap<uint32_t, Gnss_Synchro> mmap_aux;
    mmap_aux = total_mmap;
    for (mmap_iter = mmap_aux.begin();
         mmap_iter != mmap_aux.end();
         mmap_iter++)
        {
            if ((total_mmap.count(mmap_iter->second.PRN)) == 1 && (mmap_iter->second.PRN != 0))
                {
                    Gnss_Synchro gs = Gnss_Synchro();
                    gs.System = 'G';
                    gs.Signal[0] = '2';
                    gs.Signal[1] = 'S';
                    gs.Signal[2] = '\0';
                    gs.PRN = mmap_iter->second.PRN;
                    total_mmap.insert(std::pair<uint32_t, Gnss_Synchro>(mmap_iter->second.PRN, gs));
                }
        }

    std::set<uint32_t> available_prns;
    std::set<uint32_t>::iterator it;
    for (observables_iter = observablesL1.cbegin();
         observables_iter != observablesL1.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            it = available_prns.find(prn_);
            if (it == available_prns.end())
                {
                    available_prns.insert(prn_);
                }
        }

    for (observables_iter = observablesL2.cbegin();
         observables_iter != observablesL2.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            it = available_prns.find(prn_);
            if (it == available_prns.end())
                {
                    available_prns.insert(prn_);
                }
        }

    for (observables_iter = observablesL5.cbegin();
         observables_iter != observablesL5.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            it = available_prns.find(prn_);
            if (it == available_prns.end())
                {
                    available_prns.insert(prn_);
                }
        }

    const int32_t numSatellitesObserved = available_prns.size();
    line += Rinex_Printer::rightJustify(std::to_string(numSatellitesObserved), 3);
    // Receiver clock offset (optional)
    // line += rightJustify(asString(clockOffset, 12), 15);
    line += std::string(80 - line.size(), ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    std::string lineObs;
    std::pair<std::multimap<uint32_t, Gnss_Synchro>::iterator, std::multimap<uint32_t, Gnss_Synchro>::iterator> ret;
    for (it = available_prns.cbegin();
         it != available_prns.cend();
         it++)
        {
            lineObs.clear();
            lineObs += satelliteSystem.find("GPS")->second;
            if (static_cast<int32_t>(*it) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(*it));
            ret = total_mmap.equal_range(*it);
            bool have_l2 = false;
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    const std::string sig_(iter->second.Signal);
                    if (sig_ == "2S")
                        {
                            have_l2 = true;
                        }
                    if (triple_band and sig_ == "L5" and have_l2 == false)
                        {
                            lineObs += std::string(62, ' ');
                        }

                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //   {
                    //       lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //   }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // GPS CARRIER PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // GPS  DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // GPS SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
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

    const boost::posix_time::ptime p_galileo_time = Rinex_Printer::compute_Galileo_time(eph, obs_time);
    const std::string timestring = boost::posix_time::to_iso_string(p_galileo_time);
    // double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    // const double gps_t = eph.sv_clock_correction(obs_time);
    const double galileo_t = obs_time;

    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);

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
    const double seconds = fmod(galileo_t, 60);
    // Add extra 0 if seconds are < 10
    if (seconds < 10)
        {
            line += std::string(1, '0');
        }
    line += Rinex_Printer::asString(seconds, 7);
    line += std::string(2, ' ');
    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    // Number of satellites observed in current epoch

    // Get maps with Galileo observations
    std::map<int32_t, Gnss_Synchro> observablesE1B;
    std::map<int32_t, Gnss_Synchro> observablesE5A;
    std::map<int32_t, Gnss_Synchro> observablesE5B;
    std::map<int32_t, Gnss_Synchro> observablesE6B;
    std::map<int32_t, Gnss_Synchro>::const_iterator observables_iter;

    for (observables_iter = observables.cbegin();
         observables_iter != observables.cend();
         observables_iter++)
        {
            const std::string system_(&observables_iter->second.System, 1);
            const std::string sig_(observables_iter->second.Signal);
            if ((system_ == "E") && (sig_ == "1B"))
                {
                    observablesE1B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "E") && (sig_ == "5X"))
                {
                    observablesE5A.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "E") && (sig_ == "7X"))
                {
                    observablesE5B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "E") && (sig_ == "E6"))
                {
                    observablesE6B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
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
            for (observables_iter = observablesE1B.cbegin();
                 observables_iter != observablesE1B.cend();
                 observables_iter++)
                {
                    const uint32_t prn_ = observables_iter->second.PRN;
                    total_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
                    it = available_prns.find(prn_);
                    if (it == available_prns.end())
                        {
                            available_prns.insert(prn_);
                        }
                }
        }
    if (found_E5a != std::string::npos)
        {
            for (observables_iter = observablesE5A.cbegin();
                 observables_iter != observablesE5A.cend();
                 observables_iter++)
                {
                    const uint32_t prn_ = observables_iter->second.PRN;
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
                    total_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
                }
        }
    if (found_E5b != std::string::npos)
        {
            for (observables_iter = observablesE5B.cbegin();
                 observables_iter != observablesE5B.cend();
                 observables_iter++)
                {
                    const uint32_t prn_ = observables_iter->second.PRN;
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
                    total_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
                }
        }

    if (found_E6b != std::string::npos)
        {
            for (observables_iter = observablesE6B.cbegin();
                 observables_iter != observablesE6B.cend();
                 observables_iter++)
                {
                    const uint32_t prn_ = observables_iter->second.PRN;
                    total_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
                    it = available_prns.find(prn_);
                    if (it == available_prns.end())
                        {
                            available_prns.insert(prn_);
                        }
                }
        }

    const int32_t numSatellitesObserved = available_prns.size();
    line += Rinex_Printer::rightJustify(std::to_string(numSatellitesObserved), 3);
    // Receiver clock offset (optional)
    // line += rightJustify(asString(clockOffset, 12), 15);
    line += std::string(80 - line.size(), ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    std::string lineObs;
    std::pair<std::multimap<uint32_t, Gnss_Synchro>::iterator, std::multimap<uint32_t, Gnss_Synchro>::iterator> ret;
    for (it = available_prns.cbegin();
         it != available_prns.cend();
         it++)
        {
            lineObs.clear();
            lineObs += satelliteSystem.find("Galileo")->second;
            if (static_cast<int32_t>(*it) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(*it));
            ret = total_map.equal_range(*it);
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //   }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // Galileo CARRIER PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // Galileo  DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //       lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // Galileo SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Gps_Ephemeris& gps_eph, const Galileo_Ephemeris& galileo_eph, double gps_obs_time, const std::map<int32_t, Gnss_Synchro>& observables) const
{
    if (galileo_eph.ecc > 0.0)
        {
        }  // avoid warning, not needed
    std::string line;

    const boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(gps_eph, gps_obs_time);
    const std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    // double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    // double gps_t = eph.sv_clock_correction(obs_time);
    const double gps_t = gps_obs_time;

    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);

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
    line += Rinex_Printer::asString(seconds, 7);
    line += std::string(2, ' ');
    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    // Number of satellites observed in current epoch

    // Get maps with observations
    std::map<int32_t, Gnss_Synchro> observablesG1C;
    std::map<int32_t, Gnss_Synchro> observablesE1B;
    std::map<int32_t, Gnss_Synchro> observablesE5A;
    std::map<int32_t, Gnss_Synchro> observablesE5B;
    std::map<int32_t, Gnss_Synchro> observablesE6B;
    std::map<int32_t, Gnss_Synchro>::const_iterator observables_iter;

    for (observables_iter = observables.cbegin();
         observables_iter != observables.cend();
         observables_iter++)
        {
            const std::string system_(&observables_iter->second.System, 1);
            const std::string sig_(observables_iter->second.Signal);
            if ((system_ == "E") && (sig_ == "1B"))
                {
                    observablesE1B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "E") && (sig_ == "5X"))
                {
                    observablesE5A.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "E") && (sig_ == "7X"))
                {
                    observablesE5B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "E") && (sig_ == "E6"))
                {
                    observablesE6B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "G") && (sig_ == "1C"))
                {
                    observablesG1C.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
        }

    std::multimap<uint32_t, Gnss_Synchro> total_gal_map;
    std::set<uint32_t> available_gal_prns;
    std::set<uint32_t>::iterator it;
    for (observables_iter = observablesE1B.cbegin();
         observables_iter != observablesE1B.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (observables_iter = observablesE5A.cbegin();
         observables_iter != observablesE5A.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (observables_iter = observablesE5B.cbegin();
         observables_iter != observablesE5B.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (observables_iter = observablesE6B.cbegin();
         observables_iter != observablesE6B.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    const int32_t numGalSatellitesObserved = available_gal_prns.size();
    const int32_t numGpsSatellitesObserved = observablesG1C.size();
    const int32_t numSatellitesObserved = numGalSatellitesObserved + numGpsSatellitesObserved;
    line += Rinex_Printer::rightJustify(std::to_string(numSatellitesObserved), 3);

    // Receiver clock offset (optional)
    // line += rightJustify(asString(clockOffset, 12), 15);

    line += std::string(80 - line.size(), ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    std::string s;
    std::string lineObs;
    for (observables_iter = observablesG1C.cbegin();
         observables_iter != observablesG1C.cend();
         observables_iter++)
        {
            lineObs.clear();

            s.assign(1, observables_iter->second.System);
            if (s == "G")
                {
                    lineObs += satelliteSystem.find("GPS")->second;
                }
            if (s == "E")
                {
                    lineObs += satelliteSystem.find("Galileo")->second;  // should not happen
                }
            if (static_cast<int32_t>(observables_iter->second.PRN) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(observables_iter->second.PRN));
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Pseudorange_m, 3), 14);

            // Loss of lock indicator (LLI)
            int32_t lli = 0;  // Include in the observation!!
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //       lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
            //    }

            // Signal Strength Indicator (SSI)
            const int32_t ssi = Rinex_Printer::signalStrength(observables_iter->second.CN0_dB_hz);
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

            // PHASE
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_phase_rads / TWO_PI, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
            //   }
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

            // DOPPLER
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.Carrier_Doppler_hz, 3), 14);
            if (lli == 0)
                {
                    lineObs += std::string(1, ' ');
                }
            // else
            //    {
            //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
            //    }
            lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

            // SIGNAL STRENGTH
            lineObs += Rinex_Printer::rightJustify(asString(observables_iter->second.CN0_dB_hz, 3), 14);

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }

    std::pair<std::multimap<uint32_t, Gnss_Synchro>::iterator, std::multimap<uint32_t, Gnss_Synchro>::iterator> ret;
    for (it = available_gal_prns.cbegin();
         it != available_gal_prns.cend();
         it++)
        {
            lineObs.clear();
            lineObs += satelliteSystem.find("Galileo")->second;
            if (static_cast<int32_t>(*it) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(*it));
            ret = total_gal_map.equal_range(*it);
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // Galileo CARRIER PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // Galileo  DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // Galileo SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            if (lineObs.size() < 80)
                {
                    lineObs += std::string(80 - lineObs.size(), ' ');
                }
            out << lineObs << '\n';
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Gps_CNAV_Ephemeris& eph, const Galileo_Ephemeris& galileo_eph, double gps_obs_time, const std::map<int32_t, Gnss_Synchro>& observables) const
{
    if (galileo_eph.ecc > 0.0)
        {
        }  // avoid warning, not needed
    std::string line;

    const boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(eph, gps_obs_time);
    const std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    // double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    // double gps_t = eph.sv_clock_correction(obs_time);
    const double gps_t = gps_obs_time;

    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);

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
    line += Rinex_Printer::asString(seconds, 7);
    line += std::string(2, ' ');
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
    std::map<int32_t, Gnss_Synchro>::const_iterator observables_iter;

    for (observables_iter = observables.cbegin();
         observables_iter != observables.cend();
         observables_iter++)
        {
            const std::string system_(&observables_iter->second.System, 1);
            const std::string sig_(observables_iter->second.Signal);
            if ((system_ == "E") && (sig_ == "1B"))
                {
                    observablesE1B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "E") && (sig_ == "5X"))
                {
                    observablesE5A.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "E") && (sig_ == "7X"))
                {
                    observablesE5B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "E") && (sig_ == "E6"))
                {
                    observablesE6B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "G") && (sig_ == "2S"))
                {
                    observablesG2S.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "G") && (sig_ == "L5"))
                {
                    observablesGL5.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
        }

    std::multimap<uint32_t, Gnss_Synchro> total_gps_map;
    std::multimap<uint32_t, Gnss_Synchro> total_gal_map;
    std::set<uint32_t> available_gal_prns;
    std::set<uint32_t> available_gps_prns;
    std::set<uint32_t>::iterator it;
    for (observables_iter = observablesE1B.cbegin();
         observables_iter != observablesE1B.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (observables_iter = observablesE5A.cbegin();
         observables_iter != observablesE5A.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (observables_iter = observablesE5B.cbegin();
         observables_iter != observablesE5B.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (observables_iter = observablesE6B.cbegin();
         observables_iter != observablesE6B.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (observables_iter = observablesG2S.cbegin();
         observables_iter != observablesG2S.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_gps_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_gps_prns.find(prn_);
            if (it == available_gps_prns.end())
                {
                    available_gps_prns.insert(prn_);
                }
        }

    for (observables_iter = observablesGL5.cbegin();
         observables_iter != observablesGL5.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_gps_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_gps_prns.find(prn_);
            if (it == available_gps_prns.end())
                {
                    available_gps_prns.insert(prn_);
                }
        }

    const int32_t numGalSatellitesObserved = available_gal_prns.size();
    const int32_t numGpsSatellitesObserved = available_gps_prns.size();
    const int32_t numSatellitesObserved = numGalSatellitesObserved + numGpsSatellitesObserved;
    line += Rinex_Printer::rightJustify(std::to_string(numSatellitesObserved), 3);

    // Receiver clock offset (optional)
    // line += rightJustify(asString(clockOffset, 12), 15);

    line += std::string(80 - line.size(), ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    std::string s;
    std::string lineObs;

    std::pair<std::multimap<uint32_t, Gnss_Synchro>::iterator, std::multimap<uint32_t, Gnss_Synchro>::iterator> ret;
    for (it = available_gps_prns.cbegin();
         it != available_gps_prns.cend();
         it++)
        {
            lineObs.clear();
            lineObs += satelliteSystem.find("GPS")->second;
            if (static_cast<int32_t>(*it) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(*it));
            ret = total_gps_map.equal_range(*it);
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // CARRIER PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    //  DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            out << lineObs << '\n';
        }

    for (it = available_gal_prns.cbegin();
         it != available_gal_prns.cend();
         it++)
        {
            lineObs.clear();
            lineObs += satelliteSystem.find("Galileo")->second;
            if (static_cast<int32_t>(*it) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(*it));
            ret = total_gal_map.equal_range(*it);
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // Galileo CARRIER PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // Galileo  DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // Galileo SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            // if (lineObs.size() < 80) lineObs += std::string(80 - lineObs.size(), ' ');
            out << lineObs << '\n';
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Gps_Ephemeris& gps_eph, const Gps_CNAV_Ephemeris& gps_cnav_eph, const Galileo_Ephemeris& galileo_eph, double gps_obs_time, const std::map<int32_t, Gnss_Synchro>& observables, bool triple_band) const
{
    if (galileo_eph.ecc > 0.0)
        {
        }  // avoid warning, not needed
    if (gps_cnav_eph.ecc > 0.0)
        {
        }  // avoid warning, not needed
    std::string line;

    const boost::posix_time::ptime p_gps_time = Rinex_Printer::compute_GPS_time(gps_eph, gps_obs_time);
    const std::string timestring = boost::posix_time::to_iso_string(p_gps_time);
    // double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    // double gps_t = eph.sv_clock_correction(obs_time);
    const double gps_t = gps_obs_time;

    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);

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
    line += Rinex_Printer::asString(seconds, 7);
    line += std::string(2, ' ');
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
    std::map<int32_t, Gnss_Synchro>::const_iterator observables_iter;

    for (observables_iter = observables.cbegin();
         observables_iter != observables.cend();
         observables_iter++)
        {
            const std::string system_(&observables_iter->second.System, 1);
            const std::string sig_(observables_iter->second.Signal);
            if ((system_ == "E") && (sig_ == "1B"))
                {
                    observablesE1B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "E") && (sig_ == "5X"))
                {
                    observablesE5A.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "E") && (sig_ == "7X"))
                {
                    observablesE5B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "E") && (sig_ == "E6"))
                {
                    observablesE6B.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "G") && (sig_ == "2S"))
                {
                    observablesG2S.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "G") && (sig_ == "L5"))
                {
                    observablesGL5.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "G") && (sig_ == "1C"))
                {
                    observablesG1C.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
        }

    std::multimap<uint32_t, Gnss_Synchro> total_gps_map;
    std::multimap<uint32_t, Gnss_Synchro> total_gal_map;
    std::set<uint32_t> available_gal_prns;
    std::set<uint32_t> available_gps_prns;
    std::set<uint32_t>::iterator it;
    for (observables_iter = observablesE1B.cbegin();
         observables_iter != observablesE1B.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (observables_iter = observablesE5A.cbegin();
         observables_iter != observablesE5A.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (observables_iter = observablesE5B.cbegin();
         observables_iter != observablesE5B.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (observables_iter = observablesE6B.cbegin();
         observables_iter != observablesE6B.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_gal_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_gal_prns.find(prn_);
            if (it == available_gal_prns.end())
                {
                    available_gal_prns.insert(prn_);
                }
        }

    for (observables_iter = observablesG1C.cbegin();
         observables_iter != observablesG1C.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_gps_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_gps_prns.find(prn_);
            if (it == available_gps_prns.end())
                {
                    available_gps_prns.insert(prn_);
                }
        }

    for (observables_iter = observablesG2S.cbegin();
         observables_iter != observablesG2S.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_gps_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_gps_prns.find(prn_);
            if (it == available_gps_prns.end())
                {
                    available_gps_prns.insert(prn_);
                }
        }

    for (observables_iter = observablesGL5.cbegin();
         observables_iter != observablesGL5.cend();
         observables_iter++)
        {
            const uint32_t prn_ = observables_iter->second.PRN;
            total_gps_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
            it = available_gps_prns.find(prn_);
            if (it == available_gps_prns.end())
                {
                    available_gps_prns.insert(prn_);
                }
        }

    const int32_t numGalSatellitesObserved = available_gal_prns.size();
    const int32_t numGpsSatellitesObserved = available_gps_prns.size();
    const int32_t numSatellitesObserved = numGalSatellitesObserved + numGpsSatellitesObserved;
    line += Rinex_Printer::rightJustify(std::to_string(numSatellitesObserved), 3);

    // Receiver clock offset (optional)
    // line += rightJustify(asString(clockOffset, 12), 15);

    line += std::string(80 - line.size(), ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    std::string s;
    std::string lineObs;

    std::pair<std::multimap<uint32_t, Gnss_Synchro>::iterator, std::multimap<uint32_t, Gnss_Synchro>::iterator> ret;
    for (it = available_gps_prns.cbegin();
         it != available_gps_prns.cend();
         it++)
        {
            lineObs.clear();
            lineObs += satelliteSystem.find("GPS")->second;
            if (static_cast<int32_t>(*it) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(*it));
            ret = total_gps_map.equal_range(*it);
            bool have_l2 = false;
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    const std::string sig_(iter->second.Signal);
                    if (sig_ == "2S")
                        {
                            have_l2 = true;
                        }
                    if (triple_band and sig_ == "L5" and have_l2 == false)
                        {
                            lineObs += std::string(62, ' ');
                        }

                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // CARRIER PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    //  DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            out << lineObs << '\n';
        }

    for (it = available_gal_prns.cbegin();
         it != available_gal_prns.cend();
         it++)
        {
            lineObs.clear();
            lineObs += satelliteSystem.find("Galileo")->second;
            if (static_cast<int32_t>(*it) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(*it));
            ret = total_gal_map.equal_range(*it);
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // Galileo CARRIER PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // Galileo  DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    // else
                    //    {
                    //        lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int16_t>(lli), 1);
                    //    }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // Galileo SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
                }

            // if (lineObs.size() < 80) lineObs += std::string(80 - lineObs.size(), ' ');
            out << lineObs << '\n';
        }
}


void Rinex_Printer::log_rinex_obs(std::fstream& out, const Beidou_Dnav_Ephemeris& eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables, const std::string& bds_bands) const
{
    std::string line;

    const boost::posix_time::ptime p_bds_time = Rinex_Printer::compute_BDS_time(eph, obs_time);
    const std::string timestring = boost::posix_time::to_iso_string(p_bds_time);
    // double utc_t = nav_msg.utc_time(nav_msg.sv_clock_correction(obs_time));
    // double gps_t = eph.sv_clock_correction(obs_time);
    const double bds_t = obs_time;

    const std::string month(timestring, 4, 2);
    const std::string day(timestring, 6, 2);
    const std::string hour(timestring, 9, 2);
    const std::string minutes(timestring, 11, 2);

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
    const double seconds = fmod(bds_t, 60);
    // Add extra 0 if seconds are < 10
    if (seconds < 10)
        {
            line += std::string(1, '0');
        }
    line += Rinex_Printer::asString(seconds, 7);
    line += std::string(2, ' ');
    // Epoch flag 0: OK     1: power failure between previous and current epoch   <1: Special event
    line += std::string(1, '0');

    // Number of satellites observed in current epoch

    // Get maps with BeiDou observations
    std::map<int32_t, Gnss_Synchro> observablesB1I;
    std::map<int32_t, Gnss_Synchro> observablesB3I;

    std::map<int32_t, Gnss_Synchro>::const_iterator observables_iter;

    for (observables_iter = observables.cbegin();
         observables_iter != observables.cend();
         observables_iter++)
        {
            const std::string system_(&observables_iter->second.System, 1);
            const std::string sig_(observables_iter->second.Signal);
            if ((system_ == "C") && (sig_ == "B1"))
                {
                    observablesB1I.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if ((system_ == "C") && (sig_ == "B3"))
                {
                    observablesB3I.insert(std::pair<int32_t, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
        }
    const std::size_t found_B1 = bds_bands.find("B1");
    const std::size_t found_B3 = bds_bands.find("B3");

    std::multimap<uint32_t, Gnss_Synchro> total_map;
    std::set<uint32_t> available_prns;
    std::set<uint32_t>::iterator it;
    if (found_B1 != std::string::npos)
        {
            for (observables_iter = observablesB1I.cbegin();
                 observables_iter != observablesB1I.cend();
                 observables_iter++)
                {
                    const uint32_t prn_ = observables_iter->second.PRN;
                    total_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
                    it = available_prns.find(prn_);
                    if (it == available_prns.end())
                        {
                            available_prns.insert(prn_);
                        }
                }
        }
    if (found_B3 != std::string::npos)
        {
            for (observables_iter = observablesB3I.cbegin();
                 observables_iter != observablesB3I.cend();
                 observables_iter++)
                {
                    const uint32_t prn_ = observables_iter->second.PRN;
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
                    total_map.insert(std::pair<uint32_t, Gnss_Synchro>(prn_, observables_iter->second));
                }
        }

    const int32_t numSatellitesObserved = available_prns.size();
    line += Rinex_Printer::rightJustify(std::to_string(numSatellitesObserved), 3);
    // Receiver clock offset (optional)
    // line += rightJustify(asString(clockOffset, 12), 15);
    line += std::string(80 - line.size(), ' ');
    Rinex_Printer::lengthCheck(line);
    out << line << '\n';

    std::string lineObs;
    std::pair<std::multimap<uint32_t, Gnss_Synchro>::iterator, std::multimap<uint32_t, Gnss_Synchro>::iterator> ret;
    for (it = available_prns.cbegin();
         it != available_prns.cend();
         it++)
        {
            lineObs.clear();
            lineObs += satelliteSystem.find("Beidou")->second;
            if (static_cast<int32_t>(*it) < 10)
                {
                    lineObs += std::string(1, '0');
                }
            lineObs += std::to_string(static_cast<int32_t>(*it));
            ret = total_map.equal_range(*it);
            for (auto iter = ret.first; iter != ret.second; ++iter)
                {
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Pseudorange_m, 3), 14);

                    // Loss of lock indicator (LLI)
                    int32_t lli = 0;  // Include in the observation!!
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }

                    // Signal Strength Indicator (SSI)
                    const int32_t ssi = Rinex_Printer::signalStrength(iter->second.CN0_dB_hz);
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    // CARRIER PHASE
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_phase_rads / (TWO_PI), 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    //  DOPPLER
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.Carrier_Doppler_hz, 3), 14);
                    if (lli == 0)
                        {
                            lineObs += std::string(1, ' ');
                        }
                    lineObs += Rinex_Printer::rightJustify(Rinex_Printer::asString<int32_t>(ssi), 1);

                    //  SIGNAL STRENGTH
                    lineObs += Rinex_Printer::rightJustify(asString(iter->second.CN0_dB_hz, 3), 14);
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
