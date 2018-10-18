/*!
 * \file main.cc
 * \brief converts navigation RINEX files into XML files for Assisted GNSS.
 * \author Carles Fernandez-Prades, 2018. cfernandez(at)cttc.cat
 *
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


#include "gps_ephemeris.h"
#include "galileo_ephemeris.h"
#include "gps_utc_model.h"
#include "gps_iono.h"
#include "galileo_utc_model.h"
#include "galileo_iono.h"
#include <gflags/gflags.h>
#include <gpstk/Rinex3NavHeader.hpp>
#include <gpstk/Rinex3NavData.hpp>
#include <gpstk/Rinex3NavStream.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <iostream>


int main(int argc, char** argv)
{
    const std::string intro_help(
        std::string("\n rinex2assist converts navigation RINEX files into XML files for Assisted GNSS\n") +
        "Copyright (C) 2018 (see AUTHORS file for a list of contributors)\n" +
        "This program comes with ABSOLUTELY NO WARRANTY;\n" +
        "See COPYING file to see a copy of the General Public License.\n \n" +
        "Usage: \n" +
        "   rinex2assist <RINEX Nav file input>");

    google::SetUsageMessage(intro_help);
    google::SetVersionString("1.0");
    google::ParseCommandLineFlags(&argc, &argv, true);

    if ((argc != 2))
        {
            std::cerr << "Usage:" << std::endl;
            std::cerr << "   " << argv[0]
                      << " <RINEX Nav file input>"
                      << std::endl;
            google::ShutDownCommandLineFlags();
            return 1;
        }
    std::string xml_filename;

    std::map<int, Gps_Ephemeris> eph_map;
    std::map<int, Galileo_Ephemeris> eph_gal_map;

    Gps_Utc_Model gps_utc_model;
    Gps_Iono gps_iono;
    Galileo_Utc_Model gal_utc_model;
    Galileo_Iono gal_iono;

    int i = 0;
    int j = 0;
    try
        {
            // Read nav file
            gpstk::Rinex3NavStream rnffs(argv[1]);  // Open navigation data file
            gpstk::Rinex3NavData rne;
            gpstk::Rinex3NavHeader hdr;

            // Read header
            rnffs >> hdr;

            // Check that it really is a RINEX navigation file
            if (hdr.fileType.substr(0, 1).compare("N") != 0)
                {
                    std::cerr << "This is not a valid RINEX navigation file, or file not found." << std::endl;
                    std::cerr << "No XML file will be created." << std::endl;
                    return 1;
                }

            // Collect UTC parameters from RINEX header
            if (hdr.fileSys.compare("G: (GPS)") == 0 || hdr.fileSys.compare("MIXED") == 0)
                {
                    gps_utc_model.valid = (hdr.valid > 2147483648) ? true : false;
                    gps_utc_model.d_A1 = hdr.mapTimeCorr["GPUT"].A0;
                    gps_utc_model.d_A0 = hdr.mapTimeCorr["GPUT"].A1;
                    gps_utc_model.d_t_OT = hdr.mapTimeCorr["GPUT"].refSOW;
                    gps_utc_model.i_WN_T = hdr.mapTimeCorr["GPUT"].refWeek;
                    gps_utc_model.d_DeltaT_LS = hdr.leapSeconds;
                    gps_utc_model.i_WN_LSF = hdr.leapWeek;
                    gps_utc_model.i_DN = hdr.leapDay;
                    gps_utc_model.d_DeltaT_LSF = hdr.leapDelta;

                    // Collect iono parameters from RINEX header
                    gps_iono.valid = (hdr.mapIonoCorr["GPSA"].param[0] == 0) ? false : true;
                    gps_iono.d_alpha0 = hdr.mapIonoCorr["GPSA"].param[0];
                    gps_iono.d_alpha1 = hdr.mapIonoCorr["GPSA"].param[1];
                    gps_iono.d_alpha2 = hdr.mapIonoCorr["GPSA"].param[2];
                    gps_iono.d_alpha3 = hdr.mapIonoCorr["GPSA"].param[3];
                    gps_iono.d_beta0 = hdr.mapIonoCorr["GPSB"].param[0];
                    gps_iono.d_beta1 = hdr.mapIonoCorr["GPSB"].param[1];
                    gps_iono.d_beta2 = hdr.mapIonoCorr["GPSB"].param[2];
                    gps_iono.d_beta3 = hdr.mapIonoCorr["GPSB"].param[3];
                }
            if (hdr.fileSys.compare("E: (GAL)") == 0 || hdr.fileSys.compare("MIXED") == 0)
                {
                    gal_utc_model.A0_6 = hdr.mapTimeCorr["GAUT"].A0;
                    gal_utc_model.A1_6 = hdr.mapTimeCorr["GAUT"].A1;
                    gal_utc_model.Delta_tLS_6 = hdr.leapSeconds;
                    gal_utc_model.t0t_6 = hdr.mapTimeCorr["GAUT"].refSOW;
                    gal_utc_model.WNot_6 = hdr.mapTimeCorr["GAUT"].refWeek;
                    gal_utc_model.WN_LSF_6 = hdr.leapWeek;
                    gal_utc_model.DN_6 = hdr.leapDay;
                    gal_utc_model.Delta_tLSF_6 = hdr.leapDelta;
                    gal_iono.ai0_5 = hdr.mapIonoCorr["GAL"].param[0];
                    gal_iono.ai1_5 = hdr.mapIonoCorr["GAL"].param[1];
                    gal_iono.ai2_5 = hdr.mapIonoCorr["GAL"].param[2];
                    gal_iono.Region1_flag_5 = false;
                    gal_iono.Region2_flag_5 = false;
                    gal_iono.Region3_flag_5 = false;
                    gal_iono.Region4_flag_5 = false;
                    gal_iono.Region5_flag_5 = false;
                    gal_iono.TOW_5 = 0.0;
                    gal_iono.WN_5 = 0.0;
                }

            // Read navigation data
            while (rnffs >> rne)
                {
                    if (rne.satSys.compare("G") == 0 or rne.satSys.empty())
                        {
                            // Fill GPS ephemeris object
                            Gps_Ephemeris eph;
                            eph.i_satellite_PRN = rne.PRNID;
                            eph.d_TOW = rne.xmitTime;
                            eph.d_IODE_SF2 = rne.IODE;
                            eph.d_IODE_SF3 = rne.IODE;
                            eph.d_Crs = rne.Crs;
                            eph.d_Delta_n = rne.dn;
                            eph.d_M_0 = rne.M0;
                            eph.d_Cuc = rne.Cuc;
                            eph.d_e_eccentricity = rne.ecc;
                            eph.d_Cus = rne.Cus;
                            eph.d_sqrt_A = rne.Ahalf;
                            eph.d_Toe = rne.Toe;
                            eph.d_Toc = rne.Toc;
                            eph.d_Cic = rne.Cic;
                            eph.d_OMEGA0 = rne.OMEGA0;
                            eph.d_Cis = rne.Cis;
                            eph.d_i_0 = rne.i0;
                            eph.d_Crc = rne.Crc;
                            eph.d_OMEGA = rne.w;
                            eph.d_OMEGA_DOT = rne.OMEGAdot;
                            eph.d_IDOT = rne.idot;
                            eph.i_code_on_L2 = rne.codeflgs;  //
                            eph.i_GPS_week = rne.weeknum;
                            eph.b_L2_P_data_flag = rne.L2Pdata;
                            eph.i_SV_accuracy = rne.accuracy;
                            eph.i_SV_health = rne.health;
                            eph.d_TGD = rne.Tgd;
                            eph.d_IODC = rne.IODC;
                            eph.i_AODO = 0;  //
                            eph.b_fit_interval_flag = (rne.fitint > 4) ? 1 : 0;
                            eph.d_spare1 = 0.0;
                            eph.d_spare2 = 0.0;
                            eph.d_A_f0 = rne.af0;
                            eph.d_A_f1 = rne.af1;
                            eph.d_A_f2 = rne.af2;
                            eph.b_integrity_status_flag = 0;  //
                            eph.b_alert_flag = 0;             //
                            eph.b_antispoofing_flag = 0;      //
                            eph_map[i] = eph;
                            i++;
                        }
                    if (rne.satSys.compare("E") == 0)
                        {
                            // Fill Galileo ephemeris object
                            Galileo_Ephemeris eph;
                            eph.i_satellite_PRN = rne.PRNID;
                            eph.M0_1 = rne.M0;
                            eph.e_1 = rne.ecc;
                            eph.A_1 = rne.Ahalf;
                            eph.OMEGA_0_2 = rne.OMEGA0;
                            eph.i_0_2 = rne.i0;
                            eph.omega_2 = rne.w;
                            eph.OMEGA_dot_3 = rne.OMEGAdot;
                            eph.iDot_2 = rne.idot;
                            eph.C_uc_3 = rne.Cuc;
                            eph.C_us_3 = rne.Cus;
                            eph.C_rc_3 = rne.Crc;
                            eph.C_rs_3 = rne.Crs;
                            eph.C_ic_4 = rne.Cic;
                            eph.C_is_4 = rne.Cis;
                            eph.t0e_1 = rne.Toe;
                            eph.t0c_4 = rne.Toc;
                            eph.af0_4 = rne.af0;
                            eph.af1_4 = rne.af1;
                            eph.af2_4 = rne.af2;
                            eph_gal_map[j] = eph;
                            j++;
                        }
                }
        }
    catch (std::exception& e)
        {
            std::cerr << "Error reading the RINEX file: " << e.what() << std::endl;
            std::cerr << "No XML file will be created." << std::endl;
            google::ShutDownCommandLineFlags();
            return 1;
        }

    if (i == 0 and j == 0)
        {
            std::cerr << "No navigation data found in the RINEX file. No XML file will be created." << std::endl;
            google::ShutDownCommandLineFlags();
            return 1;
        }

    // Write XML ephemeris
    if (i != 0)
        {
            std::ofstream ofs;
            if (xml_filename.empty())
                {
                    xml_filename = "eph_GPS_L1CA.xml";
                }
            try
                {
                    ofs.open(xml_filename.c_str(), std::ofstream::trunc | std::ofstream::out);
                    boost::archive::xml_oarchive xml(ofs);
                    xml << boost::serialization::make_nvp("GNSS-SDR_ephemeris_map", eph_map);
                }
            catch (std::exception& e)
                {
                    std::cerr << "Problem creating the XML file " << xml_filename << ": " << e.what() << std::endl;
                    google::ShutDownCommandLineFlags();
                    return 1;
                }
            std::cout << "Generated file: " << xml_filename << std::endl;
        }
    if (j != 0)
        {
            std::ofstream ofs2;
            xml_filename = "eph_Galileo_E1.xml";
            try
                {
                    ofs2.open(xml_filename.c_str(), std::ofstream::trunc | std::ofstream::out);
                    boost::archive::xml_oarchive xml(ofs2);
                    xml << boost::serialization::make_nvp("GNSS-SDR_ephemeris_map", eph_gal_map);
                }
            catch (std::exception& e)
                {
                    std::cerr << "Problem creating the XML file " << xml_filename << ": " << e.what() << std::endl;
                    google::ShutDownCommandLineFlags();
                    return 1;
                }
            std::cout << "Generated file: " << xml_filename << std::endl;
        }

    // Write XML UTC
    if (gps_utc_model.valid)
        {
            std::ofstream ofs3;
            xml_filename = "gps_utc.xml";
            try
                {
                    ofs3.open(xml_filename.c_str(), std::ofstream::trunc | std::ofstream::out);
                    boost::archive::xml_oarchive xml(ofs3);
                    xml << boost::serialization::make_nvp("GNSS-SDR_gps_utc", gps_utc_model);
                }
            catch (std::exception& e)
                {
                    std::cerr << "Problem creating the XML file " << xml_filename << ": " << e.what() << std::endl;
                    google::ShutDownCommandLineFlags();
                    return 1;
                }
            std::cout << "Generated file: " << xml_filename << std::endl;
        }

    // Write XML iono
    if (gps_iono.valid)
        {
            std::ofstream ofs4;
            xml_filename = "gps_iono.xml";
            try
                {
                    ofs4.open(xml_filename.c_str(), std::ofstream::trunc | std::ofstream::out);
                    boost::archive::xml_oarchive xml(ofs4);
                    xml << boost::serialization::make_nvp("GNSS-SDR_gps_iono", gps_iono);
                }
            catch (std::exception& e)
                {
                    std::cerr << "Problem creating the XML file " << xml_filename << ": " << e.what() << std::endl;
                    google::ShutDownCommandLineFlags();
                    return 1;
                }
            std::cout << "Generated file: " << xml_filename << std::endl;
        }

    if (gal_utc_model.A0_6 != 0)
        {
            std::ofstream ofs5;
            xml_filename = "gal_utc.xml";
            try
                {
                    ofs5.open(xml_filename.c_str(), std::ofstream::trunc | std::ofstream::out);
                    boost::archive::xml_oarchive xml(ofs5);
                    xml << boost::serialization::make_nvp("GNSS-SDR_gal_utc", gal_utc_model);
                }
            catch (std::exception& e)
                {
                    std::cerr << "Problem creating the XML file " << xml_filename << ": " << e.what() << std::endl;
                    google::ShutDownCommandLineFlags();
                    return 1;
                }
            std::cout << "Generated file: " << xml_filename << std::endl;
        }
    if (gal_iono.ai0_5 != 0)
        {
            std::ofstream ofs7;
            xml_filename = "gal_iono.xml";
            try
                {
                    ofs7.open(xml_filename.c_str(), std::ofstream::trunc | std::ofstream::out);
                    boost::archive::xml_oarchive xml(ofs7);
                    xml << boost::serialization::make_nvp("GNSS-SDR_gal_iono", gal_iono);
                }
            catch (std::exception& e)
                {
                    std::cerr << "Problem creating the XML file " << xml_filename << ": " << e.what() << std::endl;
                    google::ShutDownCommandLineFlags();
                    return 1;
                }
            std::cout << "Generated file: " << xml_filename << std::endl;
        }
    google::ShutDownCommandLineFlags();
    return 0;
}
