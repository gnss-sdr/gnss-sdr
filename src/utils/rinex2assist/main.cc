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
        "   rinex2assist <RINEX Nav file input>  [<XML file output>]");

    google::SetUsageMessage(intro_help);
    google::SetVersionString("1.0");
    google::ParseCommandLineFlags(&argc, &argv, true);

    if ((argc < 2) or (argc > 3))
        {
            std::cerr << "Usage:" << std::endl;
            std::cerr << "   " << argv[0]
                      << " <RINEX Nav file input>  [<XML file output>]"
                      << std::endl;
            google::ShutDownCommandLineFlags();
            return 1;
        }
    std::string xml_filename;
    if (argc == 3)
        {
            xml_filename = argv[2];
        }

    std::map<int, Gps_Ephemeris> eph_map;
    std::map<int, Galileo_Ephemeris> eph_gal_map;

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

    // Write XML
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
                    std::cerr << "Problem creating the XML file: " << e.what() << std::endl;
                    google::ShutDownCommandLineFlags();
                    return 1;
                }
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
                    std::cerr << "Problem creating the XML file: " << e.what() << std::endl;
                    google::ShutDownCommandLineFlags();
                    return 1;
                }
        }
    google::ShutDownCommandLineFlags();
    return 0;
}
