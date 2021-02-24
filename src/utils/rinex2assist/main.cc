/*!
 * \file main.cc
 * \brief converts navigation RINEX files into XML files for Assisted GNSS.
 * \author Carles Fernandez-Prades, 2018. cfernandez(at)cttc.cat
 *
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


#include "galileo_ephemeris.h"  // IWYU pragma: keep
#include "galileo_iono.h"
#include "galileo_utc_model.h"
#include "gps_ephemeris.h"
#include "gps_iono.h"
#include "gps_utc_model.h"
#include <boost/archive/xml_oarchive.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/serialization/map.hpp>
#include <gflags/gflags.h>
#include <gpstk/Rinex3NavData.hpp>
#include <gpstk/Rinex3NavHeader.hpp>
#include <gpstk/Rinex3NavStream.hpp>
#include <cstddef>  // for size_t
#include <cstdlib>
#include <iostream>

#if GFLAGS_OLD_NAMESPACE
namespace gflags
{
using namespace google;
}
#endif

int main(int argc, char** argv)
{
    const std::string intro_help(
        std::string("\n rinex2assist converts navigation RINEX files into XML files for Assisted GNSS\n") +
        "Copyright (C) 2018 (see AUTHORS file for a list of contributors)\n" +
        "This program comes with ABSOLUTELY NO WARRANTY;\n" +
        "See COPYING file to see a copy of the General Public License.\n \n" +
        "Usage: \n" +
        "   rinex2assist <RINEX Nav file input>");

    gflags::SetUsageMessage(intro_help);
    google::SetVersionString("1.0");
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    if ((argc != 2))
        {
            std::cerr << "Usage:\n";
            std::cerr << "   " << argv[0]
                      << " <RINEX Nav file input>"
                      << '\n';
            gflags::ShutDownCommandLineFlags();
            return 1;
        }
    std::string xml_filename;

    // Uncompress if RINEX file is gzipped
    std::string rinex_filename(argv[1]);
    std::string input_filename = rinex_filename;
    std::size_t found = rinex_filename.find_last_of('.');
    if (found != std::string::npos)
        {
            if (rinex_filename.size() >= found + 3)
                {
                    if ((rinex_filename.substr(found + 1, found + 3) == "gz"))
                        {
                            std::ifstream file(rinex_filename, std::ios_base::in | std::ios_base::binary);
                            if (file.fail())
                                {
                                    std::cerr << "Could not open file " << rinex_filename << '\n';
                                    return 1;
                                }
                            boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
                            try
                                {
                                    in.push(boost::iostreams::gzip_decompressor());
                                }
                            catch (const boost::exception& e)
                                {
                                    std::cerr << "Could not decompress file " << rinex_filename << '\n';
                                    return 1;
                                }
                            in.push(file);
                            std::string rinex_filename_unzipped = rinex_filename.substr(0, found);
                            std::ofstream output_file(rinex_filename_unzipped.c_str(), std::ios_base::out | std::ios_base::binary | std::ios_base::trunc);
                            if (file.fail())
                                {
                                    std::cerr << "Could not create file " << rinex_filename_unzipped << '\n';
                                    return 1;
                                }
                            boost::iostreams::copy(in, output_file);
                            input_filename = rinex_filename_unzipped;
                        }
                }
            if (rinex_filename.size() >= found + 2)
                {
                    if ((rinex_filename.substr(found + 1, found + 2) == "Z"))
                        {
                            std::ifstream file(rinex_filename, std::ios_base::in | std::ios_base::binary);
                            if (file.fail())
                                {
                                    std::cerr << "Could not open file" << rinex_filename << '\n';
                                    return 1;
                                }
                            file.close();
                            std::string uncompress_executable(UNCOMPRESS_EXECUTABLE);
                            if (!uncompress_executable.empty())
                                {
                                    // option k is not always available, so we save a copy of the original file
                                    std::string argum = std::string("/bin/cp " + rinex_filename + " " + rinex_filename + ".aux");
                                    int s1 = std::system(argum.c_str());
                                    std::string argum2 = std::string(uncompress_executable + " -f " + rinex_filename);
                                    int s2 = std::system(argum2.c_str());
                                    std::string argum3 = std::string("/bin/mv " + rinex_filename + +".aux" + " " + rinex_filename);
                                    int s3 = std::system(argum3.c_str());
                                    input_filename = rinex_filename.substr(0, found);
                                    if ((s1 != 0) or (s2 != 0) or (s3 != 0))
                                        {
                                            std::cerr << "Failure uncompressing file.\n";
                                            return 1;
                                        }
                                }
                            else
                                {
                                    std::cerr << "uncompress program not found.\n";
                                    return 1;
                                }
                        }
                }
        }

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
            gpstk::Rinex3NavStream rnffs(input_filename.c_str());  // Open navigation data file
            gpstk::Rinex3NavData rne;
            gpstk::Rinex3NavHeader hdr;

            // Read header
            rnffs >> hdr;

            // Check that it really is a RINEX navigation file
            if (hdr.fileType.substr(0, 1) != "N")
                {
                    std::cerr << "This is not a valid RINEX navigation file, or file not found.\n";
                    std::cerr << "No XML file will be created.\n";
                    return 1;
                }

            // Collect UTC parameters from RINEX header
            if (hdr.fileSys == "G: (GPS)" || hdr.fileSys == "MIXED")
                {
                    gps_utc_model.valid = (hdr.valid > 2147483648) ? true : false;
                    gps_utc_model.A1 = hdr.mapTimeCorr["GPUT"].A0;
                    gps_utc_model.A0 = hdr.mapTimeCorr["GPUT"].A1;
                    gps_utc_model.tot = hdr.mapTimeCorr["GPUT"].refSOW;
                    gps_utc_model.WN_T = hdr.mapTimeCorr["GPUT"].refWeek;
                    gps_utc_model.DeltaT_LS = hdr.leapSeconds;
                    gps_utc_model.WN_LSF = hdr.leapWeek;
                    gps_utc_model.DN = hdr.leapDay;
                    gps_utc_model.DeltaT_LSF = hdr.leapDelta;

                    // Collect iono parameters from RINEX header
                    gps_iono.valid = (hdr.mapIonoCorr["GPSA"].param[0] == 0) ? false : true;
                    gps_iono.alpha0 = hdr.mapIonoCorr["GPSA"].param[0];
                    gps_iono.alpha1 = hdr.mapIonoCorr["GPSA"].param[1];
                    gps_iono.alpha2 = hdr.mapIonoCorr["GPSA"].param[2];
                    gps_iono.alpha3 = hdr.mapIonoCorr["GPSA"].param[3];
                    gps_iono.beta0 = hdr.mapIonoCorr["GPSB"].param[0];
                    gps_iono.beta1 = hdr.mapIonoCorr["GPSB"].param[1];
                    gps_iono.beta2 = hdr.mapIonoCorr["GPSB"].param[2];
                    gps_iono.beta3 = hdr.mapIonoCorr["GPSB"].param[3];
                }
            if (hdr.fileSys == "E: (GAL)" || hdr.fileSys == "MIXED")
                {
                    gal_utc_model.A0 = hdr.mapTimeCorr["GAUT"].A0;
                    gal_utc_model.A1 = hdr.mapTimeCorr["GAUT"].A1;
                    gal_utc_model.Delta_tLS = hdr.leapSeconds;
                    gal_utc_model.tot = hdr.mapTimeCorr["GAUT"].refSOW;
                    gal_utc_model.WNot = hdr.mapTimeCorr["GAUT"].refWeek;
                    gal_utc_model.WN_LSF = hdr.leapWeek;
                    gal_utc_model.DN = hdr.leapDay;
                    gal_utc_model.Delta_tLSF = hdr.leapDelta;
                    gal_utc_model.flag_utc_model = (hdr.mapTimeCorr["GAUT"].A0 == 0.0);
                    gal_iono.ai0 = hdr.mapIonoCorr["GAL"].param[0];
                    gal_iono.ai1 = hdr.mapIonoCorr["GAL"].param[1];
                    gal_iono.ai2 = hdr.mapIonoCorr["GAL"].param[2];
                    gal_iono.Region1_flag = false;
                    gal_iono.Region2_flag = false;
                    gal_iono.Region3_flag = false;
                    gal_iono.Region4_flag = false;
                    gal_iono.Region5_flag = false;
                    gal_iono.tow = 0.0;
                    gal_iono.WN = 0.0;
                }

            // Read navigation data
            while (rnffs >> rne)
                {
                    if (rne.satSys == "G" or rne.satSys.empty())
                        {
                            // Fill GPS ephemeris object
                            Gps_Ephemeris eph;
                            eph.PRN = rne.PRNID;
                            eph.tow = rne.xmitTime;
                            eph.IODE_SF2 = rne.IODE;
                            eph.IODE_SF3 = rne.IODE;
                            eph.Crs = rne.Crs;
                            eph.delta_n = rne.dn;
                            eph.M_0 = rne.M0;
                            eph.Cuc = rne.Cuc;
                            eph.ecc = rne.ecc;
                            eph.Cus = rne.Cus;
                            eph.sqrtA = rne.Ahalf;
                            eph.toe = rne.Toe;
                            eph.toc = rne.Toc;
                            eph.Cic = rne.Cic;
                            eph.OMEGA_0 = rne.OMEGA0;
                            eph.Cis = rne.Cis;
                            eph.i_0 = rne.i0;
                            eph.Crc = rne.Crc;
                            eph.omega = rne.w;
                            eph.OMEGAdot = rne.OMEGAdot;
                            eph.idot = rne.idot;
                            eph.code_on_L2 = rne.codeflgs;  //
                            eph.WN = rne.weeknum;
                            eph.L2_P_data_flag = rne.L2Pdata;
                            eph.SV_accuracy = rne.accuracy;
                            eph.SV_health = rne.health;
                            eph.TGD = rne.Tgd;
                            eph.IODC = rne.IODC;
                            eph.AODO = 0;  //
                            eph.fit_interval_flag = (rne.fitint > 4) ? true : false;
                            eph.spare1 = 0.0;
                            eph.spare2 = 0.0;
                            eph.af0 = rne.af0;
                            eph.af1 = rne.af1;
                            eph.af2 = rne.af2;
                            eph.integrity_status_flag = false;  //
                            eph.alert_flag = false;             //
                            eph.antispoofing_flag = false;      //
                            eph_map[i] = eph;
                            i++;
                        }
                    if (rne.satSys == "E")
                        {
                            // Fill Galileo ephemeris object
                            Galileo_Ephemeris eph;
                            eph.PRN = rne.PRNID;
                            eph.M_0 = rne.M0;
                            eph.ecc = rne.ecc;
                            eph.sqrtA = rne.Ahalf;
                            eph.OMEGA_0 = rne.OMEGA0;
                            eph.i_0 = rne.i0;
                            eph.omega = rne.w;
                            eph.OMEGAdot = rne.OMEGAdot;
                            eph.delta_n = rne.dn;
                            eph.idot = rne.idot;
                            eph.Cuc = rne.Cuc;
                            eph.Cus = rne.Cus;
                            eph.Crc = rne.Crc;
                            eph.Crs = rne.Crs;
                            eph.Cic = rne.Cic;
                            eph.Cis = rne.Cis;
                            eph.toe = rne.Toe;
                            eph.toc = rne.Toc;
                            eph.af0 = rne.af0;
                            eph.af1 = rne.af1;
                            eph.af2 = rne.af2;
                            eph.WN = rne.weeknum;
                            eph_gal_map[j] = eph;
                            j++;
                        }
                }
        }
    catch (std::exception& e)
        {
            std::cerr << "Error reading the RINEX file: " << e.what() << '\n';
            std::cerr << "No XML file will be created.\n";
            gflags::ShutDownCommandLineFlags();
            return 1;
        }

    if (i == 0 and j == 0)
        {
            std::cerr << "No navigation data found in the RINEX file. No XML file will be created.\n";
            gflags::ShutDownCommandLineFlags();
            return 1;
        }

    // Write XML ephemeris
    if (i != 0)
        {
            std::ofstream ofs;
            if (xml_filename.empty())
                {
                    xml_filename = "gps_ephemeris.xml";
                }
            try
                {
                    ofs.open(xml_filename.c_str(), std::ofstream::trunc | std::ofstream::out);
                    boost::archive::xml_oarchive xml(ofs);
                    xml << boost::serialization::make_nvp("GNSS-SDR_ephemeris_map", eph_map);
                }
            catch (std::exception& e)
                {
                    std::cerr << "Problem creating the XML file " << xml_filename << ": " << e.what() << '\n';
                    gflags::ShutDownCommandLineFlags();
                    return 1;
                }
            std::cout << "Generated file: " << xml_filename << '\n';
        }
    if (j != 0)
        {
            std::ofstream ofs2;
            xml_filename = "gal_ephemeris.xml";
            try
                {
                    ofs2.open(xml_filename.c_str(), std::ofstream::trunc | std::ofstream::out);
                    boost::archive::xml_oarchive xml(ofs2);
                    xml << boost::serialization::make_nvp("GNSS-SDR_gal_ephemeris_map", eph_gal_map);
                }
            catch (std::exception& e)
                {
                    std::cerr << "Problem creating the XML file " << xml_filename << ": " << e.what() << '\n';
                    gflags::ShutDownCommandLineFlags();
                    return 1;
                }
            std::cout << "Generated file: " << xml_filename << '\n';
        }

    // Write XML UTC
    if (gps_utc_model.valid)
        {
            std::ofstream ofs3;
            xml_filename = "gps_utc_model.xml";
            try
                {
                    ofs3.open(xml_filename.c_str(), std::ofstream::trunc | std::ofstream::out);
                    boost::archive::xml_oarchive xml(ofs3);
                    xml << boost::serialization::make_nvp("GNSS-SDR_utc_model", gps_utc_model);
                }
            catch (std::exception& e)
                {
                    std::cerr << "Problem creating the XML file " << xml_filename << ": " << e.what() << '\n';
                    gflags::ShutDownCommandLineFlags();
                    return 1;
                }
            std::cout << "Generated file: " << xml_filename << '\n';
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
                    xml << boost::serialization::make_nvp("GNSS-SDR_iono_model", gps_iono);
                }
            catch (std::exception& e)
                {
                    std::cerr << "Problem creating the XML file " << xml_filename << ": " << e.what() << '\n';
                    gflags::ShutDownCommandLineFlags();
                    return 1;
                }
            std::cout << "Generated file: " << xml_filename << '\n';
        }

    if (gal_utc_model.A0 != 0)
        {
            std::ofstream ofs5;
            xml_filename = "gal_utc_model.xml";
            try
                {
                    ofs5.open(xml_filename.c_str(), std::ofstream::trunc | std::ofstream::out);
                    boost::archive::xml_oarchive xml(ofs5);
                    xml << boost::serialization::make_nvp("GNSS-SDR_gal_utc_model", gal_utc_model);
                }
            catch (std::exception& e)
                {
                    std::cerr << "Problem creating the XML file " << xml_filename << ": " << e.what() << '\n';
                    gflags::ShutDownCommandLineFlags();
                    return 1;
                }
            std::cout << "Generated file: " << xml_filename << '\n';
        }
    if (gal_iono.ai0 != 0)
        {
            std::ofstream ofs7;
            xml_filename = "gal_iono.xml";
            try
                {
                    ofs7.open(xml_filename.c_str(), std::ofstream::trunc | std::ofstream::out);
                    boost::archive::xml_oarchive xml(ofs7);
                    xml << boost::serialization::make_nvp("GNSS-SDR_gal_iono_model", gal_iono);
                }
            catch (std::exception& e)
                {
                    std::cerr << "Problem creating the XML file " << xml_filename << ": " << e.what() << '\n';
                    gflags::ShutDownCommandLineFlags();
                    return 1;
                }
            std::cout << "Generated file: " << xml_filename << '\n';
        }
    gflags::ShutDownCommandLineFlags();
    return 0;
}
