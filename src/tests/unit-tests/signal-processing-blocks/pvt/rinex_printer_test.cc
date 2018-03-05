/*!
 * \file rinex_printer_test.cc
 * \brief Implements Unit Tests for the Rinex_Printer class.
 * \author Carles Fernandez-Prades, 2016. cfernandez(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2016  (see AUTHORS file for a list of contributors)
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

#include <string>
#include "rinex_printer.h"


TEST(RinexPrinterTest, GalileoObsHeader)
{
    std::string line_aux;
    std::string line_str;
    bool no_more_finds = false;
    const Galileo_Ephemeris eph = Galileo_Ephemeris();

    std::shared_ptr<Rinex_Printer> rp1;
    rp1 = std::make_shared<Rinex_Printer>();
    rp1->rinex_obs_header(rp1->obsFile, eph, 0.0);
    rp1->obsFile.seekp(0);

    while (!rp1->obsFile.eof())
        {
            std::getline(rp1->obsFile, line_str);
            if (!no_more_finds)
                {
                    if (line_str.find("SYS / # / OBS TYPES", 59) != std::string::npos)
                        {
                            no_more_finds = true;
                            line_aux = std::string(line_str);
                        }
                }
        }
    std::string expected_str("E    4 C1B L1B D1B S1B                                      SYS / # / OBS TYPES ");
    EXPECT_EQ(0, expected_str.compare(line_aux));
    if (remove(rp1->obsfilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
    line_aux.clear();

    std::shared_ptr<Rinex_Printer> rp2;
    rp2 = std::make_shared<Rinex_Printer>();
    std::string bands("1B 5X 7X");
    rp2->rinex_obs_header(rp2->obsFile, eph, 0.0, bands);
    rp2->obsFile.seekp(0);
    no_more_finds = false;
    while (!rp2->obsFile.eof())
        {
            std::getline(rp2->obsFile, line_str);
            if (!no_more_finds)
                {
                    if (line_str.find("SYS / # / OBS TYPES", 59) != std::string::npos)
                        {
                            no_more_finds = true;
                            line_aux = std::string(line_str);
                        }
                }
        }
    std::string expected_str2("E   12 C1B L1B D1B S1B C5X L5X D5X S5X C7X L7X D7X S7X      SYS / # / OBS TYPES ");
    EXPECT_EQ(0, expected_str2.compare(line_aux));

    if (remove(rp2->obsfilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
}


TEST(RinexPrinterTest, GlonassObsHeader)
{
    std::string line_aux;
    std::string line_str;
    bool no_more_finds = false;
    const Glonass_Gnav_Ephemeris eph = Glonass_Gnav_Ephemeris();

    std::shared_ptr<Rinex_Printer> rp1;
    rp1 = std::make_shared<Rinex_Printer>(3);
    const std::string bands = "1G";
    rp1->rinex_obs_header(rp1->obsFile, eph, 0.0, bands);
    rp1->obsFile.seekp(0);

    while (!rp1->obsFile.eof())
        {
            std::getline(rp1->obsFile, line_str);
            if (!no_more_finds)
                {
                    if (line_str.find("SYS / # / OBS TYPES", 59) != std::string::npos)
                        {
                            no_more_finds = true;
                            line_aux = std::string(line_str);
                        }
                }
        }
    std::string expected_str("R    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES ");
    EXPECT_EQ(0, expected_str.compare(line_aux));
    if (remove(rp1->obsfilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
    line_aux.clear();
}


TEST(RinexPrinterTest, MixedObsHeader)
{
    std::string line_aux;
    std::string line_aux2;
    std::string line_str;
    bool no_more_finds = false;
    const Galileo_Ephemeris eph_gal = Galileo_Ephemeris();
    const Gps_Ephemeris eph_gps = Gps_Ephemeris();

    std::shared_ptr<Rinex_Printer> rp1;
    rp1 = std::make_shared<Rinex_Printer>();
    rp1->rinex_obs_header(rp1->obsFile, eph_gps, eph_gal, 0.0, "1B 5X");
    rp1->obsFile.seekp(0);
    int systems_found = 0;

    while (!rp1->obsFile.eof())
        {
            std::getline(rp1->obsFile, line_str);
            if (!no_more_finds)
                {
                    if (line_str.find("SYS / # / OBS TYPES", 59) != std::string::npos)
                        {
                            systems_found++;
                            if (systems_found == 1)
                                {
                                    line_aux = std::string(line_str);
                                }
                            if (systems_found == 2)
                                {
                                    line_aux2 = std::string(line_str);
                                    no_more_finds = true;
                                }
                        }
                }
        }

    std::string expected_str("G    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES ");
    std::string expected_str2("E    8 C1B L1B D1B S1B C5X L5X D5X S5X                      SYS / # / OBS TYPES ");
    EXPECT_EQ(0, expected_str.compare(line_aux));
    EXPECT_EQ(0, expected_str2.compare(line_aux2));
    if (remove(rp1->obsfilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
}


TEST(RinexPrinterTest, MixedObsHeaderGpsGlo)
{
    std::string line_aux;
    std::string line_aux2;
    std::string line_str;
    bool no_more_finds = false;
    const Glonass_Gnav_Ephemeris eph_glo = Glonass_Gnav_Ephemeris();
    const Gps_Ephemeris eph_gps = Gps_Ephemeris();

    std::shared_ptr<Rinex_Printer> rp1;
    rp1 = std::make_shared<Rinex_Printer>();
    rp1->rinex_obs_header(rp1->obsFile, eph_gps, eph_glo, 0.0, "1G");
    rp1->obsFile.seekp(0);
    int systems_found = 0;

    while (!rp1->obsFile.eof())
        {
            std::getline(rp1->obsFile, line_str);
            if (!no_more_finds)
                {
                    if (line_str.find("SYS / # / OBS TYPES", 59) != std::string::npos)
                        {
                            systems_found++;
                            if (systems_found == 1)
                                {
                                    line_aux = std::string(line_str);
                                }
                            if (systems_found == 2)
                                {
                                    line_aux2 = std::string(line_str);
                                    no_more_finds = true;
                                }
                        }
                }
        }

    std::string expected_str("G    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES ");
    std::string expected_str2("R    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES ");
    EXPECT_EQ(0, expected_str.compare(line_aux));
    EXPECT_EQ(0, expected_str2.compare(line_aux2));
    if (remove(rp1->obsfilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
}


TEST(RinexPrinterTest, GalileoObsLog)
{
    std::string line_aux;
    std::string line_str;
    bool no_more_finds = false;
    const Galileo_Ephemeris eph = Galileo_Ephemeris();

    std::shared_ptr<Rinex_Printer> rp;
    rp = std::make_shared<Rinex_Printer>();
    rp->rinex_obs_header(rp->obsFile, eph, 0.0);

    std::map<int, Gnss_Synchro> gnss_pseudoranges_map;

    Gnss_Synchro gs1 = Gnss_Synchro();
    Gnss_Synchro gs2 = Gnss_Synchro();
    Gnss_Synchro gs3 = Gnss_Synchro();
    Gnss_Synchro gs4 = Gnss_Synchro();

    std::string sys = "E";
    gs1.System = *sys.c_str();
    gs2.System = *sys.c_str();
    gs3.System = *sys.c_str();
    gs4.System = *sys.c_str();

    std::string sig = "1B";
    std::memcpy(static_cast<void*>(gs1.Signal), sig.c_str(), 3);
    std::memcpy(static_cast<void*>(gs2.Signal), sig.c_str(), 3);
    std::memcpy(static_cast<void*>(gs3.Signal), sig.c_str(), 3);
    std::memcpy(static_cast<void*>(gs4.Signal), sig.c_str(), 3);

    gs1.PRN = 3;
    gs2.PRN = 8;
    gs3.PRN = 10;
    gs4.PRN = 22;

    gs4.Pseudorange_m = 22000000;
    gs4.Carrier_phase_rads = 23.4;
    gs4.Carrier_Doppler_hz = 1534;
    gs4.CN0_dB_hz = 42;

    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(1, gs1));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(2, gs2));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(3, gs3));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(4, gs4));

    rp->log_rinex_obs(rp->obsFile, eph, 0.0, gnss_pseudoranges_map);
    rp->obsFile.seekp(0);

    while (!rp->obsFile.eof())
        {
            std::getline(rp->obsFile, line_str);
            if (!no_more_finds)
                {
                    if (line_str.find("E22", 0) != std::string::npos)
                        {
                            no_more_finds = true;
                            line_aux = std::string(line_str);
                        }
                }
        }

    std::string expected_str("E22  22000000.000 7         3.724 7      1534.000 7        42.000               ");
    EXPECT_EQ(0, expected_str.compare(line_aux));

    if (remove(rp->obsfilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
}


TEST(RinexPrinterTest, GlonassObsLog)
{
    std::string line_aux;
    std::string line_str;
    bool no_more_finds = false;
    const Glonass_Gnav_Ephemeris eph = Glonass_Gnav_Ephemeris();

    std::shared_ptr<Rinex_Printer> rp;
    rp = std::make_shared<Rinex_Printer>();
    rp->rinex_obs_header(rp->obsFile, eph, 0.0);

    std::map<int, Gnss_Synchro> gnss_pseudoranges_map;

    Gnss_Synchro gs1 = Gnss_Synchro();
    Gnss_Synchro gs2 = Gnss_Synchro();
    Gnss_Synchro gs3 = Gnss_Synchro();
    Gnss_Synchro gs4 = Gnss_Synchro();

    std::string sys = "R";
    gs1.System = *sys.c_str();
    gs2.System = *sys.c_str();
    gs3.System = *sys.c_str();
    gs4.System = *sys.c_str();

    std::string sig = "1C";
    std::memcpy((void*)gs1.Signal, sig.c_str(), 3);
    std::memcpy((void*)gs2.Signal, sig.c_str(), 3);
    std::memcpy((void*)gs3.Signal, sig.c_str(), 3);
    std::memcpy((void*)gs4.Signal, sig.c_str(), 3);

    gs1.PRN = 3;
    gs2.PRN = 8;
    gs3.PRN = 10;
    gs4.PRN = 22;

    gs4.Pseudorange_m = 22000000;
    gs4.Carrier_phase_rads = 23.4;
    gs4.Carrier_Doppler_hz = 1534;
    gs4.CN0_dB_hz = 42;

    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(1, gs1));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(2, gs2));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(3, gs3));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(4, gs4));

    rp->log_rinex_obs(rp->obsFile, eph, 0.0, gnss_pseudoranges_map);
    rp->obsFile.seekp(0);

    while (!rp->obsFile.eof())
        {
            std::getline(rp->obsFile, line_str);
            if (!no_more_finds)
                {
                    if (line_str.find("R22", 0) != std::string::npos)
                        {
                            no_more_finds = true;
                            line_aux = std::string(line_str);
                        }
                }
        }

    std::string expected_str("R22  22000000.000 7         3.724 7      1534.000 7        42.000               ");
    EXPECT_EQ(0, expected_str.compare(line_aux));

    if (remove(rp->obsfilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
}


TEST(RinexPrinterTest, GpsObsLogDualBand)
{
    std::string line_aux;
    std::string line_str;
    bool no_more_finds = false;
    const Gps_Ephemeris eph_gps = Gps_Ephemeris();
    const Gps_CNAV_Ephemeris eph_cnav = Gps_CNAV_Ephemeris();

    std::shared_ptr<Rinex_Printer> rp;
    rp = std::make_shared<Rinex_Printer>();
    rp->rinex_obs_header(rp->obsFile, eph_gps, eph_cnav, 0.0);

    std::map<int, Gnss_Synchro> gnss_pseudoranges_map;

    Gnss_Synchro gs1 = Gnss_Synchro();
    Gnss_Synchro gs2 = Gnss_Synchro();
    Gnss_Synchro gs3 = Gnss_Synchro();
    Gnss_Synchro gs4 = Gnss_Synchro();

    std::string sys = "G";
    gs1.System = *sys.c_str();
    gs2.System = *sys.c_str();
    gs3.System = *sys.c_str();
    gs4.System = *sys.c_str();

    std::string sig = "1C";
    std::memcpy(static_cast<void*>(gs1.Signal), sig.c_str(), 3);
    std::memcpy(static_cast<void*>(gs2.Signal), sig.c_str(), 3);

    sig = "2S";
    std::memcpy(static_cast<void*>(gs3.Signal), sig.c_str(), 3);
    std::memcpy(static_cast<void*>(gs4.Signal), sig.c_str(), 3);

    gs1.PRN = 3;
    gs2.PRN = 8;
    gs3.PRN = 7;
    gs4.PRN = 8;

    gs2.Pseudorange_m = 22000002.1;
    gs2.Carrier_phase_rads = 45.4;
    gs2.Carrier_Doppler_hz = 321;
    gs2.CN0_dB_hz = 39;

    gs4.Pseudorange_m = 22000000;
    gs4.Carrier_phase_rads = 23.4;
    gs4.Carrier_Doppler_hz = 1534;
    gs4.CN0_dB_hz = 42;

    gs3.Pseudorange_m = 22000007;
    gs3.Carrier_phase_rads = -23.4;
    gs3.Carrier_Doppler_hz = -1534;
    gs3.CN0_dB_hz = 47;

    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(1, gs1));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(2, gs2));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(3, gs3));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(4, gs4));

    rp->log_rinex_obs(rp->obsFile, eph_gps, eph_cnav, 0.0, gnss_pseudoranges_map);
    rp->obsFile.seekp(0);

    while (!rp->obsFile.eof())
        {
            std::getline(rp->obsFile, line_str);
            if (!no_more_finds)
                {
                    if (line_str.find("G08", 0) != std::string::npos)
                        {
                            no_more_finds = true;
                            line_aux = std::string(line_str);
                        }
                }
        }

    std::string expected_str("G08  22000002.100 6         7.226 6       321.000 6        39.000  22000000.000 7         3.724 7      1534.000 7        42.000");
    EXPECT_EQ(0, expected_str.compare(line_aux));

    if (remove(rp->obsfilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
}


TEST(RinexPrinterTest, GalileoObsLogDualBand)
{
    std::string line_aux;
    std::string line_str;
    bool no_more_finds = false;
    const Galileo_Ephemeris eph = Galileo_Ephemeris();

    std::shared_ptr<Rinex_Printer> rp;
    rp = std::make_shared<Rinex_Printer>();
    std::string bands("1B 5X");
    rp->rinex_obs_header(rp->obsFile, eph, 0.0, bands);

    std::map<int, Gnss_Synchro> gnss_pseudoranges_map;

    Gnss_Synchro gs1 = Gnss_Synchro();
    Gnss_Synchro gs2 = Gnss_Synchro();
    Gnss_Synchro gs3 = Gnss_Synchro();
    Gnss_Synchro gs4 = Gnss_Synchro();

    std::string sys = "E";
    gs1.System = *sys.c_str();
    gs2.System = *sys.c_str();
    gs3.System = *sys.c_str();
    gs4.System = *sys.c_str();

    std::string sig = "1B";
    std::memcpy(static_cast<void*>(gs1.Signal), sig.c_str(), 3);
    std::memcpy(static_cast<void*>(gs2.Signal), sig.c_str(), 3);

    sig = "5X";
    std::memcpy(static_cast<void*>(gs3.Signal), sig.c_str(), 3);
    std::memcpy(static_cast<void*>(gs4.Signal), sig.c_str(), 3);

    gs1.PRN = 3;
    gs2.PRN = 8;
    gs3.PRN = 3;
    gs4.PRN = 8;

    gs2.Pseudorange_m = 22000002.1;
    gs2.Carrier_phase_rads = 45.4;
    gs2.Carrier_Doppler_hz = 321;
    gs2.CN0_dB_hz = 39;

    gs3.Pseudorange_m = 22000003.3;
    gs3.Carrier_phase_rads = 43.3;
    gs3.Carrier_Doppler_hz = -321;
    gs3.CN0_dB_hz = 40;

    gs4.Pseudorange_m = 22000000;
    gs4.Carrier_phase_rads = 23.4;
    gs4.Carrier_Doppler_hz = 1534;
    gs4.CN0_dB_hz = 42;

    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(1, gs1));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(2, gs2));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(3, gs3));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(4, gs4));

    rp->log_rinex_obs(rp->obsFile, eph, 0.0, gnss_pseudoranges_map, bands);
    rp->obsFile.seekp(0);

    while (!rp->obsFile.eof())
        {
            std::getline(rp->obsFile, line_str);
            if (!no_more_finds)
                {
                    if (line_str.find("E08", 0) != std::string::npos)
                        {
                            no_more_finds = true;
                            line_aux = std::string(line_str);
                        }
                }
        }

    std::string expected_str("E08  22000002.100 6         7.226 6       321.000 6        39.000  22000000.000 7         3.724 7      1534.000 7        42.000");
    EXPECT_EQ(0, expected_str.compare(line_aux));

    if (remove(rp->obsfilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
}


TEST(RinexPrinterTest, MixedObsLog)
{
    std::string line_aux;
    std::string line_str;
    bool no_more_finds = false;
    const Galileo_Ephemeris eph_gal = Galileo_Ephemeris();
    const Gps_Ephemeris eph_gps = Gps_Ephemeris();

    std::shared_ptr<Rinex_Printer> rp;
    rp = std::make_shared<Rinex_Printer>();
    rp->rinex_obs_header(rp->obsFile, eph_gps, eph_gal, 0.0, "1B 5X");

    std::map<int, Gnss_Synchro> gnss_pseudoranges_map;

    Gnss_Synchro gs1 = Gnss_Synchro();
    Gnss_Synchro gs2 = Gnss_Synchro();
    Gnss_Synchro gs3 = Gnss_Synchro();
    Gnss_Synchro gs4 = Gnss_Synchro();
    Gnss_Synchro gs5 = Gnss_Synchro();
    Gnss_Synchro gs6 = Gnss_Synchro();
    Gnss_Synchro gs7 = Gnss_Synchro();
    Gnss_Synchro gs8 = Gnss_Synchro();

    std::string sys = "G";
    gs1.System = *sys.c_str();
    gs2.System = *sys.c_str();
    gs3.System = *sys.c_str();
    gs4.System = *sys.c_str();

    sys = "E";
    gs5.System = *sys.c_str();
    gs6.System = *sys.c_str();
    gs7.System = *sys.c_str();
    gs8.System = *sys.c_str();

    std::string sig = "1C";
    std::memcpy(static_cast<void*>(gs1.Signal), sig.c_str(), 3);
    std::memcpy(static_cast<void*>(gs2.Signal), sig.c_str(), 3);
    std::memcpy(static_cast<void*>(gs3.Signal), sig.c_str(), 3);
    std::memcpy(static_cast<void*>(gs4.Signal), sig.c_str(), 3);

    sig = "5X";
    std::memcpy(static_cast<void*>(gs5.Signal), sig.c_str(), 3);
    std::memcpy(static_cast<void*>(gs6.Signal), sig.c_str(), 3);

    sig = "1B";
    std::memcpy(static_cast<void*>(gs7.Signal), sig.c_str(), 3);
    std::memcpy(static_cast<void*>(gs8.Signal), sig.c_str(), 3);

    gs1.PRN = 3;
    gs2.PRN = 8;
    gs3.PRN = 14;
    gs4.PRN = 16;
    gs5.PRN = 3;
    gs6.PRN = 16;
    gs7.PRN = 14;
    gs8.PRN = 16;

    gs2.Pseudorange_m = 22000002.1;
    gs2.Carrier_phase_rads = 45.4;
    gs2.Carrier_Doppler_hz = 321;
    gs2.CN0_dB_hz = 39;

    gs4.Pseudorange_m = 22000000;
    gs4.Carrier_phase_rads = 23.4;
    gs4.Carrier_Doppler_hz = -1534;
    gs4.CN0_dB_hz = 40;

    gs6.Pseudorange_m = 22000000;
    gs6.Carrier_phase_rads = 52.1;
    gs6.Carrier_Doppler_hz = 1534;
    gs6.CN0_dB_hz = 41;

    gs8.Pseudorange_m = 22000000;
    gs8.Carrier_phase_rads = 0.8;
    gs8.Carrier_Doppler_hz = -20;
    gs8.CN0_dB_hz = 42;

    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(1, gs1));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(2, gs2));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(3, gs3));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(4, gs4));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(5, gs5));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(6, gs6));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(7, gs7));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(8, gs8));

    rp->log_rinex_obs(rp->obsFile, eph_gps, eph_gal, 0.0, gnss_pseudoranges_map);

    rp->obsFile.seekp(0);

    while (!rp->obsFile.eof())
        {
            std::getline(rp->obsFile, line_str);
            if (!no_more_finds)
                {
                    if (line_str.find("E16", 0) != std::string::npos)
                        {
                            no_more_finds = true;
                            line_aux = std::string(line_str);
                        }
                }
        }
    std::string expected_str("E16  22000000.000 7         0.127 7       -20.000 7        42.000  22000000.000 6         8.292 6      1534.000 6        41.000");
    EXPECT_EQ(0, expected_str.compare(line_aux));

    if (remove(rp->obsfilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
}


TEST(RinexPrinterTest, MixedObsLogGpsGlo)
{
    std::string line_aux;
    std::string line_str;
    bool no_more_finds = false;
    const Glonass_Gnav_Ephemeris eph_glo = Glonass_Gnav_Ephemeris();
    const Gps_Ephemeris eph_gps = Gps_Ephemeris();

    std::shared_ptr<Rinex_Printer> rp;
    rp = std::make_shared<Rinex_Printer>();
    rp->rinex_obs_header(rp->obsFile, eph_gps, eph_glo, 0.0, "1G");

    std::map<int, Gnss_Synchro> gnss_pseudoranges_map;

    Gnss_Synchro gs1 = Gnss_Synchro();
    Gnss_Synchro gs2 = Gnss_Synchro();
    Gnss_Synchro gs3 = Gnss_Synchro();
    Gnss_Synchro gs4 = Gnss_Synchro();
    Gnss_Synchro gs5 = Gnss_Synchro();
    Gnss_Synchro gs6 = Gnss_Synchro();
    Gnss_Synchro gs7 = Gnss_Synchro();
    Gnss_Synchro gs8 = Gnss_Synchro();

    std::string sys = "G";
    gs1.System = *sys.c_str();
    gs2.System = *sys.c_str();
    gs3.System = *sys.c_str();
    gs4.System = *sys.c_str();

    sys = "R";
    gs5.System = *sys.c_str();
    gs6.System = *sys.c_str();
    gs7.System = *sys.c_str();
    gs8.System = *sys.c_str();

    std::string sig = "1C";
    std::memcpy((void*)gs1.Signal, sig.c_str(), 3);
    std::memcpy((void*)gs2.Signal, sig.c_str(), 3);
    std::memcpy((void*)gs3.Signal, sig.c_str(), 3);
    std::memcpy((void*)gs4.Signal, sig.c_str(), 3);

    sig = "1G";
    std::memcpy((void*)gs5.Signal, sig.c_str(), 3);
    std::memcpy((void*)gs6.Signal, sig.c_str(), 3);
    std::memcpy((void*)gs7.Signal, sig.c_str(), 3);
    std::memcpy((void*)gs8.Signal, sig.c_str(), 3);

    gs1.PRN = 3;
    gs2.PRN = 8;
    gs3.PRN = 14;
    gs4.PRN = 16;
    gs5.PRN = 3;
    gs6.PRN = 16;
    gs7.PRN = 14;
    gs8.PRN = 16;

    gs2.Pseudorange_m = 22000002.1;
    gs2.Carrier_phase_rads = 45.4;
    gs2.Carrier_Doppler_hz = 321;
    gs2.CN0_dB_hz = 39;

    gs4.Pseudorange_m = 22000000;
    gs4.Carrier_phase_rads = 23.4;
    gs4.Carrier_Doppler_hz = -1534;
    gs4.CN0_dB_hz = 40;

    gs6.Pseudorange_m = 22000000;
    gs6.Carrier_phase_rads = 52.1;
    gs6.Carrier_Doppler_hz = 1534;
    gs6.CN0_dB_hz = 41;

    gs8.Pseudorange_m = 22000000;
    gs8.Carrier_phase_rads = 0.8;
    gs8.Carrier_Doppler_hz = -20;
    gs8.CN0_dB_hz = 42;

    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(1, gs1));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(2, gs2));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(3, gs3));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(4, gs4));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(5, gs5));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(6, gs6));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(7, gs7));
    gnss_pseudoranges_map.insert(std::pair<int, Gnss_Synchro>(8, gs8));

    rp->log_rinex_obs(rp->obsFile, eph_gps, eph_glo, 0.0, gnss_pseudoranges_map);

    rp->obsFile.seekp(0);

    while (!rp->obsFile.eof())
        {
            std::getline(rp->obsFile, line_str);
            if (!no_more_finds)
                {
                    if (line_str.find("R16", 0) != std::string::npos)
                        {
                            no_more_finds = true;
                            line_aux = std::string(line_str);
                        }
                }
        }

    std::string expected_str("R16  22000000.000 6         8.292 6      1534.000 6        41.000  22000000.000 7         0.127 7       -20.000 7        42.000");
    EXPECT_EQ(0, expected_str.compare(line_aux));

    if (remove(rp->obsfilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
}
