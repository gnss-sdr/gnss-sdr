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


TEST(Rinex_Printer_Test, GalileoObsHeader)
{
    std::string line_aux;
    std::string line_str;
    bool no_more_finds = false;
    const Galileo_Ephemeris eph = Galileo_Ephemeris();

    std::shared_ptr<Rinex_Printer> rp1;
    rp1 = std::make_shared<Rinex_Printer>();
    rp1->rinex_obs_header(rp1->obsFile, eph, 0.0);
    rp1->obsFile.seekp(0);

    while(!rp1->obsFile.eof())
        {
            std::getline(rp1->obsFile, line_str);
            if(!no_more_finds)
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
    if(remove(rp1->obsfilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
    line_aux.clear();

    std::shared_ptr<Rinex_Printer> rp2;
    rp2 = std::make_shared<Rinex_Printer>();
    std::string bands("1B 5X 7X");
    rp2->rinex_obs_header(rp2->obsFile, eph, 0.0, bands);
    rp2->obsFile.seekp(0);
    no_more_finds = false;
    while(!rp2->obsFile.eof())
        {
            std::getline(rp2->obsFile, line_str);
            if(!no_more_finds)
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

    if(remove(rp2->obsfilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
}



TEST(Rinex_Printer_Test, GalileoObsLog)
{
    std::string line_aux;
    std::string line_str;
    bool no_more_finds = false;
    const Galileo_Ephemeris eph = Galileo_Ephemeris();

    std::shared_ptr<Rinex_Printer> rp;
    rp = std::make_shared<Rinex_Printer>();
    rp->rinex_obs_header(rp->obsFile, eph, 0.0);

    std::map<int,Gnss_Synchro> gnss_pseudoranges_map;

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

    gnss_pseudoranges_map.insert( std::pair<int, Gnss_Synchro>(1,gs1) );
    gnss_pseudoranges_map.insert( std::pair<int, Gnss_Synchro>(2,gs2) );
    gnss_pseudoranges_map.insert( std::pair<int, Gnss_Synchro>(3,gs3) );
    gnss_pseudoranges_map.insert( std::pair<int, Gnss_Synchro>(4,gs4) );

    rp->log_rinex_obs(rp->obsFile, eph, 0.0, gnss_pseudoranges_map);
    rp->obsFile.seekp(0);

    while(!rp->obsFile.eof())
        {
            std::getline(rp->obsFile, line_str);
            if(!no_more_finds)
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

    if(remove(rp->obsfilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
}

TEST(Rinex_Printer_Test, GalileoObsLogDualBand)
{
    std::string line_aux;
    std::string line_str;
    bool no_more_finds = false;
    const Galileo_Ephemeris eph = Galileo_Ephemeris();

    std::shared_ptr<Rinex_Printer> rp;
    rp = std::make_shared<Rinex_Printer>();
    std::string bands("1B 5X");
    rp->rinex_obs_header(rp->obsFile, eph, 0.0, bands);

    std::map<int,Gnss_Synchro> gnss_pseudoranges_map;

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
    std::memcpy((void*)gs1.Signal, sig.c_str(), 3);
    std::memcpy((void*)gs2.Signal, sig.c_str(), 3);

    sig = "5X";
    std::memcpy((void*)gs3.Signal, sig.c_str(), 3);
    std::memcpy((void*)gs4.Signal, sig.c_str(), 3);

    gs1.PRN = 3;
    gs2.PRN = 8;
    gs3.PRN = 3;
    gs4.PRN = 8;

    gs2.Pseudorange_m = 22000002.1;
    gs2.Carrier_phase_rads = 45.4;
    gs2.Carrier_Doppler_hz = 321;
    gs2.CN0_dB_hz = 39;

    gs4.Pseudorange_m = 22000000;
    gs4.Carrier_phase_rads = 23.4;
    gs4.Carrier_Doppler_hz = 1534;
    gs4.CN0_dB_hz = 42;

    gnss_pseudoranges_map.insert( std::pair<int, Gnss_Synchro>(1,gs1) );
    gnss_pseudoranges_map.insert( std::pair<int, Gnss_Synchro>(2,gs2) );
    gnss_pseudoranges_map.insert( std::pair<int, Gnss_Synchro>(3,gs3) );
    gnss_pseudoranges_map.insert( std::pair<int, Gnss_Synchro>(4,gs4) );

    rp->log_rinex_obs(rp->obsFile, eph, 0.0, gnss_pseudoranges_map);
    rp->obsFile.seekp(0);

    while(!rp->obsFile.eof())
        {
            std::getline(rp->obsFile, line_str);
            if(!no_more_finds)
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

    if(remove(rp->obsfilename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
}


