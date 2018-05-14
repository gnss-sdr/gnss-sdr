/*!
 * \file nma_printer_test.cc
 * \brief Implements Unit Tests for the Nmea_Printer class.
 * \author Carles Fernandez-Prades, 2017. cfernandez(at)cttc.es
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


#include <cstdio>
#include <fstream>
#include <string>
#include "nmea_printer.h"


TEST(NmeaPrinterTest, PrintLine)
{
    std::string filename("nmea_test.nmea");
    rtk_t rtk;
    prcopt_t rtklib_configuration_options;
    rtkinit(&rtk, &rtklib_configuration_options);
    std::shared_ptr<rtklib_solver> pvt_solution = std::make_shared<rtklib_solver>(12, "filename", false, rtk);

    boost::posix_time::ptime pt(boost::gregorian::date(1994, boost::date_time::Nov, 19),
        boost::posix_time::hours(22) + boost::posix_time::minutes(54) + boost::posix_time::seconds(46));  // example from http://aprs.gids.nl/nmea/#rmc
    pvt_solution->set_position_UTC_time(pt);

    arma::vec pos = {49.27416667, -123.18533333, 0};
    pvt_solution->set_rx_pos(pos);

    pvt_solution->set_valid_position(true);

    ASSERT_NO_THROW({
        std::shared_ptr<Nmea_Printer> nmea_printer = std::make_shared<Nmea_Printer>(filename, false, "");
        nmea_printer->Print_Nmea_Line(pvt_solution, false);
    }) << "Failure printing NMEA messages.";

    std::ifstream test_file(filename);
    std::string line;
    std::string GPRMC("$GPRMC");
    if (test_file.is_open())
        {
            while (getline(test_file, line))
                {
                    std::size_t found = line.find(GPRMC);
                    if (found != std::string::npos)
                        {
                            EXPECT_EQ(line, "$GPRMC,225446.000,A,4916.4500,N,12311.1199,W,0.00,0.00,191194,,*1c\r");
                        }
                }
            test_file.close();
        }
    EXPECT_EQ(0, remove(filename.c_str())) << "Failure deleting a temporary file.";
}


TEST(NmeaPrinterTest, PrintLineLessthan10min)
{
    std::string filename("nmea_test.nmea");
    rtk_t rtk;
    prcopt_t rtklib_configuration_options;
    rtkinit(&rtk, &rtklib_configuration_options);
    std::shared_ptr<rtklib_solver> pvt_solution = std::make_shared<rtklib_solver>(12, "filename", false, rtk);

    boost::posix_time::ptime pt(boost::gregorian::date(1994, boost::date_time::Nov, 19),
        boost::posix_time::hours(22) + boost::posix_time::minutes(54) + boost::posix_time::seconds(46));  // example from http://aprs.gids.nl/nmea/#rmc
    pvt_solution->set_position_UTC_time(pt);

    arma::vec pos = {49.07416667, -123.02527778, 0};
    pvt_solution->set_rx_pos(pos);

    pvt_solution->set_valid_position(true);

    ASSERT_NO_THROW({
        std::shared_ptr<Nmea_Printer> nmea_printer = std::make_shared<Nmea_Printer>(filename, false, "");
        nmea_printer->Print_Nmea_Line(pvt_solution, false);
    }) << "Failure printing NMEA messages.";

    std::ifstream test_file(filename);
    std::string line;
    std::string GPRMC("$GPRMC");
    if (test_file.is_open())
        {
            while (getline(test_file, line))
                {
                    std::size_t found = line.find(GPRMC);
                    if (found != std::string::npos)
                        {
                            EXPECT_EQ(line, "$GPRMC,225446.000,A,4904.4500,N,12301.5166,W,0.00,0.00,191194,,*1a\r");
                        }
                }
            test_file.close();
        }
    EXPECT_EQ(0, remove(filename.c_str())) << "Failure deleting a temporary file.";
}
