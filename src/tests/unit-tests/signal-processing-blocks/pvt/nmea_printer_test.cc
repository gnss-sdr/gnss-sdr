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


class NmeaPrinterTest : public ::testing::Test
{
protected:
    NmeaPrinterTest()
    {
        this->conf();
    }
    ~NmeaPrinterTest()
    {
    }
    void conf();
    rtk_t rtk;
    prcopt_t rtklib_configuration_options;
};

void NmeaPrinterTest::conf()
{
    snrmask_t snrmask = {{}, {{}, {}}};
    int positioning_mode = 0;  // Single
    int number_of_frequencies = 1;
    double elevation_mask = 5;
    int navigation_system = 1;  // GPS
    int integer_ambiguity_resolution_gps = 0;
    int integer_ambiguity_resolution_glo = 0;
    int integer_ambiguity_resolution_bds = 0;
    int outage_reset_ambiguity = 5;
    int min_lock_to_fix_ambiguity = 0;
    int iono_model = 0;
    int trop_model = 0;
    int dynamics_model = 0;
    int earth_tide = 0;
    int number_filter_iter = 1;
    double code_phase_error_ratio_l1 = 100.0;
    double code_phase_error_ratio_l2 = 100.0;
    double code_phase_error_ratio_l5 = 100.0;
    double carrier_phase_error_factor_a = 0.003;
    double carrier_phase_error_factor_b = 0.003;
    double bias_0 = 30.0;
    double iono_0 = 0.03;
    double trop_0 = 0.3;
    double sigma_bias = 1e-4;
    double sigma_iono = 1e-3;
    double sigma_trop = 1e-4;
    double sigma_acch = 1e-1;
    double sigma_accv = 1e-2;
    double sigma_pos = 0.0;
    double min_ratio_to_fix_ambiguity = 3.0;
    double min_elevation_to_fix_ambiguity = 0.0;
    double slip_threshold = 0.05;
    double threshold_reject_innovation = 30.0;
    double threshold_reject_gdop = 30.0;
    int sat_PCV = 0;
    int rec_PCV = 0;
    int phwindup = 0;
    int reject_GPS_IIA = 0;
    int raim_fde = 0;

    prcopt_t rtklib_configuration_options = {
        positioning_mode,                                                                  /* positioning mode (PMODE_XXX) see src/algorithms/libs/rtklib/rtklib.h */
        0,                                                                                 /* solution type (0:forward,1:backward,2:combined) */
        number_of_frequencies,                                                             /* number of frequencies (1:L1, 2:L1+L2, 3:L1+L2+L5)*/
        navigation_system,                                                                 /* navigation system  */
        elevation_mask * D2R,                                                              /* elevation mask angle (degrees) */
        snrmask,                                                                           /* snrmask_t snrmask    SNR mask */
        0,                                                                                 /* satellite ephemeris/clock (EPHOPT_XXX) */
        integer_ambiguity_resolution_gps,                                                  /* AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold,4:ppp-ar) */
        integer_ambiguity_resolution_glo,                                                  /* GLONASS AR mode (0:off,1:on,2:auto cal,3:ext cal) */
        integer_ambiguity_resolution_bds,                                                  /* BeiDou AR mode (0:off,1:on) */
        outage_reset_ambiguity,                                                            /* obs outage count to reset bias */
        min_lock_to_fix_ambiguity,                                                         /* min lock count to fix ambiguity */
        10,                                                                                /* min fix count to hold ambiguity */
        1,                                                                                 /* max iteration to resolve ambiguity */
        iono_model,                                                                        /* ionosphere option (IONOOPT_XXX) */
        trop_model,                                                                        /* troposphere option (TROPOPT_XXX) */
        dynamics_model,                                                                    /* dynamics model (0:none, 1:velocity, 2:accel) */
        earth_tide,                                                                        /* earth tide correction (0:off,1:solid,2:solid+otl+pole) */
        number_filter_iter,                                                                /* number of filter iteration */
        0,                                                                                 /* code smoothing window size (0:none) */
        0,                                                                                 /* interpolate reference obs (for post mission) */
        0,                                                                                 /* sbssat_t sbssat  SBAS correction options */
        0,                                                                                 /* sbsion_t sbsion[MAXBAND+1] SBAS satellite selection (0:all) */
        0,                                                                                 /* rover position for fixed mode */
        0,                                                                                 /* base position for relative mode */
                                                                                           /*    0:pos in prcopt,  1:average of single pos, */
                                                                                           /*    2:read from file, 3:rinex header, 4:rtcm pos */
        {code_phase_error_ratio_l1, code_phase_error_ratio_l2, code_phase_error_ratio_l5}, /* eratio[NFREQ] code/phase error ratio */
        {100.0, carrier_phase_error_factor_a, carrier_phase_error_factor_b, 0.0, 1.0},     /* err[5]:  measurement error factor [0]:reserved, [1-3]:error factor a/b/c of phase (m) , [4]:doppler frequency (hz) */
        {bias_0, iono_0, trop_0},                                                          /* std[3]: initial-state std [0]bias,[1]iono [2]trop*/
        {sigma_bias, sigma_iono, sigma_trop, sigma_acch, sigma_accv, sigma_pos},           /* prn[6] process-noise std */
        5e-12,                                                                             /* sclkstab: satellite clock stability (sec/sec) */
        {min_ratio_to_fix_ambiguity, 0.9999, 0.25, 0.1, 0.05, 0.0, 0.0, 0.0},              /* thresar[8]: AR validation threshold */
        min_elevation_to_fix_ambiguity,                                                    /* elevation mask of AR for rising satellite (deg) */
        0.0,                                                                               /* elevation mask to hold ambiguity (deg) */
        slip_threshold,                                                                    /* slip threshold of geometry-free phase (m) */
        30.0,                                                                              /* max difference of time (sec) */
        threshold_reject_innovation,                                                       /* reject threshold of innovation (m) */
        threshold_reject_gdop,                                                             /* reject threshold of gdop */
        {},                                                                                /* double baseline[2] baseline length constraint {const,sigma} (m) */
        {},                                                                                /* double ru[3]  rover position for fixed mode {x,y,z} (ecef) (m) */
        {},                                                                                /* double rb[3]  base position for relative mode {x,y,z} (ecef) (m) */
        {"", ""},                                                                          /* char anttype[2][MAXANT]  antenna types {rover,base}  */
        {{}, {}},                                                                          /* double antdel[2][3]   antenna delta {{rov_e,rov_n,rov_u},{ref_e,ref_n,ref_u}} */
        {},                                                                                /* pcv_t pcvr[2]   receiver antenna parameters {rov,base} */
        {},                                                                                /* unsigned char exsats[MAXSAT]  excluded satellites (1:excluded, 2:included) */
        0,                                                                                 /* max averaging epoches */
        0,                                                                                 /* initialize by restart */
        1,                                                                                 /* output single by dgps/float/fix/ppp outage */
        {"", ""},                                                                          /* char rnxopt[2][256]   rinex options {rover,base} */
        {sat_PCV, rec_PCV, phwindup, reject_GPS_IIA, raim_fde},                            /*  posopt[6] positioning options [0]: satellite and receiver antenna PCV model; [1]: interpolate antenna parameters; [2]: apply phase wind-up correction for PPP modes; [3]: exclude measurements of GPS Block IIA satellites satellite [4]: RAIM FDE (fault detection and exclusion) [5]: handle day-boundary clock jump */
        0,                                                                                 /* solution sync mode (0:off,1:on) */
        {{}, {}},                                                                          /*  odisp[2][6*11] ocean tide loading parameters {rov,base} */
        {{}, {{}, {}}, {{}, {}}, {}, {}},                                                  /*  exterr_t exterr   extended receiver error model */
        0,                                                                                 /* disable L2-AR */
        {}                                                                                 /* char pppopt[256]   ppp option   "-GAP_RESION="  default gap to reset iono parameters (ep) */
    };

    rtkinit(&rtk, &rtklib_configuration_options);
}


TEST_F(NmeaPrinterTest, PrintLine)
{
    std::string filename("nmea_test.nmea");
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


TEST_F(NmeaPrinterTest, PrintLineLessthan10min)
{
    std::string filename("nmea_test.nmea");
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
