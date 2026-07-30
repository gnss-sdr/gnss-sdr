/*!
 * \file rinex_printer_test.cc
 * \brief Implements Unit Tests for the Rinex_Printer class.
 * \author Carles Fernandez-Prades, 2016. cfernandez(at)cttc.es
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

#include "pvt_conf.h"
#include "rinex_printer.h"
#include "rtklib_rtkcmn.h"
#include "rtklib_rtkpos.h"
#include "rtklib_solver.h"
#include "signal_enabled_flags.h"
#include <gtest/gtest.h>
#include <fstream>
#include <string>
#include <utility>


class RinexPrinterTest : public ::testing::Test
{
protected:
    RinexPrinterTest()
    {
        this->conf();
    }
    ~RinexPrinterTest() = default;
    void conf();
    rtk_t rtk;
};

void RinexPrinterTest::conf()
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
        {},                                                                                /* char pppopt[256]   ppp option   "-GAP_RESION="  default gap to reset iono parameters (ep) */
        true                                                                               /* enable Bancroft initialization for the first iteration of the PVT computation, useful in some geometries */
    };

    rtkinit(&rtk, &rtklib_configuration_options);
}


void find_obs_record_lines(const std::string& obsfile, const std::string& sat, std::string& line_epoch, std::string& line_sat)
{
    std::fstream fstr(obsfile.c_str(), std::fstream::in);
    if (!fstr.is_open())
        {
            return;
        }
    fstr.seekg(0);

    std::string line_str;
    bool no_more_finds = false;

    while (!fstr.eof())
        {
            std::getline(fstr, line_str);
            if (!no_more_finds)
                {
                    if (line_str.find('>', 0) != std::string::npos)
                        {
                            line_epoch = line_str;
                        }
                    // Observation records start with the satellite identifier; header
                    // lines may also contain it (e.g. GLONASS SLOT / FRQ #), so the
                    // match is anchored to the beginning of the line
                    if (line_str.compare(0, sat.size(), sat) == 0)
                        {
                            no_more_finds = true;
                            line_sat = line_str;
                        }
                }
        }

    fstr.close();
}


std::string find_rinex_header_line(const std::string& filename, const std::string& label)
{
    std::fstream fstr(filename.c_str(), std::fstream::in);
    if (!fstr.is_open())
        {
            return {};
        }
    fstr.seekg(0);

    std::string line_str;
    while (!fstr.eof())
        {
            std::getline(fstr, line_str);
            if (line_str.find(label, 59) != std::string::npos)
                {
                    return line_str;
                }
        }

    return {};
}


std::string find_line_starting_with(const std::string& filename, const std::string& prefix)
{
    std::fstream fstr(filename.c_str(), std::fstream::in);
    if (!fstr.is_open())
        {
            return {};
        }
    fstr.seekg(0);

    std::string line_str;
    while (!fstr.eof())
        {
            std::getline(fstr, line_str);
            if (line_str.compare(0, prefix.length(), prefix) == 0)
                {
                    return line_str;
                }
        }

    return {};
}


std::string find_line_containing(const std::string& filename, const std::string& text)
{
    std::fstream fstr(filename.c_str(), std::fstream::in);
    if (!fstr.is_open())
        {
            return {};
        }
    fstr.seekg(0);

    std::string line_str;
    while (!fstr.eof())
        {
            std::getline(fstr, line_str);
            if (line_str.find(text) != std::string::npos)
                {
                    return line_str;
                }
        }

    return {};
}


int32_t count_record_body_lines(const std::string& filename, const std::string& record_header)
{
    std::fstream fstr(filename.c_str(), std::fstream::in);
    if (!fstr.is_open())
        {
            return -1;
        }
    fstr.seekg(0);

    std::string line_str;
    bool found = false;
    int32_t body_lines = 0;
    while (!fstr.eof())
        {
            std::getline(fstr, line_str);
            if (!found)
                {
                    if (line_str.compare(0, record_header.length(), record_header) == 0)
                        {
                            found = true;
                        }
                }
            else
                {
                    if (line_str.empty() || line_str[0] == '>')
                        {
                            break;
                        }
                    body_lines++;
                }
        }

    return found ? body_lines : -1;
}


int32_t expand_test_lnav_utc_week(int32_t utc_week, int32_t reference_week)
{
    int32_t expanded_week = utc_week;
    while ((expanded_week - reference_week) < -128)
        {
            expanded_week += 256;
        }
    while ((expanded_week - reference_week) > 127)
        {
            expanded_week -= 256;
        }

    return expanded_week;
}


TEST_F(RinexPrinterTest, GpsUtcHeaderExpandsEightBitWeekFields)
{
    Pvt_Conf conf;
    conf.use_e6_for_pvt = false;
    const auto signal_enabled_flags = GPS_1C;
    auto pvt_solution = std::make_shared<Rtklib_Solver>(rtk, conf, "filename", signal_enabled_flags, false, false);

    Gps_Ephemeris eph;
    eph.PRN = 1;
    eph.WN = 512;
    pvt_solution->gps_ephemeris_map[1] = eph;
    pvt_solution->gps_utc_model.A0 = 1.0e-9;
    pvt_solution->gps_utc_model.A1 = 0.0;
    pvt_solution->gps_utc_model.tot = 0;
    pvt_solution->gps_utc_model.WN_T = 5;
    pvt_solution->gps_utc_model.DeltaT_LS = 18;
    pvt_solution->gps_utc_model.WN_LSF = 7;
    pvt_solution->gps_utc_model.DN = 4;
    pvt_solution->gps_utc_model.DeltaT_LSF = 19;

    Gnss_Synchro gs{};
    gs.System = 'G';
    gs.PRN = 1;
    std::memcpy(static_cast<void*>(gs.Signal), "1C", 3);
    gs.Pseudorange_m = 22000000.0;
    gs.Carrier_phase_rads = 23.4;
    gs.Carrier_Doppler_hz = 1534.0;
    gs.CN0_dB_hz = 42.0;
    gs.Flag_valid_pseudorange = true;
    std::map<int, Gnss_Synchro> gnss_observables_map;
    gnss_observables_map[1] = gs;

    auto rp = std::make_shared<Rinex_Printer>(signal_enabled_flags);
    rp->print_rinex_annotation(pvt_solution.get(), gnss_observables_map, 42.0, true);

    const std::string obsfile = rp->get_obsfilename();
    const std::string navfile = rp->get_navfilename()[0];
    rp = nullptr;

    const int32_t reference_week = adjgpsweek(eph.WN);
    const int32_t expected_wn_t = expand_test_lnav_utc_week(pvt_solution->gps_utc_model.WN_T, reference_week);
    const int32_t expected_wn_lsf = expand_test_lnav_utc_week(pvt_solution->gps_utc_model.WN_LSF, reference_week);

    const std::string time_corr_line = find_rinex_header_line(navfile, "TIME SYSTEM CORR");
    EXPECT_NE(std::string::npos, time_corr_line.find(std::to_string(expected_wn_t)));

    const std::string leap_second_line = find_rinex_header_line(navfile, "LEAP SECONDS");
    EXPECT_NE(std::string::npos, leap_second_line.find(std::to_string(expected_wn_lsf)));

    fs::remove(obsfile);
    fs::remove(navfile);
}


TEST_F(RinexPrinterTest, Rinex4GpsNavAndObs)
{
    Pvt_Conf conf;
    conf.use_e6_for_pvt = false;
    const auto signal_enabled_flags = GPS_1C;
    auto pvt_solution = std::make_shared<Rtklib_Solver>(rtk, conf, "filename", signal_enabled_flags, false, false);

    Gps_Ephemeris eph;
    eph.PRN = 1;
    eph.WN = 512;
    pvt_solution->gps_ephemeris_map[1] = eph;
    pvt_solution->gps_iono.alpha0 = 1.1175870895385742e-08;
    pvt_solution->gps_iono.alpha1 = -7.4505805969238281e-09;
    pvt_solution->gps_iono.alpha2 = -5.9604644775390625e-08;
    pvt_solution->gps_iono.alpha3 = 1.1920928955078125e-07;
    pvt_solution->gps_iono.beta0 = 90112.0;
    pvt_solution->gps_iono.beta1 = -16384.0;
    pvt_solution->gps_iono.beta2 = -131072.0;
    pvt_solution->gps_iono.beta3 = 589824.0;
    pvt_solution->gps_utc_model.A0 = 9.3132257461547852e-10;
    pvt_solution->gps_utc_model.A1 = 8.8817841970012523e-16;
    pvt_solution->gps_utc_model.tot = 233472;
    pvt_solution->gps_utc_model.WN_T = 5;
    pvt_solution->gps_utc_model.DeltaT_LS = 18;
    pvt_solution->gps_utc_model.WN_LSF = 7;
    pvt_solution->gps_utc_model.DN = 7;
    pvt_solution->gps_utc_model.DeltaT_LSF = 18;

    Gnss_Synchro gs{};
    gs.System = 'G';
    gs.PRN = 1;
    std::memcpy(static_cast<void*>(gs.Signal), "1C", 3);
    gs.Pseudorange_m = 22000000.0;
    gs.Carrier_phase_rads = 23.4;
    gs.Carrier_Doppler_hz = 1534.0;
    gs.CN0_dB_hz = 42.0;
    gs.Flag_valid_pseudorange = true;
    std::map<int, Gnss_Synchro> gnss_observables_map;
    gnss_observables_map[1] = gs;

    auto rp = std::make_shared<Rinex_Printer>(signal_enabled_flags, 4);
    rp->print_rinex_annotation(pvt_solution.get(), gnss_observables_map, 42.0, true);

    const std::string obsfile = rp->get_obsfilename();
    const std::string navfile = rp->get_navfilename()[0];
    rp = nullptr;  // close the RINEX files so we can inspect them

    const std::string obs_version_line = find_rinex_header_line(obsfile, "RINEX VERSION / TYPE");
    EXPECT_NE(std::string::npos, obs_version_line.find("4.02"));
    EXPECT_NE(std::string::npos, obs_version_line.find("OBSERVATION DATA"));

    const std::string nav_version_line = find_rinex_header_line(navfile, "RINEX VERSION / TYPE");
    EXPECT_NE(std::string::npos, nav_version_line.find("4.02"));
    EXPECT_NE(std::string::npos, nav_version_line.find("N: GNSS NAV DATA"));

    // From RINEX 4 on, the iono and time system corrections are ION / STO data
    // records instead of header lines
    EXPECT_TRUE(find_rinex_header_line(navfile, "IONOSPHERIC CORR").empty());
    EXPECT_TRUE(find_rinex_header_line(navfile, "TIME SYSTEM CORR").empty());

    const std::string leap_second_line = find_rinex_header_line(navfile, "LEAP SECONDS");
    EXPECT_NE(std::string::npos, leap_second_line.find("18"));

    EXPECT_FALSE(find_line_starting_with(navfile, "> EPH G01 LNAV").empty());
    EXPECT_FALSE(find_line_starting_with(navfile, "> ION G   LNAV").empty());
    EXPECT_FALSE(find_line_starting_with(navfile, "> STO G   LNAV").empty());

    const std::string sto_line = find_line_containing(navfile, "GPUT");
    EXPECT_NE(std::string::npos, sto_line.find("UTC(USNO)"));

    // RINEX 4 navigation records use the 'e' exponent indicator instead of 'D'
    EXPECT_TRUE(find_line_containing(navfile, "D+0").empty());
    EXPECT_TRUE(find_line_containing(navfile, "D-0").empty());
    EXPECT_FALSE(find_line_containing(navfile, "e+0").empty());

    fs::remove(obsfile);
    fs::remove(navfile);
}


TEST_F(RinexPrinterTest, Rinex4GalileoNav)
{
    Pvt_Conf conf;
    conf.use_e6_for_pvt = false;
    const auto signal_enabled_flags = GAL_1B;
    auto pvt_solution = std::make_shared<Rtklib_Solver>(rtk, conf, "filename", signal_enabled_flags, false, false);

    auto eph = Galileo_Ephemeris();
    eph.PRN = 11;
    eph.WN = 120;
    pvt_solution->galileo_ephemeris_map[11] = std::move(eph);
    pvt_solution->galileo_iono.ai0 = 45.75;
    pvt_solution->galileo_iono.ai1 = 0.1875;
    pvt_solution->galileo_iono.ai2 = 0.0135;
    pvt_solution->galileo_utc_model.A0 = -9.3132257461547852e-10;
    pvt_solution->galileo_utc_model.A1 = -8.8817841970012523e-16;
    pvt_solution->galileo_utc_model.tot = 432000;
    pvt_solution->galileo_utc_model.WNot = 41;
    pvt_solution->galileo_utc_model.Delta_tLS = 18;
    pvt_solution->galileo_utc_model.WN_LSF = 137;
    pvt_solution->galileo_utc_model.DN = 7;
    pvt_solution->galileo_utc_model.Delta_tLSF = 18;

    std::map<int, Gnss_Synchro> gnss_observables_map;
    Gnss_Synchro gs{};
    gs.PRN = 11;
    gnss_observables_map[11] = std::move(gs);

    auto rp = std::make_shared<Rinex_Printer>(signal_enabled_flags, 4);
    rp->print_rinex_annotation(pvt_solution.get(), gnss_observables_map, 0.0, true);

    const std::string obsfile = rp->get_obsfilename();
    const std::string navfile = rp->get_navfilename()[0];
    rp = nullptr;  // close the RINEX files so we can inspect them

    EXPECT_FALSE(find_line_starting_with(navfile, "> EPH E11 INAV").empty());
    EXPECT_FALSE(find_line_starting_with(navfile, "> ION E   IFNV").empty());
    EXPECT_FALSE(find_line_starting_with(navfile, "> STO E   IFNV").empty());

    const std::string sto_line = find_line_containing(navfile, "GAUT");
    EXPECT_NE(std::string::npos, sto_line.find("UTCGAL"));

    fs::remove(obsfile);
    fs::remove(navfile);
}


TEST_F(RinexPrinterTest, Rinex4GlonassNav)
{
    Pvt_Conf conf;
    conf.use_e6_for_pvt = false;
    const auto signal_enabled_flags = GLO_1G;
    auto pvt_solution = std::make_shared<Rtklib_Solver>(rtk, conf, "filename", signal_enabled_flags, false, false);

    auto eph = Glonass_Gnav_Ephemeris();
    eph.PRN = 1;
    eph.d_yr = 2020;
    eph.d_N_T = 260;
    pvt_solution->glonass_gnav_ephemeris_map[1] = std::move(eph);
    pvt_solution->glonass_gnav_utc_model.d_tau_c = -9.3132257461547852e-09;
    pvt_solution->glonass_gnav_utc_model.d_tau_gps = 1.8626451492309570e-09;

    std::map<int, Gnss_Synchro> gnss_observables_map;
    Gnss_Synchro gs{};
    gs.PRN = 1;
    gnss_observables_map[1] = std::move(gs);

    auto rp = std::make_shared<Rinex_Printer>(signal_enabled_flags, 4);
    rp->print_rinex_annotation(pvt_solution.get(), gnss_observables_map, 0.0, true);

    const std::string obsfile = rp->get_obsfilename();
    const std::string navfile = rp->get_navfilename()[0];
    rp = nullptr;  // close the RINEX files so we can inspect them

    EXPECT_FALSE(find_line_starting_with(navfile, "> EPH R01 FDMA").empty());
    EXPECT_FALSE(find_line_starting_with(navfile, "> STO R   FDMA").empty());

    const std::string sto_line = find_line_containing(navfile, "GLUT");
    EXPECT_NE(std::string::npos, sto_line.find("UTC(SU)"));
    EXPECT_FALSE(find_line_containing(navfile, "GLGP").empty());

    // The FDMA record has the SV / EPOCH / SV CLK line plus four broadcast
    // orbit lines (see RINEX 4.02 specification, Table A15)
    EXPECT_EQ(5, count_record_body_lines(navfile, "> EPH R01 FDMA"));

    fs::remove(obsfile);
    fs::remove(navfile);
}


TEST_F(RinexPrinterTest, Rinex4GpsCnavNav)
{
    Pvt_Conf conf;
    conf.use_e6_for_pvt = false;
    const auto signal_enabled_flags = GPS_2S;
    auto pvt_solution = std::make_shared<Rtklib_Solver>(rtk, conf, "filename", signal_enabled_flags, false, false);

    Gps_CNAV_Ephemeris eph;
    eph.PRN = 1;
    eph.WN = 2338;
    eph.WNop = 34;
    pvt_solution->gps_cnav_ephemeris_map[1] = eph;
    pvt_solution->gps_cnav_iono.alpha0 = 1.1175870895385742e-08;
    pvt_solution->gps_cnav_iono.beta0 = 90112.0;
    pvt_solution->gps_cnav_utc_model.A0 = 9.3132257461547852e-10;
    pvt_solution->gps_cnav_utc_model.A1 = 8.8817841970012523e-16;
    pvt_solution->gps_cnav_utc_model.tot = 233472;
    pvt_solution->gps_cnav_utc_model.WN_T = 2338;
    pvt_solution->gps_cnav_utc_model.DeltaT_LS = 18;

    std::map<int, Gnss_Synchro> gnss_observables_map;
    Gnss_Synchro gs{};
    gs.System = 'G';
    gs.PRN = 1;
    std::memcpy(static_cast<void*>(gs.Signal), "2S", 3);
    gnss_observables_map[1] = std::move(gs);

    auto rp = std::make_shared<Rinex_Printer>(signal_enabled_flags, 4);
    rp->print_rinex_annotation(pvt_solution.get(), gnss_observables_map, 42.0, true);

    const std::string obsfile = rp->get_obsfilename();
    const std::string navfile = rp->get_navfilename()[0];
    rp = nullptr;  // close the RINEX files so we can inspect them

    EXPECT_FALSE(find_line_starting_with(navfile, "> EPH G01 CNAV").empty());
    EXPECT_FALSE(find_line_starting_with(navfile, "> ION G   CNVX").empty());
    EXPECT_FALSE(find_line_starting_with(navfile, "> STO G   CNVX").empty());

    // The CNAV record has the SV / EPOCH / SV CLK line plus eight broadcast
    // orbit lines (see RINEX 4.02 specification, Table A10)
    EXPECT_EQ(9, count_record_body_lines(navfile, "> EPH G01 CNAV"));

    fs::remove(obsfile);
    fs::remove(navfile);
}


TEST_F(RinexPrinterTest, Rinex4QzssNav)
{
    Pvt_Conf conf;
    conf.use_e6_for_pvt = false;
    const auto signal_enabled_flags = QZS_J1;
    auto pvt_solution = std::make_shared<Rtklib_Solver>(rtk, conf, "filename", signal_enabled_flags, false, false);

    Gps_Ephemeris eph;
    eph.PRN = 193;  // QZSS J01
    eph.WN = 512;
    pvt_solution->gps_ephemeris_map[193] = eph;
    pvt_solution->qzss_iono.alpha0 = 2.6077032089233398e-08;
    pvt_solution->qzss_iono.alpha1 = -7.4505805969238281e-09;
    pvt_solution->qzss_iono.beta0 = 118784.0;
    pvt_solution->qzss_iono.beta1 = -16384.0;
    pvt_solution->qzss_utc_model.A0 = -9.3132257461547852e-10;
    pvt_solution->qzss_utc_model.A1 = -8.8817841970012523e-16;
    pvt_solution->qzss_utc_model.tot = 233472;
    pvt_solution->qzss_utc_model.WN_T = 5;
    pvt_solution->qzss_utc_model.DeltaT_LS = 18;
    pvt_solution->qzss_utc_model.WN_LSF = 7;
    pvt_solution->qzss_utc_model.DN = 7;
    pvt_solution->qzss_utc_model.DeltaT_LSF = 18;

    std::map<int, Gnss_Synchro> gnss_observables_map;
    Gnss_Synchro gs{};
    gs.System = 'J';
    gs.PRN = 193;
    std::memcpy(static_cast<void*>(gs.Signal), "J1", 3);
    gnss_observables_map[193] = std::move(gs);

    auto rp = std::make_shared<Rinex_Printer>(signal_enabled_flags, 4);
    rp->print_rinex_annotation(pvt_solution.get(), gnss_observables_map, 42.0, true);

    const std::string obsfile = rp->get_obsfilename();
    const std::string navfile = rp->get_navfilename()[0];
    rp = nullptr;  // close the RINEX files so we can inspect them

    EXPECT_FALSE(find_line_starting_with(navfile, "> EPH J01 LNAV").empty());
    EXPECT_FALSE(find_line_starting_with(navfile, "> ION J   LNAV").empty());
    EXPECT_FALSE(find_line_starting_with(navfile, "> STO J   LNAV").empty());

    const std::string sto_line = find_line_containing(navfile, "QZUT");
    EXPECT_NE(std::string::npos, sto_line.find("UTC(NICT)"));

    const std::string leap_second_line = find_rinex_header_line(navfile, "LEAP SECONDS");
    EXPECT_NE(std::string::npos, leap_second_line.find("18"));

    fs::remove(obsfile);
    fs::remove(navfile);
}


TEST_F(RinexPrinterTest, GalileoObsHeader)
{
    Pvt_Conf conf;
    conf.use_e6_for_pvt = false;
    const auto signal_enabled_flags = GAL_1B;
    auto pvt_solution = std::make_shared<Rtklib_Solver>(rtk, conf, "filename", signal_enabled_flags, false, false);
    auto eph = Galileo_Ephemeris();
    eph.PRN = 1;
    pvt_solution->galileo_ephemeris_map[1] = std::move(eph);

    std::map<int, Gnss_Synchro> gnss_observables_map;
    Gnss_Synchro gs{};
    gs.PRN = 1;
    gnss_observables_map[1] = std::move(gs);

    auto rp = std::make_shared<Rinex_Printer>(signal_enabled_flags);

    rp->print_rinex_annotation(pvt_solution.get(), gnss_observables_map, 0.0, true);

    std::string obsfile = rp->get_obsfilename();
    std::string navfile = rp->get_navfilename()[0];

    rp = nullptr;  // close the RINEX files so we can inspect them

    std::fstream fstr(obsfile.c_str(), std::fstream::in);

    fstr.seekg(0);
    std::string line_aux;
    std::string line_str;
    bool no_more_finds = false;

    while (!fstr.eof())
        {
            std::getline(fstr, line_str);
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
    fstr.close();
    fs::remove(obsfile);
    fs::remove(navfile);

    auto rp2 = std::make_shared<Rinex_Printer>(GAL_1B | GAL_E5b);

    rp2->print_rinex_annotation(pvt_solution.get(), gnss_observables_map, 0.0, true);
    obsfile = rp2->get_obsfilename();
    navfile = rp2->get_navfilename()[0];

    rp2 = nullptr;  // close the RINEX files so we can inspect them

    std::fstream fstr2(obsfile.c_str(), std::fstream::in);
    fstr2.seekg(0);

    no_more_finds = false;
    while (!fstr2.eof())
        {
            std::getline(fstr2, line_str);
            if (!no_more_finds)
                {
                    if (line_str.find("SYS / # / OBS TYPES", 59) != std::string::npos)
                        {
                            no_more_finds = true;
                            line_aux = std::string(line_str);
                        }
                }
        }
    std::string expected_str2("E    8 C1B L1B D1B S1B C7X L7X D7X S7X                      SYS / # / OBS TYPES ");
    EXPECT_EQ(0, expected_str2.compare(line_aux));
    fstr2.close();
    fs::remove(obsfile);
    fs::remove(navfile);
}


TEST_F(RinexPrinterTest, GlonassObsHeader)
{
    Pvt_Conf conf;
    conf.use_e6_for_pvt = false;
    const auto signal_enabled_flags = GLO_1G;
    auto pvt_solution = std::make_shared<Rtklib_Solver>(rtk, conf, "filename", signal_enabled_flags, false, false);
    auto eph = Glonass_Gnav_Ephemeris();
    eph.PRN = 1;
    pvt_solution->glonass_gnav_ephemeris_map[1] = std::move(eph);

    std::map<int, Gnss_Synchro> gnss_observables_map;
    Gnss_Synchro gs{};
    gs.PRN = 1;
    gnss_observables_map[1] = std::move(gs);

    auto rp = std::make_shared<Rinex_Printer>(signal_enabled_flags);

    rp->print_rinex_annotation(pvt_solution.get(), gnss_observables_map, 0.0, true);

    std::string obsfile = rp->get_obsfilename();
    std::string navfile = rp->get_navfilename()[0];

    rp = nullptr;  // close the RINEX files so we can inspect them

    std::fstream fstr(obsfile.c_str(), std::fstream::in);

    fstr.seekg(0);
    std::string line_aux;
    std::string line_slot_frq;
    std::string line_str;

    while (!fstr.eof())
        {
            std::getline(fstr, line_str);
            if (line_aux.empty() && line_str.find("SYS / # / OBS TYPES", 59) != std::string::npos)
                {
                    line_aux = std::string(line_str);
                }
            if (line_slot_frq.empty() && line_str.find("GLONASS SLOT / FRQ #", 59) != std::string::npos)
                {
                    line_slot_frq = std::string(line_str);
                }
        }
    std::string expected_str("R    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES ");
    std::string expected_slot_frq(" 27 R01  1 R02 -4 R03  5 R04  6 R05  1 R06 -4 R07  5 R08  6 GLONASS SLOT / FRQ #");
    EXPECT_EQ(0, expected_str.compare(line_aux));
    EXPECT_EQ(0, expected_slot_frq.compare(line_slot_frq));
    fstr.close();
    fs::remove(obsfile);
    fs::remove(navfile);
}


TEST_F(RinexPrinterTest, MixedObsHeader)
{
    std::string line_aux;
    std::string line_aux2;
    std::string line_str;
    bool no_more_finds = false;
    auto eph_gal = Galileo_Ephemeris();
    auto eph_gps = Gps_Ephemeris();
    eph_gal.PRN = 1;
    eph_gps.PRN = 1;
    Pvt_Conf conf;
    conf.use_e6_for_pvt = false;
    const auto signal_enabled_flags = GPS_1C | GAL_1B | GAL_E5a;
    auto pvt_solution = std::make_shared<Rtklib_Solver>(rtk, conf, "filename", signal_enabled_flags, false, false);
    pvt_solution->galileo_ephemeris_map[1] = std::move(eph_gal);

    pvt_solution->gps_ephemeris_map[1] = std::move(eph_gps);

    std::map<int, Gnss_Synchro> gnss_observables_map;
    Gnss_Synchro gs{};
    gs.PRN = 1;
    gnss_observables_map[1] = gs;
    gnss_observables_map[2] = std::move(gs);

    auto rp = std::make_shared<Rinex_Printer>(signal_enabled_flags);

    rp->print_rinex_annotation(pvt_solution.get(), gnss_observables_map, 0.0, true);

    std::string obsfile = rp->get_obsfilename();
    std::string navfile = rp->get_navfilename()[0];

    rp = nullptr;  // close the RINEX files so we can inspect them

    std::fstream fstr(obsfile.c_str(), std::fstream::in);

    fstr.seekg(0);
    int systems_found = 0;

    while (!fstr.eof())
        {
            std::getline(fstr, line_str);
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
    fstr.close();
    fs::remove(obsfile);
    fs::remove(navfile);
}


TEST_F(RinexPrinterTest, MixedObsHeaderGpsGlo)
{
    std::string line_aux;
    std::string line_aux2;
    std::string line_str;
    bool no_more_finds = false;
    auto eph_glo = Glonass_Gnav_Ephemeris();
    auto eph_gps = Gps_Ephemeris();
    eph_glo.PRN = 1;
    eph_gps.PRN = 1;
    Pvt_Conf conf;
    conf.use_e6_for_pvt = false;
    const auto signal_enabled_flags = GPS_1C | GLO_1G;
    auto pvt_solution = std::make_shared<Rtklib_Solver>(rtk, conf, "filename", signal_enabled_flags, false, false);
    pvt_solution->glonass_gnav_ephemeris_map[1] = std::move(eph_glo);

    pvt_solution->gps_ephemeris_map[1] = std::move(eph_gps);

    std::map<int, Gnss_Synchro> gnss_observables_map;
    Gnss_Synchro gs{};
    gs.PRN = 1;
    gnss_observables_map[1] = gs;
    gnss_observables_map[2] = std::move(gs);

    auto rp = std::make_shared<Rinex_Printer>(signal_enabled_flags);

    rp->print_rinex_annotation(pvt_solution.get(), gnss_observables_map, 0.0, true);

    std::string obsfile = rp->get_obsfilename();
    std::string navfile = rp->get_navfilename()[0];

    rp = nullptr;  // close the RINEX files so we can inspect them

    std::fstream fstr(obsfile.c_str(), std::fstream::in);

    fstr.seekg(0);

    int systems_found = 0;
    while (!fstr.eof())
        {
            std::getline(fstr, line_str);
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
    fstr.close();
    fs::remove(obsfile);
    fs::remove(navfile);
}


TEST_F(RinexPrinterTest, GalileoObsLog)
{
    auto eph = Galileo_Ephemeris();
    eph.PRN = 1;
    Pvt_Conf conf;
    conf.use_e6_for_pvt = false;
    const auto signal_enabled_flags = GAL_1B;
    auto pvt_solution = std::make_shared<Rtklib_Solver>(rtk, conf, "filename", signal_enabled_flags, false, false);
    pvt_solution->galileo_ephemeris_map[1] = eph;
    std::map<int, Gnss_Synchro> gnss_observables_map;

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

    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(1, gs1));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(2, gs2));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(3, gs3));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(4, gs4));

    auto rp = std::make_shared<Rinex_Printer>(signal_enabled_flags);
    rp->print_rinex_annotation(pvt_solution.get(), gnss_observables_map, 0.0, true);

    std::string obsfile = rp->get_obsfilename();
    std::string navfile = rp->get_navfilename()[0];

    rp = nullptr;  // close the RINEX files so we can inspect them

    std::string line_epoch;
    std::string line_sat;
    find_obs_record_lines(obsfile, "E22", line_epoch, line_sat);

    std::string expected_epoch = "> 1999 08 22 00 00 00.0000000  0  4                                             ";
    std::string expected_sat("E22  22000000.000 7         3.724 7      1534.000 7        42.000               ");
    EXPECT_EQ(0, expected_epoch.compare(line_epoch));
    EXPECT_EQ(0, expected_sat.compare(line_sat));

    fs::remove(obsfile);
    fs::remove(navfile);
}


TEST_F(RinexPrinterTest, GlonassObsLog)
{
    auto eph = Glonass_Gnav_Ephemeris();
    eph.PRN = 22;
    Pvt_Conf conf;
    conf.use_e6_for_pvt = false;
    const auto signal_enabled_flags = GLO_1G;
    auto pvt_solution = std::make_shared<Rtklib_Solver>(rtk, conf, "filename", signal_enabled_flags, false, false);
    pvt_solution->glonass_gnav_ephemeris_map[1] = eph;
    std::map<int, Gnss_Synchro> gnss_observables_map;

    Gnss_Synchro gs1 = Gnss_Synchro();
    Gnss_Synchro gs2 = Gnss_Synchro();
    Gnss_Synchro gs3 = Gnss_Synchro();
    Gnss_Synchro gs4 = Gnss_Synchro();

    std::string sys = "R";
    gs1.System = *sys.c_str();
    gs2.System = *sys.c_str();
    gs3.System = *sys.c_str();
    gs4.System = *sys.c_str();

    std::string sig = "1G";
    std::memcpy(reinterpret_cast<void*>(gs1.Signal), sig.c_str(), 3);
    std::memcpy(reinterpret_cast<void*>(gs2.Signal), sig.c_str(), 3);
    std::memcpy(reinterpret_cast<void*>(gs3.Signal), sig.c_str(), 3);
    std::memcpy(reinterpret_cast<void*>(gs4.Signal), sig.c_str(), 3);

    gs1.PRN = 3;
    gs2.PRN = 8;
    gs3.PRN = 10;
    gs4.PRN = 22;

    gs4.Pseudorange_m = 22000000;
    gs4.Carrier_phase_rads = 23.4;
    gs4.Carrier_Doppler_hz = 1534;
    gs4.CN0_dB_hz = 42;

    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(1, gs1));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(2, gs2));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(3, gs3));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(4, gs4));

    auto rp = std::make_shared<Rinex_Printer>(signal_enabled_flags);
    rp->print_rinex_annotation(pvt_solution.get(), gnss_observables_map, 0.0, true);

    std::string obsfile = rp->get_obsfilename();
    std::string navfile = rp->get_navfilename()[0];

    rp = nullptr;  // close the RINEX files so we can inspect them

    std::string line_epoch;
    std::string line_sat;
    find_obs_record_lines(obsfile, "R22", line_epoch, line_sat);

    std::string expected_epoch = "> 1972 12 31 00 00 00.0000000  0  4                                             ";
    std::string expected_sat("R22  22000000.000 7         3.724 7      1534.000 7        42.000               ");
    EXPECT_EQ(0, expected_epoch.compare(line_epoch));
    EXPECT_EQ(0, expected_sat.compare(line_sat));

    fs::remove(navfile);
    fs::remove(obsfile);
}


TEST_F(RinexPrinterTest, GpsObsLogDualBand)
{
    auto eph = Gps_Ephemeris();
    auto eph_cnav = Gps_CNAV_Ephemeris();
    eph.PRN = 1;
    eph_cnav.PRN = 1;
    Pvt_Conf conf;
    conf.use_e6_for_pvt = false;
    const auto signal_enabled_flags = GPS_1C | GPS_2S;
    auto pvt_solution = std::make_shared<Rtklib_Solver>(rtk, conf, "filename", signal_enabled_flags, false, false);
    pvt_solution->gps_ephemeris_map[1] = std::move(eph);
    pvt_solution->gps_cnav_ephemeris_map[1] = std::move(eph_cnav);
    std::map<int, Gnss_Synchro> gnss_observables_map;

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

    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(1, gs1));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(2, gs2));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(3, gs3));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(4, gs4));

    auto rp = std::make_shared<Rinex_Printer>(signal_enabled_flags);
    rp->print_rinex_annotation(pvt_solution.get(), gnss_observables_map, 0.0, true);

    std::string obsfile = rp->get_obsfilename();
    std::string navfile = rp->get_navfilename()[0];

    rp = nullptr;  // close the RINEX files so we can inspect them

    std::string line_epoch;
    std::string line_sat;
    find_obs_record_lines(obsfile, "G08", line_epoch, line_sat);

    std::string expected_epoch = "> 2019 04 14 00 00 00.0000000  0  3                                             ";
    std::string expected_sat("G08  22000002.100 6         7.226 6       321.000 6        39.000  22000000.000 7         3.724 7      1534.000 7        42.000");
    EXPECT_EQ(0, expected_epoch.compare(line_epoch));
    EXPECT_EQ(0, expected_sat.compare(line_sat));

    fs::remove(navfile);
    fs::remove(obsfile);
}


TEST_F(RinexPrinterTest, GalileoObsLogDualBand)
{
    Pvt_Conf conf;
    conf.use_e6_for_pvt = false;
    const auto signal_enabled_flags = GAL_1B | GAL_E5a;
    auto pvt_solution = std::make_shared<Rtklib_Solver>(rtk, conf, "filename", signal_enabled_flags, false, false);
    auto eph = Galileo_Ephemeris();
    eph.PRN = 1;
    pvt_solution->galileo_ephemeris_map[1] = eph;

    std::map<int, Gnss_Synchro> gnss_observables_map;
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

    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(1, gs1));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(2, gs2));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(3, gs3));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(4, gs4));

    auto rp = std::make_shared<Rinex_Printer>(signal_enabled_flags);

    rp->print_rinex_annotation(pvt_solution.get(), gnss_observables_map, 0.0, true);

    std::string obsfile = rp->get_obsfilename();
    std::string navfile = rp->get_navfilename()[0];

    rp = nullptr;  // close the RINEX files so we can inspect them

    std::string line_epoch;
    std::string line_sat;
    find_obs_record_lines(obsfile, "E08", line_epoch, line_sat);

    std::string expected_epoch = "> 1999 08 22 00 00 00.0000000  0  2                                             ";
    std::string expected_sat("E08  22000002.100 6         7.226 6       321.000 6        39.000  22000000.000 7         3.724 7      1534.000 7        42.000");
    EXPECT_EQ(0, expected_epoch.compare(line_epoch));
    EXPECT_EQ(0, expected_sat.compare(line_sat));

    fs::remove(navfile);
    fs::remove(obsfile);
}


TEST_F(RinexPrinterTest, MixedObsLog)
{
    auto eph_gps = Gps_Ephemeris();
    auto eph_gal = Galileo_Ephemeris();
    eph_gps.PRN = 1;
    eph_gal.PRN = 1;
    Pvt_Conf conf;
    conf.use_e6_for_pvt = false;
    const auto signal_enabled_flags = GPS_1C | GAL_1B | GAL_E5a;
    auto pvt_solution = std::make_shared<Rtklib_Solver>(rtk, conf, "filename", signal_enabled_flags, false, false);
    pvt_solution->gps_ephemeris_map[1] = std::move(eph_gps);
    pvt_solution->galileo_ephemeris_map[1] = std::move(eph_gal);
    std::map<int, Gnss_Synchro> gnss_observables_map;

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

    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(1, gs1));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(2, gs2));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(3, gs3));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(4, gs4));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(5, gs5));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(6, gs6));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(7, gs7));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(8, gs8));

    auto rp = std::make_shared<Rinex_Printer>(signal_enabled_flags);

    rp->print_rinex_annotation(pvt_solution.get(), gnss_observables_map, 0.0, true);

    std::string obsfile = rp->get_obsfilename();
    std::string navfile = rp->get_navfilename()[0];

    rp = nullptr;  // close the RINEX files so we can inspect them

    std::string line_epoch;
    std::string line_sat;
    find_obs_record_lines(obsfile, "E16", line_epoch, line_sat);

    std::string expected_epoch = "> 2019 04 14 00 00 00.0000000  0  7                                             ";
    std::string expected_sat("E16  22000000.000 7         0.127 7       -20.000 7        42.000  22000000.000 6         8.292 6      1534.000 6        41.000");
    EXPECT_EQ(0, expected_epoch.compare(line_epoch));
    EXPECT_EQ(0, expected_sat.compare(line_sat));

    fs::remove(navfile);
    fs::remove(obsfile);
}


TEST_F(RinexPrinterTest, MixedObsLogGpsGlo)
{
    auto eph_gps = Gps_Ephemeris();
    auto eph_glo = Glonass_Gnav_Ephemeris();
    eph_gps.PRN = 1;
    eph_glo.PRN = 1;
    Pvt_Conf conf;
    conf.use_e6_for_pvt = false;
    const auto signal_enabled_flags = GPS_1C | GLO_1G | GLO_2G;
    auto pvt_solution = std::make_shared<Rtklib_Solver>(rtk, conf, "filename", signal_enabled_flags, false, false);
    pvt_solution->gps_ephemeris_map[1] = std::move(eph_gps);
    pvt_solution->glonass_gnav_ephemeris_map[1] = std::move(eph_glo);
    std::map<int, Gnss_Synchro> gnss_observables_map;

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
    std::memcpy(reinterpret_cast<void*>(gs1.Signal), sig.c_str(), 3);
    std::memcpy(reinterpret_cast<void*>(gs2.Signal), sig.c_str(), 3);
    std::memcpy(reinterpret_cast<void*>(gs3.Signal), sig.c_str(), 3);
    std::memcpy(reinterpret_cast<void*>(gs4.Signal), sig.c_str(), 3);

    sig = "1G";
    std::memcpy(reinterpret_cast<void*>(gs5.Signal), sig.c_str(), 3);
    std::memcpy(reinterpret_cast<void*>(gs6.Signal), sig.c_str(), 3);
    std::memcpy(reinterpret_cast<void*>(gs7.Signal), sig.c_str(), 3);

    sig = "2G";
    std::memcpy(reinterpret_cast<void*>(gs8.Signal), sig.c_str(), 3);

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

    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(1, gs1));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(2, gs2));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(3, gs3));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(4, gs4));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(5, gs5));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(6, gs6));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(7, gs7));
    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(8, gs8));

    auto rp = std::make_shared<Rinex_Printer>(signal_enabled_flags);

    rp->print_rinex_annotation(pvt_solution.get(), gnss_observables_map, 0.0, true);

    std::string obsfile = rp->get_obsfilename();
    std::string navfile = rp->get_navfilename()[0];

    rp = nullptr;  // close the RINEX files so we can inspect them

    std::string line_epoch;
    std::string line_sat;
    find_obs_record_lines(obsfile, "R16", line_epoch, line_sat);

    std::string expected_epoch = "> 2019 04 14 00 00 00.0000000  0  7                                             ";
    std::string expected_sat("R16  22000000.000 6         8.292 6      1534.000 6        41.000  22000000.000 7         0.127 7       -20.000 7        42.000");
    EXPECT_EQ(0, expected_epoch.compare(line_epoch));
    EXPECT_EQ(0, expected_sat.compare(line_sat));

    fs::remove(navfile);
    fs::remove(obsfile);
}
