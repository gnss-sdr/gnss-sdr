/*!
 * \file rinex_printer_test.cc
 * \brief Implements Unit Tests for the Rinex_Printer class.
 * \author Carles Fernandez-Prades, 2016. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "rinex_printer.h"
#include "rtklib_rtkpos.h"
#include "rtklib_solver.h"
#include <fstream>
#include <string>

#if HAS_STD_FILESYSTEM
#include <system_error>
namespace errorlib = std;
#if HAS_STD_FILESYSTEM_EXPERIMENTAL
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif
#else
#include <boost/filesystem/operations.hpp>   // for create_directories, exists
#include <boost/filesystem/path.hpp>         // for path, operator<<
#include <boost/filesystem/path_traits.hpp>  // for filesystem
#include <boost/system/error_code.hpp>       // for error_code
namespace fs = boost::filesystem;
namespace errorlib = boost::system;
#endif

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
        {}                                                                                 /* char pppopt[256]   ppp option   "-GAP_RESION="  default gap to reset iono parameters (ep) */
    };

    rtkinit(&rtk, &rtklib_configuration_options);
}


TEST_F(RinexPrinterTest, GalileoObsHeader)
{
    auto pvt_solution = std::make_shared<Rtklib_Solver>(rtk, 12, "filename", false, false);
    auto eph = Galileo_Ephemeris();
    eph.i_satellite_PRN = 1;
    pvt_solution->galileo_ephemeris_map[1] = eph;

    std::map<int, Gnss_Synchro> gnss_observables_map;
    Gnss_Synchro gs{};
    gs.PRN = 1;
    gnss_observables_map[1] = gs;

    auto rp = std::make_shared<Rinex_Printer>();

    rp->print_rinex_annotation(pvt_solution.get(),
        gnss_observables_map,
        0.0,
        4,
        true);

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

    auto rp2 = std::make_shared<Rinex_Printer>();

    rp2->print_rinex_annotation(pvt_solution.get(),
        gnss_observables_map,
        0.0,
        15,
        true);
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
    auto pvt_solution = std::make_shared<Rtklib_Solver>(rtk, 12, "filename", false, false);
    auto eph = Glonass_Gnav_Ephemeris();
    eph.i_satellite_PRN = 1;
    pvt_solution->glonass_gnav_ephemeris_map[1] = eph;

    std::map<int, Gnss_Synchro> gnss_observables_map;
    Gnss_Synchro gs{};
    gs.PRN = 1;
    gnss_observables_map[1] = gs;

    auto rp = std::make_shared<Rinex_Printer>(3);

    rp->print_rinex_annotation(pvt_solution.get(),
        gnss_observables_map,
        0.0,
        23,
        true);

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
    std::string expected_str("R    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES ");
    EXPECT_EQ(0, expected_str.compare(line_aux));
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
    eph_gal.i_satellite_PRN = 1;
    eph_gps.i_satellite_PRN = 1;
    auto pvt_solution = std::make_shared<Rtklib_Solver>(rtk, 12, "filename", false, false);
    pvt_solution->galileo_ephemeris_map[1] = eph_gal;

    pvt_solution->gps_ephemeris_map[1] = eph_gps;

    std::map<int, Gnss_Synchro> gnss_observables_map;
    Gnss_Synchro gs{};
    gs.PRN = 1;
    gnss_observables_map[1] = gs;
    gnss_observables_map[2] = gs;

    auto rp = std::make_shared<Rinex_Printer>();

    rp->print_rinex_annotation(pvt_solution.get(),
        gnss_observables_map,
        0.0,
        33,
        true);

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
    eph_glo.i_satellite_PRN = 1;
    eph_gps.i_satellite_PRN = 1;
    auto pvt_solution = std::make_shared<Rtklib_Solver>(rtk, 12, "filename", false, false);
    pvt_solution->glonass_gnav_ephemeris_map[1] = eph_glo;

    pvt_solution->gps_ephemeris_map[1] = eph_gps;

    std::map<int, Gnss_Synchro> gnss_observables_map;
    Gnss_Synchro gs{};
    gs.PRN = 1;
    gnss_observables_map[1] = gs;
    gnss_observables_map[2] = gs;

    auto rp = std::make_shared<Rinex_Printer>();

    rp->print_rinex_annotation(pvt_solution.get(),
        gnss_observables_map,
        0.0,
        26,
        true);

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
