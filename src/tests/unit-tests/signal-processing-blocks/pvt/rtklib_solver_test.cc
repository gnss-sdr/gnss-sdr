/*!
 * \file rtklib_solver_test.cc
 * \brief Implements Unit Test for the rtklib PVT solver class.
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
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

#include <gtest/gtest.h>
#include <string>
#include <iomanip>
#include <iostream>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include "rtklib_solver.h"
#include "in_memory_configuration.h"
#include "gnss_sdr_supl_client.h"
#include "geofunctions.h"
#include <armadillo>


rtk_t configure_rtklib_options()
{
    std::shared_ptr<InMemoryConfiguration> configuration;
    configuration = std::make_shared<InMemoryConfiguration>();
    std::string role = "rtklib_solver";
    // custom options
    configuration->set_property("rtklib_solver.positioning_mode", "Single");
    configuration->set_property("rtklib_solver.elevation_mask", "0");
    configuration->set_property("rtklib_solver.iono_model", "OFF");
    configuration->set_property("rtklib_solver.trop_model", "OFF");
    //RTKLIB PVT solver options

    // Settings 1
    int positioning_mode = -1;
    std::string default_pos_mode("Single");
    std::string positioning_mode_str = configuration->property(role + ".positioning_mode", default_pos_mode); /* (PMODE_XXX) see src/algorithms/libs/rtklib/rtklib.h */
    if (positioning_mode_str.compare("Single") == 0) positioning_mode = PMODE_SINGLE;
    if (positioning_mode_str.compare("Static") == 0) positioning_mode = PMODE_STATIC;
    if (positioning_mode_str.compare("Kinematic") == 0) positioning_mode = PMODE_KINEMA;
    if (positioning_mode_str.compare("PPP_Static") == 0) positioning_mode = PMODE_PPP_STATIC;
    if (positioning_mode_str.compare("PPP_Kinematic") == 0) positioning_mode = PMODE_PPP_KINEMA;

    if (positioning_mode == -1)
        {
            //warn user and set the default
            std::cout << "WARNING: Bad specification of positioning mode." << std::endl;
            std::cout << "positioning_mode possible values: Single / Static / Kinematic / PPP_Static / PPP_Kinematic" << std::endl;
            std::cout << "positioning_mode specified value: " << positioning_mode_str << std::endl;
            std::cout << "Setting positioning_mode to Single" << std::endl;
            positioning_mode = PMODE_SINGLE;
        }

    int num_bands = 1;

    //    if ((gps_1C_count > 0) || (gal_1B_count > 0) || (glo_1G_count > 0)) num_bands = 1;
    //    if (((gps_1C_count > 0) || (gal_1B_count > 0) || (glo_1G_count > 0)) && ((gps_2S_count > 0) || (glo_2G_count > 0))) num_bands = 2;
    //    if (((gps_1C_count > 0) || (gal_1B_count > 0) || (glo_1G_count > 0)) && ((gal_E5a_count > 0) || (gal_E5b_count > 0) || (gps_L5_count > 0))) num_bands = 2;
    //    if (((gps_1C_count > 0) || (gal_1B_count > 0) || (glo_1G_count > 0)) && ((gps_2S_count > 0) || (glo_2G_count > 0)) && ((gal_E5a_count > 0) || (gal_E5b_count > 0) || (gps_L5_count > 0))) num_bands = 3;

    int number_of_frequencies = configuration->property(role + ".num_bands", num_bands); /* (1:L1, 2:L1+L2, 3:L1+L2+L5) */
    if ((number_of_frequencies < 1) || (number_of_frequencies > 3))
        {
            //warn user and set the default
            number_of_frequencies = num_bands;
        }

    double elevation_mask = configuration->property(role + ".elevation_mask", 15.0);
    if ((elevation_mask < 0.0) || (elevation_mask > 90.0))
        {
            //warn user and set the default
            LOG(WARNING) << "Erroneous Elevation Mask. Setting to default value of 15.0 degrees";
            elevation_mask = 15.0;
        }

    int dynamics_model = configuration->property(role + ".dynamics_model", 0); /*  dynamics model (0:none, 1:velocity, 2:accel) */
    if ((dynamics_model < 0) || (dynamics_model > 2))
        {
            //warn user and set the default
            LOG(WARNING) << "Erroneous Dynamics Model configuration. Setting to default value of (0:none)";
            dynamics_model = 0;
        }

    std::string default_iono_model("OFF");
    std::string iono_model_str = configuration->property(role + ".iono_model", default_iono_model); /*  (IONOOPT_XXX) see src/algorithms/libs/rtklib/rtklib.h */
    int iono_model = -1;
    if (iono_model_str.compare("OFF") == 0) iono_model = IONOOPT_OFF;
    if (iono_model_str.compare("Broadcast") == 0) iono_model = IONOOPT_BRDC;
    if (iono_model_str.compare("SBAS") == 0) iono_model = IONOOPT_SBAS;
    if (iono_model_str.compare("Iono-Free-LC") == 0) iono_model = IONOOPT_IFLC;
    if (iono_model_str.compare("Estimate_STEC") == 0) iono_model = IONOOPT_EST;
    if (iono_model_str.compare("IONEX") == 0) iono_model = IONOOPT_TEC;
    if (iono_model == -1)
        {
            //warn user and set the default
            std::cout << "WARNING: Bad specification of ionospheric model." << std::endl;
            std::cout << "iono_model possible values: OFF / Broadcast / SBAS / Iono-Free-LC / Estimate_STEC / IONEX" << std::endl;
            std::cout << "iono_model specified value: " << iono_model_str << std::endl;
            std::cout << "Setting iono_model to OFF" << std::endl;
            iono_model = IONOOPT_OFF; /* 0: ionosphere option: correction off */
        }

    std::string default_trop_model("OFF");
    int trop_model = -1;
    std::string trop_model_str = configuration->property(role + ".trop_model", default_trop_model); /*  (TROPOPT_XXX) see src/algorithms/libs/rtklib/rtklib.h */
    if (trop_model_str.compare("OFF") == 0) trop_model = TROPOPT_OFF;
    if (trop_model_str.compare("Saastamoinen") == 0) trop_model = TROPOPT_SAAS;
    if (trop_model_str.compare("SBAS") == 0) trop_model = TROPOPT_SBAS;
    if (trop_model_str.compare("Estimate_ZTD") == 0) trop_model = TROPOPT_EST;
    if (trop_model_str.compare("Estimate_ZTD_Grad") == 0) trop_model = TROPOPT_ESTG;
    if (trop_model == -1)
        {
            //warn user and set the default
            std::cout << "WARNING: Bad specification of tropospheric model." << std::endl;
            std::cout << "trop_model possible values: OFF / Saastamoinen / SBAS / Estimate_ZTD / Estimate_ZTD_Grad" << std::endl;
            std::cout << "trop_model specified value: " << trop_model_str << std::endl;
            std::cout << "Setting trop_model to OFF" << std::endl;
            trop_model = TROPOPT_OFF;
        }

    /* RTKLIB positioning options */
    int sat_PCV = 0; /*  Set whether the satellite antenna PCV (phase center variation) model is used or not. This feature requires a Satellite Antenna PCV File. */
    int rec_PCV = 0; /*  Set whether the receiver antenna PCV (phase center variation) model is used or not. This feature requires a Receiver Antenna PCV File. */

    /* Set whether the phase windup correction for PPP modes is applied or not. Only applicable to PPP‐* modes.*/
    int phwindup = configuration->property(role + ".phwindup", 0);

    /* Set whether the GPS Block IIA satellites in eclipse are excluded or not.
    The eclipsing Block IIA satellites often degrade the PPP solutions due to unpredicted behavior of yaw‐attitude. Only applicable to PPP‐* modes.*/
    int reject_GPS_IIA = configuration->property(role + ".reject_GPS_IIA", 0);

    /* Set whether RAIM (receiver autonomous integrity monitoring) FDE (fault detection and exclusion) feature is enabled or not.
    In case of RAIM FDE enabled, a satellite is excluded if SSE (sum of squared errors) of residuals is over a threshold.
    The excluded satellite is selected to indicate the minimum SSE. */
    int raim_fde = configuration->property(role + ".raim_fde", 0);

    int earth_tide = configuration->property(role + ".earth_tide", 0);

    int nsys = SYS_GPS;
    //    if ((gps_1C_count > 0) || (gps_2S_count > 0) || (gps_L5_count > 0)) nsys += SYS_GPS;
    //    if ((gal_1B_count > 0) || (gal_E5a_count > 0) || (gal_E5b_count > 0)) nsys += SYS_GAL;
    //    if ((glo_1G_count > 0) || (glo_2G_count > 0)) nsys += SYS_GLO;
    int navigation_system = configuration->property(role + ".navigation_system", nsys); /* (SYS_XXX) see src/algorithms/libs/rtklib/rtklib.h */
    if ((navigation_system < 1) || (navigation_system > 255))                           /* GPS: 1   SBAS: 2   GPS+SBAS: 3 Galileo: 8  Galileo+GPS: 9 GPS+SBAS+Galileo: 11 All: 255 */
        {
            //warn user and set the default
            LOG(WARNING) << "Erroneous Navigation System. Setting to default value of (0:none)";
            navigation_system = nsys;
        }

    // Settings 2
    std::string default_gps_ar("Continuous");
    std::string integer_ambiguity_resolution_gps_str = configuration->property(role + ".AR_GPS", default_gps_ar); /* Integer Ambiguity Resolution mode for GPS (0:off,1:continuous,2:instantaneous,3:fix and hold,4:ppp-ar) */
    int integer_ambiguity_resolution_gps = -1;
    if (integer_ambiguity_resolution_gps_str.compare("OFF") == 0) integer_ambiguity_resolution_gps = ARMODE_OFF;
    if (integer_ambiguity_resolution_gps_str.compare("Continuous") == 0) integer_ambiguity_resolution_gps = ARMODE_CONT;
    if (integer_ambiguity_resolution_gps_str.compare("Instantaneous") == 0) integer_ambiguity_resolution_gps = ARMODE_INST;
    if (integer_ambiguity_resolution_gps_str.compare("Fix-and-Hold") == 0) integer_ambiguity_resolution_gps = ARMODE_FIXHOLD;
    if (integer_ambiguity_resolution_gps_str.compare("PPP-AR") == 0) integer_ambiguity_resolution_gps = ARMODE_PPPAR;
    if (integer_ambiguity_resolution_gps == -1)
        {
            //warn user and set the default
            std::cout << "WARNING: Bad specification of GPS ambiguity resolution method." << std::endl;
            std::cout << "AR_GPS possible values: OFF / Continuous / Instantaneous / Fix-and-Hold / PPP-AR" << std::endl;
            std::cout << "AR_GPS specified value: " << integer_ambiguity_resolution_gps_str << std::endl;
            std::cout << "Setting AR_GPS to OFF" << std::endl;
            integer_ambiguity_resolution_gps = ARMODE_OFF;
        }

    int integer_ambiguity_resolution_glo = configuration->property(role + ".AR_GLO", 1); /* Integer Ambiguity Resolution mode for GLONASS (0:off,1:on,2:auto cal,3:ext cal) */
    if ((integer_ambiguity_resolution_glo < 0) || (integer_ambiguity_resolution_glo > 3))
        {
            //warn user and set the default
            LOG(WARNING) << "Erroneous Integer Ambiguity Resolution for GLONASS . Setting to default value of (1:on)";
            integer_ambiguity_resolution_glo = 1;
        }

    int integer_ambiguity_resolution_bds = configuration->property(role + ".AR_DBS", 1); /* Integer Ambiguity Resolution mode for BEIDOU (0:off,1:on) */
    if ((integer_ambiguity_resolution_bds < 0) || (integer_ambiguity_resolution_bds > 1))
        {
            //warn user and set the default
            LOG(WARNING) << "Erroneous Integer Ambiguity Resolution for BEIDOU . Setting to default value of (1:on)";
            integer_ambiguity_resolution_bds = 1;
        }

    double min_ratio_to_fix_ambiguity = configuration->property(role + ".min_ratio_to_fix_ambiguity", 3.0); /* Set the integer ambiguity validation threshold for ratio‐test,
                                                                                                               which uses the ratio of squared residuals of the best integer vector to the second‐best vector. */

    int min_lock_to_fix_ambiguity = configuration->property(role + ".min_lock_to_fix_ambiguity", 0); /* Set the minimum lock count to fix integer ambiguity.
                                                                                                         If the lock count is less than the value, the ambiguity is excluded from the fixed integer vector. */

    double min_elevation_to_fix_ambiguity = configuration->property(role + ".min_elevation_to_fix_ambiguity", 0.0); /* Set the minimum elevation (deg) to fix integer ambiguity.
                                                                                                                        If the elevation of the satellite is less than the value, the ambiguity is excluded from the fixed integer vector. */

    int outage_reset_ambiguity = configuration->property(role + ".outage_reset_ambiguity", 5); /* Set the outage count to reset ambiguity. If the data outage count is over the value, the estimated ambiguity is reset to the initial value.  */

    double slip_threshold = configuration->property(role + ".slip_threshold", 0.05); /* set the cycle‐slip threshold (m) of geometry‐free LC carrier‐phase difference between epochs */

    double threshold_reject_gdop = configuration->property(role + ".threshold_reject_gdop", 30.0); /* reject threshold of GDOP. If the GDOP is over the value, the observable is excluded for the estimation process as an outlier. */

    double threshold_reject_innovation = configuration->property(role + ".threshold_reject_innovation", 30.0); /* reject threshold of innovation (m). If the innovation is over the value, the observable is excluded for the estimation process as an outlier. */

    int number_filter_iter = configuration->property(role + ".number_filter_iter", 1); /* Set the number of iteration in the measurement update of the estimation filter.
                                                                                         If the baseline length is very short like 1 m, the iteration may be effective to handle
                                                                                         the nonlinearity of measurement equation. */

    /// Statistics
    double bias_0 = configuration->property(role + ".bias_0", 30.0);

    double iono_0 = configuration->property(role + ".iono_0", 0.03);

    double trop_0 = configuration->property(role + ".trop_0", 0.3);

    double sigma_bias = configuration->property(role + ".sigma_bias", 1e-4); /* Set the process noise standard deviation of carrier‐phase
                                                                                bias (ambiguity) (cycle/sqrt(s)) */

    double sigma_iono = configuration->property(role + ".sigma_iono", 1e-3); /* Set the process noise standard deviation of vertical ionospheric delay per 10 km baseline (m/sqrt(s)). */

    double sigma_trop = configuration->property(role + ".sigma_trop", 1e-4); /* Set the process noise standard deviation of zenith tropospheric delay (m/sqrt(s)). */

    double sigma_acch = configuration->property(role + ".sigma_acch", 1e-1); /* Set the process noise standard deviation of the receiver acceleration as
                                                                                the horizontal component. (m/s2/sqrt(s)). If Receiver Dynamics is set to OFF, they are not used. */

    double sigma_accv = configuration->property(role + ".sigma_accv", 1e-2); /* Set the process noise standard deviation of the receiver acceleration as
                                                                                the vertical component. (m/s2/sqrt(s)). If Receiver Dynamics is set to OFF, they are not used. */

    double sigma_pos = configuration->property(role + ".sigma_pos", 0.0);

    double code_phase_error_ratio_l1 = configuration->property(role + ".code_phase_error_ratio_l1", 100.0);
    double code_phase_error_ratio_l2 = configuration->property(role + ".code_phase_error_ratio_l2", 100.0);
    double code_phase_error_ratio_l5 = configuration->property(role + ".code_phase_error_ratio_l5", 100.0);
    double carrier_phase_error_factor_a = configuration->property(role + ".carrier_phase_error_factor_a", 0.003);
    double carrier_phase_error_factor_b = configuration->property(role + ".carrier_phase_error_factor_b", 0.003);

    snrmask_t snrmask = {{}, {{}, {}}};

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
        {sat_PCV, rec_PCV, phwindup, reject_GPS_IIA, raim_fde},                            /* posopt[6] positioning options [0]: satellite and receiver antenna PCV model; [1]: interpolate antenna parameters; [2]: apply phase wind-up correction for PPP modes; [3]: exclude measurements of GPS Block IIA satellites satellite [4]: RAIM FDE (fault detection and exclusion) [5]: handle day-boundary clock jump */
        0,                                                                                 /* solution sync mode (0:off,1:on) */
        {{}, {}},                                                                          /* odisp[2][6*11] ocean tide loading parameters {rov,base} */
        {{}, {{}, {}}, {{}, {}}, {}, {}},                                                  /* exterr_t exterr   extended receiver error model */
        0,                                                                                 /* disable L2-AR */
        {}                                                                                 /* char pppopt[256]   ppp option   "-GAP_RESION="  default gap to reset iono parameters (ep) */
    };

    rtk_t rtk;
    rtkinit(&rtk, &rtklib_configuration_options);
    return rtk;
}


//todo: add test cases for Galileo E1, E5 and GPS L5
TEST(RTKLibSolverTest, test1)
{
    //test case #1: GPS L1 CA simulated with gnss-sim
    std::string path = std::string(TEST_PATH);
    int nchannels = 8;
    std::string dump_filename = ".rtklib_solver_dump.dat";
    bool flag_dump_to_file = false;
    rtk_t rtk = configure_rtklib_options();

    std::unique_ptr<rtklib_solver> d_ls_pvt(new rtklib_solver(nchannels, dump_filename, flag_dump_to_file, rtk));
    d_ls_pvt->set_averaging_depth(1);

    // load ephemeris
    std::string eph_xml_filename = path + "data/rtklib_test/eph_GPS_L1CA_test1.xml";
    gnss_sdr_supl_client supl_client_ephemeris_;

    std::cout << "SUPL: Try read GPS ephemeris from XML file " << eph_xml_filename << std::endl;
    if (supl_client_ephemeris_.load_ephemeris_xml(eph_xml_filename) == true)
        {
            std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter;
            for (gps_eph_iter = supl_client_ephemeris_.gps_ephemeris_map.cbegin();
                 gps_eph_iter != supl_client_ephemeris_.gps_ephemeris_map.cend();
                 gps_eph_iter++)
                {
                    std::cout << "SUPL: Read XML Ephemeris for GPS SV " << gps_eph_iter->first << std::endl;
                    std::shared_ptr<Gps_Ephemeris> tmp_obj = std::make_shared<Gps_Ephemeris>(gps_eph_iter->second);
                    // update/insert new ephemeris record to the global ephemeris map
                    d_ls_pvt->gps_ephemeris_map[gps_eph_iter->first] = *tmp_obj;
                }
        }
    else
        {
            std::cout << "ERROR: SUPL client error reading XML" << std::endl;
        }

    // insert observables epoch
    std::map<int, Gnss_Synchro> gnss_synchro_map;
    //    Gnss_Synchro tmp_obs;
    //    tmp_obs.System = 'G';
    //    std::string signal = "1C";
    //    const char* str = signal.c_str();                         // get a C style null terminated string
    //    std::memcpy(static_cast<void*>(tmp_obs.Signal), str, 3);  // copy string into synchro char array: 2 char + null
    //
    //    gnss_synchro_map[0] = tmp_obs;
    //    gnss_synchro_map[0].PRN = 1;

    //load from xml (boost serialize)
    std::string file_name = path + "data/rtklib_test/obs_test1.xml";

    std::ifstream ifs;
    try
        {
            ifs.open(file_name.c_str(), std::ifstream::binary | std::ifstream::in);
            boost::archive::xml_iarchive xml(ifs);
            gnss_synchro_map.clear();
            xml >> boost::serialization::make_nvp("GNSS-SDR_gnss_synchro_map", gnss_synchro_map);
            std::cout << "Loaded gnss_synchro map data with " << gnss_synchro_map.size() << " pseudoranges" << std::endl;
        }
    catch (std::exception& e)
        {
            std::cout << e.what() << "File: " << file_name;
        }
    ifs.close();

    // solve
    bool pvt_valid = false;
    if (d_ls_pvt->get_PVT(gnss_synchro_map, false))
        {
            // DEBUG MESSAGE: Display position in console output
            if (d_ls_pvt->is_valid_position())
                {
                    std::streamsize ss = std::cout.precision();  // save current precision
                    std::cout.setf(std::ios::fixed, std::ios::floatfield);

                    auto facet = new boost::posix_time::time_facet("%Y-%b-%d %H:%M:%S.%f %z");
                    std::cout.imbue(std::locale(std::cout.getloc(), facet));

                    std::cout << "Position at " << d_ls_pvt->get_position_UTC_time()
                              << " UTC using " << d_ls_pvt->get_num_valid_observations()
                              << std::fixed << std::setprecision(9)
                              << " observations is Lat = " << d_ls_pvt->get_latitude() << " [deg], Long = " << d_ls_pvt->get_longitude()
                              << std::fixed << std::setprecision(3)
                              << " [deg], Height = " << d_ls_pvt->get_height() << " [m]" << std::endl;
                    std::cout << std::setprecision(ss);
                    std::cout << "RX clock offset: " << d_ls_pvt->get_time_offset_s() << "[s]" << std::endl;

                    // boost::posix_time::ptime p_time;
                    // gtime_t rtklib_utc_time = gpst2time(adjgpsweek(d_ls_pvt->gps_ephemeris_map.cbegin()->second.i_GPS_week), d_rx_time);
                    // p_time = boost::posix_time::from_time_t(rtklib_utc_time.time);
                    // p_time += boost::posix_time::microseconds(round(rtklib_utc_time.sec * 1e6));
                    // std::cout << TEXT_MAGENTA << "Observable RX time (GPST) " << boost::posix_time::to_simple_string(p_time) << TEXT_RESET << std::endl;

                    std::cout << "RTKLIB Position at RX TOW = " << gnss_synchro_map.begin()->second.RX_time
                              << " in ECEF (X,Y,Z,t[meters]) = " << std::fixed << std::setprecision(16)
                              << d_ls_pvt->pvt_sol.rr[0] << ","
                              << d_ls_pvt->pvt_sol.rr[1] << ","
                              << d_ls_pvt->pvt_sol.rr[2] << std::endl;
                    /* std::cout << "Dilution of Precision at " << boost::posix_time::to_simple_string(d_ls_pvt->get_position_UTC_time())
                             << " UTC using "<< d_ls_pvt->get_num_valid_observations() <<" observations is HDOP = " << d_ls_pvt->get_hdop() << " VDOP = "
                             << d_ls_pvt->get_vdop()
                             << " GDOP = " << d_ls_pvt->get_gdop() << std::endl; */

                    //todo: check here the positioning error against the reference position generated with gnss-sim
                    //reference position on in WGS84: Lat (deg), Long (deg) , H (m): 30.286502,120.032669,100
                    arma::vec LLH = {30.286502, 120.032669, 100};  //ref position for this scenario

                    double error_LLH_m = great_circle_distance(LLH(0), LLH(1), d_ls_pvt->get_latitude(), d_ls_pvt->get_longitude());
                    std::cout << "Lat, Long, H error: " << d_ls_pvt->get_latitude() - LLH(0)
                              << "," << d_ls_pvt->get_longitude() - LLH(1)
                              << "," << d_ls_pvt->get_height() - LLH(2) << " [deg,deg,meters]" << std::endl;

                    std::cout << "Haversine Great Circle error LLH distance: " << error_LLH_m << " [meters]" << std::endl;

                    arma::vec v_eb_n = {0.0, 0.0, 0.0};
                    arma::vec true_r_eb_e;
                    arma::vec true_v_eb_e;
                    pv_Geo_to_ECEF(degtorad(LLH(0)), degtorad(LLH(1)), LLH(2), v_eb_n, true_r_eb_e, true_v_eb_e);

                    arma::vec measured_r_eb_e = {d_ls_pvt->pvt_sol.rr[0], d_ls_pvt->pvt_sol.rr[1], d_ls_pvt->pvt_sol.rr[2]};

                    arma::vec error_r_eb_e = measured_r_eb_e - true_r_eb_e;

                    std::cout << "ECEF position error vector: " << error_r_eb_e << " [meters]" << std::endl;

                    double error_3d_m = arma::norm(error_r_eb_e, 2);

                    std::cout << "3D positioning error: " << error_3d_m << " [meters]" << std::endl;

                    //check results against the test tolerance
                    ASSERT_LT(error_3d_m, 1.0);
                    pvt_valid = true;
                }
        }

    EXPECT_EQ(true, pvt_valid);
}
