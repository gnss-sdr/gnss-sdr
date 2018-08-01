/*!
 * \file obs_system_test.cc
 * \brief  This class implements a test for the validation of generated observables.
 * \author Carles Fernandez-Prades, 2016. cfernandez(at)cttc.es
 *         Antonio Ramos, 2017. antonio.ramos(at)cttc.es
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

#include "gnuplot_i.h"
#include "test_flags.h"
#include "concurrent_map.h"
#include "concurrent_queue.h"
#include "control_thread.h"
#include "file_configuration.h"
#include <armadillo>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <gpstk/RinexUtilities.hpp>
#include <gpstk/Rinex3ObsBase.hpp>
#include <gpstk/Rinex3ObsData.hpp>
#include <gpstk/Rinex3ObsHeader.hpp>
#include <gpstk/Rinex3ObsStream.hpp>
#include <algorithm>
#include <chrono>
#include <boost/filesystem.hpp>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <unistd.h>


// For GPS NAVIGATION (L1)
concurrent_queue<Gps_Acq_Assist> global_gps_acq_assist_queue;
concurrent_map<Gps_Acq_Assist> global_gps_acq_assist_map;

DEFINE_string(configuration_file, "./default_configuration.conf", "Path of configuration file");
DEFINE_string(filename_rinex_true, "./default_rinex.txt", "Path of RINEX true observations");
DEFINE_string(filename_rinex_obs, "default_string", "Path of RINEX true observations");
DEFINE_double(pr_error_mean_max, 25.0, "Maximum mean error in pseudorange");
DEFINE_double(pr_error_std_max, 15.0, "Maximum standard deviation in pseudorange");
DEFINE_double(cp_error_mean_max, 5.0, "Maximum mean error in carrier phase");
DEFINE_double(cp_error_std_max, 2.5, "Maximum standard deviation in carrier phase");
DEFINE_double(dp_error_mean_max, 75.0, "Maximum mean error in Doppler frequency");
DEFINE_double(dp_error_std_max, 25.0, "Maximum standard deviation in Doppler frequency");
DEFINE_bool(plot_obs_sys_test, false, "Plots results of ObsSystemTest with gnuplot");

class ObsSystemTest : public ::testing::Test
{
public:
    int configure_receiver();
    int run_receiver();
    void check_results();
    bool check_valid_rinex_obs(std::string filename, int rinex_ver);  // return true if the file is a valid Rinex observation file.
    void read_rinex_files(
        std::vector<arma::mat>& pseudorange_ref,
        std::vector<arma::mat>& carrierphase_ref,
        std::vector<arma::mat>& doppler_ref,
        std::vector<arma::mat>& pseudorange_meas,
        std::vector<arma::mat>& carrierphase_meas,
        std::vector<arma::mat>& doppler_meas,
        arma::mat& sow_prn_ref,
        int signal_type);
    void time_alignment_diff(
        std::vector<arma::mat>& ref,
        std::vector<arma::mat>& meas,
        std::vector<arma::vec>& diff);
    void time_alignment_diff_cp(
        std::vector<arma::mat>& ref,
        std::vector<arma::mat>& meas,
        std::vector<arma::vec>& diff);
    void time_alignment_diff_pr(
        std::vector<arma::mat>& ref,
        std::vector<arma::mat>& meas,
        std::vector<arma::vec>& diff,
        arma::mat& sow_prn_ref);
    void compute_pseudorange_error(std::vector<arma::vec>& diff,
        double error_th_mean, double error_th_std,
        std::string signal_name);
    void compute_carrierphase_error(
        std::vector<arma::vec>& diff,
        double error_th_mean, double error_th_std,
        std::string signal_name);
    void compute_doppler_error(
        std::vector<arma::vec>& diff,
        double error_th_mean, double error_th_std,
        std::string signal_name);
    std::string filename_rinex_obs = FLAGS_filename_rinex_true;
    std::string generated_rinex_obs = FLAGS_filename_rinex_obs;
    std::string configuration_file_ = FLAGS_configuration_file;
    std::shared_ptr<FileConfiguration> config;
    bool gps_1C = false;
    bool gps_L5 = false;
    bool gal_1B = false;
    bool gal_E5a = false;
    bool internal_rinex_generation = false;

    /****************/
    const int num_prn_gps = 33;
    const int num_prn_gal = 31;

    double pseudorange_error_th_mean = FLAGS_pr_error_mean_max;
    double pseudorange_error_th_std = FLAGS_pr_error_std_max;
    double carrierphase_error_th_mean = FLAGS_cp_error_mean_max;
    double carrierphase_error_th_std = FLAGS_cp_error_std_max;
    double doppler_error_th_mean = FLAGS_dp_error_mean_max;
    double doppler_error_th_std = FLAGS_dp_error_std_max;
};


bool ObsSystemTest::check_valid_rinex_obs(std::string filename, int rinex_ver)
{
    bool res = false;
    if (rinex_ver == 2)
        {
            res = gpstk::isRinexObsFile(filename);
        }
    if (rinex_ver == 3)
        {
            res = gpstk::isRinex3ObsFile(filename);
        }
    return res;
}


void ObsSystemTest::read_rinex_files(
    std::vector<arma::mat>& pseudorange_ref,
    std::vector<arma::mat>& carrierphase_ref,
    std::vector<arma::mat>& doppler_ref,
    std::vector<arma::mat>& pseudorange_meas,
    std::vector<arma::mat>& carrierphase_meas,
    std::vector<arma::mat>& doppler_meas,
    arma::mat& sow_prn_ref,
    int signal_type)
{
    bool ref_exist = false;
    bool meas_exist = false;
    gpstk::SatID::SatelliteSystem sat_type = gpstk::SatID::systemUnknown;
    int max_prn = 0;
    std::string pr_string;
    std::string cp_string;
    std::string dp_string;
    std::string signal_type_string;
    sow_prn_ref.reset();

    switch (signal_type)
        {
        case 0:  //GPS L1

            sat_type = gpstk::SatID::systemGPS;
            max_prn = num_prn_gps;
            pr_string = "C1C";
            cp_string = "L1C";
            dp_string = "D1C";
            signal_type_string = "GPS L1 C/A";
            break;

        case 1:  //Galileo E1B

            sat_type = gpstk::SatID::systemGalileo;
            max_prn = num_prn_gal;
            pr_string = "C1B";
            cp_string = "L1B";
            dp_string = "D1B";
            signal_type_string = "Galileo E1B";
            break;

        case 2:  //GPS L5

            sat_type = gpstk::SatID::systemGPS;
            max_prn = num_prn_gps;
            pr_string = "C5X";
            cp_string = "L5X";
            dp_string = "D5X";
            signal_type_string = "GPS L5";
            break;

        case 3:  //Galileo E5a

            sat_type = gpstk::SatID::systemGalileo;
            max_prn = num_prn_gal;
            pr_string = "C5X";
            cp_string = "L5X";
            dp_string = "D5X";
            signal_type_string = "Galileo E5a";
            break;
        }

    // Open and read reference RINEX observables file
    std::cout << "Read: RINEX " << signal_type_string << " True" << std::endl;
    try
        {
            gpstk::Rinex3ObsStream r_ref(filename_rinex_obs);
            r_ref.exceptions(std::ios::failbit);
            gpstk::Rinex3ObsData r_ref_data;
            gpstk::Rinex3ObsHeader r_ref_header;
            gpstk::RinexDatum dataobj;
            r_ref >> r_ref_header;

            while (r_ref >> r_ref_data)
                {
                    for (int myprn = 1; myprn < max_prn; myprn++)
                        {
                            gpstk::SatID prn(myprn, sat_type);
                            gpstk::CommonTime time = r_ref_data.time;
                            double sow(static_cast<gpstk::GPSWeekSecond>(time).sow);
                            gpstk::Rinex3ObsData::DataMap::iterator pointer = r_ref_data.obs.find(prn);
                            if (pointer == r_ref_data.obs.end())
                                {
                                    // PRN not present; do nothing
                                }
                            else
                                {
                                    dataobj = r_ref_data.getObs(prn, pr_string, r_ref_header);
                                    double P1 = dataobj.data;
                                    pseudorange_ref.at(myprn).insert_rows(pseudorange_ref.at(myprn).n_rows, arma::rowvec({sow, P1}));

                                    dataobj = r_ref_data.getObs(prn, cp_string, r_ref_header);
                                    double L1 = dataobj.data;
                                    carrierphase_ref.at(myprn).insert_rows(carrierphase_ref.at(myprn).n_rows, arma::rowvec({sow, L1}));

                                    dataobj = r_ref_data.getObs(prn, dp_string, r_ref_header);
                                    double D1 = dataobj.data;
                                    doppler_ref.at(myprn).insert_rows(doppler_ref.at(myprn).n_rows, arma::rowvec({sow, D1}));

                                    ref_exist = true;
                                }  // End of 'if( pointer == roe.obs.end() )'
                        }          // end for
                }                  // end while
        }                          // End of 'try' block
    catch (const gpstk::FFStreamError& e)
        {
            std::cout << e;
            exit(1);
        }
    catch (const gpstk::Exception& e)
        {
            std::cout << e;
            exit(1);
        }
    catch (...)
        {
            std::cout << "unknown error.  I don't feel so well..." << std::endl;
            exit(1);
        }

    // Open and read measured RINEX observables file
    std::cout << "Read: RINEX " << signal_type_string << " measures" << std::endl;
    try
        {
            std::string arg2_gen;
            if (internal_rinex_generation)
                {
                    arg2_gen = std::string("./") + generated_rinex_obs;
                }
            else
                {
                    arg2_gen = generated_rinex_obs;
                }
            gpstk::Rinex3ObsStream r_meas(arg2_gen);
            r_meas.exceptions(std::ios::failbit);
            gpstk::Rinex3ObsData r_meas_data;
            gpstk::Rinex3ObsHeader r_meas_header;
            gpstk::RinexDatum dataobj;
            r_meas >> r_meas_header;

            while (r_meas >> r_meas_data)
                {
                    double pr_min = 0.0;
                    double sow_insert = 0.0;
                    double prn_min = 0.0;
                    bool set_pr_min = true;
                    for (int myprn = 1; myprn < max_prn; myprn++)
                        {
                            gpstk::SatID prn(myprn, sat_type);
                            gpstk::CommonTime time = r_meas_data.time;
                            double sow(static_cast<gpstk::GPSWeekSecond>(time).sow);
                            gpstk::Rinex3ObsData::DataMap::iterator pointer = r_meas_data.obs.find(prn);
                            if (pointer == r_meas_data.obs.end())
                                {
                                    // PRN not present; do nothing
                                }
                            else
                                {
                                    dataobj = r_meas_data.getObs(prn, pr_string, r_meas_header);
                                    double P1 = dataobj.data;
                                    pseudorange_meas.at(myprn).insert_rows(pseudorange_meas.at(myprn).n_rows, arma::rowvec({sow, P1}));
                                    if (set_pr_min || (P1 < pr_min))
                                        {
                                            set_pr_min = false;
                                            pr_min = P1;
                                            sow_insert = sow;
                                            prn_min = static_cast<double>(myprn);
                                        }

                                    dataobj = r_meas_data.getObs(prn, cp_string, r_meas_header);
                                    double L1 = dataobj.data;
                                    carrierphase_meas.at(myprn).insert_rows(carrierphase_meas.at(myprn).n_rows, arma::rowvec({sow, L1}));

                                    dataobj = r_meas_data.getObs(prn, dp_string, r_meas_header);
                                    double D1 = dataobj.data;
                                    doppler_meas.at(myprn).insert_rows(doppler_meas.at(myprn).n_rows, arma::rowvec({sow, D1}));

                                    meas_exist = true;
                                }  // End of 'if( pointer == roe.obs.end() )'
                        }          // end for
                    if (!set_pr_min)
                        {
                            sow_prn_ref.insert_rows(sow_prn_ref.n_rows, arma::rowvec({sow_insert, pr_min, prn_min}));
                        }
                }  // end while
        }          // End of 'try' block
    catch (const gpstk::FFStreamError& e)
        {
            std::cout << e;
            exit(1);
        }
    catch (const gpstk::Exception& e)
        {
            std::cout << e;
            exit(1);
        }
    catch (...)
        {
            std::cout << "unknown error.  I don't feel so well..." << std::endl;
            exit(1);
        }
    EXPECT_TRUE(ref_exist) << "RINEX reference file does not contain " << signal_type_string << " information";
    EXPECT_TRUE(meas_exist) << "RINEX generated file does not contain " << signal_type_string << " information";
}


void ObsSystemTest::time_alignment_diff(
    std::vector<arma::mat>& ref,
    std::vector<arma::mat>& meas,
    std::vector<arma::vec>& diff)
{
    std::vector<arma::mat>::iterator iter_ref;
    std::vector<arma::mat>::iterator iter_meas;
    std::vector<arma::vec>::iterator iter_diff;
    arma::mat mat_aux;

    iter_ref = ref.begin();
    iter_diff = diff.begin();
    for (iter_meas = meas.begin(); iter_meas != meas.end(); iter_meas++)
        {
            if (!iter_meas->is_empty() && !iter_ref->is_empty())
                {
                    arma::uvec index_ = arma::find(iter_meas->col(0) > iter_ref->at(0, 0));
                    arma::uword index_min = arma::min(index_);
                    index_ = arma::find(iter_meas->col(0) < iter_ref->at(iter_ref->n_rows - 1, 0));
                    arma::uword index_max = arma::max(index_);
                    mat_aux = iter_meas->rows(index_min, index_max);
                    arma::vec ref_aligned;
                    arma::interp1(iter_ref->col(0), iter_ref->col(1), mat_aux.col(0), ref_aligned);
                    *iter_diff = ref_aligned - mat_aux.col(1);
                }
            iter_ref++;
            iter_diff++;
        }
}


void ObsSystemTest::time_alignment_diff_cp(
    std::vector<arma::mat>& ref,
    std::vector<arma::mat>& meas,
    std::vector<arma::vec>& diff)
{
    std::vector<arma::mat>::iterator iter_ref;
    std::vector<arma::mat>::iterator iter_meas;
    std::vector<arma::vec>::iterator iter_diff;
    arma::mat mat_aux;

    iter_ref = ref.begin();
    iter_diff = diff.begin();
    for (iter_meas = meas.begin(); iter_meas != meas.end(); iter_meas++)
        {
            if (!iter_meas->is_empty() && !iter_ref->is_empty())
                {
                    arma::uvec index_ = arma::find(iter_meas->col(0) > iter_ref->at(0, 0));
                    arma::uword index_min = arma::min(index_);
                    index_ = arma::find(iter_meas->col(0) < iter_ref->at(iter_ref->n_rows - 1, 0));
                    arma::uword index_max = arma::max(index_);
                    mat_aux = iter_meas->rows(index_min, index_max);
                    mat_aux.col(1) -= mat_aux.col(1)(0);
                    arma::vec ref_aligned;
                    arma::interp1(iter_ref->col(0), iter_ref->col(1), mat_aux.col(0), ref_aligned);
                    ref_aligned -= ref_aligned(0);
                    *iter_diff = ref_aligned - mat_aux.col(1);
                }
            iter_ref++;
            iter_diff++;
        }
}


void ObsSystemTest::time_alignment_diff_pr(
    std::vector<arma::mat>& ref,
    std::vector<arma::mat>& meas,
    std::vector<arma::vec>& diff,
    arma::mat& sow_prn_ref)
{
    std::vector<arma::mat>::iterator iter_ref;
    std::vector<arma::mat>::iterator iter_meas;
    std::vector<arma::vec>::iterator iter_diff;
    arma::mat mat_aux;
    arma::vec subtraction_meas;
    arma::vec subtraction_ref;

    arma::mat subtraction_pr_ref = sow_prn_ref;
    arma::vec::iterator iter_vec0 = subtraction_pr_ref.begin_col(0);
    arma::vec::iterator iter_vec1 = subtraction_pr_ref.begin_col(1);
    arma::vec::iterator iter_vec2 = subtraction_pr_ref.begin_col(2);

    for (iter_vec1 = subtraction_pr_ref.begin_col(1); iter_vec1 != subtraction_pr_ref.end_col(1); iter_vec1++)
        {
            arma::vec aux_pr;                  //vector with only 1 element
            arma::vec aux_sow = {*iter_vec0};  //vector with only 1 element
            arma::interp1(ref.at(static_cast<int>(*iter_vec2)).col(0),
                ref.at(static_cast<int>(*iter_vec2)).col(1),
                aux_sow,
                aux_pr);
            *iter_vec1 = aux_pr(0);
            iter_vec0++;
            iter_vec2++;
        }

    iter_ref = ref.begin();
    iter_diff = diff.begin();
    for (iter_meas = meas.begin(); iter_meas != meas.end(); iter_meas++)
        {
            if (!iter_meas->is_empty() && !iter_ref->is_empty())
                {
                    arma::uvec index_ = arma::find(iter_meas->col(0) > iter_ref->at(0, 0));
                    arma::uword index_min = arma::min(index_);
                    index_ = arma::find(iter_meas->col(0) < iter_ref->at(iter_ref->n_rows - 1, 0));
                    arma::uword index_max = arma::max(index_);
                    mat_aux = iter_meas->rows(index_min, index_max);
                    arma::interp1(sow_prn_ref.col(0), sow_prn_ref.col(1), mat_aux.col(0), subtraction_meas);
                    mat_aux.col(1) -= subtraction_meas;
                    arma::vec ref_aligned;
                    arma::interp1(iter_ref->col(0), iter_ref->col(1), mat_aux.col(0), ref_aligned);
                    arma::interp1(subtraction_pr_ref.col(0), subtraction_pr_ref.col(1), mat_aux.col(0), subtraction_ref);
                    ref_aligned -= subtraction_ref;
                    *iter_diff = ref_aligned - mat_aux.col(1);
                }
            iter_ref++;
            iter_diff++;
        }
}


int ObsSystemTest::configure_receiver()
{
    config = std::make_shared<FileConfiguration>(configuration_file_);

    if (config->property("Channels_1C.count", 0) > 0)
        {
            gps_1C = true;
        }
    if (config->property("Channels_1B.count", 0) > 0)
        {
            gal_1B = true;
        }
    if (config->property("Channels_5X.count", 0) > 0)
        {
            gal_E5a = true;
        }
    if (config->property("Channels_7X.count", 0) > 0)  //NOT DEFINITIVE!!!!!
        {
            gps_L5 = true;
        }

    return 0;
}


int ObsSystemTest::run_receiver()
{
    std::shared_ptr<ControlThread> control_thread;
    control_thread = std::make_shared<ControlThread>(config);
    // start receiver
    try
        {
            control_thread->run();
        }
    catch (const boost::exception& e)
        {
            std::cout << "Boost exception: " << boost::diagnostic_information(e);
        }
    catch (const std::exception& ex)
        {
            std::cout << "STD exception: " << ex.what();
        }
    // Get the name of the RINEX obs file generated by the receiver
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    FILE* fp;
    std::string argum2 = std::string("/bin/ls *O | grep GSDR | tail -1");
    char buffer[1035];
    fp = popen(&argum2[0], "r");
    if (fp == NULL)
        {
            std::cout << "Failed to run command: " << argum2 << std::endl;
            return -1;
        }
    while (fgets(buffer, sizeof(buffer), fp) != NULL)
        {
            std::string aux = std::string(buffer);
            generated_rinex_obs = aux.erase(aux.length() - 1, 1);
            internal_rinex_generation = true;
        }
    pclose(fp);
    return 0;
}


void ObsSystemTest::compute_pseudorange_error(
    std::vector<arma::vec>& diff,
    double error_th_mean, double error_th_std,
    std::string signal_name)
{
    int prn_id = 0;
    std::vector<arma::vec>::iterator iter_diff;
    std::vector<double> means;
    std::vector<double> stddevs;
    std::vector<double> prns;
    for (iter_diff = diff.begin(); iter_diff != diff.end(); iter_diff++)
        {
            if (!iter_diff->is_empty())
                {
                    while (iter_diff->has_nan())
                        {
                            bool nan_found = false;
                            int k_aux = 0;
                            while (!nan_found)
                                {
                                    if (!iter_diff->row(k_aux).is_finite())
                                        {
                                            nan_found = true;
                                            iter_diff->shed_row(k_aux);
                                        }
                                    k_aux++;
                                }
                        }
                    double d_mean = std::sqrt(arma::mean(arma::square(*iter_diff)));
                    means.push_back(d_mean);
                    double d_stddev = arma::stddev(*iter_diff);
                    stddevs.push_back(d_stddev);
                    prns.push_back(static_cast<double>(prn_id));
                    std::cout << "-- RMS pseudorange difference for sat " << prn_id << ": " << d_mean;
                    std::cout << ". Std. dev.: " << d_stddev;
                    std::cout << " [m]" << std::endl;
                    EXPECT_LT(d_mean, error_th_mean);
                    EXPECT_LT(d_stddev, error_th_std);
                }
            prn_id++;
        }
    if (FLAGS_plot_obs_sys_test == true)
        {
            const std::string gnuplot_executable(FLAGS_gnuplot_executable);
            if (gnuplot_executable.empty())
                {
                    std::cout << "WARNING: Although the flag plot_obs_sys_test has been set to TRUE," << std::endl;
                    std::cout << "gnuplot has not been found in your system." << std::endl;
                    std::cout << "Test results will not be plotted." << std::endl;
                }
            else
                {
                    try
                        {
                            boost::filesystem::path p(gnuplot_executable);
                            boost::filesystem::path dir = p.parent_path();
                            std::string gnuplot_path = dir.native();
                            Gnuplot::set_GNUPlotPath(gnuplot_path);

                            Gnuplot g1("linespoints");
                            if (FLAGS_show_plots)
                                {
                                    g1.showonscreen();  // window output
                                }
                            else
                                {
                                    g1.disablescreen();
                                }
                            g1.set_title(signal_name + " Pseudorange error");
                            g1.set_grid();
                            g1.set_xlabel("PRN");
                            g1.set_ylabel("Pseudorange error [m]");
                            g1.plot_xy(prns, means, "RMS error");
                            g1.plot_xy(prns, stddevs, "Standard deviation");
                            size_t char_pos = signal_name.find(" ");
                            while (char_pos != std::string::npos)
                                {
                                    signal_name.replace(char_pos, 1, "_");
                                    char_pos = signal_name.find(" ");
                                }
                            char_pos = signal_name.find("/");
                            while (char_pos != std::string::npos)
                                {
                                    signal_name.replace(char_pos, 1, "_");
                                    char_pos = signal_name.find("/");
                                }
                            g1.savetops("Pseudorange_error_" + signal_name);
                            g1.savetopdf("Pseudorange_error_" + signal_name, 18);
                        }
                    catch (const GnuplotException& ge)
                        {
                            std::cout << ge.what() << std::endl;
                        }
                }
        }
}


void ObsSystemTest::compute_carrierphase_error(
    std::vector<arma::vec>& diff,
    double error_th_mean, double error_th_std,
    std::string signal_name)
{
    int prn_id = 0;
    std::vector<double> means;
    std::vector<double> stddevs;
    std::vector<double> prns;
    std::vector<arma::vec>::iterator iter_diff;
    for (iter_diff = diff.begin(); iter_diff != diff.end(); iter_diff++)
        {
            if (!iter_diff->is_empty())
                {
                    while (iter_diff->has_nan())
                        {
                            bool nan_found = false;
                            int k_aux = 0;
                            while (!nan_found)
                                {
                                    if (!iter_diff->row(k_aux).is_finite())
                                        {
                                            nan_found = true;
                                            iter_diff->shed_row(k_aux);
                                        }
                                    k_aux++;
                                }
                        }
                    double d_mean = std::sqrt(arma::mean(arma::square(*iter_diff)));
                    means.push_back(d_mean);
                    double d_stddev = arma::stddev(*iter_diff);
                    stddevs.push_back(d_stddev);
                    prns.push_back(static_cast<double>(prn_id));
                    std::cout << "-- RMS carrier phase difference for sat " << prn_id << ": " << d_mean;
                    std::cout << ". Std. dev.: " << d_stddev;
                    std::cout << " whole cycles" << std::endl;
                    EXPECT_LT(d_mean, error_th_mean);
                    EXPECT_LT(d_stddev, error_th_std);
                }
            prn_id++;
        }
    if (FLAGS_plot_obs_sys_test == true)
        {
            const std::string gnuplot_executable(FLAGS_gnuplot_executable);
            if (gnuplot_executable.empty())
                {
                    std::cout << "WARNING: Although the flag plot_obs_sys_test has been set to TRUE," << std::endl;
                    std::cout << "gnuplot has not been found in your system." << std::endl;
                    std::cout << "Test results will not be plotted." << std::endl;
                }
            else
                {
                    try
                        {
                            boost::filesystem::path p(gnuplot_executable);
                            boost::filesystem::path dir = p.parent_path();
                            std::string gnuplot_path = dir.native();
                            Gnuplot::set_GNUPlotPath(gnuplot_path);

                            Gnuplot g1("linespoints");
                            if (FLAGS_show_plots)
                                {
                                    g1.showonscreen();  // window output
                                }
                            else
                                {
                                    g1.disablescreen();
                                }
                            g1.set_title(signal_name + " Carrier phase error");
                            g1.set_grid();
                            g1.set_xlabel("PRN");
                            g1.set_ylabel("Carrier phase error [whole cycles]");
                            g1.plot_xy(prns, means, "RMS error");
                            g1.plot_xy(prns, stddevs, "Standard deviation");
                            size_t char_pos = signal_name.find(" ");
                            while (char_pos != std::string::npos)
                                {
                                    signal_name.replace(char_pos, 1, "_");
                                    char_pos = signal_name.find(" ");
                                }
                            char_pos = signal_name.find("/");
                            while (char_pos != std::string::npos)
                                {
                                    signal_name.replace(char_pos, 1, "_");
                                    char_pos = signal_name.find("/");
                                }
                            g1.savetops("Carrier_phase_error_" + signal_name);
                            g1.savetopdf("Carrier_phase_error_" + signal_name, 18);
                        }
                    catch (const GnuplotException& ge)
                        {
                            std::cout << ge.what() << std::endl;
                        }
                }
        }
}


void ObsSystemTest::compute_doppler_error(
    std::vector<arma::vec>& diff,
    double error_th_mean, double error_th_std,
    std::string signal_name)
{
    int prn_id = 0;
    std::vector<double> means;
    std::vector<double> stddevs;
    std::vector<double> prns;
    std::vector<arma::vec>::iterator iter_diff;
    for (iter_diff = diff.begin(); iter_diff != diff.end(); iter_diff++)
        {
            if (!iter_diff->is_empty())
                {
                    while (iter_diff->has_nan())
                        {
                            bool nan_found = false;
                            int k_aux = 0;
                            while (!nan_found)
                                {
                                    if (!iter_diff->row(k_aux).is_finite())
                                        {
                                            nan_found = true;
                                            iter_diff->shed_row(k_aux);
                                        }
                                    k_aux++;
                                }
                        }
                    double d_mean = std::sqrt(arma::mean(arma::square(*iter_diff)));
                    means.push_back(d_mean);
                    double d_stddev = arma::stddev(*iter_diff);
                    stddevs.push_back(d_stddev);
                    prns.push_back(static_cast<double>(prn_id));
                    std::cout << "-- RMS Doppler difference for sat " << prn_id << ": " << d_mean;
                    std::cout << ". Std. dev.: " << d_stddev;
                    std::cout << " [Hz]" << std::endl;
                    EXPECT_LT(d_mean, error_th_mean);
                    EXPECT_LT(d_stddev, error_th_std);
                }
            prn_id++;
        }
    if (FLAGS_plot_obs_sys_test == true)
        {
            const std::string gnuplot_executable(FLAGS_gnuplot_executable);
            if (gnuplot_executable.empty())
                {
                    std::cout << "WARNING: Although the flag plot_obs_sys_test has been set to TRUE," << std::endl;
                    std::cout << "gnuplot has not been found in your system." << std::endl;
                    std::cout << "Test results will not be plotted." << std::endl;
                }
            else
                {
                    try
                        {
                            boost::filesystem::path p(gnuplot_executable);
                            boost::filesystem::path dir = p.parent_path();
                            std::string gnuplot_path = dir.native();
                            Gnuplot::set_GNUPlotPath(gnuplot_path);

                            Gnuplot g1("linespoints");
                            if (FLAGS_show_plots)
                                {
                                    g1.showonscreen();  // window output
                                }
                            else
                                {
                                    g1.disablescreen();
                                }
                            g1.set_title(signal_name + " Doppler error");
                            g1.set_grid();
                            g1.set_xlabel("PRN");
                            g1.set_ylabel("Doppler error [Hz]");
                            g1.plot_xy(prns, means, "RMS error");
                            g1.plot_xy(prns, stddevs, "Standard deviation");
                            size_t char_pos = signal_name.find(" ");
                            while (char_pos != std::string::npos)
                                {
                                    signal_name.replace(char_pos, 1, "_");
                                    char_pos = signal_name.find(" ");
                                }
                            char_pos = signal_name.find("/");
                            while (char_pos != std::string::npos)
                                {
                                    signal_name.replace(char_pos, 1, "_");
                                    char_pos = signal_name.find("/");
                                }
                            g1.savetops("Doppler_error_" + signal_name);
                            g1.savetopdf("Doppler_error_" + signal_name, 18);
                        }
                    catch (const GnuplotException& ge)
                        {
                            std::cout << ge.what() << std::endl;
                        }
                }
        }
}


void ObsSystemTest::check_results()
{
    arma::mat sow_prn_ref;
    if (gps_1C)
        {
            std::vector<arma::mat> pseudorange_ref(num_prn_gps);
            std::vector<arma::mat> carrierphase_ref(num_prn_gps);
            std::vector<arma::mat> doppler_ref(num_prn_gps);

            std::vector<arma::mat> pseudorange_meas(num_prn_gps);
            std::vector<arma::mat> carrierphase_meas(num_prn_gps);
            std::vector<arma::mat> doppler_meas(num_prn_gps);

            read_rinex_files(pseudorange_ref, carrierphase_ref, doppler_ref, pseudorange_meas, carrierphase_meas, doppler_meas, sow_prn_ref, 0);

            // Time alignment and difference computation

            std::vector<arma::vec> pr_diff(num_prn_gps);
            std::vector<arma::vec> cp_diff(num_prn_gps);
            std::vector<arma::vec> dp_diff(num_prn_gps);
            time_alignment_diff_pr(pseudorange_ref, pseudorange_meas, pr_diff, sow_prn_ref);
            time_alignment_diff_cp(carrierphase_ref, carrierphase_meas, cp_diff);
            time_alignment_diff(doppler_ref, doppler_meas, dp_diff);

            // Results
            std::cout << std::endl;
            std::cout << std::endl;
            std::cout << "GPS L1 C/A obs. results" << std::endl;

            // Compute pseudorange error

            compute_pseudorange_error(pr_diff, pseudorange_error_th_mean, pseudorange_error_th_std, "GPS L1 C/A");

            // Compute carrier phase error

            compute_carrierphase_error(cp_diff, carrierphase_error_th_mean, carrierphase_error_th_std, "GPS L1 C/A");

            // Compute Doppler error

            compute_doppler_error(dp_diff, doppler_error_th_mean, doppler_error_th_std, "GPS L1 C/A");
        }
    if (gps_L5)
        {
            std::vector<arma::mat> pseudorange_ref(num_prn_gps);
            std::vector<arma::mat> carrierphase_ref(num_prn_gps);
            std::vector<arma::mat> doppler_ref(num_prn_gps);

            std::vector<arma::mat> pseudorange_meas(num_prn_gps);
            std::vector<arma::mat> carrierphase_meas(num_prn_gps);
            std::vector<arma::mat> doppler_meas(num_prn_gps);

            read_rinex_files(pseudorange_ref, carrierphase_ref, doppler_ref, pseudorange_meas, carrierphase_meas, doppler_meas, sow_prn_ref, 2);

            // Time alignment and difference computation

            std::vector<arma::vec> pr_diff(num_prn_gps);
            std::vector<arma::vec> cp_diff(num_prn_gps);
            std::vector<arma::vec> dp_diff(num_prn_gps);
            time_alignment_diff_pr(pseudorange_ref, pseudorange_meas, pr_diff, sow_prn_ref);
            time_alignment_diff_cp(carrierphase_ref, carrierphase_meas, cp_diff);
            time_alignment_diff(doppler_ref, doppler_meas, dp_diff);

            // Results
            std::cout << std::endl;
            std::cout << std::endl;
            std::cout << "GPS L5 obs. results" << std::endl;

            // Compute pseudorange error

            compute_pseudorange_error(pr_diff, pseudorange_error_th_mean, pseudorange_error_th_std, "GPS L5");

            // Compute carrier phase error

            compute_carrierphase_error(cp_diff, carrierphase_error_th_mean, carrierphase_error_th_std, "GPS L5");

            // Compute Doppler error

            compute_doppler_error(dp_diff, doppler_error_th_mean, doppler_error_th_std, "GPS L5");
        }
    if (gal_1B)
        {
            std::vector<arma::mat> pseudorange_ref(num_prn_gal);
            std::vector<arma::mat> carrierphase_ref(num_prn_gal);
            std::vector<arma::mat> doppler_ref(num_prn_gal);

            std::vector<arma::mat> pseudorange_meas(num_prn_gal);
            std::vector<arma::mat> carrierphase_meas(num_prn_gal);
            std::vector<arma::mat> doppler_meas(num_prn_gal);

            read_rinex_files(pseudorange_ref, carrierphase_ref, doppler_ref, pseudorange_meas, carrierphase_meas, doppler_meas, sow_prn_ref, 1);

            // Time alignment and difference computation

            std::vector<arma::vec> pr_diff(num_prn_gal);
            std::vector<arma::vec> cp_diff(num_prn_gal);
            std::vector<arma::vec> dp_diff(num_prn_gal);
            time_alignment_diff_pr(pseudorange_ref, pseudorange_meas, pr_diff, sow_prn_ref);
            time_alignment_diff_cp(carrierphase_ref, carrierphase_meas, cp_diff);
            time_alignment_diff(doppler_ref, doppler_meas, dp_diff);

            // Results
            std::cout << std::endl;
            std::cout << std::endl;
            std::cout << "Galileo E1B obs. results" << std::endl;

            // Compute pseudorange error

            compute_pseudorange_error(pr_diff, pseudorange_error_th_mean, pseudorange_error_th_std, "Galileo E1B");

            // Compute carrier phase error

            compute_carrierphase_error(cp_diff, carrierphase_error_th_mean, carrierphase_error_th_std, "Galileo E1B");

            // Compute Doppler error

            compute_doppler_error(dp_diff, doppler_error_th_mean, doppler_error_th_std, "Galileo E1B");
        }
    if (gal_E5a)
        {
            std::vector<arma::mat> pseudorange_ref(num_prn_gal);
            std::vector<arma::mat> carrierphase_ref(num_prn_gal);
            std::vector<arma::mat> doppler_ref(num_prn_gal);

            std::vector<arma::mat> pseudorange_meas(num_prn_gal);
            std::vector<arma::mat> carrierphase_meas(num_prn_gal);
            std::vector<arma::mat> doppler_meas(num_prn_gal);

            read_rinex_files(pseudorange_ref, carrierphase_ref, doppler_ref, pseudorange_meas, carrierphase_meas, doppler_meas, sow_prn_ref, 3);

            // Time alignment and difference computation

            std::vector<arma::vec> pr_diff(num_prn_gal);
            std::vector<arma::vec> cp_diff(num_prn_gal);
            std::vector<arma::vec> dp_diff(num_prn_gal);
            time_alignment_diff_pr(pseudorange_ref, pseudorange_meas, pr_diff, sow_prn_ref);
            time_alignment_diff_cp(carrierphase_ref, carrierphase_meas, cp_diff);
            time_alignment_diff(doppler_ref, doppler_meas, dp_diff);

            // Results
            std::cout << std::endl;
            std::cout << std::endl;
            std::cout << "Galileo E5a obs. results" << std::endl;

            // Compute pseudorange error

            compute_pseudorange_error(pr_diff, pseudorange_error_th_mean, pseudorange_error_th_std, "Galileo E5a");

            // Compute carrier phase error

            compute_carrierphase_error(cp_diff, carrierphase_error_th_mean, carrierphase_error_th_std, "Galileo E5a");

            // Compute Doppler error

            compute_doppler_error(dp_diff, doppler_error_th_mean, doppler_error_th_std, "Galileo E5a");
        }
}


TEST_F(ObsSystemTest, Observables_system_test)
{
    std::cout << "Validating input RINEX obs (TRUE) file: " << filename_rinex_obs << " ..." << std::endl;
    bool is_rinex_obs_valid = check_valid_rinex_obs(filename_rinex_obs, 3);
    ASSERT_EQ(true, is_rinex_obs_valid) << "The RINEX observation file " << filename_rinex_obs << " is not well formed. Only RINEX v. 3.00 files are allowed";
    std::cout << "The file is valid." << std::endl;
    // Configure receiver
    configure_receiver();
    if (generated_rinex_obs.compare("default_string") == 0)
        {
            // Run the receiver
            ASSERT_EQ(run_receiver(), 0) << "Problem executing the software-defined signal generator";
        }
    std::cout << "Validating RINEX obs file obtained by GNSS-SDR: " << generated_rinex_obs << " ..." << std::endl;
    bool is_gen_rinex_obs_valid = false;
    if (internal_rinex_generation)
        {
            is_gen_rinex_obs_valid = check_valid_rinex_obs("./" + generated_rinex_obs, config->property("PVT.rinex_version", 3));
        }
    else
        {
            is_gen_rinex_obs_valid = check_valid_rinex_obs(generated_rinex_obs, config->property("PVT.rinex_version", 3));
        }
    ASSERT_EQ(true, is_gen_rinex_obs_valid) << "The RINEX observation file " << generated_rinex_obs << ", generated by GNSS-SDR, is not well formed.";
    std::cout << "The file is valid." << std::endl;
    // Check results
    check_results();
}


int main(int argc, char** argv)
{
    std::cout << "Running GNSS-SDR in Space Observables validation test..." << std::endl;
    int res = 0;
    try
        {
            testing::InitGoogleTest(&argc, argv);
        }
    catch (...)
        {
        }  // catch the "testing::internal::<unnamed>::ClassUniqueToAlwaysTrue" from gtest

    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    // Run the Tests
    try
        {
            res = RUN_ALL_TESTS();
        }
    catch (...)
        {
            LOG(WARNING) << "Unexpected catch";
        }
    google::ShutDownCommandLineFlags();
    return res;
}
