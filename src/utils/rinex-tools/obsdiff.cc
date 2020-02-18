/*!
 * \file obsdiff.cc
 * \brief  This class implements a single difference and double difference
 * comparison algorithm to evaluate receiver's performance at observable level
 * \authors <ul>
 *          <li> Javier Arribas, 2020. jarribas(at)cttc.es
 *          </ul>
 *
 *
 * -------------------------------------------------------------------------
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
 * -------------------------------------------------------------------------
 */

#include "MATH_CONSTANTS.h"
#include "gnuplot_i.h"
#include "obsdiff_flags.h"
#include <armadillo>
#include <gpstk/Rinex3ObsBase.hpp>
#include <gpstk/Rinex3ObsData.hpp>
#include <gpstk/Rinex3ObsHeader.hpp>
#include <gpstk/Rinex3ObsStream.hpp>
#include <gpstk/RinexUtilities.hpp>
#include <matio.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <numeric>
#include <set>
#include <thread>


// Create the lists of GNSS satellites
std::set<int> available_gps_prn = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
    29, 30, 31, 32};

std::set<int> available_sbas_prn = {123, 131, 135, 136, 138};

std::set<int> available_galileo_prn = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
    29, 30, 31, 32, 33, 34, 35, 36};


bool file_exist(const char* fileName)
{
    std::ifstream infile(fileName);
    return infile.good();
}


std::map<int, arma::mat> ReadRinexObs(std::string rinex_file, char system, std::string signal)
{
    std::map<int, arma::mat> obs_map;
    if (not file_exist(rinex_file.c_str()))
        {
            std::cout << "Warning: RINEX Obs file " << rinex_file << " does not exist\n";
            return obs_map;
        }
    // Open and read reference RINEX observables file
    try
        {
            gpstk::Rinex3ObsStream r_ref(rinex_file);
            r_ref.exceptions(std::ios::failbit);
            gpstk::Rinex3ObsData r_ref_data;
            gpstk::Rinex3ObsHeader r_ref_header;

            gpstk::RinexDatum dataobj;

            r_ref >> r_ref_header;

            std::set<int> PRN_set;
            gpstk::SatID prn;
            switch (system)
                {
                case 'G':
                    prn.system = gpstk::SatID::systemGPS;
                    PRN_set = available_gps_prn;
                    break;
                case 'E':
                    prn.system = gpstk::SatID::systemGalileo;
                    PRN_set = available_galileo_prn;
                    break;
                default:
                    prn.system = gpstk::SatID::systemGPS;
                    PRN_set = available_gps_prn;
                }

            std::cout << "Reading RINEX OBS file " << rinex_file << " ...\n";
            while (r_ref >> r_ref_data)
                {
                    for (std::set<int>::iterator prn_it = PRN_set.begin(); prn_it != PRN_set.end(); ++prn_it)
                        {
                            prn.id = *prn_it;
                            gpstk::CommonTime time = r_ref_data.time;
                            double sow(static_cast<gpstk::GPSWeekSecond>(time).sow);

                            auto pointer = r_ref_data.obs.find(prn);

                            if (pointer != r_ref_data.obs.end())
                                {
                                    // insert next column
                                    try
                                        {
                                            obs_map.at(prn.id);
                                        }
                                    catch (const std::out_of_range& oor)
                                        {
                                            obs_map[prn.id] = arma::mat();
                                        }

                                    arma::mat& obs_mat = obs_map[prn.id];
                                    obs_mat.insert_rows(obs_mat.n_rows, arma::zeros<arma::mat>(1, 4));

                                    if (strcmp("1C\0", signal.c_str()) == 0)
                                        {
                                            obs_mat.at(obs_mat.n_rows - 1, 0) = sow;
                                            dataobj = r_ref_data.getObs(prn, "C1C", r_ref_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 1) = dataobj.data;  //C1C P1 (psudorange L1)
                                            dataobj = r_ref_data.getObs(prn, "D1C", r_ref_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 2) = dataobj.data;  //D1C Carrier Doppler
                                            dataobj = r_ref_data.getObs(prn, "L1C", r_ref_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 3) = dataobj.data;  //L1C Carrier Phase
                                        }
                                    else if (strcmp("1B\0", signal.c_str()) == 0)
                                        {
                                            obs_mat.at(obs_mat.n_rows - 1, 0) = sow;
                                            dataobj = r_ref_data.getObs(prn, "C1B", r_ref_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 1) = dataobj.data;
                                            dataobj = r_ref_data.getObs(prn, "D1B", r_ref_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 2) = dataobj.data;
                                            dataobj = r_ref_data.getObs(prn, "L1B", r_ref_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 3) = dataobj.data;
                                        }
                                    else if (strcmp("2S\0", signal.c_str()) == 0)  //L2M
                                        {
                                            obs_mat.at(obs_mat.n_rows - 1, 0) = sow;
                                            dataobj = r_ref_data.getObs(prn, "C2S", r_ref_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 1) = dataobj.data;
                                            dataobj = r_ref_data.getObs(prn, "D2S", r_ref_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 2) = dataobj.data;
                                            dataobj = r_ref_data.getObs(prn, "L2S", r_ref_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 3) = dataobj.data;
                                        }
                                    else if (strcmp("L5\0", signal.c_str()) == 0)
                                        {
                                            obs_mat.at(obs_mat.n_rows - 1, 0) = sow;
                                            dataobj = r_ref_data.getObs(prn, "C5I", r_ref_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 1) = dataobj.data;
                                            dataobj = r_ref_data.getObs(prn, "D5I", r_ref_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 2) = dataobj.data;
                                            dataobj = r_ref_data.getObs(prn, "L5I", r_ref_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 3) = dataobj.data;
                                        }
                                    else if (strcmp("5X\0", signal.c_str()) == 0)  // Simulator gives RINEX with E5a+E5b. Doppler and accumulated Carrier phase WILL differ
                                        {
                                            obs_mat.at(obs_mat.n_rows - 1, 0) = sow;
                                            dataobj = r_ref_data.getObs(prn, "C8I", r_ref_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 1) = dataobj.data;
                                            dataobj = r_ref_data.getObs(prn, "D8I", r_ref_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 2) = dataobj.data;
                                            dataobj = r_ref_data.getObs(prn, "L8I", r_ref_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 3) = dataobj.data;
                                        }
                                    else
                                        {
                                            std::cout << "ReadRinexObs unknown signal requested: " << signal << std::endl;
                                            return obs_map;
                                        }
                                }
                        }
                }  // end while
        }          // End of 'try' block
    catch (const gpstk::FFStreamError& e)
        {
            std::cout << e;
            return obs_map;
        }
    catch (const gpstk::Exception& e)
        {
            std::cout << e;
            return obs_map;
        }
    catch (const std::exception& e)
        {
            std::cout << "Exception: " << e.what();
            std::cout << "unknown error.  I don't feel so well..." << std::endl;
            return obs_map;
        }

    return obs_map;
}


bool save_mat_xy(std::vector<double>& x, std::vector<double>& y, std::string filename)
{
    try
        {
            // WRITE MAT FILE
            mat_t* matfp;
            matvar_t* matvar;
            filename.append(".mat");
            // std::cout << "save_mat_xy write " << filename << std::endl;
            matfp = Mat_CreateVer(filename.c_str(), nullptr, MAT_FT_MAT5);
            if (reinterpret_cast<int64_t*>(matfp) != nullptr)
                {
                    size_t dims[2] = {1, x.size()};
                    matvar = Mat_VarCreate("x", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, &x[0], 0);
                    Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
                    Mat_VarFree(matvar);

                    matvar = Mat_VarCreate("y", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, &y[0], 0);
                    Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
                    Mat_VarFree(matvar);
                }
            else
                {
                    std::cout << "save_mat_xy: error creating file" << std::endl;
                }
            Mat_Close(matfp);
            return true;
        }
    catch (const std::exception& ex)
        {
            std::cout << "save_mat_xy: " << ex.what() << std::endl;
            return false;
        }
}


bool save_mat_x(std::vector<double>* x, std::string filename)
{
    try
        {
            // WRITE MAT FILE
            mat_t* matfp;
            matvar_t* matvar;
            filename.append(".mat");
            std::cout << "save_mat_x write " << filename << std::endl;
            matfp = Mat_CreateVer(filename.c_str(), nullptr, MAT_FT_MAT5);
            if (reinterpret_cast<int64_t*>(matfp) != nullptr)
                {
                    std::array<size_t, 2> dims{1, x->size()};
                    matvar = Mat_VarCreate("x", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), &x[0], 0);
                    Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
                    Mat_VarFree(matvar);
                }
            else
                {
                    std::cout << "save_mat_x: error creating file" << std::endl;
                }
            Mat_Close(matfp);
            return true;
        }
    catch (const std::exception& ex)
        {
            std::cout << "save_mat_x: " << ex.what() << std::endl;
            return false;
        }
}


void check_results_carrier_phase_double_diff(
    arma::mat& true_ch0,
    arma::mat& true_ch1,
    arma::mat& measured_ch0,
    arma::mat& measured_ch1,
    const std::string& data_title)
{
    // 1. True value interpolation to match the measurement times
    arma::vec measurement_time = measured_ch0.col(0);

    arma::vec true_ch0_carrier_phase_interp;
    arma::vec true_ch1_carrier_phase_interp;
    arma::interp1(true_ch0.col(0), true_ch0.col(3), measurement_time, true_ch0_carrier_phase_interp);
    arma::interp1(true_ch1.col(0), true_ch1.col(3), measurement_time, true_ch1_carrier_phase_interp);

    arma::vec meas_ch1_carrier_phase_interp;
    arma::interp1(measured_ch1.col(0), measured_ch1.col(3), measurement_time, meas_ch1_carrier_phase_interp);
    // generate double difference accumulated carrier phases
    // compute error without the accumulated carrier phase offsets (which depends on the receiver starting time)
    arma::vec delta_true_carrier_phase_cycles = (true_ch0_carrier_phase_interp - true_ch0_carrier_phase_interp(0)) - (true_ch1_carrier_phase_interp - true_ch1_carrier_phase_interp(0));
    arma::vec delta_measured_carrier_phase_cycles = (measured_ch0.col(3) - measured_ch0.col(3)(0)) - (meas_ch1_carrier_phase_interp - meas_ch1_carrier_phase_interp(0));

    // remove NaN
    arma::uvec NaN_in_true_data = arma::find_nonfinite(delta_true_carrier_phase_cycles);
    arma::uvec NaN_in_measured_data = arma::find_nonfinite(delta_measured_carrier_phase_cycles);

    arma::mat tmp_mat = arma::conv_to<arma::mat>::from(delta_true_carrier_phase_cycles);
    tmp_mat.shed_rows(arma::join_cols(NaN_in_true_data, NaN_in_measured_data));
    delta_true_carrier_phase_cycles = tmp_mat.col(0);

    tmp_mat = arma::conv_to<arma::mat>::from(delta_measured_carrier_phase_cycles);
    tmp_mat.shed_rows(arma::join_cols(NaN_in_true_data, NaN_in_measured_data));
    delta_measured_carrier_phase_cycles = tmp_mat.col(0);

    tmp_mat = arma::conv_to<arma::mat>::from(measurement_time);
    tmp_mat.shed_rows(arma::join_cols(NaN_in_true_data, NaN_in_measured_data));
    measurement_time = tmp_mat.col(0);

    std::vector<double>
        time_vector(measurement_time.colptr(0), measurement_time.colptr(0) + measurement_time.n_rows);

    if (measurement_time.size() > 0)
        {
            // debug
            //    std::vector<double> tmp_time_vec(measurement_time.colptr(0),
            //        measurement_time.colptr(0) + measurement_time.n_rows);
            //    std::vector<double> tmp_vector_y6(delta_true_carrier_doppler_cycles.colptr(0),
            //        delta_true_carrier_doppler_cycles.colptr(0) + delta_true_carrier_doppler_cycles.n_rows);
            //    save_mat_xy(tmp_time_vec, tmp_vector_y6, std::string("true_delta_doppler"));
            //    std::vector<double> tmp_vector_y7(delta_measured_carrier_doppler_cycles.colptr(0),
            //        delta_measured_carrier_doppler_cycles.colptr(0) + delta_measured_carrier_doppler_cycles.n_rows);
            //    save_mat_xy(tmp_time_vec, tmp_vector_y7, std::string("measured_delta_doppler"));

            // 2. RMSE
            arma::vec err;
            err = delta_measured_carrier_phase_cycles - delta_true_carrier_phase_cycles;
            arma::vec err2 = arma::square(err);
            double rmse = sqrt(arma::mean(err2));

            // 3. Mean err and variance
            double error_mean = arma::mean(err);
            double error_var = arma::var(err);

            // 4. Peaks
            double max_error = arma::max(err);
            double min_error = arma::min(err);

            // 5. report
            std::streamsize ss = std::cout.precision();
            std::cout << std::setprecision(10) << data_title << "Double diff Carrier Phase RMSE = "
                      << rmse << ", mean = " << error_mean
                      << ", stdev = " << sqrt(error_var)
                      << " (max,min) = " << max_error
                      << "," << min_error
                      << " [Cycles]" << std::endl;
            std::cout.precision(ss);

            // plots
            if (FLAGS_show_plots)
                {
                    Gnuplot g3("linespoints");
                    g3.set_title(data_title + "Double diff Carrier Phase error [Cycles]");
                    g3.set_grid();
                    g3.set_xlabel("Time [s]");
                    g3.set_ylabel("Double diff Carrier Phase error [Cycles]");
                    // conversion between arma::vec and std:vector
                    std::vector<double> range_error_m(err.colptr(0), err.colptr(0) + err.n_rows);
                    g3.cmd("set key box opaque");
                    g3.plot_xy(time_vector, range_error_m,
                        "Double diff Carrier Phase error");
                    g3.set_legend();
                    g3.savetops(data_title + "double_diff_carrier_phase_error");

                    g3.showonscreen();  // window output
                }
            // check results against the test tolerance
            //            ASSERT_LT(rmse, 0.25);
            //            ASSERT_LT(error_mean, 0.2);
            //            ASSERT_GT(error_mean, -0.2);
            //            ASSERT_LT(error_var, 0.5);
            //            ASSERT_LT(max_error, 0.5);
            //            ASSERT_GT(min_error, -0.5);
        }
    else
        {
            std::cout << "No valid data\n";
        }
}


void check_results_carrier_phase_single_diff(
    arma::mat& measured_ch0,
    arma::mat& measured_ch1,
    const std::string& data_title)
{
    // 1. True value interpolation to match the measurement times
    arma::vec measurement_time = measured_ch0.col(0);

    arma::vec meas_ch1_carrier_phase_interp;
    arma::interp1(measured_ch1.col(0), measured_ch1.col(3), measurement_time, meas_ch1_carrier_phase_interp);
    // generate single difference accumulated carrier phases
    // compute error without the accumulated carrier phase offsets (which depends on the receiver starting time)
    arma::vec delta_measured_carrier_phase_cycles = (measured_ch0.col(3) - measured_ch0.col(3)(0)) - (meas_ch1_carrier_phase_interp - meas_ch1_carrier_phase_interp(0));

    // remove NaN
    arma::uvec NaN_in_measured_data = arma::find_nonfinite(delta_measured_carrier_phase_cycles);

    arma::mat tmp_mat = arma::conv_to<arma::mat>::from(delta_measured_carrier_phase_cycles);
    tmp_mat.shed_rows(NaN_in_measured_data);
    delta_measured_carrier_phase_cycles = tmp_mat.col(0);

    tmp_mat = arma::conv_to<arma::mat>::from(measurement_time);
    tmp_mat.shed_rows(NaN_in_measured_data);
    measurement_time = tmp_mat.col(0);

    std::vector<double>
        time_vector(measurement_time.colptr(0), measurement_time.colptr(0) + measurement_time.n_rows);

    if (measurement_time.size() > 0)
        {
            // 2. RMSE
            arma::vec err;
            err = delta_measured_carrier_phase_cycles;
            arma::vec err2 = arma::square(err);
            double rmse = sqrt(arma::mean(err2));

            // 3. Mean err and variance
            double error_mean = arma::mean(err);
            double error_var = arma::var(err);

            // 4. Peaks
            double max_error = arma::max(err);
            double min_error = arma::min(err);

            // 5. report
            std::streamsize ss = std::cout.precision();
            std::cout << std::setprecision(10) << data_title << "Single diff Carrier Phase RMSE = "
                      << rmse << ", mean = " << error_mean
                      << ", stdev = " << sqrt(error_var)
                      << " (max,min) = " << max_error
                      << "," << min_error
                      << " [Cycles]" << std::endl;
            std::cout.precision(ss);

            // plots
            if (FLAGS_show_plots)
                {
                    Gnuplot g3("linespoints");
                    g3.set_title(data_title + "Single diff Carrier Phase error [Cycles]");
                    g3.set_grid();
                    g3.set_xlabel("Time [s]");
                    g3.set_ylabel("Single diff Carrier Phase error [Cycles]");
                    // conversion between arma::vec and std:vector
                    std::vector<double> range_error_m(err.colptr(0), err.colptr(0) + err.n_rows);
                    g3.cmd("set key box opaque");
                    g3.plot_xy(time_vector, range_error_m,
                        "Single diff Carrier Phase error");
                    g3.set_legend();
                    g3.savetops(data_title + "single_diff_carrier_phase_error");

                    g3.showonscreen();  // window output
                }
        }
    else
        {
            std::cout << "No valid data\n";
        }
}


void check_results_carrier_doppler_double_diff(
    arma::mat& true_ch0,
    arma::mat& true_ch1,
    arma::mat& measured_ch0,
    arma::mat& measured_ch1,
    const std::string& data_title)
{
    // 1. True value interpolation to match the measurement times
    arma::vec measurement_time = measured_ch0.col(0);

    arma::vec true_ch0_carrier_doppler_interp;
    arma::vec true_ch1_carrier_doppler_interp;
    arma::interp1(true_ch0.col(0), true_ch0.col(2), measurement_time, true_ch0_carrier_doppler_interp);
    arma::interp1(true_ch1.col(0), true_ch1.col(2), measurement_time, true_ch1_carrier_doppler_interp);

    arma::vec meas_ch1_carrier_doppler_interp;
    arma::interp1(measured_ch1.col(0), measured_ch1.col(2), measurement_time, meas_ch1_carrier_doppler_interp);
    // generate double difference carrier Doppler
    arma::vec delta_true_carrier_doppler_cycles = true_ch0_carrier_doppler_interp - true_ch1_carrier_doppler_interp;
    arma::vec delta_measured_carrier_doppler_cycles = measured_ch0.col(2) - meas_ch1_carrier_doppler_interp;

    // remove NaN
    arma::uvec NaN_in_true_data = arma::find_nonfinite(delta_true_carrier_doppler_cycles);
    arma::uvec NaN_in_measured_data = arma::find_nonfinite(delta_measured_carrier_doppler_cycles);

    arma::mat tmp_mat = arma::conv_to<arma::mat>::from(delta_true_carrier_doppler_cycles);
    tmp_mat.shed_rows(arma::join_cols(NaN_in_true_data, NaN_in_measured_data));
    delta_true_carrier_doppler_cycles = tmp_mat.col(0);

    tmp_mat = arma::conv_to<arma::mat>::from(delta_measured_carrier_doppler_cycles);
    tmp_mat.shed_rows(arma::join_cols(NaN_in_true_data, NaN_in_measured_data));
    delta_measured_carrier_doppler_cycles = tmp_mat.col(0);

    tmp_mat = arma::conv_to<arma::mat>::from(measurement_time);
    tmp_mat.shed_rows(arma::join_cols(NaN_in_true_data, NaN_in_measured_data));
    measurement_time = tmp_mat.col(0);

    std::vector<double>
        time_vector(measurement_time.colptr(0), measurement_time.colptr(0) + measurement_time.n_rows);

    if (measurement_time.size() > 0)
        {
            // debug
            //    std::vector<double> tmp_time_vec(measurement_time.colptr(0),
            //        measurement_time.colptr(0) + measurement_time.n_rows);
            //    std::vector<double> tmp_vector_y6(delta_true_carrier_doppler_cycles.colptr(0),
            //        delta_true_carrier_doppler_cycles.colptr(0) + delta_true_carrier_doppler_cycles.n_rows);
            //    save_mat_xy(tmp_time_vec, tmp_vector_y6, std::string("true_delta_doppler"));
            //    std::vector<double> tmp_vector_y7(delta_measured_carrier_doppler_cycles.colptr(0),
            //        delta_measured_carrier_doppler_cycles.colptr(0) + delta_measured_carrier_doppler_cycles.n_rows);
            //    save_mat_xy(tmp_time_vec, tmp_vector_y7, std::string("measured_delta_doppler"));

            // 2. RMSE
            arma::vec err;
            err = delta_measured_carrier_doppler_cycles - delta_true_carrier_doppler_cycles;
            arma::vec err2 = arma::square(err);
            double rmse = sqrt(arma::mean(err2));

            // 3. Mean err and variance
            double error_mean = arma::mean(err);
            double error_var = arma::var(err);

            // 4. Peaks
            double max_error = arma::max(err);
            double min_error = arma::min(err);

            // 5. report
            std::streamsize ss = std::cout.precision();
            std::cout << std::setprecision(10) << data_title << "Double diff Carrier Doppler RMSE = "
                      << rmse << ", mean = " << error_mean
                      << ", stdev = " << sqrt(error_var)
                      << " (max,min) = " << max_error
                      << "," << min_error
                      << " [Hz]" << std::endl;
            std::cout.precision(ss);

            // plots
            if (FLAGS_show_plots)
                {
                    Gnuplot g3("linespoints");
                    g3.set_title(data_title + "Double diff Carrier Doppler error [Hz]");
                    g3.set_grid();
                    g3.set_xlabel("Time [s]");
                    g3.set_ylabel("Double diff Carrier Doppler error [Hz]");
                    // conversion between arma::vec and std:vector
                    std::vector<double> range_error_m(err.colptr(0), err.colptr(0) + err.n_rows);
                    g3.cmd("set key box opaque");
                    g3.plot_xy(time_vector, range_error_m,
                        "Double diff Carrier Doppler error");
                    g3.set_legend();
                    g3.savetops(data_title + "double_diff_carrier_doppler_error");

                    g3.showonscreen();  // window output
                }

            // check results against the test tolerance
            //            ASSERT_LT(error_mean, 5);
            //            ASSERT_GT(error_mean, -5);
            //            // assuming PLL BW=35
            //            ASSERT_LT(error_var, 250);
            //            ASSERT_LT(max_error, 100);
            //            ASSERT_GT(min_error, -100);
            //            ASSERT_LT(rmse, 30);
        }
    else
        {
            std::cout << "No valid data\n";
        }
}


void check_results_carrier_doppler_single_diff(
    arma::mat& measured_ch0,
    arma::mat& measured_ch1,
    const std::string& data_title)
{
    // 1. True value interpolation to match the measurement times
    arma::vec measurement_time = measured_ch0.col(0);

    arma::vec meas_ch1_carrier_doppler_interp;
    arma::interp1(measured_ch1.col(0), measured_ch1.col(2), measurement_time, meas_ch1_carrier_doppler_interp);
    // generate single difference carrier Doppler
    arma::vec delta_measured_carrier_doppler_cycles = measured_ch0.col(2) - meas_ch1_carrier_doppler_interp;

    // remove NaN
    arma::uvec NaN_in_measured_data = arma::find_nonfinite(delta_measured_carrier_doppler_cycles);

    arma::mat tmp_mat = arma::conv_to<arma::mat>::from(delta_measured_carrier_doppler_cycles);
    tmp_mat.shed_rows(NaN_in_measured_data);
    delta_measured_carrier_doppler_cycles = tmp_mat.col(0);

    tmp_mat = arma::conv_to<arma::mat>::from(measurement_time);
    tmp_mat.shed_rows(NaN_in_measured_data);
    measurement_time = tmp_mat.col(0);

    std::vector<double>
        time_vector(measurement_time.colptr(0), measurement_time.colptr(0) + measurement_time.n_rows);

    if (measurement_time.size() > 0)
        {
            // 2. RMSE
            arma::vec err;
            err = delta_measured_carrier_doppler_cycles;
            arma::vec err2 = arma::square(err);
            double rmse = sqrt(arma::mean(err2));

            // 3. Mean err and variance
            double error_mean = arma::mean(err);
            double error_var = arma::var(err);

            // 4. Peaks
            double max_error = arma::max(err);
            double min_error = arma::min(err);

            // 5. report
            std::streamsize ss = std::cout.precision();
            std::cout << std::setprecision(10) << data_title << "Single diff Carrier Doppler RMSE = "
                      << rmse << ", mean = " << error_mean
                      << ", stdev = " << sqrt(error_var)
                      << " (max,min) = " << max_error
                      << "," << min_error
                      << " [Hz]" << std::endl;
            std::cout.precision(ss);

            // plots
            if (FLAGS_show_plots)
                {
                    Gnuplot g3("linespoints");
                    g3.set_title(data_title + "Single diff Carrier Doppler error [Hz]");
                    g3.set_grid();
                    g3.set_xlabel("Time [s]");
                    g3.set_ylabel("Single diff Carrier Doppler error [Hz]");
                    // conversion between arma::vec and std:vector
                    std::vector<double> range_error_m(err.colptr(0), err.colptr(0) + err.n_rows);
                    g3.cmd("set key box opaque");
                    g3.plot_xy(time_vector, range_error_m,
                        "Single diff Carrier Doppler error");
                    g3.set_legend();
                    g3.savetops(data_title + "single_diff_carrier_doppler_error");

                    g3.showonscreen();  // window output
                }
        }
    else
        {
            std::cout << "No valid data\n";
        }
}


void check_results_code_pseudorange_double_diff(
    arma::mat& true_ch0,
    arma::mat& true_ch1,
    arma::mat& measured_ch0,
    arma::mat& measured_ch1,
    const std::string& data_title)
{
    // 1. True value interpolation to match the measurement times
    arma::vec measurement_time = measured_ch0.col(0);
    arma::vec true_ch0_obs_interp;
    arma::vec true_ch1_obs_interp;
    arma::interp1(true_ch0.col(0), true_ch0.col(1), measurement_time, true_ch0_obs_interp);
    arma::interp1(true_ch1.col(0), true_ch1.col(1), measurement_time, true_ch1_obs_interp);

    arma::vec meas_ch1_obs_interp;
    arma::interp1(measured_ch1.col(0), measured_ch1.col(1), measurement_time, meas_ch1_obs_interp);
    // generate double difference carrier Doppler
    arma::vec delta_true_obs = true_ch0_obs_interp - true_ch1_obs_interp;
    arma::vec delta_measured_obs = measured_ch0.col(1) - meas_ch1_obs_interp;

    // remove NaN
    arma::uvec NaN_in_true_data = arma::find_nonfinite(delta_true_obs);
    arma::uvec NaN_in_measured_data = arma::find_nonfinite(delta_measured_obs);

    arma::mat tmp_mat = arma::conv_to<arma::mat>::from(delta_true_obs);
    tmp_mat.shed_rows(arma::join_cols(NaN_in_true_data, NaN_in_measured_data));
    delta_true_obs = tmp_mat.col(0);

    tmp_mat = arma::conv_to<arma::mat>::from(delta_measured_obs);
    tmp_mat.shed_rows(arma::join_cols(NaN_in_true_data, NaN_in_measured_data));
    delta_measured_obs = tmp_mat.col(0);

    tmp_mat = arma::conv_to<arma::mat>::from(measurement_time);
    tmp_mat.shed_rows(arma::join_cols(NaN_in_true_data, NaN_in_measured_data));
    measurement_time = tmp_mat.col(0);

    std::vector<double>
        time_vector(measurement_time.colptr(0), measurement_time.colptr(0) + measurement_time.n_rows);

    if (measurement_time.size() > 0)
        {
            // debug
            //    std::vector<double> tmp_time_vec(measurement_time.colptr(0),
            //        measurement_time.colptr(0) + measurement_time.n_rows);
            //    std::vector<double> tmp_vector_y6(delta_true_carrier_doppler_cycles.colptr(0),
            //        delta_true_carrier_doppler_cycles.colptr(0) + delta_true_carrier_doppler_cycles.n_rows);
            //    save_mat_xy(tmp_time_vec, tmp_vector_y6, std::string("true_delta_doppler"));
            //    std::vector<double> tmp_vector_y7(delta_measured_carrier_doppler_cycles.colptr(0),
            //        delta_measured_carrier_doppler_cycles.colptr(0) + delta_measured_carrier_doppler_cycles.n_rows);
            //    save_mat_xy(tmp_time_vec, tmp_vector_y7, std::string("measured_delta_doppler"));

            // 2. RMSE
            arma::vec err;

            err = delta_measured_obs - delta_true_obs;

            arma::vec err2 = arma::square(err);
            double rmse = sqrt(arma::mean(err2));

            // 3. Mean err and variance
            double error_mean = arma::mean(err);
            double error_var = arma::var(err);

            // 4. Peaks
            double max_error = arma::max(err);
            double min_error = arma::min(err);

            // 5. report
            std::streamsize ss = std::cout.precision();
            std::cout << std::setprecision(10) << data_title << "Double diff Pseudorange RMSE = "
                      << rmse << ", mean = " << error_mean
                      << ", stdev = " << sqrt(error_var)
                      << " (max,min) = " << max_error
                      << "," << min_error
                      << " [meters]" << std::endl;
            std::cout.precision(ss);

            // plots
            if (FLAGS_show_plots)
                {
                    Gnuplot g3("linespoints");
                    g3.set_title(data_title + "Double diff Pseudorange error [m]");
                    g3.set_grid();
                    g3.set_xlabel("Time [s]");
                    g3.set_ylabel("Double diff Pseudorange error [m]");
                    // conversion between arma::vec and std:vector
                    std::vector<double> range_error_m(err.colptr(0), err.colptr(0) + err.n_rows);
                    g3.cmd("set key box opaque");
                    g3.plot_xy(time_vector, range_error_m,
                        "Double diff Pseudorrange error");
                    g3.set_legend();
                    g3.savetops(data_title + "double_diff_pseudorrange_error");

                    g3.showonscreen();  // window output
                }

            // check results against the test tolerance
            //            ASSERT_LT(rmse, 3.0);
            //            ASSERT_LT(error_mean, 1.0);
            //            ASSERT_GT(error_mean, -1.0);
            //            ASSERT_LT(error_var, 10.0);
            //            ASSERT_LT(max_error, 10.0);
            //            ASSERT_GT(min_error, -10.0);
        }
    else
        {
            std::cout << "No valid data\n";
        }
}


void check_results_code_pseudorange_single_diff(
    arma::mat& measured_ch0,
    arma::mat& measured_ch1,
    const std::string& data_title)
{
    // 1. True value interpolation to match the measurement times
    arma::vec measurement_time = measured_ch0.col(0);

    arma::vec meas_ch1_obs_interp;
    arma::interp1(measured_ch1.col(0), measured_ch1.col(1), measurement_time, meas_ch1_obs_interp);
    // generate single difference carrier Doppler
    arma::vec delta_measured_obs = measured_ch0.col(1) - meas_ch1_obs_interp;

    // remove NaN
    arma::uvec NaN_in_measured_data = arma::find_nonfinite(delta_measured_obs);

    arma::mat tmp_mat = arma::conv_to<arma::mat>::from(delta_measured_obs);
    tmp_mat.shed_rows(NaN_in_measured_data);
    delta_measured_obs = tmp_mat.col(0);

    tmp_mat = arma::conv_to<arma::mat>::from(measurement_time);
    tmp_mat.shed_rows(NaN_in_measured_data);
    measurement_time = tmp_mat.col(0);

    std::vector<double>
        time_vector(measurement_time.colptr(0), measurement_time.colptr(0) + measurement_time.n_rows);

    if (measurement_time.size() > 0)
        {
            // 2. RMSE
            arma::vec err;

            err = delta_measured_obs;

            arma::vec err2 = arma::square(err);
            double rmse = sqrt(arma::mean(err2));

            // 3. Mean err and variance
            double error_mean = arma::mean(err);
            double error_var = arma::var(err);

            // 4. Peaks
            double max_error = arma::max(err);
            double min_error = arma::min(err);

            // 5. report
            std::streamsize ss = std::cout.precision();
            std::cout << std::setprecision(10) << data_title << "Single diff Pseudorange RMSE = "
                      << rmse << ", mean = " << error_mean
                      << ", stdev = " << sqrt(error_var)
                      << " (max,min) = " << max_error
                      << "," << min_error
                      << " [meters]" << std::endl;
            std::cout.precision(ss);

            // plots
            if (FLAGS_show_plots)
                {
                    Gnuplot g3("linespoints");
                    g3.set_title(data_title + "Single diff Pseudorange error [m]");
                    g3.set_grid();
                    g3.set_xlabel("Time [s]");
                    g3.set_ylabel("Single diff Pseudorange error [m]");
                    // conversion between arma::vec and std:vector
                    std::vector<double> range_error_m(err.colptr(0), err.colptr(0) + err.n_rows);
                    g3.cmd("set key box opaque");
                    g3.plot_xy(time_vector, range_error_m,
                        "Single diff Pseudorrange error");
                    g3.set_legend();
                    g3.savetops(data_title + "Single_diff_pseudorrange_error");

                    g3.showonscreen();  // window output
                }
        }
    else
        {
            std::cout << "No valid data\n";
        }
}


void RINEX_doublediff_dupli_sat()
{
    // special test mode for duplicated satellites
    // read rinex receiver-under-test observations
    std::map<int, arma::mat> test_obs = ReadRinexObs(FLAGS_test_rinex_obs, 'G', std::string("1C\0"));
    if (test_obs.size() == 0)
        {
            return;
        }
    // Cut measurement initial transitory of the measurements
    double initial_transitory_s = FLAGS_skip_obs_transitory_s;
    std::cout << "Skipping initial transitory of " << initial_transitory_s << " [s]" << std::endl;
    arma::uvec index;
    for (std::map<int, arma::mat>::iterator it = test_obs.begin(); it != test_obs.end(); ++it)
        {
            index = arma::find(it->second.col(0) >= (it->second.col(0)(0) + initial_transitory_s), 1, "first");
            if ((!index.empty()) and (index(0) > 0))
                {
                    it->second.shed_rows(0, index(0));
                }
        }

    std::vector<unsigned int> prn_pairs;
    std::stringstream ss(FLAGS_dupli_sat_prns);
    unsigned int i;
    while (ss >> i)
        {
            prn_pairs.push_back(i);
            if (ss.peek() == ',')
                {
                    ss.ignore();
                }
        }

    if (prn_pairs.size() % 2 != 0)
        {
            std::cout << "Test settings error: duplicated_satellites_prns are even\n";
        }
    else
        {
            for (unsigned int n = 0; n < prn_pairs.size(); n = n + 2)
                {
                    // compute double differences
                    if (test_obs.find(prn_pairs.at(n)) != test_obs.end() and test_obs.find(prn_pairs.at(n + 1)) != test_obs.end())
                        {
                            std::cout << "Computing single difference observables for duplicated SV pairs..." << std::endl;
                            std::cout << "SD = OBS_ROVER(SV" << prn_pairs.at(n) << ") - OBS_ROVER(SV" << prn_pairs.at(n + 1) << ")" << std::endl;

                            check_results_code_pseudorange_single_diff(test_obs.at(prn_pairs.at(n)),
                                test_obs.at(prn_pairs.at(n + 1)),
                                "SD = OBS(SV" + std::to_string(prn_pairs.at(n)) + ") - OBS(SV" + std::to_string(prn_pairs.at(n + 1)) + ") ");

                            check_results_carrier_phase_single_diff(test_obs.at(prn_pairs.at(n)),
                                test_obs.at(prn_pairs.at(n + 1)),
                                "SD = OBS(SV" + std::to_string(prn_pairs.at(n)) + ") - OBS(SV" + std::to_string(prn_pairs.at(n + 1)) + ") ");

                            check_results_carrier_doppler_single_diff(test_obs.at(prn_pairs.at(n)),
                                test_obs.at(prn_pairs.at(n + 1)),
                                "SD = OBS(SV" + std::to_string(prn_pairs.at(n)) + ") - OBS(SV" + std::to_string(prn_pairs.at(n + 1)) + ") ");
                        }
                    else
                        {
                            std::cout << "Satellite ID " << prn_pairs.at(n) << " and/or " << prn_pairs.at(n + 1) << " not found in RINEX file\n";
                        }
                }
        }
}


void RINEX_doublediff()
{
    // read rinex reference observations
    std::map<int, arma::mat> ref_obs = ReadRinexObs(FLAGS_ref_rinex_obs, 'G', std::string("1C\0"));
    // read rinex receiver-under-test observations
    std::map<int, arma::mat> test_obs = ReadRinexObs(FLAGS_test_rinex_obs, 'G', std::string("1C\0"));

    if (ref_obs.size() == 0 or test_obs.size() == 0)
        {
            return;
        }
    // Cut measurement initial transitory of the measurements
    double initial_transitory_s = FLAGS_skip_obs_transitory_s;
    std::cout << "Skipping initial transitory of " << initial_transitory_s << " [s]" << std::endl;
    arma::uvec index;
    for (std::map<int, arma::mat>::iterator it = test_obs.begin(); it != test_obs.end(); ++it)
        {
            index = arma::find(it->second.col(0) >= (it->second.col(0)(0) + initial_transitory_s), 1, "first");
            if ((!index.empty()) and (index(0) > 0))
                {
                    it->second.shed_rows(0, index(0));
                }
        }

    // Cut observation vectors ends to the shortest one (base or rover)
    arma::colvec ref_obs_time = ref_obs.begin()->second.col(0);
    arma::colvec test_obs_time = test_obs.begin()->second.col(0);

    if (ref_obs_time.back() < test_obs_time.back())
        {
            // there are more rover observations than base observations
            // cut rover vector
            std::cout << "Cutting rover observations vector end.." << std::endl;
            arma::uvec index2;
            for (std::map<int, arma::mat>::iterator it = test_obs.begin(); it != test_obs.end(); ++it)
                {
                    index = arma::find(it->second.col(0) >= ref_obs_time.back(), 1, "first");
                    if ((!index.empty()) and (index(0) > 0))
                        {
                            it->second.shed_rows(index(0), it->second.n_rows - 1);
                        }
                }
        }
    else
        {
            // there are more base observations than rover observations
            // cut base vector
            std::cout << "Cutting base observations vector end.." << std::endl;
            for (std::map<int, arma::mat>::iterator it = ref_obs.begin(); it != ref_obs.end(); ++it)
                {
                    index = arma::find(it->second.col(0) >= test_obs_time.back(), 1, "first");
                    if ((!index.empty()) and (index(0) > 0))
                        {
                            it->second.shed_rows(index(0), it->second.n_rows - 1);
                        }
                }
        }

    // also skip last seconds of the observations (some artifacts are present in some RINEX endings)
    ref_obs_time = ref_obs.begin()->second.col(0);
    test_obs_time = test_obs.begin()->second.col(0);

    double skip_ends_s = FLAGS_skip_obs_ends_s;
    std::cout << "Skipping last " << skip_ends_s << " [s] of observations" << std::endl;
    for (std::map<int, arma::mat>::iterator it = test_obs.begin(); it != test_obs.end(); ++it)
        {
            index = arma::find(it->second.col(0) >= (test_obs_time.back() - skip_ends_s), 1, "first");
            if ((!index.empty()) and (index(0) > 0))
                {
                    it->second.shed_rows(index(0), it->second.n_rows - 1);
                }
        }
    for (std::map<int, arma::mat>::iterator it = ref_obs.begin(); it != ref_obs.end(); ++it)
        {
            index = arma::find(it->second.col(0) >= (ref_obs_time.back() - skip_ends_s), 1, "first");
            if ((!index.empty()) and (index(0) > 0))
                {
                    it->second.shed_rows(index(0), it->second.n_rows - 1);
                }
        }

    // Save observations in .mat files
    std::cout << "Saving RAW observables inputs to .mat files...\n";
    for (std::map<int, arma::mat>::iterator it = ref_obs.begin(); it != ref_obs.end(); ++it)
        {
            //            std::cout << it->first << " => " << it->second.n_rows << '\n';
            //            std::cout << it->first << " has NaN values: " << it->second.has_nan() << '\n';
            // debug
            std::vector<double> tmp_time_vec(it->second.col(0).colptr(0),
                it->second.col(0).colptr(0) + it->second.n_rows);
            std::vector<double> tmp_vector_y6(it->second.col(2).colptr(0),
                it->second.col(2).colptr(0) + it->second.n_rows);
            save_mat_xy(tmp_time_vec, tmp_vector_y6, std::string("ref_doppler_sat" + std::to_string(it->first)));
        }
    for (std::map<int, arma::mat>::iterator it = test_obs.begin(); it != test_obs.end(); ++it)
        {
            //            std::cout << it->first << " => " << it->second.n_rows << '\n';
            //            std::cout << it->first << " has NaN values: " << it->second.has_nan() << '\n';
            // debug
            std::vector<double> tmp_time_vec(it->second.col(0).colptr(0),
                it->second.col(0).colptr(0) + it->second.n_rows);
            std::vector<double> tmp_vector_y6(it->second.col(2).colptr(0),
                it->second.col(2).colptr(0) + it->second.n_rows);
            save_mat_xy(tmp_time_vec, tmp_vector_y6, std::string("measured_doppler_sat" + std::to_string(it->first)));
        }

    // select reference satellite
    std::set<int> PRN_set = available_gps_prn;
    double abs_min_doppler = 1e6;
    int ref_sat_id = 1;
    for (std::set<int>::iterator ref_prn_it = PRN_set.begin(); ref_prn_it != PRN_set.end(); ++ref_prn_it)
        {
            if (ref_obs.find(*ref_prn_it) != ref_obs.end() and test_obs.find(*ref_prn_it) != test_obs.end())
                {
                    if (fabs(test_obs.at(*ref_prn_it).at(0, 2)) < abs_min_doppler)
                        {
                            abs_min_doppler = fabs(test_obs.at(*ref_prn_it).at(0, 2));
                            ref_sat_id = *ref_prn_it;
                        }
                }
        }

    // compute double differences
    if (ref_obs.find(ref_sat_id) != ref_obs.end() and test_obs.find(ref_sat_id) != test_obs.end())
        {
            std::cout << "Using reference satellite SV " << ref_sat_id << " with minimum abs Doppler of " << abs_min_doppler << " [Hz]" << std::endl;
            for (std::set<int>::iterator current_prn_it = PRN_set.begin(); current_prn_it != PRN_set.end(); ++current_prn_it)
                {
                    int current_sat_id = *current_prn_it;
                    if (current_sat_id != ref_sat_id)
                        {
                            if (ref_obs.find(current_sat_id) != ref_obs.end() and test_obs.find(current_sat_id) != test_obs.end())
                                {
                                    std::cout << "Computing double difference observables for SV " << current_sat_id << std::endl;
                                    std::cout << "DD = (OBS_ROVER(SV" << current_sat_id << ") - OBS_ROVER(SV" << ref_sat_id << "))"
                                              << " - (OBS_BASE(SV" << current_sat_id << ") - OBS_BASE(SV" << ref_sat_id << "))" << std::endl;

                                    check_results_code_pseudorange_double_diff(ref_obs.at(ref_sat_id),
                                        ref_obs.at(current_sat_id),
                                        test_obs.at(ref_sat_id),
                                        test_obs.at(current_sat_id),
                                        "PRN " + std::to_string(current_sat_id) + " ");

                                    check_results_carrier_phase_double_diff(ref_obs.at(ref_sat_id),
                                        ref_obs.at(current_sat_id),
                                        test_obs.at(ref_sat_id),
                                        test_obs.at(current_sat_id),
                                        "PRN " + std::to_string(current_sat_id) + " ");

                                    check_results_carrier_doppler_double_diff(ref_obs.at(ref_sat_id),
                                        ref_obs.at(current_sat_id),
                                        test_obs.at(ref_sat_id),
                                        test_obs.at(current_sat_id),
                                        "PRN " + std::to_string(current_sat_id) + " ");
                                }
                        }
                }
        }
    else
        {
            std::cout << "Satellite ID " << ref_sat_id << " not found in both RINEX files\n";
        }
}


int main(int argc, char** argv)
{
    std::cout << "Running RINEX observables difference tool..." << std::endl;
    google::ParseCommandLineFlags(&argc, &argv, true);
    if (FLAGS_dupli_sat)
        {
            RINEX_doublediff_dupli_sat();
        }
    else
        {
            RINEX_doublediff();
        }
    google::ShutDownCommandLineFlags();
    return 0;
}
