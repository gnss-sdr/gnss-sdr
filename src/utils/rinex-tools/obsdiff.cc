/*!
 * \file obsdiff.cc
 * \brief This program implements a single difference and double difference
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

#include "gnuplot_i.h"
#include "obsdiff_flags.h"
#include <armadillo>
// Classes for handling observations RINEX files (data)
#include <gpstk/Rinex3ObsData.hpp>
#include <gpstk/Rinex3ObsHeader.hpp>
#include <gpstk/Rinex3ObsStream.hpp>

// Classes for handling satellite navigation parameters RINEX
// files (ephemerides)
#include <gpstk/Rinex3NavData.hpp>
#include <gpstk/Rinex3NavHeader.hpp>
#include <gpstk/Rinex3NavStream.hpp>

// Classes for handling RINEX files with meteorological parameters
#include <gpstk/RinexMetBase.hpp>
#include <gpstk/RinexMetData.hpp>
#include <gpstk/RinexMetHeader.hpp>
#include <gpstk/RinexMetStream.hpp>

// Class for handling tropospheric model
#include <gpstk/GGTropModel.hpp>

// Class for storing >broadcast-type> ephemerides
#include <gpstk/GPSEphemerisStore.hpp>

// Class for handling RAIM
#include <gpstk/PRSolution.hpp>
#include <gpstk/PRSolution2.hpp>

// Class defining GPS system constants
#include <gpstk/GNSSconstants.hpp>
#include <matio.h>
#include <array>
#include <fstream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>


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


void carrier_phase_double_diff(
    arma::mat& true_ch0,
    arma::mat& true_ch1,
    arma::mat& measured_ch0,
    arma::mat& measured_ch1,
    const std::string& data_title, double common_rx_clock_error_s)
{
    // 1. True value interpolation to match the measurement times
    arma::vec measurement_time = measured_ch0.col(0);

    arma::vec true_ch0_carrier_phase_interp;
    arma::vec true_ch1_carrier_phase_interp;
    //interpolate measured observables accounting with the receiver clock differences
    arma::interp1(true_ch0.col(0), true_ch0.col(3), measurement_time - common_rx_clock_error_s, true_ch0_carrier_phase_interp);
    arma::interp1(true_ch1.col(0), true_ch1.col(3), measurement_time - common_rx_clock_error_s, true_ch1_carrier_phase_interp);

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
        }
    else
        {
            std::cout << "No valid data\n";
        }
}


void carrier_phase_single_diff(
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


void carrier_doppler_double_diff(
    arma::mat& true_ch0,
    arma::mat& true_ch1,
    arma::mat& measured_ch0,
    arma::mat& measured_ch1,
    const std::string& data_title, double common_rx_clock_error_s)
{
    // 1. True value interpolation to match the measurement times
    arma::vec measurement_time = measured_ch0.col(0);

    arma::vec true_ch0_carrier_doppler_interp;
    arma::vec true_ch1_carrier_doppler_interp;
    //interpolate measured observables accounting with the receiver clock differences
    arma::interp1(true_ch0.col(0), true_ch0.col(2), measurement_time - common_rx_clock_error_s, true_ch0_carrier_doppler_interp);
    arma::interp1(true_ch1.col(0), true_ch1.col(2), measurement_time - common_rx_clock_error_s, true_ch1_carrier_doppler_interp);

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
        }
    else
        {
            std::cout << "No valid data\n";
        }
}


void carrier_doppler_single_diff(
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


void code_pseudorange_double_diff(
    arma::mat& true_ch0,
    arma::mat& true_ch1,
    arma::mat& measured_ch0,
    arma::mat& measured_ch1,
    const std::string& data_title, double common_rx_clock_error_s)
{
    // 1. True value interpolation to match the measurement times
    arma::vec measurement_time = measured_ch0.col(0);
    arma::vec true_ch0_obs_interp;
    arma::vec true_ch1_obs_interp;
    //interpolate measured observables accounting with the receiver clock differences
    arma::interp1(true_ch0.col(0), true_ch0.col(1), measurement_time - common_rx_clock_error_s, true_ch0_obs_interp);
    arma::interp1(true_ch1.col(0), true_ch1.col(1), measurement_time - common_rx_clock_error_s, true_ch1_obs_interp);

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
        }
    else
        {
            std::cout << "No valid data\n";
        }
}


void code_pseudorange_single_diff(
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


double compute_rx_clock_error(std::string rinex_nav_filename, std::string rinex_obs_file)
{
    std::cout << "Computing receiver's clock error..." << std::endl;
    if (not file_exist(rinex_nav_filename.c_str()))
        {
            std::cout << "Warning: RINEX Nav file " << rinex_nav_filename << " does not exist, receiver's clock error could not be computed!\n";
            return 0.0;
        }
    // Declaration of objects for storing ephemerides and handling RAIM
    gpstk::GPSEphemerisStore bcestore;
    gpstk::PRSolution raimSolver;

    // Object for void-type tropospheric model (in case no meteorological
    // RINEX is available)
    gpstk::ZeroTropModel noTropModel;

    // Object for GG-type tropospheric model (Goad and Goodman, 1974)
    // Default constructor => default values for model
    gpstk::GGTropModel ggTropModel;

    // Pointer to one of the two available tropospheric models. It points
    // to the void model by default
    gpstk::TropModel* tropModelPtr = &noTropModel;

    double rx_clock_error_s = 0.0;
    try
        {
            // Read nav file and store unique list of ephemerides
            gpstk::Rinex3NavStream rnffs(rinex_nav_filename.c_str());  // Open ephemerides data file
            gpstk::Rinex3NavData rne;
            gpstk::Rinex3NavHeader hdr;

            // Let's read the header (may be skipped)
            rnffs >> hdr;

            // Storing the ephemeris in "bcstore"
            while (rnffs >> rne) bcestore.addEphemeris(rne);

            // Setting the criteria for looking up ephemeris
            bcestore.SearchNear();

            // Open and read the observation file one epoch at a time.
            // For each epoch, compute and print a position solution
            gpstk::Rinex3ObsStream roffs(rinex_obs_file.c_str());  // Open observations data file

            // In order to throw exceptions, it is necessary to set the failbit
            roffs.exceptions(std::ios::failbit);

            gpstk::Rinex3ObsHeader roh;
            gpstk::Rinex3ObsData rod;

            // Let's read the header
            roffs >> roh;

            int indexC1;
            try
                {
                    indexC1 = roh.getObsIndex("C1");
                }
            catch (...)
                {
                    std::cerr << "The observation file doesn't have C1 pseudoranges, RX clock error could not be computed!" << std::endl;
                    return (0.0);
                }

            // Let's process all lines of observation data, one by one
            while (roffs >> rod)
                {
                    // Apply editing criteria
                    if (rod.epochFlag == 0 || rod.epochFlag == 1)  // Begin usable data
                        {
                            std::vector<gpstk::SatID> prnVec;
                            std::vector<double> rangeVec;

                            // Define the "it" iterator to visit the observations PRN map.
                            // Rinex3ObsData::DataMap is a map from RinexSatID to
                            // vector<RinexDatum>:
                            //      std::map<RinexSatID, vector<RinexDatum> >
                            gpstk::Rinex3ObsData::DataMap::const_iterator it;

                            // This part gets the PRN numbers and ionosphere-corrected
                            // pseudoranges for the current epoch. They are correspondly fed
                            // into "prnVec" and "rangeVec"; "obs" is a public attribute of
                            // Rinex3ObsData to get the map of observations
                            for (it = rod.obs.begin(); it != rod.obs.end(); it++)
                                {
                                    // The RINEX file may have P1 observations, but the current
                                    // satellite may not have them.
                                    double C1(0.0);
                                    try
                                        {
                                            C1 = rod.getObs((*it).first, indexC1).data;
                                        }
                                    catch (...)
                                        {
                                            // Ignore this satellite if P1 is not found
                                            continue;
                                        }
                                    // Now, we include the current PRN number in the first part
                                    // of "it" iterator into the vector holding the satellites.
                                    // All satellites in view at this epoch that have P1 or P1+P2
                                    // observations will be included.
                                    prnVec.push_back((*it).first);

                                    // The same is done for the vector of doubles holding the
                                    // corrected ranges
                                    rangeVec.push_back(C1);

                                    // WARNING: Please note that so far no further correction
                                    // is done on data: Relativistic effects, tropospheric
                                    // correction, instrumental delays, etc.

                                }  // End of 'for( it = rod.obs.begin(); it!= rod.obs.end(); ...'

                            // The default constructor for PRSolution2 objects (like
                            // "raimSolver") is to set a RMSLimit of 6.5. We change that
                            // here. With this value of 3e6 the solution will have a lot
                            // more dispersion.
                            raimSolver.RMSLimit = 3e6;

                            // In order to compute positions we need the current time, the
                            // vector of visible satellites, the vector of corresponding
                            // ranges, the object containing satellite ephemerides, and a
                            // pointer to the tropospheric model to be applied

                            try
                                {
                                    std::vector<gpstk::SatID::SatelliteSystem> Syss;
                                    gpstk::Matrix<double> invMC;
                                    int iret;
                                    // Call RAIMCompute.

                                    iret = raimSolver.RAIMCompute(rod.time, prnVec, Syss, rangeVec, invMC,
                                        &bcestore, tropModelPtr);
                                    switch (iret)
                                        {
                                        /// @return Return values:
                                        ///  1  solution is ok, but may be degraded; check TropFlag,
                                        ///  RMSFlag, SlopeFlag 0  ok
                                        case -1:

                                            std::cout << " algorithm failed to converge at time: " << rod.time
                                                      << " \n";
                                            break;
                                        case -2:
                                            std::cout
                                                << " singular problem, no solution is possible at time: "
                                                << rod.time << " \n";
                                            break;
                                        case -3:
                                            std::cout << " not enough good data (> 4) to form a (RAIM) "
                                                         "solution at time: "
                                                      << rod.time << " \n";
                                            break;
                                        case -4:
                                            std::cout
                                                << "ephemeris not found for all the satellites at time: "
                                                << rod.time << " \n";
                                            break;
                                        default:
                                            break;
                                        }

                                    // return iret;
                                }
                            catch (gpstk::Exception& e)
                                {
                                }

                            // If we got a valid solution, let's print it

                            if (raimSolver.isValid())
                                {
                                    std::cout << "RX POS ECEF [XYZ] " << std::fixed << std::setprecision(3) << " "
                                              << std::setw(12) << raimSolver.Solution(0) << " "
                                              << std::setw(12) << raimSolver.Solution(1) << " "
                                              << std::setw(12) << raimSolver.Solution(2) << "\n";

                                    std::cout << "RX CLK " << std::fixed << std::setprecision(16)
                                              << raimSolver.Solution(3) / gpstk::C_MPS << " [s] \n";

                                    std::cout << "NSATS, DOPs " << std::setw(2) << raimSolver.Nsvs << std::fixed
                                              << std::setprecision(2) << " "
                                              << std::setw(4) << raimSolver.PDOP << " " << std::setw(4) << raimSolver.GDOP
                                              << " " << std::setw(8) << raimSolver.RMSResidual << "\n";
                                    gpstk::Position rx_pos;
                                    rx_pos.setECEF(gpstk::Triple(raimSolver.Solution(0), raimSolver.Solution(1), raimSolver.Solution(2)));

                                    double lat_deg = rx_pos.geodeticLatitude();
                                    double lon_deg = rx_pos.longitude();
                                    double Alt_m = rx_pos.getAltitude();
                                    std::cout << "RX POS GEO [Lat,Long,H]" << std::fixed << std::setprecision(10) << " "
                                              << std::setw(12) << lat_deg << ","
                                              << std::setw(12) << lon_deg << ","
                                              << std::setw(12) << Alt_m << " [deg],[deg],[m]\n";

                                    //set computed RX clock error and stop iterating obs epochs
                                    rx_clock_error_s = raimSolver.Solution(3) / gpstk::C_MPS;
                                    break;
                                }  // End of 'if( raimSolver.isValid() )'

                        }  // End of 'if( rod.epochFlag == 0 || rod.epochFlag == 1 )'

                }  // End of 'while( roffs >> rod )'
        }
    catch (gpstk::Exception& e)
        {
            std::cout << "GPSTK exception: " << e << std::endl;
        }
    catch (...)
        {
            std::cout << "Caught an unexpected exception." << std::endl;
        }

    return rx_clock_error_s;
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

                            code_pseudorange_single_diff(test_obs.at(prn_pairs.at(n)),
                                test_obs.at(prn_pairs.at(n + 1)),
                                "SD = OBS(SV" + std::to_string(prn_pairs.at(n)) + ") - OBS(SV" + std::to_string(prn_pairs.at(n + 1)) + ") ");

                            carrier_phase_single_diff(test_obs.at(prn_pairs.at(n)),
                                test_obs.at(prn_pairs.at(n + 1)),
                                "SD = OBS(SV" + std::to_string(prn_pairs.at(n)) + ") - OBS(SV" + std::to_string(prn_pairs.at(n + 1)) + ") ");

                            carrier_doppler_single_diff(test_obs.at(prn_pairs.at(n)),
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


void RINEX_doublediff(bool remove_rx_clock_error)
{
    // read rinex reference observations
    std::map<int, arma::mat> ref_obs = ReadRinexObs(FLAGS_ref_rinex_obs, 'G', std::string("1C\0"));
    // read rinex receiver-under-test observations
    std::map<int, arma::mat> test_obs = ReadRinexObs(FLAGS_test_rinex_obs, 'G', std::string("1C\0"));

    if (ref_obs.size() == 0 or test_obs.size() == 0)
        {
            return;
        }

    //compute rx clock errors
    double ref_rx_clock_error_s = 0.0;
    double test_rx_clock_error_s = 0.0;
    if (remove_rx_clock_error == true)
        {
            ref_rx_clock_error_s = compute_rx_clock_error(FLAGS_rinex_nav, FLAGS_ref_rinex_obs);
            test_rx_clock_error_s = compute_rx_clock_error(FLAGS_rinex_nav, FLAGS_test_rinex_obs);
        }

    double common_clock_error_s = test_rx_clock_error_s - ref_rx_clock_error_s;


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
            std::vector<double> tmp_time_vec(it->second.col(0).colptr(0),
                it->second.col(0).colptr(0) + it->second.n_rows);
            std::vector<double> tmp_vector(it->second.col(2).colptr(0),
                it->second.col(2).colptr(0) + it->second.n_rows);
            save_mat_xy(tmp_time_vec, tmp_vector, std::string("ref_doppler_sat" + std::to_string(it->first)));

            std::vector<double> tmp_vector2(it->second.col(3).colptr(0),
                it->second.col(3).colptr(0) + it->second.n_rows);
            save_mat_xy(tmp_time_vec, tmp_vector2, std::string("ref_carrier_phase_sat" + std::to_string(it->first)));

            std::vector<double> tmp_vector3(it->second.col(1).colptr(0),
                it->second.col(1).colptr(0) + it->second.n_rows);
            save_mat_xy(tmp_time_vec, tmp_vector3, std::string("ref_pseudorange_sat" + std::to_string(it->first)));
        }
    for (std::map<int, arma::mat>::iterator it = test_obs.begin(); it != test_obs.end(); ++it)
        {
            //            std::cout << it->first << " => " << it->second.n_rows << '\n';
            //            std::cout << it->first << " has NaN values: " << it->second.has_nan() << '\n';
            std::vector<double> tmp_time_vec(it->second.col(0).colptr(0),
                it->second.col(0).colptr(0) + it->second.n_rows);
            std::vector<double> tmp_vector(it->second.col(2).colptr(0),
                it->second.col(2).colptr(0) + it->second.n_rows);
            save_mat_xy(tmp_time_vec, tmp_vector, std::string("measured_doppler_sat" + std::to_string(it->first)));

            std::vector<double> tmp_vector2(it->second.col(3).colptr(0),
                it->second.col(3).colptr(0) + it->second.n_rows);
            save_mat_xy(tmp_time_vec, tmp_vector2, std::string("measured_carrier_phase_sat" + std::to_string(it->first)));

            std::vector<double> tmp_vector3(it->second.col(1).colptr(0),
                it->second.col(1).colptr(0) + it->second.n_rows);
            save_mat_xy(tmp_time_vec, tmp_vector3, std::string("measured_pseudorange_sat" + std::to_string(it->first)));
        }

    // select reference satellite
    std::set<int> PRN_set = available_gps_prn;
    double min_range = std::numeric_limits<double>::max();
    int ref_sat_id = 1;
    for (std::set<int>::iterator ref_prn_it = PRN_set.begin(); ref_prn_it != PRN_set.end(); ++ref_prn_it)
        {
            if (ref_obs.find(*ref_prn_it) != ref_obs.end() and test_obs.find(*ref_prn_it) != test_obs.end())
                {
                    if (test_obs.at(*ref_prn_it).at(0, 1) < min_range)
                        {
                            min_range = test_obs.at(*ref_prn_it).at(0, 1);
                            ref_sat_id = *ref_prn_it;
                        }
                }
        }

    // compute double differences
    if (ref_obs.find(ref_sat_id) != ref_obs.end() and test_obs.find(ref_sat_id) != test_obs.end())
        {
            std::cout << "Using reference satellite SV " << ref_sat_id << " with minimum range of " << min_range << " [meters]" << std::endl;
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

                                    code_pseudorange_double_diff(ref_obs.at(ref_sat_id),
                                        ref_obs.at(current_sat_id),
                                        test_obs.at(ref_sat_id),
                                        test_obs.at(current_sat_id),
                                        "PRN " + std::to_string(current_sat_id) + " ", common_clock_error_s);

                                    carrier_phase_double_diff(ref_obs.at(ref_sat_id),
                                        ref_obs.at(current_sat_id),
                                        test_obs.at(ref_sat_id),
                                        test_obs.at(current_sat_id),
                                        "PRN " + std::to_string(current_sat_id) + " ", common_clock_error_s);

                                    carrier_doppler_double_diff(ref_obs.at(ref_sat_id),
                                        ref_obs.at(current_sat_id),
                                        test_obs.at(ref_sat_id),
                                        test_obs.at(current_sat_id),
                                        "PRN " + std::to_string(current_sat_id) + " ", common_clock_error_s);
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
            RINEX_doublediff(FLAGS_remove_rx_clock_error);
        }

    google::ShutDownCommandLineFlags();
    return 0;
}
