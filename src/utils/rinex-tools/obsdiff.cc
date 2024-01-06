/*!
 * \file obsdiff.cc
 * \brief This program implements a single difference and double difference
 * comparison algorithm to evaluate receiver's performance at observable level
 * \authors <ul>
 *          <li> Javier Arribas, 2020. jarribas(at)cttc.es
 *          </ul>
 *
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

#include "gnuplot_i.h"
#include "obsdiff_flags.h"
#include <armadillo>
#include <matio.h>
#include <algorithm>
#include <array>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#if GNSSTK_USES_GPSTK_NAMESPACE
#include <gpstk/GGTropModel.hpp>
#include <gpstk/GNSSconstants.hpp>
#include <gpstk/GPSEphemerisStore.hpp>
#include <gpstk/GPSWeekSecond.hpp>
#include <gpstk/PRSolution.hpp>
#include <gpstk/Rinex3NavData.hpp>
#include <gpstk/Rinex3NavHeader.hpp>
#include <gpstk/Rinex3NavStream.hpp>
#include <gpstk/Rinex3ObsData.hpp>
#include <gpstk/Rinex3ObsHeader.hpp>
#include <gpstk/Rinex3ObsStream.hpp>
#include <gpstk/RinexMetBase.hpp>
#include <gpstk/RinexMetData.hpp>
#include <gpstk/RinexMetHeader.hpp>
#include <gpstk/RinexMetStream.hpp>
namespace gnsstk = gpstk;
#else
// Classes for handling observations RINEX files (data)
#include <gnsstk/Rinex3ObsData.hpp>
#include <gnsstk/Rinex3ObsHeader.hpp>
#include <gnsstk/Rinex3ObsStream.hpp>

// Classes for handling RINEX files with meteorological parameters
#include <gnsstk/RinexMetBase.hpp>
#include <gnsstk/RinexMetData.hpp>
#include <gnsstk/RinexMetHeader.hpp>
#include <gnsstk/RinexMetStream.hpp>

// Class for handling tropospheric model
#include <gnsstk/GGTropModel.hpp>
#include <gnsstk/GNSSconstants.hpp>
#include <gnsstk/PRSolution.hpp>

// Class for storing <broadcast-type> ephemeris
#include <gnsstk/GPSWeekSecond.hpp>
#if GNSSTK_OLDER_THAN_13
#include <gnsstk/GPSEphemerisStore.hpp>
#include <gnsstk/Rinex3NavData.hpp>
#include <gnsstk/Rinex3NavHeader.hpp>
#include <gnsstk/Rinex3NavStream.hpp>
#else
#include <gnsstk/MultiFormatNavDataFactory.hpp>
#include <gnsstk/NavLibrary.hpp>
#endif
#endif

#if GFLAGS_OLD_NAMESPACE
namespace gflags
{
using namespace google;
}
#endif

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


std::map<int, arma::mat> ReadRinexObs(const std::string& rinex_file, char system, const std::string& signal)
{
    std::map<int, arma::mat> obs_map;
    if (not file_exist(rinex_file.c_str()))
        {
            std::cout << "Warning: RINEX Obs file " << rinex_file << " does not exist\n";
            return obs_map;
        }
    // Open and read _baseerence RINEX observables file
    try
        {
            gnsstk::Rinex3ObsStream r_base(rinex_file);

            gnsstk::Rinex3ObsData r_base_data;
            gnsstk::Rinex3ObsHeader r_base_header;

            gnsstk::RinexDatum dataobj;

            r_base >> r_base_header;

            std::set<int> PRN_set;
            gnsstk::SatID prn;
            switch (system)
                {
                case 'G':
#if OLD_GPSTK
                    prn.system = gnsstk::SatID::systemGPS;
#else
                    prn.system = gnsstk::SatelliteSystem::GPS;
#endif
                    PRN_set = available_gps_prn;
                    break;
                case 'E':
#if OLD_GPSTK
                    prn.system = gnsstk::SatID::systemGalileo;
#else
                    prn.system = gnsstk::SatelliteSystem::Galileo;
#endif
                    PRN_set = available_galileo_prn;
                    break;
                default:
#if OLD_GPSTK
                    prn.system = gnsstk::SatID::systemGPS;
#else
                    prn.system = gnsstk::SatelliteSystem::GPS;
#endif
                    PRN_set = available_gps_prn;
                }

            std::cout << "Reading RINEX OBS file " << rinex_file << " ...\n";
            while (r_base >> r_base_data)
                {
                    for (const auto& prn_it : PRN_set)
                        {
                            prn.id = prn_it;
                            gnsstk::CommonTime time = r_base_data.time;

#if GNSSTK_OLDER_THAN_9
                            double sow(static_cast<gnsstk::GPSWeekSecond>(time).sow);
#else
                            gnsstk::GPSWeekSecond gws(time);
                            double sow(gws.getSOW());
#endif

                            auto pointer = r_base_data.obs.find(prn);

                            if (pointer != r_base_data.obs.end())
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
                                            dataobj = r_base_data.getObs(prn, "C1C", r_base_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 1) = dataobj.data;  // C1C P1 (psudorange L1)
                                            dataobj = r_base_data.getObs(prn, "D1C", r_base_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 2) = dataobj.data;  // D1C Carrier Doppler
                                            dataobj = r_base_data.getObs(prn, "L1C", r_base_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 3) = dataobj.data;  // L1C Carrier Phase
                                        }
                                    else if (strcmp("1B\0", signal.c_str()) == 0)
                                        {
                                            obs_mat.at(obs_mat.n_rows - 1, 0) = sow;
                                            dataobj = r_base_data.getObs(prn, "C1B", r_base_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 1) = dataobj.data;
                                            dataobj = r_base_data.getObs(prn, "D1B", r_base_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 2) = dataobj.data;
                                            dataobj = r_base_data.getObs(prn, "L1B", r_base_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 3) = dataobj.data;
                                        }
                                    else if (strcmp("2S\0", signal.c_str()) == 0)  // L2M
                                        {
                                            obs_mat.at(obs_mat.n_rows - 1, 0) = sow;
                                            dataobj = r_base_data.getObs(prn, "C2S", r_base_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 1) = dataobj.data;
                                            dataobj = r_base_data.getObs(prn, "D2S", r_base_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 2) = dataobj.data;
                                            dataobj = r_base_data.getObs(prn, "L2S", r_base_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 3) = dataobj.data;
                                        }
                                    else if (strcmp("L5\0", signal.c_str()) == 0)
                                        {
                                            obs_mat.at(obs_mat.n_rows - 1, 0) = sow;
                                            dataobj = r_base_data.getObs(prn, "C5I", r_base_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 1) = dataobj.data;
                                            dataobj = r_base_data.getObs(prn, "D5I", r_base_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 2) = dataobj.data;
                                            dataobj = r_base_data.getObs(prn, "L5I", r_base_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 3) = dataobj.data;
                                        }
                                    else if (strcmp("5X\0", signal.c_str()) == 0)  // Simulator gives RINEX with E5a+E5b. Doppler and accumulated Carrier phase WILL differ
                                        {
                                            obs_mat.at(obs_mat.n_rows - 1, 0) = sow;
                                            dataobj = r_base_data.getObs(prn, "C8I", r_base_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 1) = dataobj.data;
                                            dataobj = r_base_data.getObs(prn, "D8I", r_base_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 2) = dataobj.data;
                                            dataobj = r_base_data.getObs(prn, "L8I", r_base_header);
                                            obs_mat.at(obs_mat.n_rows - 1, 3) = dataobj.data;
                                        }
                                    else
                                        {
                                            std::cout << "ReadRinexObs unknown signal requested: " << signal << '\n';
                                            return obs_map;
                                        }
                                }
                        }
                }  // end while
        }          // End of 'try' block
    catch (const gnsstk::FFStreamError& e)
        {
            std::cout << e;
            return obs_map;
        }
    catch (const gnsstk::Exception& e)
        {
            std::cout << e;
            return obs_map;
        }
    catch (const std::exception& e)
        {
            std::cout << "Exception: " << e.what();
            std::cout << "unknown error.  I don't feel so well...\n";
            return obs_map;
        }
    if (obs_map.empty())
        {
            std::cout << "Warning: file "
                      << rinex_file
                      << " contains no data.\n";
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
            // std::cout << "save_mat_xy write " << filename << '\n';
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
                    std::cout << "save_mat_xy: error creating file\n";
                }
            Mat_Close(matfp);
            return true;
        }
    catch (const std::exception& ex)
        {
            std::cout << "save_mat_xy: " << ex.what() << '\n';
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
            std::cout << "save_mat_x write " << filename << '\n';
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
                    std::cout << "save_mat_x: error creating file\n";
                }
            Mat_Close(matfp);
            return true;
        }
    catch (const std::exception& ex)
        {
            std::cout << "save_mat_x: " << ex.what() << '\n';
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
    // interpolate measured observables accounting with the receiver clock differences
    arma::interp1(true_ch0.col(0), true_ch0.col(3), measurement_time - common_rx_clock_error_s, true_ch0_carrier_phase_interp);
    arma::interp1(true_ch1.col(0), true_ch1.col(3), measurement_time - common_rx_clock_error_s, true_ch1_carrier_phase_interp);

    arma::vec meas_ch1_carrier_phase_interp;
    arma::interp1(measured_ch1.col(0), measured_ch1.col(3), measurement_time, meas_ch1_carrier_phase_interp);
    // generate double difference accumulated carrier phases

    arma::vec delta_true_carrier_phase_cycles = true_ch0_carrier_phase_interp - true_ch1_carrier_phase_interp;
    arma::vec delta_measured_carrier_phase_cycles = measured_ch0.col(3) - meas_ch1_carrier_phase_interp;

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

    if (!measurement_time.empty())
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

            // compute error without the accumulated carrier phase offsets (which depends on the receiver starting time)
            err = err - arma::mean(err);

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
                      << " [Cycles]\n";
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
                    std::string data_title_aux = data_title;
                    std::replace(data_title_aux.begin(), data_title_aux.end(), ' ', '_');
                    std::replace(data_title_aux.begin(), data_title_aux.end(), '(', '_');
                    std::replace(data_title_aux.begin(), data_title_aux.end(), ')', '_');
                    g3.savetops(data_title_aux + "double_diff_carrier_phase_error");

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
    arma::vec delta_measured_carrier_phase_cycles = measured_ch0.col(3) - meas_ch1_carrier_phase_interp;
    delta_measured_carrier_phase_cycles = delta_measured_carrier_phase_cycles - arma::mean(delta_measured_carrier_phase_cycles);
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

    if (!measurement_time.empty())
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
                      << " [Cycles]\n";
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
                    std::string data_title_aux = data_title;
                    std::replace(data_title_aux.begin(), data_title_aux.end(), ' ', '_');
                    std::replace(data_title_aux.begin(), data_title_aux.end(), '(', '_');
                    std::replace(data_title_aux.begin(), data_title_aux.end(), ')', '_');
                    g3.savetops(data_title_aux + "single_diff_carrier_phase_error");

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
    // interpolate measured observables accounting with the receiver clock differences
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

    if (!measurement_time.empty())
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
                      << " [Hz]\n";
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
                    std::string data_title_aux = data_title;
                    std::replace(data_title_aux.begin(), data_title_aux.end(), ' ', '_');
                    std::replace(data_title_aux.begin(), data_title_aux.end(), '(', '_');
                    std::replace(data_title_aux.begin(), data_title_aux.end(), ')', '_');
                    g3.savetops(data_title_aux + "double_diff_carrier_doppler_error");

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

    if (!measurement_time.empty())
        {
            // 2. RMSE
            arma::vec err;
            err = std::move(delta_measured_carrier_doppler_cycles);
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
                      << " [Hz]\n";
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
                    std::string data_title_aux = data_title;
                    std::replace(data_title_aux.begin(), data_title_aux.end(), ' ', '_');
                    std::replace(data_title_aux.begin(), data_title_aux.end(), '(', '_');
                    std::replace(data_title_aux.begin(), data_title_aux.end(), ')', '_');
                    g3.savetops(data_title_aux + "single_diff_carrier_doppler_error");

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
    // interpolate measured observables accounting with the receiver clock differences
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

    if (!measurement_time.empty())
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
                      << " [meters]\n";
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
                        "Double diff Pseudorange error");
                    g3.set_legend();
                    std::string data_title_aux = data_title;
                    std::replace(data_title_aux.begin(), data_title_aux.end(), ' ', '_');
                    std::replace(data_title_aux.begin(), data_title_aux.end(), '(', '_');
                    std::replace(data_title_aux.begin(), data_title_aux.end(), ')', '_');
                    g3.savetops(data_title_aux + "double_diff_pseudorange_error");

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

    if (!measurement_time.empty())
        {
            // 2. RMSE
            arma::vec err;

            err = std::move(delta_measured_obs);

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
                      << " [meters]\n";
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
                        "Single diff Pseudorange error");
                    g3.set_legend();
                    std::string data_title_aux = data_title;
                    std::replace(data_title_aux.begin(), data_title_aux.end(), ' ', '_');
                    std::replace(data_title_aux.begin(), data_title_aux.end(), '(', '_');
                    std::replace(data_title_aux.begin(), data_title_aux.end(), ')', '_');
                    g3.savetops(data_title_aux + "single_diff_pseudorange_error");

                    g3.showonscreen();  // window output
                }
        }
    else
        {
            std::cout << "No valid data\n";
        }
}


void coderate_phaserate_consistence(
    arma::mat& measured_ch0,
    const std::string& data_title)
{
    arma::vec measurement_time = measured_ch0.col(0);
    arma::vec delta_time = measurement_time.subvec(1, measurement_time.n_elem - 1) - measurement_time.subvec(0, measurement_time.n_elem - 2);

    // Test 4 is for the pseudorange phase consistency
    //
    // 1) Checks for the value of the pseudoranges to be within a certain threshold.
    arma::vec prange = measured_ch0.col(1);

    // todo: This code is only valid for L1/E1 carrier frequency.
    arma::vec phase = measured_ch0.col(3) * (gnsstk::C_MPS / gnsstk::L1_FREQ_GPS);

    double mincodeval = 5000000.0;
    double maxcodeval = 40000000.0;
    arma::uvec idx = arma::find(prange < mincodeval);
    if (idx.n_elem > 0)
        {
            std::cout << "Warning: Pseudorange measurement is less than minimum acceptable value of " << mincodeval << " meters.\n";
        }

    idx = arma::find(prange > maxcodeval);
    if (idx.n_elem > 0)
        {
            std::cout << "Warning: Pseudorange measurement is above than maximum acceptable value of " << maxcodeval << " meters.\n";
        }

    // 2) It checks that the pseduorange rate is within a certain threshold
    // check code rate
    arma::vec coderate = (prange.subvec(1, prange.n_elem - 1) - prange.subvec(0, prange.n_elem - 2)) / delta_time;

    // remove NaN
    arma::uvec NaN_in_measured_data = arma::find_nonfinite(coderate);

    if (NaN_in_measured_data.n_elem > 0)
        {
            std::cout << "Warning: Pseudorange rate have NaN values. \n";
        }

    double mincoderate = 0.001;
    double maxcoderate = 5000.0;

    idx = arma::find(coderate > maxcoderate and coderate < mincoderate);
    if (idx.n_elem > 0)
        {
            std::cout << "Warning: bad code rate \n";
        }

    // 3) It checks that the phase rate is within a certain threshold
    arma::vec phaserate = (phase.subvec(1, prange.n_elem - 1) - phase.subvec(0, prange.n_elem - 2)) / delta_time;

    // remove NaN
    NaN_in_measured_data = arma::find_nonfinite(phase);

    if (NaN_in_measured_data.n_elem > 0)
        {
            std::cout << "Warning: Carrier phase rate have NaN values. \n";
        }

    double minphaserate = 0.001;
    double maxphaserate = 5000.0;

    idx = arma::find(phaserate > maxphaserate and phaserate < minphaserate);
    if (idx.n_elem > 0)
        {
            std::cout << "Warning: bad phase rate \n";
        }

    // 4) It checks the difference between code and phase rates
    // check difference between code and phase rates
    arma::vec ratediff = phaserate - coderate;

    // debug
    std::vector<double> tmp_time_vec(measurement_time.colptr(0),
        measurement_time.colptr(0) + measurement_time.n_rows);
    std::vector<double> tmp_vector_y6(phaserate.colptr(0),
        phaserate.colptr(0) + phaserate.n_rows);
    save_mat_xy(tmp_time_vec, tmp_vector_y6, std::string("phaserate_" + data_title));
    std::vector<double> tmp_vector_y7(coderate.colptr(0),
        coderate.colptr(0) + coderate.n_rows);
    save_mat_xy(tmp_time_vec, tmp_vector_y7, std::string("coderate_" + data_title));

    double maxratediff = 5;
    idx = arma::find(ratediff > maxratediff);
    if (idx.n_elem > 0)
        {
            std::cout << "Warning: bad code and phase rate difference \n";
        }

    std::vector<double>
        time_vector(measurement_time.colptr(0) + 1, measurement_time.colptr(0) + measurement_time.n_rows);

    // 2. RMSE
    arma::vec err;
    err = std::move(ratediff);

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
    std::cout << std::setprecision(10) << data_title << " RMSE = "
              << rmse << ", mean = " << error_mean
              << ", stdev = " << sqrt(error_var)
              << " (max,min) = " << max_error
              << "," << min_error
              << " [m/s]\n";
    std::cout.precision(ss);

    // plots
    if (FLAGS_show_plots)
        {
            Gnuplot g3("linespoints");
            g3.set_title(data_title + "Code rate - phase rate [m/s]");
            g3.set_grid();
            g3.set_xlabel("Time [s]");
            g3.set_ylabel("Code rate - phase rate [m/s]");
            // conversion between arma::vec and std:vector
            std::vector<double> range_error_m(err.colptr(0), err.colptr(0) + err.n_rows);
            g3.cmd("set key box opaque");
            g3.plot_xy(time_vector, range_error_m,
                "Code rate - phase rate");
            g3.set_legend();
            std::string data_title_aux = data_title;
            std::replace(data_title_aux.begin(), data_title_aux.end(), ' ', '_');
            std::replace(data_title_aux.begin(), data_title_aux.end(), '(', '_');
            std::replace(data_title_aux.begin(), data_title_aux.end(), ')', '_');
            g3.savetops(data_title_aux + "Code_rate_minus_phase_rate");

            g3.showonscreen();  // window output
        }
}


void code_phase_diff(
    arma::mat& measured_ch0,
    arma::mat& measured_ch1,
    const std::string& data_title)
{
    // 1. True value interpolation to match the measurement times
    arma::vec measurement_time = measured_ch0.col(0);

    arma::vec code_range_ch1_obs_interp;
    arma::interp1(measured_ch1.col(0), measured_ch1.col(1), measurement_time, code_range_ch1_obs_interp);
    arma::vec carrier_phase_ch1_obs_interp;
    arma::interp1(measured_ch1.col(0), measured_ch1.col(3), measurement_time, carrier_phase_ch1_obs_interp);

    // generate Code - Phase vector
    arma::vec code_minus_phase = (measured_ch0.col(1) - code_range_ch1_obs_interp) - (measured_ch0.col(3) - carrier_phase_ch1_obs_interp) * (gnsstk::C_MPS / gnsstk::L1_FREQ_GPS);

    // remove NaN
    arma::uvec NaN_in_measured_data = arma::find_nonfinite(code_minus_phase);

    arma::mat tmp_mat = arma::conv_to<arma::mat>::from(code_minus_phase);
    tmp_mat.shed_rows(NaN_in_measured_data);
    code_minus_phase = tmp_mat.col(0);

    code_minus_phase = code_minus_phase - code_minus_phase(0);

    tmp_mat = arma::conv_to<arma::mat>::from(measurement_time);
    tmp_mat.shed_rows(NaN_in_measured_data);
    measurement_time = tmp_mat.col(0);

    std::vector<double>
        time_vector(measurement_time.colptr(0), measurement_time.colptr(0) + measurement_time.n_rows);

    if (!measurement_time.empty())
        {
            // 2. RMSE
            arma::vec err;
            err = std::move(code_minus_phase);

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
            std::cout << std::setprecision(10) << data_title << " RMSE = "
                      << rmse << ", mean = " << error_mean
                      << ", stdev = " << sqrt(error_var)
                      << " (max,min) = " << max_error
                      << "," << min_error
                      << " [meters]\n";
            std::cout.precision(ss);

            // plots
            if (FLAGS_show_plots)
                {
                    Gnuplot g3("linespoints");
                    g3.set_title(data_title + "Code range - Carrier phase range [m]");
                    g3.set_grid();
                    g3.set_xlabel("Time [s]");
                    g3.set_ylabel("Code range - Carrier phase range [m]");
                    // conversion between arma::vec and std:vector
                    std::vector<double> range_error_m(err.colptr(0), err.colptr(0) + err.n_rows);
                    g3.cmd("set key box opaque");
                    g3.plot_xy(time_vector, range_error_m,
                        "Code range - Carrier phase range");
                    g3.set_legend();
                    std::string data_title_aux = data_title;
                    std::replace(data_title_aux.begin(), data_title_aux.end(), ' ', '_');
                    std::replace(data_title_aux.begin(), data_title_aux.end(), '(', '_');
                    std::replace(data_title_aux.begin(), data_title_aux.end(), ')', '_');
                    g3.savetops(data_title_aux + "Code_range_Carrier_phase_range");

                    g3.showonscreen();  // window output
                }
        }
    else
        {
            std::cout << "No valid data\n";
        }
}


double compute_rx_clock_error(const std::string& rinex_nav_filename, const std::string& rinex_obs_file)
{
    std::cout << "Computing receiver's clock error...\n";
    if (not file_exist(rinex_nav_filename.c_str()))
        {
            std::cout << "Warning: RINEX Nav file " << rinex_nav_filename << " does not exist, receiver's clock error could not be computed!\n";
            return 0.0;
        }
        // Declaration of objects for storing ephemerides and handling RAIM
#if GNSSTK_OLDER_THAN_13
    gnsstk::GPSEphemerisStore bcestore;
#else
    gnsstk::NavLibrary navLib;
    // Construct a NavDataFactory object
    gnsstk::NavDataFactoryPtr ndfp(
        std::make_shared<gnsstk::MultiFormatNavDataFactory>());
    // Add the NavDataFactory to the NavLibrary
    navLib.addFactory(ndfp);
#endif
    gnsstk::PRSolution raimSolver;

    // Object for void-type tropospheric model (in case no meteorological
    // RINEX is available)
    gnsstk::ZeroTropModel noTropModel;

    // Object for GG-type tropospheric model (Goad and Goodman, 1974)
    // Default constructor => default values for model
    gnsstk::GGTropModel ggTropModel;

    // Pointer to one of the two available tropospheric models. It points
    // to the void model by default
    gnsstk::TropModel* tropModelPtr = &noTropModel;

    double rx_clock_error_s = 0.0;
    try
        {
#if GNSSTK_OLDER_THAN_13
            // Read nav file and store unique list of ephemerides
            gnsstk::Rinex3NavStream rnffs(rinex_nav_filename.c_str());  // Open ephemerides data file
            gnsstk::Rinex3NavData rne;
            gnsstk::Rinex3NavHeader hdr;
            // Let's read the header (may be skipped)
            rnffs >> hdr;
            while (rnffs >> rne)
                {
                    bcestore.addEphemeris(rne);
                }
            // Setting the criteria for looking up ephemeris
            bcestore.SearchNear();
#else
            if (!ndfp->addDataSource(rinex_nav_filename))
                {
                    std::cerr << "Unable to load " << rinex_nav_filename << '\n';
                    return 0.0;
                }
#endif

            // Open and read the observation file one epoch at a time.
            // For each epoch, compute and print a position solution
            gnsstk::Rinex3ObsStream roffs(rinex_obs_file.c_str());  // Open observations data file

            gnsstk::Rinex3ObsHeader roh;
            gnsstk::Rinex3ObsData rod;

            // Let's read the header
            roffs >> roh;

            int indexC1;
            try
                {
                    indexC1 = roh.getObsIndex("C1");
                }
            catch (...)
                {
                    std::cerr << "The observation file doesn't have C1 pseudoranges, RX clock error could not be computed!\n";
                    return (0.0);
                }

            // Let's process all lines of observation data, one by one
            while (roffs >> rod)
                {
                    // Apply editing criteria
                    if (rod.epochFlag == 0 || rod.epochFlag == 1)  // Begin usable data
                        {
                            std::vector<gnsstk::SatID> prnVec;
                            std::vector<double> rangeVec;

                            // Define the "it" iterator to visit the observations PRN map.
                            // Rinex3ObsData::DataMap is a map from RinexSatID to
                            // vector<RinexDatum>:
                            //      std::map<RinexSatID, vector<RinexDatum> >
                            gnsstk::Rinex3ObsData::DataMap::const_iterator it;

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
#if OLD_GPSTK
                                    std::vector<gpstk::SatID::SatelliteSystem> Syss;
#endif
                                    gnsstk::Matrix<double> invMC;
                                    int iret;
                                    // Call RAIMCompute
#if OLD_GPSTK
                                    iret = raimSolver.RAIMCompute(rod.time, prnVec, Syss, rangeVec, invMC,
                                        &bcestore, tropModelPtr);
#else
#if GNSSTK_OLDER_THAN_13
                                    iret = raimSolver.RAIMCompute(rod.time, prnVec, rangeVec, invMC,
                                        &bcestore, tropModelPtr);
#else
                                    iret = raimSolver.RAIMCompute(rod.time, prnVec, rangeVec, invMC,
                                        navLib, tropModelPtr);
#endif
#endif
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
                            catch (const gnsstk::Exception& e)
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
                                              << raimSolver.Solution(3) / gnsstk::C_MPS << " [s] \n";

                                    std::cout << "NSATS, DOPs " << std::setw(2) << raimSolver.Nsvs << std::fixed
                                              << std::setprecision(2) << " "
                                              << std::setw(4) << raimSolver.PDOP << " " << std::setw(4) << raimSolver.GDOP
                                              << " " << std::setw(8) << raimSolver.RMSResidual << "\n";
                                    gnsstk::Position rx_pos;
                                    rx_pos.setECEF(gnsstk::Triple(raimSolver.Solution(0), raimSolver.Solution(1), raimSolver.Solution(2)));

                                    double lat_deg = rx_pos.geodeticLatitude();
                                    double lon_deg = rx_pos.longitude();
                                    double Alt_m = rx_pos.getAltitude();
                                    std::cout << "RX POS GEO [Lat,Long,H]" << std::fixed << std::setprecision(10) << " "
                                              << std::setw(12) << lat_deg << ","
                                              << std::setw(12) << lon_deg << ","
                                              << std::setw(12) << Alt_m << " [deg],[deg],[m]\n";

                                    // set computed RX clock error and stop iterating obs epochs
                                    rx_clock_error_s = raimSolver.Solution(3) / gnsstk::C_MPS;
                                    break;
                                }  // End of 'if( raimSolver.isValid() )'
                        }          // End of 'if( rod.epochFlag == 0 || rod.epochFlag == 1 )'
                }                  // End of 'while( roffs >> rod )'
        }
    catch (const gnsstk::FFStreamError& e)
        {
            std::cout << "GPSTK exception: " << e << '\n';
        }
    catch (const gnsstk::Exception& e)
        {
            std::cout << "GPSTK exception: " << e << '\n';
        }
    catch (...)
        {
            std::cout << "Caught an unexpected exception.\n";
        }

    return rx_clock_error_s;
}


void RINEX_doublediff_dupli_sat()
{
    // special test mode for duplicated satellites
    // read rinex receiver-under-test observations
    std::map<int, arma::mat> rover_obs = ReadRinexObs(FLAGS_rover_rinex_obs, 'G', std::string("1C"));
    if (rover_obs.empty())
        {
            return;
        }
    // Cut measurement initial transitory of the measurements
    double initial_transitory_s = FLAGS_skip_obs_transitory_s;
    std::cout << "Skipping initial transitory of " << initial_transitory_s << " [s]\n";
    arma::uvec index;
    for (auto& rover_ob : rover_obs)
        {
            index = arma::find(rover_ob.second.col(0) >= (rover_ob.second.col(0)(0) + initial_transitory_s), 1, "first");
            if ((!index.empty()) and (index(0) > 0))
                {
                    rover_ob.second.shed_rows(0, index(0));
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
                    if (rover_obs.find(prn_pairs.at(n)) != rover_obs.end() and rover_obs.find(prn_pairs.at(n + 1)) != rover_obs.end())
                        {
                            std::cout << "Computing single difference observables for duplicated SV pairs...\n";
                            std::cout << "SD = OBS_ROVER(SV" << prn_pairs.at(n) << ") - OBS_ROVER(SV" << prn_pairs.at(n + 1) << ")\n";

                            code_pseudorange_single_diff(rover_obs.at(prn_pairs.at(n)),
                                rover_obs.at(prn_pairs.at(n + 1)),
                                "SD = OBS(SV" + std::to_string(prn_pairs.at(n)) + ") - OBS(SV" + std::to_string(prn_pairs.at(n + 1)) + ") ");

                            carrier_phase_single_diff(rover_obs.at(prn_pairs.at(n)),
                                rover_obs.at(prn_pairs.at(n + 1)),
                                "SD = OBS(SV" + std::to_string(prn_pairs.at(n)) + ") - OBS(SV" + std::to_string(prn_pairs.at(n + 1)) + ") ");

                            carrier_doppler_single_diff(rover_obs.at(prn_pairs.at(n)),
                                rover_obs.at(prn_pairs.at(n + 1)),
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
    // read rinex base observations
    std::map<int, arma::mat> base_obs = ReadRinexObs(FLAGS_base_rinex_obs, FLAGS_system.c_str()[0], FLAGS_signal);
    // read rinex receiver-under-test (rover) observations
    std::map<int, arma::mat> rover_obs = ReadRinexObs(FLAGS_rover_rinex_obs, FLAGS_system.c_str()[0], FLAGS_signal);

    if (base_obs.empty() or rover_obs.empty())
        {
            return;
        }

    // compute rx clock errors
    double base_rx_clock_error_s = 0.0;
    double rover_rx_clock_error_s = 0.0;
    if (remove_rx_clock_error == true)
        {
            base_rx_clock_error_s = compute_rx_clock_error(FLAGS_rinex_nav, FLAGS_base_rinex_obs);
            rover_rx_clock_error_s = compute_rx_clock_error(FLAGS_rinex_nav, FLAGS_rover_rinex_obs);
        }

    double common_clock_error_s = rover_rx_clock_error_s - base_rx_clock_error_s;

    // Cut measurement initial transitory of the measurements
    double initial_transitory_s = FLAGS_skip_obs_transitory_s;
    std::cout << "Skipping initial transitory of " << initial_transitory_s << " [s]\n";
    arma::uvec index;
    for (auto& rover_ob : rover_obs)
        {
            index = arma::find(rover_ob.second.col(0) >= (rover_ob.second.col(0)(0) + initial_transitory_s), 1, "first");
            if ((!index.empty()) and (index(0) > 0))
                {
                    rover_ob.second.shed_rows(0, index(0));
                }
        }

    // Cut observation vectors ends to the shortest one (base or rover)
    arma::colvec base_obs_time = base_obs.begin()->second.col(0);
    arma::colvec rover_obs_time = rover_obs.begin()->second.col(0);

    if (base_obs_time.back() < rover_obs_time.back())
        {
            // there are more rover observations than base observations
            // cut rover vector
            std::cout << "Cutting rover observations vector end..\n";
            arma::uvec index2;
            for (auto& rover_ob : rover_obs)
                {
                    index = arma::find(rover_ob.second.col(0) >= base_obs_time.back(), 1, "first");
                    if ((!index.empty()) and (index(0) > 0))
                        {
                            rover_ob.second.shed_rows(index(0), rover_ob.second.n_rows - 1);
                        }
                }
        }
    else
        {
            // there are more base observations than rover observations
            // cut base vector
            std::cout << "Cutting base observations vector end..\n";
            for (auto& base_ob : base_obs)
                {
                    index = arma::find(base_ob.second.col(0) >= rover_obs_time.back(), 1, "first");
                    if ((!index.empty()) and (index(0) > 0))
                        {
                            base_ob.second.shed_rows(index(0), base_ob.second.n_rows - 1);
                        }
                }
        }

    // also skip last seconds of the observations (some artifacts are present in some RINEX endings)
    base_obs_time = base_obs.begin()->second.col(0);
    rover_obs_time = rover_obs.begin()->second.col(0);

    double skip_ends_s = FLAGS_skip_obs_ends_s;
    std::cout << "Skipping last " << skip_ends_s << " [s] of observations\n";
    for (auto& rover_ob : rover_obs)
        {
            index = arma::find(rover_ob.second.col(0) >= (rover_obs_time.back() - skip_ends_s), 1, "first");
            if ((!index.empty()) and (index(0) > 0))
                {
                    rover_ob.second.shed_rows(index(0), rover_ob.second.n_rows - 1);
                }
        }
    for (auto& base_ob : base_obs)
        {
            index = arma::find(base_ob.second.col(0) >= (base_obs_time.back() - skip_ends_s), 1, "first");
            if ((!index.empty()) and (index(0) > 0))
                {
                    base_ob.second.shed_rows(index(0), base_ob.second.n_rows - 1);
                }
        }

    // Save observations in .mat files
    std::cout << "Saving RAW observables inputs to .mat files...\n";
    for (auto& base_ob : base_obs)
        {
            // std::cout << it->first << " => " << it->second.n_rows << '\n';
            // std::cout << it->first << " has NaN values: " << it->second.has_nan() << '\n';
            std::vector<double> tmp_time_vec(base_ob.second.col(0).colptr(0),
                base_ob.second.col(0).colptr(0) + base_ob.second.n_rows);
            std::vector<double> tmp_vector(base_ob.second.col(2).colptr(0),
                base_ob.second.col(2).colptr(0) + base_ob.second.n_rows);
            save_mat_xy(tmp_time_vec, tmp_vector, std::string("base_doppler_sat" + std::to_string(base_ob.first)));

            std::vector<double> tmp_vector2(base_ob.second.col(3).colptr(0),
                base_ob.second.col(3).colptr(0) + base_ob.second.n_rows);
            save_mat_xy(tmp_time_vec, tmp_vector2, std::string("base_carrier_phase_sat" + std::to_string(base_ob.first)));

            std::vector<double> tmp_vector3(base_ob.second.col(1).colptr(0),
                base_ob.second.col(1).colptr(0) + base_ob.second.n_rows);
            save_mat_xy(tmp_time_vec, tmp_vector3, std::string("base_pseudorange_sat" + std::to_string(base_ob.first)));
        }
    for (auto& rover_ob : rover_obs)
        {
            // std::cout << it->first << " => " << it->second.n_rows << '\n';
            // std::cout << it->first << " has NaN values: " << it->second.has_nan() << '\n';
            std::vector<double> tmp_time_vec(rover_ob.second.col(0).colptr(0),
                rover_ob.second.col(0).colptr(0) + rover_ob.second.n_rows);
            std::vector<double> tmp_vector(rover_ob.second.col(2).colptr(0),
                rover_ob.second.col(2).colptr(0) + rover_ob.second.n_rows);
            save_mat_xy(tmp_time_vec, tmp_vector, std::string("measured_doppler_sat" + std::to_string(rover_ob.first)));

            std::vector<double> tmp_vector2(rover_ob.second.col(3).colptr(0),
                rover_ob.second.col(3).colptr(0) + rover_ob.second.n_rows);
            save_mat_xy(tmp_time_vec, tmp_vector2, std::string("measured_carrier_phase_sat" + std::to_string(rover_ob.first)));

            std::vector<double> tmp_vector3(rover_ob.second.col(1).colptr(0),
                rover_ob.second.col(1).colptr(0) + rover_ob.second.n_rows);
            save_mat_xy(tmp_time_vec, tmp_vector3, std::string("measured_pseudorange_sat" + std::to_string(rover_ob.first)));
        }

    // select reference satellite
    std::set<int> PRN_set = available_gps_prn;
    double min_range = std::numeric_limits<double>::max();
    int reference_sat_id = 1;
    for (const auto& base_prn_it : PRN_set)
        {
            if (base_obs.find(base_prn_it) != base_obs.end() and rover_obs.find(base_prn_it) != rover_obs.end())
                {
                    if (rover_obs.at(base_prn_it).at(0, 1) < min_range)
                        {
                            min_range = rover_obs.at(base_prn_it).at(0, 1);
                            reference_sat_id = base_prn_it;
                        }
                }
        }

    // compute double differences
    if (base_obs.find(reference_sat_id) != base_obs.end() and rover_obs.find(reference_sat_id) != rover_obs.end())
        {
            std::cout << "Using reference satellite SV " << reference_sat_id << " with minimum range of " << min_range << " [meters]\n";
            for (const auto& current_sat_id : PRN_set)
                {
                    if (current_sat_id != reference_sat_id)
                        {
                            if (base_obs.find(current_sat_id) != base_obs.end() and rover_obs.find(current_sat_id) != rover_obs.end())
                                {
                                    std::cout << "Computing double difference observables for SV " << current_sat_id << '\n';
                                    std::cout << "DD = (OBS_ROVER(SV" << current_sat_id << ") - OBS_ROVER(SV" << reference_sat_id << "))"
                                              << " - (OBS_BASE(SV" << current_sat_id << ") - OBS_BASE(SV" << reference_sat_id << "))\n";

                                    code_pseudorange_double_diff(base_obs.at(reference_sat_id),
                                        base_obs.at(current_sat_id),
                                        rover_obs.at(reference_sat_id),
                                        rover_obs.at(current_sat_id),
                                        "PRN " + std::to_string(current_sat_id) + " ", common_clock_error_s);

                                    carrier_phase_double_diff(base_obs.at(reference_sat_id),
                                        base_obs.at(current_sat_id),
                                        rover_obs.at(reference_sat_id),
                                        rover_obs.at(current_sat_id),
                                        "PRN " + std::to_string(current_sat_id) + " ", common_clock_error_s);

                                    carrier_doppler_double_diff(base_obs.at(reference_sat_id),
                                        base_obs.at(current_sat_id),
                                        rover_obs.at(reference_sat_id),
                                        rover_obs.at(current_sat_id),
                                        "PRN " + std::to_string(current_sat_id) + " ", common_clock_error_s);
                                }
                        }
                }
        }
    else
        {
            std::cout << "Satellite ID " << reference_sat_id << " not found in both RINEX files\n";
        }
}


void RINEX_singlediff()
{
    // read rinex receiver-under-test observations
    std::map<int, arma::mat> rover_obs = ReadRinexObs(FLAGS_rover_rinex_obs, FLAGS_system.c_str()[0], FLAGS_signal);

    if (rover_obs.empty())
        {
            return;
        }

    // Cut measurement initial transitory of the measurements
    double initial_transitory_s = FLAGS_skip_obs_transitory_s;
    std::cout << "Skipping initial transitory of " << initial_transitory_s << " [s]\n";
    arma::uvec index;
    for (auto& rover_ob : rover_obs)
        {
            index = arma::find(rover_ob.second.col(0) >= (rover_ob.second.col(0)(0) + initial_transitory_s), 1, "first");
            if ((!index.empty()) and (index(0) > 0))
                {
                    rover_ob.second.shed_rows(0, index(0));
                }
        }

    // also skip last seconds of the observations (some artifacts are present in some RINEX endings)
    arma::colvec rover_obs_time = rover_obs.begin()->second.col(0);

    double skip_ends_s = FLAGS_skip_obs_ends_s;
    std::cout << "Skipping last " << skip_ends_s << " [s] of observations\n";
    for (auto& rover_ob : rover_obs)
        {
            index = arma::find(rover_ob.second.col(0) >= (rover_obs_time.back() - skip_ends_s), 1, "first");
            if ((!index.empty()) and (index(0) > 0))
                {
                    rover_ob.second.shed_rows(index(0), rover_ob.second.n_rows - 1);
                }
        }

    // Save observations in .mat files
    std::cout << "Saving RAW observables inputs to .mat files...\n";

    for (auto& rover_ob : rover_obs)
        {
            // std::cout << it->first << " => " << it->second.n_rows << '\n';
            // std::cout << it->first << " has NaN values: " << it->second.has_nan() << '\n';
            std::vector<double> tmp_time_vec(rover_ob.second.col(0).colptr(0),
                rover_ob.second.col(0).colptr(0) + rover_ob.second.n_rows);
            std::vector<double> tmp_vector(rover_ob.second.col(2).colptr(0),
                rover_ob.second.col(2).colptr(0) + rover_ob.second.n_rows);
            save_mat_xy(tmp_time_vec, tmp_vector, std::string("measured_doppler_sat" + std::to_string(rover_ob.first)));

            std::vector<double> tmp_vector2(rover_ob.second.col(3).colptr(0),
                rover_ob.second.col(3).colptr(0) + rover_ob.second.n_rows);
            save_mat_xy(tmp_time_vec, tmp_vector2, std::string("measured_carrier_phase_sat" + std::to_string(rover_ob.first)));

            std::vector<double> tmp_vector3(rover_ob.second.col(1).colptr(0),
                rover_ob.second.col(1).colptr(0) + rover_ob.second.n_rows);
            save_mat_xy(tmp_time_vec, tmp_vector3, std::string("measured_pseudorange_sat" + std::to_string(rover_ob.first)));
        }

    // compute single differences
    std::set<int> PRN_set = available_gps_prn;
    std::cout << "Computing Code Pseudorange rate vs. Carrier phase rate difference...\n";
    for (const auto& current_sat_id : PRN_set)
        {
            if (rover_obs.find(current_sat_id) != rover_obs.end())
                {
                    std::cout << "RateError = PR_rate(SV" << current_sat_id << ") - Phase_rate(SV" << current_sat_id << ")\n";
                    coderate_phaserate_consistence(rover_obs.at(current_sat_id), "PRN " + std::to_string(current_sat_id) + " ");
                }
        }
}


int main(int argc, char** argv)
{
    std::cout << "Running RINEX observables difference tool...\n";
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    try
        {
            if (FLAGS_single_diff)
                {
                    if (FLAGS_dupli_sat)
                        {
                            RINEX_doublediff_dupli_sat();
                        }
                    else
                        {
                            RINEX_singlediff();
                        }
                }
            else
                {
                    RINEX_doublediff(FLAGS_remove_rx_clock_error);
                }
        }
    catch (const gnsstk::Exception& e)
        {
            std::cerr << e;
            gflags::ShutDownCommandLineFlags();
            return 1;
        }
    catch (const std::exception& e)
        {
            std::cerr << "Exception: " << e.what();
            gflags::ShutDownCommandLineFlags();
            return 1;
        }
    catch (...)
        {
            std::cerr << "Unknown error\n";
            gflags::ShutDownCommandLineFlags();
            return 1;
        }
    gflags::ShutDownCommandLineFlags();
    return 0;
}
