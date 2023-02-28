/*!
 * \file rtklib_solver.cc
 * \brief PVT solver based on rtklib library functions adapted to the GNSS-SDR
 *  data flow and structures
 * \authors <ul>
 *          <li> 2017-2019, Javier Arribas
 *          <li> 2017-2023, Carles Fernandez
 *          <li> 2007-2013, T. Takasu
 *          </ul>
 *
 * This is a derived work from RTKLIB http://www.rtklib.com/
 * The original source code at https://github.com/tomojitakasu/RTKLIB is
 * released under the BSD 2-clause license with an additional exclusive clause
 * that does not apply here. This additional clause is reproduced below:
 *
 * " The software package includes some companion executive binaries or shared
 * libraries necessary to execute APs on Windows. These licenses succeed to the
 * original ones of these software. "
 *
 * Neither the executive binaries nor the shared libraries are required by, used
 * or included in GNSS-SDR.
 *
 * -----------------------------------------------------------------------------
 * Copyright (C) 2007-2013, T. Takasu
 * Copyright (C) 2017-2019, Javier Arribas
 * Copyright (C) 2017-2023, Carles Fernandez
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * -----------------------------------------------------------------------*/

#include "rtklib_solver.h"
#include "Beidou_DNAV.h"
#include "gnss_sdr_filesystem.h"
#include "rtklib_rtkpos.h"
#include "rtklib_solution.h"
#include <glog/logging.h>
#include <matio.h>
#include <algorithm>
#include <cmath>
#include <exception>
#include <utility>
#include <vector>


Rtklib_Solver::Rtklib_Solver(const rtk_t &rtk,
    const std::string &dump_filename,
    uint32_t type_of_rx,
    bool flag_dump_to_file,
    bool flag_dump_to_mat,
    bool use_e6_for_pvt) : d_dump_filename(dump_filename),
                           d_rtk(rtk),
                           d_type_of_rx(type_of_rx),
                           d_flag_dump_enabled(flag_dump_to_file),
                           d_flag_dump_mat_enabled(flag_dump_to_mat),
                           d_use_e6_for_pvt(use_e6_for_pvt)
{
    this->set_averaging_flag(false);

    // see freq index at src/algorithms/libs/rtklib/rtklib_rtkcmn.cc
    // function: satwavelen
    d_rtklib_freq_index[0] = 0;
    d_rtklib_freq_index[1] = 1;
    d_rtklib_freq_index[2] = 2;

    d_rtklib_band_index["1G"] = 0;
    d_rtklib_band_index["1C"] = 0;
    d_rtklib_band_index["1B"] = 0;
    d_rtklib_band_index["B1"] = 0;
    d_rtklib_band_index["B3"] = 2;
    d_rtklib_band_index["2G"] = 1;
    d_rtklib_band_index["2S"] = 1;
    d_rtklib_band_index["7X"] = 2;
    d_rtklib_band_index["5X"] = 2;
    d_rtklib_band_index["L5"] = 2;
    d_rtklib_band_index["E6"] = 0;

    switch (d_type_of_rx)
        {
        case 6:  // E5b only
            d_rtklib_freq_index[2] = 4;
            break;
        case 11:  // GPS L1 C/A + Galileo E5b
            d_rtklib_freq_index[2] = 4;
            break;
        case 15:  // Galileo E1B + Galileo E5b
            d_rtklib_freq_index[2] = 4;
            break;
        case 18:  // GPS L2C + Galileo E5b
            d_rtklib_freq_index[2] = 4;
            break;
        case 19:  // Galileo E5a + Galileo E5b
            d_rtklib_band_index["5X"] = 0;
            d_rtklib_freq_index[0] = 2;
            d_rtklib_freq_index[2] = 4;
            break;
        case 20:  // GPS L5 + Galileo E5b
            d_rtklib_band_index["L5"] = 0;
            d_rtklib_freq_index[0] = 2;
            d_rtklib_freq_index[2] = 4;
            break;
        case 100:  // E6B only
            d_rtklib_freq_index[0] = 3;
            break;
        case 101:  // E1 + E6B
            d_rtklib_band_index["E6"] = 1;
            d_rtklib_freq_index[1] = 3;
            break;
        case 102:  // E5a + E6B
            d_rtklib_band_index["E6"] = 1;
            d_rtklib_freq_index[1] = 3;
            break;
        case 103:  // E5b + E6B
            d_rtklib_band_index["E6"] = 1;
            d_rtklib_freq_index[1] = 3;
            d_rtklib_freq_index[2] = 4;
            break;
        case 104:  // Galileo E1B + Galileo E5a + Galileo E6B
            d_rtklib_band_index["E6"] = 1;
            d_rtklib_freq_index[1] = 3;
            break;
        case 105:  // Galileo E1B + Galileo E5b + Galileo E6B
            d_rtklib_freq_index[2] = 4;
            d_rtklib_band_index["E6"] = 1;
            d_rtklib_freq_index[1] = 3;
            break;
        case 106:  // GPS L1 C/A + Galileo E1B + Galileo E6B
        case 107:  // GPS L1 C/A + Galileo E6B
            d_rtklib_band_index["E6"] = 1;
            d_rtklib_freq_index[1] = 3;
            break;
        }
    // auto empty_map = std::map < int, HAS_obs_corrections >> ();
    // d_has_obs_corr_map["L1 C/A"] = empty_map;

    // ############# ENABLE DATA FILE LOG #################
    if (d_flag_dump_enabled == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                        {
                            d_dump_file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "PVT lib dump enabled Log file: " << d_dump_filename.c_str();
                        }
                    catch (const std::ofstream::failure &e)
                        {
                            LOG(WARNING) << "Exception opening RTKLIB dump file " << e.what();
                        }
                }
        }
}


Rtklib_Solver::~Rtklib_Solver()
{
    DLOG(INFO) << "Rtklib_Solver destructor called.";
    if (d_dump_file.is_open() == true)
        {
            const auto pos = d_dump_file.tellp();
            try
                {
                    d_dump_file.close();
                }
            catch (const std::exception &ex)
                {
                    LOG(WARNING) << "Exception in destructor closing the RTKLIB dump file " << ex.what();
                }
            if (pos == 0)
                {
                    errorlib::error_code ec;
                    if (!fs::remove(fs::path(d_dump_filename), ec))
                        {
                            std::cerr << "Problem removing temporary file " << d_dump_filename << '\n';
                        }
                    d_flag_dump_mat_enabled = false;
                }
        }
    if (d_flag_dump_mat_enabled)
        {
            try
                {
                    save_matfile();
                }
            catch (const std::exception &ex)
                {
                    LOG(WARNING) << "Exception in destructor saving the PVT .mat dump file " << ex.what();
                }
        }
}


bool Rtklib_Solver::save_matfile() const
{
    // READ DUMP FILE
    const std::string dump_filename = d_dump_filename;
    const int32_t number_of_double_vars = 21;
    const int32_t number_of_uint32_vars = 2;
    const int32_t number_of_uint8_vars = 3;
    const int32_t number_of_float_vars = 2;
    const int32_t epoch_size_bytes = sizeof(double) * number_of_double_vars +
                                     sizeof(uint32_t) * number_of_uint32_vars +
                                     sizeof(uint8_t) * number_of_uint8_vars +
                                     sizeof(float) * number_of_float_vars;
    std::ifstream dump_file;
    dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try
        {
            dump_file.open(dump_filename.c_str(), std::ios::binary | std::ios::ate);
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Problem opening dump file:" << e.what() << '\n';
            return false;
        }
    // count number of epochs and rewind
    int64_t num_epoch = 0LL;
    if (dump_file.is_open())
        {
            std::cout << "Generating .mat file for " << dump_filename << '\n';
            const std::ifstream::pos_type size = dump_file.tellg();
            num_epoch = static_cast<int64_t>(size) / static_cast<int64_t>(epoch_size_bytes);
            dump_file.seekg(0, std::ios::beg);
        }
    else
        {
            return false;
        }

    auto TOW_at_current_symbol_ms = std::vector<uint32_t>(num_epoch);
    auto week = std::vector<uint32_t>(num_epoch);
    auto RX_time = std::vector<double>(num_epoch);
    auto user_clk_offset = std::vector<double>(num_epoch);
    auto pos_x = std::vector<double>(num_epoch);
    auto pos_y = std::vector<double>(num_epoch);
    auto pos_z = std::vector<double>(num_epoch);
    auto vel_x = std::vector<double>(num_epoch);
    auto vel_y = std::vector<double>(num_epoch);
    auto vel_z = std::vector<double>(num_epoch);
    auto cov_xx = std::vector<double>(num_epoch);
    auto cov_yy = std::vector<double>(num_epoch);
    auto cov_zz = std::vector<double>(num_epoch);
    auto cov_xy = std::vector<double>(num_epoch);
    auto cov_yz = std::vector<double>(num_epoch);
    auto cov_zx = std::vector<double>(num_epoch);
    auto latitude = std::vector<double>(num_epoch);
    auto longitude = std::vector<double>(num_epoch);
    auto height = std::vector<double>(num_epoch);
    auto valid_sats = std::vector<uint8_t>(num_epoch);
    auto solution_status = std::vector<uint8_t>(num_epoch);
    auto solution_type = std::vector<uint8_t>(num_epoch);
    auto AR_ratio_factor = std::vector<float>(num_epoch);
    auto AR_ratio_threshold = std::vector<float>(num_epoch);
    auto gdop = std::vector<double>(num_epoch);
    auto pdop = std::vector<double>(num_epoch);
    auto hdop = std::vector<double>(num_epoch);
    auto vdop = std::vector<double>(num_epoch);

    try
        {
            if (dump_file.is_open())
                {
                    for (int64_t i = 0; i < num_epoch; i++)
                        {
                            dump_file.read(reinterpret_cast<char *>(&TOW_at_current_symbol_ms[i]), sizeof(uint32_t));
                            dump_file.read(reinterpret_cast<char *>(&week[i]), sizeof(uint32_t));
                            dump_file.read(reinterpret_cast<char *>(&RX_time[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&user_clk_offset[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&pos_x[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&pos_y[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&pos_z[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&vel_x[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&vel_y[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&vel_z[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&cov_xx[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&cov_yy[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&cov_zz[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&cov_xy[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&cov_yz[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&cov_zx[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&latitude[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&longitude[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&height[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&valid_sats[i]), sizeof(uint8_t));
                            dump_file.read(reinterpret_cast<char *>(&solution_status[i]), sizeof(uint8_t));
                            dump_file.read(reinterpret_cast<char *>(&solution_type[i]), sizeof(uint8_t));
                            dump_file.read(reinterpret_cast<char *>(&AR_ratio_factor[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&AR_ratio_threshold[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&gdop[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&pdop[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&hdop[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&vdop[i]), sizeof(double));
                        }
                }
            dump_file.close();
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Problem reading dump file:" << e.what() << '\n';
            return false;
        }

    // WRITE MAT FILE
    mat_t *matfp;
    matvar_t *matvar;
    std::string filename = dump_filename;
    filename.erase(filename.length() - 4, 4);
    filename.append(".mat");
    matfp = Mat_CreateVer(filename.c_str(), nullptr, MAT_FT_MAT73);
    if (reinterpret_cast<int64_t *>(matfp) != nullptr)
        {
            std::array<size_t, 2> dims{1, static_cast<size_t>(num_epoch)};
            matvar = Mat_VarCreate("TOW_at_current_symbol_ms", MAT_C_UINT32, MAT_T_UINT32, 2, dims.data(), TOW_at_current_symbol_ms.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("week", MAT_C_UINT32, MAT_T_UINT32, 2, dims.data(), week.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("RX_time", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), RX_time.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("user_clk_offset", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), user_clk_offset.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("pos_x", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), pos_x.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("pos_y", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), pos_y.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("pos_z", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), pos_z.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("vel_x", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), vel_x.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("vel_y", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), vel_y.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("vel_z", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), vel_z.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("cov_xx", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), cov_xx.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("cov_yy", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), cov_yy.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("cov_zz", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), cov_zz.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("cov_xy", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), cov_xy.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("cov_yz", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), cov_yz.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("cov_zx", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), cov_zx.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("latitude", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), latitude.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("longitude", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), longitude.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("height", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), height.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("valid_sats", MAT_C_UINT8, MAT_T_UINT8, 2, dims.data(), valid_sats.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("solution_status", MAT_C_UINT8, MAT_T_UINT8, 2, dims.data(), solution_status.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("solution_type", MAT_C_UINT8, MAT_T_UINT8, 2, dims.data(), solution_type.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("AR_ratio_factor", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), AR_ratio_factor.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("AR_ratio_threshold", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), AR_ratio_threshold.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("gdop", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), gdop.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("pdop", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), pdop.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("hdop", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), hdop.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("vdop", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), vdop.data(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);
        }

    Mat_Close(matfp);
    return true;
}


double Rtklib_Solver::get_gdop() const
{
    return d_dop[0];
}


double Rtklib_Solver::get_pdop() const
{
    return d_dop[1];
}


double Rtklib_Solver::get_hdop() const
{
    return d_dop[2];
}


double Rtklib_Solver::get_vdop() const
{
    return d_dop[3];
}


Monitor_Pvt Rtklib_Solver::get_monitor_pvt() const
{
    return d_monitor_pvt;
}


void Rtklib_Solver::store_has_data(const Galileo_HAS_data &new_has_data)
{
    //  Compute time of application HAS SIS ICD, Issue 1.0, Section 7.7
    uint16_t toh = new_has_data.header.toh;
    uint32_t hr = std::floor(new_has_data.tow / 3600);
    uint32_t tmt = 0;
    if ((hr * 3600 + toh) <= new_has_data.tow)
        {
            tmt = hr * 3600 + toh;
        }
    else
        {
            tmt = (hr - 1) * 3600 + toh;
        }

    const std::string gps_str("GPS");
    const std::string gal_str("Galileo");
    if (new_has_data.header.orbit_correction_flag)
        {
            LOG(INFO) << "Received HAS orbit corrections";
            // for each satellite in GPS ephemeris
            for (const auto &gpseph : gps_ephemeris_map)
                {
                    int prn = gpseph.second.PRN;
                    int32_t sis_iod = gpseph.second.IODE_SF3;
                    uint16_t gnss_iod = new_has_data.get_gnss_iod(gps_str, prn);
                    if (static_cast<int32_t>(gnss_iod) == sis_iod)
                        {
                            float radial_m = new_has_data.get_delta_radial_m(gps_str, prn);
                            if (std::fabs(radial_m + 10.24) < 0.001)  // -10.24 means not available
                                {
                                    radial_m = 0.0;
                                }
                            float in_track_m = new_has_data.get_delta_in_track_m(gps_str, prn);
                            if (std::fabs(in_track_m + 16.384) < 0.001)  // -16.384 means not available
                                {
                                    in_track_m = 0.0;
                                }
                            float cross_track_m = new_has_data.get_delta_in_track_m(gps_str, prn);
                            if (std::fabs(cross_track_m + 16.384) < 0.001)  // -16.384 means not available
                                {
                                    cross_track_m = 0.0;
                                }
                            d_has_orbit_corrections_store_map[gps_str][prn].radial_m = radial_m;
                            d_has_orbit_corrections_store_map[gps_str][prn].in_track_m = in_track_m;
                            d_has_orbit_corrections_store_map[gps_str][prn].cross_track_m = cross_track_m;
                            d_has_orbit_corrections_store_map[gps_str][prn].valid_until = tmt +
                                                                                          new_has_data.get_validity_interval_s(new_has_data.validity_interval_index_orbit_corrections);
                            d_has_orbit_corrections_store_map[gps_str][prn].iod = gnss_iod;
                            // TODO: check for end of week
                        }
                }

            // for each satellite in Galileo ephemeris
            for (const auto &galeph : galileo_ephemeris_map)
                {
                    int prn = galeph.second.PRN;
                    int32_t sis_iod = galeph.second.IOD_ephemeris;
                    uint16_t gnss_iod = new_has_data.get_gnss_iod(gal_str, prn);
                    if (static_cast<int32_t>(gnss_iod) == sis_iod)
                        {
                            float radial_m = new_has_data.get_delta_radial_m(gal_str, prn);
                            if (std::fabs(radial_m + 10.24) < 0.001)  // -10.24 means not available
                                {
                                    radial_m = 0.0;
                                }
                            float in_track_m = new_has_data.get_delta_in_track_m(gal_str, prn);
                            if (std::fabs(in_track_m + 16.384) < 0.001)  // -16.384 means not available
                                {
                                    in_track_m = 0.0;
                                }
                            float cross_track_m = new_has_data.get_delta_in_track_m(gal_str, prn);
                            if (std::fabs(cross_track_m + 16.384) < 0.001)  // -16.384 means not available
                                {
                                    cross_track_m = 0.0;
                                }
                            d_has_orbit_corrections_store_map[gal_str][prn].radial_m = radial_m;
                            d_has_orbit_corrections_store_map[gal_str][prn].in_track_m = in_track_m;
                            d_has_orbit_corrections_store_map[gal_str][prn].cross_track_m = cross_track_m;
                            d_has_orbit_corrections_store_map[gal_str][prn].valid_until = tmt +
                                                                                          new_has_data.get_validity_interval_s(new_has_data.validity_interval_index_orbit_corrections);
                            d_has_orbit_corrections_store_map[gal_str][prn].iod = gnss_iod;
                            // TODO: check for end of week
                        }
                }
        }
    if (new_has_data.header.clock_fullset_flag)
        {
            LOG(INFO) << "Received HAS clock fullset corrections";
            for (const auto &gpseph : gps_ephemeris_map)
                {
                    int prn = gpseph.second.PRN;
                    int32_t sis_iod = gpseph.second.IODE_SF3;
                    auto it = d_has_orbit_corrections_store_map[gps_str].find(prn);
                    if (it != d_has_orbit_corrections_store_map[gps_str].end())
                        {
                            uint16_t gnss_iod = it->second.iod;
                            if (static_cast<int32_t>(gnss_iod) == sis_iod)
                                {
                                    float clock_correction_mult_m = new_has_data.get_clock_correction_mult_m(gps_str, prn);
                                    if ((std::fabs(clock_correction_mult_m + 10.24) < 0.001) ||
                                        (std::fabs(clock_correction_mult_m + 20.48) < 0.001) ||
                                        (std::fabs(clock_correction_mult_m + 30.72) < 0.001) ||
                                        (std::fabs(clock_correction_mult_m + 40.96) < 0.001))
                                        {
                                            clock_correction_mult_m = 0.0;
                                        }
                                    if ((std::fabs(clock_correction_mult_m - 10.2375) < 0.001) ||
                                        (std::fabs(clock_correction_mult_m - 20.475) < 0.001) ||
                                        (std::fabs(clock_correction_mult_m - 30.7125) < 0.001) ||
                                        (std::fabs(clock_correction_mult_m - 40.95) < 0.001))
                                        {
                                            // Satellite should not be used!
                                            clock_correction_mult_m = 0.0;
                                        }
                                    d_has_clock_corrections_store_map[gps_str][prn].clock_correction_m = clock_correction_mult_m;
                                    d_has_clock_corrections_store_map[gps_str][prn].valid_until = tmt +
                                                                                                  new_has_data.get_validity_interval_s(new_has_data.validity_interval_index_clock_fullset_corrections);
                                    // TODO: check for end of week
                                }
                        }
                }

            // for each satellite in Galileo ephemeris
            for (const auto &galeph : galileo_ephemeris_map)
                {
                    int prn = galeph.second.PRN;
                    int32_t iod_sis = galeph.second.IOD_ephemeris;
                    auto it = d_has_orbit_corrections_store_map[gal_str].find(prn);
                    if (it != d_has_orbit_corrections_store_map[gal_str].end())
                        {
                            uint16_t gnss_iod = it->second.iod;
                            if (static_cast<int32_t>(gnss_iod) == iod_sis)
                                {
                                    float clock_correction_mult_m = new_has_data.get_clock_correction_mult_m(gal_str, prn);
                                    // std::cout << "Galileo Satellite " << prn
                                    //           << " clock correction=" << new_has_data.get_clock_correction_mult_m(gal_str, prn)
                                    //           << std::endl;
                                    if ((std::fabs(clock_correction_mult_m + 10.24) < 0.001) ||
                                        (std::fabs(clock_correction_mult_m + 20.48) < 0.001) ||
                                        (std::fabs(clock_correction_mult_m + 30.72) < 0.001) ||
                                        (std::fabs(clock_correction_mult_m + 40.96) < 0.001))
                                        {
                                            clock_correction_mult_m = 0.0;
                                        }
                                    d_has_clock_corrections_store_map[gal_str][prn].clock_correction_m = clock_correction_mult_m;
                                    d_has_clock_corrections_store_map[gal_str][prn].valid_until = tmt +
                                                                                                  new_has_data.get_validity_interval_s(new_has_data.validity_interval_index_clock_fullset_corrections);
                                    // TODO: check for end of week
                                }
                        }
                }
        }
    if (new_has_data.header.clock_subset_flag)
        {
            LOG(INFO) << "Received HAS clock subset corrections";
            for (const auto &gpseph : gps_ephemeris_map)
                {
                    int prn = gpseph.second.PRN;
                    int32_t sis_iod = gpseph.second.IODE_SF3;
                    int32_t gnss_iod = d_has_orbit_corrections_store_map[gps_str][prn].iod;
                    if (gnss_iod == sis_iod)
                        {
                            // d_has_clock_corrections_store_map[gps_str][prn].clock_correction_m = new_has_data.get_clock_subset_correction_mult_m(gps_str, prn);
                            // d_has_clock_corrections_store_map[gps_str][prn].valid_until = tmt + new_has_data.get_validity_interval_s(new_has_data.validity_interval_index_clock_subset_corrections);
                            // TODO: check for end of week
                        }
                }
        }
    if (new_has_data.header.code_bias_flag)
        {
            LOG(INFO) << "Received HAS code bias corrections";
            uint32_t valid_until = tmt +
                                   new_has_data.get_validity_interval_s(new_has_data.validity_interval_index_code_bias_corrections);
            auto signals_gal = new_has_data.get_signals_in_mask(gal_str);
            for (const auto &it : signals_gal)
                {
                    auto prns = new_has_data.get_PRNs_in_mask(gal_str);
                    for (auto prn : prns)
                        {
                            float code_bias_m = new_has_data.get_code_bias_m(it, prn);
                            if ((std::fabs(code_bias_m + 20.48) < 0.01))  // -20.48 means not available
                                {
                                    code_bias_m = 0.0;
                                }
                            d_has_code_bias_store_map[it][prn] = {code_bias_m, valid_until};
                        }
                }
            auto signals_gps = new_has_data.get_signals_in_mask(gps_str);
            for (const auto &it : signals_gps)
                {
                    auto prns = new_has_data.get_PRNs_in_mask(gps_str);
                    for (auto prn : prns)
                        {
                            float code_bias_m = new_has_data.get_code_bias_m(it, prn);
                            if ((std::fabs(code_bias_m + 20.48) < 0.01))  // -20.48 means not available
                                {
                                    code_bias_m = 0.0;
                                }
                            d_has_code_bias_store_map[it][prn] = {code_bias_m, valid_until};
                        }
                }
        }
    if (new_has_data.header.phase_bias_flag)
        {
            LOG(INFO) << "Received HAS phase bias corrections";
            uint32_t valid_until = tmt +
                                   new_has_data.get_validity_interval_s(new_has_data.validity_interval_index_phase_bias_corrections);

            auto signals_gal = new_has_data.get_signals_in_mask(gal_str);
            for (const auto &it : signals_gal)
                {
                    auto prns = new_has_data.get_PRNs_in_mask(gal_str);
                    for (auto prn : prns)
                        {
                            float phase_bias_correction_cycles = new_has_data.get_phase_bias_cycle(it, prn);
                            if (std::fabs(phase_bias_correction_cycles + 10.24) < 0.001)  // -10.24 means not available
                                {
                                    phase_bias_correction_cycles = 0.0;
                                }
                            d_has_phase_bias_store_map[it][prn] = {phase_bias_correction_cycles, valid_until};
                            // TODO: process Phase Discontinuity Indicator
                        }
                }
            auto signals_gps = new_has_data.get_signals_in_mask(gps_str);
            for (const auto &it : signals_gps)
                {
                    auto prns = new_has_data.get_PRNs_in_mask(gps_str);
                    for (auto prn : prns)
                        {
                            float phase_bias_correction_cycles = new_has_data.get_phase_bias_cycle(it, prn);
                            if (std::fabs(phase_bias_correction_cycles + 10.24) < 0.001)  // -10.24 means not available
                                {
                                    phase_bias_correction_cycles = 0.0;
                                }
                            d_has_phase_bias_store_map[it][prn] = {phase_bias_correction_cycles, valid_until};
                            // TODO: process Phase Discontinuity Indicator
                        }
                }
        }
}


void Rtklib_Solver::update_has_corrections(const std::map<int, Gnss_Synchro> &obs_map)
{
    this->check_has_orbit_clock_validity(obs_map);
    this->get_has_biases(obs_map);
}


void Rtklib_Solver::check_has_orbit_clock_validity(const std::map<int, Gnss_Synchro> &obs_map)
{
    for (const auto &it : obs_map)
        {
            uint32_t obs_tow = it.second.interp_TOW_ms / 1000.0;
            auto prn = static_cast<int>(it.second.PRN);

            if (it.second.System == 'G')
                {
                    auto it_sys = d_has_orbit_corrections_store_map.find("GPS");
                    if (it_sys != d_has_orbit_corrections_store_map.end())
                        {
                            auto it_map_corr = it_sys->second.find(prn);
                            if (it_map_corr != it_sys->second.end())
                                {
                                    auto has_data_valid_until = it_map_corr->second.valid_until;
                                    if (has_data_valid_until < obs_tow)
                                        {
                                            // Delete outdated data
                                            it_sys->second.erase(prn);
                                        }
                                }
                        }
                    auto it_sys_clock = d_has_clock_corrections_store_map.find("GPS");
                    if (it_sys_clock != d_has_clock_corrections_store_map.end())
                        {
                            auto it_map_corr = it_sys_clock->second.find(prn);
                            if (it_map_corr != it_sys_clock->second.end())
                                {
                                    auto has_data_valid_until = it_map_corr->second.valid_until;
                                    if (has_data_valid_until < obs_tow)
                                        {
                                            // Delete outdated data
                                            it_sys_clock->second.erase(prn);
                                        }
                                }
                        }
                }
            if (it.second.System == 'E')
                {
                    auto it_sys = d_has_orbit_corrections_store_map.find("Galileo");
                    if (it_sys != d_has_orbit_corrections_store_map.end())
                        {
                            auto it_map_corr = it_sys->second.find(prn);
                            if (it_map_corr != it_sys->second.end())
                                {
                                    auto has_data_valid_until = it_map_corr->second.valid_until;
                                    if (has_data_valid_until < obs_tow)
                                        {
                                            // Delete outdated data
                                            it_sys->second.erase(prn);
                                        }
                                }
                        }
                    auto it_sys_clock = d_has_clock_corrections_store_map.find("Galileo");
                    if (it_sys_clock != d_has_clock_corrections_store_map.end())
                        {
                            auto it_map_corr = it_sys_clock->second.find(prn);
                            if (it_map_corr != it_sys_clock->second.end())
                                {
                                    auto has_data_valid_until = it_map_corr->second.valid_until;
                                    if (has_data_valid_until < obs_tow)
                                        {
                                            // Delete outdated data
                                            it_sys_clock->second.erase(prn);
                                        }
                                }
                        }
                }
        }
}


void Rtklib_Solver::get_has_biases(const std::map<int, Gnss_Synchro> &obs_map)
{
    d_has_obs_corr_map.clear();
    if (!d_has_clock_corrections_store_map.empty() && !d_has_orbit_corrections_store_map.empty())
        {
            const std::vector<std::string> e1b_signals = {"E1-B I/NAV OS", "E1-C", "E1-B + E1-C"};
            const std::vector<std::string> e6_signals = {"E6-B C/NAV HAS", "E6-C", "E6-B + E6-C"};
            const std::vector<std::string> e5_signals = {"E5a-I F/NAV OS", "E5a-Q", "E5a-I+E5a-Q"};
            const std::vector<std::string> e7_signals = {"E5bI I/NAV OS", "E5b-Q", "E5b-I+E5b-Q"};
            const std::vector<std::string> g1c_signals = {"L1 C/A"};
            const std::vector<std::string> g2s_signals = {"L2 CM", "L2 CL", "L2 CM+CL", "L2 P"};
            const std::vector<std::string> g5_signals = {"L5 I", "L5 Q", "L5 I + L5 Q"};

            for (const auto &it : obs_map)
                {
                    uint32_t obs_tow = it.second.interp_TOW_ms / 1000.0;
                    int prn = static_cast<int>(it.second.PRN);
                    std::string sig(it.second.Signal, 2);
                    if (it.second.System == 'E')
                        {
                            auto it_sys_clock = d_has_clock_corrections_store_map.find("Galileo");
                            if (it_sys_clock != d_has_clock_corrections_store_map.end())
                                {
                                    auto it_map_corr = it_sys_clock->second.find(prn);
                                    if (it_map_corr != it_sys_clock->second.end())
                                        {
                                            if (sig == "1B")
                                                {
                                                    for (const auto &has_signal : e1b_signals)
                                                        {
                                                            this->get_current_has_obs_correction(has_signal, obs_tow, prn);
                                                        }
                                                }
                                            else if (sig == "E6")
                                                {
                                                    for (const auto &has_signal : e6_signals)
                                                        {
                                                            this->get_current_has_obs_correction(has_signal, obs_tow, prn);
                                                        }
                                                }
                                            else if (sig == "5X")
                                                {
                                                    for (const auto &has_signal : e5_signals)
                                                        {
                                                            this->get_current_has_obs_correction(has_signal, obs_tow, prn);
                                                        }
                                                }
                                            else if (sig == "7X")
                                                {
                                                    for (const auto &has_signal : e7_signals)
                                                        {
                                                            this->get_current_has_obs_correction(has_signal, obs_tow, prn);
                                                        }
                                                }
                                        }
                                }
                        }
                    if (it.second.System == 'G')
                        {
                            auto it_sys_clock = d_has_clock_corrections_store_map.find("GPS");
                            if (it_sys_clock != d_has_clock_corrections_store_map.end())
                                {
                                    auto it_map_corr = it_sys_clock->second.find(prn);
                                    if (it_map_corr != it_sys_clock->second.end())
                                        {
                                            if (sig == "1C")
                                                {
                                                    for (const auto &has_signal : g1c_signals)
                                                        {
                                                            this->get_current_has_obs_correction(has_signal, obs_tow, prn);
                                                        }
                                                }
                                            else if (sig == "2S")
                                                {
                                                    for (const auto &has_signal : g2s_signals)
                                                        {
                                                            this->get_current_has_obs_correction(has_signal, obs_tow, prn);
                                                        }
                                                }
                                            else if (sig == "L5")
                                                {
                                                    for (const auto &has_signal : g5_signals)
                                                        {
                                                            this->get_current_has_obs_correction(has_signal, obs_tow, prn);
                                                        }
                                                }
                                        }
                                }
                        }
                }
        }
}


void Rtklib_Solver::get_current_has_obs_correction(const std::string &signal, uint32_t tow_obs, int prn)
{
    auto code_bias_pair_it = this->d_has_code_bias_store_map[signal].find(prn);
    if (code_bias_pair_it != this->d_has_code_bias_store_map[signal].end())
        {
            uint32_t valid_until = code_bias_pair_it->second.second;
            if (valid_until > tow_obs)
                {
                    this->d_has_obs_corr_map[signal][prn].code_bias_m = code_bias_pair_it->second.first;
                }
        }
    auto phase_bias_pair_it = this->d_has_phase_bias_store_map[signal].find(prn);
    if (phase_bias_pair_it != this->d_has_phase_bias_store_map[signal].end())
        {
            uint32_t valid_until = phase_bias_pair_it->second.second;
            if (valid_until > tow_obs)
                {
                    this->d_has_obs_corr_map[signal][prn].phase_bias_cycle = phase_bias_pair_it->second.first;
                }
        }
}


bool Rtklib_Solver::get_PVT(const std::map<int, Gnss_Synchro> &gnss_observables_map, bool flag_averaging)
{
    std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
    std::map<int, Galileo_Ephemeris>::const_iterator galileo_ephemeris_iter;
    std::map<int, Gps_Ephemeris>::const_iterator gps_ephemeris_iter;
    std::map<int, Gps_CNAV_Ephemeris>::const_iterator gps_cnav_ephemeris_iter;
    std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_ephemeris_iter;
    std::map<int, Beidou_Dnav_Ephemeris>::const_iterator beidou_ephemeris_iter;

    const Glonass_Gnav_Utc_Model &gnav_utc = this->glonass_gnav_utc_model;

    this->set_averaging_flag(flag_averaging);

    // ********************************************************************************
    // ****** PREPARE THE DATA (SV EPHEMERIS AND OBSERVATIONS) ************************
    // ********************************************************************************
    int valid_obs = 0;      // valid observations counter
    int glo_valid_obs = 0;  // GLONASS L1/L2 valid observations counter

    d_obs_data.fill({});
    std::vector<eph_t> eph_data(MAXOBS);
    std::vector<geph_t> geph_data(MAXOBS);

    // Workaround for NAV/CNAV clash problem
    bool gps_dual_band = false;
    bool band1 = false;
    bool band2 = false;

    for (gnss_observables_iter = gnss_observables_map.cbegin();
         gnss_observables_iter != gnss_observables_map.cend();
         ++gnss_observables_iter)
        {
            switch (gnss_observables_iter->second.System)
                {
                case 'G':
                    {
                        const std::string sig_(gnss_observables_iter->second.Signal, 2);
                        if (sig_ == "1C")
                            {
                                band1 = true;
                            }
                        if (sig_ == "2S")
                            {
                                band2 = true;
                            }
                    }
                    break;
                default:
                    {
                    }
                }
        }
    if (band1 == true and band2 == true)
        {
            gps_dual_band = true;
        }

    for (gnss_observables_iter = gnss_observables_map.cbegin();
         gnss_observables_iter != gnss_observables_map.cend();
         ++gnss_observables_iter)  // CHECK INCONSISTENCY when combining GLONASS + other system
        {
            switch (gnss_observables_iter->second.System)
                {
                case 'E':
                    {
                        const std::string gal_str("Galileo");
                        const std::string sig_(gnss_observables_iter->second.Signal, 2);
                        // Galileo E1
                        if (sig_ == "1B")
                            {
                                // 1 Gal - find the ephemeris for the current GALILEO SV observation. The SV PRN ID is the map key
                                galileo_ephemeris_iter = galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (galileo_ephemeris_iter != galileo_ephemeris_map.cend())
                                    {
                                        // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                        eph_data[valid_obs] = eph_to_rtklib(galileo_ephemeris_iter->second,
                                            this->d_has_orbit_corrections_store_map[gal_str],
                                            this->d_has_clock_corrections_store_map[gal_str]);
                                        // convert observation from GNSS-SDR class to RTKLIB structure
                                        obsd_t newobs{};
                                        d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                            gnss_observables_iter->second,
                                            d_has_obs_corr_map,
                                            galileo_ephemeris_iter->second.WN,
                                            d_rtklib_band_index[sig_]);
                                        valid_obs++;
                                    }
                                else  // the ephemeris are not available for this SV
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                                    }
                            }

                        // Galileo E5
                        if ((sig_ == "5X") || (sig_ == "7X"))
                            {
                                // 1 Gal - find the ephemeris for the current GALILEO SV observation. The SV PRN ID is the map key
                                galileo_ephemeris_iter = galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (galileo_ephemeris_iter != galileo_ephemeris_map.cend())
                                    {
                                        bool found_E1_obs = false;
                                        for (int i = 0; i < valid_obs; i++)
                                            {
                                                if (eph_data[i].sat == (static_cast<int>(gnss_observables_iter->second.PRN + NSATGPS + NSATGLO)))
                                                    {
                                                        d_obs_data[i + glo_valid_obs] = insert_obs_to_rtklib(d_obs_data[i + glo_valid_obs],
                                                            gnss_observables_iter->second,
                                                            d_has_obs_corr_map,
                                                            galileo_ephemeris_iter->second.WN,
                                                            d_rtklib_band_index[sig_]);
                                                        found_E1_obs = true;
                                                        break;
                                                    }
                                            }
                                        if (!found_E1_obs)
                                            {
                                                // insert Galileo E5 obs as new obs and also insert its ephemeris
                                                // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                                eph_data[valid_obs] = eph_to_rtklib(galileo_ephemeris_iter->second,
                                                    this->d_has_orbit_corrections_store_map[gal_str],
                                                    this->d_has_clock_corrections_store_map[gal_str]);
                                                // convert observation from GNSS-SDR class to RTKLIB structure
                                                const auto default_code_ = static_cast<unsigned char>(CODE_NONE);
                                                obsd_t newobs = {{0, 0}, '0', '0', {}, {},
                                                    {default_code_, default_code_, default_code_},
                                                    {}, {0.0, 0.0, 0.0}, {}};
                                                d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    d_has_obs_corr_map,
                                                    galileo_ephemeris_iter->second.WN,
                                                    d_rtklib_band_index[sig_]);
                                                valid_obs++;
                                            }
                                    }
                                else  // the ephemeris are not available for this SV
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                                    }
                            }
                        if (sig_ == "E6" && d_use_e6_for_pvt)
                            {
                                galileo_ephemeris_iter = galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (galileo_ephemeris_iter != galileo_ephemeris_map.cend())
                                    {
                                        bool found_E1_obs = false;
                                        for (int i = 0; i < valid_obs; i++)
                                            {
                                                if (eph_data[i].sat == (static_cast<int>(gnss_observables_iter->second.PRN + NSATGPS + NSATGLO)))
                                                    {
                                                        d_obs_data[i + glo_valid_obs] = insert_obs_to_rtklib(d_obs_data[i + glo_valid_obs],
                                                            gnss_observables_iter->second,
                                                            d_has_obs_corr_map,
                                                            galileo_ephemeris_iter->second.WN,
                                                            d_rtklib_band_index[sig_]);
                                                        found_E1_obs = true;
                                                        break;
                                                    }
                                            }
                                        if (!found_E1_obs)
                                            {
                                                // insert Galileo E6 obs as new obs and also insert its ephemeris
                                                // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                                eph_data[valid_obs] = eph_to_rtklib(galileo_ephemeris_iter->second,
                                                    this->d_has_orbit_corrections_store_map[gal_str],
                                                    this->d_has_clock_corrections_store_map[gal_str]);
                                                // convert observation from GNSS-SDR class to RTKLIB structure
                                                const auto default_code_ = static_cast<unsigned char>(CODE_NONE);
                                                obsd_t newobs = {{0, 0}, '0', '0', {}, {},
                                                    {default_code_, default_code_, default_code_},
                                                    {}, {0.0, 0.0, 0.0}, {}};
                                                d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    d_has_obs_corr_map,
                                                    galileo_ephemeris_iter->second.WN,
                                                    d_rtklib_band_index[sig_]);
                                                valid_obs++;
                                            }
                                    }
                                else  // the ephemeris are not available for this SV
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                                    }
                            }
                        break;
                    }
                case 'G':
                    {
                        // GPS L1
                        // 1 GPS - find the ephemeris for the current GPS SV observation. The SV PRN ID is the map key
                        const std::string gps_str("GPS");
                        const std::string sig_(gnss_observables_iter->second.Signal, 2);
                        if (sig_ == "1C")
                            {
                                gps_ephemeris_iter = gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (gps_ephemeris_iter != gps_ephemeris_map.cend())
                                    {
                                        // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                        eph_data[valid_obs] = eph_to_rtklib(gps_ephemeris_iter->second,
                                            this->d_has_orbit_corrections_store_map[gps_str],
                                            this->d_has_clock_corrections_store_map[gps_str],
                                            this->is_pre_2009());
                                        // convert observation from GNSS-SDR class to RTKLIB structure
                                        obsd_t newobs{};
                                        d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                            gnss_observables_iter->second,
                                            d_has_obs_corr_map,
                                            gps_ephemeris_iter->second.WN,
                                            d_rtklib_band_index[sig_],
                                            this->is_pre_2009());
                                        valid_obs++;
                                    }
                                else  // the ephemeris are not available for this SV
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->first;
                                    }
                            }
                        // GPS L2 (todo: solve NAV/CNAV clash)
                        if ((sig_ == "2S") and (gps_dual_band == false))
                            {
                                gps_cnav_ephemeris_iter = gps_cnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (gps_cnav_ephemeris_iter != gps_cnav_ephemeris_map.cend())
                                    {
                                        // 1. Find the same satellite in GPS L1 band
                                        gps_ephemeris_iter = gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                        if (gps_ephemeris_iter != gps_ephemeris_map.cend())
                                            {
                                                /* By the moment, GPS L2 observables are not used in pseudorange computations if GPS L1 is available
                                                // 2. If found, replace the existing GPS L1 ephemeris with the GPS L2 ephemeris
                                                // (more precise!), and attach the L2 observation to the L1 observation in RTKLIB structure
                                                for (int i = 0; i < valid_obs; i++)
                                                    {
                                                        if (eph_data[i].sat == static_cast<int>(gnss_observables_iter->second.PRN))
                                                            {
                                                                eph_data[i] = eph_to_rtklib(gps_cnav_ephemeris_iter->second);
                                                                d_obs_data[i + glo_valid_obs] = insert_obs_to_rtklib(d_obs_data[i + glo_valid_obs],
                                                                    gnss_observables_iter->second,
                                                                    eph_data[i].week,
                                                                    d_rtklib_band_index[sig_]);
                                                                break;
                                                            }
                                                    }
                                                */
                                            }
                                        else
                                            {
                                                // 3. If not found, insert the GPS L2 ephemeris and the observation
                                                // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                                eph_data[valid_obs] = eph_to_rtklib(gps_cnav_ephemeris_iter->second);
                                                // convert observation from GNSS-SDR class to RTKLIB structure
                                                const auto default_code_ = static_cast<unsigned char>(CODE_NONE);
                                                obsd_t newobs = {{0, 0}, '0', '0', {}, {},
                                                    {default_code_, default_code_, default_code_},
                                                    {}, {0.0, 0.0, 0.0}, {}};
                                                d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    gps_cnav_ephemeris_iter->second.WN,
                                                    d_rtklib_band_index[sig_]);
                                                valid_obs++;
                                            }
                                    }
                                else  // the ephemeris are not available for this SV
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                                    }
                            }
                        // GPS L5
                        if (sig_ == "L5")
                            {
                                gps_cnav_ephemeris_iter = gps_cnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (gps_cnav_ephemeris_iter != gps_cnav_ephemeris_map.cend())
                                    {
                                        // 1. Find the same satellite in GPS L1 band
                                        gps_ephemeris_iter = gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                        if (gps_ephemeris_iter != gps_ephemeris_map.cend())
                                            {
                                                // 2. If found, replace the existing GPS L1 ephemeris with the GPS L5 ephemeris
                                                // (more precise!), and attach the L5 observation to the L1 observation in RTKLIB structure
                                                for (int i = 0; i < valid_obs; i++)
                                                    {
                                                        if (eph_data[i].sat == static_cast<int>(gnss_observables_iter->second.PRN))
                                                            {
                                                                eph_data[i] = eph_to_rtklib(gps_cnav_ephemeris_iter->second);
                                                                d_obs_data[i + glo_valid_obs] = insert_obs_to_rtklib(d_obs_data[i],
                                                                    gnss_observables_iter->second,
                                                                    gps_cnav_ephemeris_iter->second.WN,
                                                                    d_rtklib_band_index[sig_]);
                                                                break;
                                                            }
                                                    }
                                            }
                                        else
                                            {
                                                // 3. If not found, insert the GPS L5 ephemeris and the observation
                                                // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                                eph_data[valid_obs] = eph_to_rtklib(gps_cnav_ephemeris_iter->second);
                                                // convert observation from GNSS-SDR class to RTKLIB structure
                                                const auto default_code_ = static_cast<unsigned char>(CODE_NONE);
                                                obsd_t newobs = {{0, 0}, '0', '0', {}, {},
                                                    {default_code_, default_code_, default_code_},
                                                    {}, {0.0, 0.0, 0.0}, {}};
                                                d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    d_has_obs_corr_map,
                                                    gps_cnav_ephemeris_iter->second.WN,
                                                    d_rtklib_band_index[sig_]);
                                                valid_obs++;
                                            }
                                    }
                                else  // the ephemeris are not available for this SV
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                                    }
                            }
                        break;
                    }
                case 'R':  // TODO This should be using rtk lib nomenclature
                    {
                        const std::string sig_(gnss_observables_iter->second.Signal);
                        // GLONASS GNAV L1
                        if (sig_ == "1G")
                            {
                                // 1 Glo - find the ephemeris for the current GLONASS SV observation. The SV Slot Number (PRN ID) is the map key
                                glonass_gnav_ephemeris_iter = glonass_gnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (glonass_gnav_ephemeris_iter != glonass_gnav_ephemeris_map.cend())
                                    {
                                        // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                        geph_data[glo_valid_obs] = eph_to_rtklib(glonass_gnav_ephemeris_iter->second, gnav_utc);
                                        // convert observation from GNSS-SDR class to RTKLIB structure
                                        obsd_t newobs{};
                                        d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                            gnss_observables_iter->second,
                                            glonass_gnav_ephemeris_iter->second.d_WN,
                                            d_rtklib_band_index[sig_]);
                                        glo_valid_obs++;
                                    }
                                else  // the ephemeris are not available for this SV
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                                    }
                            }
                        // GLONASS GNAV L2
                        if (sig_ == "2G")
                            {
                                // 1 GLONASS - find the ephemeris for the current GLONASS SV observation. The SV PRN ID is the map key
                                glonass_gnav_ephemeris_iter = glonass_gnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (glonass_gnav_ephemeris_iter != glonass_gnav_ephemeris_map.cend())
                                    {
                                        bool found_L1_obs = false;
                                        for (int i = 0; i < glo_valid_obs; i++)
                                            {
                                                if (geph_data[i].sat == (static_cast<int>(gnss_observables_iter->second.PRN + NSATGPS)))
                                                    {
                                                        d_obs_data[i + valid_obs] = insert_obs_to_rtklib(d_obs_data[i + valid_obs],
                                                            gnss_observables_iter->second,
                                                            glonass_gnav_ephemeris_iter->second.d_WN,
                                                            d_rtklib_band_index[sig_]);
                                                        found_L1_obs = true;
                                                        break;
                                                    }
                                            }
                                        if (!found_L1_obs)
                                            {
                                                // insert GLONASS GNAV L2 obs as new obs and also insert its ephemeris
                                                // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                                geph_data[glo_valid_obs] = eph_to_rtklib(glonass_gnav_ephemeris_iter->second, gnav_utc);
                                                // convert observation from GNSS-SDR class to RTKLIB structure
                                                obsd_t newobs{};
                                                d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    glonass_gnav_ephemeris_iter->second.d_WN,
                                                    d_rtklib_band_index[sig_]);
                                                glo_valid_obs++;
                                            }
                                    }
                                else  // the ephemeris are not available for this SV
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                                    }
                            }
                        break;
                    }
                case 'C':
                    {
                        // BEIDOU B1I
                        //  - find the ephemeris for the current BEIDOU SV observation. The SV PRN ID is the map key
                        const std::string sig_(gnss_observables_iter->second.Signal);
                        if (sig_ == "B1")
                            {
                                beidou_ephemeris_iter = beidou_dnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (beidou_ephemeris_iter != beidou_dnav_ephemeris_map.cend())
                                    {
                                        // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                        eph_data[valid_obs] = eph_to_rtklib(beidou_ephemeris_iter->second);
                                        // convert observation from GNSS-SDR class to RTKLIB structure
                                        obsd_t newobs{};
                                        d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                            gnss_observables_iter->second,
                                            beidou_ephemeris_iter->second.WN + BEIDOU_DNAV_BDT2GPST_WEEK_NUM_OFFSET,
                                            d_rtklib_band_index[sig_]);
                                        valid_obs++;
                                    }
                                else  // the ephemeris are not available for this SV
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->first;
                                    }
                            }
                        // BeiDou B3
                        if (sig_ == "B3")
                            {
                                beidou_ephemeris_iter = beidou_dnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (beidou_ephemeris_iter != beidou_dnav_ephemeris_map.cend())
                                    {
                                        bool found_B1I_obs = false;
                                        for (int i = 0; i < valid_obs; i++)
                                            {
                                                if (eph_data[i].sat == (static_cast<int>(gnss_observables_iter->second.PRN + NSATGPS + NSATGLO + NSATGAL + NSATQZS)))
                                                    {
                                                        d_obs_data[i + glo_valid_obs] = insert_obs_to_rtklib(d_obs_data[i + glo_valid_obs],
                                                            gnss_observables_iter->second,
                                                            beidou_ephemeris_iter->second.WN + BEIDOU_DNAV_BDT2GPST_WEEK_NUM_OFFSET,
                                                            d_rtklib_band_index[sig_]);
                                                        found_B1I_obs = true;
                                                        break;
                                                    }
                                            }
                                        if (!found_B1I_obs)
                                            {
                                                // insert BeiDou B3I obs as new obs and also insert its ephemeris
                                                // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                                eph_data[valid_obs] = eph_to_rtklib(beidou_ephemeris_iter->second);
                                                // convert observation from GNSS-SDR class to RTKLIB structure
                                                const auto default_code_ = static_cast<unsigned char>(CODE_NONE);
                                                obsd_t newobs = {{0, 0}, '0', '0', {}, {},
                                                    {default_code_, default_code_, default_code_},
                                                    {}, {0.0, 0.0, 0.0}, {}};
                                                d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    beidou_ephemeris_iter->second.WN + BEIDOU_DNAV_BDT2GPST_WEEK_NUM_OFFSET,
                                                    d_rtklib_band_index[sig_]);
                                                valid_obs++;
                                            }
                                    }
                                else  // the ephemeris are not available for this SV
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                                    }
                            }
                        break;
                    }

                default:
                    DLOG(INFO) << "Hybrid observables: Unknown GNSS";
                    break;
                }
        }

    // **********************************************************************
    // ****** SOLVE PVT******************************************************
    // **********************************************************************

    this->set_valid_position(false);
    if ((valid_obs + glo_valid_obs) > 3)
        {
            int result = 0;
            d_nav_data = {};
            d_nav_data.eph = eph_data.data();
            d_nav_data.geph = geph_data.data();
            d_nav_data.n = valid_obs;
            d_nav_data.ng = glo_valid_obs;
            if (gps_iono.valid)
                {
                    d_nav_data.ion_gps[0] = gps_iono.alpha0;
                    d_nav_data.ion_gps[1] = gps_iono.alpha1;
                    d_nav_data.ion_gps[2] = gps_iono.alpha2;
                    d_nav_data.ion_gps[3] = gps_iono.alpha3;
                    d_nav_data.ion_gps[4] = gps_iono.beta0;
                    d_nav_data.ion_gps[5] = gps_iono.beta1;
                    d_nav_data.ion_gps[6] = gps_iono.beta2;
                    d_nav_data.ion_gps[7] = gps_iono.beta3;
                }
            if (!(gps_iono.valid) and gps_cnav_iono.valid)
                {
                    d_nav_data.ion_gps[0] = gps_cnav_iono.alpha0;
                    d_nav_data.ion_gps[1] = gps_cnav_iono.alpha1;
                    d_nav_data.ion_gps[2] = gps_cnav_iono.alpha2;
                    d_nav_data.ion_gps[3] = gps_cnav_iono.alpha3;
                    d_nav_data.ion_gps[4] = gps_cnav_iono.beta0;
                    d_nav_data.ion_gps[5] = gps_cnav_iono.beta1;
                    d_nav_data.ion_gps[6] = gps_cnav_iono.beta2;
                    d_nav_data.ion_gps[7] = gps_cnav_iono.beta3;
                }
            if (galileo_iono.ai0 != 0.0)
                {
                    d_nav_data.ion_gal[0] = galileo_iono.ai0;
                    d_nav_data.ion_gal[1] = galileo_iono.ai1;
                    d_nav_data.ion_gal[2] = galileo_iono.ai2;
                    d_nav_data.ion_gal[3] = 0.0;
                }
            if (beidou_dnav_iono.valid)
                {
                    d_nav_data.ion_cmp[0] = beidou_dnav_iono.alpha0;
                    d_nav_data.ion_cmp[1] = beidou_dnav_iono.alpha1;
                    d_nav_data.ion_cmp[2] = beidou_dnav_iono.alpha2;
                    d_nav_data.ion_cmp[3] = beidou_dnav_iono.alpha3;
                    d_nav_data.ion_cmp[4] = beidou_dnav_iono.beta0;
                    d_nav_data.ion_cmp[5] = beidou_dnav_iono.beta0;
                    d_nav_data.ion_cmp[6] = beidou_dnav_iono.beta0;
                    d_nav_data.ion_cmp[7] = beidou_dnav_iono.beta3;
                }
            if (gps_utc_model.valid)
                {
                    d_nav_data.utc_gps[0] = gps_utc_model.A0;
                    d_nav_data.utc_gps[1] = gps_utc_model.A1;
                    d_nav_data.utc_gps[2] = gps_utc_model.tot;
                    d_nav_data.utc_gps[3] = gps_utc_model.WN_T;
                    d_nav_data.leaps = gps_utc_model.DeltaT_LS;
                }
            if (!(gps_utc_model.valid) and gps_cnav_utc_model.valid)
                {
                    d_nav_data.utc_gps[0] = gps_cnav_utc_model.A0;
                    d_nav_data.utc_gps[1] = gps_cnav_utc_model.A1;
                    d_nav_data.utc_gps[2] = gps_cnav_utc_model.tot;
                    d_nav_data.utc_gps[3] = gps_cnav_utc_model.WN_T;
                    d_nav_data.leaps = gps_cnav_utc_model.DeltaT_LS;
                }
            if (glonass_gnav_utc_model.valid)
                {
                    d_nav_data.utc_glo[0] = glonass_gnav_utc_model.d_tau_c;  // ??
                    d_nav_data.utc_glo[1] = 0.0;                             // ??
                    d_nav_data.utc_glo[2] = 0.0;                             // ??
                    d_nav_data.utc_glo[3] = 0.0;                             // ??
                }
            if (galileo_utc_model.A0 != 0.0)
                {
                    d_nav_data.utc_gal[0] = galileo_utc_model.A0;
                    d_nav_data.utc_gal[1] = galileo_utc_model.A1;
                    d_nav_data.utc_gal[2] = galileo_utc_model.tot;
                    d_nav_data.utc_gal[3] = galileo_utc_model.WNot;
                    d_nav_data.leaps = galileo_utc_model.Delta_tLS;
                }
            if (beidou_dnav_utc_model.valid)
                {
                    d_nav_data.utc_cmp[0] = beidou_dnav_utc_model.A0_UTC;
                    d_nav_data.utc_cmp[1] = beidou_dnav_utc_model.A1_UTC;
                    d_nav_data.utc_cmp[2] = 0.0;  // ??
                    d_nav_data.utc_cmp[3] = 0.0;  // ??
                    d_nav_data.leaps = beidou_dnav_utc_model.DeltaT_LS;
                }

            /* update carrier wave length using native function call in RTKlib */
            for (int i = 0; i < MAXSAT; i++)
                {
                    for (int j = 0; j < NFREQ; j++)
                        {
                            d_nav_data.lam[i][j] = satwavelen(i + 1, d_rtklib_freq_index[j], &d_nav_data);
                        }
                }

            result = rtkpos(&d_rtk, d_obs_data.data(), valid_obs + glo_valid_obs, &d_nav_data);

            if (result == 0)
                {
                    LOG(INFO) << "RTKLIB rtkpos error: " << d_rtk.errbuf;
                    d_rtk.neb = 0;                 // clear error buffer to avoid repeating the error message
                    this->set_time_offset_s(0.0);  // reset rx time estimation
                    this->set_num_valid_observations(0);
                }
            else
                {
                    this->set_num_valid_observations(d_rtk.sol.ns);  // record the number of valid satellites used by the PVT solver
                    pvt_sol = d_rtk.sol;
                    // DOP computation
                    unsigned int used_sats = 0;
                    for (unsigned int i = 0; i < MAXSAT; i++)
                        {
                            pvt_ssat[i] = d_rtk.ssat[i];
                            if (d_rtk.ssat[i].vs == 1)
                                {
                                    used_sats++;
                                }
                        }

                    std::vector<double> azel(used_sats * 2);
                    int index_aux = 0;
                    for (auto &i : d_rtk.ssat)
                        {
                            if (i.vs == 1)
                                {
                                    azel[2 * index_aux] = i.azel[0];
                                    azel[2 * index_aux + 1] = i.azel[1];
                                    index_aux++;
                                }
                        }

                    if (index_aux > 0)
                        {
                            dops(index_aux, azel.data(), 0.0, d_dop.data());
                        }
                    this->set_valid_position(true);
                    std::array<double, 4> rx_position_and_time{};
                    rx_position_and_time[0] = pvt_sol.rr[0];  // [m]
                    rx_position_and_time[1] = pvt_sol.rr[1];  // [m]
                    rx_position_and_time[2] = pvt_sol.rr[2];  // [m]
                    // todo: fix this ambiguity in the RTKLIB units in receiver clock offset!
                    if (d_rtk.opt.mode == PMODE_SINGLE)
                        {
                            // if the RTKLIB solver is set to SINGLE, the dtr is already expressed in [s]
                            // add also the clock offset from gps to galileo (pvt_sol.dtr[2])
                            rx_position_and_time[3] = pvt_sol.dtr[0] + pvt_sol.dtr[2];
                        }
                    else
                        {
                            // the receiver clock offset is expressed in [meters], so we convert it into [s]
                            // add also the clock offset from gps to galileo (pvt_sol.dtr[2])
                            rx_position_and_time[3] = pvt_sol.dtr[2] + pvt_sol.dtr[0] / SPEED_OF_LIGHT_M_S;
                        }
                    this->set_rx_pos({rx_position_and_time[0], rx_position_and_time[1], rx_position_and_time[2]});  // save ECEF position for the next iteration

                    // compute Ground speed and COG
                    double ground_speed_ms = 0.0;
                    std::array<double, 3> pos{};
                    std::array<double, 3> enuv{};
                    ecef2pos(pvt_sol.rr, pos.data());
                    ecef2enu(pos.data(), &pvt_sol.rr[3], enuv.data());
                    this->set_speed_over_ground(norm_rtk(enuv.data(), 2));
                    double new_cog;
                    if (ground_speed_ms >= 1.0)
                        {
                            new_cog = atan2(enuv[0], enuv[1]) * R2D;
                            if (new_cog < 0.0)
                                {
                                    new_cog += 360.0;
                                }
                            this->set_course_over_ground(new_cog);
                        }

                    this->set_time_offset_s(rx_position_and_time[3]);

                    DLOG(INFO) << "RTKLIB Position at RX TOW = " << gnss_observables_map.cbegin()->second.RX_time
                               << " in ECEF (X,Y,Z,t[meters]) = " << rx_position_and_time[0] << ", " << rx_position_and_time[1] << ", " << rx_position_and_time[2] << ", " << rx_position_and_time[3];

                    // gtime_t rtklib_utc_time = gpst2utc(pvt_sol.time); // Corrected RX Time (Non integer multiply of 1 ms of granularity)
                    // Uncorrected RX Time (integer multiply of 1 ms and the same observables time reported in RTCM and RINEX)
                    const gtime_t rtklib_time = timeadd(pvt_sol.time, rx_position_and_time[3]);  // uncorrected rx time
                    const gtime_t rtklib_utc_time = gpst2utc(rtklib_time);
                    boost::posix_time::ptime p_time = boost::posix_time::from_time_t(rtklib_utc_time.time);
                    p_time += boost::posix_time::microseconds(static_cast<long>(round(rtklib_utc_time.sec * 1e6)));  // NOLINT(google-runtime-int)

                    this->set_position_UTC_time(p_time);

                    DLOG(INFO) << "RTKLIB Position at " << boost::posix_time::to_simple_string(p_time)
                               << " is Lat = " << this->get_latitude() << " [deg], Long = " << this->get_longitude()
                               << " [deg], Height= " << this->get_height() << " [m]"
                               << " RX time offset= " << this->get_time_offset_s() << " [s]";

                    // ######## PVT MONITOR #########
                    // TOW
                    d_monitor_pvt.TOW_at_current_symbol_ms = gnss_observables_map.cbegin()->second.TOW_at_current_symbol_ms;
                    // WEEK
                    d_monitor_pvt.week = adjgpsweek(d_nav_data.eph[0].week, this->is_pre_2009());
                    // PVT GPS time
                    d_monitor_pvt.RX_time = gnss_observables_map.cbegin()->second.RX_time;
                    // User clock offset [s]
                    d_monitor_pvt.user_clk_offset = rx_position_and_time[3];

                    // ECEF POS X,Y,X [m] + ECEF VEL X,Y,X [m/s] (6 x double)
                    d_monitor_pvt.pos_x = pvt_sol.rr[0];
                    d_monitor_pvt.pos_y = pvt_sol.rr[1];
                    d_monitor_pvt.pos_z = pvt_sol.rr[2];
                    d_monitor_pvt.vel_x = pvt_sol.rr[3];
                    d_monitor_pvt.vel_y = pvt_sol.rr[4];
                    d_monitor_pvt.vel_z = pvt_sol.rr[5];

                    // position variance/covariance (m^2) {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx} (6 x double)
                    d_monitor_pvt.cov_xx = pvt_sol.qr[0];
                    d_monitor_pvt.cov_yy = pvt_sol.qr[1];
                    d_monitor_pvt.cov_zz = pvt_sol.qr[2];
                    d_monitor_pvt.cov_xy = pvt_sol.qr[3];
                    d_monitor_pvt.cov_yz = pvt_sol.qr[4];
                    d_monitor_pvt.cov_zx = pvt_sol.qr[5];

                    // GEO user position Latitude [deg]
                    d_monitor_pvt.latitude = this->get_latitude();
                    // GEO user position Longitude [deg]
                    d_monitor_pvt.longitude = this->get_longitude();
                    // GEO user position Height [m]
                    d_monitor_pvt.height = this->get_height();

                    // NUMBER OF VALID SATS
                    d_monitor_pvt.valid_sats = pvt_sol.ns;
                    // RTKLIB solution status
                    d_monitor_pvt.solution_status = pvt_sol.stat;
                    // RTKLIB solution type (0:xyz-ecef,1:enu-baseline)
                    d_monitor_pvt.solution_type = pvt_sol.type;
                    // AR ratio factor for validation
                    d_monitor_pvt.AR_ratio_factor = pvt_sol.ratio;
                    // AR ratio threshold for validation
                    d_monitor_pvt.AR_ratio_threshold = pvt_sol.thres;

                    // GDOP / PDOP/ HDOP/ VDOP
                    d_monitor_pvt.gdop = d_dop[0];
                    d_monitor_pvt.pdop = d_dop[1];
                    d_monitor_pvt.hdop = d_dop[2];
                    d_monitor_pvt.vdop = d_dop[3];

                    this->set_rx_vel({enuv[0], enuv[1], enuv[2]});

                    const double clock_drift_ppm = pvt_sol.dtr[5] / SPEED_OF_LIGHT_M_S * 1e6;

                    this->set_clock_drift_ppm(clock_drift_ppm);
                    // User clock drift [ppm]
                    d_monitor_pvt.user_clk_drift_ppm = clock_drift_ppm;

                    // ######## LOG FILE #########
                    if (d_flag_dump_enabled == true)
                        {
                            // MULTIPLEXED FILE RECORDING - Record results to file
                            try
                                {
                                    double tmp_double;
                                    uint32_t tmp_uint32;
                                    // TOW
                                    tmp_uint32 = gnss_observables_map.cbegin()->second.TOW_at_current_symbol_ms;
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_uint32), sizeof(uint32_t));
                                    // WEEK
                                    tmp_uint32 = adjgpsweek(d_nav_data.eph[0].week, this->is_pre_2009());
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_uint32), sizeof(uint32_t));
                                    // PVT GPS time
                                    tmp_double = gnss_observables_map.cbegin()->second.RX_time;
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                                    // User clock offset [s]
                                    tmp_double = rx_position_and_time[3];
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));

                                    // ECEF POS X,Y,X [m] + ECEF VEL X,Y,X [m/s] (6 x double)
                                    tmp_double = pvt_sol.rr[0];
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                                    tmp_double = pvt_sol.rr[1];
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                                    tmp_double = pvt_sol.rr[2];
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                                    tmp_double = pvt_sol.rr[3];
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                                    tmp_double = pvt_sol.rr[4];
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                                    tmp_double = pvt_sol.rr[5];
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));

                                    // position variance/covariance (m^2) {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx} (6 x double)
                                    tmp_double = pvt_sol.qr[0];
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                                    tmp_double = pvt_sol.qr[1];
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                                    tmp_double = pvt_sol.qr[2];
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                                    tmp_double = pvt_sol.qr[3];
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                                    tmp_double = pvt_sol.qr[4];
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                                    tmp_double = pvt_sol.qr[5];
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));

                                    // GEO user position Latitude [deg]
                                    tmp_double = this->get_latitude();
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                                    // GEO user position Longitude [deg]
                                    tmp_double = this->get_longitude();
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                                    // GEO user position Height [m]
                                    tmp_double = this->get_height();
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));

                                    // NUMBER OF VALID SATS
                                    d_dump_file.write(reinterpret_cast<char *>(&pvt_sol.ns), sizeof(uint8_t));
                                    // RTKLIB solution status
                                    d_dump_file.write(reinterpret_cast<char *>(&pvt_sol.stat), sizeof(uint8_t));
                                    // RTKLIB solution type (0:xyz-ecef,1:enu-baseline)
                                    d_dump_file.write(reinterpret_cast<char *>(&pvt_sol.type), sizeof(uint8_t));
                                    // AR ratio factor for validation
                                    d_dump_file.write(reinterpret_cast<char *>(&pvt_sol.ratio), sizeof(float));
                                    // AR ratio threshold for validation
                                    d_dump_file.write(reinterpret_cast<char *>(&pvt_sol.thres), sizeof(float));

                                    // GDOP / PDOP / HDOP / VDOP
                                    d_dump_file.write(reinterpret_cast<char *>(&d_dop[0]), sizeof(double));
                                    d_dump_file.write(reinterpret_cast<char *>(&d_dop[1]), sizeof(double));
                                    d_dump_file.write(reinterpret_cast<char *>(&d_dop[2]), sizeof(double));
                                    d_dump_file.write(reinterpret_cast<char *>(&d_dop[3]), sizeof(double));
                                }
                            catch (const std::ofstream::failure &e)
                                {
                                    LOG(WARNING) << "Exception writing RTKLIB dump file " << e.what();
                                }
                        }
                }
        }
    return this->is_valid_position();
}
