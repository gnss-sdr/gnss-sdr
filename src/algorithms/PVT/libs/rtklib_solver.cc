/*!
 * \file rtklib_solver.cc
 * \brief PVT solver based on rtklib library functions adapted to the GNSS-SDR
 *  data flow and structures
 * \authors <ul>
 *          <li> 2017-2019, Javier Arribas
 *          <li> 2017-2019, Carles Fernandez
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
 * -------------------------------------------------------------------------
 * Copyright (C) 2007-2013, T. Takasu
 * Copyright (C) 2017-2019, Javier Arribas
 * Copyright (C) 2017-2019, Carles Fernandez
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * -----------------------------------------------------------------------*/

#include "rtklib_solver.h"
#include "Beidou_B1I.h"
#include "Beidou_B3I.h"
#include "Beidou_DNAV.h"
#include "GLONASS_L1_L2_CA.h"
#include "GPS_L1_CA.h"
#include "Galileo_E1.h"
#include "rtklib_conversions.h"
#include "rtklib_rtkpos.h"
#include "rtklib_solution.h"
#include <glog/logging.h>
#include <matio.h>
#include <exception>
#include <utility>
#include <vector>

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


Rtklib_Solver::Rtklib_Solver(int nchannels, std::string dump_filename, bool flag_dump_to_file, bool flag_dump_to_mat, const rtk_t &rtk)
{
    // init empty ephemeris for all the available GNSS channels
    d_nchannels = nchannels;
    d_dump_filename = std::move(dump_filename);
    d_flag_dump_enabled = flag_dump_to_file;
    d_flag_dump_mat_enabled = flag_dump_to_mat;
    this->set_averaging_flag(false);
    rtk_ = rtk;

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
    if (d_dump_file.is_open() == true)
        {
            auto pos = d_dump_file.tellp();
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


bool Rtklib_Solver::save_matfile()
{
    // READ DUMP FILE
    std::string dump_filename = d_dump_filename;
    std::ifstream::pos_type size;
    int32_t number_of_double_vars = 21;
    int32_t number_of_uint32_vars = 2;
    int32_t number_of_uint8_vars = 3;
    int32_t number_of_float_vars = 2;
    int32_t epoch_size_bytes = sizeof(double) * number_of_double_vars +
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
            std::cerr << "Problem opening dump file:" << e.what() << std::endl;
            return false;
        }
    // count number of epochs and rewind
    int64_t num_epoch = 0LL;
    if (dump_file.is_open())
        {
            std::cout << "Generating .mat file for " << dump_filename << std::endl;
            size = dump_file.tellg();
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
            std::cerr << "Problem reading dump file:" << e.what() << std::endl;
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
    return dop_[0];
}


double Rtklib_Solver::get_pdop() const
{
    return dop_[1];
}


double Rtklib_Solver::get_hdop() const
{
    return dop_[2];
}


double Rtklib_Solver::get_vdop() const
{
    return dop_[3];
}


Monitor_Pvt Rtklib_Solver::get_monitor_pvt() const
{
    return monitor_pvt;
}


bool Rtklib_Solver::get_PVT(const std::map<int, Gnss_Synchro> &gnss_observables_map, bool flag_averaging)
{
    std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
    std::map<int, Galileo_Ephemeris>::const_iterator galileo_ephemeris_iter;
    std::map<int, Gps_Ephemeris>::const_iterator gps_ephemeris_iter;
    std::map<int, Gps_CNAV_Ephemeris>::const_iterator gps_cnav_ephemeris_iter;
    std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_ephemeris_iter;
    std::map<int, Beidou_Dnav_Ephemeris>::const_iterator beidou_ephemeris_iter;

    const Glonass_Gnav_Utc_Model gnav_utc = this->glonass_gnav_utc_model;

    this->set_averaging_flag(flag_averaging);

    // ********************************************************************************
    // ****** PREPARE THE DATA (SV EPHEMERIS AND OBSERVATIONS) ************************
    // ********************************************************************************
    int valid_obs = 0;      // valid observations counter
    int glo_valid_obs = 0;  // GLONASS L1/L2 valid observations counter

    obs_data.fill({});
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
                        std::string sig_(gnss_observables_iter->second.Signal);
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
                        std::string sig_(gnss_observables_iter->second.Signal);
                        // Galileo E1
                        if (sig_ == "1B")
                            {
                                // 1 Gal - find the ephemeris for the current GALILEO SV observation. The SV PRN ID is the map key
                                galileo_ephemeris_iter = galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (galileo_ephemeris_iter != galileo_ephemeris_map.cend())
                                    {
                                        // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                        eph_data[valid_obs] = eph_to_rtklib(galileo_ephemeris_iter->second);
                                        // convert observation from GNSS-SDR class to RTKLIB structure
                                        obsd_t newobs = {{0, 0}, '0', '0', {}, {}, {}, {}, {}, {}};
                                        obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                            gnss_observables_iter->second,
                                            galileo_ephemeris_iter->second.WN_5,
                                            0);
                                        valid_obs++;
                                    }
                                else  // the ephemeris are not available for this SV
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                                    }
                            }

                        // Galileo E5
                        if (sig_ == "5X")
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
                                                        obs_data[i + glo_valid_obs] = insert_obs_to_rtklib(obs_data[i + glo_valid_obs],
                                                            gnss_observables_iter->second,
                                                            galileo_ephemeris_iter->second.WN_5,
                                                            2);  // Band 3 (L5/E5)
                                                        found_E1_obs = true;
                                                        break;
                                                    }
                                            }
                                        if (!found_E1_obs)
                                            {
                                                // insert Galileo E5 obs as new obs and also insert its ephemeris
                                                // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                                eph_data[valid_obs] = eph_to_rtklib(galileo_ephemeris_iter->second);
                                                // convert observation from GNSS-SDR class to RTKLIB structure
                                                auto default_code_ = static_cast<unsigned char>(CODE_NONE);
                                                obsd_t newobs = {{0, 0}, '0', '0', {}, {},
                                                    {default_code_, default_code_, default_code_},
                                                    {}, {0.0, 0.0, 0.0}, {}};
                                                obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    galileo_ephemeris_iter->second.WN_5,
                                                    2);  // Band 3 (L5/E5)
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
                        std::string sig_(gnss_observables_iter->second.Signal);
                        if (sig_ == "1C")
                            {
                                gps_ephemeris_iter = gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (gps_ephemeris_iter != gps_ephemeris_map.cend())
                                    {
                                        // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                        eph_data[valid_obs] = eph_to_rtklib(gps_ephemeris_iter->second);
                                        // convert observation from GNSS-SDR class to RTKLIB structure
                                        obsd_t newobs = {{0, 0}, '0', '0', {}, {}, {}, {}, {}, {}};
                                        obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                            gnss_observables_iter->second,
                                            gps_ephemeris_iter->second.i_GPS_week,
                                            0);
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
                                                                obs_data[i + glo_valid_obs] = insert_obs_to_rtklib(obs_data[i + glo_valid_obs],
                                                                    gnss_observables_iter->second,
                                                                    eph_data[i].week,
                                                                    1);  // Band 2 (L2)
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
                                                auto default_code_ = static_cast<unsigned char>(CODE_NONE);
                                                obsd_t newobs = {{0, 0}, '0', '0', {}, {},
                                                    {default_code_, default_code_, default_code_},
                                                    {}, {0.0, 0.0, 0.0}, {}};
                                                obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    gps_cnav_ephemeris_iter->second.i_GPS_week,
                                                    1);  // Band 2 (L2)
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
                                                                obs_data[i + glo_valid_obs] = insert_obs_to_rtklib(obs_data[i],
                                                                    gnss_observables_iter->second,
                                                                    gps_cnav_ephemeris_iter->second.i_GPS_week,
                                                                    2);  // Band 3 (L5)
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
                                                auto default_code_ = static_cast<unsigned char>(CODE_NONE);
                                                obsd_t newobs = {{0, 0}, '0', '0', {}, {},
                                                    {default_code_, default_code_, default_code_},
                                                    {}, {0.0, 0.0, 0.0}, {}};
                                                obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    gps_cnav_ephemeris_iter->second.i_GPS_week,
                                                    2);  // Band 3 (L5)
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
                case 'R':  //TODO This should be using rtk lib nomenclature
                    {
                        std::string sig_(gnss_observables_iter->second.Signal);
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
                                        obsd_t newobs = {{0, 0}, '0', '0', {}, {}, {}, {}, {}, {}};
                                        obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                            gnss_observables_iter->second,
                                            glonass_gnav_ephemeris_iter->second.d_WN,
                                            0);  // Band 0 (L1)
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
                                                        obs_data[i + valid_obs] = insert_obs_to_rtklib(obs_data[i + valid_obs],
                                                            gnss_observables_iter->second,
                                                            glonass_gnav_ephemeris_iter->second.d_WN,
                                                            1);  // Band 1 (L2)
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
                                                obsd_t newobs = {{0, 0}, '0', '0', {}, {}, {}, {}, {}, {}};
                                                obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    glonass_gnav_ephemeris_iter->second.d_WN,
                                                    1);  // Band 1 (L2)
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
                        std::string sig_(gnss_observables_iter->second.Signal);
                        if (sig_ == "B1")
                            {
                                beidou_ephemeris_iter = beidou_dnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (beidou_ephemeris_iter != beidou_dnav_ephemeris_map.cend())
                                    {
                                        // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                        eph_data[valid_obs] = eph_to_rtklib(beidou_ephemeris_iter->second);
                                        // convert observation from GNSS-SDR class to RTKLIB structure
                                        obsd_t newobs = {{0, 0}, '0', '0', {}, {}, {}, {}, {}, {}};
                                        obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                            gnss_observables_iter->second,
                                            beidou_ephemeris_iter->second.i_BEIDOU_week + BEIDOU_DNAV_BDT2GPST_WEEK_NUM_OFFSET,
                                            0);
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
                                                        obs_data[i + glo_valid_obs] = insert_obs_to_rtklib(obs_data[i + glo_valid_obs],
                                                            gnss_observables_iter->second,
                                                            beidou_ephemeris_iter->second.i_BEIDOU_week + BEIDOU_DNAV_BDT2GPST_WEEK_NUM_OFFSET,
                                                            2);  // Band 3 (L2/G2/B3)
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
                                                auto default_code_ = static_cast<unsigned char>(CODE_NONE);
                                                obsd_t newobs = {{0, 0}, '0', '0', {}, {},
                                                    {default_code_, default_code_, default_code_},
                                                    {}, {0.0, 0.0, 0.0}, {}};
                                                obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    beidou_ephemeris_iter->second.i_BEIDOU_week + BEIDOU_DNAV_BDT2GPST_WEEK_NUM_OFFSET,
                                                    2);  // Band 2 (L2/G2)
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
            nav_t nav_data{};
            nav_data.eph = eph_data.data();
            nav_data.geph = geph_data.data();
            nav_data.n = valid_obs;
            nav_data.ng = glo_valid_obs;
            if (gps_iono.valid)
                {
                    nav_data.ion_gps[0] = gps_iono.d_alpha0;
                    nav_data.ion_gps[1] = gps_iono.d_alpha1;
                    nav_data.ion_gps[2] = gps_iono.d_alpha2;
                    nav_data.ion_gps[3] = gps_iono.d_alpha3;
                    nav_data.ion_gps[4] = gps_iono.d_beta0;
                    nav_data.ion_gps[5] = gps_iono.d_beta1;
                    nav_data.ion_gps[6] = gps_iono.d_beta2;
                    nav_data.ion_gps[7] = gps_iono.d_beta3;
                }
            if (!(gps_iono.valid) and gps_cnav_iono.valid)
                {
                    nav_data.ion_gps[0] = gps_cnav_iono.d_alpha0;
                    nav_data.ion_gps[1] = gps_cnav_iono.d_alpha1;
                    nav_data.ion_gps[2] = gps_cnav_iono.d_alpha2;
                    nav_data.ion_gps[3] = gps_cnav_iono.d_alpha3;
                    nav_data.ion_gps[4] = gps_cnav_iono.d_beta0;
                    nav_data.ion_gps[5] = gps_cnav_iono.d_beta1;
                    nav_data.ion_gps[6] = gps_cnav_iono.d_beta2;
                    nav_data.ion_gps[7] = gps_cnav_iono.d_beta3;
                }
            if (galileo_iono.ai0_5 != 0.0)
                {
                    nav_data.ion_gal[0] = galileo_iono.ai0_5;
                    nav_data.ion_gal[1] = galileo_iono.ai1_5;
                    nav_data.ion_gal[2] = galileo_iono.ai2_5;
                    nav_data.ion_gal[3] = 0.0;
                }
            if (beidou_dnav_iono.valid)
                {
                    nav_data.ion_cmp[0] = beidou_dnav_iono.d_alpha0;
                    nav_data.ion_cmp[1] = beidou_dnav_iono.d_alpha1;
                    nav_data.ion_cmp[2] = beidou_dnav_iono.d_alpha2;
                    nav_data.ion_cmp[3] = beidou_dnav_iono.d_alpha3;
                    nav_data.ion_cmp[4] = beidou_dnav_iono.d_beta0;
                    nav_data.ion_cmp[5] = beidou_dnav_iono.d_beta0;
                    nav_data.ion_cmp[6] = beidou_dnav_iono.d_beta0;
                    nav_data.ion_cmp[7] = beidou_dnav_iono.d_beta3;
                }
            if (gps_utc_model.valid)
                {
                    nav_data.utc_gps[0] = gps_utc_model.d_A0;
                    nav_data.utc_gps[1] = gps_utc_model.d_A1;
                    nav_data.utc_gps[2] = gps_utc_model.d_t_OT;
                    nav_data.utc_gps[3] = gps_utc_model.i_WN_T;
                    nav_data.leaps = gps_utc_model.d_DeltaT_LS;
                }
            if (!(gps_utc_model.valid) and gps_cnav_utc_model.valid)
                {
                    nav_data.utc_gps[0] = gps_cnav_utc_model.d_A0;
                    nav_data.utc_gps[1] = gps_cnav_utc_model.d_A1;
                    nav_data.utc_gps[2] = gps_cnav_utc_model.d_t_OT;
                    nav_data.utc_gps[3] = gps_cnav_utc_model.i_WN_T;
                    nav_data.leaps = gps_cnav_utc_model.d_DeltaT_LS;
                }
            if (glonass_gnav_utc_model.valid)
                {
                    nav_data.utc_glo[0] = glonass_gnav_utc_model.d_tau_c;  // ??
                    nav_data.utc_glo[1] = 0.0;                             // ??
                    nav_data.utc_glo[2] = 0.0;                             // ??
                    nav_data.utc_glo[3] = 0.0;                             // ??
                }
            if (galileo_utc_model.A0_6 != 0.0)
                {
                    nav_data.utc_gal[0] = galileo_utc_model.A0_6;
                    nav_data.utc_gal[1] = galileo_utc_model.A1_6;
                    nav_data.utc_gal[2] = galileo_utc_model.t0t_6;
                    nav_data.utc_gal[3] = galileo_utc_model.WNot_6;
                    nav_data.leaps = galileo_utc_model.Delta_tLS_6;
                }
            if (beidou_dnav_utc_model.valid)
                {
                    nav_data.utc_cmp[0] = beidou_dnav_utc_model.d_A0_UTC;
                    nav_data.utc_cmp[1] = beidou_dnav_utc_model.d_A1_UTC;
                    nav_data.utc_cmp[2] = 0.0;  // ??
                    nav_data.utc_cmp[3] = 0.0;  // ??
                    nav_data.leaps = beidou_dnav_utc_model.d_DeltaT_LS;
                }

            /* update carrier wave length using native function call in RTKlib */
            for (int i = 0; i < MAXSAT; i++)
                {
                    for (int j = 0; j < NFREQ; j++)
                        {
                            nav_data.lam[i][j] = satwavelen(i + 1, j, &nav_data);
                        }
                }

            result = rtkpos(&rtk_, obs_data.data(), valid_obs + glo_valid_obs, &nav_data);

            if (result == 0)
                {
                    LOG(INFO) << "RTKLIB rtkpos error";
                    DLOG(INFO) << "RTKLIB rtkpos error message: " << rtk_.errbuf;
                    this->set_time_offset_s(0.0);  // reset rx time estimation
                    this->set_num_valid_observations(0);
                }
            else
                {
                    this->set_num_valid_observations(rtk_.sol.ns);  // record the number of valid satellites used by the PVT solver
                    pvt_sol = rtk_.sol;
                    // DOP computation
                    unsigned int used_sats = 0;
                    for (unsigned int i = 0; i < MAXSAT; i++)
                        {
                            pvt_ssat[i] = rtk_.ssat[i];
                            if (rtk_.ssat[i].vs == 1)
                                {
                                    used_sats++;
                                }
                        }

                    std::vector<double> azel;
                    azel.reserve(used_sats * 2);
                    unsigned int index_aux = 0;
                    for (auto &i : rtk_.ssat)
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
                            dops(index_aux, azel.data(), 0.0, dop_.data());
                        }
                    this->set_valid_position(true);
                    arma::vec rx_position_and_time(4);
                    rx_position_and_time(0) = pvt_sol.rr[0];  // [m]
                    rx_position_and_time(1) = pvt_sol.rr[1];  // [m]
                    rx_position_and_time(2) = pvt_sol.rr[2];  // [m]
                    //todo: fix this ambiguity in the RTKLIB units in receiver clock offset!
                    if (rtk_.opt.mode == PMODE_SINGLE)
                        {
                            // if the RTKLIB solver is set to SINGLE, the dtr is already expressed in [s]
                            // add also the clock offset from gps to galileo (pvt_sol.dtr[2])
                            rx_position_and_time(3) = pvt_sol.dtr[0] + pvt_sol.dtr[2];
                        }
                    else
                        {
                            // the receiver clock offset is expressed in [meters], so we convert it into [s]
                            // add also the clock offset from gps to galileo (pvt_sol.dtr[2])
                            rx_position_and_time(3) = pvt_sol.dtr[2] + pvt_sol.dtr[0] / GPS_C_M_S;
                        }
                    this->set_rx_pos(rx_position_and_time.rows(0, 2));  // save ECEF position for the next iteration

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

                    this->set_time_offset_s(rx_position_and_time(3));

                    DLOG(INFO) << "RTKLIB Position at RX TOW = " << gnss_observables_map.begin()->second.RX_time
                               << " in ECEF (X,Y,Z,t[meters]) = " << rx_position_and_time;

                    boost::posix_time::ptime p_time;
                    // gtime_t rtklib_utc_time = gpst2utc(pvt_sol.time); // Corrected RX Time (Non integer multiply of 1 ms of granularity)
                    // Uncorrected RX Time (integer multiply of 1 ms and the same observables time reported in RTCM and RINEX)
                    gtime_t rtklib_time = timeadd(pvt_sol.time, rx_position_and_time(3));  // uncorrected rx time
                    gtime_t rtklib_utc_time = gpst2utc(rtklib_time);
                    p_time = boost::posix_time::from_time_t(rtklib_utc_time.time);
                    p_time += boost::posix_time::microseconds(static_cast<long>(round(rtklib_utc_time.sec * 1e6)));  // NOLINT(google-runtime-int)

                    this->set_position_UTC_time(p_time);
                    cart2geo(static_cast<double>(rx_position_and_time(0)), static_cast<double>(rx_position_and_time(1)), static_cast<double>(rx_position_and_time(2)), 4);

                    DLOG(INFO) << "RTKLIB Position at " << boost::posix_time::to_simple_string(p_time)
                               << " is Lat = " << this->get_latitude() << " [deg], Long = " << this->get_longitude()
                               << " [deg], Height= " << this->get_height() << " [m]"
                               << " RX time offset= " << this->get_time_offset_s() << " [s]";

                    // ######## PVT MONITOR #########
                    // TOW
                    monitor_pvt.TOW_at_current_symbol_ms = gnss_observables_map.begin()->second.TOW_at_current_symbol_ms;
                    // WEEK
                    monitor_pvt.week = adjgpsweek(nav_data.eph[0].week);
                    // PVT GPS time
                    monitor_pvt.RX_time = gnss_observables_map.begin()->second.RX_time;
                    // User clock offset [s]
                    monitor_pvt.user_clk_offset = rx_position_and_time(3);

                    // ECEF POS X,Y,X [m] + ECEF VEL X,Y,X [m/s] (6 x double)
                    monitor_pvt.pos_x = pvt_sol.rr[0];
                    monitor_pvt.pos_y = pvt_sol.rr[1];
                    monitor_pvt.pos_z = pvt_sol.rr[2];
                    monitor_pvt.vel_x = pvt_sol.rr[3];
                    monitor_pvt.vel_y = pvt_sol.rr[4];
                    monitor_pvt.vel_z = pvt_sol.rr[5];

                    // position variance/covariance (m^2) {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx} (6 x double)
                    monitor_pvt.cov_xx = pvt_sol.qr[0];
                    monitor_pvt.cov_yy = pvt_sol.qr[1];
                    monitor_pvt.cov_zz = pvt_sol.qr[2];
                    monitor_pvt.cov_xy = pvt_sol.qr[3];
                    monitor_pvt.cov_yz = pvt_sol.qr[4];
                    monitor_pvt.cov_zx = pvt_sol.qr[5];

                    // GEO user position Latitude [deg]
                    monitor_pvt.latitude = get_latitude();
                    // GEO user position Longitude [deg]
                    monitor_pvt.longitude = get_longitude();
                    // GEO user position Height [m]
                    monitor_pvt.height = get_height();

                    // NUMBER OF VALID SATS
                    monitor_pvt.valid_sats = pvt_sol.ns;
                    // RTKLIB solution status
                    monitor_pvt.solution_status = pvt_sol.stat;
                    // RTKLIB solution type (0:xyz-ecef,1:enu-baseline)
                    monitor_pvt.solution_type = pvt_sol.type;
                    // AR ratio factor for validation
                    monitor_pvt.AR_ratio_factor = pvt_sol.ratio;
                    // AR ratio threshold for validation
                    monitor_pvt.AR_ratio_threshold = pvt_sol.thres;

                    // GDOP / PDOP/ HDOP/ VDOP
                    monitor_pvt.gdop = dop_[0];
                    monitor_pvt.pdop = dop_[1];
                    monitor_pvt.hdop = dop_[2];
                    monitor_pvt.vdop = dop_[3];

                    // ######## LOG FILE #########
                    if (d_flag_dump_enabled == true)
                        {
                            // MULTIPLEXED FILE RECORDING - Record results to file
                            try
                                {
                                    double tmp_double;
                                    uint32_t tmp_uint32;
                                    // TOW
                                    tmp_uint32 = gnss_observables_map.begin()->second.TOW_at_current_symbol_ms;
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_uint32), sizeof(uint32_t));
                                    // WEEK
                                    tmp_uint32 = adjgpsweek(nav_data.eph[0].week);
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_uint32), sizeof(uint32_t));
                                    // PVT GPS time
                                    tmp_double = gnss_observables_map.begin()->second.RX_time;
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                                    // User clock offset [s]
                                    tmp_double = rx_position_and_time(3);
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
                                    tmp_double = get_latitude();
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                                    // GEO user position Longitude [deg]
                                    tmp_double = get_longitude();
                                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                                    // GEO user position Height [m]
                                    tmp_double = get_height();
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

                                    // GDOP / PDOP/ HDOP/ VDOP
                                    d_dump_file.write(reinterpret_cast<char *>(&dop_[0]), sizeof(double));
                                    d_dump_file.write(reinterpret_cast<char *>(&dop_[1]), sizeof(double));
                                    d_dump_file.write(reinterpret_cast<char *>(&dop_[2]), sizeof(double));
                                    d_dump_file.write(reinterpret_cast<char *>(&dop_[3]), sizeof(double));
                                }
                            catch (const std::ifstream::failure &e)
                                {
                                    LOG(WARNING) << "Exception writing RTKLIB dump file " << e.what();
                                }
                        }
                }
        }
    return is_valid_position();
}
