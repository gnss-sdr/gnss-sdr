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
#include "Beidou_CNAV1.h"
#include "Beidou_DNAV.h"
#include "Galileo_CNAV.h"
#include "gnss_obs_codes.h"
#include "gnss_sdr_filesystem.h"
#include "matlab_writter_helper.h"
#include "ntrip_rtcm_client.h"
#include "ntrip_rtk_signals.h"
#include "rtklib_rtkcmn.h"
#include "rtklib_rtkpos.h"
#include "signal_enabled_flags.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

/* out-of-line definition: pre-C++17 builds require it when the constant is
   odr-used (e.g. streamed to a logger by const reference); in C++17 and
   later this is a harmless redeclaration */
constexpr int Rtklib_Solver::NTRIP_MIN_COMMON_SATELLITES;

Rtklib_Solver::Rtklib_Solver(const rtk_t &rtk,
    const Pvt_Conf &conf,
    const std::string &dump_filename,
    uint32_t signal_enabled_flags,
    bool flag_dump_to_file,
    bool flag_dump_to_mat) : d_dump_filename(dump_filename),
                             d_sbas_corrections(rtk.opt.sbassatsel),
                             d_conf(conf),
                             d_signal_enabled_flags(signal_enabled_flags),
                             d_flag_dump_enabled(flag_dump_to_file),
                             d_flag_dump_mat_enabled(flag_dump_to_mat)
{
    // Solver instances need correction policy but never caster credentials.
    // Remove secrets from this long-lived configuration copy immediately.
    secure_clear_ntrip_credentials(&d_conf);

    // see freq index at src/algorithms/libs/rtklib/rtklib_rtkcmn.cc
    // function: satwavelen
    d_rtklib_freq_index[0] = 0;
    d_rtklib_freq_index[1] = 1;
    d_rtklib_freq_index[2] = 2;

    // BDS: B1I and B1C share slot 0 (prefer-B1C XOR). Override lam[0] for B1C (FREQ1).
    d_rtklib_band_index["1G"] = 0;
    d_rtklib_band_index["1C"] = 0;
    d_rtklib_band_index["1B"] = 0;
    d_rtklib_band_index["B1"] = 0;
    d_rtklib_band_index["1D"] = 0;
    d_rtklib_band_index["B3"] = 2;
    d_rtklib_band_index["2G"] = 1;
    d_rtklib_band_index["2S"] = 1;
    d_rtklib_band_index["L5"] = 2;
    d_rtklib_band_index["E6"] = 0;
    d_rtklib_band_index["J1"] = 0;
    d_rtklib_band_index["J5"] = 2;

    const Signal_Enabled_Flags flags(d_signal_enabled_flags);

    // The Galileo OS SIS ICD defines E1/E5b as the I/NAV service and E1/E5a
    // as the F/NAV service. Select one service deterministically for the whole
    // receiver so channel and message arrival order cannot change the clock
    // model used by PVT. I/NAV has priority when E5b is enabled; otherwise an
    // enabled E5a selects F/NAV. E1-only receivers use I/NAV.
    if (flags.check_any_enabled(GAL_E5b))
        {
            d_galileo_nav_message_type_for_pvt = Galileo_Nav_Message_Type::INAV;
        }
    else if (flags.check_any_enabled(GAL_E5a))
        {
            d_galileo_nav_message_type_for_pvt = Galileo_Nav_Message_Type::FNAV;
        }

    // Only the E5 signal belonging to the selected Galileo navigation service
    // is admitted to RTKLIB. Keeping the inactive signal out of this map makes
    // the shared third observation slot an explicit invariant rather than an
    // ordering-dependent overwrite.
    if (d_galileo_nav_message_type_for_pvt == Galileo_Nav_Message_Type::INAV &&
        flags.check_any_enabled(GAL_E5b))
        {
            d_rtklib_band_index["7X"] = 2;
        }
    else if (d_galileo_nav_message_type_for_pvt == Galileo_Nav_Message_Type::FNAV &&
             flags.check_any_enabled(GAL_E5a))
        {
            d_rtklib_band_index["5X"] = 2;
        }

    const bool has_selected_galileo_e5 =
        d_rtklib_band_index.find("5X") != d_rtklib_band_index.cend() ||
        d_rtklib_band_index.find("7X") != d_rtklib_band_index.cend();

    // RTKLIB's Galileo SPP/PPP paths use slots 0 and 2 for their primary
    // dual-frequency pair. Put E6 in slot 2 when E1+E6 is that pair. If the
    // automatically selected E5 signal already owns slot 2, keep E6 in the
    // remaining slot so E1, E5, and E6 can coexist without overwriting one
    // another. E6-only reception stays in the primary slot.
    if (flags.check_any_enabled(GAL_E6))
        {
            if (has_selected_galileo_e5)
                {
                    d_rtklib_band_index["E6"] = 1;
                }
            else if (flags.check_any_enabled(GAL_1B))
                {
                    d_rtklib_band_index["E6"] = 2;
                }
        }

    // GPS L5 needs the primary slot in this single-frequency GPS setup. The
    // Galileo wavelength is corrected from its observation code below, so the
    // two constellations do not need a shared nominal frequency index.
    if (flags.check_only_enabled(GPS_L5, GAL_E5b))
        {
            d_rtklib_band_index["L5"] = 0;
            d_rtklib_freq_index[0] = 2;
        }

    // In automatic I/NAV mode E5a observations are not admitted to PVT, so
    // Galileo-only multi-band configurations must use the E5b wavelength in
    // the third RTKLIB slot. This also covers E1+E5a+E5b, which was not part
    // of the legacy exact-flag special cases above.
    if (d_galileo_nav_message_type_for_pvt == Galileo_Nav_Message_Type::INAV &&
        flags.check_any_enabled(GAL_E5b) &&
        !flags.check_any_enabled(GPS_L5, QZS_J5))
        {
            d_rtklib_freq_index[2] = 4;
        }

    if (d_conf.ntrip_client_enabled)
        {
            // the fixed-base path pairs rover and base observations by slot,
            // so the signal table's slot column and this solver's finished
            // band map must agree for every enabled NTRIP signal — fail
            // loudly at construction instead of silently pairing across bands
            for (const auto &signal : ntrip_rtk_signals())
                {
                    if (!flags.check_any_enabled(signal.flag))
                        {
                            continue;
                        }
                    const auto mapped_slot = d_rtklib_band_index.find(signal.signal);
                    if (mapped_slot == d_rtklib_band_index.cend() ||
                        mapped_slot->second != signal.band_slot)
                        {
                            throw std::runtime_error(
                                std::string("NTRIP RTK signal table and solver band mapping disagree for signal ") +
                                signal.signal);
                        }
                }
        }

    // resolve the per-system second-band slots once: the channel set is
    // fixed for the solver's lifetime, and the per-epoch fixed-base path
    // must not rescan the signal table per observation
    d_ntrip_second_band_slot_gps = ntrip_rtk_second_band_slot(SYS_GPS, flags);
    d_ntrip_second_band_slot_gal = ntrip_rtk_second_band_slot(SYS_GAL, flags);
    d_ntrip_second_band_slot_bds = ntrip_rtk_second_band_slot(SYS_BDS, flags);

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

    // rtk_t owns dynamically allocated filter state. A struct copy would make
    // the adapter, clock solver, and user solver alias the same state and would
    // make independent RTK updates unsafe. Initialize it after all potentially
    // throwing C++ setup so a failed constructor cannot leak the C allocation.
    rtkinit(&d_rtk, &rtk.opt);
}


Rtklib_Solver::~Rtklib_Solver() noexcept
{
    DLOG(INFO) << "Rtklib_Solver destructor called.";
    if (d_dump_file.is_open() == true)
        {
            std::ofstream::pos_type pos = -1;
            try
                {
                    pos = d_dump_file.tellp();
                    d_dump_file.close();
                }
            catch (const std::exception &ex)
                {
                    LOG(WARNING) << "Exception in destructor closing the RTKLIB dump file " << ex.what();
                }
            if (pos == 0)
                {
                    try
                        {
                            errorlib::error_code ec;
                            if (!fs::remove(fs::path(d_dump_filename), ec))
                                {
                                    std::cerr << "Problem removing temporary file " << d_dump_filename << '\n';
                                }
                            d_flag_dump_mat_enabled = false;
                        }
                    catch (const std::exception &ex)
                        {
                            LOG(WARNING) << "Exception in destructor removing the RTKLIB dump file " << ex.what();
                        }
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
    rtkfree(&d_rtk);
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
    std::string filename = dump_filename;
    filename.erase(filename.length() - 4, 4);
    filename.append(".mat");
    mat_t *matfp = Mat_CreateVer(filename.c_str(), nullptr, MAT_FT_MAT73);
    if (reinterpret_cast<int64_t *>(matfp) != nullptr)
        {
            std::array<size_t, 2> dims{1, static_cast<size_t>(num_epoch)};
            write_matlab_var<2>("TOW_at_current_symbol_ms", TOW_at_current_symbol_ms.data(), matfp, dims);
            write_matlab_var<2>("week", week.data(), matfp, dims);
            write_matlab_var<2>("RX_time", RX_time.data(), matfp, dims);
            write_matlab_var<2>("user_clk_offset", user_clk_offset.data(), matfp, dims);
            write_matlab_var<2>("pos_x", pos_x.data(), matfp, dims);
            write_matlab_var<2>("pos_y", pos_y.data(), matfp, dims);
            write_matlab_var<2>("pos_z", pos_z.data(), matfp, dims);
            write_matlab_var<2>("vel_x", vel_x.data(), matfp, dims);
            write_matlab_var<2>("vel_y", vel_y.data(), matfp, dims);
            write_matlab_var<2>("vel_z", vel_z.data(), matfp, dims);
            write_matlab_var<2>("cov_xx", cov_xx.data(), matfp, dims);
            write_matlab_var<2>("cov_yy", cov_yy.data(), matfp, dims);
            write_matlab_var<2>("cov_zz", cov_zz.data(), matfp, dims);
            write_matlab_var<2>("cov_xy", cov_xy.data(), matfp, dims);
            write_matlab_var<2>("cov_yz", cov_yz.data(), matfp, dims);
            write_matlab_var<2>("cov_zx", cov_zx.data(), matfp, dims);
            write_matlab_var<2>("latitude", latitude.data(), matfp, dims);
            write_matlab_var<2>("longitude", longitude.data(), matfp, dims);
            write_matlab_var<2>("height", height.data(), matfp, dims);
            write_matlab_var<2>("valid_sats", valid_sats.data(), matfp, dims);
            write_matlab_var<2>("solution_status", solution_status.data(), matfp, dims);
            write_matlab_var<2>("solution_type", solution_type.data(), matfp, dims);
            write_matlab_var<2>("AR_ratio_factor", AR_ratio_factor.data(), matfp, dims);
            write_matlab_var<2>("AR_ratio_threshold", AR_ratio_threshold.data(), matfp, dims);
            write_matlab_var<2>("gdop", gdop.data(), matfp, dims);
            write_matlab_var<2>("pdop", pdop.data(), matfp, dims);
            write_matlab_var<2>("hdop", hdop.data(), matfp, dims);
            write_matlab_var<2>("vdop", vdop.data(), matfp, dims);
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


Rtklib_Fixed_Base_Status Rtklib_Solver::get_fixed_base_status() const
{
    return d_fixed_base_status;
}


double Rtklib_Solver::get_fixed_base_age_s() const
{
    return d_fixed_base_age_s;
}


std::size_t Rtklib_Solver::get_fixed_base_common_satellites() const
{
    return d_fixed_base_common_satellites;
}


void Rtklib_Solver::store_sbas_message(const Sbas_Raw_Message &message)
{
    d_sbas_corrections.push(message);
}


void Rtklib_Solver::store_gps_ephemeris(const Gps_Ephemeris &ephemeris)
{
    const auto current_ephemeris = gps_ephemeris_map.find(ephemeris.PRN);
    if (current_ephemeris != gps_ephemeris_map.cend() &&
        current_ephemeris->second.IODE_SF3 != ephemeris.IODE_SF3)
        {
            // SBAS long-term corrections identify the broadcast ephemeris by
            // IODE. Retain the preceding issue across the handover so RTKLIB
            // can keep applying a still-current MT24/25 correction.
            gps_previous_ephemeris_map[ephemeris.PRN] = current_ephemeris->second;
        }
    gps_ephemeris_map[ephemeris.PRN] = ephemeris;
}


void Rtklib_Solver::clear_gps_ephemerides()
{
    gps_ephemeris_map.clear();
    gps_previous_ephemeris_map.clear();
}


void Rtklib_Solver::clear_has_corrections()
{
    d_has_orbit_corrections_store_map.clear();
    d_has_clock_corrections_store_map.clear();
    d_has_satellite_do_not_use_until.clear();
    d_has_code_bias_store_map.clear();
    d_has_phase_bias_store_map.clear();
    d_has_phase_discontinuity_indicator_store_map.clear();
    d_has_phase_bias_discontinuity_map.clear();
    d_has_obs_corr_map.clear();
}


void Rtklib_Solver::store_has_data(const Galileo_HAS_data &new_has_data)
{
    if (new_has_data.week == GALILEO_HAS_INVALID_WEEK || new_has_data.tow >= GALILEO_HAS_SECONDS_PER_WEEK)
        {
            LOG(INFO) << "Ignoring Galileo HAS corrections without valid GST";
            return;
        }

    //  Compute time of application HAS SIS ICD, Issue 1.0, Section 7.7
    const uint32_t tmt = new_has_data.get_time_of_message_s();

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
                            float in_track_m = new_has_data.get_delta_in_track_m(gps_str, prn);
                            float cross_track_m = new_has_data.get_delta_cross_track_m(gps_str, prn);
                            if (Galileo_HAS_data::is_orbit_radial_unavailable(radial_m) ||
                                Galileo_HAS_data::is_orbit_along_cross_unavailable(in_track_m) ||
                                Galileo_HAS_data::is_orbit_along_cross_unavailable(cross_track_m))
                                {
                                    d_has_orbit_corrections_store_map[gps_str].erase(prn);
                                    d_has_clock_corrections_store_map[gps_str].erase(prn);
                                    continue;
                                }
                            auto &orbit_correction = d_has_orbit_corrections_store_map[gps_str][prn];
                            orbit_correction.radial_m = radial_m;
                            orbit_correction.in_track_m = in_track_m;
                            orbit_correction.cross_track_m = cross_track_m;
                            orbit_correction.valid_until = tmt +
                                                           new_has_data.get_validity_interval_s(new_has_data.validity_interval_index_orbit_corrections);
                            orbit_correction.iod = gnss_iod;
                            orbit_correction.mask_id = new_has_data.header.mask_id;
                            orbit_correction.iod_set_id = new_has_data.header.iod_set_id;
                        }
                }

            // for each satellite in Galileo ephemeris
            for (const auto &galeph : galileo_ephemeris_store.by_source(d_galileo_nav_message_type_for_pvt))
                {
                    int prn = galeph.second.PRN;
                    int32_t sis_iod = galeph.second.IOD_ephemeris;
                    uint16_t gnss_iod = new_has_data.get_gnss_iod(gal_str, prn);
                    if (static_cast<int32_t>(gnss_iod) == sis_iod)
                        {
                            float radial_m = new_has_data.get_delta_radial_m(gal_str, prn);
                            float in_track_m = new_has_data.get_delta_in_track_m(gal_str, prn);
                            float cross_track_m = new_has_data.get_delta_cross_track_m(gal_str, prn);
                            if (Galileo_HAS_data::is_orbit_radial_unavailable(radial_m) ||
                                Galileo_HAS_data::is_orbit_along_cross_unavailable(in_track_m) ||
                                Galileo_HAS_data::is_orbit_along_cross_unavailable(cross_track_m))
                                {
                                    d_has_orbit_corrections_store_map[gal_str].erase(prn);
                                    d_has_clock_corrections_store_map[gal_str].erase(prn);
                                    continue;
                                }
                            auto &orbit_correction = d_has_orbit_corrections_store_map[gal_str][prn];
                            orbit_correction.radial_m = radial_m;
                            orbit_correction.in_track_m = in_track_m;
                            orbit_correction.cross_track_m = cross_track_m;
                            orbit_correction.valid_until = tmt +
                                                           new_has_data.get_validity_interval_s(new_has_data.validity_interval_index_orbit_corrections);
                            orbit_correction.iod = gnss_iod;
                            orbit_correction.mask_id = new_has_data.header.mask_id;
                            orbit_correction.iod_set_id = new_has_data.header.iod_set_id;
                        }
                }
        }
    if (new_has_data.header.clock_fullset_flag)
        {
            LOG(INFO) << "Received HAS clock fullset corrections";
            const uint32_t valid_until = tmt +
                                         new_has_data.get_validity_interval_s(new_has_data.validity_interval_index_clock_fullset_corrections);
            const auto gps_prns = new_has_data.get_PRNs_in_mask(gps_str);
            for (const auto &gpseph : gps_ephemeris_map)
                {
                    int prn = gpseph.second.PRN;
                    if (std::find(gps_prns.cbegin(), gps_prns.cend(), prn) == gps_prns.cend())
                        {
                            continue;
                        }
                    int32_t sis_iod = gpseph.second.IODE_SF3;
                    auto it = d_has_orbit_corrections_store_map[gps_str].find(prn);
                    if (it != d_has_orbit_corrections_store_map[gps_str].end())
                        {
                            const uint16_t gnss_iod = new_has_data.get_gnss_iod(gps_str, prn);
                            if ((static_cast<int32_t>(gnss_iod) == sis_iod) &&
                                (it->second.iod == gnss_iod) &&
                                (it->second.mask_id == new_has_data.header.mask_id) &&
                                (it->second.iod_set_id == new_has_data.header.iod_set_id))
                                {
                                    float clock_correction_mult_m = new_has_data.get_clock_correction_mult_m(gps_str, prn);
                                    if (Galileo_HAS_data::is_clock_do_not_use(clock_correction_mult_m))
                                        {
                                            d_has_clock_corrections_store_map[gps_str].erase(prn);
                                            d_has_satellite_do_not_use_until[gps_str][prn] = valid_until;
                                            continue;
                                        }
                                    if (Galileo_HAS_data::is_clock_unavailable(clock_correction_mult_m))
                                        {
                                            d_has_clock_corrections_store_map[gps_str].erase(prn);
                                            continue;
                                        }
                                    d_has_satellite_do_not_use_until[gps_str].erase(prn);
                                    auto &clock_correction = d_has_clock_corrections_store_map[gps_str][prn];
                                    clock_correction.clock_correction_m = clock_correction_mult_m;
                                    clock_correction.valid_until = valid_until;
                                    clock_correction.iod = gnss_iod;
                                    clock_correction.mask_id = new_has_data.header.mask_id;
                                    clock_correction.iod_set_id = new_has_data.header.iod_set_id;
                                }
                        }
                }

            // for each satellite in Galileo ephemeris
            const auto gal_prns = new_has_data.get_PRNs_in_mask(gal_str);
            for (const auto &galeph : galileo_ephemeris_store.by_source(d_galileo_nav_message_type_for_pvt))
                {
                    int prn = galeph.second.PRN;
                    if (std::find(gal_prns.cbegin(), gal_prns.cend(), prn) == gal_prns.cend())
                        {
                            continue;
                        }
                    int32_t iod_sis = galeph.second.IOD_ephemeris;
                    auto it = d_has_orbit_corrections_store_map[gal_str].find(prn);
                    if (it != d_has_orbit_corrections_store_map[gal_str].end())
                        {
                            const uint16_t gnss_iod = new_has_data.get_gnss_iod(gal_str, prn);
                            if ((static_cast<int32_t>(gnss_iod) == iod_sis) &&
                                (it->second.iod == gnss_iod) &&
                                (it->second.mask_id == new_has_data.header.mask_id) &&
                                (it->second.iod_set_id == new_has_data.header.iod_set_id))
                                {
                                    float clock_correction_mult_m = new_has_data.get_clock_correction_mult_m(gal_str, prn);
                                    // std::cout << "Galileo Satellite " << prn
                                    //           << " clock correction=" << new_has_data.get_clock_correction_mult_m(gal_str, prn)
                                    //           << std::endl;
                                    if (Galileo_HAS_data::is_clock_do_not_use(clock_correction_mult_m))
                                        {
                                            d_has_clock_corrections_store_map[gal_str].erase(prn);
                                            d_has_satellite_do_not_use_until[gal_str][prn] = valid_until;
                                            continue;
                                        }
                                    if (Galileo_HAS_data::is_clock_unavailable(clock_correction_mult_m))
                                        {
                                            d_has_clock_corrections_store_map[gal_str].erase(prn);
                                            continue;
                                        }
                                    d_has_satellite_do_not_use_until[gal_str].erase(prn);
                                    auto &clock_correction = d_has_clock_corrections_store_map[gal_str][prn];
                                    clock_correction.clock_correction_m = clock_correction_mult_m;
                                    clock_correction.valid_until = valid_until;
                                    clock_correction.iod = gnss_iod;
                                    clock_correction.mask_id = new_has_data.header.mask_id;
                                    clock_correction.iod_set_id = new_has_data.header.iod_set_id;
                                }
                        }
                }
        }
    if (new_has_data.header.clock_subset_flag)
        {
            LOG(INFO) << "Received HAS clock subset corrections";
            const uint32_t valid_until = tmt +
                                         new_has_data.get_validity_interval_s(new_has_data.validity_interval_index_clock_subset_corrections);
            const auto systems = new_has_data.get_systems_string();

            const auto gps_sys_it = std::find(systems.cbegin(), systems.cend(), gps_str);
            if (gps_sys_it != systems.cend())
                {
                    const auto gps_sys_index = static_cast<uint8_t>(std::distance(systems.cbegin(), gps_sys_it));
                    const auto gps_subset_prns = new_has_data.get_PRNs_in_submask(gps_sys_index);
                    for (const auto &gpseph : gps_ephemeris_map)
                        {
                            int prn = gpseph.second.PRN;
                            if (std::find(gps_subset_prns.cbegin(), gps_subset_prns.cend(), prn) == gps_subset_prns.cend())
                                {
                                    continue;
                                }
                            int32_t sis_iod = gpseph.second.IODE_SF3;
                            auto it = d_has_orbit_corrections_store_map[gps_str].find(prn);
                            if (it != d_has_orbit_corrections_store_map[gps_str].end())
                                {
                                    const uint16_t gnss_iod = new_has_data.get_gnss_iod(gps_str, prn);
                                    if ((static_cast<int32_t>(gnss_iod) == sis_iod) &&
                                        (it->second.iod == gnss_iod) &&
                                        (it->second.mask_id == new_has_data.header.mask_id) &&
                                        (it->second.iod_set_id == new_has_data.header.iod_set_id))
                                        {
                                            float clock_correction_mult_m = new_has_data.get_clock_subset_correction_mult_m(gps_str, prn);
                                            if (Galileo_HAS_data::is_clock_do_not_use(clock_correction_mult_m))
                                                {
                                                    d_has_clock_corrections_store_map[gps_str].erase(prn);
                                                    d_has_satellite_do_not_use_until[gps_str][prn] = valid_until;
                                                    continue;
                                                }
                                            if (Galileo_HAS_data::is_clock_unavailable(clock_correction_mult_m))
                                                {
                                                    d_has_clock_corrections_store_map[gps_str].erase(prn);
                                                    continue;
                                                }
                                            d_has_satellite_do_not_use_until[gps_str].erase(prn);
                                            auto &clock_correction = d_has_clock_corrections_store_map[gps_str][prn];
                                            clock_correction.clock_correction_m = clock_correction_mult_m;
                                            clock_correction.valid_until = valid_until;
                                            clock_correction.iod = gnss_iod;
                                            clock_correction.mask_id = new_has_data.header.mask_id;
                                            clock_correction.iod_set_id = new_has_data.header.iod_set_id;
                                        }
                                }
                        }
                }

            const auto gal_sys_it = std::find(systems.cbegin(), systems.cend(), gal_str);
            if (gal_sys_it != systems.cend())
                {
                    const auto gal_sys_index = static_cast<uint8_t>(std::distance(systems.cbegin(), gal_sys_it));
                    const auto gal_subset_prns = new_has_data.get_PRNs_in_submask(gal_sys_index);
                    for (const auto &galeph : galileo_ephemeris_store.by_source(d_galileo_nav_message_type_for_pvt))
                        {
                            int prn = galeph.second.PRN;
                            if (std::find(gal_subset_prns.cbegin(), gal_subset_prns.cend(), prn) == gal_subset_prns.cend())
                                {
                                    continue;
                                }
                            int32_t iod_sis = galeph.second.IOD_ephemeris;
                            auto it = d_has_orbit_corrections_store_map[gal_str].find(prn);
                            if (it != d_has_orbit_corrections_store_map[gal_str].end())
                                {
                                    const uint16_t gnss_iod = new_has_data.get_gnss_iod(gal_str, prn);
                                    if ((static_cast<int32_t>(gnss_iod) == iod_sis) &&
                                        (it->second.iod == gnss_iod) &&
                                        (it->second.mask_id == new_has_data.header.mask_id) &&
                                        (it->second.iod_set_id == new_has_data.header.iod_set_id))
                                        {
                                            float clock_correction_mult_m = new_has_data.get_clock_subset_correction_mult_m(gal_str, prn);
                                            if (Galileo_HAS_data::is_clock_do_not_use(clock_correction_mult_m))
                                                {
                                                    d_has_clock_corrections_store_map[gal_str].erase(prn);
                                                    d_has_satellite_do_not_use_until[gal_str][prn] = valid_until;
                                                    continue;
                                                }
                                            if (Galileo_HAS_data::is_clock_unavailable(clock_correction_mult_m))
                                                {
                                                    d_has_clock_corrections_store_map[gal_str].erase(prn);
                                                    continue;
                                                }
                                            d_has_satellite_do_not_use_until[gal_str].erase(prn);
                                            auto &clock_correction = d_has_clock_corrections_store_map[gal_str][prn];
                                            clock_correction.clock_correction_m = clock_correction_mult_m;
                                            clock_correction.valid_until = valid_until;
                                            clock_correction.iod = gnss_iod;
                                            clock_correction.mask_id = new_has_data.header.mask_id;
                                            clock_correction.iod_set_id = new_has_data.header.iod_set_id;
                                        }
                                }
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
                            if (Galileo_HAS_data::is_code_bias_unavailable(code_bias_m))
                                {
                                    d_has_code_bias_store_map[it].erase(prn);
                                    continue;
                                }
                            auto &code_bias_correction = d_has_code_bias_store_map[it][prn];
                            code_bias_correction.bias = code_bias_m;
                            code_bias_correction.valid_until = valid_until;
                            code_bias_correction.mask_id = new_has_data.header.mask_id;
                            code_bias_correction.iod_set_id = new_has_data.header.iod_set_id;
                        }
                }
            auto signals_gps = new_has_data.get_signals_in_mask(gps_str);
            for (const auto &it : signals_gps)
                {
                    auto prns = new_has_data.get_PRNs_in_mask(gps_str);
                    for (auto prn : prns)
                        {
                            float code_bias_m = new_has_data.get_code_bias_m(it, prn);
                            if (Galileo_HAS_data::is_code_bias_unavailable(code_bias_m))
                                {
                                    d_has_code_bias_store_map[it].erase(prn);
                                    continue;
                                }
                            auto &code_bias_correction = d_has_code_bias_store_map[it][prn];
                            code_bias_correction.bias = code_bias_m;
                            code_bias_correction.valid_until = valid_until;
                            code_bias_correction.mask_id = new_has_data.header.mask_id;
                            code_bias_correction.iod_set_id = new_has_data.header.iod_set_id;
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
                            if (Galileo_HAS_data::is_phase_bias_unavailable(phase_bias_correction_cycles))
                                {
                                    d_has_phase_bias_store_map[it].erase(prn);
                                    d_has_phase_discontinuity_indicator_store_map[it].erase(prn);
                                    d_has_phase_bias_discontinuity_map[it].erase(prn);
                                    continue;
                                }
                            auto &phase_bias_correction = d_has_phase_bias_store_map[it][prn];
                            phase_bias_correction.bias = phase_bias_correction_cycles;
                            phase_bias_correction.valid_until = valid_until;
                            phase_bias_correction.mask_id = new_has_data.header.mask_id;
                            phase_bias_correction.iod_set_id = new_has_data.header.iod_set_id;
                            uint8_t phase_discontinuity_indicator = new_has_data.get_phase_discontinuity_indicator(it, prn);
                            auto stored_pdi_it = d_has_phase_discontinuity_indicator_store_map[it].find(prn);
                            if ((stored_pdi_it != d_has_phase_discontinuity_indicator_store_map[it].end()) &&
                                (stored_pdi_it->second != phase_discontinuity_indicator))
                                {
                                    d_has_phase_bias_discontinuity_map[it][prn] = true;
                                }
                            else if (d_has_phase_bias_discontinuity_map[it].find(prn) == d_has_phase_bias_discontinuity_map[it].end())
                                {
                                    d_has_phase_bias_discontinuity_map[it][prn] = false;
                                }
                            d_has_phase_discontinuity_indicator_store_map[it][prn] = phase_discontinuity_indicator;
                        }
                }
            auto signals_gps = new_has_data.get_signals_in_mask(gps_str);
            for (const auto &it : signals_gps)
                {
                    auto prns = new_has_data.get_PRNs_in_mask(gps_str);
                    for (auto prn : prns)
                        {
                            float phase_bias_correction_cycles = new_has_data.get_phase_bias_cycle(it, prn);
                            if (Galileo_HAS_data::is_phase_bias_unavailable(phase_bias_correction_cycles))
                                {
                                    d_has_phase_bias_store_map[it].erase(prn);
                                    d_has_phase_discontinuity_indicator_store_map[it].erase(prn);
                                    d_has_phase_bias_discontinuity_map[it].erase(prn);
                                    continue;
                                }
                            auto &phase_bias_correction = d_has_phase_bias_store_map[it][prn];
                            phase_bias_correction.bias = phase_bias_correction_cycles;
                            phase_bias_correction.valid_until = valid_until;
                            phase_bias_correction.mask_id = new_has_data.header.mask_id;
                            phase_bias_correction.iod_set_id = new_has_data.header.iod_set_id;
                            uint8_t phase_discontinuity_indicator = new_has_data.get_phase_discontinuity_indicator(it, prn);
                            auto stored_pdi_it = d_has_phase_discontinuity_indicator_store_map[it].find(prn);
                            if ((stored_pdi_it != d_has_phase_discontinuity_indicator_store_map[it].end()) &&
                                (stored_pdi_it->second != phase_discontinuity_indicator))
                                {
                                    d_has_phase_bias_discontinuity_map[it][prn] = true;
                                }
                            else if (d_has_phase_bias_discontinuity_map[it].find(prn) == d_has_phase_bias_discontinuity_map[it].end())
                                {
                                    d_has_phase_bias_discontinuity_map[it][prn] = false;
                                }
                            d_has_phase_discontinuity_indicator_store_map[it][prn] = phase_discontinuity_indicator;
                        }
                }
        }
}


void Rtklib_Solver::update_has_corrections(const std::map<int, Gnss_Synchro> &obs_map)
{
    this->check_has_orbit_clock_validity(obs_map);
    this->get_has_biases(obs_map);
}


bool Rtklib_Solver::has_active_has_do_not_use(const std::string &system, int prn, uint32_t tow_obs) const
{
    const auto it_sys = d_has_satellite_do_not_use_until.find(system);
    if (it_sys == d_has_satellite_do_not_use_until.cend())
        {
            return false;
        }
    const auto it_prn = it_sys->second.find(prn);
    if (it_prn == it_sys->second.cend())
        {
            return false;
        }
    return Galileo_HAS_data::has_validity_interval_at_tow(it_prn->second, tow_obs);
}


bool Rtklib_Solver::get_active_has_context(const std::string &system, int prn, uint16_t sis_iod, uint32_t tow_obs, uint8_t &mask_id, uint8_t &iod_set_id) const
{
    const auto orbit_sys_it = d_has_orbit_corrections_store_map.find(system);
    if (orbit_sys_it == d_has_orbit_corrections_store_map.cend())
        {
            return false;
        }
    const auto orbit_prn_it = orbit_sys_it->second.find(prn);
    if (orbit_prn_it == orbit_sys_it->second.cend())
        {
            return false;
        }
    if (!Galileo_HAS_data::has_validity_interval_at_tow(orbit_prn_it->second.valid_until, tow_obs))
        {
            return false;
        }
    if (orbit_prn_it->second.iod != sis_iod)
        {
            return false;
        }

    const auto clock_sys_it = d_has_clock_corrections_store_map.find(system);
    if (clock_sys_it == d_has_clock_corrections_store_map.cend())
        {
            return false;
        }
    const auto clock_prn_it = clock_sys_it->second.find(prn);
    if (clock_prn_it == clock_sys_it->second.cend())
        {
            return false;
        }
    if (!Galileo_HAS_data::has_validity_interval_at_tow(clock_prn_it->second.valid_until, tow_obs))
        {
            return false;
        }
    if (clock_prn_it->second.iod != sis_iod)
        {
            return false;
        }
    if ((orbit_prn_it->second.mask_id != clock_prn_it->second.mask_id) ||
        (orbit_prn_it->second.iod_set_id != clock_prn_it->second.iod_set_id))
        {
            return false;
        }

    mask_id = orbit_prn_it->second.mask_id;
    iod_set_id = orbit_prn_it->second.iod_set_id;
    return true;
}


bool Rtklib_Solver::has_active_has_orbit_clock(const std::string &system, int prn, uint16_t sis_iod, uint32_t tow_obs) const
{
    uint8_t mask_id = 0;
    uint8_t iod_set_id = 0;
    return get_active_has_context(system, prn, sis_iod, tow_obs, mask_id, iod_set_id);
}


void Rtklib_Solver::check_has_orbit_clock_validity(const std::map<int, Gnss_Synchro> &obs_map)
{
    for (const auto &it : obs_map)
        {
            uint32_t obs_tow = it.second.interp_TOW_ms / 1000.0;
            auto prn = static_cast<int>(it.second.PRN);
            std::string system;
            bool have_sis_iod = false;
            uint16_t sis_iod = 0;

            if (it.second.System == 'G')
                {
                    system = "GPS";
                    auto eph_it = gps_ephemeris_map.find(it.second.PRN);
                    if (eph_it != gps_ephemeris_map.end())
                        {
                            sis_iod = static_cast<uint16_t>(eph_it->second.IODE_SF3);
                            have_sis_iod = true;
                        }
                }
            else if (it.second.System == 'E')
                {
                    system = "Galileo";
                    const auto &galileo_ephemeris_map = galileo_ephemeris_store.by_source(d_galileo_nav_message_type_for_pvt);
                    auto eph_it = galileo_ephemeris_map.find(it.second.PRN);
                    if (eph_it != galileo_ephemeris_map.end())
                        {
                            sis_iod = static_cast<uint16_t>(eph_it->second.IOD_ephemeris);
                            have_sis_iod = true;
                        }
                }
            else
                {
                    continue;
                }

            auto orbit_sys_it = d_has_orbit_corrections_store_map.find(system);
            auto clock_sys_it = d_has_clock_corrections_store_map.find(system);
            bool erase_orbit = false;
            bool erase_clock = false;

            if (orbit_sys_it != d_has_orbit_corrections_store_map.end())
                {
                    auto orbit_it = orbit_sys_it->second.find(prn);
                    if (orbit_it != orbit_sys_it->second.end())
                        {
                            if (!Galileo_HAS_data::has_validity_interval_at_tow(orbit_it->second.valid_until, obs_tow) ||
                                (have_sis_iod && orbit_it->second.iod != sis_iod))
                                {
                                    erase_orbit = true;
                                }
                        }
                }

            if (clock_sys_it != d_has_clock_corrections_store_map.end())
                {
                    auto clock_it = clock_sys_it->second.find(prn);
                    if (clock_it != clock_sys_it->second.end())
                        {
                            if (!Galileo_HAS_data::has_validity_interval_at_tow(clock_it->second.valid_until, obs_tow) ||
                                (have_sis_iod && clock_it->second.iod != sis_iod))
                                {
                                    erase_clock = true;
                                }
                        }
                }

            if (have_sis_iod && !erase_orbit && !erase_clock &&
                orbit_sys_it != d_has_orbit_corrections_store_map.end() &&
                clock_sys_it != d_has_clock_corrections_store_map.end())
                {
                    auto orbit_it = orbit_sys_it->second.find(prn);
                    auto clock_it = clock_sys_it->second.find(prn);
                    if (orbit_it != orbit_sys_it->second.end() &&
                        clock_it != clock_sys_it->second.end() &&
                        ((orbit_it->second.mask_id != clock_it->second.mask_id) ||
                            (orbit_it->second.iod_set_id != clock_it->second.iod_set_id)))
                        {
                            erase_clock = true;
                        }
                }

            if (erase_orbit && orbit_sys_it != d_has_orbit_corrections_store_map.end())
                {
                    orbit_sys_it->second.erase(prn);
                    erase_clock = true;
                }
            if (erase_clock && clock_sys_it != d_has_clock_corrections_store_map.end())
                {
                    clock_sys_it->second.erase(prn);
                }

            auto dnu_sys_it = d_has_satellite_do_not_use_until.find(system);
            if (dnu_sys_it != d_has_satellite_do_not_use_until.end())
                {
                    auto dnu_it = dnu_sys_it->second.find(prn);
                    if (dnu_it != dnu_sys_it->second.end())
                        {
                            if (!Galileo_HAS_data::has_validity_interval_at_tow(dnu_it->second, obs_tow))
                                {
                                    dnu_sys_it->second.erase(prn);
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
            const std::vector<std::string> e7_signals = {"E5b-I I/NAV OS", "E5b-Q", "E5b-I+E5b-Q"};
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
                            const auto &galileo_ephemeris_map = galileo_ephemeris_store.by_source(d_galileo_nav_message_type_for_pvt);
                            auto eph_it = galileo_ephemeris_map.find(it.second.PRN);
                            uint8_t mask_id = 0;
                            uint8_t iod_set_id = 0;
                            if (eph_it != galileo_ephemeris_map.end() &&
                                get_active_has_context("Galileo", prn, static_cast<uint16_t>(eph_it->second.IOD_ephemeris), obs_tow, mask_id, iod_set_id))
                                {
                                    if (sig == "1B")
                                        {
                                            for (const auto &has_signal : e1b_signals)
                                                {
                                                    this->get_current_has_obs_correction(has_signal, obs_tow, prn, mask_id, iod_set_id);
                                                }
                                        }
                                    else if (sig == "E6")
                                        {
                                            for (const auto &has_signal : e6_signals)
                                                {
                                                    this->get_current_has_obs_correction(has_signal, obs_tow, prn, mask_id, iod_set_id);
                                                }
                                        }
                                    else if (sig == "5X")
                                        {
                                            for (const auto &has_signal : e5_signals)
                                                {
                                                    this->get_current_has_obs_correction(has_signal, obs_tow, prn, mask_id, iod_set_id);
                                                }
                                        }
                                    else if (sig == "7X")
                                        {
                                            for (const auto &has_signal : e7_signals)
                                                {
                                                    this->get_current_has_obs_correction(has_signal, obs_tow, prn, mask_id, iod_set_id);
                                                }
                                        }
                                }
                        }
                    if (it.second.System == 'G')
                        {
                            auto eph_it = gps_ephemeris_map.find(it.second.PRN);
                            uint8_t mask_id = 0;
                            uint8_t iod_set_id = 0;
                            if (eph_it != gps_ephemeris_map.end() &&
                                get_active_has_context("GPS", prn, static_cast<uint16_t>(eph_it->second.IODE_SF3), obs_tow, mask_id, iod_set_id))
                                {
                                    if (sig == "1C")
                                        {
                                            for (const auto &has_signal : g1c_signals)
                                                {
                                                    this->get_current_has_obs_correction(has_signal, obs_tow, prn, mask_id, iod_set_id);
                                                }
                                        }
                                    else if (sig == "2S")
                                        {
                                            for (const auto &has_signal : g2s_signals)
                                                {
                                                    this->get_current_has_obs_correction(has_signal, obs_tow, prn, mask_id, iod_set_id);
                                                }
                                        }
                                    else if (sig == "L5")
                                        {
                                            for (const auto &has_signal : g5_signals)
                                                {
                                                    this->get_current_has_obs_correction(has_signal, obs_tow, prn, mask_id, iod_set_id);
                                                }
                                        }
                                }
                        }
                }
        }
}


void Rtklib_Solver::get_current_has_obs_correction(const std::string &signal, uint32_t tow_obs, int prn, uint8_t mask_id, uint8_t iod_set_id)
{
    auto code_bias_signal_it = this->d_has_code_bias_store_map.find(signal);
    if (code_bias_signal_it != this->d_has_code_bias_store_map.end())
        {
            auto code_bias_pair_it = code_bias_signal_it->second.find(prn);
            if (code_bias_pair_it != code_bias_signal_it->second.end())
                {
                    const auto &code_bias_correction = code_bias_pair_it->second;
                    if (Galileo_HAS_data::has_validity_interval_at_tow(code_bias_correction.valid_until, tow_obs) &&
                        (code_bias_correction.mask_id == mask_id) &&
                        (code_bias_correction.iod_set_id == iod_set_id))
                        {
                            auto &has_obs_correction = this->d_has_obs_corr_map[signal][prn];
                            has_obs_correction.signal = signal;
                            has_obs_correction.code_bias_m = code_bias_correction.bias;
                        }
                }
        }
    auto phase_bias_signal_it = this->d_has_phase_bias_store_map.find(signal);
    if (phase_bias_signal_it != this->d_has_phase_bias_store_map.end())
        {
            auto phase_bias_pair_it = phase_bias_signal_it->second.find(prn);
            if (phase_bias_pair_it != phase_bias_signal_it->second.end())
                {
                    const auto &phase_bias_correction = phase_bias_pair_it->second;
                    if (Galileo_HAS_data::has_validity_interval_at_tow(phase_bias_correction.valid_until, tow_obs) &&
                        (phase_bias_correction.mask_id == mask_id) &&
                        (phase_bias_correction.iod_set_id == iod_set_id))
                        {
                            auto &has_obs_correction = this->d_has_obs_corr_map[signal][prn];
                            has_obs_correction.signal = signal;
                            has_obs_correction.phase_bias_cycle = phase_bias_correction.bias;

                            const auto pdi_signal_it = this->d_has_phase_discontinuity_indicator_store_map.find(signal);
                            if (pdi_signal_it != this->d_has_phase_discontinuity_indicator_store_map.end())
                                {
                                    const auto pdi_it = pdi_signal_it->second.find(prn);
                                    if (pdi_it != pdi_signal_it->second.end())
                                        {
                                            has_obs_correction.phase_discontinuity_indicator = pdi_it->second;
                                        }
                                }

                            auto discontinuity_signal_it = this->d_has_phase_bias_discontinuity_map.find(signal);
                            if (discontinuity_signal_it != this->d_has_phase_bias_discontinuity_map.end())
                                {
                                    auto discontinuity_it = discontinuity_signal_it->second.find(prn);
                                    if (discontinuity_it != discontinuity_signal_it->second.end())
                                        {
                                            has_obs_correction.phase_bias_discontinuity = discontinuity_it->second;
                                        }
                                }
                        }
                }
        }
}


bool Rtklib_Solver::galileo_ephemeris_is_usable(const Galileo_Ephemeris &ephemeris, uint32_t observation_tow) const
{
    if (observation_tow >= 604800U)
        {
            return false;
        }
    double toe_distance = std::fabs(static_cast<double>(observation_tow) - ephemeris.toe);
    if (toe_distance > 302400.0)
        {
            toe_distance = 604800.0 - toe_distance;
        }
    return toe_distance <= MAXDTOE_GAL;
}


void Rtklib_Solver::update_galileo_observation_wavelengths(const obsd_t &observation)
{
    if (satsys(observation.sat, nullptr) != SYS_GAL)
        {
            return;
        }

    for (int slot = 0; slot < NFREQ; ++slot)
        {
            if (observation.code[slot] == CODE_NONE)
                {
                    continue;
                }

            int frequency = 0;
            code2obs(observation.code[slot], &frequency);
            if (frequency <= 0)
                {
                    continue;
                }

            const double wavelength = satwavelen(observation.sat, frequency - 1, &d_nav_data);
            if (wavelength > 0.0)
                {
                    d_nav_data.lam[observation.sat - 1][slot] = wavelength;
                }
        }
}


void Rtklib_Solver::clear_applied_has_phase_bias_discontinuity(const HAS_obs_corrections *has_correction, int prn)
{
    if (has_correction == nullptr || !has_correction->phase_bias_discontinuity || has_correction->signal.empty())
        {
            return;
        }

    auto discontinuity_signal_it = this->d_has_phase_bias_discontinuity_map.find(has_correction->signal);
    if (discontinuity_signal_it == this->d_has_phase_bias_discontinuity_map.end())
        {
            return;
        }

    auto discontinuity_it = discontinuity_signal_it->second.find(prn);
    if (discontinuity_it != discontinuity_signal_it->second.end())
        {
            discontinuity_it->second = false;
        }
}


bool Rtklib_Solver::store_galileo_ephemeris(const Galileo_Ephemeris &ephemeris)
{
    Galileo_Ephemeris source_ephemeris = ephemeris;
    if (source_ephemeris.nav_message_type == Galileo_Nav_Message_Type::Unknown)
        {
            source_ephemeris.nav_message_type = d_galileo_nav_message_type_for_pvt;
            LOG(WARNING) << "Galileo ephemeris for PRN " << source_ephemeris.PRN
                         << " has no navigation-message provenance; assigning the receiver's automatic PVT source";
        }
    if (!galileo_ephemeris_store.insert(source_ephemeris))
        {
            return false;
        }
    const auto compatibility_entry = galileo_ephemeris_map.find(static_cast<int>(source_ephemeris.PRN));
    if (source_ephemeris.nav_message_type == d_galileo_nav_message_type_for_pvt ||
        compatibility_entry == galileo_ephemeris_map.cend())
        {
            galileo_ephemeris_map[static_cast<int>(source_ephemeris.PRN)] = source_ephemeris;
        }
    return true;
}


Galileo_Nav_Message_Type Rtklib_Solver::galileo_nav_message_type_for_pvt() const
{
    return d_galileo_nav_message_type_for_pvt;
}


bool Rtklib_Solver::is_galileo_signal_used_in_pvt(const std::string &signal) const
{
    if (signal == "E6")
        {
            return d_conf.use_e6_for_pvt;
        }
    if (d_galileo_nav_message_type_for_pvt == Galileo_Nav_Message_Type::FNAV)
        {
            return signal == "1B" ||
                   (signal == "5X" && d_rtklib_band_index.find(signal) != d_rtklib_band_index.cend());
        }
    return signal == "1B" ||
           (signal == "7X" && d_rtklib_band_index.find(signal) != d_rtklib_band_index.cend());
}


bool Rtklib_Solver::get_galileo_signal_health(uint32_t prn, const std::string &signal,
    uint32_t observation_tow, bool &healthy) const
{
    Galileo_Nav_Message_Type health_source = Galileo_Nav_Message_Type::Unknown;
    if (signal == "5X")
        {
            health_source = Galileo_Nav_Message_Type::FNAV;
        }
    else if (signal == "1B" || signal == "7X")
        {
            health_source = Galileo_Nav_Message_Type::INAV;
        }
    else
        {
            return false;
        }

    const Galileo_Ephemeris *health_ephemeris = galileo_ephemeris_store.find(static_cast<int>(prn), health_source);
    if (health_ephemeris == nullptr || !galileo_ephemeris_is_usable(*health_ephemeris, observation_tow))
        {
            return false;
        }

    if (signal == "1B")
        {
            healthy = !health_ephemeris->E1B_DVS && health_ephemeris->E1B_HS == 0;
        }
    else if (signal == "5X")
        {
            healthy = !health_ephemeris->E5a_DVS && health_ephemeris->E5a_HS == 0;
        }
    else
        {
            healthy = !health_ephemeris->E5b_DVS && health_ephemeris->E5b_HS == 0;
        }
    return true;
}


std::map<int, Galileo_Ephemeris> Rtklib_Solver::get_galileo_ephemeris_map_for_pvt() const
{
    auto result = galileo_ephemeris_store.combined_view(d_galileo_nav_message_type_for_pvt);
    for (const auto &ephemeris : galileo_ephemeris_map)
        {
            if (result.find(ephemeris.first) == result.cend())
                {
                    result.insert(ephemeris);
                }
        }
    return result;
}


bool Rtklib_Solver::select_galileo_ephemeris(uint32_t prn, const std::string &signal, uint32_t observation_tow,
    Galileo_Ephemeris &ephemeris, bool &from_reduced_ced) const
{
    from_reduced_ced = false;
    if (!is_galileo_signal_used_in_pvt(signal) || observation_tow >= 604800U)
        {
            return false;
        }

    const Galileo_Ephemeris *full_ephemeris = galileo_ephemeris_store.find(
        static_cast<int>(prn), d_galileo_nav_message_type_for_pvt);
    const auto compatibility_ephemeris = galileo_ephemeris_map.find(static_cast<int>(prn));
    if (full_ephemeris == nullptr && compatibility_ephemeris != galileo_ephemeris_map.cend() &&
        (compatibility_ephemeris->second.nav_message_type == Galileo_Nav_Message_Type::Unknown ||
            compatibility_ephemeris->second.nav_message_type == d_galileo_nav_message_type_for_pvt))
        {
            full_ephemeris = &compatibility_ephemeris->second;
        }
    if (full_ephemeris != nullptr && galileo_ephemeris_is_usable(*full_ephemeris, observation_tow))
        {
            ephemeris = *full_ephemeris;
            return true;
        }

    // The ICD only defines Reduced CED use for the E1/E5b service.
    if (d_galileo_nav_message_type_for_pvt != Galileo_Nav_Message_Type::INAV ||
        (signal != "1B" && signal != "7X"))
        {
            return false;
        }

    const auto reduced_ced = galileo_reduced_ced_map.find(static_cast<int>(prn));
    if (reduced_ced == galileo_reduced_ced_map.cend())
        {
            return false;
        }

    uint32_t observation_week = reduced_ced->second.WN;
    uint32_t week_reference_tow = reduced_ced->second.TOTRedCED;
    // A later GST model prevents a stale Reduced CED from becoming valid
    // again after a week. Older assistance data must not override the directly
    // decoded Reduced CED epoch.
    if (galileo_iono.WN > 0 &&
        static_cast<uint32_t>(galileo_iono.WN) > reduced_ced->second.WN &&
        galileo_iono.tow >= 0 && galileo_iono.tow < 604800)
        {
            observation_week = static_cast<uint32_t>(galileo_iono.WN);
            week_reference_tow = static_cast<uint32_t>(galileo_iono.tow);
        }

    if (observation_tow < week_reference_tow &&
        week_reference_tow - observation_tow > 302400U)
        {
            if (observation_week == std::numeric_limits<uint32_t>::max())
                {
                    return false;
                }
            ++observation_week;
        }
    else if (observation_tow > week_reference_tow &&
             observation_tow - week_reference_tow > 302400U)
        {
            if (observation_week == 0U)
                {
                    return false;
                }
            --observation_week;
        }

    if (!reduced_ced->second.is_valid_at(observation_week, observation_tow))
        {
            return false;
        }

    ephemeris = reduced_ced->second.compute_eph();
    from_reduced_ced = true;
    return true;
}


void Rtklib_Solver::reset_relative_filter()
{
    const prcopt_t options = d_rtk.opt;
    rtkfree(&d_rtk);
    rtkinit(&d_rtk, &options);
    if (d_conf.enable_pvt_kf)
        {
            d_pvt_kf.reset_Kf();
        }
}


bool Rtklib_Solver::prepare_fixed_base_observations(const Ntrip_Rtcm_Snapshot &fixed_base,
    int &rover_observation_count,
    int &base_observation_count)
{
    // Per-constellation band-to-slot mapping, shared with the adapter's
    // channel-set validation (see ntrip_rtk_signals() in ntrip_rtk_signals.h)
    // and resolved once at construction
    const auto second_band_slot = [this](int system) {
        switch (system)
            {
            case SYS_GAL:
                return d_ntrip_second_band_slot_gal;
            case SYS_BDS:
                return d_ntrip_second_band_slot_bds;
            default:
                return d_ntrip_second_band_slot_gps;
            }
    };
    const auto clear_unsupported_slots = [&second_band_slot](obsd_t &observation, int system) {
        const int second = second_band_slot(system);
        for (int frequency = 1; frequency < NFREQ + NEXOBS; ++frequency)
            {
                if (frequency == second)
                    {
                        continue;
                    }
                observation.P[frequency] = 0.0;
                observation.L[frequency] = 0.0;
                observation.D[frequency] = 0.0F;
                observation.SNR[frequency] = 0;
                observation.LLI[frequency] = 0;
                observation.code[frequency] = CODE_NONE;
            }
    };
    const auto has_supported_band_measurement = [&second_band_slot](const obsd_t &observation, int system) {
        const std::array<int, 2> slots = {0, second_band_slot(system)};
        for (const int frequency : slots)
            {
                if (observation.code[frequency] != CODE_NONE &&
                    (observation.P[frequency] != 0.0 || observation.L[frequency] != 0.0))
                    {
                        return true;
                    }
            }
        return false;
    };
    const auto merge_observation = [](obsd_t &destination, const obsd_t &source) {
        for (int frequency = 0; frequency < NFREQ + NEXOBS; ++frequency)
            {
                if (source.code[frequency] == CODE_NONE && source.P[frequency] == 0.0 && source.L[frequency] == 0.0)
                    {
                        continue;
                    }
                destination.P[frequency] = source.P[frequency];
                destination.L[frequency] = source.L[frequency];
                destination.D[frequency] = source.D[frequency];
                destination.SNR[frequency] = source.SNR[frequency];
                destination.LLI[frequency] = source.LLI[frequency];
                destination.code[frequency] = source.code[frequency];
            }
    };
    const auto sort_and_merge = [&merge_observation](std::vector<obsd_t> &observations) {
        std::sort(observations.begin(), observations.end(), [](const obsd_t &left, const obsd_t &right) {
            return left.sat < right.sat;
        });
        /* in-place merge of same-satellite entries: no scratch allocation */
        std::size_t out = 0;
        for (std::size_t in = 0; in < observations.size(); ++in)
            {
                if (out > 0 && observations[out - 1].sat == observations[in].sat)
                    {
                        merge_observation(observations[out - 1], observations[in]);
                    }
                else
                    {
                        if (out != in)
                            {
                                observations[out] = observations[in];
                            }
                        ++out;
                    }
            }
        observations.resize(out);
    };

    /* member scratch vectors: their capacity persists across epochs, so this
       per-epoch path stops allocating once warmed up */
    std::vector<obsd_t> &rover_observations = d_fixed_base_rover_scratch;
    rover_observations.clear();
    rover_observations.reserve(static_cast<std::size_t>(rover_observation_count));
    for (int index = 0; index < rover_observation_count; ++index)
        {
            obsd_t observation = d_obs_data[index];
            /* the fixed-base path admits exactly the systems the whole chain
               supports (signal table, adapter navigation-system mask, client
               observation filter, and this merge): QZSS is deliberately not
               among them yet — its rover observations would be excluded by
               satexclude() anyway and could never find base counterparts */
            const int system = observation.sat > 0 ? satsys(observation.sat, nullptr) : 0;
            if (system != SYS_GPS && system != SYS_GAL && system != SYS_BDS)
                {
                    continue;
                }
            observation.rcv = 1;
            clear_unsupported_slots(observation, system);
            if (has_supported_band_measurement(observation, system))
                {
                    rover_observations.push_back(observation);
                }
        }
    sort_and_merge(rover_observations);
    if (rover_observations.size() > MAXOBS)
        {
            rover_observations.resize(MAXOBS);
        }
    /* d_obs_data and the counts are rewritten only when the base data is
       actually applied (at the end of this function): every early return
       below leaves the full multi-constellation rover set untouched so the
       single-point fallback solves with the same satellites as a receiver
       without NTRIP */

    if (!fixed_base.has_observations || rover_observations.empty())
        {
            d_fixed_base_status = Rtklib_Fixed_Base_Status::MISSING_OBSERVATIONS;
            return false;
        }
    if (!fixed_base.has_base_position)
        {
            d_fixed_base_status = Rtklib_Fixed_Base_Status::MISSING_POSITION;
            return false;
        }

    d_fixed_base_age_s = timediff(rover_observations.front().time, fixed_base.observation_time);
    if (!std::isfinite(d_fixed_base_age_s) || std::fabs(d_fixed_base_age_s) > d_conf.ntrip_max_correction_age_s)
        {
            d_fixed_base_status = Rtklib_Fixed_Base_Status::STALE;
            return false;
        }

    double base_position_norm_squared = 0.0;
    for (const double coordinate : fixed_base.base_position_ecef_m)
        {
            if (!std::isfinite(coordinate))
                {
                    d_fixed_base_status = Rtklib_Fixed_Base_Status::INVALID_POSITION;
                    return false;
                }
            base_position_norm_squared += coordinate * coordinate;
        }
    if (base_position_norm_squared < 1.0e12 || base_position_norm_squared > 1.0e16)
        {
            d_fixed_base_status = Rtklib_Fixed_Base_Status::INVALID_POSITION;
            return false;
        }

    std::vector<obsd_t> &base_observations = d_fixed_base_base_scratch;
    base_observations.clear();
    base_observations.reserve(fixed_base.observations.size());
    for (auto observation : fixed_base.observations)
        {
            const int system = observation.sat > 0 ? satsys(observation.sat, nullptr) : 0;
            if (system != SYS_GPS && system != SYS_GAL && system != SYS_BDS)
                {
                    continue;
                }
            observation.rcv = 2;
            clear_unsupported_slots(observation, system);
            if (!has_supported_band_measurement(observation, system))
                {
                    continue;
                }
            const auto rover = std::lower_bound(rover_observations.cbegin(), rover_observations.cend(), observation.sat,
                [](const obsd_t &candidate, int satellite) { return candidate.sat < satellite; });
            if (rover != rover_observations.cend() && rover->sat == observation.sat)
                {
                    base_observations.push_back(observation);
                }
        }
    sort_and_merge(base_observations);
    if (base_observations.size() > MAXOBS)
        {
            base_observations.resize(MAXOBS);
        }
    d_fixed_base_common_satellites = base_observations.size();
    if (d_fixed_base_common_satellites < NTRIP_MIN_COMMON_SATELLITES)
        {
            d_fixed_base_status = Rtklib_Fixed_Base_Status::INSUFFICIENT_COMMON_SATELLITES;
            return false;
        }

    bool base_changed = d_fixed_base_initialized && d_fixed_base_station_id != fixed_base.station_id;
    double position_delta_squared = 0.0;
    for (std::size_t index = 0; index < d_fixed_base_position_ecef_m.size(); ++index)
        {
            const double delta = fixed_base.base_position_ecef_m[index] - d_fixed_base_position_ecef_m[index];
            position_delta_squared += delta * delta;
        }
    base_changed = base_changed || (d_fixed_base_initialized && position_delta_squared > 1.0e-6);
    const bool correction_gap = d_fixed_base_was_applied &&
                                d_fixed_base_last_observation_time.time != 0 &&
                                std::fabs(timediff(fixed_base.observation_time,
                                    d_fixed_base_last_observation_time)) > d_conf.ntrip_max_correction_age_s;
    if (base_changed || correction_gap)
        {
            reset_relative_filter();
        }
    d_fixed_base_initialized = true;
    d_fixed_base_station_id = fixed_base.station_id;
    d_fixed_base_position_ecef_m = fixed_base.base_position_ecef_m;
    d_fixed_base_last_observation_time = fixed_base.observation_time;
    for (std::size_t index = 0; index < d_fixed_base_position_ecef_m.size(); ++index)
        {
            d_rtk.opt.rb[index] = d_fixed_base_position_ecef_m[index];
        }

    /* slots beyond the entry rover count are already zero (get_PVT clears the
       whole array at epoch start), so only the leftover entry-rover slots the
       filtered set no longer covers need clearing */
    const int previously_filled = rover_observation_count;
    rover_observation_count = static_cast<int>(rover_observations.size());
    std::copy(rover_observations.cbegin(), rover_observations.cend(), d_obs_data.begin());
    std::copy(base_observations.cbegin(), base_observations.cend(), d_obs_data.begin() + rover_observation_count);
    base_observation_count = static_cast<int>(base_observations.size());
    const int used = rover_observation_count + base_observation_count;
    if (used < previously_filled)
        {
            std::fill(d_obs_data.begin() + used, d_obs_data.begin() + previously_filled, obsd_t{});
        }
    d_fixed_base_status = Rtklib_Fixed_Base_Status::APPLIED;
    return true;
}


/* Latest base-stream broadcast ephemeris for a satellite, or nullptr. The
   records are RTKLIB-format already (RTCM MT1019/1044/1045/1046/1042), so a
   satellite the rover has not finished decoding can still contribute. */
static const eph_t *find_base_ephemeris(const Ntrip_Rtcm_Snapshot *fixed_base, int sat)
{
    if (fixed_base == nullptr)
        {
            return nullptr;
        }
    // the client keeps the vector sorted by satellite number
    const auto candidate = std::lower_bound(
        fixed_base->ephemerides.cbegin(), fixed_base->ephemerides.cend(), sat,
        [](const eph_t &stored, int wanted) { return stored.sat < wanted; });
    if (candidate != fixed_base->ephemerides.cend() && candidate->sat == sat)
        {
            return &(*candidate);
        }
    return nullptr;
}


bool Rtklib_Solver::substitute_base_ephemeris(const Ntrip_Rtcm_Snapshot *fixed_base,
    int sat,
    const Gnss_Synchro &gnss_synchro,
    const std::string &band_key,
    std::vector<eph_t> &eph_data,
    int &valid_obs,
    int glo_valid_obs)
{
    const eph_t *base_ephemeris = find_base_ephemeris(fixed_base, sat);
    if (base_ephemeris == nullptr)
        {
            return false;
        }
    // the substituted record's own reference time, expressed in the week
    // convention the rover-decoded path uses for the system
    const int system = satsys(sat, nullptr);
    int week = 0;
    switch (system)
        {
        case SYS_GAL:
            time2gst(base_ephemeris->toe, &week);
            break;
        case SYS_BDS:
            time2gpst(base_ephemeris->toe, &week);
            break;
        default:
            // GPS/QZSS: RTCM MT1019/MT1044 carry the full GPS week
            week = base_ephemeris->week;
            break;
        }
    eph_data[valid_obs] = *base_ephemeris;
    obsd_t newobs{};
    if (system == SYS_BDS)
        {
            // HAS corrections cover GPS and Galileo only
            d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                gnss_synchro, week, d_rtklib_band_index.at(band_key));
        }
    else
        {
            // HAS observation corrections apply to the measurement no matter
            // where the ephemeris came from: mirror the rover-decoded path
            const HAS_obs_corrections *applied_has_correction = nullptr;
            d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                gnss_synchro,
                d_has_obs_corr_map,
                week,
                d_rtklib_band_index.at(band_key),
                &applied_has_correction,
                system == SYS_GAL ? 0 : this->get_ref_gps_week());
            clear_applied_has_phase_bias_discontinuity(applied_has_correction,
                static_cast<int>(gnss_synchro.PRN));
        }
    valid_obs++;
    return true;
}


bool Rtklib_Solver::get_PVT(const std::map<int, Gnss_Synchro> &gnss_observables_map,
    double kf_update_interval_s,
    const SensorDataAggregator &sensor_data_aggregator,
    bool dump_this_epoch,
    const Ntrip_Rtcm_Snapshot *fixed_base)
{
    std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
    std::map<int, Gps_Ephemeris>::const_iterator gps_ephemeris_iter;
    std::map<int, Gps_CNAV_Ephemeris>::const_iterator gps_cnav_ephemeris_iter;
    std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_ephemeris_iter;
    std::map<int, Beidou_Dnav_Ephemeris>::const_iterator beidou_ephemeris_iter;

    const Glonass_Gnav_Utc_Model &gnav_utc = this->glonass_gnav_utc_model;

    // ********************************************************************************
    // ****** PREPARE THE DATA (SV EPHEMERIS AND OBSERVATIONS) ************************
    // ********************************************************************************
    int valid_obs = 0;      // valid observations counter
    int glo_valid_obs = 0;  // GLONASS L1/L2 valid observations counter

    d_fixed_base_status = fixed_base == nullptr ? Rtklib_Fixed_Base_Status::NOT_REQUESTED : Rtklib_Fixed_Base_Status::MISSING_OBSERVATIONS;
    d_fixed_base_age_s = 0.0;
    d_fixed_base_common_satellites = 0;

    d_obs_data.fill({});
    std::vector<eph_t> eph_data(MAXOBS);
    std::vector<geph_t> geph_data(MAXOBS);

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
                        const uint32_t obs_tow = gnss_observables_iter->second.interp_TOW_ms / 1000.0;
                        const int prn = static_cast<int>(gnss_observables_iter->second.PRN);
                        Galileo_Ephemeris selected_galileo_ephemeris;
                        bool selected_from_reduced_ced = false;
                        const bool has_selected_galileo_ephemeris = select_galileo_ephemeris(
                            gnss_observables_iter->second.PRN, sig_, obs_tow,
                            selected_galileo_ephemeris, selected_from_reduced_ced);
                        // the do-not-use exclusion must hold whether the ephemeris
                        // comes from the rover or from the base stream: checking it
                        // only for a selected rover ephemeris would let the
                        // base-ephemeris substitution admit a HAS-flagged satellite
                        // (the GPS block below checks it unconditionally too)
                        if (!selected_from_reduced_ced && has_active_has_do_not_use(gal_str, prn, obs_tow))
                            {
                                break;
                            }
                        // Galileo E1
                        if (sig_ == "1B")
                            {
                                if (has_selected_galileo_ephemeris)
                                    {
                                        // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                        eph_data[valid_obs] = selected_from_reduced_ced ? eph_to_rtklib(selected_galileo_ephemeris) : eph_to_rtklib(selected_galileo_ephemeris, this->d_has_orbit_corrections_store_map[gal_str], this->d_has_clock_corrections_store_map[gal_str]);
                                        // convert observation from GNSS-SDR class to RTKLIB structure
                                        obsd_t newobs{};
                                        const HAS_obs_corrections *applied_has_correction = nullptr;
                                        if (selected_from_reduced_ced)
                                            {
                                                d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    selected_galileo_ephemeris.WN,
                                                    d_rtklib_band_index.at(sig_));
                                            }
                                        else
                                            {
                                                d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    d_has_obs_corr_map,
                                                    selected_galileo_ephemeris.WN,
                                                    d_rtklib_band_index.at(sig_),
                                                    &applied_has_correction,
                                                    false);
                                            }
                                        clear_applied_has_phase_bias_discontinuity(applied_has_correction, prn);
                                        valid_obs++;
                                    }
                                // the rover has not decoded this satellite's I/NAV yet: the
                                // base stream may have delivered its broadcast ephemeris over
                                // NTRIP (RTCM MT1045/MT1046), so the satellite contributes
                                // without waiting for the page decoding. E5a joins once the
                                // rover ephemeris is available.
                                else if (!substitute_base_ephemeris(fixed_base, satno(SYS_GAL, prn),
                                             gnss_observables_iter->second, sig_,
                                             eph_data, valid_obs, glo_valid_obs))
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                                    }
                            }

                        // Galileo E5
                        if ((sig_ == "5X") || (sig_ == "7X"))
                            {
                                if (has_selected_galileo_ephemeris)
                                    {
                                        bool found_E1_obs = false;
                                        for (int i = 0; i < valid_obs; i++)
                                            {
                                                if (eph_data[i].sat == (static_cast<int>(gnss_observables_iter->second.PRN + NSATGPS + NSATGLO)))
                                                    {
                                                        const HAS_obs_corrections *applied_has_correction = nullptr;
                                                        if (selected_from_reduced_ced)
                                                            {
                                                                d_obs_data[i + glo_valid_obs] = insert_obs_to_rtklib(d_obs_data[i + glo_valid_obs],
                                                                    gnss_observables_iter->second,
                                                                    selected_galileo_ephemeris.WN,
                                                                    d_rtklib_band_index.at(sig_));
                                                            }
                                                        else
                                                            {
                                                                d_obs_data[i + glo_valid_obs] = insert_obs_to_rtklib(d_obs_data[i + glo_valid_obs],
                                                                    gnss_observables_iter->second,
                                                                    d_has_obs_corr_map,
                                                                    selected_galileo_ephemeris.WN,
                                                                    d_rtklib_band_index.at(sig_),
                                                                    &applied_has_correction,
                                                                    false);
                                                            }
                                                        clear_applied_has_phase_bias_discontinuity(applied_has_correction, prn);
                                                        found_E1_obs = true;
                                                        break;
                                                    }
                                            }
                                        if (!found_E1_obs)
                                            {
                                                // insert Galileo E5 obs as new obs and also insert its ephemeris
                                                // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                                eph_data[valid_obs] = selected_from_reduced_ced ? eph_to_rtklib(selected_galileo_ephemeris) : eph_to_rtklib(selected_galileo_ephemeris, this->d_has_orbit_corrections_store_map[gal_str], this->d_has_clock_corrections_store_map[gal_str]);
                                                // convert observation from GNSS-SDR class to RTKLIB structure
                                                const auto default_code_ = static_cast<unsigned char>(CODE_NONE);
                                                obsd_t newobs = {{0, 0}, '0', '0', {}, {},
                                                    {default_code_, default_code_, default_code_},
                                                    {}, {0.0, 0.0, 0.0}, {}, {}, {}};
                                                const HAS_obs_corrections *applied_has_correction = nullptr;
                                                if (selected_from_reduced_ced)
                                                    {
                                                        d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                            gnss_observables_iter->second,
                                                            selected_galileo_ephemeris.WN,
                                                            d_rtklib_band_index.at(sig_));
                                                    }
                                                else
                                                    {
                                                        d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                            gnss_observables_iter->second,
                                                            d_has_obs_corr_map,
                                                            selected_galileo_ephemeris.WN,
                                                            d_rtklib_band_index.at(sig_),
                                                            &applied_has_correction,
                                                            false);
                                                    }
                                                clear_applied_has_phase_bias_discontinuity(applied_has_correction, prn);
                                                valid_obs++;
                                            }
                                    }
                                else  // the ephemeris are not available for this SV
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->second.PRN;
                                    }
                            }
                        if (sig_ == "E6" && d_conf.use_e6_for_pvt)
                            {
                                if (has_selected_galileo_ephemeris)
                                    {
                                        bool found_E1_obs = false;
                                        for (int i = 0; i < valid_obs; i++)
                                            {
                                                if (eph_data[i].sat == (static_cast<int>(gnss_observables_iter->second.PRN + NSATGPS + NSATGLO)))
                                                    {
                                                        const HAS_obs_corrections *applied_has_correction = nullptr;
                                                        d_obs_data[i + glo_valid_obs] = insert_obs_to_rtklib(d_obs_data[i + glo_valid_obs],
                                                            gnss_observables_iter->second,
                                                            d_has_obs_corr_map,
                                                            selected_galileo_ephemeris.WN,
                                                            d_rtklib_band_index.at(sig_),
                                                            &applied_has_correction,
                                                            false);
                                                        clear_applied_has_phase_bias_discontinuity(applied_has_correction, prn);
                                                        found_E1_obs = true;
                                                        break;
                                                    }
                                            }
                                        if (!found_E1_obs)
                                            {
                                                // insert Galileo E6 obs as new obs and also insert its ephemeris
                                                // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                                eph_data[valid_obs] = eph_to_rtklib(selected_galileo_ephemeris,
                                                    this->d_has_orbit_corrections_store_map[gal_str],
                                                    this->d_has_clock_corrections_store_map[gal_str]);
                                                // convert observation from GNSS-SDR class to RTKLIB structure
                                                const auto default_code_ = static_cast<unsigned char>(CODE_NONE);
                                                obsd_t newobs = {{0, 0}, '0', '0', {}, {},
                                                    {default_code_, default_code_, default_code_},
                                                    {}, {0.0, 0.0, 0.0}, {}, {}, {}};
                                                const HAS_obs_corrections *applied_has_correction = nullptr;
                                                d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    d_has_obs_corr_map,
                                                    selected_galileo_ephemeris.WN,
                                                    d_rtklib_band_index.at(sig_),
                                                    &applied_has_correction,
                                                    false);
                                                clear_applied_has_phase_bias_discontinuity(applied_has_correction, prn);
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
                case 'J':
                    {
                        const bool is_qzss = (gnss_observables_iter->second.PRN >= 193 && gnss_observables_iter->second.PRN <= 206);
                        // GPS/QZSS L1
                        // find the ephemeris for the current SV observation. The SV PRN ID is the map key
                        const std::string gnss_str = is_qzss ? "QZSS" : "GPS";
                        const int sat_sys = is_qzss ? SYS_QZS : SYS_GPS;
                        const int sat = satno(sat_sys, gnss_observables_iter->second.PRN);
                        const int prn = static_cast<int>(gnss_observables_iter->second.PRN);
                        const std::string sig_(gnss_observables_iter->second.Signal, 2);
                        const uint32_t obs_tow = gnss_observables_iter->second.interp_TOW_ms / 1000.0;
                        if (!is_qzss && has_active_has_do_not_use(gnss_str, prn, obs_tow))
                            {
                                break;
                            }
                        const bool is_l1_ca = (sig_ == "1C") || (sig_ == "J1");
                        const bool is_l2 = (sig_ == "2S");
                        const bool is_l5 = (sig_ == "L5") || (sig_ == "J5");
                        const std::string rtklib_sig = is_qzss ? (is_l1_ca ? "J1" : (is_l5 ? "J5" : sig_)) : sig_;
                        if (is_l1_ca)
                            {
                                gps_ephemeris_iter = gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (gps_ephemeris_iter != gps_ephemeris_map.cend())
                                    {
                                        // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                        eph_data[valid_obs] = eph_to_rtklib(gps_ephemeris_iter->second,
                                            this->d_has_orbit_corrections_store_map[gnss_str],
                                            this->d_has_clock_corrections_store_map[gnss_str],
                                            this->get_ref_gps_week());
                                        // convert observation from GNSS-SDR class to RTKLIB structure
                                        obsd_t newobs{};
                                        const HAS_obs_corrections *applied_has_correction = nullptr;
                                        d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                            gnss_observables_iter->second,
                                            d_has_obs_corr_map,
                                            gps_ephemeris_iter->second.WN,
                                            d_rtklib_band_index.at(rtklib_sig),
                                            &applied_has_correction,
                                            this->get_ref_gps_week());
                                        clear_applied_has_phase_bias_discontinuity(applied_has_correction, prn);
                                        valid_obs++;
                                    }
                                // the rover has not decoded this satellite's LNAV yet: the
                                // base stream may have delivered its broadcast ephemeris over
                                // NTRIP (RTCM MT1019, already in RTKLIB format), so the
                                // satellite contributes without waiting the ~30 s of subframe
                                // decoding
                                else if (!substitute_base_ephemeris(fixed_base, sat,
                                             gnss_observables_iter->second, rtklib_sig,
                                             eph_data, valid_obs, glo_valid_obs))
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->first;
                                    }
                            }
                        if (is_l2 || is_l5)
                            {
                                gps_cnav_ephemeris_iter = gps_cnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (gps_cnav_ephemeris_iter != gps_cnav_ephemeris_map.cend())
                                    {
                                        // 1. Find the same satellite in current GPS/QZSS observations (typically L1)
                                        bool found_existing_obs = false;
                                        for (int i = 0; i < valid_obs; i++)
                                            {
                                                if (eph_data[i].sat == sat)
                                                    {
                                                        // 2. If found, attach the L2/L5 observation to the existing observation in RTKLIB structure
                                                        // The existing entry carries the LNAV ephemeris, which has no
                                                        // inter-signal corrections: take the ISCs from CNAV so the
                                                        // dual-frequency correction in prange() can apply them.
                                                        eph_data[i].isc[0] = gps_cnav_ephemeris_iter->second.ISCL1;
                                                        eph_data[i].isc[1] = gps_cnav_ephemeris_iter->second.ISCL2;
                                                        eph_data[i].isc[2] = gps_cnav_ephemeris_iter->second.ISCL5I;
                                                        eph_data[i].isc[3] = gps_cnav_ephemeris_iter->second.ISCL5Q;
                                                        if (eph_data[i].apply_has_corrections)
                                                            {
                                                                const HAS_obs_corrections *applied_has_correction = nullptr;
                                                                d_obs_data[i + glo_valid_obs] = insert_obs_to_rtklib(d_obs_data[i + glo_valid_obs],
                                                                    gnss_observables_iter->second,
                                                                    d_has_obs_corr_map,
                                                                    gps_cnav_ephemeris_iter->second.WN,
                                                                    d_rtklib_band_index.at(rtklib_sig),
                                                                    &applied_has_correction,
                                                                    false);
                                                                clear_applied_has_phase_bias_discontinuity(applied_has_correction, prn);
                                                            }
                                                        else
                                                            {
                                                                d_obs_data[i + glo_valid_obs] = insert_obs_to_rtklib(d_obs_data[i + glo_valid_obs],
                                                                    gnss_observables_iter->second,
                                                                    gps_cnav_ephemeris_iter->second.WN,
                                                                    d_rtklib_band_index.at(rtklib_sig));
                                                            }
                                                        found_existing_obs = true;
                                                        break;
                                                    }
                                            }
                                        if (!found_existing_obs)
                                            {
                                                // 3. If not found, insert the L2/L5 ephemeris and the observation
                                                // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                                eph_data[valid_obs] = eph_to_rtklib(gps_cnav_ephemeris_iter->second);
                                                // convert observation from GNSS-SDR class to RTKLIB structure
                                                const auto default_code_ = static_cast<unsigned char>(CODE_NONE);
                                                obsd_t newobs = {{0, 0}, '0', '0', {}, {},
                                                    {default_code_, default_code_, default_code_},
                                                    {}, {0.0, 0.0, 0.0}, {}, {}, {}};
                                                d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    gps_cnav_ephemeris_iter->second.WN,
                                                    d_rtklib_band_index.at(rtklib_sig));
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
                case 'R':
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
                                        geph_data[glo_valid_obs] = eph_to_rtklib(glonass_gnav_ephemeris_iter->second, gnav_utc, d_conf.glonass_strict_health);
                                        // convert observation from GNSS-SDR class to RTKLIB structure
                                        obsd_t newobs{};
                                        d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                            gnss_observables_iter->second,
                                            glonass_gnav_ephemeris_iter->second.d_WN,
                                            d_rtklib_band_index.at(sig_));
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
                                                            d_rtklib_band_index.at(sig_));
                                                        found_L1_obs = true;
                                                        break;
                                                    }
                                            }
                                        if (!found_L1_obs)
                                            {
                                                // insert GLONASS GNAV L2 obs as new obs and also insert its ephemeris
                                                // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                                geph_data[glo_valid_obs] = eph_to_rtklib(glonass_gnav_ephemeris_iter->second, gnav_utc, d_conf.glonass_strict_health);
                                                // convert observation from GNSS-SDR class to RTKLIB structure
                                                obsd_t newobs{};
                                                d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    glonass_gnav_ephemeris_iter->second.d_WN,
                                                    d_rtklib_band_index.at(sig_));
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
                        // BeiDou B1I / B1C / B3I (prefer B1C when CNAV1 + B1C obs are both present)
                        const std::string sig_(gnss_observables_iter->second.Signal);
                        const int bds_sat = static_cast<int>(gnss_observables_iter->second.PRN + NSATGPS + NSATGLO + NSATGAL + NSATQZS);

                        if (sig_ == "B1")
                            {
                                bool prefer_b1c = false;
                                for (const auto &obs_pair : gnss_observables_map)
                                    {
                                        if (obs_pair.second.System == 'C' &&
                                            obs_pair.second.PRN == gnss_observables_iter->second.PRN &&
                                            std::string(obs_pair.second.Signal, 2) == "1D" &&
                                            beidou_cnav1_ephemeris_map.find(obs_pair.second.PRN) != beidou_cnav1_ephemeris_map.cend())
                                            {
                                                prefer_b1c = true;
                                                break;
                                            }
                                    }
                                if (prefer_b1c)
                                    {
                                        DLOG(INFO) << "Skip B1I for PRN " << gnss_observables_iter->second.PRN
                                                   << " (B1C present; prefer CNAV1)";
                                        break;
                                    }
                                beidou_ephemeris_iter = beidou_dnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (beidou_ephemeris_iter != beidou_dnav_ephemeris_map.cend())
                                    {
                                        eph_data[valid_obs] = eph_to_rtklib(beidou_ephemeris_iter->second);
                                        obsd_t newobs{};
                                        d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                            gnss_observables_iter->second,
                                            beidou_ephemeris_iter->second.WN + BEIDOU_DNAV_BDT2GPST_WEEK_NUM_OFFSET,
                                            d_rtklib_band_index.at(sig_));
                                        valid_obs++;
                                    }
                                else
                                    {
                                        DLOG(INFO) << "No ephemeris data for SV " << gnss_observables_iter->first;
                                    }
                            }
                        if (sig_ == "1D")
                            {
                                const auto cnav1_iter = beidou_cnav1_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (cnav1_iter != beidou_cnav1_ephemeris_map.cend())
                                    {
                                        eph_data[valid_obs] = eph_to_rtklib(cnav1_iter->second);
                                        const auto page_it = beidou_cnav1_page_data_map.find(cnav1_iter->second.PRN);
                                        if (page_it != beidou_cnav1_page_data_map.cend())
                                            {
                                                eph_data[valid_obs].svh = page_it->second.common.hs;
                                            }
                                        obsd_t newobs{};
                                        d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                            gnss_observables_iter->second,
                                            cnav1_iter->second.WN + BEIDOU_DNAV_BDT2GPST_WEEK_NUM_OFFSET,
                                            d_rtklib_band_index.at(sig_));
                                        valid_obs++;
                                    }
                                // Base-stream broadcast ephemeris substitution (RTCM MT1042):
                                // use its DNAV orbit/clock while the rover B-CNAV1 decode
                                // completes. B1C TGD/ISC selection remains CNAV1-only.
                                else if (!substitute_base_ephemeris(fixed_base, bds_sat,
                                             gnss_observables_iter->second, sig_,
                                             eph_data, valid_obs, glo_valid_obs))
                                    {
                                        DLOG(INFO) << "No B-CNAV1 ephemeris data for SV " << gnss_observables_iter->second.PRN;
                                    }
                            }
                        // BeiDou B3: merge with DNAV/B1I only
                        if (sig_ == "B3")
                            {
                                beidou_ephemeris_iter = beidou_dnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (beidou_ephemeris_iter != beidou_dnav_ephemeris_map.cend())
                                    {
                                        bool found_B1I_obs = false;
                                        for (int i = 0; i < valid_obs; i++)
                                            {
                                                if (eph_data[i].sat == bds_sat && eph_data[i].code != BDS_EPH_SOURCE_CNAV1)
                                                    {
                                                        const unsigned char c0 = d_obs_data[i + glo_valid_obs].code[0];
                                                        if (c0 == CODE_L2I || c0 == CODE_L1I)
                                                            {
                                                                d_obs_data[i + glo_valid_obs] = insert_obs_to_rtklib(d_obs_data[i + glo_valid_obs],
                                                                    gnss_observables_iter->second,
                                                                    beidou_ephemeris_iter->second.WN + BEIDOU_DNAV_BDT2GPST_WEEK_NUM_OFFSET,
                                                                    d_rtklib_band_index.at(sig_));
                                                                found_B1I_obs = true;
                                                                break;
                                                            }
                                                    }
                                            }
                                        if (!found_B1I_obs)
                                            {
                                                eph_data[valid_obs] = eph_to_rtklib(beidou_ephemeris_iter->second);
                                                const auto default_code_ = static_cast<unsigned char>(CODE_NONE);
                                                obsd_t newobs = {{0, 0}, '0', '0', {}, {},
                                                    {default_code_, default_code_, default_code_},
                                                    {}, {0.0, 0.0, 0.0}, {}, {}, {}};
                                                d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                                    gnss_observables_iter->second,
                                                    beidou_ephemeris_iter->second.WN + BEIDOU_DNAV_BDT2GPST_WEEK_NUM_OFFSET,
                                                    d_rtklib_band_index.at(sig_));
                                                valid_obs++;
                                            }
                                    }
                                else
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

    int navigation_ephemerides = valid_obs;
    if (d_rtk.opt.sateph == EPHOPT_SBAS)
        {
            for (const auto &previous_ephemeris : gps_previous_ephemeris_map)
                {
                    const bool is_qzss = MINPRNQZS <= previous_ephemeris.first && previous_ephemeris.first <= MAXPRNQZS;
                    const int satellite = satno(is_qzss ? SYS_QZS : SYS_GPS, previous_ephemeris.first);
                    bool satellite_is_observed = false;
                    for (int index = 0; index < valid_obs; ++index)
                        {
                            if (eph_data[index].sat == satellite)
                                {
                                    satellite_is_observed = true;
                                    break;
                                }
                        }
                    if (!satellite_is_observed)
                        {
                            continue;
                        }

                    if (navigation_ephemerides >= static_cast<int>(eph_data.size()))
                        {
                            eph_data.resize(static_cast<size_t>(navigation_ephemerides) + 1);
                        }
                    const std::string system = is_qzss ? "QZSS" : "GPS";
                    eph_data[navigation_ephemerides] = eph_to_rtklib(previous_ephemeris.second,
                        d_has_orbit_corrections_store_map[system],
                        d_has_clock_corrections_store_map[system],
                        this->get_ref_gps_week());
                    ++navigation_ephemerides;
                }
        }

    // **********************************************************************
    // ****** SOLVE PVT******************************************************
    // **********************************************************************

    int rover_observation_count = valid_obs + glo_valid_obs;
    int base_observation_count = 0;
    bool fixed_base_applied = false;
    if (fixed_base != nullptr)
        {
            const bool fixed_base_was_applied = d_fixed_base_was_applied;
            fixed_base_applied = prepare_fixed_base_observations(*fixed_base, rover_observation_count, base_observation_count);
            if (!fixed_base_applied && fixed_base_was_applied)
                {
                    // Do not resume carrier-phase ambiguities after an outage.
                    // The temporary single-point fallback updates solution time
                    // but does not make the prior relative filter state valid.
                    reset_relative_filter();
                }
            d_fixed_base_was_applied = fixed_base_applied;
        }
    const bool use_single_fallback = fixed_base != nullptr && !fixed_base_applied && d_conf.ntrip_fallback_to_single;
    const int nobs_total = rover_observation_count + base_observation_count;

    this->set_valid_position(false);
    if (fixed_base != nullptr && !fixed_base_applied && !d_conf.ntrip_fallback_to_single)
        {
            this->set_num_valid_observations(0);
            return false;
        }
    if (rover_observation_count > 3)
        {
            int result = 0;

            const auto sbas_time_reference = std::find_if(
                gnss_observables_map.cbegin(), gnss_observables_map.cend(),
                [](const std::pair<const int, Gnss_Synchro> &entry) {
                    return entry.second.fs > 0;
                });
            if (sbas_time_reference != gnss_observables_map.cend())
                {
                    int gps_week = 0;
                    const double gps_tow_s = time2gpst(d_obs_data[0].time, &gps_week);
                    const auto &reference_observation = sbas_time_reference->second;
                    const double receiver_sample_stamp_s =
                        (static_cast<double>(reference_observation.Tracking_sample_counter) +
                            reference_observation.Code_phase_samples) /
                        static_cast<double>(reference_observation.fs);
                    d_sbas_corrections.update(gps_week, gps_tow_s, receiver_sample_stamp_s);
                }

            d_nav_data = {};
            d_nav_data.eph = eph_data.data();
            d_nav_data.geph = geph_data.data();
            d_nav_data.n = navigation_ephemerides;
            d_nav_data.ng = glo_valid_obs;
            d_sbas_corrections.copy_to(d_nav_data);
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
            if (!(gps_iono.valid) && gps_cnav_iono.valid)
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
            if (qzss_iono.valid || qzss_cnav_iono.valid)
                {
                    const Gps_Iono &qzss_iono_ref = qzss_iono.valid ? static_cast<const Gps_Iono &>(qzss_iono) : static_cast<const Gps_Iono &>(qzss_cnav_iono);
                    d_nav_data.ion_qzs[0] = qzss_iono_ref.alpha0;
                    d_nav_data.ion_qzs[1] = qzss_iono_ref.alpha1;
                    d_nav_data.ion_qzs[2] = qzss_iono_ref.alpha2;
                    d_nav_data.ion_qzs[3] = qzss_iono_ref.alpha3;
                    d_nav_data.ion_qzs[4] = qzss_iono_ref.beta0;
                    d_nav_data.ion_qzs[5] = qzss_iono_ref.beta1;
                    d_nav_data.ion_qzs[6] = qzss_iono_ref.beta2;
                    d_nav_data.ion_qzs[7] = qzss_iono_ref.beta3;
                    if (!(gps_iono.valid) && !(gps_cnav_iono.valid))
                        {
                            // Keep the GPS-interoperable QZSS Klobuchar coefficients available
                            // to the broadcast ionospheric model in QZSS-only configurations
                            d_nav_data.ion_gps[0] = qzss_iono_ref.alpha0;
                            d_nav_data.ion_gps[1] = qzss_iono_ref.alpha1;
                            d_nav_data.ion_gps[2] = qzss_iono_ref.alpha2;
                            d_nav_data.ion_gps[3] = qzss_iono_ref.alpha3;
                            d_nav_data.ion_gps[4] = qzss_iono_ref.beta0;
                            d_nav_data.ion_gps[5] = qzss_iono_ref.beta1;
                            d_nav_data.ion_gps[6] = qzss_iono_ref.beta2;
                            d_nav_data.ion_gps[7] = qzss_iono_ref.beta3;
                        }
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
                    d_nav_data.ion_cmp[5] = beidou_dnav_iono.beta1;
                    d_nav_data.ion_cmp[6] = beidou_dnav_iono.beta2;
                    d_nav_data.ion_cmp[7] = beidou_dnav_iono.beta3;
                }
            if (beidou_cnav1_iono.valid)
                {
                    d_nav_data.ion_bdgim[0] = beidou_cnav1_iono.alpha1;
                    d_nav_data.ion_bdgim[1] = beidou_cnav1_iono.alpha2;
                    d_nav_data.ion_bdgim[2] = beidou_cnav1_iono.alpha3;
                    d_nav_data.ion_bdgim[3] = beidou_cnav1_iono.alpha4;
                    d_nav_data.ion_bdgim[4] = beidou_cnav1_iono.alpha5;
                    d_nav_data.ion_bdgim[5] = beidou_cnav1_iono.alpha6;
                    d_nav_data.ion_bdgim[6] = beidou_cnav1_iono.alpha7;
                    d_nav_data.ion_bdgim[7] = beidou_cnav1_iono.alpha8;
                    d_nav_data.ion_bdgim[8] = beidou_cnav1_iono.alpha9;
                    d_nav_data.ion_bdgim_valid = 1;
                }
            else
                {
                    d_nav_data.ion_bdgim_valid = 0;
                }
            if (gps_utc_model.valid)
                {
                    d_nav_data.utc_gps[0] = gps_utc_model.A0;
                    d_nav_data.utc_gps[1] = gps_utc_model.A1;
                    d_nav_data.utc_gps[2] = gps_utc_model.tot;
                    d_nav_data.utc_gps[3] = gps_utc_model.WN_T;
                    d_nav_data.leaps = gps_utc_model.DeltaT_LS;
                }
            if (!(gps_utc_model.valid) && gps_cnav_utc_model.valid)
                {
                    d_nav_data.utc_gps[0] = gps_cnav_utc_model.A0;
                    d_nav_data.utc_gps[1] = gps_cnav_utc_model.A1;
                    d_nav_data.utc_gps[2] = gps_cnav_utc_model.tot;
                    d_nav_data.utc_gps[3] = gps_cnav_utc_model.WN_T;
                    d_nav_data.leaps = gps_cnav_utc_model.DeltaT_LS;
                }
            if (qzss_utc_model.valid || qzss_cnav_utc_model.valid)
                {
                    const Gps_Utc_Model &qzss_utc_ref = qzss_utc_model.valid ? static_cast<const Gps_Utc_Model &>(qzss_utc_model) : static_cast<const Gps_Utc_Model &>(qzss_cnav_utc_model);
                    d_nav_data.utc_qzs[0] = qzss_utc_ref.A0;
                    d_nav_data.utc_qzs[1] = qzss_utc_ref.A1;
                    d_nav_data.utc_qzs[2] = qzss_utc_ref.tot;
                    d_nav_data.utc_qzs[3] = qzss_utc_ref.WN_T;
                    if (!(gps_utc_model.valid) && !(gps_cnav_utc_model.valid))
                        {
                            // QZSS time is aligned with GPS time, so its leap seconds apply
                            // in QZSS-only configurations
                            d_nav_data.leaps = qzss_utc_ref.DeltaT_LS;
                        }
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
                    // RTKLIB stores the GPS-UTC leap-second count, whereas the
                    // BeiDou message broadcasts BDT-UTC.
                    d_nav_data.leaps = beidou_dnav_utc_model.DeltaT_LS + BEIDOU_DNAV_BDT2GPST_LEAP_SEC_OFFSET;
                }
            else if (beidou_cnav1_utc_model.valid)
                {
                    d_nav_data.utc_cmp[0] = beidou_cnav1_utc_model.A0;
                    d_nav_data.utc_cmp[1] = beidou_cnav1_utc_model.A1;
                    d_nav_data.utc_cmp[2] = static_cast<double>(beidou_cnav1_utc_model.tot);
                    d_nav_data.utc_cmp[3] = static_cast<double>(beidou_cnav1_utc_model.WN_t);
                    // RTKLIB stores the GPS-UTC leap-second count, whereas the
                    // BeiDou message broadcasts BDT-UTC.
                    d_nav_data.leaps = beidou_cnav1_utc_model.delta_t_LS + BEIDOU_DNAV_BDT2GPST_LEAP_SEC_OFFSET;
                }

            /* update carrier wave length using native function call in RTKlib */
            for (int i = 0; i < MAXSAT; i++)
                {
                    for (int j = 0; j < NFREQ; j++)
                        {
                            d_nav_data.lam[i][j] = satwavelen(i + 1, d_rtklib_freq_index[j], &d_nav_data);
                        }
                }
            for (int i = 0; i < nobs_total; ++i)
                {
                    update_galileo_observation_wavelengths(d_obs_data[i]);
                }

            /* B1C on slot 0: override lam[0] to FREQ1 (satwavelen frq0 is B1I). */
            for (int k = 0; k < nobs_total; k++)
                {
                    if (satsys(d_obs_data[k].sat, nullptr) != SYS_BDS)
                        {
                            continue;
                        }
                    const unsigned char c0 = d_obs_data[k].code[0];
                    if (is_bds_b1c_code(c0))
                        {
                            d_nav_data.lam[d_obs_data[k].sat - 1][0] = SPEED_OF_LIGHT_M_S / FREQ1;
                        }
                }
            const int configured_positioning_mode = d_rtk.opt.mode;
            if (use_single_fallback)
                {
                    d_rtk.opt.mode = PMODE_SINGLE;
                }
            result = rtkpos(&d_rtk, d_obs_data.data(), nobs_total, &d_nav_data);
            d_rtk.opt.mode = configured_positioning_mode;
            if (fixed_base_applied && result != 0 && d_rtk.sol.stat == SOLQ_SINGLE)
                {
                    d_fixed_base_status = Rtklib_Fixed_Base_Status::SOLVER_FALLBACK;
                }
            else if (fixed_base_applied && (result == 0 || d_rtk.sol.stat == SOLQ_NONE))
                {
                    d_fixed_base_status = Rtklib_Fixed_Base_Status::SOLVER_FAILURE;
                }

            if (result == 0 || d_rtk.sol.stat == SOLQ_NONE)
                {
                    if (d_rtk.neb > 0)
                        {
                            LOG(INFO) << "RTKLIB rtkpos error: " << d_rtk.errbuf;
                        }
                    d_rtk.neb = 0;                 // clear error buffer to avoid repeating the error message
                    this->set_time_offset_s(0.0);  // reset rx time estimation
                    this->set_num_valid_observations(0);
                    if (d_conf.enable_pvt_kf == true)
                        {
                            d_pvt_kf.reset_Kf();
                        }
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

                    if (d_conf.enable_pvt_kf == true)
                        {
                            arma::vec p = {pvt_sol.rr[0], pvt_sol.rr[1], pvt_sol.rr[2]};
                            arma::vec v = {pvt_sol.rr[3], pvt_sol.rr[4], pvt_sol.rr[5]};
                            if (d_conf.kf_use_imu_vel)
                                {
                                    v = {
                                        sensor_data_aggregator.get_last_f32(SensorIdentifier::IMU_VEL_X).value,
                                        sensor_data_aggregator.get_last_f32(SensorIdentifier::IMU_VEL_Y).value,
                                        sensor_data_aggregator.get_last_f32(SensorIdentifier::IMU_VEL_Z).value};
                                }

                            if (d_pvt_kf.is_initialized() == false)
                                {
                                    d_pvt_kf.init_Kf(p,
                                        v,
                                        kf_update_interval_s,
                                        d_conf.measures_ecef_pos_sd_m,
                                        d_conf.measures_ecef_vel_sd_ms,
                                        d_conf.system_ecef_pos_sd_m,
                                        d_conf.system_ecef_vel_sd_ms);
                                }
                            else
                                {
                                    d_pvt_kf.run_Kf(p, v);
                                    d_pvt_kf.get_pv_Kf(p, v);
                                    pvt_sol.rr[0] = p[0];  // [m]
                                    pvt_sol.rr[1] = p[1];  // [m]
                                    pvt_sol.rr[2] = p[2];  // [m]
                                    pvt_sol.rr[3] = v[0];  // [ms]
                                    pvt_sol.rr[4] = v[1];  // [ms]
                                    pvt_sol.rr[5] = v[2];  // [ms]
                                }
                        }

                    rx_position_and_time[0] = pvt_sol.rr[0];  // [m]
                    rx_position_and_time[1] = pvt_sol.rr[1];  // [m]
                    rx_position_and_time[2] = pvt_sol.rr[2];  // [m]

                    const bool is_ppp_solution = d_rtk.opt.mode >= PMODE_PPP_KINEMA && pvt_sol.stat == SOLQ_PPP;
                    if (!is_ppp_solution)
                        {
                            // SPP and relative-positioning solutions express dtr in seconds.
                            rx_position_and_time[3] = pvt_sol.dtr[0] + pvt_sol.dtr[2];
                        }
                    else
                        {
                            // PPP stores its estimated receiver-clock state in metres. Inter-system
                            // offsets retained from the point solution remain in seconds.
                            rx_position_and_time[3] = pvt_sol.dtr[2] + pvt_sol.dtr[0] / SPEED_OF_LIGHT_M_S;
                        }
                    this->set_rx_pos({rx_position_and_time[0], rx_position_and_time[1], rx_position_and_time[2]});  // save ECEF position for the next iteration

                    /* RTKLIB stamps SOLQ_PPP whenever the PPP filter runs, even on
                       broadcast orbits and clocks; only claim PPP when precise
                       products are loaded. Demoting here - after the PPP dtr-unit
                       handling above, which must follow the filter's internal
                       state - keeps the console, Monitor, dump and NMEA outputs
                       consistent with each other */
                    if (pvt_sol.stat == SOLQ_PPP && !has_precise_ephemeris())
                        {
                            pvt_sol.stat = SOLQ_SINGLE;
                        }

                    // compute Ground speed and COG
                    double ground_speed_ms = 0.0;
                    std::array<double, 3> pos{};
                    std::array<double, 3> enuv{};
                    ecef2pos(pvt_sol.rr, pos.data());
                    ecef2enu(pos.data(), &pvt_sol.rr[3], enuv.data());
                    this->set_speed_over_ground(norm_rtk(enuv.data(), 2));
                    double new_cog = -9999.0;  // COG not estimated due to insufficient velocity
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
                    // Note: estpos() corrects the solution time only by the receiver-to-GPS clock
                    // state dtr[0], so only that term must be added back here. In solutions without
                    // GPS satellites, the receiver clock offset is absorbed by the inter-system
                    // states (dtr[1..3]) and pvt_sol.time already holds the uncorrected epoch.
                    const double dtr0_s = is_ppp_solution ? pvt_sol.dtr[0] / SPEED_OF_LIGHT_M_S : pvt_sol.dtr[0];
                    const gtime_t rtklib_time = timeadd(pvt_sol.time, dtr0_s);  // uncorrected rx time
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
                    d_monitor_pvt.week = adjgpsweek(d_nav_data.eph[0].week, this->get_ref_gps_week());
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

                    // ENU vel [m/s]
                    d_monitor_pvt.vel_e = enuv[0];
                    d_monitor_pvt.vel_n = enuv[1];
                    d_monitor_pvt.vel_u = enuv[2];

                    // Course Over Ground (cog) [deg]
                    d_monitor_pvt.cog = new_cog;

                    // Galileo HAS status: 1- HAS messages decoded and applied, 0 - HAS not available
                    if (d_has_obs_corr_map.empty())
                        {
                            d_monitor_pvt.galhas_status = 0;
                        }
                    else
                        {
                            d_monitor_pvt.galhas_status = 1;
                        }

                    const double clock_drift_ppm = pvt_sol.dtr[5] / SPEED_OF_LIGHT_M_S * 1e6;

                    this->set_clock_drift_ppm(clock_drift_ppm);
                    // User clock drift [ppm]
                    d_monitor_pvt.user_clk_drift_ppm = clock_drift_ppm;

                    // write UTC time string

                    // Use a facet to display time in a custom format (only hour and minutes).
                    auto *facet = new boost::posix_time::time_facet();
                    facet->format("%Y-%m-%dT%H:%M:%S%F");
                    std::stringstream stream;
                    stream.imbue(std::locale(std::locale::classic(), facet));
                    stream << p_time;
                    stream << 'Z';
                    d_monitor_pvt.utc_time = stream.str();

                    // ######## LOG FILE #########
                    if (d_flag_dump_enabled == true && dump_this_epoch == true)
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
                                    tmp_uint32 = adjgpsweek(d_nav_data.eph[0].week, this->get_ref_gps_week());
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
