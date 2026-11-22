/*!
 * \file rtklib_solver.cc
 * \brief PVT solver based on rtklib library functions adapted to the GNSS-SDR
 *  data flow and structures
 * \authors <ul>
 *          <li> 2017-2019, Javier Arribas
 *          <li> 2017-2026, Carles Fernandez
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
 * Copyright (C) 2017-2026, Carles Fernandez
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * -----------------------------------------------------------------------*/

#include "rtklib_solver.h"
#include "Beidou_CNAV1.h"
#include "Beidou_CNAV2.h"
#include "Beidou_DNAV.h"
#include "Galileo_CNAV.h"
#include "gnss_frequencies.h"
#include "gnss_obs_codes.h"
#include "gnss_sdr_filesystem.h"
#include "matlab_writter_helper.h"
#include "ntrip_rtcm_client.h"
#include "ntrip_rtk_signals.h"
#include "rtklib_pntpos.h"
#include "rtklib_ppp.h"
#include "rtklib_rtkcmn.h"
#include "rtklib_rtkpos.h"
#include "signal_enabled_flags.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
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

namespace
{
struct Rtklib_Pvt_Signal
{
    uint32_t flag;
    char system;
    const char *signal;
};

// Signals the solver can hand to RTKLIB. SBAS L1 ranging is left out, since it
// never provides a second band.
const std::array<Rtklib_Pvt_Signal, 16> RTKLIB_PVT_SIGNALS{{
    {GPS_1C, 'G', "1C"},
    {GPS_L1C, 'G', "1S"},
    {GPS_2S, 'G', "2S"},
    {GPS_L5, 'G', "L5"},
    {GAL_1B, 'E', "1B"},
    {GAL_E5a, 'E', "5X"},
    {GAL_E5b, 'E', "7X"},
    {GAL_E6, 'E', "E6"},
    {GLO_1G, 'R', "1G"},
    {GLO_2G, 'R', "2G"},
    {BDS_B1, 'C', "B1"},
    {BDS_B3, 'C', "B3"},
    {BDS_B1C, 'C', "1D"},
    {BDS_B2A, 'C', "5D"},
    {QZS_J1, 'J', "J1"},
    {QZS_J5, 'J', "J5"},
}};

struct Rtklib_Pvt_System
{
    char system;
    int rtklib_system;
    int prn;  // any valid PRN of the system
    const char *name;
};

const std::array<Rtklib_Pvt_System, 5> RTKLIB_PVT_SYSTEMS{{
    {'G', SYS_GPS, MINPRNGPS, "GPS"},
    {'E', SYS_GAL, MINPRNGAL, "Galileo"},
    {'R', SYS_GLO, MINPRNGLO, "GLONASS"},
    {'C', SYS_BDS, MINPRNBDS, "BeiDou"},
    {'J', SYS_QZS, MINPRNQZS, "QZSS"},
}};

// Galileo_HAS_data::get_gnss_iod() for a satellite without a reference issue of data
const uint16_t HAS_GNSS_IOD_NOT_FOUND = 65535;

// Satellites of a system in a HAS clock full-set or subset block
std::vector<int> has_clock_block_prns(const Galileo_HAS_data &has_data, const std::string &system, bool clock_subset)
{
    if (!clock_subset)
        {
            return has_data.get_PRNs_in_mask(system);
        }
    const auto systems = has_data.get_systems_string();
    const auto system_it = std::find(systems.cbegin(), systems.cend(), system);
    if (system_it == systems.cend())
        {
            return {};
        }
    return has_data.get_PRNs_in_submask(static_cast<uint8_t>(std::distance(systems.cbegin(), system_it)));
}


// Keeps the stored HAS correction of a satellite when the one about to replace
// it belongs to another Mask ID or IOD Set ID
template <typename T>
void keep_superseded_has_set(const std::map<int, T> &store, std::map<int, T> &previous_store, int prn, uint8_t mask_id, uint8_t iod_set_id)
{
    const auto it = store.find(prn);
    if (it != store.cend() && (it->second.mask_id != mask_id || it->second.iod_set_id != iod_set_id))
        {
            previous_store[prn] = it->second;
        }
}


// HAS correction of a satellite for a Mask ID and IOD Set ID, or nullptr. The
// first key of the store is the system or the signal, the second is the PRN
template <typename T>
const T *find_has_set(const std::map<std::string, std::map<int, T>> &store, const std::string &key, int prn, uint8_t mask_id, uint8_t iod_set_id)
{
    const auto key_it = store.find(key);
    if (key_it == store.cend())
        {
            return nullptr;
        }
    const auto prn_it = key_it->second.find(prn);
    if (prn_it == key_it->second.cend() || prn_it->second.mask_id != mask_id || prn_it->second.iod_set_id != iod_set_id)
        {
            return nullptr;
        }
    return &prn_it->second;
}
}  // namespace


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
    d_rtklib_band_index["1S"] = 0;
    d_rtklib_band_index["1B"] = 0;
    d_rtklib_band_index["B1"] = 0;
    d_rtklib_band_index["1D"] = 0;
    d_rtklib_band_index["5D"] = 2;
    d_rtklib_band_index["B3"] = 2;
    d_rtklib_band_index["2G"] = 1;
    d_rtklib_band_index["2S"] = 1;
    d_rtklib_band_index["L5"] = 2;
    d_rtklib_band_index["E6"] = 0;
    d_rtklib_band_index["J1"] = 0;
    d_rtklib_band_index["J5"] = 2;

    const Signal_Enabled_Flags flags(d_signal_enabled_flags);

    // Prefer the ICD clock model for E1/E5b (I/NAV) or E1/E5a (F/NAV).
    // E5b takes priority; E1-only receivers use I/NAV.
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

    // Resolve BeiDou slots independently of the other constellations. B2a
    // owns slot 2 in a frequency pair; B3I uses the remaining slot when a
    // B1 signal is enabled, or the primary slot for B3I+B2a reception.
    if (flags.check_any_enabled(BDS_B2A))
        {
            if (flags.check_any_enabled(BDS_B1, BDS_B1C))
                {
                    d_rtklib_band_index["B3"] = 1;
                }
            else if (flags.check_any_enabled(BDS_B3))
                {
                    d_rtklib_band_index["B3"] = 0;
                }
            else
                {
                    d_rtklib_band_index["5D"] = 0;
                }
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

    if (d_conf.ntrip_client_enabled && !d_conf.ntrip_ssr_corrections)
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

    prcopt_t options = rtk.opt;
    if (options.ionoopt == IONOOPT_IFLC)
        {
            check_iono_free_band_pairs(options);
        }
    load_antenna_calibrations(options);

    // rtk_t owns dynamically allocated filter state. A struct copy would make
    // the adapter, clock solver, and user solver alias the same state and would
    // make independent RTK updates unsafe. Initialize it after all potentially
    // throwing C++ setup so a failed constructor cannot leak the C allocation.
    rtkinit(&d_rtk, &options);
}


namespace
{
char antex_system_letter(int sys)
{
    switch (sys)
        {
        case SYS_GPS:
            return 'G';
        case SYS_GLO:
            return 'R';
        case SYS_GAL:
            return 'E';
        case SYS_BDS:
            return 'C';
        case SYS_QZS:
            return 'J';
        default:
            return '\0';
        }
}


// ANTEX frequency code of an observation code: "E07" for the Galileo "7X"
std::string antex_frequency_code(char system, unsigned char code)
{
    const char *observation = code2obs(code, nullptr);
    if (observation == nullptr || observation[0] == '\0')
        {
            return std::string();
        }
    return std::string(1, system) + '0' + observation[0];
}
}  // namespace


void Rtklib_Solver::load_antenna_calibrations(prcopt_t &options)
{
    options.posopt[0] = 0;  // satellite antenna model
    options.posopt[1] = 0;  // variation of the phase center of the receiver antenna
    if (options.mode != PMODE_PPP_KINEMA && options.mode != PMODE_PPP_STATIC)
        {
            return;
        }
    // Satellite antennas
    if (!d_conf.antex_file.empty())
        {
            if (!d_antex.read(d_conf.antex_file))
                {
                    LOG(WARNING) << "No antenna calibrations read from the ANTEX file " << d_conf.antex_file << ". Its antennas are not modeled.";
                }
            else
                {
                    LOG(INFO) << "Calibrations of " << d_antex.get_number_of_satellite_antennas() << " satellite antennas and "
                              << d_antex.get_number_of_receiver_antennas() << " receiver antennas read from " << d_conf.antex_file;
                    bool satellites_found = d_antex.get_number_of_satellite_antennas() > 0;
                    if (d_antex.has_satellites_by_svn())
                        {
                            // ANTEX 2.0: the satellite of an antenna is given by its SVN
                            if (d_conf.satellite_metadata_file.empty())
                                {
                                    LOG(WARNING) << d_conf.antex_file << " identifies the satellites by their SVN: set PVT.satellite_metadata_file to the IGS satellite metadata file. The satellite antennas are not modeled.";
                                    satellites_found = false;
                                }
                            else if (!d_antex.read_satellite_metadata(d_conf.satellite_metadata_file))
                                {
                                    LOG(WARNING) << "No PRN assignments read from the satellite metadata file " << d_conf.satellite_metadata_file << ". The satellite antennas are not modeled.";
                                    satellites_found = false;
                                }
                        }
                    if (satellites_found)
                        {
                            d_satellite_antennas.assign(MAXSAT, Satellite_Antenna());
                            options.posopt[0] = 1;
                        }
                }
        }

    // Receiver antenna, from its own file if given
    if (!d_conf.antenna_type.empty())
        {
            const std::string &receiver_file = d_conf.receiver_antex_file.empty() ? d_conf.antex_file : d_conf.receiver_antex_file;
            if (receiver_file.empty())
                {
                    LOG(WARNING) << "PVT.antenna_type needs the ANTEX file of PVT.antex_file or PVT.receiver_antex_file. The receiver antenna is not modeled.";
                    return;
                }
            const Antex_Reader *reader = &d_antex;
            if (!d_conf.receiver_antex_file.empty())
                {
                    reader = &d_receiver_antex;
                    if (!d_receiver_antex.read(d_conf.receiver_antex_file))
                        {
                            LOG(WARNING) << "No antenna calibrations read from the ANTEX file " << d_conf.receiver_antex_file << ". The receiver antenna is not modeled.";
                            return;
                        }
                    LOG(INFO) << "Calibrations of " << d_receiver_antex.get_number_of_receiver_antennas() << " receiver antennas read from " << d_conf.receiver_antex_file;
                }
            d_receiver_calibration = reader->find_receiver(d_conf.antenna_type);
            if (d_receiver_calibration == nullptr)
                {
                    LOG(WARNING) << "Receiver antenna " << d_conf.antenna_type << " not found in " << receiver_file << ". It is not modeled.";
                }
            else
                {
                    LOG(INFO) << "Receiver antenna calibration: " << d_receiver_calibration->type;
                    d_receiver_antennas.assign(NSYS, Receiver_Antenna());
                    options.posopt[1] = 1;
                }
        }
}


void Rtklib_Solver::log_antenna_message_once(const std::string &message, bool warning)
{
    if (d_antenna_messages.insert(message).second)
        {
            if (warning)
                {
                    LOG(WARNING) << message;
                }
            else
                {
                    LOG(INFO) << message;
                }
        }
}


bool Rtklib_Solver::satellite_antenna_reference(const Antex_Antenna &calibration, int sys, bool ssr_orbit, std::array<double, 3> &reference) const
{
    // The broadcast orbit, and the Galileo HAS corrections to it, refer to the
    // phase center of the ionosphere-free combination of the signals of the
    // clock model of the navigation message (HAS SIS ICD, Issue 1.0, Section
    // 7.1.2): GPS L1 and L2, Galileo E1 and E5b (I/NAV) or E5a (F/NAV)
    std::string first("G01");
    std::string second("G02");
    double first_frequency_hz = FREQ1;
    double second_frequency_hz = FREQ2;
    if (ssr_orbit && d_conf.ssr_reference_point == 2)
        {
            // center of mass: the origin of the offsets of the calibration
            reference.fill(0.0);
            return true;
        }
    if (ssr_orbit && d_conf.ssr_reference_point == 0)
        {
            // phase center of the primary frequency of the system, L1 or E1
            // (IGS SSR v1.00, Table 2)
            const auto primary = calibration.frequencies.find(sys == SYS_GAL ? "E01" : "G01");
            if (primary == calibration.frequencies.cend())
                {
                    return false;
                }
            reference = primary->second.offset;
            return true;
        }
    if (sys == SYS_GAL)
        {
            first = "E01";
            const bool fnav = d_galileo_nav_message_type_for_pvt == Galileo_Nav_Message_Type::FNAV;
            second = fnav ? "E05" : "E07";
            second_frequency_hz = fnav ? FREQ5 : FREQ7;
        }
    const auto first_calibration = calibration.frequencies.find(first);
    const auto second_calibration = calibration.frequencies.find(second);
    if (first_calibration == calibration.frequencies.cend() || second_calibration == calibration.frequencies.cend())
        {
            return false;
        }
    const double first_coefficient = first_frequency_hz * first_frequency_hz /
                                     (first_frequency_hz * first_frequency_hz - second_frequency_hz * second_frequency_hz);
    for (size_t i = 0; i < 3; i++)
        {
            reference[i] = first_coefficient * first_calibration->second.offset[i] +
                           (1.0 - first_coefficient) * second_calibration->second.offset[i];
        }
    return true;
}


void Rtklib_Solver::update_satellite_antenna(const obsd_t &observation, int sys, int prn)
{
    Satellite_Antenna &antenna = d_satellite_antennas[observation.sat - 1];
    const char system = antex_system_letter(sys);
    const auto time_s = static_cast<int64_t>(observation.time.time);
    char satellite[8];
    std::snprintf(satellite, sizeof(satellite), "%c%02d", system, prn % 100);

    const bool expired = antenna.calibration != nullptr && antenna.calibration->valid_until != 0 &&
                         time_s > antenna.calibration->valid_until;
    if ((antenna.calibration == nullptr || expired) && time_s >= antenna.next_search_s)
        {
            antenna = Satellite_Antenna();
            antenna.code.fill(0xFF);  // set every slot again
            antenna.next_search_s = time_s + 3600;
            antenna.calibration = d_antex.find_satellite(satellite, time_s);
            if (antenna.calibration == nullptr)
                {
                    log_antenna_message_once(std::string("No antenna calibration for satellite ") + satellite + " in the ANTEX file. Its antenna is not modeled.", true);
                }
            else
                {
                    antenna.has_reference = satellite_antenna_reference(*antenna.calibration, sys, false, antenna.reference);
                    if (!antenna.has_reference)
                        {
                            log_antenna_message_once(std::string("The antenna calibration of satellite ") + satellite + " lacks the frequencies of its navigation message. Its antenna is not modeled.", true);
                        }
                }
        }

    // The orbit of a satellite with SSR corrections refers to the point that
    // its provider states, and the broadcast orbit, to the one of the
    // navigation message. The corrections going on or off already count as a
    // cycle slip
    const bool ssr_orbit = d_conf.ntrip_ssr_corrections && has_corrections_applied(sys == SYS_GAL ? "Galileo" : "GPS", prn);
    if (antenna.calibration != nullptr && ssr_orbit != antenna.ssr_orbit)
        {
            antenna.ssr_orbit = ssr_orbit;
            antenna.has_reference = satellite_antenna_reference(*antenna.calibration, sys, ssr_orbit, antenna.reference);
            antenna.code.fill(0xFF);  // set every slot again
            if (!antenna.has_reference)
                {
                    log_antenna_message_once(std::string("The antenna calibration of satellite ") + satellite + " lacks the frequency that its orbit refers to. Its antenna is not modeled.", true);
                }
        }

    for (int slot = 0; slot < NFREQ; slot++)
        {
            const unsigned char code = observation.code[slot];
            if (code == antenna.code[slot])
                {
                    continue;
                }
            antenna.code[slot] = code;
            // A signal with no calibration is left at the reference point of the orbit: no correction
            for (int i = 0; i < 3; i++)
                {
                    antenna.model.off[slot][i] = antenna.reference[i];
                }
            for (double &variation : antenna.model.var[slot])
                {
                    variation = 0.0;
                }
            if (code == CODE_NONE || antenna.calibration == nullptr || !antenna.has_reference)
                {
                    continue;
                }
            const std::string frequency_code = antex_frequency_code(system, code);
            const auto frequency = antenna.calibration->frequencies.find(frequency_code);
            if (frequency == antenna.calibration->frequencies.cend())
                {
                    log_antenna_message_once(std::string("The antenna calibration of satellite ") + satellite + " lacks the frequency " + frequency_code + ". That signal is not corrected.", true);
                    continue;
                }
            for (int i = 0; i < 3; i++)
                {
                    antenna.model.off[slot][i] = frequency->second.offset[i];
                }
            std::copy(frequency->second.variation.cbegin(), frequency->second.variation.cend(), antenna.model.var[slot]);
        }

    d_nav_data.pcvs[observation.sat - 1] = antenna.model;
    for (int i = 0; i < 3; i++)
        {
            d_nav_data.pcoref[observation.sat - 1][i] = antenna.reference[i];
        }
}


void Rtklib_Solver::update_receiver_antenna(const obsd_t &observation, int sys)
{
    Receiver_Antenna &antenna = d_receiver_antennas[ppp_system_index(sys)];
    const char system = antex_system_letter(sys);
    for (int slot = 0; slot < NFREQ; slot++)
        {
            const unsigned char code = observation.code[slot];
            if (code == CODE_NONE || code == antenna.code[slot])
                {
                    continue;
                }
            antenna.code[slot] = code;
            for (double &offset : antenna.model.off[slot])
                {
                    offset = 0.0;
                }
            for (double &variation : antenna.model.var[slot])
                {
                    variation = 0.0;
                }
            // Calibration of the frequency or, as most antennas only have those
            // of GPS, the GPS one of the same band, or else the one of GPS L2
            const std::string frequency_code = antex_frequency_code(system, code);
            if (frequency_code.empty())
                {
                    continue;
                }
            std::string used_code = frequency_code;
            auto frequency = d_receiver_calibration->frequencies.find(used_code);
            if (frequency == d_receiver_calibration->frequencies.cend())
                {
                    used_code = std::string("G0") + frequency_code[2];
                    frequency = d_receiver_calibration->frequencies.find(used_code);
                }
            if (frequency == d_receiver_calibration->frequencies.cend() && frequency_code[2] != '1')
                {
                    used_code = "G02";
                    frequency = d_receiver_calibration->frequencies.find(used_code);
                }
            if (frequency == d_receiver_calibration->frequencies.cend())
                {
                    log_antenna_message_once("The calibration of the receiver antenna lacks the frequency " + frequency_code + ". Only the antenna delta applies to that signal.", true);
                    continue;
                }
            if (used_code != frequency_code)
                {
                    log_antenna_message_once("The calibration of the receiver antenna lacks the frequency " + frequency_code + ": using the one of " + used_code + '.', false);
                }
            // ANTEX: north, east, up. RTKLIB: east, north, up
            antenna.model.off[slot][0] = frequency->second.offset[1];
            antenna.model.off[slot][1] = frequency->second.offset[0];
            antenna.model.off[slot][2] = frequency->second.offset[2];
            std::copy(frequency->second.variation.cbegin(), frequency->second.variation.cend(), antenna.model.var[slot]);
        }
}


void Rtklib_Solver::update_antenna_models(const obsd_t *observations, int number_of_observations)
{
    if (d_satellite_antennas.empty() && d_receiver_calibration == nullptr)
        {
            return;
        }
    for (int i = 0; i < number_of_observations; i++)
        {
            const obsd_t &observation = observations[i];
            if (observation.rcv != 1 || observation.sat < 1 || observation.sat > MAXSAT)
                {
                    continue;
                }
            int prn = 0;
            const int sys = satsys(observation.sat, &prn);
            if (antex_system_letter(sys) == '\0')
                {
                    continue;
                }
            if (d_receiver_calibration != nullptr)
                {
                    update_receiver_antenna(observation, sys);
                }
            // The reference point of the orbit is only defined here for GPS and Galileo
            if (!d_satellite_antennas.empty() && (sys == SYS_GPS || sys == SYS_GAL))
                {
                    update_satellite_antenna(observation, sys, prn);
                }
        }
    for (size_t i = 0; i < d_receiver_antennas.size(); i++)
        {
            d_nav_data.pcvrcv[i] = d_receiver_antennas[i].model;
        }
}


void Rtklib_Solver::check_iono_free_band_pairs(prcopt_t &options) const
{
    // Iono-Free-LC combines, for each satellite, the two bands that
    // prange_band_pair() selects for its system, and leaves out satellites
    // that lack either one. Run that same selection on the slots the enabled
    // signals are mapped to.
    const Signal_Enabled_Flags flags(d_signal_enabled_flags);
    std::string paired_systems;
    std::string unpaired_systems;
    for (const auto &system : RTKLIB_PVT_SYSTEMS)
        {
            obsd_t observation{};
            observation.sat = satno(system.rtklib_system, system.prn);
            bool has_signals = false;
            for (const auto &signal : RTKLIB_PVT_SIGNALS)
                {
                    if (signal.system != system.system || !flags.check_any_enabled(signal.flag) ||
                        (signal.system == 'E' && !is_galileo_signal_used_in_pvt(signal.signal)))
                        {
                            continue;
                        }
                    const auto band = d_rtklib_band_index.find(signal.signal);
                    if (band == d_rtklib_band_index.cend() || band->second < 0 || band->second >= NFREQ)
                        {
                            continue;
                        }
                    observation.code[band->second] = rtklib_obs_code(system.system, signal.signal, band->second);
                    observation.P[band->second] = 1.0;  // bds_observation_slots() skips unmeasured bands
                    has_signals = true;
                }
            if (!has_signals)
                {
                    continue;
                }
            int i = 0;
            int j = 0;
            prange_band_pair(&observation, &i, &j);
            const bool paired = i != j && observation.code[i] != CODE_NONE && observation.code[j] != CODE_NONE;
            std::string &systems = paired ? paired_systems : unpaired_systems;
            systems += (systems.empty() ? "" : ", ") + std::string(system.name);
        }

    if (unpaired_systems.empty())
        {
            return;
        }
    if (paired_systems.empty())
        {
            LOG(WARNING) << "PVT.iono_model=Iono-Free-LC combines two frequency bands of each satellite "
                         << "(for instance, GPS L1 with L2C or L5, or Galileo E1 with E5a, E5b or E6), "
                         << "but the enabled signals do not provide such a pair. Using the Broadcast "
                         << "ionospheric model instead";
            options.ionoopt = IONOOPT_BRDC;
            return;
        }
    LOG(WARNING) << "PVT.iono_model=Iono-Free-LC: the enabled signals do not provide a pair of frequency bands for "
                 << unpaired_systems << " satellites, which are left out of the solution";
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
    const int32_t number_of_uint8_vars = 4;
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
    auto has_corrected_sats = std::vector<uint8_t>(num_epoch);

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
                            dump_file.read(reinterpret_cast<char *>(&has_corrected_sats[i]), sizeof(uint8_t));
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
            write_matlab_var<2>("has_corrected_sats", has_corrected_sats.data(), matfp, dims);
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
    d_has_orbit_corrections_previous_store_map.clear();
    d_has_satellite_do_not_use_intervals.clear();
    d_has_code_bias_store_map.clear();
    d_has_phase_bias_store_map.clear();
    d_has_code_bias_previous_store_map.clear();
    d_has_phase_bias_previous_store_map.clear();
    d_has_phase_discontinuity_indicator_store_map.clear();
    d_has_phase_bias_discontinuity_map.clear();
    d_has_obs_corr_map.clear();
    d_has_orbit_corrections_applied_map.clear();
    d_has_clock_corrections_applied_map.clear();
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
            store_has_orbit_corrections(new_has_data, gps_str);
            store_has_orbit_corrections(new_has_data, gal_str);
        }
    if (new_has_data.header.clock_fullset_flag)
        {
            LOG(INFO) << "Received HAS clock fullset corrections";
            const uint32_t valid_until = tmt +
                                         new_has_data.get_validity_interval_s(new_has_data.validity_interval_index_clock_fullset_corrections);
            update_has_do_not_use(new_has_data, gps_str, false, valid_until);
            update_has_do_not_use(new_has_data, gal_str, false, valid_until);
            store_has_clock_corrections(new_has_data, gps_str, false, valid_until);
            store_has_clock_corrections(new_has_data, gal_str, false, valid_until);
        }
    if (new_has_data.header.clock_subset_flag)
        {
            LOG(INFO) << "Received HAS clock subset corrections";
            const uint32_t valid_until = tmt +
                                         new_has_data.get_validity_interval_s(new_has_data.validity_interval_index_clock_subset_corrections);
            update_has_do_not_use(new_has_data, gps_str, true, valid_until);
            update_has_do_not_use(new_has_data, gal_str, true, valid_until);
            store_has_clock_corrections(new_has_data, gps_str, true, valid_until);
            store_has_clock_corrections(new_has_data, gal_str, true, valid_until);
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
                                    d_has_code_bias_previous_store_map[it].erase(prn);
                                    continue;
                                }
                            keep_superseded_has_set(d_has_code_bias_store_map[it], d_has_code_bias_previous_store_map[it],
                                prn, new_has_data.header.mask_id, new_has_data.header.iod_set_id);
                            auto &code_bias_correction = d_has_code_bias_store_map[it][prn];
                            code_bias_correction.bias = code_bias_m;
                            code_bias_correction.valid_from = tmt;
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
                                    d_has_code_bias_previous_store_map[it].erase(prn);
                                    continue;
                                }
                            keep_superseded_has_set(d_has_code_bias_store_map[it], d_has_code_bias_previous_store_map[it],
                                prn, new_has_data.header.mask_id, new_has_data.header.iod_set_id);
                            auto &code_bias_correction = d_has_code_bias_store_map[it][prn];
                            code_bias_correction.bias = code_bias_m;
                            code_bias_correction.valid_from = tmt;
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
                                    d_has_phase_bias_previous_store_map[it].erase(prn);
                                    d_has_phase_discontinuity_indicator_store_map[it].erase(prn);
                                    d_has_phase_bias_discontinuity_map[it].erase(prn);
                                    continue;
                                }
                            keep_superseded_has_set(d_has_phase_bias_store_map[it], d_has_phase_bias_previous_store_map[it],
                                prn, new_has_data.header.mask_id, new_has_data.header.iod_set_id);
                            auto &phase_bias_correction = d_has_phase_bias_store_map[it][prn];
                            phase_bias_correction.bias = phase_bias_correction_cycles;
                            phase_bias_correction.valid_from = tmt;
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
                                    d_has_phase_bias_previous_store_map[it].erase(prn);
                                    d_has_phase_discontinuity_indicator_store_map[it].erase(prn);
                                    d_has_phase_bias_discontinuity_map[it].erase(prn);
                                    continue;
                                }
                            keep_superseded_has_set(d_has_phase_bias_store_map[it], d_has_phase_bias_previous_store_map[it],
                                prn, new_has_data.header.mask_id, new_has_data.header.iod_set_id);
                            auto &phase_bias_correction = d_has_phase_bias_store_map[it][prn];
                            phase_bias_correction.bias = phase_bias_correction_cycles;
                            phase_bias_correction.valid_from = tmt;
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


void Rtklib_Solver::store_has_orbit_corrections(const Galileo_HAS_data &new_has_data, const std::string &system)
{
    // Stored for every satellite of the mask, whether or not the receiver has
    // the broadcast ephemeris they refer to: it may still be on its way, at
    // the start of the receiver or when HAS follows an update first. They are
    // applied once it is there, see get_active_has_context().
    const uint32_t tmt = new_has_data.get_time_of_message_s();
    const uint32_t valid_until = tmt +
                                 new_has_data.get_validity_interval_s(new_has_data.validity_interval_index_orbit_corrections);
    auto &orbit_store = d_has_orbit_corrections_store_map[system];
    auto &previous_orbit_store = d_has_orbit_corrections_previous_store_map[system];
    for (const int prn : new_has_data.get_PRNs_in_mask(system))
        {
            const uint16_t gnss_iod = new_has_data.get_gnss_iod(system, prn);
            if (gnss_iod == HAS_GNSS_IOD_NOT_FOUND)
                {
                    continue;
                }
            const float radial_m = new_has_data.get_delta_radial_m(system, prn);
            const float in_track_m = new_has_data.get_delta_in_track_m(system, prn);
            const float cross_track_m = new_has_data.get_delta_cross_track_m(system, prn);
            if (Galileo_HAS_data::is_orbit_radial_unavailable(radial_m) ||
                Galileo_HAS_data::is_orbit_along_cross_unavailable(in_track_m) ||
                Galileo_HAS_data::is_orbit_along_cross_unavailable(cross_track_m))
                {
                    orbit_store.erase(prn);
                    previous_orbit_store.erase(prn);
                    d_has_clock_corrections_store_map[system].erase(prn);
                    continue;
                }
            keep_superseded_has_set(orbit_store, previous_orbit_store, prn, new_has_data.header.mask_id, new_has_data.header.iod_set_id);
            auto &orbit_correction = orbit_store[prn];
            orbit_correction.radial_m = radial_m;
            orbit_correction.in_track_m = in_track_m;
            orbit_correction.cross_track_m = cross_track_m;
            orbit_correction.valid_from = tmt;
            orbit_correction.valid_until = valid_until;
            orbit_correction.iod = gnss_iod;
            orbit_correction.mask_id = new_has_data.header.mask_id;
            orbit_correction.iod_set_id = new_has_data.header.iod_set_id;
        }
}


void Rtklib_Solver::store_has_clock_corrections(const Galileo_HAS_data &new_has_data, const std::string &system, bool clock_subset, uint32_t valid_until)
{
    const auto orbit_store_it = d_has_orbit_corrections_store_map.find(system);
    if (orbit_store_it == d_has_orbit_corrections_store_map.cend())
        {
            return;
        }
    const uint32_t tmt = new_has_data.get_time_of_message_s();
    auto &clock_store = d_has_clock_corrections_store_map[system];
    for (const int prn : has_clock_block_prns(new_has_data, system, clock_subset))
        {
            // Clock corrections go with the orbit corrections of their Mask ID
            // and IOD Set ID (HAS SIS ICD 1.0 Section 7.6): the latest received
            // or, until the clock corrections of a new set arrive, the ones
            // they superseded
            const uint16_t gnss_iod = new_has_data.get_gnss_iod(system, prn);
            const HAS_orbit_corrections *orbit_correction = find_has_orbit_correction(system, prn,
                new_has_data.header.mask_id, new_has_data.header.iod_set_id);
            if (orbit_correction == nullptr || orbit_correction->iod != gnss_iod)
                {
                    continue;
                }
            const float clock_correction_mult_m = clock_subset ? new_has_data.get_clock_subset_correction_mult_m(system, prn)
                                                               : new_has_data.get_clock_correction_mult_m(system, prn);
            // do-not-use flags were handled by update_has_do_not_use()
            if (Galileo_HAS_data::is_clock_do_not_use(clock_correction_mult_m) ||
                Galileo_HAS_data::is_clock_unavailable(clock_correction_mult_m))
                {
                    clock_store.erase(prn);
                    continue;
                }
            // A correction for a broadcast ephemeris that the receiver does not
            // have yet does not replace one that it can still apply
            const auto stored_it = clock_store.find(prn);
            if (stored_it != clock_store.end() &&
                !has_reference_ephemeris_stored(system, prn, gnss_iod) &&
                has_reference_ephemeris_stored(system, prn, stored_it->second.iod) &&
                Galileo_HAS_data::has_validity_interval_at_tow(stored_it->second.valid_from, stored_it->second.valid_until, tmt))
                {
                    continue;
                }
            auto &clock_correction = clock_store[prn];
            clock_correction.clock_correction_m = clock_correction_mult_m;
            clock_correction.valid_from = tmt;
            clock_correction.valid_until = valid_until;
            clock_correction.iod = gnss_iod;
            clock_correction.mask_id = new_has_data.header.mask_id;
            clock_correction.iod_set_id = new_has_data.header.iod_set_id;
        }
}


void Rtklib_Solver::update_has_do_not_use(const Galileo_HAS_data &new_has_data, const std::string &system, bool clock_subset, uint32_t valid_until)
{
    // HAS SIS ICD 1.0 Table 31: the satellite shall not be used, whatever
    // the issue of data it refers to, and whether or not the receiver has
    // its ephemeris or orbit corrections yet
    const std::vector<int> prns = has_clock_block_prns(new_has_data, system, clock_subset);

    const uint32_t tmt = new_has_data.get_time_of_message_s();
    for (const int prn : prns)
        {
            const float clock_correction_mult_m = clock_subset ? new_has_data.get_clock_subset_correction_mult_m(system, prn)
                                                               : new_has_data.get_clock_correction_mult_m(system, prn);
            if (Galileo_HAS_data::is_clock_do_not_use(clock_correction_mult_m))
                {
                    d_has_clock_corrections_store_map[system].erase(prn);
                    // The flag holds over this message's validity interval. Earlier
                    // messages keep theirs, for observations that precede this one.
                    auto &intervals = d_has_satellite_do_not_use_intervals[system][prn];
                    const auto same_message = std::find_if(intervals.begin(), intervals.end(),
                        [tmt](const Has_Do_Not_Use_Interval &interval) { return interval.valid_from == tmt; });
                    if (same_message != intervals.end())
                        {
                            same_message->valid_until = std::max(same_message->valid_until, valid_until);
                        }
                    else
                        {
                            Has_Do_Not_Use_Interval new_interval;
                            new_interval.valid_from = tmt;
                            new_interval.valid_until = valid_until;
                            intervals.push_back(new_interval);
                        }
                }
            else if (!Galileo_HAS_data::is_clock_unavailable(clock_correction_mult_m))
                {
                    // A clock correction for the satellite supersedes the flag from
                    // this message's epoch on: earlier epochs stay excluded
                    auto system_it = d_has_satellite_do_not_use_intervals.find(system);
                    if (system_it == d_has_satellite_do_not_use_intervals.end())
                        {
                            continue;
                        }
                    auto prn_it = system_it->second.find(prn);
                    if (prn_it == system_it->second.end())
                        {
                            continue;
                        }
                    auto &intervals = prn_it->second;
                    for (auto interval_it = intervals.begin(); interval_it != intervals.end();)
                        {
                            if (Galileo_HAS_data::has_validity_interval_at_tow(interval_it->valid_from, interval_it->valid_until, tmt))
                                {
                                    const uint32_t elapsed_s = (tmt + GALILEO_HAS_SECONDS_PER_WEEK - interval_it->valid_from) % GALILEO_HAS_SECONDS_PER_WEEK;
                                    if (elapsed_s == 0)
                                        {
                                            interval_it = intervals.erase(interval_it);
                                            continue;
                                        }
                                    interval_it->valid_until = interval_it->valid_from + elapsed_s;
                                }
                            ++interval_it;
                        }
                    if (intervals.empty())
                        {
                            system_it->second.erase(prn_it);
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
    const auto it_sys = d_has_satellite_do_not_use_intervals.find(system);
    if (it_sys == d_has_satellite_do_not_use_intervals.cend())
        {
            return false;
        }
    const auto it_prn = it_sys->second.find(prn);
    if (it_prn == it_sys->second.cend())
        {
            return false;
        }
    return std::any_of(it_prn->second.cbegin(), it_prn->second.cend(), [tow_obs](const Has_Do_Not_Use_Interval &interval) {
        return Galileo_HAS_data::has_validity_interval_at_tow(interval.valid_from, interval.valid_until, tow_obs);
    });
}


void Rtklib_Solver::prune_has_do_not_use_intervals(std::vector<Has_Do_Not_Use_Interval> &intervals, uint32_t tow_obs)
{
    // Observations come in time order: an interval is no longer needed once it
    // has expired, or once another interval in force covers what is left of it
    const auto active = [tow_obs](const Has_Do_Not_Use_Interval &interval) {
        return Galileo_HAS_data::has_validity_interval_at_tow(interval.valid_from, interval.valid_until, tow_obs);
    };
    const auto remaining_s = [tow_obs](const Has_Do_Not_Use_Interval &interval) {
        return (interval.valid_until % GALILEO_HAS_SECONDS_PER_WEEK + GALILEO_HAS_SECONDS_PER_WEEK - tow_obs % GALILEO_HAS_SECONDS_PER_WEEK) % GALILEO_HAS_SECONDS_PER_WEEK;
    };
    std::vector<Has_Do_Not_Use_Interval> kept;
    for (size_t i = 0; i < intervals.size(); i++)
        {
            const Has_Do_Not_Use_Interval &interval = intervals[i];
            if (Galileo_HAS_data::has_validity_interval_expired_at_tow(interval.valid_until, tow_obs))
                {
                    continue;
                }
            bool covered = false;
            if (active(interval))
                {
                    for (size_t j = 0; j < intervals.size() && !covered; j++)
                        {
                            if (j == i || !active(intervals[j]))
                                {
                                    continue;
                                }
                            const uint32_t own = remaining_s(interval);
                            const uint32_t other = remaining_s(intervals[j]);
                            covered = (own < other) || (own == other && j < i);
                        }
                }
            if (!covered)
                {
                    kept.push_back(interval);
                }
        }
    intervals = std::move(kept);
}


bool Rtklib_Solver::get_active_has_context(const std::string &system, int prn, uint32_t tow_obs, uint8_t &mask_id, uint8_t &iod_set_id) const
{
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
    const HAS_clock_corrections &clock_correction = clock_prn_it->second;
    if (!Galileo_HAS_data::has_validity_interval_at_tow(clock_correction.valid_from, clock_correction.valid_until, tow_obs))
        {
            return false;
        }
    // The broadcast ephemeris the corrections refer to: the current one or,
    // until the corrections follow an update, the one it replaced
    if (!has_reference_ephemeris_stored(system, prn, clock_correction.iod))
        {
            return false;
        }

    // The orbit correction of the same Mask ID and IOD Set ID (HAS SIS ICD 1.0
    // Section 7.6.1). After a change of set it is the superseded one, until
    // the clock corrections of the new set arrive.
    const HAS_orbit_corrections *orbit_correction = find_has_orbit_correction(system, prn, clock_correction.mask_id, clock_correction.iod_set_id);
    if (orbit_correction == nullptr)
        {
            return false;
        }
    if (!Galileo_HAS_data::has_validity_interval_at_tow(orbit_correction->valid_from, orbit_correction->valid_until, tow_obs))
        {
            return false;
        }
    if (orbit_correction->iod != clock_correction.iod)
        {
            return false;
        }

    mask_id = clock_correction.mask_id;
    iod_set_id = clock_correction.iod_set_id;
    return true;
}


const HAS_orbit_corrections *Rtklib_Solver::find_has_orbit_correction(const std::string &system, int prn, uint8_t mask_id, uint8_t iod_set_id) const
{
    const HAS_orbit_corrections *orbit_correction = find_has_set(d_has_orbit_corrections_store_map, system, prn, mask_id, iod_set_id);
    if (orbit_correction == nullptr)
        {
            orbit_correction = find_has_set(d_has_orbit_corrections_previous_store_map, system, prn, mask_id, iod_set_id);
        }
    return orbit_correction;
}


bool Rtklib_Solver::has_active_has_orbit_clock(const std::string &system, int prn, uint32_t tow_obs) const
{
    uint8_t mask_id = 0;
    uint8_t iod_set_id = 0;
    return get_active_has_context(system, prn, tow_obs, mask_id, iod_set_id);
}


const Gps_Ephemeris *Rtklib_Solver::find_gps_ephemeris_issue(int prn, uint16_t iode) const
{
    const auto current = gps_ephemeris_map.find(prn);
    if (current != gps_ephemeris_map.cend() && current->second.IODE_SF3 == static_cast<int32_t>(iode))
        {
            return &current->second;
        }
    const auto previous = gps_previous_ephemeris_map.find(prn);
    if (previous != gps_previous_ephemeris_map.cend() && previous->second.IODE_SF3 == static_cast<int32_t>(iode))
        {
            return &previous->second;
        }
    return nullptr;
}


bool Rtklib_Solver::has_reference_ephemeris_stored(const std::string &system, int prn, uint16_t iod) const
{
    if (system == "Galileo")
        {
            return galileo_ephemeris_store.find_inav_issue(prn, static_cast<int32_t>(iod)) != nullptr;
        }
    if (system == "GPS")
        {
            return find_gps_ephemeris_issue(prn, iod) != nullptr;
        }
    return false;
}


void Rtklib_Solver::check_has_orbit_clock_validity(const std::map<int, Gnss_Synchro> &obs_map)
{
    for (const auto &it : obs_map)
        {
            uint32_t obs_tow = it.second.interp_TOW_ms / 1000.0;
            auto prn = static_cast<int>(it.second.PRN);
            std::string system;
            if (it.second.System == 'G')
                {
                    system = "GPS";
                }
            else if (it.second.System == 'E')
                {
                    system = "Galileo";
                }
            else
                {
                    continue;
                }

            // Corrections stay until they expire: whether the receiver has the
            // broadcast ephemeris they refer to is checked when they are applied
            auto orbit_sys_it = d_has_orbit_corrections_store_map.find(system);
            auto clock_sys_it = d_has_clock_corrections_store_map.find(system);
            bool erase_orbit = false;
            bool erase_clock = false;

            if (orbit_sys_it != d_has_orbit_corrections_store_map.end())
                {
                    auto orbit_it = orbit_sys_it->second.find(prn);
                    if (orbit_it != orbit_sys_it->second.end())
                        {
                            if (Galileo_HAS_data::has_validity_interval_expired_at_tow(orbit_it->second.valid_until, obs_tow))
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
                            if (Galileo_HAS_data::has_validity_interval_expired_at_tow(clock_it->second.valid_until, obs_tow))
                                {
                                    erase_clock = true;
                                }
                        }
                }

            // The orbit correction superseded by one of another set goes with
            // the latest one, and when it expires
            auto previous_orbit_sys_it = d_has_orbit_corrections_previous_store_map.find(system);
            if (previous_orbit_sys_it != d_has_orbit_corrections_previous_store_map.end())
                {
                    auto previous_orbit_it = previous_orbit_sys_it->second.find(prn);
                    if (previous_orbit_it != previous_orbit_sys_it->second.end() &&
                        (erase_orbit ||
                            Galileo_HAS_data::has_validity_interval_expired_at_tow(previous_orbit_it->second.valid_until, obs_tow)))
                        {
                            previous_orbit_sys_it->second.erase(previous_orbit_it);
                        }
                }

            if (!erase_orbit && !erase_clock &&
                orbit_sys_it != d_has_orbit_corrections_store_map.end() &&
                clock_sys_it != d_has_clock_corrections_store_map.end())
                {
                    auto orbit_it = orbit_sys_it->second.find(prn);
                    auto clock_it = clock_sys_it->second.find(prn);
                    if (orbit_it != orbit_sys_it->second.end() &&
                        clock_it != clock_sys_it->second.end())
                        {
                            if ((orbit_it->second.mask_id == clock_it->second.mask_id) &&
                                (orbit_it->second.iod_set_id == clock_it->second.iod_set_id))
                                {
                                    // The clock corrections of the latest set arrived: the
                                    // superseded orbit correction is no longer needed
                                    if (previous_orbit_sys_it != d_has_orbit_corrections_previous_store_map.end())
                                        {
                                            previous_orbit_sys_it->second.erase(prn);
                                        }
                                }
                            else if (find_has_orbit_correction(system, prn, clock_it->second.mask_id, clock_it->second.iod_set_id) == nullptr)
                                {
                                    // No orbit correction left for the set of the clock correction
                                    erase_clock = true;
                                }
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

            auto dnu_sys_it = d_has_satellite_do_not_use_intervals.find(system);
            if (dnu_sys_it != d_has_satellite_do_not_use_intervals.end())
                {
                    auto dnu_it = dnu_sys_it->second.find(prn);
                    if (dnu_it != dnu_sys_it->second.end())
                        {
                            prune_has_do_not_use_intervals(dnu_it->second, obs_tow);
                            if (dnu_it->second.empty())
                                {
                                    dnu_sys_it->second.erase(dnu_it);
                                }
                        }
                }
        }
}


std::map<std::pair<char, int>, Rtklib_Solver::Corrected_Satellite_Signals> Rtklib_Solver::corrected_satellite_signals(const std::map<int, Gnss_Synchro> &obs_map) const
{
    // Signals that get_PVT() inserts next to each satellite's corrected
    // ephemeris: the Galileo signals admitted to the PVT solution, and GPS
    // L1 C/A plus the L2C and L5 observations that join its LNAV record when
    // the CNAV ephemeris is available
    std::map<std::pair<char, int>, Corrected_Satellite_Signals> satellites;
    for (const auto &it : obs_map)
        {
            const std::string sig(it.second.Signal, 2);
            const int prn = static_cast<int>(it.second.PRN);
            bool used_in_pvt = false;
            if (it.second.System == 'E')
                {
                    used_in_pvt = is_galileo_signal_used_in_pvt(sig);
                }
            else if (it.second.System == 'G')
                {
                    used_in_pvt = (sig == "1C") ||
                                  ((sig == "2S" || sig == "L5") && gps_cnav_ephemeris_map.find(prn) != gps_cnav_ephemeris_map.cend());
                }
            if (used_in_pvt)
                {
                    auto &satellite = satellites[std::make_pair(it.second.System, prn)];
                    satellite.tow_s = it.second.interp_TOW_ms / 1000.0;
                    satellite.tow = static_cast<uint32_t>(satellite.tow_s);
                    satellite.signals.push_back(sig);
                }
        }
    return satellites;
}


void Rtklib_Solver::erase_obs_corrections(char system_id, const std::vector<std::string> &signals, int prn)
{
    for (const auto &signal : signals)
        {
            for (const auto &has_signal : has_signal_candidates(system_id, signal))
                {
                    auto signal_it = d_has_obs_corr_map.find(has_signal);
                    if (signal_it != d_has_obs_corr_map.end())
                        {
                            signal_it->second.erase(prn);
                            if (signal_it->second.empty())
                                {
                                    d_has_obs_corr_map.erase(signal_it);
                                }
                        }
                }
        }
}


void Rtklib_Solver::update_ssr_corrections(const std::map<int, Gnss_Synchro> &obs_map, const Ntrip_Rtcm_Snapshot &corrections)
{
    // The SSR corrections take the place of the Galileo HAS ones, and reach the
    // broadcast orbit and clock and the observations through the same fields
    d_has_obs_corr_map.clear();
    d_has_orbit_corrections_applied_map.clear();
    d_has_clock_corrections_applied_map.clear();
    if (corrections.ssr_corrections.empty())
        {
            return;
        }

    const auto satellites = corrected_satellite_signals(obs_map);
    const auto &inav_ephemerides = galileo_ephemeris_store.by_source(Galileo_Nav_Message_Type::INAV);
    for (const auto &satellite : satellites)
        {
            const char system_id = satellite.first.first;
            const int prn = satellite.first.second;
            const std::vector<std::string> &signals = satellite.second.signals;
            const std::string system = (system_id == 'E') ? "Galileo" : "GPS";

            // The SSR corrections refer to the Galileo I/NAV and GPS LNAV messages (IGS SSR v1.00, Table 1)
            if (system_id == 'E')
                {
                    if (inav_ephemerides.find(prn) == inav_ephemerides.cend())
                        {
                            continue;
                        }
                }
            else if (gps_ephemeris_map.find(prn) == gps_ephemeris_map.cend() || std::find(signals.cbegin(), signals.cend(), "1C") == signals.cend())
                {
                    continue;
                }

            const int sat = satno(system_id == 'E' ? SYS_GAL : SYS_GPS, prn);
            const auto stored = std::lower_bound(
                corrections.ssr_corrections.cbegin(), corrections.ssr_corrections.cend(), sat,
                [](const std::pair<int, ssr_t> &entry, int satellite_number) { return entry.first < satellite_number; });
            if (sat == 0 || stored == corrections.ssr_corrections.cend() || stored->first != sat)
                {
                    continue;
                }
            const ssr_t &ssr = stored->second;

            // time of the observations, in the week of the corrections
            int week = 0;
            const double ssr_tow = time2gpst(ssr.t0[0], &week);
            double tow = satellite.second.tow_s;
            if (tow < ssr_tow - 302400.0)
                {
                    tow += 604800.0;
                }
            else if (tow > ssr_tow + 302400.0)
                {
                    tow -= 604800.0;
                }
            HAS_orbit_corrections orbit;
            HAS_clock_corrections clock;
            if (!ssr_orbit_and_clock_corrections(ssr, gpst2time(week, tow), d_conf.ntrip_max_correction_age_s, orbit, clock))
                {
                    continue;
                }
            if (system_id == 'E' && ssr.iode_lsb != 0)
                {
                    // IGS SSR gives the eight least significant bits of the IODnav
                    const Galileo_Ephemeris *reference = galileo_ephemeris_store.find_inav_issue_lsb(prn, static_cast<int>(orbit.iod));
                    if (reference == nullptr)
                        {
                            continue;
                        }
                    orbit.iod = static_cast<uint16_t>(reference->IOD_ephemeris);
                    clock.iod = orbit.iod;
                }
            // the corrections are for an issue of the broadcast ephemeris
            if (!has_reference_ephemeris_stored(system, prn, orbit.iod))
                {
                    continue;
                }

            // The SSR clock comes with the code bias of each signal, in place of
            // the broadcast group delays: all the signals of the satellite need theirs
            bool all_code_biases_available = true;
            for (const auto &signal : signals)
                {
                    for (const auto &has_signal : has_signal_candidates(system_id, signal))
                        {
                            const unsigned char code = has_signal_obs_code(system_id, has_signal);
                            if (code != CODE_NONE && ssr.vcbias[code - 1] != 0)
                                {
                                    auto &correction = d_has_obs_corr_map[has_signal][prn];
                                    correction.signal = has_signal;
                                    correction.code_bias_m = ssr.cbias[code - 1];
                                    correction.code_bias_available = true;
                                }
                        }
                    const HAS_obs_corrections *applied = find_has_obs_correction(d_has_obs_corr_map, system_id, signal, prn);
                    if (applied == nullptr || !applied->code_bias_available)
                        {
                            all_code_biases_available = false;
                            DLOG(INFO) << "SSR: no code bias for " << system << " PRN " << prn << " signal " << signal
                                       << ", using the broadcast orbit, clock and group delays for this satellite";
                        }
                }
            if (!all_code_biases_available)
                {
                    erase_obs_corrections(system_id, signals, prn);
                    continue;
                }

            d_has_orbit_corrections_applied_map[system][prn] = orbit;
            d_has_clock_corrections_applied_map[system][prn] = clock;
        }
}


void Rtklib_Solver::get_has_biases(const std::map<int, Gnss_Synchro> &obs_map)
{
    d_has_obs_corr_map.clear();
    d_has_orbit_corrections_applied_map.clear();
    d_has_clock_corrections_applied_map.clear();
    if (d_has_clock_corrections_store_map.empty() || d_has_orbit_corrections_store_map.empty())
        {
            return;
        }

    const auto satellites = corrected_satellite_signals(obs_map);
    const auto &inav_ephemerides = galileo_ephemeris_store.by_source(Galileo_Nav_Message_Type::INAV);
    for (const auto &satellite : satellites)
        {
            const char system_id = satellite.first.first;
            const int prn = satellite.first.second;
            const std::vector<std::string> &signals = satellite.second.signals;
            const uint32_t obs_tow = satellite.second.tow;
            const std::string system = (system_id == 'E') ? "Galileo" : "GPS";

            if (system_id == 'E')
                {
                    if (inav_ephemerides.find(prn) == inav_ephemerides.cend())
                        {
                            continue;
                        }
                }
            else
                {
                    // Only the L1 C/A record carries the HAS-corrected LNAV ephemeris
                    if (gps_ephemeris_map.find(prn) == gps_ephemeris_map.cend() || std::find(signals.cbegin(), signals.cend(), "1C") == signals.cend())
                        {
                            continue;
                        }
                }

            uint8_t mask_id = 0;
            uint8_t iod_set_id = 0;
            if (!get_active_has_context(system, prn, obs_tow, mask_id, iod_set_id))
                {
                    continue;
                }

            bool all_code_biases_available = true;
            for (const auto &signal : signals)
                {
                    for (const auto &has_signal : has_signal_candidates(system_id, signal))
                        {
                            this->get_current_has_obs_correction(has_signal, obs_tow, prn, mask_id, iod_set_id);
                        }
                    // The correction that insert_obs_to_rtklib() will apply must carry a code bias
                    const HAS_obs_corrections *has_correction = find_has_obs_correction(d_has_obs_corr_map, system_id, signal, prn);
                    if (has_correction == nullptr || !has_correction->code_bias_available)
                        {
                            all_code_biases_available = false;
                            DLOG(INFO) << "Galileo HAS: no code bias for " << system << " PRN " << prn << " signal " << signal
                                       << ", using the broadcast orbit, clock and group delays for this satellite";
                        }
                }

            if (!all_code_biases_available)
                {
                    erase_obs_corrections(system_id, signals, prn);
                    continue;
                }

            // The orbit correction of the set selected by get_active_has_context()
            const HAS_orbit_corrections *orbit_correction = find_has_orbit_correction(system, prn, mask_id, iod_set_id);
            if (orbit_correction != nullptr)
                {
                    d_has_orbit_corrections_applied_map[system][prn] = *orbit_correction;
                    d_has_clock_corrections_applied_map[system][prn] = d_has_clock_corrections_store_map[system][prn];
                }
        }
}


bool Rtklib_Solver::uses_precise_orbits_and_clocks(bool precise_products_loaded,
    unsigned int satellites_used,
    unsigned int has_corrected_satellites_used)
{
    // A satellite on broadcast orbit and clock degrades the whole solution
    return precise_products_loaded ||
           (satellites_used > 0 && has_corrected_satellites_used >= satellites_used);
}


bool Rtklib_Solver::has_corrections_applied(const std::string &system, int prn) const
{
    const auto system_it = d_has_orbit_corrections_applied_map.find(system);
    return system_it != d_has_orbit_corrections_applied_map.cend() &&
           system_it->second.find(prn) != system_it->second.cend();
}


bool Rtklib_Solver::select_galileo_has_ephemeris(uint32_t prn, uint32_t observation_tow, Galileo_Ephemeris &ephemeris) const
{
    const auto system_it = d_has_orbit_corrections_applied_map.find("Galileo");
    if (system_it == d_has_orbit_corrections_applied_map.cend())
        {
            return false;
        }
    const auto orbit_it = system_it->second.find(static_cast<int>(prn));
    if (orbit_it == system_it->second.cend())
        {
            return false;
        }
    // the issue of data the corrections refer to: the current ephemeris or,
    // until they follow an update, the one it replaced
    const Galileo_Ephemeris *inav_ephemeris = galileo_ephemeris_store.find_inav_issue(static_cast<int>(prn), static_cast<int32_t>(orbit_it->second.iod));
    if (inav_ephemeris == nullptr ||
        !galileo_ephemeris_is_usable(*inav_ephemeris, observation_tow))
        {
            return false;
        }
    ephemeris = *inav_ephemeris;
    return true;
}


const Gps_Ephemeris *Rtklib_Solver::select_gps_has_ephemeris(int prn, uint32_t observation_tow) const
{
    const auto system_it = d_has_orbit_corrections_applied_map.find("GPS");
    if (system_it == d_has_orbit_corrections_applied_map.cend())
        {
            return nullptr;
        }
    const auto orbit_it = system_it->second.find(prn);
    if (orbit_it == system_it->second.cend())
        {
            return nullptr;
        }
    const Gps_Ephemeris *ephemeris = find_gps_ephemeris_issue(prn, orbit_it->second.iod);
    if (ephemeris == nullptr)
        {
            return nullptr;
        }
    double toe_distance = std::fabs(static_cast<double>(observation_tow) - ephemeris->toe);
    if (toe_distance > 302400.0)
        {
            toe_distance = 604800.0 - toe_distance;
        }
    return toe_distance <= MAXDTOE ? ephemeris : nullptr;
}


void Rtklib_Solver::get_current_has_obs_correction(const std::string &signal, uint32_t tow_obs, int prn, uint8_t mask_id, uint8_t iod_set_id)
{
    // Biases of the requested Mask ID and IOD Set ID: the latest received or,
    // until the clock corrections of a new set arrive, the ones they superseded
    const HAS_bias_corrections *code_bias_correction = find_has_set(d_has_code_bias_store_map, signal, prn, mask_id, iod_set_id);
    if (code_bias_correction == nullptr)
        {
            code_bias_correction = find_has_set(d_has_code_bias_previous_store_map, signal, prn, mask_id, iod_set_id);
        }
    if (code_bias_correction != nullptr &&
        Galileo_HAS_data::has_validity_interval_at_tow(code_bias_correction->valid_from, code_bias_correction->valid_until, tow_obs))
        {
            auto &has_obs_correction = this->d_has_obs_corr_map[signal][prn];
            has_obs_correction.signal = signal;
            has_obs_correction.code_bias_m = code_bias_correction->bias;
            has_obs_correction.code_bias_available = true;
        }

    const HAS_bias_corrections *phase_bias_correction = find_has_set(d_has_phase_bias_store_map, signal, prn, mask_id, iod_set_id);
    const bool latest_phase_bias = phase_bias_correction != nullptr;
    if (!latest_phase_bias)
        {
            phase_bias_correction = find_has_set(d_has_phase_bias_previous_store_map, signal, prn, mask_id, iod_set_id);
        }
    if (phase_bias_correction != nullptr &&
        Galileo_HAS_data::has_validity_interval_at_tow(phase_bias_correction->valid_from, phase_bias_correction->valid_until, tow_obs))
        {
            auto &has_obs_correction = this->d_has_obs_corr_map[signal][prn];
            has_obs_correction.signal = signal;
            has_obs_correction.phase_bias_cycle = phase_bias_correction->bias;
            has_obs_correction.phase_bias_available = true;

            // The stored Phase Discontinuity Indicator and its change belong to
            // the latest phase bias: a change is reported when that bias is applied
            if (latest_phase_bias)
                {
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


obsd_t Rtklib_Solver::insert_obs_with_has_corrections(obsd_t &rtklib_obs,
    const Gnss_Synchro &gnss_synchro,
    int week,
    int band,
    int ref_week,
    bool has_corrected_ephemeris)
{
    const HAS_obs_corrections *applied_has_correction = nullptr;
    obsd_t observation = has_corrected_ephemeris
                             ? insert_obs_to_rtklib(rtklib_obs, gnss_synchro, d_has_obs_corr_map, week, band, &applied_has_correction, ref_week)
                             : insert_obs_to_rtklib(rtklib_obs, gnss_synchro, week, band, ref_week);
    clear_applied_has_phase_bias_discontinuity(applied_has_correction, static_cast<int>(gnss_synchro.PRN));

    // Turning corrections on or off, or selecting another HAS phase-bias
    // signal, steps the carrier phase: restart its ambiguity
    const std::string phase_bias_signal = applied_has_correction != nullptr && applied_has_correction->phase_bias_available
                                              ? applied_has_correction->signal
                                              : std::string{};
    if (d_has_carrier_continuity.changed(observation.sat, std::string(gnss_synchro.Signal, 2), has_corrected_ephemeris, phase_bias_signal))
        {
            observation.LLI[band] |= 1U;
        }
    return observation;
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


bool Rtklib_Solver::is_galileo_observable_valid_outside_pvt(uint32_t prn, const std::string &signal, uint32_t observation_tow) const
{
    if (is_galileo_signal_used_in_pvt(signal))
        {
            return false;
        }
    bool healthy = false;
    return get_galileo_signal_health(prn, signal, observation_tow, healthy) &&
           (d_conf.use_unhealthy_sats || healthy);
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


bool Rtklib_Solver::get_broadcast_signal_health(char system, uint32_t prn, const std::string &signal,
    uint32_t observation_tow, bool &healthy) const
{
    const auto prn_key = static_cast<int>(prn);
    switch (system)
        {
        case 'G':
        case 'J':
            {
                if (signal == "1S")
                    {
                        // L1C: bit 33 of CNAV-2 subframe 2 (0 = signal OK, IS-GPS-800J 3.5.3.4)
                        const auto cnav2_it = gps_cnav2_ephemeris_map.find(prn_key);
                        if (cnav2_it != gps_cnav2_ephemeris_map.cend())
                            {
                                healthy = (cnav2_it->second.l1c_health == 0);
                                return true;
                            }
                        return false;
                    }
                // L2C and L5 health comes from the CNAV message that carries them:
                // bits 52-54 of message type 10 are the L1, L2 and L5 signal health
                // (0 = OK, IS-GPS-200N 30.3.3.1.1.2).
                // Exception: GPS L5 is still pre-operational (IS-GPS-200N 6.3.7), and
                // its CNAV L5 health bit is broadcast as "unhealthy" on every SV
                // regardless of the actual signal status. Reporting that bit would
                // flag every GPS L5 signal as unhealthy, so GPS L5 reports the LNAV
                // SV health of the same satellite (as L1 C/A does) instead. QZSS L5
                // keeps the CNAV bit.
                if (signal == "2S" || signal == "J5")
                    {
                        const auto cnav_it = gps_cnav_ephemeris_map.find(prn_key);
                        if (cnav_it != gps_cnav_ephemeris_map.cend())
                            {
                                const int32_t signal_bit = (signal == "2S") ? 0x2 : 0x1;
                                healthy = ((cnav_it->second.signal_health & signal_bit) == 0);
                                return true;
                            }
                    }
                // L1 C/A and GPS L5: per-satellite SV health from LNAV subframe 1, falling back
                // to the almanac (broadcast by every satellite) when no ephemeris has
                // been decoded for this satellite yet.
                const auto eph_it = gps_ephemeris_map.find(prn_key);
                if (eph_it != gps_ephemeris_map.cend())
                    {
                        healthy = (eph_it->second.SV_health == 0);
                        return true;
                    }
                const auto alm_it = gps_almanac_map.find(prn_key);
                if (alm_it != gps_almanac_map.cend())
                    {
                        healthy = (alm_it->second.SV_health == 0);
                        return true;
                    }
                return false;
            }
        case 'E':
            {
                if (get_galileo_signal_health(prn, signal, observation_tow, healthy))
                    {
                        return true;
                    }
                // No usable ephemeris for this signal's service yet -- fall back
                // to the almanac health status of the same signal. The I/NAV
                // almanac carries E1B_HS and E5b_HS; E5a_HS only comes with the
                // F/NAV almanac. E6 has no broadcast health status.
                const auto alm_it = galileo_almanac_map.find(prn_key);
                if (alm_it == galileo_almanac_map.cend())
                    {
                        return false;
                    }
                if (signal == "1B")
                    {
                        healthy = (alm_it->second.E1B_HS == 0);
                    }
                else if (signal == "7X")
                    {
                        healthy = (alm_it->second.E5b_HS == 0);
                    }
                else if (signal == "5X")
                    {
                        healthy = (alm_it->second.E5a_HS == 0);
                    }
                else
                    {
                        return false;
                    }
                return true;
            }
        case 'R':
            {
                // Same predicate as the ephemeris handed to RTKLIB (see
                // eph_to_rtklib()), so a GLONASS satellite excluded from the solve
                // for health reasons is reported with used = false, healthy = false.
                const auto eph_it = glonass_gnav_ephemeris_map.find(prn_key);
                if (eph_it != glonass_gnav_ephemeris_map.cend())
                    {
                        healthy = glonass_gnav_is_healthy(eph_it->second, d_conf.glonass_strict_health);
                        return true;
                    }
                return false;
            }
        case 'C':
            {
                if (signal == "1D")
                    {
                        // B1C: B-CNAV1 subframe 3 health status (HS, 0 = healthy). The
                        // latest page data is preferred over the ephemeris record, as
                        // when the ephemeris is handed to RTKLIB.
                        const auto page_it = beidou_cnav1_page_data_map.find(prn_key);
                        if (page_it != beidou_cnav1_page_data_map.cend())
                            {
                                healthy = (page_it->second.common.hs == 0);
                                return true;
                            }
                        const auto cnav1_it = beidou_cnav1_ephemeris_map.find(prn_key);
                        if (cnav1_it != beidou_cnav1_ephemeris_map.cend())
                            {
                                healthy = (cnav1_it->second.hs == 0);
                                return true;
                            }
                        return false;
                    }
                if (signal == "5D")
                    {
                        // B2a: B-CNAV2 health status (HS, 0 = healthy)
                        const auto cnav2_it = beidou_cnav2_ephemeris_map.find(prn_key);
                        if (cnav2_it != beidou_cnav2_ephemeris_map.cend())
                            {
                                healthy = (cnav2_it->second.hs == 0);
                                return true;
                            }
                        return false;
                    }
                // B1I / B3I: DNAV satellite health (SatH1, 0 = healthy)
                const auto dnav_it = beidou_dnav_ephemeris_map.find(prn_key);
                if (dnav_it != beidou_dnav_ephemeris_map.cend())
                    {
                        healthy = (dnav_it->second.SV_health == 0);
                        return true;
                    }
                return false;
            }
        default:
            return false;
        }
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

    // Prefer the primary service while its ephemeris is fresh.
    const Galileo_Ephemeris *full_ephemeris = galileo_ephemeris_store.find(
        static_cast<int>(prn), d_galileo_nav_message_type_for_pvt);
    const auto compatibility_ephemeris = galileo_ephemeris_map.find(static_cast<int>(prn));
    if ((full_ephemeris == nullptr || !galileo_ephemeris_is_usable(*full_ephemeris, observation_tow)) &&
        compatibility_ephemeris != galileo_ephemeris_map.cend() &&
        (compatibility_ephemeris->second.nav_message_type == Galileo_Nav_Message_Type::Unknown ||
            compatibility_ephemeris->second.nav_message_type == d_galileo_nav_message_type_for_pvt))
        {
            full_ephemeris = &compatibility_ephemeris->second;
        }
    if (full_ephemeris != nullptr && galileo_ephemeris_is_usable(*full_ephemeris, observation_tow))
        {
            ephemeris = *full_ephemeris;
            if (ephemeris.nav_message_type == Galileo_Nav_Message_Type::Unknown)
                {
                    ephemeris.nav_message_type = d_galileo_nav_message_type_for_pvt;
                }
            return true;
        }

    // E1 can use either clock model with its matching BGD. Other signals
    // must retain the primary service's clock reference.
    if (signal == "1B")
        {
            const auto other_nav_message_type = (d_galileo_nav_message_type_for_pvt == Galileo_Nav_Message_Type::INAV)
                                                    ? Galileo_Nav_Message_Type::FNAV
                                                    : Galileo_Nav_Message_Type::INAV;
            full_ephemeris = galileo_ephemeris_store.find(static_cast<int>(prn), other_nav_message_type);
            if ((full_ephemeris == nullptr || !galileo_ephemeris_is_usable(*full_ephemeris, observation_tow)) &&
                compatibility_ephemeris != galileo_ephemeris_map.cend() &&
                compatibility_ephemeris->second.nav_message_type == other_nav_message_type)
                {
                    full_ephemeris = &compatibility_ephemeris->second;
                }
            if (full_ephemeris != nullptr && galileo_ephemeris_is_usable(*full_ephemeris, observation_tow))
                {
                    ephemeris = *full_ephemeris;
                    return true;
                }
        }

    // The ICD defines Reduced CED only for E1/E5b.
    if (signal != "1B" && signal != "7X")
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


void Rtklib_Solver::update_beidou_observation_wavelengths(const obsd_t &observation)
{
    if (satsys(observation.sat, nullptr) != SYS_BDS)
        {
            return;
        }
    for (int band = 0; band < NFREQ; ++band)
        {
            const unsigned char code = observation.code[band];
            double frequency = 0.0;
            if (is_bds_b1c_code(code))
                {
                    frequency = FREQ1;
                }
            else if (is_bds_b2a_code(code))
                {
                    frequency = FREQ5;
                }
            else if (code == CODE_L6I || code == CODE_L6Q)
                {
                    frequency = FREQ3_BDS;
                }
            else if (code == CODE_L2I || code == CODE_L1I)
                {
                    frequency = FREQ1_BDS;
                }
            if (frequency > 0.0)
                {
                    d_nav_data.lam[observation.sat - 1][band] = SPEED_OF_LIGHT_M_S / frequency;
                }
        }
}


int Rtklib_Solver::merge_duplicated_rover_observations(int rover_observation_count)
{
    /* Two channels can deliver the same satellite on the same signal (e.g. a
       GLONASS false lock labelled with a slot another channel already tracks).
       rescode() only rejects duplicates that are adjacent, and an inconsistent
       duplicate makes every solution fail validation. Keep, per frequency
       slot, the entry with the highest C/N0; preserve the order of the rest. */
    int out = 0;
    bool merged_any = false;
    for (int in = 0; in < rover_observation_count; ++in)
        {
            const obsd_t &source = d_obs_data[in];
            int match = -1;
            for (int k = 0; k < out && source.sat > 0; ++k)
                {
                    if (d_obs_data[k].sat == source.sat)
                        {
                            match = k;
                            break;
                        }
                }
            if (match < 0)
                {
                    if (out != in)
                        {
                            d_obs_data[out] = source;
                        }
                    ++out;
                    continue;
                }
            obsd_t &destination = d_obs_data[match];
            bool overlapping_signal = false;
            for (int frequency = 0; frequency < NFREQ + NEXOBS; ++frequency)
                {
                    const bool source_has_data = source.code[frequency] != CODE_NONE &&
                                                 (source.P[frequency] != 0.0 || source.L[frequency] != 0.0);
                    if (!source_has_data)
                        {
                            continue;
                        }
                    const bool destination_has_data = destination.code[frequency] != CODE_NONE &&
                                                      (destination.P[frequency] != 0.0 || destination.L[frequency] != 0.0);
                    overlapping_signal = overlapping_signal || destination_has_data;
                    if (destination_has_data && destination.SNR[frequency] >= source.SNR[frequency])
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
            merged_any = merged_any || overlapping_signal;
            if (overlapping_signal && !d_duplicated_rover_observations_logged)
                {
                    LOG(WARNING) << "Duplicated observation of satellite " << satno2id(source.sat)
                                 << " received from two channels: keeping the measurement with the highest C/N0";
                    d_duplicated_rover_observations_logged = true;
                }
        }
    if (!merged_any)
        {
            d_duplicated_rover_observations_logged = false;
        }
    return out;
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
    // The base-stream broadcast ephemeris carries no HAS orbit and clock
    // corrections and keeps its group delays, so the observation must not
    // get HAS biases either (HAS SIS ICD 1.0 Section 7.4)
    obsd_t newobs{};
    d_obs_data[valid_obs + glo_valid_obs] = insert_obs_with_has_corrections(newobs,
        gnss_synchro,
        week,
        d_rtklib_band_index.at(band_key),
        (system == SYS_GAL || system == SYS_BDS) ? 0 : this->get_ref_gps_week(),
        false);
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
    d_solution_attempted = false;
    d_has_corrected_satellite.fill(false);
    d_has_corrected_satellites_used = 0;

    // Before merging, a multi-band satellite contributes several channel
    // records and ephemerides. MAXOBS limits satellites, not input channels.
    d_obs_data.assign(std::max(static_cast<size_t>(MAXOBS * 2), gnss_observables_map.size()), obsd_t{});
    std::vector<eph_t> eph_data(std::max(static_cast<size_t>(MAXOBS), gnss_observables_map.size()));
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
                        // HAS corrects the I/NAV orbit and clock, and its code biases make
                        // every signal consistent with the refined clock, so a corrected
                        // satellite uses its I/NAV ephemeris whatever the primary service
                        const bool has_selected_galileo_ephemeris =
                            (is_galileo_signal_used_in_pvt(sig_) &&
                                select_galileo_has_ephemeris(gnss_observables_iter->second.PRN, obs_tow, selected_galileo_ephemeris)) ||
                            select_galileo_ephemeris(gnss_observables_iter->second.PRN, sig_, obs_tow,
                                selected_galileo_ephemeris, selected_from_reduced_ced);
                        // the HAS do-not-use flag excludes the satellite whatever its
                        // ephemeris: rover full or Reduced CED, or from the base stream
                        // (HAS SIS ICD 1.0 Table 31; the GPS block below checks it too)
                        if (has_active_has_do_not_use(gal_str, prn, obs_tow))
                            {
                                break;
                            }
                        // Galileo E1
                        if (sig_ == "1B")
                            {
                                if (has_selected_galileo_ephemeris)
                                    {
                                        // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                        eph_data[valid_obs] = selected_from_reduced_ced ? eph_to_rtklib(selected_galileo_ephemeris) : eph_to_rtklib(selected_galileo_ephemeris, this->d_has_orbit_corrections_applied_map[gal_str], this->d_has_clock_corrections_applied_map[gal_str]);
                                        // convert observation from GNSS-SDR class to RTKLIB structure
                                        // HAS biases go only with a HAS-corrected ephemeris
                                        obsd_t newobs{};
                                        d_obs_data[valid_obs + glo_valid_obs] = insert_obs_with_has_corrections(newobs,
                                            gnss_observables_iter->second,
                                            selected_galileo_ephemeris.WN,
                                            d_rtklib_band_index.at(sig_),
                                            0,
                                            eph_data[valid_obs].apply_has_corrections);
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
                                                        d_obs_data[i + glo_valid_obs] = insert_obs_with_has_corrections(d_obs_data[i + glo_valid_obs],
                                                            gnss_observables_iter->second,
                                                            selected_galileo_ephemeris.WN,
                                                            d_rtklib_band_index.at(sig_),
                                                            0,
                                                            eph_data[i].apply_has_corrections);
                                                        found_E1_obs = true;
                                                        break;
                                                    }
                                            }
                                        if (!found_E1_obs)
                                            {
                                                // insert Galileo E5 obs as new obs and also insert its ephemeris
                                                // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                                eph_data[valid_obs] = selected_from_reduced_ced ? eph_to_rtklib(selected_galileo_ephemeris) : eph_to_rtklib(selected_galileo_ephemeris, this->d_has_orbit_corrections_applied_map[gal_str], this->d_has_clock_corrections_applied_map[gal_str]);
                                                // convert observation from GNSS-SDR class to RTKLIB structure
                                                const auto default_code_ = static_cast<unsigned char>(CODE_NONE);
                                                obsd_t newobs = {{0, 0}, '0', '0', {}, {},
                                                    {default_code_, default_code_, default_code_},
                                                    {}, {0.0, 0.0, 0.0}, {}, {}, {}};
                                                d_obs_data[valid_obs + glo_valid_obs] = insert_obs_with_has_corrections(newobs,
                                                    gnss_observables_iter->second,
                                                    selected_galileo_ephemeris.WN,
                                                    d_rtklib_band_index.at(sig_),
                                                    0,
                                                    eph_data[valid_obs].apply_has_corrections);
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
                                                        d_obs_data[i + glo_valid_obs] = insert_obs_with_has_corrections(d_obs_data[i + glo_valid_obs],
                                                            gnss_observables_iter->second,
                                                            selected_galileo_ephemeris.WN,
                                                            d_rtklib_band_index.at(sig_),
                                                            0,
                                                            eph_data[i].apply_has_corrections);
                                                        found_E1_obs = true;
                                                        break;
                                                    }
                                            }
                                        if (!found_E1_obs)
                                            {
                                                // insert Galileo E6 obs as new obs and also insert its ephemeris
                                                // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                                eph_data[valid_obs] = eph_to_rtklib(selected_galileo_ephemeris,
                                                    this->d_has_orbit_corrections_applied_map[gal_str],
                                                    this->d_has_clock_corrections_applied_map[gal_str]);
                                                // convert observation from GNSS-SDR class to RTKLIB structure
                                                const auto default_code_ = static_cast<unsigned char>(CODE_NONE);
                                                obsd_t newobs = {{0, 0}, '0', '0', {}, {},
                                                    {default_code_, default_code_, default_code_},
                                                    {}, {0.0, 0.0, 0.0}, {}, {}, {}};
                                                d_obs_data[valid_obs + glo_valid_obs] = insert_obs_with_has_corrections(newobs,
                                                    gnss_observables_iter->second,
                                                    selected_galileo_ephemeris.WN,
                                                    d_rtklib_band_index.at(sig_),
                                                    0,
                                                    eph_data[valid_obs].apply_has_corrections);
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
                        const bool is_l1c = (sig_ == "1S");
                        const bool is_l2 = (sig_ == "2S");
                        const bool is_l5 = (sig_ == "L5") || (sig_ == "J5");
                        const std::string rtklib_sig = is_qzss ? (is_l1_ca ? "J1" : (is_l5 ? "J5" : sig_)) : sig_;
                        if (is_l1_ca)
                            {
                                gps_ephemeris_iter = gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (gps_ephemeris_iter != gps_ephemeris_map.cend())
                                    {
                                        // HAS corrections that still refer to the ephemeris replaced
                                        // by the current one are applied to it
                                        const Gps_Ephemeris *has_ephemeris = is_qzss ? nullptr : select_gps_has_ephemeris(prn, obs_tow);
                                        const Gps_Ephemeris &gps_ephemeris = has_ephemeris != nullptr ? *has_ephemeris : gps_ephemeris_iter->second;
                                        // convert ephemeris from GNSS-SDR class to RTKLIB structure
                                        eph_data[valid_obs] = eph_to_rtklib(gps_ephemeris,
                                            this->d_has_orbit_corrections_applied_map[gnss_str],
                                            this->d_has_clock_corrections_applied_map[gnss_str],
                                            this->get_ref_gps_week());
                                        // convert observation from GNSS-SDR class to RTKLIB structure
                                        // HAS biases go only with a HAS-corrected ephemeris
                                        obsd_t newobs{};
                                        d_obs_data[valid_obs + glo_valid_obs] = insert_obs_with_has_corrections(newobs,
                                            gnss_observables_iter->second,
                                            gps_ephemeris.WN,
                                            d_rtklib_band_index.at(rtklib_sig),
                                            this->get_ref_gps_week(),
                                            eph_data[valid_obs].apply_has_corrections);
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
                        if (is_l1c && !is_qzss)
                            {
                                // L1 C/A and L1C share the first RTKLIB slot. Keep L1 C/A
                                // when both are tracked and its LNAV ephemeris is available.
                                bool l1_ca_in_use = false;
                                if (gps_ephemeris_map.find(gnss_observables_iter->second.PRN) != gps_ephemeris_map.cend())
                                    {
                                        for (const auto &obs_pair : gnss_observables_map)
                                            {
                                                if (obs_pair.second.System == 'G' && obs_pair.second.PRN == gnss_observables_iter->second.PRN &&
                                                    std::string(obs_pair.second.Signal, 2) == "1C")
                                                    {
                                                        l1_ca_in_use = true;
                                                        break;
                                                    }
                                            }
                                    }
                                const auto cnav2_iter = gps_cnav2_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (l1_ca_in_use)
                                    {
                                        DLOG(INFO) << "Skip L1C for PRN " << gnss_observables_iter->second.PRN << " (L1 C/A present)";
                                    }
                                else if (cnav2_iter != gps_cnav2_ephemeris_map.cend())
                                    {
                                        bool found_existing_obs = false;
                                        for (int i = 0; i < valid_obs; i++)
                                            {
                                                if (eph_data[i].sat == sat)
                                                    {
                                                        // An L2C or L5 observation came first: L1C fills the first
                                                        // slot, and its ISC replaces the L1 C/A one
                                                        eph_data[i].isc[0] = gnss_observables_iter->second.tracking_component == Gnss_Synchro::TrackingComponent::Data ? cnav2_iter->second.ISCL1CD : cnav2_iter->second.ISCL1CP;
                                                        d_obs_data[i + glo_valid_obs] = insert_obs_with_has_corrections(d_obs_data[i + glo_valid_obs],
                                                            gnss_observables_iter->second,
                                                            cnav2_iter->second.WN,
                                                            d_rtklib_band_index.at(sig_),
                                                            0,
                                                            false);
                                                        found_existing_obs = true;
                                                        break;
                                                    }
                                            }
                                        if (!found_existing_obs)
                                            {
                                                eph_data[valid_obs] = eph_to_rtklib(cnav2_iter->second);
                                                if (gnss_observables_iter->second.tracking_component == Gnss_Synchro::TrackingComponent::Data)
                                                    {
                                                        eph_data[valid_obs].isc[0] = cnav2_iter->second.ISCL1CD;
                                                    }
                                                obsd_t newobs{};
                                                d_obs_data[valid_obs + glo_valid_obs] = insert_obs_with_has_corrections(newobs,
                                                    gnss_observables_iter->second,
                                                    cnav2_iter->second.WN,
                                                    d_rtklib_band_index.at(sig_),
                                                    0,
                                                    false);
                                                valid_obs++;
                                            }
                                    }
                                else
                                    {
                                        DLOG(INFO) << "No CNAV-2 ephemeris data for SV " << gnss_observables_iter->second.PRN;
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
                                                        // dual-frequency correction in prange() can apply them, unless
                                                        // HAS code biases replace the broadcast group delays.
                                                        add_gps_cnav_iscs(eph_data[i], gps_cnav_ephemeris_iter->second, d_obs_data[i + glo_valid_obs].code[0]);
                                                        d_obs_data[i + glo_valid_obs] = insert_obs_with_has_corrections(d_obs_data[i + glo_valid_obs],
                                                            gnss_observables_iter->second,
                                                            gps_cnav_ephemeris_iter->second.WN,
                                                            d_rtklib_band_index.at(rtklib_sig),
                                                            0,
                                                            eph_data[i].apply_has_corrections);
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
                                                d_obs_data[valid_obs + glo_valid_obs] = insert_obs_with_has_corrections(newobs,
                                                    gnss_observables_iter->second,
                                                    gps_cnav_ephemeris_iter->second.WN,
                                                    d_rtklib_band_index.at(rtklib_sig),
                                                    0,
                                                    false);
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
                        if (sig_ == "5D")
                            {
                                const auto cnav2_iter = beidou_cnav2_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                if (cnav2_iter != beidou_cnav2_ephemeris_map.cend() &&
                                    cnav2_iter->second.sig_type == BDS_EPH_SOURCE_CNAV2 &&
                                    cnav2_iter->second.sat_type != 1 &&
                                    cnav2_iter->second.hs == 0)
                                    {
                                        eph_data[valid_obs] = eph_to_rtklib(cnav2_iter->second);
                                        obsd_t newobs{};
                                        d_obs_data[valid_obs + glo_valid_obs] = insert_obs_to_rtklib(newobs,
                                            gnss_observables_iter->second,
                                            cnav2_iter->second.WN + BEIDOU_DNAV_BDT2GPST_WEEK_NUM_OFFSET,
                                            d_rtklib_band_index.at(sig_));
                                        valid_obs++;
                                    }
                                else
                                    {
                                        DLOG(INFO) << "No B-CNAV2 ephemeris data for SV " << gnss_observables_iter->second.PRN;
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
                        d_has_orbit_corrections_applied_map[system],
                        d_has_clock_corrections_applied_map[system],
                        this->get_ref_gps_week());
                    ++navigation_ephemerides;
                }
        }

    // **********************************************************************
    // ****** SOLVE PVT******************************************************
    // **********************************************************************

    int rover_observation_count = merge_duplicated_rover_observations(valid_obs + glo_valid_obs);
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
            d_solution_attempted = true;

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

            for (int k = 0; k < nobs_total; k++)
                {
                    update_beidou_observation_wavelengths(d_obs_data[k]);
                }
            update_antenna_models(d_obs_data.data(), nobs_total);
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

                    // Satellites whose orbit and clock carry HAS corrections. RTKLIB
                    // picks one of the satellite's broadcast records, so all of them
                    // must have the corrections
                    std::array<bool, MAXSAT> without_has_corrections{};
                    for (int i = 0; i < d_nav_data.n; i++)
                        {
                            const int sat = d_nav_data.eph[i].sat;
                            if (sat < 1 || sat > MAXSAT)
                                {
                                    continue;
                                }
                            if (d_nav_data.eph[i].apply_has_corrections)
                                {
                                    d_has_corrected_satellite[sat - 1] = true;
                                }
                            else
                                {
                                    without_has_corrections[sat - 1] = true;
                                }
                        }
                    // Satellites of the reported solution, the ones that pvt_sol.ns
                    // counts. pntpos() flags them in vs. The PPP and relative
                    // filters that follow it, in vsat[0]: they may leave out
                    // satellites of that pre-solution, or take others in.
                    const bool filter_solution = pvt_sol.stat != SOLQ_SINGLE && pvt_sol.stat != SOLQ_SBAS;
                    unsigned int solution_sats = 0;
                    unsigned int has_corrected_sats = 0;
                    for (unsigned int i = 0; i < MAXSAT; i++)
                        {
                            d_has_corrected_satellite[i] = d_has_corrected_satellite[i] && !without_has_corrections[i];
                            const bool in_solution = filter_solution ? d_rtk.ssat[i].vsat[0] != 0 : d_rtk.ssat[i].vs == 1;
                            if (in_solution)
                                {
                                    solution_sats++;
                                    if (d_has_corrected_satellite[i])
                                        {
                                            has_corrected_sats++;
                                        }
                                }
                        }
                    d_has_corrected_satellites_used = static_cast<uint8_t>(std::min(has_corrected_sats, 255U));

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
                       broadcast orbits and clocks; only claim PPP on precise orbits
                       and clocks: loaded products, or HAS corrections for every
                       satellite in the solution. Demoting here - after the PPP
                       dtr-unit handling above, which must follow the filter's
                       internal state - keeps the console, Monitor, dump and NMEA
                       outputs consistent with each other */
                    if (pvt_sol.stat == SOLQ_PPP &&
                        !uses_precise_orbits_and_clocks(has_precise_ephemeris(), solution_sats, d_has_corrected_satellites_used))
                        {
                            pvt_sol.stat = SOLQ_SINGLE;
                        }

                    // compute Ground speed and COG
                    std::array<double, 3> pos{};
                    std::array<double, 3> enuv{};
                    ecef2pos(pvt_sol.rr, pos.data());
                    ecef2enu(pos.data(), &pvt_sol.rr[3], enuv.data());
                    const double ground_speed_ms = norm_rtk(enuv.data(), 2);
                    this->set_speed_over_ground(ground_speed_ms);
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

                    // USED SATELLITES: per-satellite azimuth/elevation and which
                    // signal(s) contributed to this fix. Signals are listed
                    // individually (one entry per satellite per signal); the two
                    // signals of an iono-free pair (e.g. Galileo E1+E5a -- see the
                    // "dual-frequency" branch of prange() in rtklib_pntpos.cc) are
                    // both flagged combined = true, and a third band of the same
                    // satellite is not.
                    //
                    // pntpos() fills ssat[].azel for every observation; only
                    // ssat[].vs is gated by PVT.elevation_mask / RAIM FDE. A
                    // tracked satellite excluded from the solve is therefore
                    // reported with used = false rather than dropped, so the
                    // monitor can tell "excluded" from "not tracked".
                    //
                    // NaN az/el (corrupt ephemeris/almanac) is deliberately
                    // reported as-is for consumers to handle; rescode()
                    // already keeps NaN out of the position solve.
                    //
                    // combined/used below come from d_obs_data's own record, not from configuration.
                    // Bounded to rover_observation_count to skip any base-station entries past it.
                    // Which bands entered the fix depends on the estimator that produced it:
                    // pntpos()/prange() for a single-point fix, relpos() for DGPS/RTK.
                    // pntpos() forces the broadcast iono model when it runs as the pre-pass
                    // of a precise mode, so a single-point fix there never combines bands.
                    const bool spp_solution = d_rtk.opt.mode == PMODE_SINGLE ||
                                              pvt_sol.stat == SOLQ_SINGLE || pvt_sol.stat == SOLQ_SBAS;
                    const bool relative_solution = !spp_solution && d_rtk.opt.mode <= PMODE_FIXED;
                    const int spp_ionoopt = (d_rtk.opt.mode == PMODE_SINGLE || use_single_fallback) ? d_rtk.opt.ionoopt : IONOOPT_BRDC;
                    std::array<int, MAXSAT> sat_to_obs_index{};
                    sat_to_obs_index.fill(-1);
                    for (int obs_idx = 0; obs_idx < rover_observation_count; obs_idx++)
                        {
                            const int sat = d_obs_data[obs_idx].sat;
                            if (sat > 0 && sat <= MAXSAT)
                                {
                                    sat_to_obs_index[sat - 1] = obs_idx;
                                }
                        }
                    d_monitor_pvt.tracked_satellites.clear();
                    for (int sat_idx = 0; sat_idx < MAXSAT; sat_idx++)
                        {
                            const bool has_azel = (pvt_ssat[sat_idx].azel[0] != 0.0) || (pvt_ssat[sat_idx].azel[1] != 0.0);
                            if (!has_azel)
                                {
                                    continue;
                                }
                            const bool used = pvt_ssat[sat_idx].vs != 0;
                            int prn = 0;
                            char sys_char = '?';
                            switch (satsys(sat_idx + 1, &prn))
                                {
                                case SYS_GPS:
                                    sys_char = 'G';
                                    break;
                                case SYS_GAL:
                                    sys_char = 'E';
                                    break;
                                case SYS_GLO:
                                    sys_char = 'R';
                                    break;
                                case SYS_BDS:
                                    sys_char = 'C';
                                    break;
                                case SYS_SBS:
                                    sys_char = 'S';
                                    break;
                                case SYS_QZS:
                                    sys_char = 'J';
                                    break;
                                default:
                                    break;
                                }

                            std::vector<const Gnss_Synchro *> contributing_signals;
                            for (const auto &observable_pair : gnss_observables_map)
                                {
                                    const Gnss_Synchro &synchro = observable_pair.second;
                                    if (synchro.System == sys_char && static_cast<int>(synchro.PRN) == prn)
                                        {
                                            contributing_signals.push_back(&synchro);
                                        }
                                }
                            // Band pair prange() itself reads for this satellite this epoch.
                            int prange_i = 0;
                            int prange_j = 1;
                            bool both_bands_present = false;
                            int picked_slot = -1;
                            const int obs_idx = sat_to_obs_index[sat_idx];
                            const obsd_t *obs_record = obs_idx >= 0 ? &d_obs_data[obs_idx] : nullptr;
                            if (obs_record != nullptr)
                                {
                                    prange_band_pair(obs_record, &prange_i, &prange_j);
                                    both_bands_present = prange_i != prange_j &&
                                                         obs_record->P[prange_i] != 0.0 && obs_record->P[prange_j] != 0.0;
                                    picked_slot = obs_record->code[prange_i] != CODE_NONE ? prange_i : prange_j;
                                }
                            // satazel() (rtklib_rtkcmn.cc) returns azimuth in [0, 2*pi); wrap to
                            // (-180, 180] deg, the convention expected downstream (monitor sky plot).
                            double az_deg = pvt_ssat[sat_idx].azel[0] * R2D;
                            if (az_deg > 180.0)
                                {
                                    az_deg -= 360.0;
                                }
                            for (const Gnss_Synchro *synchro : contributing_signals)
                                {
                                    Monitor_Pvt::TrackedSatelliteInfo info;
                                    info.prn = static_cast<uint32_t>(prn);
                                    info.system = sys_char;
                                    info.signal = std::string(synchro->Signal, 2);
                                    info.observation_code = synchro->observation_code();
                                    info.azimuth_deg = az_deg;
                                    info.elevation_deg = pvt_ssat[sat_idx].azel[1] * R2D;
                                    const auto band_it = d_rtklib_band_index.find(info.signal);
                                    const int signal_slot = band_it != d_rtklib_band_index.cend() ? band_it->second : -1;
                                    // The slot must hold this signal's own code: a signal left out of the
                                    // solver input (B1I next to B1C, the non-selected E5, E6 per
                                    // PVT.use_e6_for_pvt) or dropped by the fixed-base path fails this.
                                    const bool in_record = obs_record != nullptr && signal_slot >= 0 && signal_slot < NFREQ &&
                                                           obs_record->code[signal_slot] != CODE_NONE &&
                                                           obs_record->code[signal_slot] == obs2code(synchro->observation_code(), nullptr);
                                    info.combined = false;
                                    info.used = false;
                                    if (in_record && spp_solution)
                                        {
                                            if (spp_ionoopt == IONOOPT_IFLC)
                                                {
                                                    info.combined = used && both_bands_present &&
                                                                    (signal_slot == prange_i || signal_slot == prange_j);
                                                    info.used = info.combined;
                                                }
                                            else
                                                {
                                                    info.used = used && signal_slot == picked_slot;
                                                }
                                        }
                                    else if (in_record && relative_solution)
                                        {
                                            // relpos() flags each band it differenced in vsat[]; with
                                            // Iono-Free-LC it differences one L1/L2 combination, flagged in vsat[0].
                                            if (d_rtk.opt.ionoopt == IONOOPT_IFLC)
                                                {
                                                    info.combined = pvt_ssat[sat_idx].vsat[0] != 0 && signal_slot <= 1;
                                                    info.used = info.combined;
                                                }
                                            else
                                                {
                                                    info.used = pvt_ssat[sat_idx].vsat[signal_slot] != 0;
                                                }
                                        }
                                    else if (in_record)
                                        {
                                            // PPP: its band pairing (ifmeas()) is not modelled here.
                                            info.used = used;
                                        }
                                    // Broadcast health of this signal, as reported by the
                                    // navigation message that carries it (true when no health
                                    // information is available). Independent of `used`.
                                    const auto observation_tow = static_cast<uint32_t>(synchro->interp_TOW_ms / 1000.0);
                                    bool healthy = true;
                                    if (get_broadcast_signal_health(sys_char, static_cast<uint32_t>(prn), info.signal, observation_tow, healthy))
                                        {
                                            info.healthy = healthy;
                                        }
                                    // HAS orbit and clock corrections for the satellite, and
                                    // the HAS code bias for this signal
                                    info.has_corrected = in_record && d_has_corrected_satellite[sat_idx];
                                    d_monitor_pvt.tracked_satellites.push_back(std::move(info));
                                }
                        }

                    this->set_rx_vel({enuv[0], enuv[1], enuv[2]});

                    // ENU vel [m/s]
                    d_monitor_pvt.vel_e = enuv[0];
                    d_monitor_pvt.vel_n = enuv[1];
                    d_monitor_pvt.vel_u = enuv[2];

                    // Course Over Ground (cog) [deg]
                    d_monitor_pvt.cog = new_cog;

                    // Galileo HAS status: 1 - HAS corrections applied to at least one
                    // satellite of the solution, 0 - not applied
                    d_monitor_pvt.galhas_status = d_has_corrected_satellites_used > 0 ? 1 : 0;

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
                                    // NUMBER OF SATELLITES WITH HAS ORBIT AND CLOCK CORRECTIONS
                                    d_dump_file.write(reinterpret_cast<char *>(&d_has_corrected_satellites_used), sizeof(uint8_t));
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
