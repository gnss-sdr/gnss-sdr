/*!
 * \file rtklib_fixed_base_test.cc
 * \brief Unit tests for the RTKLIB fixed-base observation seam
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * SPDX-FileCopyrightText: 2026 Carles Fernandez-Prades <carles.fernandez@cttc.es>
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "galileo_ephemeris.h"
#include "gps_cnav_ephemeris.h"
#include "in_memory_configuration.h"
#include "ntrip_rtcm_client.h"
#include "pvt_conf.h"
#include "rtklib_conversions.h"
#include "rtklib_ephemeris.h"
#include "rtklib_rtkcmn.h"
#include "rtklib_rtksvr.h"
#include "rtklib_solver.h"
#include "sensor_data/sensor_data_aggregator.h"
#include "sensor_data/sensor_data_source_configuration.h"
#include "signal_flag.h"
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

namespace rtklib_fixed_base_test_detail
{
constexpr int TEST_GPS_WEEK = 2300;
constexpr double TEST_TOW_S = 100000.0;
constexpr double TEST_MAX_AGE_S = 2.0;


struct Synthetic_Relative_Epoch
{
    std::map<int, Gnss_Synchro> rover_observations;
    Ntrip_Rtcm_Snapshot base_snapshot;
};


Gnss_Synchro make_rover_observation(unsigned int prn);


prcopt_t fixed_base_options()
{
    prcopt_t options = PRCOPT_DEFAULT;
    options.mode = PMODE_STATIC;
    options.nf = 2;
    options.navsys = SYS_GPS;
    options.elmin = -90.0 * D2R;
    options.ionoopt = IONOOPT_OFF;
    options.tropopt = TROPOPT_OFF;
    options.refpos = 0;
    options.outsingle = 1;
    options.maxgdop = 1000.0;
    options.maxinno[0] = options.maxinno[1] = 100.0;
    options.niter = 3;
    return options;
}


Gps_Ephemeris make_ephemeris(unsigned int prn)
{
    Gps_Ephemeris ephemeris;
    ephemeris.PRN = prn;
    ephemeris.WN = TEST_GPS_WEEK;
    ephemeris.toe = static_cast<int32_t>(TEST_TOW_S);
    ephemeris.toc = static_cast<int32_t>(TEST_TOW_S);
    ephemeris.tow = static_cast<int32_t>(TEST_TOW_S);
    ephemeris.sqrtA = 5153.7954775;
    ephemeris.ecc = 0.01;
    ephemeris.i_0 = 0.94;
    ephemeris.M_0 = 0.5 * static_cast<double>(prn);
    ephemeris.OMEGA_0 = 0.8 * static_cast<double>(prn);
    ephemeris.omega = 0.2 * static_cast<double>(prn);
    ephemeris.OMEGAdot = -8.0e-9;
    ephemeris.IODE_SF2 = static_cast<int32_t>(prn);
    ephemeris.IODE_SF3 = static_cast<int32_t>(prn);
    ephemeris.IODC = static_cast<int32_t>(prn);
    ephemeris.SV_health = 0;
    return ephemeris;
}


Gps_Ephemeris make_relative_ephemeris(unsigned int prn)
{
    Gps_Ephemeris ephemeris = make_ephemeris(prn);
    const unsigned int plane = (prn - 1U) / 8U;
    const unsigned int slot = (prn - 1U) % 8U;
    ephemeris.ecc = 0.0;
    ephemeris.M_0 = 2.0 * GNSS_PI * static_cast<double>(slot) / 8.0;
    ephemeris.OMEGA_0 = 2.0 * GNSS_PI * static_cast<double>(plane) / 4.0;
    ephemeris.omega = 0.0;
    return ephemeris;
}


Galileo_Ephemeris make_relative_galileo_ephemeris(unsigned int prn)
{
    Galileo_Ephemeris ephemeris;
    ephemeris.PRN = prn;
    // eph_to_rtklib() adds 1024 to the GST week number
    ephemeris.WN = TEST_GPS_WEEK - 1024;
    ephemeris.toe = static_cast<int32_t>(TEST_TOW_S);
    ephemeris.toc = static_cast<int32_t>(TEST_TOW_S);
    ephemeris.tow = static_cast<int32_t>(TEST_TOW_S);
    ephemeris.sqrtA = 5440.588;  // Galileo semi-major axis (29600 km)
    ephemeris.ecc = 0.0;
    ephemeris.i_0 = 0.9774;  // 56 deg
    const unsigned int plane = (prn - 1U) / 9U;
    const unsigned int slot = (prn - 1U) % 9U;
    // Offset the planes and slots so the constellation does not overlap the
    // synthetic GPS satellites in the sky
    ephemeris.M_0 = 2.0 * GNSS_PI * (static_cast<double>(slot) + 0.5) / 9.0;
    ephemeris.OMEGA_0 = 2.0 * GNSS_PI * static_cast<double>(plane) / 3.0 + 0.35;
    ephemeris.omega = 0.0;
    ephemeris.OMEGAdot = -5.3e-9;
    ephemeris.IOD_ephemeris = static_cast<int32_t>(prn);
    ephemeris.nav_message_type = Galileo_Nav_Message_Type::FNAV;
    return ephemeris;
}


Gps_CNAV_Ephemeris make_cnav_ephemeris(const Gps_Ephemeris& lnav)
{
    Gps_CNAV_Ephemeris cnav;
    cnav.PRN = lnav.PRN;
    cnav.WN = lnav.WN;
    cnav.toe = lnav.toe;
    cnav.toe1 = lnav.toe;
    cnav.toe2 = lnav.toe;
    cnav.toc = lnav.toc;
    cnav.tow = lnav.tow;
    cnav.sqrtA = lnav.sqrtA;
    cnav.ecc = lnav.ecc;
    cnav.M_0 = lnav.M_0;
    cnav.delta_n = lnav.delta_n;
    cnav.OMEGA_0 = lnav.OMEGA_0;
    cnav.i_0 = lnav.i_0;
    cnav.omega = lnav.omega;
    cnav.OMEGAdot = lnav.OMEGAdot;
    cnav.idot = lnav.idot;
    cnav.Cuc = lnav.Cuc;
    cnav.Cus = lnav.Cus;
    cnav.Crc = lnav.Crc;
    cnav.Crs = lnav.Crs;
    cnav.Cic = lnav.Cic;
    cnav.Cis = lnav.Cis;
    cnav.af0 = lnav.af0;
    cnav.af1 = lnav.af1;
    cnav.af2 = lnav.af2;
    cnav.URAED = 0;
    cnav.URANED0 = 0;
    cnav.signal_health = 0;
    return cnav;
}


double modeled_signal_range(const eph_t& rtklib_ephemeris,
    const gtime_t& reception_time,
    const double* receiver_position_ecef,
    double receiver_clock_m,
    double* elevation_rad)
{
    double pseudorange_m = 22000000.0 + receiver_clock_m;
    double modeled_range_m = 0.0;
    for (int iteration = 0; iteration < 8; ++iteration)
        {
            const gtime_t transmission_time = timeadd(
                reception_time, -pseudorange_m / SPEED_OF_LIGHT_M_S);
            double satellite_position_ecef[3]{};
            double satellite_clock_s = 0.0;
            double variance = 0.0;
            eph2pos(transmission_time,
                &rtklib_ephemeris,
                satellite_position_ecef,
                &satellite_clock_s,
                &variance);

            double line_of_sight[3]{};
            const double geometric_range_m = geodist(
                satellite_position_ecef, receiver_position_ecef, line_of_sight);
            double receiver_position_geodetic[3]{};
            double azimuth_elevation[2]{};
            ecef2pos(receiver_position_ecef, receiver_position_geodetic);
            satazel(receiver_position_geodetic, line_of_sight, azimuth_elevation);
            const double hydrostatic_troposphere_m = tropmodel(
                reception_time, receiver_position_geodetic, azimuth_elevation, 0.0);
            modeled_range_m = geometric_range_m - SPEED_OF_LIGHT_M_S * satellite_clock_s + hydrostatic_troposphere_m;
            pseudorange_m = modeled_range_m + receiver_clock_m;
            if (elevation_rad != nullptr)
                {
                    *elevation_rad = azimuth_elevation[1];
                }
        }
    return modeled_range_m;
}


double modeled_signal_range(const Gps_Ephemeris& ephemeris,
    const gtime_t& reception_time,
    const double* receiver_position_ecef,
    double receiver_clock_m,
    double* elevation_rad)
{
    return modeled_signal_range(eph_to_rtklib(ephemeris, TEST_GPS_WEEK),
        reception_time, receiver_position_ecef, receiver_clock_m, elevation_rad);
}


double modeled_signal_range(const Galileo_Ephemeris& ephemeris,
    const gtime_t& reception_time,
    const double* receiver_position_ecef,
    double receiver_clock_m,
    double* elevation_rad)
{
    return modeled_signal_range(eph_to_rtklib(ephemeris),
        reception_time, receiver_position_ecef, receiver_clock_m, elevation_rad);
}


std::vector<unsigned int> select_relative_satellites(const double* base_position_ecef,
    const double* rover_position_ecef)
{
    const gtime_t reception_time = gpst2time(TEST_GPS_WEEK, TEST_TOW_S);
    std::vector<unsigned int> satellites;
    for (unsigned int prn = 1; prn <= 32; ++prn)
        {
            const Gps_Ephemeris ephemeris = make_relative_ephemeris(prn);
            double base_elevation_rad = 0.0;
            double rover_elevation_rad = 0.0;
            modeled_signal_range(ephemeris, reception_time, base_position_ecef, 0.0, &base_elevation_rad);
            modeled_signal_range(ephemeris, reception_time, rover_position_ecef, 0.0, &rover_elevation_rad);
            if (base_elevation_rad > 15.0 * D2R && rover_elevation_rad > 15.0 * D2R)
                {
                    satellites.push_back(prn);
                }
        }
    return satellites;
}


std::vector<unsigned int> select_relative_galileo_satellites(const double* base_position_ecef,
    const double* rover_position_ecef)
{
    const gtime_t reception_time = gpst2time(TEST_GPS_WEEK, TEST_TOW_S);
    std::vector<unsigned int> satellites;
    for (unsigned int prn = 1; prn <= 27; ++prn)
        {
            const Galileo_Ephemeris ephemeris = make_relative_galileo_ephemeris(prn);
            double base_elevation_rad = 0.0;
            double rover_elevation_rad = 0.0;
            modeled_signal_range(ephemeris, reception_time, base_position_ecef, 0.0, &base_elevation_rad);
            modeled_signal_range(ephemeris, reception_time, rover_position_ecef, 0.0, &rover_elevation_rad);
            if (base_elevation_rad > 15.0 * D2R && rover_elevation_rad > 15.0 * D2R)
                {
                    satellites.push_back(prn);
                }
        }
    return satellites;
}


Synthetic_Relative_Epoch make_relative_epoch(Rtklib_Solver& solver,
    int epoch_index,
    const double* base_position_ecef,
    const double* rover_position_ecef,
    const std::vector<unsigned int>& satellites,
    bool gps_l1_l5 = false)
{
    const double tow_s = TEST_TOW_S + static_cast<double>(epoch_index);
    const gtime_t reception_time = gpst2time(TEST_GPS_WEEK, tow_s);
    // The GPS second band is L2 in slot 1, or L5 in slot 2 (shared with E5a)
    const int second_slot = gps_l1_l5 ? 2 : 1;
    const double wavelengths_m[2] = {
        SPEED_OF_LIGHT_M_S / FREQ1,
        SPEED_OF_LIGHT_M_S / (gps_l1_l5 ? FREQ5 : FREQ2)};
    constexpr double BASE_CLOCK_M = -45000.0;
    constexpr double ROVER_CLOCK_M = 75000.0;

    Synthetic_Relative_Epoch epoch;
    epoch.base_snapshot.has_observations = true;
    epoch.base_snapshot.observation_time = reception_time;
    epoch.base_snapshot.has_base_position = true;
    epoch.base_snapshot.base_position_ecef_m = {{base_position_ecef[0], base_position_ecef[1], base_position_ecef[2]}};
    epoch.base_snapshot.station_id = 19;

    for (const unsigned int prn : satellites)
        {
            const Gps_Ephemeris ephemeris = make_relative_ephemeris(prn);
            solver.gps_ephemeris_map[static_cast<int>(prn)] = ephemeris;
            solver.gps_cnav_ephemeris_map[static_cast<int>(prn)] = make_cnav_ephemeris(ephemeris);

            const double base_range_m = modeled_signal_range(
                ephemeris, reception_time, base_position_ecef, BASE_CLOCK_M, nullptr);
            const double rover_range_m = modeled_signal_range(
                ephemeris, reception_time, rover_position_ecef, ROVER_CLOCK_M, nullptr);

            obsd_t base_observation{};
            base_observation.time = reception_time;
            base_observation.sat = static_cast<unsigned char>(satno(SYS_GPS, static_cast<int>(prn)));
            base_observation.rcv = 2;

            Gnss_Synchro rover_l1 = make_rover_observation(prn);
            rover_l1.RX_time = tow_s;
            rover_l1.interp_TOW_ms = tow_s * 1000.0;
            Gnss_Synchro rover_l2 = rover_l1;
            std::memcpy(rover_l2.Signal, gps_l1_l5 ? "L5" : "2S", 3);

            for (int frequency = 0; frequency < 2; ++frequency)
                {
                    const int slot = frequency == 0 ? 0 : second_slot;
                    const double base_ambiguity_cycles = 100000.0 + 31.0 * static_cast<double>(prn) + 700.0 * frequency;
                    const double rover_ambiguity_cycles = base_ambiguity_cycles + 20.0 +
                                                          3.0 * static_cast<double>(prn) + 11.0 * frequency;
                    base_observation.P[slot] = base_range_m + BASE_CLOCK_M;
                    base_observation.L[slot] = (base_range_m + BASE_CLOCK_M) / wavelengths_m[frequency] +
                                               base_ambiguity_cycles;
                    base_observation.SNR[slot] = 200;
                    base_observation.code[slot] = frequency == 0 ? CODE_L1C : (gps_l1_l5 ? CODE_L5Q : CODE_L2S);

                    Gnss_Synchro& rover_observation = frequency == 0 ? rover_l1 : rover_l2;
                    rover_observation.Pseudorange_m = rover_range_m + ROVER_CLOCK_M;
                    rover_observation.Carrier_phase_rads = ((rover_range_m + ROVER_CLOCK_M) / wavelengths_m[frequency] +
                                                               rover_ambiguity_cycles) *
                                                           (2.0 * GNSS_PI);
                }

            epoch.rover_observations[static_cast<int>(prn * 2U)] = rover_l1;
            epoch.rover_observations[static_cast<int>(prn * 2U + 1U)] = rover_l2;
            epoch.base_snapshot.observations.push_back(base_observation);
        }
    return epoch;
}


void add_galileo_to_relative_epoch(Rtklib_Solver& solver,
    Synthetic_Relative_Epoch& epoch,
    int epoch_index,
    const double* base_position_ecef,
    const double* rover_position_ecef,
    const std::vector<unsigned int>& satellites)
{
    const double tow_s = TEST_TOW_S + static_cast<double>(epoch_index);
    const gtime_t reception_time = gpst2time(TEST_GPS_WEEK, tow_s);
    // The Galileo pair occupies RTKLIB slots 0 (E1) and 2 (E5a)
    const int slots[2] = {0, 2};
    const double wavelengths_m[2] = {
        SPEED_OF_LIGHT_M_S / FREQ1,
        SPEED_OF_LIGHT_M_S / FREQ5};
    constexpr double BASE_CLOCK_M = -45000.0;
    constexpr double ROVER_CLOCK_M = 75000.0;
    constexpr int ROVER_KEY_OFFSET = 1000;

    for (const unsigned int prn : satellites)
        {
            const Galileo_Ephemeris ephemeris = make_relative_galileo_ephemeris(prn);
            solver.store_galileo_ephemeris(ephemeris);

            const double base_range_m = modeled_signal_range(
                ephemeris, reception_time, base_position_ecef, BASE_CLOCK_M, nullptr);
            const double rover_range_m = modeled_signal_range(
                ephemeris, reception_time, rover_position_ecef, ROVER_CLOCK_M, nullptr);

            obsd_t base_observation{};
            base_observation.time = reception_time;
            base_observation.sat = static_cast<unsigned char>(satno(SYS_GAL, static_cast<int>(prn)));
            base_observation.rcv = 2;

            Gnss_Synchro rover_e1 = make_rover_observation(prn);
            rover_e1.System = 'E';
            std::memcpy(rover_e1.Signal, "1B", 3);
            rover_e1.RX_time = tow_s;
            rover_e1.interp_TOW_ms = tow_s * 1000.0;
            Gnss_Synchro rover_e5a = rover_e1;
            std::memcpy(rover_e5a.Signal, "5X", 3);

            for (int band = 0; band < 2; ++band)
                {
                    const int slot = slots[band];
                    const double base_ambiguity_cycles = 200000.0 + 37.0 * static_cast<double>(prn) + 900.0 * band;
                    const double rover_ambiguity_cycles = base_ambiguity_cycles + 40.0 +
                                                          5.0 * static_cast<double>(prn) + 13.0 * band;
                    base_observation.P[slot] = base_range_m + BASE_CLOCK_M;
                    base_observation.L[slot] = (base_range_m + BASE_CLOCK_M) / wavelengths_m[band] +
                                               base_ambiguity_cycles;
                    base_observation.SNR[slot] = 200;
                    base_observation.code[slot] = band == 0 ? CODE_L1X : CODE_L5X;

                    Gnss_Synchro& rover_observation = band == 0 ? rover_e1 : rover_e5a;
                    rover_observation.Pseudorange_m = rover_range_m + ROVER_CLOCK_M;
                    rover_observation.Carrier_phase_rads = ((rover_range_m + ROVER_CLOCK_M) / wavelengths_m[band] +
                                                              rover_ambiguity_cycles) *
                                                          (2.0 * GNSS_PI);
                }

            epoch.rover_observations[static_cast<int>(ROVER_KEY_OFFSET + prn * 2U)] = rover_e1;
            epoch.rover_observations[static_cast<int>(ROVER_KEY_OFFSET + prn * 2U + 1U)] = rover_e5a;
            epoch.base_snapshot.observations.push_back(base_observation);
        }
}


Gnss_Synchro make_rover_observation(unsigned int prn)
{
    Gnss_Synchro observation;
    observation.System = 'G';
    std::memcpy(observation.Signal, "1C", 3);
    observation.PRN = prn;
    observation.Pseudorange_m = 22000000.0 + 1000.0 * static_cast<double>(prn);
    observation.Carrier_phase_rads = 100000.0 + static_cast<double>(prn);
    observation.CN0_dB_hz = 45.0;
    observation.RX_time = TEST_TOW_S;
    observation.interp_TOW_ms = TEST_TOW_S * 1000.0;
    observation.Flag_valid_pseudorange = true;
    return observation;
}


obsd_t make_base_observation(unsigned int prn, const gtime_t& time)
{
    obsd_t observation{};
    observation.time = time;
    observation.sat = static_cast<unsigned char>(satno(SYS_GPS, static_cast<int>(prn)));
    observation.rcv = 2;
    observation.P[0] = 21000000.0 + 1000.0 * static_cast<double>(prn);
    observation.L[0] = 110000000.0 + static_cast<double>(prn);
    observation.SNR[0] = 180;
    observation.code[0] = CODE_L1C;
    return observation;
}


Ntrip_Rtcm_Snapshot complete_snapshot(std::size_t satellite_count = 4)
{
    Ntrip_Rtcm_Snapshot snapshot;
    snapshot.has_observations = true;
    snapshot.observation_time = gpst2time(TEST_GPS_WEEK, TEST_TOW_S);
    snapshot.has_base_position = true;
    snapshot.base_position_ecef_m = {{1113194.9, -4841695.5, 3985350.2}};
    snapshot.station_id = 7;
    for (std::size_t index = 0; index < satellite_count; ++index)
        {
            snapshot.observations.push_back(make_base_observation(
                static_cast<unsigned int>(index + 1), snapshot.observation_time));
        }
    return snapshot;
}


std::map<int, Gnss_Synchro> make_consistent_rover_observations(Rtklib_Solver& solver)
{
    const gtime_t reception_time = gpst2time(TEST_GPS_WEEK, TEST_TOW_S);
    const double receiver_position_geodetic[3] = {45.0 * D2R, 8.0 * D2R, 100.0};
    double receiver_position_ecef[3]{};
    pos2ecef(receiver_position_geodetic, receiver_position_ecef);

    std::map<int, Gnss_Synchro> observations;
    for (unsigned int prn = 1; prn <= 8; ++prn)
        {
            Gps_Ephemeris ephemeris = make_ephemeris(prn);
            ephemeris.ecc = 0.0;
            ephemeris.M_0 = 0.75 * static_cast<double>(prn - 1);
            ephemeris.OMEGA_0 = 2.1 * static_cast<double>(prn - 1);
            ephemeris.omega = 0.0;
            solver.gps_ephemeris_map[static_cast<int>(prn)] = ephemeris;

            const eph_t rtklib_ephemeris = eph_to_rtklib(ephemeris, TEST_GPS_WEEK);
            double pseudorange_m = 22000000.0;
            for (int iteration = 0; iteration < 6; ++iteration)
                {
                    const gtime_t transmission_time = timeadd(
                        reception_time, -pseudorange_m / SPEED_OF_LIGHT_M_S);
                    double satellite_position_ecef[3]{};
                    double satellite_clock_s = 0.0;
                    double variance = 0.0;
                    eph2pos(transmission_time,
                        &rtklib_ephemeris,
                        satellite_position_ecef,
                        &satellite_clock_s,
                        &variance);
                    double line_of_sight[3]{};
                    pseudorange_m = geodist(
                                        satellite_position_ecef,
                                        receiver_position_ecef,
                                        line_of_sight) -
                                    SPEED_OF_LIGHT_M_S * satellite_clock_s;
                }

            Gnss_Synchro observation = make_rover_observation(prn);
            observation.Pseudorange_m = pseudorange_m;
            observations[static_cast<int>(prn)] = observation;
        }
    return observations;
}
}  // namespace rtklib_fixed_base_test_detail


class RtklibFixedBaseTest : public ::testing::Test
{
protected:
    RtklibFixedBaseTest()
        : d_sensor_source_configuration(&d_configuration),
          d_sensor_data_aggregator(d_sensor_source_configuration, {})
    {
    }

    std::unique_ptr<Rtklib_Solver> make_solver(bool fallback_to_single = false)
    {
        using namespace rtklib_fixed_base_test_detail;
        return make_solver_with_options(fixed_base_options(), fallback_to_single);
    }

    std::unique_ptr<Rtklib_Solver> make_solver_with_options(const prcopt_t& options,
        bool fallback_to_single = false,
        uint32_t signal_flags = GPS_1C | GPS_2S)
    {
        using namespace rtklib_fixed_base_test_detail;

        rtk_t seed{};
        rtkinit(&seed, &options);

        Pvt_Conf configuration;
        configuration.ntrip_client_enabled = true;
        configuration.ntrip_fallback_to_single = fallback_to_single;
        configuration.ntrip_max_correction_age_s = TEST_MAX_AGE_S;
        configuration.use_e6_for_pvt = false;

        std::unique_ptr<Rtklib_Solver> solver(new Rtklib_Solver(
            seed,
            configuration,
            ".rtklib_fixed_base_test.dat",
            signal_flags,
            false,
            false));
        solver->set_ref_gps_week(TEST_GPS_WEEK);
        rtkfree(&seed);
        return solver;
    }

    std::map<int, Gnss_Synchro> make_rover_observations(Rtklib_Solver& solver, std::size_t satellite_count = 4)
    {
        using namespace rtklib_fixed_base_test_detail;

        std::map<int, Gnss_Synchro> observations;
        for (std::size_t index = 0; index < satellite_count; ++index)
            {
                const unsigned int prn = static_cast<unsigned int>(index + 1);
                solver.gps_ephemeris_map[static_cast<int>(prn)] = make_ephemeris(prn);
                observations[static_cast<int>(prn)] = make_rover_observation(prn);
            }
        return observations;
    }

    bool solve(Rtklib_Solver& solver,
        const std::map<int, Gnss_Synchro>& observations,
        const Ntrip_Rtcm_Snapshot* snapshot)
    {
        return solver.get_PVT(observations, 1.0, d_sensor_data_aggregator, false, snapshot);
    }

private:
    InMemoryConfiguration d_configuration;
    SensorDataSourceConfiguration d_sensor_source_configuration;
    SensorDataAggregator d_sensor_data_aggregator;
};


TEST_F(RtklibFixedBaseTest, SnapshotCopiesOwnTheirObservationStorage)
{
    using namespace rtklib_fixed_base_test_detail;

    Ntrip_Rtcm_Snapshot original = complete_snapshot();
    Ntrip_Rtcm_Snapshot copy = original;

    ASSERT_EQ(4U, original.observations.size());
    ASSERT_EQ(4U, copy.observations.size());
    copy.observations.front().P[0] += 10.0;
    copy.base_position_ecef_m[0] += 20.0;

    EXPECT_NE(original.observations.front().P[0], copy.observations.front().P[0]);
    EXPECT_NE(original.base_position_ecef_m[0], copy.base_position_ecef_m[0]);
}


TEST_F(RtklibFixedBaseTest, SolversOwnIndependentRtklibState)
{
    static_assert(!std::is_copy_constructible<Rtklib_Solver>::value,
        "RTKLIB solver state must not be copied");
    static_assert(!std::is_copy_assignable<Rtklib_Solver>::value,
        "RTKLIB solver state must not be copy-assigned");
    static_assert(!std::is_move_constructible<Rtklib_Solver>::value,
        "RTKLIB solver state must not be moved");
    static_assert(!std::is_move_assignable<Rtklib_Solver>::value,
        "RTKLIB solver state must not be move-assigned");

    using namespace rtklib_fixed_base_test_detail;
    rtk_t seed{};
    const prcopt_t options = fixed_base_options();
    rtkinit(&seed, &options);

    Pvt_Conf configuration;
    configuration.ntrip_client_enabled = true;
    std::unique_ptr<Rtklib_Solver> first(new Rtklib_Solver(
        seed, configuration, ".rtklib_fixed_base_first.dat", GPS_1C | GPS_2S, false, false));
    std::unique_ptr<Rtklib_Solver> second(new Rtklib_Solver(
        seed, configuration, ".rtklib_fixed_base_second.dat", GPS_1C | GPS_2S, false, false));

    // The seed and either solver may be destroyed independently. A shallow
    // rtk_t copy would leave both solvers pointing at the freed seed state.
    rtkfree(&seed);
    const std::map<int, Gnss_Synchro> no_observations;
    EXPECT_FALSE(solve(*first, no_observations, nullptr));
    EXPECT_FALSE(solve(*second, no_observations, nullptr));
    EXPECT_EQ(Rtklib_Fixed_Base_Status::NOT_REQUESTED, first->get_fixed_base_status());
    EXPECT_EQ(Rtklib_Fixed_Base_Status::NOT_REQUESTED, second->get_fixed_base_status());

    first.reset();
    EXPECT_FALSE(solve(*second, no_observations, nullptr));
    EXPECT_EQ(Rtklib_Fixed_Base_Status::NOT_REQUESTED, second->get_fixed_base_status());
}


TEST_F(RtklibFixedBaseTest, ReportsMissingObservations)
{
    auto solver = make_solver();
    const auto rover = make_rover_observations(*solver);
    Ntrip_Rtcm_Snapshot snapshot;
    snapshot.has_base_position = true;
    snapshot.base_position_ecef_m = {{1113194.9, -4841695.5, 3985350.2}};

    EXPECT_FALSE(solve(*solver, rover, &snapshot));
    EXPECT_EQ(Rtklib_Fixed_Base_Status::MISSING_OBSERVATIONS, solver->get_fixed_base_status());
    EXPECT_DOUBLE_EQ(0.0, solver->get_fixed_base_age_s());
    EXPECT_EQ(0U, solver->get_fixed_base_common_satellites());
}


TEST_F(RtklibFixedBaseTest, ReportsMissingPosition)
{
    using namespace rtklib_fixed_base_test_detail;
    auto solver = make_solver();
    const auto rover = make_rover_observations(*solver);
    Ntrip_Rtcm_Snapshot snapshot = complete_snapshot();
    snapshot.has_base_position = false;

    EXPECT_FALSE(solve(*solver, rover, &snapshot));
    EXPECT_EQ(Rtklib_Fixed_Base_Status::MISSING_POSITION, solver->get_fixed_base_status());
    EXPECT_DOUBLE_EQ(0.0, solver->get_fixed_base_age_s());
    EXPECT_EQ(0U, solver->get_fixed_base_common_satellites());
}


TEST_F(RtklibFixedBaseTest, ReportsStaleObservationsAndSignedAge)
{
    using namespace rtklib_fixed_base_test_detail;
    auto solver = make_solver();
    const auto rover = make_rover_observations(*solver);
    Ntrip_Rtcm_Snapshot snapshot = complete_snapshot();
    snapshot.observation_time = timeadd(snapshot.observation_time, -(TEST_MAX_AGE_S + 0.25));

    EXPECT_FALSE(solve(*solver, rover, &snapshot));
    EXPECT_EQ(Rtklib_Fixed_Base_Status::STALE, solver->get_fixed_base_status());
    EXPECT_NEAR(TEST_MAX_AGE_S + 0.25, solver->get_fixed_base_age_s(), 1.0e-9);
    EXPECT_EQ(0U, solver->get_fixed_base_common_satellites());
}


TEST_F(RtklibFixedBaseTest, ReportsInvalidBasePosition)
{
    using namespace rtklib_fixed_base_test_detail;
    auto solver = make_solver();
    const auto rover = make_rover_observations(*solver);
    Ntrip_Rtcm_Snapshot snapshot = complete_snapshot();
    snapshot.base_position_ecef_m[1] = std::numeric_limits<double>::quiet_NaN();

    EXPECT_FALSE(solve(*solver, rover, &snapshot));
    EXPECT_EQ(Rtklib_Fixed_Base_Status::INVALID_POSITION, solver->get_fixed_base_status());
    EXPECT_NEAR(0.0, solver->get_fixed_base_age_s(), 1.0e-9);
    EXPECT_EQ(0U, solver->get_fixed_base_common_satellites());
}


TEST_F(RtklibFixedBaseTest, ReportsInsufficientCommonSatellites)
{
    using namespace rtklib_fixed_base_test_detail;
    auto solver = make_solver();
    const auto rover = make_rover_observations(*solver);
    const Ntrip_Rtcm_Snapshot snapshot = complete_snapshot(3);

    EXPECT_FALSE(solve(*solver, rover, &snapshot));
    EXPECT_EQ(Rtklib_Fixed_Base_Status::INSUFFICIENT_COMMON_SATELLITES, solver->get_fixed_base_status());
    EXPECT_NEAR(0.0, solver->get_fixed_base_age_s(), 1.0e-9);
    EXPECT_EQ(3U, solver->get_fixed_base_common_satellites());
}


TEST_F(RtklibFixedBaseTest, DisabledFallbackDoesNotPublishARejectedBaseSolution)
{
    using namespace rtklib_fixed_base_test_detail;
    auto solver = make_solver(false);
    const auto rover = make_rover_observations(*solver);
    Ntrip_Rtcm_Snapshot snapshot = complete_snapshot();
    snapshot.has_base_position = false;

    EXPECT_FALSE(solve(*solver, rover, &snapshot));
    EXPECT_FALSE(solver->is_valid_position());
    EXPECT_EQ(Rtklib_Fixed_Base_Status::MISSING_POSITION, solver->get_fixed_base_status());
    EXPECT_EQ(0, solver->get_num_valid_observations());
}


TEST_F(RtklibFixedBaseTest, EnabledFallbackPublishesAConsistentSingleSolution)
{
    using namespace rtklib_fixed_base_test_detail;
    auto solver = make_solver(true);
    const auto rover = make_consistent_rover_observations(*solver);
    Ntrip_Rtcm_Snapshot snapshot;
    snapshot.has_base_position = true;
    snapshot.base_position_ecef_m = {{1113194.9, -4841695.5, 3985350.2}};

    EXPECT_TRUE(solve(*solver, rover, &snapshot));
    EXPECT_TRUE(solver->is_valid_position());
    EXPECT_EQ(SOLQ_SINGLE, solver->pvt_sol.stat);
    EXPECT_EQ(Rtklib_Fixed_Base_Status::MISSING_OBSERVATIONS, solver->get_fixed_base_status());
    EXPECT_GE(solver->get_num_valid_observations(), 4);
}


TEST_F(RtklibFixedBaseTest, AppliedButUnsolvableBaseReportsFailureWithoutPublishingPosition)
{
    using namespace rtklib_fixed_base_test_detail;
    auto solver = make_solver(true);
    const auto rover = make_consistent_rover_observations(*solver);
    const Ntrip_Rtcm_Snapshot snapshot = complete_snapshot();

    EXPECT_FALSE(solve(*solver, rover, &snapshot));
    EXPECT_FALSE(solver->is_valid_position());
    EXPECT_EQ(Rtklib_Fixed_Base_Status::SOLVER_FAILURE, solver->get_fixed_base_status());
    EXPECT_NEAR(0.0, solver->get_fixed_base_age_s(), 1.0e-9);
    EXPECT_EQ(4U, solver->get_fixed_base_common_satellites());
}


TEST_F(RtklibFixedBaseTest, ConsistentDualFrequencyBaseProducesRelativeSolution)
{
    using namespace rtklib_fixed_base_test_detail;
    const double base_position_geodetic[3] = {45.0 * D2R, 8.0 * D2R, 100.0};
    double base_position_ecef[3]{};
    pos2ecef(base_position_geodetic, base_position_ecef);
    const double baseline_enu_m[3] = {8.0, 4.0, 1.0};
    double baseline_ecef_m[3]{};
    enu2ecef(base_position_geodetic, baseline_enu_m, baseline_ecef_m);
    const double rover_position_ecef[3] = {
        base_position_ecef[0] + baseline_ecef_m[0],
        base_position_ecef[1] + baseline_ecef_m[1],
        base_position_ecef[2] + baseline_ecef_m[2]};

    auto solver = make_solver(false);
    const std::vector<unsigned int> satellites = select_relative_satellites(
        base_position_ecef, rover_position_ecef);
    ASSERT_GE(satellites.size(), 6U);

    bool obtained_float = false;
    bool obtained_fix = false;
    for (int epoch_index = 0; epoch_index < 8; ++epoch_index)
        {
            const Synthetic_Relative_Epoch epoch = make_relative_epoch(
                *solver, epoch_index, base_position_ecef, rover_position_ecef, satellites);
            ASSERT_EQ(satellites.size() * 2U, epoch.rover_observations.size());
            ASSERT_EQ(satellites.size(), epoch.base_snapshot.observations.size());
            for (const unsigned int prn : satellites)
                {
                    const auto rover_l1 = epoch.rover_observations.find(static_cast<int>(prn * 2U));
                    const auto rover_l2 = epoch.rover_observations.find(static_cast<int>(prn * 2U + 1U));
                    ASSERT_NE(epoch.rover_observations.cend(), rover_l1);
                    ASSERT_NE(epoch.rover_observations.cend(), rover_l2);
                    EXPECT_EQ("1C", std::string(rover_l1->second.Signal, 2));
                    EXPECT_EQ("2S", std::string(rover_l2->second.Signal, 2));
                }
            for (const obsd_t& base_observation : epoch.base_snapshot.observations)
                {
                    EXPECT_EQ(2, base_observation.rcv);
                    EXPECT_EQ(CODE_L1C, base_observation.code[0]);
                    EXPECT_EQ(CODE_L2S, base_observation.code[1]);
                    EXPECT_NE(0.0, base_observation.P[0]);
                    EXPECT_NE(0.0, base_observation.P[1]);
                    EXPECT_NE(0.0, base_observation.L[0]);
                    EXPECT_NE(0.0, base_observation.L[1]);
                }

            SCOPED_TRACE(::testing::Message() << "epoch=" << epoch_index);
            ASSERT_TRUE(solve(*solver, epoch.rover_observations, &epoch.base_snapshot));
            EXPECT_EQ(Rtklib_Fixed_Base_Status::APPLIED, solver->get_fixed_base_status());
            EXPECT_EQ(satellites.size(), solver->get_fixed_base_common_satellites());
            obtained_float = obtained_float || solver->pvt_sol.stat == SOLQ_FLOAT;
            obtained_fix = obtained_fix || solver->pvt_sol.stat == SOLQ_FIX;
            EXPECT_TRUE(solver->pvt_sol.stat == SOLQ_FLOAT || solver->pvt_sol.stat == SOLQ_FIX);
        }

    EXPECT_TRUE(obtained_float || obtained_fix);
    EXPECT_TRUE(obtained_fix);
    const double position_error_m = std::sqrt(
        std::pow(solver->pvt_sol.rr[0] - rover_position_ecef[0], 2.0) +
        std::pow(solver->pvt_sol.rr[1] - rover_position_ecef[1], 2.0) +
        std::pow(solver->pvt_sol.rr[2] - rover_position_ecef[2], 2.0));
    EXPECT_LT(position_error_m, 0.1);
}


TEST_F(RtklibFixedBaseTest, CombinedGpsGalileoBaseProducesRelativeSolution)
{
    using namespace rtklib_fixed_base_test_detail;
    const double base_position_geodetic[3] = {45.0 * D2R, 8.0 * D2R, 100.0};
    double base_position_ecef[3]{};
    pos2ecef(base_position_geodetic, base_position_ecef);
    const double baseline_enu_m[3] = {8.0, 4.0, 1.0};
    double baseline_ecef_m[3]{};
    enu2ecef(base_position_geodetic, baseline_enu_m, baseline_ecef_m);
    const double rover_position_ecef[3] = {
        base_position_ecef[0] + baseline_ecef_m[0],
        base_position_ecef[1] + baseline_ecef_m[1],
        base_position_ecef[2] + baseline_ecef_m[2]};

    prcopt_t options = fixed_base_options();
    // Galileo E5a lives in the third frequency slot: the frequency loops must
    // span it, mirroring the adapter's num_bands handling
    options.nf = 3;
    options.navsys = SYS_GPS | SYS_GAL;
    auto solver = make_solver_with_options(options, false,
        GPS_1C | GPS_2S | GAL_1B | GAL_E5a);

    const std::vector<unsigned int> gps_satellites = select_relative_satellites(
        base_position_ecef, rover_position_ecef);
    const std::vector<unsigned int> galileo_satellites = select_relative_galileo_satellites(
        base_position_ecef, rover_position_ecef);
    ASSERT_GE(gps_satellites.size(), 4U);
    ASSERT_GE(galileo_satellites.size(), 4U);

    bool obtained_fix = false;
    for (int epoch_index = 0; epoch_index < 8; ++epoch_index)
        {
            Synthetic_Relative_Epoch epoch = make_relative_epoch(
                *solver, epoch_index, base_position_ecef, rover_position_ecef, gps_satellites);
            add_galileo_to_relative_epoch(
                *solver, epoch, epoch_index, base_position_ecef, rover_position_ecef, galileo_satellites);
            ASSERT_EQ((gps_satellites.size() + galileo_satellites.size()) * 2U,
                epoch.rover_observations.size());
            for (const obsd_t& base_observation : epoch.base_snapshot.observations)
                {
                    if (satsys(base_observation.sat, nullptr) == SYS_GAL)
                        {
                            EXPECT_EQ(CODE_L1X, base_observation.code[0]);
                            EXPECT_EQ(CODE_L5X, base_observation.code[2]);
                            EXPECT_EQ(CODE_NONE, base_observation.code[1]);
                            EXPECT_NE(0.0, base_observation.L[2]);
                        }
                }

            SCOPED_TRACE(::testing::Message() << "epoch=" << epoch_index);
            ASSERT_TRUE(solve(*solver, epoch.rover_observations, &epoch.base_snapshot));
            EXPECT_EQ(Rtklib_Fixed_Base_Status::APPLIED, solver->get_fixed_base_status());
            // Both constellations must survive the base pairing
            EXPECT_EQ(gps_satellites.size() + galileo_satellites.size(),
                solver->get_fixed_base_common_satellites());
            obtained_fix = obtained_fix || solver->pvt_sol.stat == SOLQ_FIX;
            EXPECT_TRUE(solver->pvt_sol.stat == SOLQ_FLOAT || solver->pvt_sol.stat == SOLQ_FIX);
        }

    EXPECT_TRUE(obtained_fix);
    const double position_error_m = std::sqrt(
        std::pow(solver->pvt_sol.rr[0] - rover_position_ecef[0], 2.0) +
        std::pow(solver->pvt_sol.rr[1] - rover_position_ecef[1], 2.0) +
        std::pow(solver->pvt_sol.rr[2] - rover_position_ecef[2], 2.0));
    EXPECT_LT(position_error_m, 0.1);
}


TEST_F(RtklibFixedBaseTest, CombinedGpsL1L5GalileoBaseProducesRelativeSolution)
{
    using namespace rtklib_fixed_base_test_detail;
    const double base_position_geodetic[3] = {45.0 * D2R, 8.0 * D2R, 100.0};
    double base_position_ecef[3]{};
    pos2ecef(base_position_geodetic, base_position_ecef);
    const double baseline_enu_m[3] = {8.0, 4.0, 1.0};
    double baseline_ecef_m[3]{};
    enu2ecef(base_position_geodetic, baseline_enu_m, baseline_ecef_m);
    const double rover_position_ecef[3] = {
        base_position_ecef[0] + baseline_ecef_m[0],
        base_position_ecef[1] + baseline_ecef_m[1],
        base_position_ecef[2] + baseline_ecef_m[2]};

    prcopt_t options = fixed_base_options();
    // Both second bands (GPS L5 and Galileo E5a) live in the third slot
    options.nf = 3;
    options.navsys = SYS_GPS | SYS_GAL;
    auto solver = make_solver_with_options(options, false,
        GPS_1C | GPS_L5 | GAL_1B | GAL_E5a);

    const std::vector<unsigned int> gps_satellites = select_relative_satellites(
        base_position_ecef, rover_position_ecef);
    const std::vector<unsigned int> galileo_satellites = select_relative_galileo_satellites(
        base_position_ecef, rover_position_ecef);
    ASSERT_GE(gps_satellites.size(), 4U);
    ASSERT_GE(galileo_satellites.size(), 4U);

    bool obtained_fix = false;
    for (int epoch_index = 0; epoch_index < 8; ++epoch_index)
        {
            Synthetic_Relative_Epoch epoch = make_relative_epoch(
                *solver, epoch_index, base_position_ecef, rover_position_ecef,
                gps_satellites, true);
            add_galileo_to_relative_epoch(
                *solver, epoch, epoch_index, base_position_ecef, rover_position_ecef, galileo_satellites);
            for (const obsd_t& base_observation : epoch.base_snapshot.observations)
                {
                    // Every satellite of either system pairs in slots 0/2
                    EXPECT_EQ(CODE_NONE, base_observation.code[1]);
                    EXPECT_NE(0.0, base_observation.L[2]);
                    if (satsys(base_observation.sat, nullptr) == SYS_GPS)
                        {
                            EXPECT_EQ(CODE_L5Q, base_observation.code[2]);
                        }
                }

            SCOPED_TRACE(::testing::Message() << "epoch=" << epoch_index);
            ASSERT_TRUE(solve(*solver, epoch.rover_observations, &epoch.base_snapshot));
            EXPECT_EQ(Rtklib_Fixed_Base_Status::APPLIED, solver->get_fixed_base_status());
            EXPECT_EQ(gps_satellites.size() + galileo_satellites.size(),
                solver->get_fixed_base_common_satellites());
            obtained_fix = obtained_fix || solver->pvt_sol.stat == SOLQ_FIX;
            EXPECT_TRUE(solver->pvt_sol.stat == SOLQ_FLOAT || solver->pvt_sol.stat == SOLQ_FIX);
        }

    EXPECT_TRUE(obtained_fix);
    const double position_error_m = std::sqrt(
        std::pow(solver->pvt_sol.rr[0] - rover_position_ecef[0], 2.0) +
        std::pow(solver->pvt_sol.rr[1] - rover_position_ecef[1], 2.0) +
        std::pow(solver->pvt_sol.rr[2] - rover_position_ecef[2], 2.0));
    EXPECT_LT(position_error_m, 0.1);
}


TEST_F(RtklibFixedBaseTest, RepeatedPhaseOutliersResetTheStaleAmbiguity)
{
    using namespace rtklib_fixed_base_test_detail;
    const double base_position_geodetic[3] = {45.0 * D2R, 8.0 * D2R, 100.0};
    double base_position_ecef[3]{};
    pos2ecef(base_position_geodetic, base_position_ecef);
    const double baseline_enu_m[3] = {8.0, 4.0, 1.0};
    double baseline_ecef_m[3]{};
    enu2ecef(base_position_geodetic, baseline_enu_m, baseline_ecef_m);
    const double rover_position_ecef[3] = {
        base_position_ecef[0] + baseline_ecef_m[0],
        base_position_ecef[1] + baseline_ecef_m[1],
        base_position_ecef[2] + baseline_ecef_m[2]};

    /* a long outage counter keeps the satellite from being rescued by the
       maxout path, so the recovery below can only come from the reject counter */
    prcopt_t options = fixed_base_options();
    options.maxout = 50;
    auto solver = make_solver_with_options(options);
    const std::vector<unsigned int> satellites = select_relative_satellites(
        base_position_ecef, rover_position_ecef);
    ASSERT_GE(satellites.size(), 6U);

    /* a carrier-phase jump larger than the innovation threshold (100 m here)
       makes the double differences of one satellite outliers from then on */
    const unsigned int corrupted_prn = satellites.back();
    constexpr double PHASE_JUMP_M = 300.0;
    constexpr int FIRST_CORRUPTED_EPOCH = 4;
    const double wavelengths_m[2] = {
        SPEED_OF_LIGHT_M_S / FREQ1,
        SPEED_OF_LIGHT_M_S / FREQ2};

    std::vector<unsigned int> valid_satellites;
    for (int epoch_index = 0; epoch_index < 12; ++epoch_index)
        {
            Synthetic_Relative_Epoch epoch = make_relative_epoch(
                *solver, epoch_index, base_position_ecef, rover_position_ecef, satellites);
            if (epoch_index >= FIRST_CORRUPTED_EPOCH)
                {
                    for (unsigned int band = 0; band < 2U; ++band)
                        {
                            const auto observation = epoch.rover_observations.find(
                                static_cast<int>(corrupted_prn * 2U + band));
                            ASSERT_NE(epoch.rover_observations.end(), observation);
                            observation->second.Carrier_phase_rads +=
                                PHASE_JUMP_M / wavelengths_m[band] * 2.0 * GNSS_PI;
                        }
                }

            SCOPED_TRACE(::testing::Message() << "epoch=" << epoch_index);
            ASSERT_TRUE(solve(*solver, epoch.rover_observations, &epoch.base_snapshot));
            valid_satellites.push_back(solver->pvt_sol.ns);
        }

    const unsigned int all_satellites = static_cast<unsigned int>(satellites.size());
    EXPECT_EQ(all_satellites, valid_satellites[FIRST_CORRUPTED_EPOCH - 1]);
    /* the jump first takes the satellite out of the solution ... */
    EXPECT_LT(valid_satellites[FIRST_CORRUPTED_EPOCH], all_satellites);
    /* ... and the stale bias is then reinitialized instead of being rejected
       forever, so the satellite comes back on its own */
    EXPECT_EQ(all_satellites, valid_satellites.back());
}


TEST_F(RtklibFixedBaseTest, AnUnresolvedHalfCycleAmbiguityIsDeweighted)
{
    using namespace rtklib_fixed_base_test_detail;
    const double base_position_geodetic[3] = {45.0 * D2R, 8.0 * D2R, 100.0};
    double base_position_ecef[3]{};
    pos2ecef(base_position_geodetic, base_position_ecef);
    const double baseline_enu_m[3] = {8.0, 4.0, 1.0};
    double baseline_ecef_m[3]{};
    enu2ecef(base_position_geodetic, baseline_enu_m, baseline_ecef_m);
    const double rover_position_ecef[3] = {
        base_position_ecef[0] + baseline_ecef_m[0],
        base_position_ecef[1] + baseline_ecef_m[1],
        base_position_ecef[2] + baseline_ecef_m[2]};

    /* keep the float solution: a satellite flagged as half-cycle ambiguous is
       excluded from ambiguity resolution on its own, and this exercises the
       measurement weighting instead */
    prcopt_t options = fixed_base_options();
    options.modear = ARMODE_OFF;

    const auto position_variance_with_flag = [&](bool report_half_cycle) {
        auto solver = make_solver_with_options(options);
        const std::vector<unsigned int> satellites = select_relative_satellites(
            base_position_ecef, rover_position_ecef);
        EXPECT_GE(satellites.size(), 6U);

        for (int epoch_index = 0; epoch_index < 6; ++epoch_index)
            {
                Synthetic_Relative_Epoch epoch = make_relative_epoch(
                    *solver, epoch_index, base_position_ecef, rover_position_ecef, satellites);
                for (auto& rover_observation : epoch.rover_observations)
                    {
                        rover_observation.second.Flag_half_cycle_slip = report_half_cycle;
                    }

                EXPECT_TRUE(solve(*solver, epoch.rover_observations, &epoch.base_snapshot));
            }

        return static_cast<double>(solver->pvt_sol.qr[0]) +
               static_cast<double>(solver->pvt_sol.qr[1]) +
               static_cast<double>(solver->pvt_sol.qr[2]);
    };

    const double variance_unflagged = position_variance_with_flag(false);
    const double variance_flagged = position_variance_with_flag(true);

    // the flagged carrier phases are worth their half wavelength rather than a
    // few millimeters, and the reported solution says so
    EXPECT_GT(variance_flagged, 1.001 * variance_unflagged)
        << "flagged: " << variance_flagged << " m^2, unflagged: " << variance_unflagged << " m^2";
}


TEST_F(RtklibFixedBaseTest, TooFewPhaseObservationsFallBackToACodeDifferencedSolution)
{
    using namespace rtklib_fixed_base_test_detail;
    const double base_position_geodetic[3] = {45.0 * D2R, 8.0 * D2R, 100.0};
    double base_position_ecef[3]{};
    pos2ecef(base_position_geodetic, base_position_ecef);
    const double baseline_enu_m[3] = {8.0, 4.0, 1.0};
    double baseline_ecef_m[3]{};
    enu2ecef(base_position_geodetic, baseline_enu_m, baseline_ecef_m);
    const double rover_position_ecef[3] = {
        base_position_ecef[0] + baseline_ecef_m[0],
        base_position_ecef[1] + baseline_ecef_m[1],
        base_position_ecef[2] + baseline_ecef_m[2]};

    auto solver = make_solver(false);
    const std::vector<unsigned int> satellites = select_relative_satellites(
        base_position_ecef, rover_position_ecef);
    ASSERT_GE(satellites.size(), 6U);

    /* keep the code observations of every satellite but leave carrier phase on
       only two of them, so fewer than four phase measurements survive */
    const std::vector<unsigned int> phase_satellites(satellites.cbegin(), satellites.cbegin() + 2);

    for (int epoch_index = 0; epoch_index < 4; ++epoch_index)
        {
            Synthetic_Relative_Epoch epoch = make_relative_epoch(
                *solver, epoch_index, base_position_ecef, rover_position_ecef, satellites);
            for (const unsigned int prn : satellites)
                {
                    if (std::find(phase_satellites.cbegin(), phase_satellites.cend(), prn) !=
                        phase_satellites.cend())
                        {
                            continue;
                        }
                    for (unsigned int band = 0; band < 2U; ++band)
                        {
                            const auto observation = epoch.rover_observations.find(
                                static_cast<int>(prn * 2U + band));
                            ASSERT_NE(epoch.rover_observations.end(), observation);
                            observation->second.Carrier_phase_rads = 0.0;
                        }
                    for (obsd_t& base_observation : epoch.base_snapshot.observations)
                        {
                            if (base_observation.sat == satno(SYS_GPS, static_cast<int>(prn)))
                                {
                                    base_observation.L[0] = base_observation.L[1] = 0.0;
                                }
                        }
                }

            SCOPED_TRACE(::testing::Message() << "epoch=" << epoch_index);
            /* the epoch is not dropped: the code-differenced solution is kept */
            ASSERT_TRUE(solve(*solver, epoch.rover_observations, &epoch.base_snapshot));
            EXPECT_EQ(Rtklib_Fixed_Base_Status::APPLIED, solver->get_fixed_base_status());
            EXPECT_EQ(SOLQ_DGPS, solver->pvt_sol.stat);
        }

    const double position_error_m = std::sqrt(
        std::pow(solver->pvt_sol.rr[0] - rover_position_ecef[0], 2.0) +
        std::pow(solver->pvt_sol.rr[1] - rover_position_ecef[1], 2.0) +
        std::pow(solver->pvt_sol.rr[2] - rover_position_ecef[2], 2.0));
    EXPECT_LT(position_error_m, 10.0);
}


TEST_F(RtklibFixedBaseTest, Demo5ArManagementProducesFixedSolutionWithFixAndHold)
{
    using namespace rtklib_fixed_base_test_detail;
    const double base_position_geodetic[3] = {45.0 * D2R, 8.0 * D2R, 100.0};
    double base_position_ecef[3]{};
    pos2ecef(base_position_geodetic, base_position_ecef);
    const double baseline_enu_m[3] = {8.0, 4.0, 1.0};
    double baseline_ecef_m[3]{};
    enu2ecef(base_position_geodetic, baseline_enu_m, baseline_ecef_m);
    const double rover_position_ecef[3] = {
        base_position_ecef[0] + baseline_ecef_m[0],
        base_position_ecef[1] + baseline_ecef_m[1],
        base_position_ecef[2] + baseline_ecef_m[2]};

    /* enable the demo5 AR management features: fix-and-hold with minimum-satellite
       gates, AR filtering, the position-variance gate, the satellite-count-dependent
       AR ratio threshold, and SNR-dependent observation weighting */
    prcopt_t options = fixed_base_options();
    options.modear = ARMODE_FIXHOLD;
    options.minfix = 2;
    options.arfilter = 1;
    options.minfixsats = 4;
    options.minholdsats = 5;
    options.mindropsats = 10;
    options.varholdamb = 0.1;
    options.armaxposvar = 4.0;
    options.thresar[5] = 2.0;
    options.thresar[6] = 4.0;
    options.err[5] = 52.0;
    options.err[6] = 0.005;
    auto solver = make_solver_with_options(options);

    const std::vector<unsigned int> satellites = select_relative_satellites(
        base_position_ecef, rover_position_ecef);
    ASSERT_GE(satellites.size(), 6U);

    bool obtained_fix = false;
    for (int epoch_index = 0; epoch_index < 8; ++epoch_index)
        {
            const Synthetic_Relative_Epoch epoch = make_relative_epoch(
                *solver, epoch_index, base_position_ecef, rover_position_ecef, satellites);
            SCOPED_TRACE(::testing::Message() << "epoch=" << epoch_index);
            ASSERT_TRUE(solve(*solver, epoch.rover_observations, &epoch.base_snapshot));
            EXPECT_EQ(Rtklib_Fixed_Base_Status::APPLIED, solver->get_fixed_base_status());
            EXPECT_TRUE(solver->pvt_sol.stat == SOLQ_FLOAT || solver->pvt_sol.stat == SOLQ_FIX);
            obtained_fix = obtained_fix || solver->pvt_sol.stat == SOLQ_FIX;
        }
    EXPECT_TRUE(obtained_fix);

    const double position_error_m = std::sqrt(
        std::pow(solver->pvt_sol.rr[0] - rover_position_ecef[0], 2.0) +
        std::pow(solver->pvt_sol.rr[1] - rover_position_ecef[1], 2.0) +
        std::pow(solver->pvt_sol.rr[2] - rover_position_ecef[2], 2.0));
    EXPECT_LT(position_error_m, 0.1);
}


TEST_F(RtklibFixedBaseTest, CorrectionOutageResetsRelativeStateBeforeReacquisition)
{
    using namespace rtklib_fixed_base_test_detail;
    const double base_position_geodetic[3] = {45.0 * D2R, 8.0 * D2R, 100.0};
    double base_position_ecef[3]{};
    pos2ecef(base_position_geodetic, base_position_ecef);

    const double initial_baseline_enu_m[3] = {8.0, 4.0, 1.0};
    double initial_baseline_ecef_m[3]{};
    enu2ecef(base_position_geodetic, initial_baseline_enu_m, initial_baseline_ecef_m);
    const double initial_rover_position_ecef[3] = {
        base_position_ecef[0] + initial_baseline_ecef_m[0],
        base_position_ecef[1] + initial_baseline_ecef_m[1],
        base_position_ecef[2] + initial_baseline_ecef_m[2]};

    // A discontinuous rover position makes stale static-filter state observable:
    // reacquisition must initialize from the post-outage measurements.
    const double reacquired_baseline_enu_m[3] = {208.0, 104.0, 1.0};
    double reacquired_baseline_ecef_m[3]{};
    enu2ecef(base_position_geodetic, reacquired_baseline_enu_m, reacquired_baseline_ecef_m);
    const double reacquired_rover_position_ecef[3] = {
        base_position_ecef[0] + reacquired_baseline_ecef_m[0],
        base_position_ecef[1] + reacquired_baseline_ecef_m[1],
        base_position_ecef[2] + reacquired_baseline_ecef_m[2]};

    auto solver = make_solver(true);
    const std::vector<unsigned int> satellites = select_relative_satellites(
        base_position_ecef, reacquired_rover_position_ecef);
    ASSERT_GE(satellites.size(), 6U);

    bool obtained_fix = false;
    for (int epoch_index = 0; epoch_index < 8; ++epoch_index)
        {
            const Synthetic_Relative_Epoch epoch = make_relative_epoch(
                *solver, epoch_index, base_position_ecef, initial_rover_position_ecef, satellites);
            ASSERT_TRUE(solve(*solver, epoch.rover_observations, &epoch.base_snapshot));
            ASSERT_EQ(Rtklib_Fixed_Base_Status::APPLIED, solver->get_fixed_base_status());
            ASSERT_TRUE(solver->pvt_sol.stat == SOLQ_FLOAT || solver->pvt_sol.stat == SOLQ_FIX);
            obtained_fix = obtained_fix || solver->pvt_sol.stat == SOLQ_FIX;
        }
    ASSERT_TRUE(obtained_fix);

    const Synthetic_Relative_Epoch outage_epoch = make_relative_epoch(
        *solver, 8, base_position_ecef, reacquired_rover_position_ecef, satellites);
    Ntrip_Rtcm_Snapshot unavailable_snapshot;
    unavailable_snapshot.has_base_position = true;
    unavailable_snapshot.base_position_ecef_m = {{base_position_ecef[0], base_position_ecef[1], base_position_ecef[2]}};
    ASSERT_TRUE(solve(*solver, outage_epoch.rover_observations, &unavailable_snapshot));
    EXPECT_EQ(Rtklib_Fixed_Base_Status::MISSING_OBSERVATIONS, solver->get_fixed_base_status());
    EXPECT_EQ(SOLQ_SINGLE, solver->pvt_sol.stat);
    EXPECT_TRUE(solver->is_valid_position());

    const Synthetic_Relative_Epoch reacquired_epoch = make_relative_epoch(
        *solver, 9, base_position_ecef, reacquired_rover_position_ecef, satellites);
    ASSERT_TRUE(solve(*solver, reacquired_epoch.rover_observations, &reacquired_epoch.base_snapshot));
    EXPECT_EQ(Rtklib_Fixed_Base_Status::APPLIED, solver->get_fixed_base_status());
    EXPECT_TRUE(solver->pvt_sol.stat == SOLQ_FLOAT || solver->pvt_sol.stat == SOLQ_FIX);

    const double position_error_m = std::sqrt(
        std::pow(solver->pvt_sol.rr[0] - reacquired_rover_position_ecef[0], 2.0) +
        std::pow(solver->pvt_sol.rr[1] - reacquired_rover_position_ecef[1], 2.0) +
        std::pow(solver->pvt_sol.rr[2] - reacquired_rover_position_ecef[2], 2.0));
    EXPECT_LT(position_error_m, 0.1);
}
