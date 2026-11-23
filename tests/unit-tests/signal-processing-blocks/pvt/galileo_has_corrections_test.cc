/*!
 * \file galileo_has_corrections_test.cc
 * \brief Unit tests for the selection of the satellites that get Galileo HAS
 * corrections in the PVT solution
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "Galileo_CNAV.h"
#include "galileo_ephemeris.h"
#include "galileo_has_data.h"
#include "gnss_synchro.h"
#include "gps_ephemeris.h"
#include "in_memory_configuration.h"
#include "monitor_pvt.h"
#include "ntrip_rtcm_client.h"
#include "pvt_conf.h"
#include "rtklib_conversions.h"
#include "rtklib_ephemeris.h"
#include "rtklib_rtkcmn.h"
#include "rtklib_rtkpos.h"
#include "rtklib_rtksvr.h"
#include "rtklib_solver.h"
#include "sensor_data/sensor_data_aggregator.h"
#include "sensor_data/sensor_data_source_configuration.h"
#include "signal_flag.h"
#include <gtest/gtest.h>
#include <matio.h>
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

class GalileoHasCorrectionsTest : public ::testing::Test
{
protected:
    // Used by value only: C++11 would need out-of-class definitions to bind
    // them to references
    static constexpr uint32_t PRN = 12U;
    static constexpr int32_t IOD = 77;
    static constexpr uint32_t WEEK = 1400U;
    static constexpr uint32_t MESSAGE_TOW = 600U;
    static constexpr uint8_t MASK_ID = 5U;
    static constexpr uint8_t IOD_SET_ID = 3U;
    static constexpr int16_t CODE_BIAS_UNAVAILABLE_RAW = -1024;  // HAS SIS ICD 1.0 Table 37
    static constexpr int16_t E1_CODE_BIAS_RAW = 25;              // 0.50 m
    static constexpr int16_t E5A_CODE_BIAS_RAW = 10;             // 0.20 m
    static constexpr int16_t E6_CODE_BIAS_RAW = -40;             // -0.80 m
    static constexpr uint8_t E1_C_INDEX = 1;                     // HAS SIS ICD 1.0 Table 20
    static constexpr uint8_t E5A_Q_INDEX = 4;
    static constexpr uint8_t E6_C_INDEX = 13;

    struct Code_Bias
    {
        uint8_t signal_index;
        int16_t raw;
    };
    using Code_Biases = std::vector<Code_Bias>;

    static Code_Biases e1_e6_biases(int16_t e6_code_bias_raw = E6_CODE_BIAS_RAW)
    {
        return {{E1_C_INDEX, E1_CODE_BIAS_RAW}, {E6_C_INDEX, e6_code_bias_raw}};
    }

    // Galileo E1 + E6 receiver, which takes its clock model from I/NAV, with
    // the I/NAV ephemeris of the satellite
    std::unique_ptr<Rtklib_Solver> make_solver(bool use_e6_for_pvt = true)
    {
        auto solver = make_solver_without_ephemeris(GAL_1B | GAL_E6, use_e6_for_pvt);
        EXPECT_EQ(Galileo_Nav_Message_Type::INAV, solver->galileo_nav_message_type_for_pvt());
        EXPECT_TRUE(solver->store_galileo_ephemeris(make_ephemeris(Galileo_Nav_Message_Type::INAV)));
        return solver;
    }

    static std::unique_ptr<Rtklib_Solver> make_solver_without_ephemeris(uint32_t signal_flags, bool use_e6_for_pvt = true)
    {
        const rtk_t rtk{};
        Pvt_Conf conf;
        conf.use_e6_for_pvt = use_e6_for_pvt;
        return std::unique_ptr<Rtklib_Solver>(new Rtklib_Solver(
            rtk, conf, ".rtklib_solver_galileo_has_test.dat", signal_flags, false, false));
    }

    static Galileo_Ephemeris make_ephemeris(Galileo_Nav_Message_Type source, int32_t iod = IOD)
    {
        Galileo_Ephemeris ephemeris;
        ephemeris.PRN = PRN;
        ephemeris.WN = static_cast<int32_t>(WEEK);
        ephemeris.tow = static_cast<int32_t>(MESSAGE_TOW);
        ephemeris.toe = static_cast<int32_t>(MESSAGE_TOW);
        ephemeris.IOD_ephemeris = iod;
        ephemeris.af0 = source == Galileo_Nav_Message_Type::INAV ? 1.0e-4 : 2.0e-4;
        ephemeris.BGD_E1E5a = 2.0e-8;
        ephemeris.BGD_E1E5b = 4.0e-8;
        ephemeris.nav_message_type = source;
        return ephemeris;
    }

    // MT1 with the content blocks selected by the arguments, for one Galileo
    // satellite. No code bias block if code_biases is empty. Validity
    // intervals of 300 s.
    static Galileo_HAS_data make_message(bool orbit_and_clock, const Code_Biases& code_biases,
        uint8_t iod_set_id = IOD_SET_ID, int32_t iod_ref = IOD, int16_t delta_clock_correction = 40)
    {
        Galileo_HAS_data has_data;
        has_data.week = WEEK;
        has_data.tow = MESSAGE_TOW;
        has_data.has_status = 1;
        has_data.message_id = 0;
        has_data.header = mt1_header{};
        has_data.header.toh = static_cast<uint16_t>(MESSAGE_TOW);
        has_data.header.mask_id = MASK_ID;
        has_data.header.iod_set_id = iod_set_id;
        has_data.header.mask_flag = true;
        has_data.header.orbit_correction_flag = orbit_and_clock;
        has_data.header.clock_fullset_flag = orbit_and_clock;
        has_data.header.code_bias_flag = !code_biases.empty();

        has_data.Nsys = 1;
        has_data.Nsys_sub = 0;
        has_data.gnss_id_mask = {HAS_MSG_GALILEO_SYSTEM};
        has_data.satellite_mask = {uint64_t{1} << (HAS_MSG_NUMBER_SATELLITE_IDS - PRN)};  // MSB is PRN 1
        // MSB is Signal Index 0; biases are sent in increasing signal index
        uint16_t signal_mask = 0;
        std::vector<int16_t> code_bias_row;
        for (const auto& code_bias : code_biases)
            {
                signal_mask |= static_cast<uint16_t>(1U << (HAS_MSG_NUMBER_SIGNAL_MASKS - 1 - code_bias.signal_index));
                code_bias_row.push_back(code_bias.raw);
            }
        has_data.signal_mask = {code_biases.empty() ? static_cast<uint16_t>(1U << (HAS_MSG_NUMBER_SIGNAL_MASKS - 1 - E1_C_INDEX)) : signal_mask};
        has_data.cell_mask_availability_flag = {false};
        has_data.nav_message = {0};

        has_data.validity_interval_index_orbit_corrections = 10;
        has_data.validity_interval_index_clock_fullset_corrections = 10;
        has_data.validity_interval_index_clock_subset_corrections = 10;
        has_data.validity_interval_index_code_bias_corrections = 10;
        has_data.validity_interval_index_phase_bias_corrections = 10;
        if (orbit_and_clock)
            {
                has_data.gnss_iod = {static_cast<uint16_t>(iod_ref)};
                has_data.delta_radial = {100};
                has_data.delta_in_track = {-50};
                has_data.delta_cross_track = {20};
                has_data.delta_clock_multiplier = {1};
                has_data.delta_clock_correction = {delta_clock_correction};
            }
        if (!code_biases.empty())
            {
                has_data.code_bias = {code_bias_row};
            }
        return has_data;
    }

    // MT1 with only a clock subset block for the satellite
    static Galileo_HAS_data make_clock_subset_message(int16_t delta_clock_correction)
    {
        Galileo_HAS_data has_data = make_message(false, {});
        has_data.header.mask_flag = false;
        has_data.header.clock_subset_flag = true;
        // The decoder resolves the reference IOD from the preceding orbit block.
        has_data.gnss_iod = {static_cast<uint16_t>(IOD)};
        has_data.Nsys_sub = 1;
        has_data.gnss_id_clock_subset = {HAS_MSG_GALILEO_SYSTEM};
        has_data.satellite_submask = {1};  // one bit, set, for the only satellite in the mask
        has_data.delta_clock_multiplier_clock_subset = {1};
        has_data.delta_clock_correction_clock_subset = {{delta_clock_correction}};
        return has_data;
    }

    // MT1 with orbit and clock corrections and no code biases, whose message
    // time is tow_s
    static Galileo_HAS_data make_message_at(uint32_t tow_s, int16_t delta_clock_correction)
    {
        Galileo_HAS_data has_data = make_message(true, {}, IOD_SET_ID, IOD, delta_clock_correction);
        has_data.tow = tow_s;
        has_data.header.toh = static_cast<uint16_t>(tow_s % 3600U);
        return has_data;
    }

    static Gnss_Synchro make_observation(const char* signal, uint32_t tow_s = MESSAGE_TOW + 1U)
    {
        Gnss_Synchro observation{};
        observation.System = 'E';
        observation.Signal[0] = signal[0];
        observation.Signal[1] = signal[1];
        observation.Signal[2] = '\0';
        observation.PRN = PRN;
        observation.interp_TOW_ms = static_cast<double>(tow_s) * 1000.0;
        observation.RX_time = static_cast<double>(tow_s);
        observation.Pseudorange_m = 2.2e7;
        observation.Flag_valid_pseudorange = true;
        return observation;
    }

    static std::map<int, Gnss_Synchro> make_observations(bool with_e1, bool with_e6, uint32_t tow_s = MESSAGE_TOW + 1U, bool with_e5a = false)
    {
        std::map<int, Gnss_Synchro> observations;
        if (with_e1)
            {
                observations[0] = make_observation("1B", tow_s);
            }
        if (with_e6)
            {
                observations[1] = make_observation("E6", tow_s);
            }
        if (with_e5a)
            {
                observations[2] = make_observation("5X", tow_s);
            }
        return observations;
    }

    static Ntrip_Rtcm_Snapshot make_ssr_snapshot()
    {
        ssr_t ssr{};
        ssr.t0[0] = ssr.t0[1] = ssr.t0[4] = gpst2time(WEEK + 1024U, MESSAGE_TOW);
        ssr.iod[0] = ssr.iod[1] = ssr.iod[4] = IOD_SET_ID;
        ssr.iode = IOD;
        ssr.vcbias[CODE_L1C - 1] = ssr.vcbias[CODE_L6C - 1] = 1;
        ssr.cbias[CODE_L1C - 1] = 0.5;
        ssr.cbias[CODE_L6C - 1] = -0.8;
        Ntrip_Rtcm_Snapshot snapshot;
        snapshot.ssr_corrections.emplace_back(satno(SYS_GAL, PRN), ssr);
        return snapshot;
    }
};


TEST_F(GalileoHasCorrectionsTest, SsrRequiresMatchingCodeBiasIssue)
{
    auto solver = make_solver();
    auto snapshot = make_ssr_snapshot();
    const auto observations = make_observations(true, true);
    solver->update_ssr_corrections(observations, snapshot);
    ASSERT_TRUE(solver->has_corrections_applied("Galileo", PRN));

    auto& ssr = snapshot.ssr_corrections.front().second;
    ssr.iod[0] = ssr.iod[1] = IOD_SET_ID + 1;
    solver->update_ssr_corrections(observations, snapshot);
    EXPECT_FALSE(solver->has_corrections_applied("Galileo", PRN));

    ssr.iod[4] = ssr.iod[0];
    solver->update_ssr_corrections(observations, snapshot);
    EXPECT_TRUE(solver->has_corrections_applied("Galileo", PRN));
}


TEST_F(GalileoHasCorrectionsTest, SsrRejectsExpiredCodeBias)
{
    auto solver = make_solver();
    auto snapshot = make_ssr_snapshot();
    const auto observations = make_observations(true, true);
    solver->update_ssr_corrections(observations, snapshot);
    ASSERT_TRUE(solver->has_corrections_applied("Galileo", PRN));

    auto& ssr = snapshot.ssr_corrections.front().second;
    ssr.t0[4] = timeadd(ssr.t0[4], -86400.0);
    solver->update_ssr_corrections(observations, snapshot);
    EXPECT_FALSE(solver->has_corrections_applied("Galileo", PRN));
}


TEST_F(GalileoHasCorrectionsTest, SsrRequiresCodeBiasEpoch)
{
    auto solver = make_solver();
    auto snapshot = make_ssr_snapshot();
    snapshot.ssr_corrections.front().second.t0[4] = gtime_t{};
    solver->update_ssr_corrections(make_observations(true, true), snapshot);
    EXPECT_FALSE(solver->has_corrections_applied("Galileo", PRN));
}


TEST_F(GalileoHasCorrectionsTest, SsrCodeBiasAgeBoundary)
{
    auto solver = make_solver();
    auto snapshot = make_ssr_snapshot();
    const auto observations = make_observations(true, true);
    auto& ssr = snapshot.ssr_corrections.front().second;
    const double max_age_s = Pvt_Conf().ntrip_max_correction_age_s;
    const gtime_t observation_time = timeadd(ssr.t0[0], 1.0);
    for (const double direction : {-1.0, 1.0})
        {
            ssr.t0[4] = timeadd(observation_time, direction * max_age_s);
            solver->update_ssr_corrections(observations, snapshot);
            EXPECT_TRUE(solver->has_corrections_applied("Galileo", PRN));

            ssr.t0[4] = timeadd(ssr.t0[4], direction * 0.001);
            solver->update_ssr_corrections(observations, snapshot);
            EXPECT_FALSE(solver->has_corrections_applied("Galileo", PRN));
        }
}


TEST_F(GalileoHasCorrectionsTest, SsrCodeBiasAgeAcrossWeekRollover)
{
    auto solver = make_solver();
    auto snapshot = make_ssr_snapshot();
    auto& ssr = snapshot.ssr_corrections.front().second;
    ssr.t0[0] = ssr.t0[1] = gpst2time(WEEK + 1024U, 604799.0);
    ssr.t0[4] = gpst2time(WEEK + 1024U, 604795.0);
    solver->update_ssr_corrections(make_observations(true, true, 0U), snapshot);
    ASSERT_TRUE(solver->has_corrections_applied("Galileo", PRN));

    ssr.t0[4] = timeadd(ssr.t0[4], -1.0);
    solver->update_ssr_corrections(make_observations(true, true, 0U), snapshot);
    EXPECT_FALSE(solver->has_corrections_applied("Galileo", PRN));
}


TEST_F(GalileoHasCorrectionsTest, SsrAcceptsValidZeroCodeBiases)
{
    auto solver = make_solver();
    auto snapshot = make_ssr_snapshot();
    auto& ssr = snapshot.ssr_corrections.front().second;
    ssr.cbias[CODE_L1C - 1] = ssr.cbias[CODE_L6C - 1] = 0.0;
    solver->update_ssr_corrections(make_observations(true, true), snapshot);
    EXPECT_TRUE(solver->has_corrections_applied("Galileo", PRN));
}


TEST_F(GalileoHasCorrectionsTest, AppliedWhenEverySignalHasACodeBias)
{
    auto solver = make_solver();
    solver->store_has_data(make_message(true, e1_e6_biases()));
    solver->update_has_corrections(make_observations(true, true));
    EXPECT_TRUE(solver->has_corrections_applied("Galileo", PRN));

    // the validity interval of 300 s ends at TOW 900
    solver->update_has_corrections(make_observations(true, true, MESSAGE_TOW + 300U));
    EXPECT_FALSE(solver->has_corrections_applied("Galileo", PRN));
}


TEST_F(GalileoHasCorrectionsTest, NotAppliedWhenASignalInUseLacksItsCodeBias)
{
    auto solver = make_solver();
    solver->store_has_data(make_message(true, e1_e6_biases(CODE_BIAS_UNAVAILABLE_RAW)));
    solver->update_has_corrections(make_observations(true, true));
    EXPECT_FALSE(solver->has_corrections_applied("Galileo", PRN));

    // E1 alone has everything it needs
    solver->update_has_corrections(make_observations(true, false));
    EXPECT_TRUE(solver->has_corrections_applied("Galileo", PRN));
}


TEST_F(GalileoHasCorrectionsTest, SignalsLeftOutOfThePvtDoNotNeedACodeBias)
{
    auto solver = make_solver(false);
    solver->store_has_data(make_message(true, e1_e6_biases(CODE_BIAS_UNAVAILABLE_RAW)));
    solver->update_has_corrections(make_observations(true, true));
    EXPECT_TRUE(solver->has_corrections_applied("Galileo", PRN));
}


TEST_F(GalileoHasCorrectionsTest, NotAppliedWithoutCodeBiases)
{
    auto solver = make_solver();
    solver->store_has_data(make_message(true, {}));
    solver->update_has_corrections(make_observations(true, false));
    EXPECT_FALSE(solver->has_corrections_applied("Galileo", PRN));
}


TEST_F(GalileoHasCorrectionsTest, ClockCorrectionsOfTheSupersededIodSetAreStored)
{
    // The orbit corrections and biases of a new IOD Set ID arrive before its
    // clock corrections. Meanwhile, a clock correction of the previous set
    // still goes with that set's orbit corrections and biases.
    constexpr uint8_t short_validity_index = 2;  // 15 s
    const auto dated = [](Galileo_HAS_data message, uint32_t tow) {
        message.tow = tow;
        message.header.toh = static_cast<uint16_t>(tow % 3600U);
        message.validity_interval_index_clock_fullset_corrections = short_validity_index;
        return message;
    };
    auto solver = make_solver();
    solver->store_has_data(dated(make_message(true, e1_e6_biases()), MESSAGE_TOW));

    Galileo_HAS_data new_set_orbit = dated(make_message(true, e1_e6_biases(), IOD_SET_ID + 1U), MESSAGE_TOW + 10U);
    new_set_orbit.header.clock_fullset_flag = false;
    solver->store_has_data(new_set_orbit);

    Galileo_HAS_data previous_set_clock = dated(make_message(true, {}, IOD_SET_ID), MESSAGE_TOW + 12U);
    previous_set_clock.header.orbit_correction_flag = false;
    solver->store_has_data(previous_set_clock);

    // the clock of the first message has expired (MESSAGE_TOW + 15): the one
    // of the previous set received at MESSAGE_TOW + 12 applies
    solver->update_has_corrections(make_observations(true, true, MESSAGE_TOW + 20U));
    EXPECT_TRUE(solver->has_corrections_applied("Galileo", PRN));

    // the clock corrections of the new set take over
    solver->store_has_data(dated(make_message(true, e1_e6_biases(), IOD_SET_ID + 1U), MESSAGE_TOW + 22U));
    solver->update_has_corrections(make_observations(true, true, MESSAGE_TOW + 30U));
    EXPECT_TRUE(solver->has_corrections_applied("Galileo", PRN));
}


TEST_F(GalileoHasCorrectionsTest, NotAppliedWhenTheCodeBiasesBelongToAnotherIodSet)
{
    // Biases for a new IOD Set ID arrive while the stored orbit and clock
    // corrections still belong to the previous one (HAS SIS ICD 1.0
    // Section 7.6.1): they cannot be combined
    auto solver = make_solver();
    solver->store_has_data(make_message(true, {}));
    solver->store_has_data(make_message(false, e1_e6_biases(), IOD_SET_ID + 1U));
    solver->update_has_corrections(make_observations(true, true));
    EXPECT_FALSE(solver->has_corrections_applied("Galileo", PRN));
}


TEST_F(GalileoHasCorrectionsTest, ClearingTheCorrectionsStopsTheirApplication)
{
    auto solver = make_solver();
    solver->store_has_data(make_message(true, e1_e6_biases()));
    solver->update_has_corrections(make_observations(true, true));
    ASSERT_TRUE(solver->has_corrections_applied("Galileo", PRN));

    solver->clear_has_corrections();
    EXPECT_FALSE(solver->has_corrections_applied("Galileo", PRN));
}


TEST_F(GalileoHasCorrectionsTest, MatchedAgainstTheInavIssueOfDataWithAnFnavPrimaryService)
{
    // E1 + E5a + E6 receiver: F/NAV is the primary service, but HAS corrects
    // I/NAV (HAS SIS ICD 1.0 Table 21). Different IODs make the match explicit.
    auto solver = make_solver_without_ephemeris(GAL_1B | GAL_E5a | GAL_E6);
    ASSERT_EQ(Galileo_Nav_Message_Type::FNAV, solver->galileo_nav_message_type_for_pvt());
    ASSERT_TRUE(solver->store_galileo_ephemeris(make_ephemeris(Galileo_Nav_Message_Type::FNAV, IOD + 1)));
    ASSERT_TRUE(solver->store_galileo_ephemeris(make_ephemeris(Galileo_Nav_Message_Type::INAV, IOD)));

    const Code_Biases biases = {{E1_C_INDEX, E1_CODE_BIAS_RAW}, {E5A_Q_INDEX, E5A_CODE_BIAS_RAW}, {E6_C_INDEX, E6_CODE_BIAS_RAW}};
    solver->store_has_data(make_message(true, biases));
    solver->update_has_corrections(make_observations(true, true, MESSAGE_TOW + 1U, true));
    ASSERT_TRUE(solver->has_corrections_applied("Galileo", PRN));

    // every signal of the satellite takes the corrected I/NAV ephemeris
    Galileo_Ephemeris selected;
    ASSERT_TRUE(solver->select_galileo_has_ephemeris(PRN, MESSAGE_TOW + 1U, selected));
    EXPECT_EQ(Galileo_Nav_Message_Type::INAV, selected.nav_message_type);
    EXPECT_EQ(static_cast<int32_t>(IOD), selected.IOD_ephemeris);
}


TEST_F(GalileoHasCorrectionsTest, NotAppliedWithoutAnInavEphemeris)
{
    auto solver = make_solver_without_ephemeris(GAL_1B | GAL_E5a | GAL_E6);
    ASSERT_EQ(Galileo_Nav_Message_Type::FNAV, solver->galileo_nav_message_type_for_pvt());
    ASSERT_TRUE(solver->store_galileo_ephemeris(make_ephemeris(Galileo_Nav_Message_Type::FNAV)));

    const Code_Biases biases = {{E1_C_INDEX, E1_CODE_BIAS_RAW}, {E5A_Q_INDEX, E5A_CODE_BIAS_RAW}, {E6_C_INDEX, E6_CODE_BIAS_RAW}};
    solver->store_has_data(make_message(true, biases));
    solver->update_has_corrections(make_observations(true, true, MESSAGE_TOW + 1U, true));
    EXPECT_FALSE(solver->has_corrections_applied("Galileo", PRN));
    Galileo_Ephemeris selected;
    EXPECT_FALSE(solver->select_galileo_has_ephemeris(PRN, MESSAGE_TOW + 1U, selected));
}


TEST_F(GalileoHasCorrectionsTest, DoNotUseHoldsWithoutEphemerisOrMatchingOrbit)
{
    // HAS SIS ICD 1.0 Table 31: the satellite shall not be used, even if the
    // receiver has no ephemeris for it, or its orbit corrections refer to an
    // issue of data the receiver does not have
    auto without_ephemeris = make_solver_without_ephemeris(GAL_1B | GAL_E6);
    without_ephemeris->store_has_data(make_message(true, {}, IOD_SET_ID, IOD, HAS_MSG_DELTA_CLOCK_DO_NOT_USE_RAW));
    EXPECT_TRUE(without_ephemeris->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 1U));
    EXPECT_TRUE(without_ephemeris->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 299U));
    EXPECT_FALSE(without_ephemeris->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 300U));
    EXPECT_FALSE(without_ephemeris->has_active_has_do_not_use("Galileo", PRN + 1, MESSAGE_TOW + 1U));
    EXPECT_FALSE(without_ephemeris->has_active_has_do_not_use("GPS", PRN, MESSAGE_TOW + 1U));

    auto other_iod = make_solver();
    other_iod->store_has_data(make_message(true, {}, IOD_SET_ID, IOD + 1, HAS_MSG_DELTA_CLOCK_DO_NOT_USE_RAW));
    EXPECT_TRUE(other_iod->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 1U));
}


TEST_F(GalileoHasCorrectionsTest, DoNotUseFromTheClockSubset)
{
    auto solver = make_solver_without_ephemeris(GAL_1B | GAL_E6);
    solver->store_has_data(make_clock_subset_message(HAS_MSG_DELTA_CLOCK_DO_NOT_USE_RAW));
    EXPECT_TRUE(solver->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 1U));
}


TEST_F(GalileoHasCorrectionsTest, ClockCorrectionLiftsDoNotUse)
{
    auto solver = make_solver();
    solver->store_has_data(make_message(true, {}, IOD_SET_ID, IOD, HAS_MSG_DELTA_CLOCK_DO_NOT_USE_RAW));
    ASSERT_TRUE(solver->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 1U));

    // "data not available" carries no information about the satellite
    solver->store_has_data(make_clock_subset_message(HAS_MSG_DELTA_CLOCK_UNAVAILABLE_RAW));
    EXPECT_TRUE(solver->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 1U));

    // a clock correction does, whatever the issue of data it refers to
    solver->store_has_data(make_message(true, {}, IOD_SET_ID, IOD + 1, 40));
    EXPECT_FALSE(solver->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 1U));
}


TEST_F(GalileoHasCorrectionsTest, RepeatedDoNotUseKeepsTheEarlierExclusion)
{
    auto solver = make_solver();
    solver->store_has_data(make_message_at(MESSAGE_TOW, HAS_MSG_DELTA_CLOCK_DO_NOT_USE_RAW));
    solver->store_has_data(make_message_at(MESSAGE_TOW + 10U, HAS_MSG_DELTA_CLOCK_DO_NOT_USE_RAW));

    // observations that precede the second message keep the flag of the first
    EXPECT_TRUE(solver->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 5U));
    solver->update_has_corrections(make_observations(true, true, MESSAGE_TOW + 5U));
    EXPECT_TRUE(solver->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 5U));
    EXPECT_TRUE(solver->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 15U));

    // past the second message's epoch, its own interval holds
    solver->update_has_corrections(make_observations(true, true, MESSAGE_TOW + 15U));
    EXPECT_TRUE(solver->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 305U));
    EXPECT_FALSE(solver->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 310U));
}


TEST_F(GalileoHasCorrectionsTest, ClockCorrectionLiftsDoNotUseFromItsMessageEpoch)
{
    auto solver = make_solver();
    solver->store_has_data(make_message_at(MESSAGE_TOW, HAS_MSG_DELTA_CLOCK_DO_NOT_USE_RAW));
    solver->store_has_data(make_message_at(MESSAGE_TOW + 10U, 40));

    // observations that precede the clock correction keep the flag
    EXPECT_TRUE(solver->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 5U));
    EXPECT_TRUE(solver->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 9U));
    EXPECT_FALSE(solver->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 10U));
    EXPECT_FALSE(solver->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 15U));

    // a later flag excludes the satellite again, from its own epoch
    solver->store_has_data(make_message_at(MESSAGE_TOW + 20U, HAS_MSG_DELTA_CLOCK_DO_NOT_USE_RAW));
    EXPECT_TRUE(solver->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 5U));
    EXPECT_FALSE(solver->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 15U));
    EXPECT_TRUE(solver->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 25U));
}


TEST(GalileoHasEphemerisConversionTest, CorrectionsApplyOnlyToInav)
{
    std::map<int, HAS_orbit_corrections> orbit_corrections;
    std::map<int, HAS_clock_corrections> clock_corrections;
    orbit_corrections[12].radial_m = 0.25F;
    orbit_corrections[12].iod = 77;
    clock_corrections[12].clock_correction_m = 0.1F;
    clock_corrections[12].iod = 77;

    for (const auto source : {Galileo_Nav_Message_Type::INAV, Galileo_Nav_Message_Type::FNAV, Galileo_Nav_Message_Type::Unknown})
        {
            SCOPED_TRACE(static_cast<int>(source));
            Galileo_Ephemeris ephemeris;
            ephemeris.PRN = 12;
            ephemeris.WN = 1400;
            ephemeris.IOD_ephemeris = 77;
            ephemeris.BGD_E1E5a = 2.0e-8;
            ephemeris.BGD_E1E5b = 4.0e-8;
            ephemeris.nav_message_type = source;
            const eph_t rtklib_ephemeris = eph_to_rtklib(ephemeris, orbit_corrections, clock_corrections);
            const bool is_inav = source == Galileo_Nav_Message_Type::INAV;
            EXPECT_EQ(is_inav, rtklib_ephemeris.apply_has_corrections);
            EXPECT_DOUBLE_EQ(is_inav ? 0.0 : ephemeris.BGD_E1E5a, rtklib_ephemeris.tgd[0]);
            EXPECT_DOUBLE_EQ(is_inav ? 0.0 : ephemeris.BGD_E1E5b, rtklib_ephemeris.tgd[1]);
        }
}


TEST(GalileoHasCarrierContinuityTest, ChangesInTheAppliedCorrectionsAreSlips)
{
    HAS_carrier_continuity continuity;
    const int sat = 12 + NSATGPS + NSATGLO;

    // the first observation of a signal starts its ambiguity
    EXPECT_FALSE(continuity.changed(sat, "1B", false, ""));
    EXPECT_FALSE(continuity.changed(sat, "1B", false, ""));

    // HAS orbit and clock corrections turned on, then off
    EXPECT_TRUE(continuity.changed(sat, "1B", true, ""));
    EXPECT_FALSE(continuity.changed(sat, "1B", true, ""));
    EXPECT_TRUE(continuity.changed(sat, "1B", false, ""));

    // a phase bias that appears, disappears and comes back
    EXPECT_TRUE(continuity.changed(sat, "1B", true, "E1-C"));
    EXPECT_FALSE(continuity.changed(sat, "1B", true, "E1-C"));
    EXPECT_TRUE(continuity.changed(sat, "1B", true, ""));
    EXPECT_TRUE(continuity.changed(sat, "1B", true, "E1-C"));
}


TEST(GalileoHasCarrierContinuityTest, SignalsAndSatellitesAreIndependent)
{
    HAS_carrier_continuity continuity;
    const int sat = 12 + NSATGPS + NSATGLO;
    EXPECT_FALSE(continuity.changed(sat, "1B", true, "E1-C"));
    EXPECT_FALSE(continuity.changed(sat, "E6", false, ""));
    EXPECT_FALSE(continuity.changed(sat + 1, "1B", false, ""));
    EXPECT_FALSE(continuity.changed(12, "1B", false, ""));  // GPS PRN 12

    EXPECT_TRUE(continuity.changed(sat, "E6", true, "E1-C"));
    EXPECT_FALSE(continuity.changed(sat, "1B", true, "E1-C"));
    EXPECT_FALSE(continuity.changed(sat + 1, "1B", false, ""));
    EXPECT_FALSE(continuity.changed(12, "1B", false, ""));
}


TEST(GalileoHasObservationCorrectionTest, LookupFollowsTheCandidateOrder)
{
    std::map<std::string, std::map<int, HAS_obs_corrections>> has_obs_corr;
    has_obs_corr["E6-C"][12].code_bias_m = -0.8F;
    has_obs_corr["E6-C"][12].code_bias_available = true;
    has_obs_corr["E1-C"][12].code_bias_m = 0.5F;
    has_obs_corr["E1-C"][12].code_bias_available = true;
    has_obs_corr["E1-B I/NAV OS"][12].code_bias_m = 0.4F;
    has_obs_corr["E1-B I/NAV OS"][12].code_bias_available = true;

    const HAS_obs_corrections* e1 = find_has_obs_correction(has_obs_corr, 'E', "1B", 12);
    ASSERT_NE(nullptr, e1);
    EXPECT_FLOAT_EQ(0.4F, e1->code_bias_m);
    const HAS_obs_corrections* e1_pilot = find_has_obs_correction(has_obs_corr, 'E', "1B", 12, true);
    ASSERT_NE(nullptr, e1_pilot);
    EXPECT_FLOAT_EQ(0.5F, e1_pilot->code_bias_m);
    const HAS_obs_corrections* e6 = find_has_obs_correction(has_obs_corr, 'E', "E6", 12);
    ASSERT_NE(nullptr, e6);
    EXPECT_FLOAT_EQ(-0.8F, e6->code_bias_m);
    EXPECT_EQ(nullptr, find_has_obs_correction(has_obs_corr, 'E', "7X", 12));
    EXPECT_EQ(nullptr, find_has_obs_correction(has_obs_corr, 'E', "1B", 11));
    EXPECT_EQ(nullptr, find_has_obs_correction(has_obs_corr, 'G', "1C", 12));
    EXPECT_TRUE(has_signal_candidates('C', "B1").empty());
    // The bias of another signal does not correct L2C
    has_obs_corr["L2 P"][12].code_bias_m = 1.0F;
    has_obs_corr["L2 P"][12].code_bias_available = true;
    EXPECT_EQ(nullptr, find_has_obs_correction(has_obs_corr, 'G', "2S", 12));

    // insert_obs_to_rtklib() applies the same correction
    Gnss_Synchro observation{};
    observation.System = 'E';
    observation.Signal[0] = 'E';
    observation.Signal[1] = '6';
    observation.tracking_component = Gnss_Synchro::TrackingComponent::Data;
    observation.PRN = 12;
    observation.Pseudorange_m = 2.2e7;
    observation.RX_time = 601.0;
    obsd_t rtklib_observation{};
    rtklib_observation = insert_obs_to_rtklib(rtklib_observation, observation, has_obs_corr, 1400, 2);
    EXPECT_NEAR(2.2e7 - 0.8, rtklib_observation.P[2], 1e-6);
}


TEST_F(GalileoHasCorrectionsTest, PhaseOnlyCandidateDoesNotHideAnAvailableCodeBias)
{
    auto solver = make_solver();
    auto message = make_message(true, {{0, CODE_BIAS_UNAVAILABLE_RAW}, {E1_C_INDEX, E1_CODE_BIAS_RAW}});
    message.header.phase_bias_flag = true;
    message.phase_bias = {{100, -1024}};
    message.phase_discontinuity_indicator = {{0, 0}};
    solver->store_has_data(message);
    solver->update_has_corrections(make_observations(true, false));
    EXPECT_TRUE(solver->has_corrections_applied("Galileo", PRN));
}


TEST(GalileoHasObservationCorrectionTest, ConversionSkipsPhaseOnlyCandidates)
{
    std::map<std::string, std::map<int, HAS_obs_corrections>> corrections;
    auto& phase_only = corrections["E1-B I/NAV OS"][12];
    phase_only.phase_bias_available = true;
    phase_only.phase_bias_cycle = 5.0F;
    auto& usable = corrections["E1-C"][12];
    usable.signal = "E1-C";
    usable.code_bias_available = true;
    usable.code_bias_m = 0.5F;
    usable.phase_bias_available = true;
    usable.phase_bias_cycle = 1.0F;

    Gnss_Synchro observation{};
    observation.System = 'E';
    observation.Signal[0] = '1';
    observation.Signal[1] = 'B';
    observation.tracking_component = Gnss_Synchro::TrackingComponent::Data;
    observation.PRN = 12;
    observation.Pseudorange_m = 2.2e7;
    observation.RX_time = 601.0;
    obsd_t raw{};
    const HAS_obs_corrections* applied = nullptr;
    const auto corrected = insert_obs_to_rtklib(raw, observation, corrections, 1400, 0, &applied, 0);
    ASSERT_EQ(&usable, applied);
    EXPECT_NEAR(2.2e7 + 0.5, corrected.P[0], 1e-6);
    EXPECT_DOUBLE_EQ(1.0, corrected.L[0]);

    corrections["E1-C"].clear();
    EXPECT_EQ(nullptr, find_has_obs_correction(corrections, 'E', "1B", 12));
}


TEST(GalileoHasCarrierContinuityTest, SwitchingTheSelectedPhaseBiasIsASlip)
{
    HAS_carrier_continuity continuity;
    const int sat = 12 + NSATGPS + NSATGLO;
    EXPECT_FALSE(continuity.changed(sat, "1B", true, "E1-C"));
    EXPECT_FALSE(continuity.changed(sat, "1B", true, "E1-C"));
    EXPECT_TRUE(continuity.changed(sat, "1B", true, "E1-B I/NAV OS"));
    EXPECT_FALSE(continuity.changed(sat, "1B", true, "E1-B I/NAV OS"));
    EXPECT_TRUE(continuity.changed(sat, "1B", true, "E1-C"));
    EXPECT_TRUE(continuity.changed(sat, "1B", true, ""));
    EXPECT_FALSE(continuity.changed(sat, "1B", true, ""));
}


TEST_F(GalileoHasCorrectionsTest, FutureCorrectionsAreRetainedUntilTheirStart)
{
    // The longest VI must also survive an observation just before t_MT1.
    for (const uint8_t vi_index : {uint8_t{1}, uint8_t{14}})
        {
            SCOPED_TRACE(static_cast<int>(vi_index));
            auto solver = make_solver();
            auto message = make_message(true, e1_e6_biases());
            message.validity_interval_index_orbit_corrections = vi_index;
            message.validity_interval_index_clock_fullset_corrections = vi_index;
            message.validity_interval_index_code_bias_corrections = vi_index;
            solver->store_has_data(message);
            solver->update_has_corrections(make_observations(true, true, MESSAGE_TOW - 1U));
            EXPECT_FALSE(solver->has_corrections_applied("Galileo", PRN));
            solver->update_has_corrections(make_observations(true, true, MESSAGE_TOW));
            EXPECT_TRUE(solver->has_corrections_applied("Galileo", PRN));
            const uint32_t end = MESSAGE_TOW + message.get_validity_interval_s(vi_index);
            solver->update_has_corrections(make_observations(true, true, end));
            EXPECT_FALSE(solver->has_corrections_applied("Galileo", PRN));
        }
}


TEST_F(GalileoHasCorrectionsTest, EachContentBlockUsesItsOwnStart)
{
    for (int block = 0; block < 4; ++block)
        {
            SCOPED_TRACE(block);
            auto solver = make_solver();
            solver->store_has_data(make_message(true, e1_e6_biases()));
            auto future = block == 2 ? make_clock_subset_message(40) : make_message(true, e1_e6_biases());
            future.header.orbit_correction_flag = block == 0;
            future.header.clock_fullset_flag = block == 1;
            future.header.clock_subset_flag = block == 2;
            future.header.code_bias_flag = block == 3;
            future.tow = MESSAGE_TOW + 10U;
            future.header.toh = MESSAGE_TOW + 10U;
            solver->store_has_data(future);
            solver->update_has_corrections(make_observations(true, true, MESSAGE_TOW + 9U));
            EXPECT_FALSE(solver->has_corrections_applied("Galileo", PRN));
            solver->update_has_corrections(make_observations(true, true, MESSAGE_TOW + 10U));
            EXPECT_TRUE(solver->has_corrections_applied("Galileo", PRN));
        }
}


TEST_F(GalileoHasCorrectionsTest, DoNotUseStartsAtItsMessageEpoch)
{
    for (const bool subset : {false, true})
        {
            SCOPED_TRACE(subset);
            auto solver = make_solver();
            const auto message = subset ? make_clock_subset_message(HAS_MSG_DELTA_CLOCK_DO_NOT_USE_RAW)
                                        : make_message(true, {}, IOD_SET_ID, IOD, HAS_MSG_DELTA_CLOCK_DO_NOT_USE_RAW);
            solver->store_has_data(message);
            solver->update_has_corrections(make_observations(true, false, MESSAGE_TOW - 1U));
            EXPECT_FALSE(solver->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW - 1U));
            solver->update_has_corrections(make_observations(true, false, MESSAGE_TOW));
            EXPECT_TRUE(solver->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW));
            EXPECT_FALSE(solver->has_active_has_do_not_use("Galileo", PRN, MESSAGE_TOW + 300U));
        }
}


// Single-point GPS L1 C/A solutions on synthetic, noise-free pseudoranges, with
// HAS corrections (all zero, so the pseudoranges stay consistent) for some
// satellites
class GalileoHasSolutionReportTest : public ::testing::Test
{
protected:
    // Used by value only: C++11 would need out-of-class definitions to bind
    // them to references
    static constexpr int WEEK = 2300;
    static constexpr uint32_t TOW = 100000U;

    GalileoHasSolutionReportTest()
        : d_sensor_source_configuration(&d_configuration),
          d_sensor_data_aggregator(d_sensor_source_configuration, {})
    {
        const double receiver_position_geodetic[3] = {45.0 * D2R, 8.0 * D2R, 100.0};
        pos2ecef(receiver_position_geodetic, d_receiver_position_ecef.data());
        // the first eight satellites above 15 deg, sorted by elevation
        for (unsigned int prn = 1; prn <= 32 && d_satellites.size() < 8; prn++)
            {
                const double elevation_rad = elevation(prn);
                if (elevation_rad > 15.0 * D2R)
                    {
                        d_satellites.push_back({prn, elevation_rad});
                    }
            }
        std::sort(d_satellites.begin(), d_satellites.end(),
            [](const Satellite& a, const Satellite& b) { return a.elevation_rad < b.elevation_rad; });
    }

    struct Satellite
    {
        unsigned int prn;
        double elevation_rad;
    };

    static Gps_Ephemeris make_gps_ephemeris(unsigned int prn)
    {
        Gps_Ephemeris ephemeris;
        ephemeris.PRN = prn;
        ephemeris.WN = WEEK;
        ephemeris.toe = static_cast<int32_t>(TOW);
        ephemeris.toc = static_cast<int32_t>(TOW);
        ephemeris.tow = static_cast<int32_t>(TOW);
        ephemeris.sqrtA = 5153.7954775;
        ephemeris.ecc = 0.0;
        ephemeris.i_0 = 0.94;
        ephemeris.M_0 = 0.75 * static_cast<double>(prn - 1);
        ephemeris.OMEGA_0 = 2.1 * static_cast<double>(prn - 1);
        ephemeris.OMEGAdot = -8.0e-9;
        ephemeris.IODE_SF2 = static_cast<int32_t>(prn);
        ephemeris.IODE_SF3 = static_cast<int32_t>(prn);
        ephemeris.IODC = static_cast<int32_t>(prn);
        ephemeris.TGD = 0.0;
        ephemeris.SV_health = 0;
        return ephemeris;
    }

    double elevation(unsigned int prn) const
    {
        const eph_t ephemeris = eph_to_rtklib(make_gps_ephemeris(prn), WEEK);
        double satellite_position[3]{};
        double satellite_clock_s = 0.0;
        double variance = 0.0;
        eph2pos(gpst2time(WEEK, TOW), &ephemeris, satellite_position, &satellite_clock_s, &variance);
        double line_of_sight[3]{};
        geodist(satellite_position, d_receiver_position_ecef.data(), line_of_sight);
        double receiver_position_geodetic[3]{};
        ecef2pos(d_receiver_position_ecef.data(), receiver_position_geodetic);
        double azel[2]{};
        satazel(receiver_position_geodetic, line_of_sight, azel);
        return azel[1];
    }

    std::unique_ptr<Rtklib_Solver> make_solver(double elevation_mask_rad, const std::string& dump_filename = {}) const
    {
        prcopt_t options = PRCOPT_DEFAULT;
        options.mode = PMODE_SINGLE;
        options.nf = 1;
        options.navsys = SYS_GPS;
        options.elmin = elevation_mask_rad;
        options.ionoopt = IONOOPT_OFF;
        options.tropopt = TROPOPT_OFF;
        rtk_t seed{};
        rtkinit(&seed, &options);
        const Pvt_Conf configuration;
        const bool dump = !dump_filename.empty();
        std::unique_ptr<Rtklib_Solver> solver(new Rtklib_Solver(
            seed, configuration, dump ? dump_filename : ".rtklib_solver_has_report_test.dat", GPS_1C, dump, dump));
        solver->set_ref_gps_week(WEEK);
        rtkfree(&seed);
        for (const auto& satellite : d_satellites)
            {
                solver->gps_ephemeris_map[static_cast<int>(satellite.prn)] = make_gps_ephemeris(satellite.prn);
            }
        return solver;
    }

    std::map<int, Gnss_Synchro> make_observations() const
    {
        const gtime_t reception_time = gpst2time(WEEK, TOW);
        std::map<int, Gnss_Synchro> observations;
        for (const auto& satellite : d_satellites)
            {
                const eph_t ephemeris = eph_to_rtklib(make_gps_ephemeris(satellite.prn), WEEK);
                double pseudorange_m = 22000000.0;
                for (int iteration = 0; iteration < 6; iteration++)
                    {
                        const gtime_t transmission_time = timeadd(reception_time, -pseudorange_m / SPEED_OF_LIGHT_M_S);
                        double satellite_position[3]{};
                        double satellite_clock_s = 0.0;
                        double variance = 0.0;
                        eph2pos(transmission_time, &ephemeris, satellite_position, &satellite_clock_s, &variance);
                        double line_of_sight[3]{};
                        pseudorange_m = geodist(satellite_position, d_receiver_position_ecef.data(), line_of_sight) -
                                        SPEED_OF_LIGHT_M_S * satellite_clock_s;
                    }
                Gnss_Synchro observation{};
                observation.System = 'G';
                std::memcpy(observation.Signal, "1C", 3);
                observation.tracking_component = Gnss_Synchro::TrackingComponent::Data;
                observation.PRN = satellite.prn;
                observation.Pseudorange_m = pseudorange_m;
                observation.CN0_dB_hz = 45.0;
                observation.RX_time = static_cast<double>(TOW);
                observation.interp_TOW_ms = static_cast<double>(TOW) * 1000.0;
                observation.Flag_valid_pseudorange = true;
                observations[static_cast<int>(satellite.prn)] = observation;
            }
        return observations;
    }

    // MT1 for GPS with zero orbit, clock and L1 C/A code bias corrections
    static Galileo_HAS_data make_gps_message(std::vector<unsigned int> prns)
    {
        std::sort(prns.begin(), prns.end());  // HAS lists satellites by increasing PRN
        const uint32_t message_tow = TOW - 5U;
        Galileo_HAS_data has_data;
        has_data.week = WEEK - 1024;
        has_data.tow = message_tow;
        has_data.has_status = 1;
        has_data.header = mt1_header{};
        has_data.header.toh = static_cast<uint16_t>(message_tow % 3600U);
        has_data.header.mask_id = 1;
        has_data.header.iod_set_id = 1;
        has_data.header.mask_flag = true;
        has_data.header.orbit_correction_flag = true;
        has_data.header.clock_fullset_flag = true;
        has_data.header.code_bias_flag = true;
        has_data.Nsys = 1;
        has_data.gnss_id_mask = {HAS_MSG_GPS_SYSTEM};
        uint64_t satellite_mask = 0;
        for (const unsigned int prn : prns)
            {
                satellite_mask |= uint64_t{1} << (HAS_MSG_NUMBER_SATELLITE_IDS - static_cast<int32_t>(prn));  // MSB is PRN 1
                has_data.gnss_iod.push_back(static_cast<uint16_t>(prn));
                has_data.delta_radial.push_back(0);
                has_data.delta_in_track.push_back(0);
                has_data.delta_cross_track.push_back(0);
                has_data.delta_clock_correction.push_back(0);
                has_data.code_bias.push_back({0});
            }
        has_data.satellite_mask = {satellite_mask};
        has_data.signal_mask = {static_cast<uint16_t>(1U << (HAS_MSG_NUMBER_SIGNAL_MASKS - 1))};  // L1 C/A, Signal Index 0
        has_data.cell_mask_availability_flag = {false};
        has_data.nav_message = {0};
        has_data.delta_clock_multiplier = {1};
        has_data.validity_interval_index_orbit_corrections = 10;
        has_data.validity_interval_index_clock_fullset_corrections = 10;
        has_data.validity_interval_index_code_bias_corrections = 10;
        return has_data;
    }

    bool solve(Rtklib_Solver& solver, const std::vector<unsigned int>& has_prns, bool dump_this_epoch = false)
    {
        const std::map<int, Gnss_Synchro> observations = make_observations();
        solver.store_has_data(make_gps_message(has_prns));
        solver.update_has_corrections(observations);
        return solver.get_PVT(observations, 1.0, d_sensor_data_aggregator, dump_this_epoch);
    }

    static const Monitor_Pvt::TrackedSatelliteInfo* find_tracked(const Monitor_Pvt& monitor, unsigned int prn)
    {
        for (const auto& satellite : monitor.tracked_satellites)
            {
                if (satellite.system == 'G' && satellite.prn == prn)
                    {
                        return &satellite;
                    }
            }
        return nullptr;
    }

    std::array<double, 3> d_receiver_position_ecef{};
    std::vector<Satellite> d_satellites;

private:
    InMemoryConfiguration d_configuration;
    SensorDataSourceConfiguration d_sensor_source_configuration;
    SensorDataAggregator d_sensor_data_aggregator;
};


TEST_F(GalileoHasSolutionReportTest, ReportsTheSatellitesWithHasCorrections)
{
    ASSERT_EQ(8U, d_satellites.size());
    auto solver = make_solver(-90.0 * D2R);
    const std::vector<unsigned int> has_prns = {d_satellites[2].prn, d_satellites[4].prn, d_satellites[6].prn};
    ASSERT_TRUE(solve(*solver, has_prns));

    EXPECT_EQ(3, solver->get_num_has_corrected_satellites());
    const Monitor_Pvt monitor = solver->get_monitor_pvt();
    EXPECT_EQ(1U, monitor.galhas_status);
    for (const auto& satellite : d_satellites)
        {
            SCOPED_TRACE(satellite.prn);
            const Monitor_Pvt::TrackedSatelliteInfo* tracked = find_tracked(monitor, satellite.prn);
            ASSERT_NE(nullptr, tracked);
            EXPECT_TRUE(tracked->used);
            const bool has_corrected = std::find(has_prns.cbegin(), has_prns.cend(), satellite.prn) != has_prns.cend();
            EXPECT_EQ(has_corrected, tracked->has_corrected);
        }
    // a mixed solution is not on precise orbits and clocks
    EXPECT_FALSE(Rtklib_Solver::uses_precise_orbits_and_clocks(false, monitor.valid_sats, solver->get_num_has_corrected_satellites()));
}


TEST_F(GalileoHasSolutionReportTest, AllSatellitesWithHasCorrections)
{
    ASSERT_EQ(8U, d_satellites.size());
    auto solver = make_solver(-90.0 * D2R);
    std::vector<unsigned int> has_prns;
    for (const auto& satellite : d_satellites)
        {
            has_prns.push_back(satellite.prn);
        }
    ASSERT_TRUE(solve(*solver, has_prns));
    EXPECT_EQ(8, solver->get_num_has_corrected_satellites());
    EXPECT_EQ(1U, solver->get_monitor_pvt().galhas_status);
}


TEST_F(GalileoHasSolutionReportTest, StatusIsOffWhenNoHasSatelliteEntersTheSolution)
{
    ASSERT_EQ(8U, d_satellites.size());
    // only the lowest satellite gets HAS corrections, and the elevation mask
    // leaves it out of the solution
    const double mask_rad = 0.5 * (d_satellites[0].elevation_rad + d_satellites[1].elevation_rad);
    auto solver = make_solver(mask_rad);
    const unsigned int masked_prn = d_satellites[0].prn;
    ASSERT_TRUE(solve(*solver, {masked_prn}));
    ASSERT_TRUE(solver->has_corrections_applied("GPS", static_cast<int>(masked_prn)));

    EXPECT_EQ(0, solver->get_num_has_corrected_satellites());
    const Monitor_Pvt monitor = solver->get_monitor_pvt();
    EXPECT_EQ(0U, monitor.galhas_status);
    const Monitor_Pvt::TrackedSatelliteInfo* tracked = find_tracked(monitor, masked_prn);
    ASSERT_NE(nullptr, tracked);
    EXPECT_FALSE(tracked->used);
    EXPECT_TRUE(tracked->has_corrected);
}


TEST_F(GalileoHasSolutionReportTest, DumpRecordsTheNumberOfHasSatellites)
{
    ASSERT_EQ(8U, d_satellites.size());
    const std::string dump_filename("./rtklib_solver_has_report_dump_test.dat");
    const std::string mat_filename("./rtklib_solver_has_report_dump_test.mat");
    // 21 doubles, 2 uint32, 3 uint8 and 2 floats, then the new uint8
    constexpr std::streamoff record_bytes = 21 * 8 + 2 * 4 + 3 * 1 + 2 * 4 + 1;
    // enough records for a wrong record size to change the number of epochs
    // that save_matfile() counts
    constexpr int records = 200;
    {
        auto solver = make_solver(-90.0 * D2R, dump_filename);
        ASSERT_TRUE(solve(*solver, {d_satellites[0].prn, d_satellites[1].prn, d_satellites[2].prn}, true));
        for (int record = 1; record < records; record++)
            {
                // the first message keeps its three satellites corrected
                // (validity of 300 s), and this one adds a fourth
                ASSERT_TRUE(solve(*solver, {d_satellites[3].prn}, true));
            }
    }

    std::ifstream dump(dump_filename, std::ios::binary | std::ios::ate);
    ASSERT_TRUE(dump.is_open());
    ASSERT_EQ(records * record_bytes, static_cast<std::streamoff>(dump.tellg()));
    const uint8_t expected[2] = {3, 4};
    for (int record = 0; record < 2; record++)
        {
            dump.seekg(record * record_bytes + record_bytes - 1);
            char count = 0;
            dump.read(&count, 1);
            EXPECT_EQ(expected[record], static_cast<uint8_t>(count));
        }
    dump.close();

    mat_t* matfile = Mat_Open(mat_filename.c_str(), MAT_ACC_RDONLY);
    ASSERT_NE(nullptr, matfile);
    for (const char* name : {"has_corrected_sats", "valid_sats"})
        {
            SCOPED_TRACE(name);
            matvar_t* variable = Mat_VarRead(matfile, name);
            ASSERT_NE(nullptr, variable);
            ASSERT_EQ(static_cast<size_t>(records), variable->nbytes / variable->data_size);
            const auto* values = static_cast<const uint8_t*>(variable->data);
            const bool valid_sats = std::string(name) == "valid_sats";
            EXPECT_EQ(valid_sats ? 8 : expected[0], values[0]);
            EXPECT_EQ(valid_sats ? 8 : expected[1], values[records - 1]);
            Mat_VarFree(variable);
        }
    Mat_Close(matfile);
    std::remove(dump_filename.c_str());
    std::remove(mat_filename.c_str());
}


TEST(GalileoHasPreciseSolutionTest, PppNeedsHasCorrectionsForEverySatellite)
{
    EXPECT_TRUE(Rtklib_Solver::uses_precise_orbits_and_clocks(true, 6, 0));
    EXPECT_TRUE(Rtklib_Solver::uses_precise_orbits_and_clocks(false, 6, 6));
    EXPECT_FALSE(Rtklib_Solver::uses_precise_orbits_and_clocks(false, 6, 5));
    EXPECT_FALSE(Rtklib_Solver::uses_precise_orbits_and_clocks(false, 0, 0));
}
