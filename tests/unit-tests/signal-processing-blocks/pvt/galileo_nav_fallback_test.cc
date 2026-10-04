/*!
 * \file galileo_nav_fallback_test.cc
 * \brief Unit tests for Galileo navigation-service fallback in PVT
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

#include "pvt_conf.h"
#include "rtklib_conversions.h"
#include "rtklib_pntpos.h"
#include "rtklib_solver.h"
#include "signal_flag.h"
#include <gtest/gtest.h>
#include <memory>

class GalileoNavFallbackTest : public ::testing::Test
{
protected:
    std::unique_ptr<Rtklib_Solver> make_solver(Galileo_Nav_Message_Type primary_source,
        bool use_unhealthy_sats = false)
    {
        const rtk_t rtk{};
        Pvt_Conf conf;
        conf.use_e6_for_pvt = true;
        conf.use_unhealthy_sats = use_unhealthy_sats;
        const uint32_t signal_flags = GPS_1C | GAL_1B | GAL_E6 |
                                      (primary_source == Galileo_Nav_Message_Type::FNAV ? GAL_E5a : GAL_E5b);
        auto solver = std::unique_ptr<Rtklib_Solver>(new Rtklib_Solver(
            rtk, conf, ".rtklib_solver_galileo_fallback_test.dat", signal_flags, false, false));
        EXPECT_EQ(primary_source, solver->galileo_nav_message_type_for_pvt());
        return solver;
    }

    Galileo_Ephemeris make_ephemeris(Galileo_Nav_Message_Type source, double toe = 600.0)
    {
        Galileo_Ephemeris ephemeris;
        ephemeris.PRN = 12U;
        ephemeris.WN = 1400;
        ephemeris.tow = static_cast<int>(toe);
        ephemeris.toe = toe;
        ephemeris.af0 = source == Galileo_Nav_Message_Type::INAV ? 1.0e-4 : 2.0e-4;
        ephemeris.BGD_E1E5a = 2.0e-8;
        ephemeris.BGD_E1E5b = 4.0e-8;
        ephemeris.nav_message_type = source;
        return ephemeris;
    }

    Galileo_Nav_Message_Type alternate_source(Galileo_Nav_Message_Type primary_source)
    {
        return primary_source == Galileo_Nav_Message_Type::FNAV ? Galileo_Nav_Message_Type::INAV : Galileo_Nav_Message_Type::FNAV;
    }

    const char* primary_e5_signal(Galileo_Nav_Message_Type primary_source)
    {
        return primary_source == Galileo_Nav_Message_Type::FNAV ? "5X" : "7X";
    }

    void expect_e1_clock_source(const Galileo_Ephemeris& ephemeris, Galileo_Nav_Message_Type source)
    {
        EXPECT_EQ(source, ephemeris.nav_message_type);
        eph_t rtklib_ephemeris = eph_to_rtklib(ephemeris);
        nav_t nav{};
        nav.n = 1;
        nav.eph = &rtklib_ephemeris;
        const int bgd_index = galileo_bgd_index(CODE_L1C, rtklib_ephemeris.sat, &nav);
        ASSERT_EQ(source == Galileo_Nav_Message_Type::FNAV ? 0 : 1, bgd_index);
        EXPECT_DOUBLE_EQ(source == Galileo_Nav_Message_Type::FNAV ? ephemeris.BGD_E1E5a : ephemeris.BGD_E1E5b,
            rtklib_ephemeris.tgd[bgd_index]);
    }
};


TEST_F(GalileoNavFallbackTest, AlternateFullEphemerisIsLimitedToE1)
{
    for (const auto primary_source : {Galileo_Nav_Message_Type::FNAV, Galileo_Nav_Message_Type::INAV})
        {
            SCOPED_TRACE(static_cast<int>(primary_source));
            for (const bool compatibility_only : {false, true})
                {
                    SCOPED_TRACE(compatibility_only);
                    for (const bool use_unhealthy_sats : {false, true})
                        {
                            SCOPED_TRACE(use_unhealthy_sats);
                            auto solver = make_solver(primary_source, use_unhealthy_sats);
                            const auto alternate = make_ephemeris(alternate_source(primary_source));
                            if (compatibility_only)
                                {
                                    solver->galileo_ephemeris_map[12] = alternate;
                                }
                            else
                                {
                                    ASSERT_TRUE(solver->store_galileo_ephemeris(alternate));
                                }

                            const Rtklib_Solver& selector = *solver;
                            Galileo_Ephemeris selected;
                            bool from_reduced_ced = true;
                            ASSERT_TRUE(selector.select_galileo_ephemeris(12U, "1B", 600U, selected, from_reduced_ced));
                            EXPECT_FALSE(from_reduced_ced);
                            expect_e1_clock_source(selected, alternate.nav_message_type);
                            EXPECT_DOUBLE_EQ(alternate.af0, selected.af0);

                            for (const char* signal : {primary_e5_signal(primary_source), "E6"})
                                {
                                    SCOPED_TRACE(signal);
                                    ASSERT_TRUE(selector.is_galileo_signal_used_in_pvt(signal));
                                    EXPECT_FALSE(selector.select_galileo_ephemeris(12U, signal, 600U, selected, from_reduced_ced));
                                }
                        }
                }
        }
}


TEST_F(GalileoNavFallbackTest, PromotesToPrimaryAndFallsBackAfterExpiry)
{
    for (const auto primary_source : {Galileo_Nav_Message_Type::FNAV, Galileo_Nav_Message_Type::INAV})
        {
            SCOPED_TRACE(static_cast<int>(primary_source));
            auto solver = make_solver(primary_source);
            const auto alternate = alternate_source(primary_source);
            ASSERT_TRUE(solver->store_galileo_ephemeris(make_ephemeris(alternate)));

            Galileo_Ephemeris selected;
            bool from_reduced_ced = true;
            ASSERT_TRUE(solver->select_galileo_ephemeris(12U, "1B", 600U, selected, from_reduced_ced));
            expect_e1_clock_source(selected, alternate);

            ASSERT_TRUE(solver->store_galileo_ephemeris(make_ephemeris(primary_source)));
            for (const char* signal : {"1B", primary_e5_signal(primary_source), "E6"})
                {
                    SCOPED_TRACE(signal);
                    ASSERT_TRUE(solver->select_galileo_ephemeris(12U, signal, 600U, selected, from_reduced_ced));
                    EXPECT_FALSE(from_reduced_ced);
                    expect_e1_clock_source(selected, primary_source);
                }

            ASSERT_TRUE(solver->store_galileo_ephemeris(make_ephemeris(alternate, 12000.0)));
            ASSERT_TRUE(solver->select_galileo_ephemeris(12U, "1B", 12000U, selected, from_reduced_ced));
            EXPECT_FALSE(from_reduced_ced);
            expect_e1_clock_source(selected, alternate);
            EXPECT_FALSE(solver->select_galileo_ephemeris(12U, primary_e5_signal(primary_source), 12000U, selected, from_reduced_ced));
            EXPECT_FALSE(solver->select_galileo_ephemeris(12U, "E6", 12000U, selected, from_reduced_ced));
            EXPECT_FALSE(solver->select_galileo_ephemeris(12U, "1B", 24000U, selected, from_reduced_ced));
        }
}


TEST_F(GalileoNavFallbackTest, CompatibilityPrimaryPrecedesAlternateStore)
{
    for (const auto primary_source : {Galileo_Nav_Message_Type::FNAV, Galileo_Nav_Message_Type::INAV})
        {
            SCOPED_TRACE(static_cast<int>(primary_source));
            for (const bool legacy_provenance : {false, true})
                {
                    SCOPED_TRACE(legacy_provenance);
                    auto solver = make_solver(primary_source);
                    ASSERT_TRUE(solver->store_galileo_ephemeris(make_ephemeris(alternate_source(primary_source))));
                    auto primary = make_ephemeris(primary_source);
                    if (legacy_provenance)
                        {
                            primary.nav_message_type = Galileo_Nav_Message_Type::Unknown;
                        }
                    solver->galileo_ephemeris_map[12] = primary;

                    Galileo_Ephemeris selected;
                    bool from_reduced_ced = true;
                    for (const char* signal : {"1B", primary_e5_signal(primary_source), "E6"})
                        {
                            SCOPED_TRACE(signal);
                            ASSERT_TRUE(solver->select_galileo_ephemeris(12U, signal, 600U, selected, from_reduced_ced));
                            EXPECT_FALSE(from_reduced_ced);
                            expect_e1_clock_source(selected, primary_source);
                            EXPECT_DOUBLE_EQ(primary.af0, selected.af0);
                        }
                }
        }
}


TEST_F(GalileoNavFallbackTest, ReducedCedRetainsE1AndE5bScope)
{
    for (const auto primary_source : {Galileo_Nav_Message_Type::FNAV, Galileo_Nav_Message_Type::INAV})
        {
            SCOPED_TRACE(static_cast<int>(primary_source));
            auto solver = make_solver(primary_source, true);
            Galileo_Reduced_CED reduced_ced;
            reduced_ced.PRN = 12U;
            reduced_ced.WN = 1400U;
            reduced_ced.TOTRedCED = 15U;
            solver->galileo_reduced_ced_map[12] = reduced_ced;

            Galileo_Ephemeris selected;
            bool from_reduced_ced = false;
            ASSERT_TRUE(solver->select_galileo_ephemeris(12U, "1B", 600U, selected, from_reduced_ced));
            EXPECT_TRUE(from_reduced_ced);
            EXPECT_EQ(Galileo_Nav_Message_Type::INAV, selected.nav_message_type);
            EXPECT_EQ(-1, selected.IOD_nav);
            EXPECT_EQ(primary_source == Galileo_Nav_Message_Type::INAV,
                solver->select_galileo_ephemeris(12U, "7X", 600U, selected, from_reduced_ced));
            EXPECT_FALSE(solver->select_galileo_ephemeris(12U, "5X", 600U, selected, from_reduced_ced));
            EXPECT_FALSE(solver->select_galileo_ephemeris(12U, "E6", 600U, selected, from_reduced_ced));
            EXPECT_FALSE(solver->select_galileo_ephemeris(12U, "1B", 601U, selected, from_reduced_ced));
        }
}
