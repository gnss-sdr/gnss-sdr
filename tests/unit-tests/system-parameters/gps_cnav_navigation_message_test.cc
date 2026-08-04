/*!
 * \file gps_cnav_navigation_message_test.cc
 * \brief  Tests for GPS CNAV navigation message decoding
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

#include "GPS_CNAV.h"
#include "gps_cnav_navigation_message.h"
#include "rtklib_conversions.h"
#include <bitset>
#include <cstdint>
#include <utility>
#include <vector>

static void set_gps_cnav_unsigned_field(std::bitset<GPS_CNAV_DATA_PAGE_BITS>& bits,
    const std::vector<std::pair<int32_t, int32_t>>& parameter,
    uint64_t value)
{
    int32_t pending_bits = 0;
    for (const auto& p : parameter)
        {
            pending_bits += p.second;
        }

    for (const auto& p : parameter)
        {
            for (int32_t j = 0; j < p.second; ++j)
                {
                    --pending_bits;
                    bits[GPS_CNAV_DATA_PAGE_BITS - p.first - j] = ((value >> pending_bits) & 1ULL) != 0ULL;
                }
        }
}


static void set_gps_cnav_signed_field(std::bitset<GPS_CNAV_DATA_PAGE_BITS>& bits,
    const std::vector<std::pair<int32_t, int32_t>>& parameter,
    int64_t value)
{
    int32_t field_bits = 0;
    for (const auto& p : parameter)
        {
            field_bits += p.second;
        }
    const uint64_t mask = (1ULL << field_bits) - 1ULL;
    set_gps_cnav_unsigned_field(bits, parameter, static_cast<uint64_t>(value) & mask);
}


static std::bitset<GPS_CNAV_DATA_PAGE_BITS> gps_cnav_page(uint64_t message_type, uint64_t prn = 1)
{
    std::bitset<GPS_CNAV_DATA_PAGE_BITS> page;

    set_gps_cnav_unsigned_field(page, CNAV_PRN, prn);
    set_gps_cnav_unsigned_field(page, CNAV_MSG_TYPE, message_type);
    set_gps_cnav_unsigned_field(page, CNAV_TOW, 1);

    return page;
}


TEST(GpsCnavNavigationMessageTest, EarthOrientationPageDecodesAllParameters)
{
    Gps_CNAV_Navigation_Message nav_message;

    auto mt32 = gps_cnav_page(32, 7);
    set_gps_cnav_unsigned_field(mt32, CNAV_TOW, 42208);
    set_gps_cnav_unsigned_field(mt32, CNAV_T_EOP, 31488);
    set_gps_cnav_signed_field(mt32, CNAV_PM_X, 217144);
    set_gps_cnav_signed_field(mt32, CNAV_PM_X_DOT, -562);
    set_gps_cnav_signed_field(mt32, CNAV_PM_Y, 355725);
    set_gps_cnav_signed_field(mt32, CNAV_PM_Y_DOT, -2938);
    set_gps_cnav_signed_field(mt32, CNAV_DELTA_UT1_GPS, -1476220);
    set_gps_cnav_signed_field(mt32, CNAV_DELTA_UT1_GPS_DOT, -2676);
    nav_message.decode_page(mt32);

    ASSERT_TRUE(nav_message.have_new_eop());
    EXPECT_FALSE(nav_message.have_new_eop());
    const auto eop = nav_message.get_eop();
    EXPECT_TRUE(eop.valid);
    EXPECT_EQ(7U, eop.PRN);
    EXPECT_DOUBLE_EQ(42208.0 * CNAV_TOW_LSB, eop.tow);
    EXPECT_DOUBLE_EQ(31488.0 * CNAV_T_EOP_LSB, eop.t_eop);
    EXPECT_DOUBLE_EQ(217144.0 * CNAV_PM_X_LSB, eop.pm_x);
    EXPECT_DOUBLE_EQ(-562.0 * CNAV_PM_X_DOT_LSB, eop.pm_x_dot);
    EXPECT_DOUBLE_EQ(355725.0 * CNAV_PM_Y_LSB, eop.pm_y);
    EXPECT_DOUBLE_EQ(-2938.0 * CNAV_PM_Y_DOT_LSB, eop.pm_y_dot);
    EXPECT_DOUBLE_EQ(-1476220.0 * CNAV_DELTA_UT1_GPS_LSB, eop.delta_ut1_gps);
    EXPECT_DOUBLE_EQ(-2676.0 * CNAV_DELTA_UT1_GPS_DOT_LSB, eop.delta_ut1_gps_dot);
}


TEST(GpsCnavNavigationMessageTest, EphemerisRequiresMatchingClockPage)
{
    Gps_CNAV_Navigation_Message nav_message;

    auto mt10 = gps_cnav_page(10);
    set_gps_cnav_unsigned_field(mt10, CNAV_TOP1, 100);
    set_gps_cnav_unsigned_field(mt10, CNAV_TOE1, 200);
    nav_message.decode_page(mt10);
    EXPECT_FALSE(nav_message.have_new_ephemeris());

    auto mt11 = gps_cnav_page(11);
    set_gps_cnav_unsigned_field(mt11, CNAV_TOE2, 200);
    nav_message.decode_page(mt11);
    EXPECT_FALSE(nav_message.have_new_ephemeris());

    auto mt30_bad_toc = gps_cnav_page(30);
    set_gps_cnav_unsigned_field(mt30_bad_toc, CNAV_TOP2, 100);
    set_gps_cnav_unsigned_field(mt30_bad_toc, CNAV_TOC, 201);
    nav_message.decode_page(mt30_bad_toc);
    EXPECT_FALSE(nav_message.have_new_ephemeris());

    auto mt30_matching = gps_cnav_page(30);
    set_gps_cnav_unsigned_field(mt30_matching, CNAV_TOP2, 100);
    set_gps_cnav_unsigned_field(mt30_matching, CNAV_TOC, 200);
    nav_message.decode_page(mt30_matching);
    EXPECT_TRUE(nav_message.have_new_ephemeris());
    EXPECT_EQ(200 * CNAV_TOE1_LSB, nav_message.get_ephemeris().toe);
    EXPECT_FALSE(nav_message.have_new_ephemeris());
}


TEST(GpsCnavNavigationMessageTest, ClockAndUtcPageUsesClockTopField)
{
    Gps_CNAV_Navigation_Message nav_message;

    auto mt10 = gps_cnav_page(10);
    set_gps_cnav_unsigned_field(mt10, CNAV_TOP1, 40);
    set_gps_cnav_unsigned_field(mt10, CNAV_TOE1, 80);
    nav_message.decode_page(mt10);

    auto mt11 = gps_cnav_page(11);
    set_gps_cnav_unsigned_field(mt11, CNAV_TOE2, 80);
    nav_message.decode_page(mt11);

    auto mt33 = gps_cnav_page(33);
    set_gps_cnav_unsigned_field(mt33, CNAV_TOP2, 40);
    set_gps_cnav_unsigned_field(mt33, CNAV_TOC, 80);
    nav_message.decode_page(mt33);

    EXPECT_TRUE(nav_message.have_new_ephemeris());
    EXPECT_TRUE(nav_message.have_new_utc_model());
}


TEST(GpsCnavNavigationMessageTest, CnavAccuracyFieldsReachRtklibEphemeris)
{
    Gps_CNAV_Navigation_Message nav_message;

    auto mt10 = gps_cnav_page(10);
    set_gps_cnav_unsigned_field(mt10, CNAV_TOP1, 120);
    set_gps_cnav_unsigned_field(mt10, CNAV_TOE1, 240);
    set_gps_cnav_unsigned_field(mt10, CNAV_URA, 1);
    nav_message.decode_page(mt10);

    auto mt11 = gps_cnav_page(11);
    set_gps_cnav_unsigned_field(mt11, CNAV_TOE2, 240);
    nav_message.decode_page(mt11);

    auto mt30 = gps_cnav_page(30);
    set_gps_cnav_unsigned_field(mt30, CNAV_TOP2, 120);
    set_gps_cnav_unsigned_field(mt30, CNAV_TOC, 240);
    set_gps_cnav_unsigned_field(mt30, CNAV_URA_NED0, 1);
    set_gps_cnav_unsigned_field(mt30, CNAV_URA_NED1, 3);
    set_gps_cnav_unsigned_field(mt30, CNAV_URA_NED2, 5);
    set_gps_cnav_unsigned_field(mt30, CNAV_WNOP, 42);
    nav_message.decode_page(mt30);

    ASSERT_TRUE(nav_message.have_new_ephemeris());
    const auto ephemeris = nav_message.get_ephemeris();
    EXPECT_EQ(1, ephemeris.URAED);
    EXPECT_EQ(1, ephemeris.URANED0);
    EXPECT_EQ(3, ephemeris.URANED1);
    EXPECT_EQ(5, ephemeris.URANED2);
    EXPECT_EQ(42, ephemeris.WNop);

    const auto rtklib_ephemeris = eph_to_rtklib(ephemeris);
    EXPECT_EQ(1, rtklib_ephemeris.cnav_ura_valid);
    EXPECT_EQ(1, rtklib_ephemeris.cnav_uraed);
    EXPECT_EQ(1, rtklib_ephemeris.cnav_uraned0);
    EXPECT_EQ(3, rtklib_ephemeris.cnav_uraned1);
    EXPECT_EQ(5, rtklib_ephemeris.cnav_uraned2);
    EXPECT_EQ(120 * CNAV_TOP1_LSB, rtklib_ephemeris.cnav_top);
    EXPECT_EQ(42, rtklib_ephemeris.cnav_wnop);
    EXPECT_EQ(2, rtklib_ephemeris.sva);
}


TEST(GpsCnavNavigationMessageTest, QzssCnavAccuracyFieldsUseSharedLayout)
{
    Gps_CNAV_Navigation_Message nav_message(CnavSystem::QZSS);

    auto mt10 = gps_cnav_page(10, 5);
    set_gps_cnav_unsigned_field(mt10, CNAV_TOP1, 120);
    set_gps_cnav_unsigned_field(mt10, CNAV_TOE1, 240);
    set_gps_cnav_unsigned_field(mt10, CNAV_URA, 31);
    nav_message.decode_page(mt10);

    auto mt11 = gps_cnav_page(11, 5);
    set_gps_cnav_unsigned_field(mt11, CNAV_TOE2, 240);
    nav_message.decode_page(mt11);

    auto mt30 = gps_cnav_page(30, 5);
    set_gps_cnav_unsigned_field(mt30, CNAV_TOP2, 120);
    set_gps_cnav_unsigned_field(mt30, CNAV_TOC, 240);
    set_gps_cnav_unsigned_field(mt30, CNAV_URA_NED0, 31);
    set_gps_cnav_unsigned_field(mt30, CNAV_URA_NED1, 3);
    set_gps_cnav_unsigned_field(mt30, CNAV_URA_NED2, 5);
    set_gps_cnav_unsigned_field(mt30, CNAV_WNOP, 42);
    nav_message.decode_page(mt30);

    ASSERT_TRUE(nav_message.have_new_ephemeris());
    const auto ephemeris = nav_message.get_ephemeris();
    EXPECT_EQ(197, ephemeris.PRN);
    EXPECT_EQ(-1, ephemeris.URAED);
    EXPECT_EQ(-1, ephemeris.URANED0);
    EXPECT_EQ(3, ephemeris.URANED1);
    EXPECT_EQ(5, ephemeris.URANED2);
    EXPECT_EQ(42, ephemeris.WNop);

    const auto rtklib_ephemeris = eph_to_rtklib(ephemeris);
    EXPECT_EQ(satno(SYS_QZS, ephemeris.PRN), rtklib_ephemeris.sat);
    EXPECT_EQ(1, rtklib_ephemeris.cnav_ura_valid);
    EXPECT_EQ(-1, rtklib_ephemeris.cnav_uraed);
    EXPECT_EQ(-1, rtklib_ephemeris.cnav_uraned0);
    EXPECT_EQ(3, rtklib_ephemeris.cnav_uraned1);
    EXPECT_EQ(5, rtklib_ephemeris.cnav_uraned2);
    EXPECT_EQ(42, rtklib_ephemeris.cnav_wnop);
    EXPECT_EQ(1, rtklib_ephemeris.sva);

    auto mt32 = gps_cnav_page(32, 5);
    set_gps_cnav_unsigned_field(mt32, CNAV_T_EOP, 100);
    nav_message.decode_page(mt32);
    ASSERT_TRUE(nav_message.have_new_eop());
    EXPECT_EQ(197U, nav_message.get_eop().PRN);
}
