/*!
 * \file qzss_lnav_navigation_message_test.cc
 * \brief Tests for QZSS LNAV navigation message decoding
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

#include "GPS_L1_CA.h"
#include "gps_navigation_message.h"
#include "rtklib_conversions.h"
#include <array>
#include <bitset>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <utility>
#include <vector>

namespace
{
int32_t qzss_lnav_field_width(const std::vector<std::pair<int32_t, int32_t>>& parameter)
{
    int32_t width = 0;
    for (const auto& p : parameter)
        {
            width += p.second;
        }
    return width;
}


void set_qzss_lnav_unsigned_field(std::bitset<GPS_SUBFRAME_BITS>& bits,
    const std::vector<std::pair<int32_t, int32_t>>& parameter,
    uint64_t value)
{
    int32_t pending_bits = qzss_lnav_field_width(parameter);

    for (const auto& p : parameter)
        {
            for (int32_t j = 0; j < p.second; ++j)
                {
                    --pending_bits;
                    bits[GPS_SUBFRAME_BITS - p.first - j] = ((value >> pending_bits) & 1ULL) != 0ULL;
                }
        }
}


void set_qzss_lnav_signed_field(std::bitset<GPS_SUBFRAME_BITS>& bits,
    const std::vector<std::pair<int32_t, int32_t>>& parameter,
    int64_t value)
{
    const int32_t width = qzss_lnav_field_width(parameter);
    const uint64_t mask = (width == 64) ? std::numeric_limits<uint64_t>::max() : ((1ULL << width) - 1ULL);
    set_qzss_lnav_unsigned_field(bits, parameter, static_cast<uint64_t>(value) & mask);
}


std::array<char, GPS_SUBFRAME_LENGTH> qzss_lnav_subframe_bytes(const std::bitset<GPS_SUBFRAME_BITS>& bits)
{
    std::array<char, GPS_SUBFRAME_LENGTH> subframe{};

    for (int32_t i = 0; i < 10; ++i)
        {
            uint32_t word = 0;
            for (int32_t j = 0; j < GPS_WORD_BITS; ++j)
                {
                    if (bits[GPS_WORD_BITS * (9 - i) + j])
                        {
                            word |= (1U << j);
                        }
                }
            std::memcpy(&subframe[i * GPS_WORD_LENGTH], &word, sizeof(word));
        }

    return subframe;
}


std::bitset<GPS_SUBFRAME_BITS> qzss_lnav_page(int32_t subframe_id, int32_t sv_id)
{
    std::bitset<GPS_SUBFRAME_BITS> bits;

    set_qzss_lnav_unsigned_field(bits, TOW, 1);
    set_qzss_lnav_unsigned_field(bits, SUBFRAME_ID, static_cast<uint64_t>(subframe_id));
    set_qzss_lnav_unsigned_field(bits, SV_DATA_ID, 3);
    set_qzss_lnav_unsigned_field(bits, SV_PAGE, static_cast<uint64_t>(sv_id));

    return bits;
}
}  // namespace


TEST(QzssLnavNavigationMessageTest, DecodesQzoAlmanacFromSubframe4)
{
    Gps_Navigation_Message nav_message(LnavSystem::QZSS);

    auto bits = qzss_lnav_page(4, 2);
    set_qzss_lnav_unsigned_field(bits, ALM_ECC, 100);
    set_qzss_lnav_unsigned_field(bits, ALM_TOA, 7);
    set_qzss_lnav_signed_field(bits, ALM_DELTAI, -25);
    set_qzss_lnav_signed_field(bits, ALM_OMEGADOT, -30);
    set_qzss_lnav_unsigned_field(bits, ALM_SVHEALTH, 0xAD);
    set_qzss_lnav_unsigned_field(bits, ALM_SQUAREA, 5440000);
    set_qzss_lnav_signed_field(bits, ALM_OMEGAZERO, -3000);
    set_qzss_lnav_signed_field(bits, ALM_OMEGA, 2000);
    set_qzss_lnav_signed_field(bits, ALM_MZERO, -1000);
    set_qzss_lnav_signed_field(bits, ALM_AF0, -12);
    set_qzss_lnav_signed_field(bits, ALM_AF1, 13);

    const auto subframe = qzss_lnav_subframe_bytes(bits);
    EXPECT_EQ(4, nav_message.subframe_decoder(subframe.data()));

    ASSERT_FALSE(nav_message.almanac_validation());
    const auto almanac = nav_message.get_almanac();

    EXPECT_EQ('J', almanac.get_system());
    EXPECT_EQ(194U, almanac.PRN);
    EXPECT_EQ(7 * ALM_TOA_LSB, almanac.toa);
    EXPECT_EQ(0xAD, almanac.SV_health);
    EXPECT_NEAR(0.06 + 100.0 * ALM_ECC_LSB, almanac.ecc, 1e-14);
    EXPECT_NEAR(0.25 - 25.0 * ALM_DELTAI_LSB, almanac.delta_i, 1e-14);
    EXPECT_NEAR(-30.0 * ALM_OMEGADOT_LSB, almanac.OMEGAdot, 1e-18);

    const auto rtklib_almanac = alm_to_rtklib(almanac);
    EXPECT_EQ(satno(SYS_QZS, 194), rtklib_almanac.sat);
    EXPECT_NEAR(almanac.delta_i * GNSS_PI, rtklib_almanac.i0, 1e-12);
}


TEST(GpsLnavNavigationMessageTest, KeepsGpsAlmanacReferenceConvention)
{
    Gps_Navigation_Message nav_message;

    auto bits = qzss_lnav_page(5, 2);
    set_qzss_lnav_unsigned_field(bits, ALM_ECC, 100);
    set_qzss_lnav_unsigned_field(bits, ALM_TOA, 7);
    set_qzss_lnav_signed_field(bits, ALM_DELTAI, -25);
    set_qzss_lnav_unsigned_field(bits, ALM_SVHEALTH, 0x12);

    const auto subframe = qzss_lnav_subframe_bytes(bits);
    EXPECT_EQ(5, nav_message.subframe_decoder(subframe.data()));

    const auto almanac = nav_message.get_almanac();
    EXPECT_EQ('G', almanac.get_system());
    EXPECT_EQ(2U, almanac.PRN);
    EXPECT_EQ(7 * ALM_TOA_LSB, almanac.toa);
    EXPECT_EQ(0x12, almanac.SV_health);
    EXPECT_NEAR(100.0 * ALM_ECC_LSB, almanac.ecc, 1e-14);
    EXPECT_NEAR(-25.0 * ALM_DELTAI_LSB, almanac.delta_i, 1e-14);

    const auto rtklib_almanac = alm_to_rtklib(almanac);
    EXPECT_EQ(satno(SYS_GPS, 2), rtklib_almanac.sat);
    EXPECT_NEAR((0.3 + almanac.delta_i) * GNSS_PI, rtklib_almanac.i0, 1e-12);
}


TEST(QzssLnavNavigationMessageTest, DecodesQzssAlmanacEpochAndHealthFromSubframe4)
{
    Gps_Navigation_Message nav_message(LnavSystem::QZSS);

    auto bits = qzss_lnav_page(4, 51);
    set_qzss_lnav_unsigned_field(bits, T_OA, 9);
    set_qzss_lnav_unsigned_field(bits, WN_A, 77);
    set_qzss_lnav_unsigned_field(bits, HEALTH_SV1, 1);
    set_qzss_lnav_unsigned_field(bits, HEALTH_SV4, 4);
    set_qzss_lnav_unsigned_field(bits, HEALTH_SV8, 8);
    set_qzss_lnav_unsigned_field(bits, HEALTH_SV10, 10);

    const auto subframe = qzss_lnav_subframe_bytes(bits);
    EXPECT_EQ(4, nav_message.subframe_decoder(subframe.data()));

    EXPECT_EQ(1, nav_message.get_almanac_health(193));
    EXPECT_EQ(4, nav_message.get_almanac_health(196));
    EXPECT_EQ(8, nav_message.get_almanac_health(200));
    EXPECT_EQ(10, nav_message.get_almanac_health(202));
}


TEST(QzssLnavNavigationMessageTest, DecodesJapanAreaIonoUtcFromSubframe5)
{
    Gps_Navigation_Message nav_message(LnavSystem::QZSS);

    auto bits = qzss_lnav_page(5, 61);
    set_qzss_lnav_signed_field(bits, ALPHA_0, -1);
    set_qzss_lnav_signed_field(bits, ALPHA_1, 2);
    set_qzss_lnav_signed_field(bits, ALPHA_2, -3);
    set_qzss_lnav_signed_field(bits, ALPHA_3, 4);
    set_qzss_lnav_signed_field(bits, BETA_0, -5);
    set_qzss_lnav_signed_field(bits, BETA_1, 6);
    set_qzss_lnav_signed_field(bits, BETA_2, -7);
    set_qzss_lnav_signed_field(bits, BETA_3, 8);
    set_qzss_lnav_signed_field(bits, A_1, -9);
    set_qzss_lnav_signed_field(bits, A_0, 10);
    set_qzss_lnav_unsigned_field(bits, T_OT, 11);
    set_qzss_lnav_unsigned_field(bits, WN_T, 12);
    set_qzss_lnav_signed_field(bits, DELTAT_LS, -18);
    set_qzss_lnav_unsigned_field(bits, WN_LSF, 13);
    set_qzss_lnav_unsigned_field(bits, DN, 4);
    set_qzss_lnav_signed_field(bits, DELTAT_LSF, -17);

    const auto subframe = qzss_lnav_subframe_bytes(bits);
    EXPECT_EQ(5, nav_message.subframe_decoder(subframe.data()));

    ASSERT_TRUE(nav_message.get_flag_iono_valid());
    ASSERT_TRUE(nav_message.get_flag_utc_model_valid());

    const auto iono = nav_message.get_iono();
    EXPECT_TRUE(iono.valid);
    EXPECT_NEAR(-1.0 * ALPHA_0_LSB, iono.alpha0, 1e-18);
    EXPECT_NEAR(2.0 * ALPHA_1_LSB, iono.alpha1, 1e-18);
    EXPECT_NEAR(-3.0 * ALPHA_2_LSB, iono.alpha2, 1e-18);
    EXPECT_NEAR(4.0 * ALPHA_3_LSB, iono.alpha3, 1e-18);
    EXPECT_NEAR(-5.0 * BETA_0_LSB, iono.beta0, 1e-12);
    EXPECT_NEAR(6.0 * BETA_1_LSB, iono.beta1, 1e-12);

    const auto utc = nav_message.get_utc_model();
    EXPECT_TRUE(utc.valid);
    EXPECT_NEAR(-9.0 * A_1_LSB, utc.A1, 1e-18);
    EXPECT_NEAR(10.0 * A_0_LSB, utc.A0, 1e-18);
    EXPECT_EQ(11 * T_OT_LSB, utc.tot);
    EXPECT_EQ(12, utc.WN_T);
    EXPECT_EQ(-18, utc.DeltaT_LS);
    EXPECT_EQ(13, utc.WN_LSF);
    EXPECT_EQ(4, utc.DN);
    EXPECT_EQ(-17, utc.DeltaT_LSF);
}
