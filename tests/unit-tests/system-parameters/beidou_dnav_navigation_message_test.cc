/*!
 * \file beidou_dnav_navigation_message_test.cc
 * \brief Tests for BeiDou DNAV navigation message consistency checks
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

#include "Beidou_DNAV.h"
#include "beidou_dnav_navigation_message.h"
#include "rtklib_conversions.h"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace
{
void set_unsigned_field(std::string& bits, const std::vector<std::pair<int32_t, int32_t>>& field, uint64_t value)
{
    int32_t field_length = 0;
    for (const auto& part : field)
        {
            field_length += part.second;
        }

    int32_t bit_offset = 0;
    for (const auto& part : field)
        {
            for (int32_t j = 0; j < part.second; ++j)
                {
                    const auto value_bit = static_cast<uint64_t>(1ULL << (field_length - bit_offset - 1));
                    bits[static_cast<std::size_t>(part.first + j - 1)] = ((value & value_bit) != 0U) ? '1' : '0';
                    ++bit_offset;
                }
        }
}


void set_signed_field(std::string& bits, const std::vector<std::pair<int32_t, int32_t>>& field, int64_t value)
{
    int32_t field_length = 0;
    for (const auto& part : field)
        {
            field_length += part.second;
        }
    const uint64_t mask = (field_length == 64) ? std::numeric_limits<uint64_t>::max() : ((1ULL << field_length) - 1ULL);
    set_unsigned_field(bits, field, static_cast<uint64_t>(value) & mask);
}


void set_navigation_data_field(std::string& bits, int32_t first_bit, int32_t logical_offset, int32_t length, uint64_t value)
{
    int32_t logical_bit = 0;
    int32_t transmitted_bit = first_bit;
    while (logical_bit < logical_offset + length)
        {
            const int32_t word_bit = ((transmitted_bit - 1) % static_cast<int32_t>(BEIDOU_DNAV_WORD_LENGTH_BITS)) + 1;
            const bool parity_bit = (transmitted_bit <= static_cast<int32_t>(BEIDOU_DNAV_WORD_LENGTH_BITS)) ? (word_bit > 26) : (word_bit > 22);
            if (parity_bit == false)
                {
                    if (logical_bit >= logical_offset)
                        {
                            const int32_t value_bit = length - (logical_bit - logical_offset) - 1;
                            bits[static_cast<std::size_t>(transmitted_bit - 1)] = ((value >> value_bit) & 1ULL) != 0ULL ? '1' : '0';
                        }
                    ++logical_bit;
                }
            ++transmitted_bit;
        }
}


std::string make_d1_subframe(int32_t subframe_ID, uint32_t sow, int32_t page_ID = 0)
{
    std::string subframe(BEIDOU_DNAV_SUBFRAME_DATA_BITS, '0');
    subframe.replace(0, BEIDOU_DNAV_PREAMBLE_LENGTH_BITS, BEIDOU_DNAV_PREAMBLE);
    set_unsigned_field(subframe, D1_FRAID, static_cast<uint64_t>(subframe_ID));
    set_unsigned_field(subframe, D1_SOW, sow);
    if (subframe_ID == 4 || subframe_ID == 5)
        {
            set_unsigned_field(subframe, D1_PNUM, static_cast<uint64_t>(page_ID));
        }
    return subframe;
}


std::string make_d2_subframe(int32_t subframe_ID, uint32_t sow, int32_t page_ID = 0)
{
    std::string subframe(BEIDOU_DNAV_SUBFRAME_DATA_BITS, '0');
    subframe.replace(0, BEIDOU_DNAV_PREAMBLE_LENGTH_BITS, BEIDOU_DNAV_PREAMBLE);
    set_unsigned_field(subframe, D2_FRAID, static_cast<uint64_t>(subframe_ID));
    set_unsigned_field(subframe, D2_SOW, sow);
    if (subframe_ID == 1)
        {
            set_unsigned_field(subframe, D2_PNUM1, static_cast<uint64_t>(page_ID));
        }
    else if (subframe_ID == 2)
        {
            set_unsigned_field(subframe, D2_PNUM2, static_cast<uint64_t>(page_ID));
        }
    else if (subframe_ID == 5)
        {
            set_unsigned_field(subframe, D2_PNUM5, static_cast<uint64_t>(page_ID));
        }
    return subframe;
}


std::string make_d2_subframe1_page(int32_t page_ID, uint32_t sow)
{
    return make_d2_subframe(1, sow, page_ID);
}
}  // namespace


TEST(BeidouDnavNavigationMessageTest, D2Subframe1PagesAcceptContiguousSow)
{
    auto dnav_msg = Beidou_Dnav_Navigation_Message();
    dnav_msg.set_satellite_PRN(1);

    constexpr uint32_t first_sow = 1000;
    for (int32_t page_ID = 1; page_ID <= 10; ++page_ID)
        {
            const auto sow = first_sow + static_cast<uint32_t>((page_ID - 1) * BEIDOU_DNAV_D2_SUBFRAME1_PAGE_PERIOD_SECONDS);
            EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(make_d2_subframe1_page(page_ID, sow)));
            EXPECT_TRUE(dnav_msg.get_flag_CRC_test());
            EXPECT_EQ(static_cast<double>(sow), dnav_msg.get_SOW());
        }
}


TEST(BeidouDnavNavigationMessageTest, D2Subframe1PageRejectsUnexpectedSow)
{
    auto dnav_msg = Beidou_Dnav_Navigation_Message();
    dnav_msg.set_satellite_PRN(1);

    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(make_d2_subframe1_page(1, 1000)));
    EXPECT_TRUE(dnav_msg.get_flag_CRC_test());

    EXPECT_EQ(0, dnav_msg.d2_subframe_decoder(make_d2_subframe1_page(2, 1004)));
    EXPECT_FALSE(dnav_msg.get_flag_CRC_test());
    EXPECT_EQ(1000.0, dnav_msg.get_SOW());

    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(make_d2_subframe1_page(1, 2000)));
    EXPECT_TRUE(dnav_msg.get_flag_CRC_test());
    EXPECT_EQ(2000.0, dnav_msg.get_SOW());
}


TEST(BeidouDnavNavigationMessageTest, D2Subframe1PageRejectsUnexpectedPageOrder)
{
    auto dnav_msg = Beidou_Dnav_Navigation_Message();
    dnav_msg.set_satellite_PRN(1);

    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(make_d2_subframe1_page(1, 1000)));
    EXPECT_TRUE(dnav_msg.get_flag_CRC_test());

    EXPECT_EQ(0, dnav_msg.d2_subframe_decoder(make_d2_subframe1_page(3, 1003)));
    EXPECT_FALSE(dnav_msg.get_flag_CRC_test());
}


TEST(BeidouDnavNavigationMessageTest, D2Subframe1PageRejectsSowOutOfRange)
{
    auto dnav_msg = Beidou_Dnav_Navigation_Message();
    dnav_msg.set_satellite_PRN(1);

    EXPECT_EQ(0, dnav_msg.d2_subframe_decoder(make_d2_subframe1_page(1, 604800)));
    EXPECT_FALSE(dnav_msg.get_flag_CRC_test());
}


TEST(BeidouDnavNavigationMessageTest, D2Subframe1PagesAcceptSowWeekRollover)
{
    auto dnav_msg = Beidou_Dnav_Navigation_Message();
    dnav_msg.set_satellite_PRN(1);

    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(make_d2_subframe1_page(1, 604797)));
    EXPECT_TRUE(dnav_msg.get_flag_CRC_test());

    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(make_d2_subframe1_page(2, 0)));
    EXPECT_TRUE(dnav_msg.get_flag_CRC_test());

    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(make_d2_subframe1_page(3, 3)));
    EXPECT_TRUE(dnav_msg.get_flag_CRC_test());
}


TEST(BeidouDnavNavigationMessageTest, DecodesD1AlmanacAndConvertsItToRtklib)
{
    Beidou_Dnav_Navigation_Message dnav_msg;
    dnav_msg.set_satellite_PRN(10);

    auto sf1 = make_d1_subframe(1, 1000);
    set_unsigned_field(sf1, D1_WN, 1000);
    EXPECT_EQ(1, dnav_msg.d1_subframe_decoder(sf1));

    auto health_page = make_d1_subframe(5, 1030, 7);
    set_unsigned_field(health_page, D1_HEA6, 0x12);
    EXPECT_EQ(5, dnav_msg.d1_subframe_decoder(health_page));

    auto epoch_page = make_d1_subframe(5, 1060, 8);
    set_unsigned_field(epoch_page, D1_WNA, 240);
    set_unsigned_field(epoch_page, D1_TOA2, 11);
    EXPECT_EQ(5, dnav_msg.d1_subframe_decoder(epoch_page));

    auto almanac_page = make_d1_subframe(4, 1090, 6);
    set_unsigned_field(almanac_page, D1_SQRT_A_ALMANAC, 5000000);
    set_signed_field(almanac_page, D1_A1_ALMANAC, -7);
    set_signed_field(almanac_page, D1_A0_ALMANAC, 9);
    set_signed_field(almanac_page, D1_OMEGA0_ALMANAC, -1000);
    set_unsigned_field(almanac_page, D1_E_ALMANAC, 1234);
    set_signed_field(almanac_page, D1_DELTA_I, -33);
    set_unsigned_field(almanac_page, D1_TOA, 11);
    set_signed_field(almanac_page, D1_OMEGA_DOT_ALMANAC, -44);
    set_signed_field(almanac_page, D1_OMEGA_ALMANAC, 2000);
    set_signed_field(almanac_page, D1_M0_ALMANAC, -3000);
    set_unsigned_field(almanac_page, D1_AMEPID, 3);
    EXPECT_EQ(4, dnav_msg.d1_subframe_decoder(almanac_page));

    ASSERT_TRUE(dnav_msg.have_new_almanac());
    const auto almanac = dnav_msg.get_almanac();
    EXPECT_EQ(6U, almanac.PRN);
    EXPECT_EQ(1008, almanac.WNa);
    EXPECT_EQ(11 * D1_TOA_LSB, almanac.toa);
    EXPECT_EQ(0x12, almanac.SV_health);
    EXPECT_EQ(3, almanac.AmEpID);
    EXPECT_FALSE(almanac.expanded);
    EXPECT_NEAR(5000000.0 * D1_SQRT_A_ALMANAC_LSB, almanac.sqrtA, 1e-12);
    EXPECT_NEAR(-33.0 * D1_DELTA_I_LSB, almanac.delta_i, 1e-18);
    EXPECT_NEAR(-3000.0 * D1_M0_ALMANAC_LSB, almanac.M_0, 1e-15);

    const auto rtklib_almanac = alm_to_rtklib(almanac);
    EXPECT_EQ(satno(SYS_BDS, 6), rtklib_almanac.sat);
    EXPECT_EQ(1008 + BEIDOU_DNAV_BDT2GPST_WEEK_NUM_OFFSET, rtklib_almanac.week);
    EXPECT_NEAR(almanac.delta_i + 0.3 * GNSS_PI, rtklib_almanac.i0, 1e-15);
    EXPECT_NEAR(almanac.OMEGA_0, rtklib_almanac.OMG0, 1e-15);
}


TEST(BeidouDnavNavigationMessageTest, DecodesAllD1EphemerisParameters)
{
    Beidou_Dnav_Navigation_Message dnav_msg;
    dnav_msg.set_satellite_PRN(6);
    dnav_msg.set_signal_type(1);

    auto sf1 = make_d1_subframe(1, 1000);
    set_unsigned_field(sf1, D1_SAT_H1, 1);
    set_unsigned_field(sf1, D1_AODC, 3);
    set_unsigned_field(sf1, D1_URAI, 4);
    set_unsigned_field(sf1, D1_WN, 1000);
    set_unsigned_field(sf1, D1_TOC, 123);
    set_signed_field(sf1, D1_TGD1, -5);
    set_signed_field(sf1, D1_TGD2, 6);
    set_signed_field(sf1, D1_ALPHA0, -1);
    set_signed_field(sf1, D1_ALPHA1, 2);
    set_signed_field(sf1, D1_ALPHA2, -3);
    set_signed_field(sf1, D1_ALPHA3, 4);
    set_signed_field(sf1, D1_BETA0, -5);
    set_signed_field(sf1, D1_BETA1, 6);
    set_signed_field(sf1, D1_BETA2, -7);
    set_signed_field(sf1, D1_BETA3, 8);
    set_signed_field(sf1, D1_A2, -9);
    set_signed_field(sf1, D1_A0, 1000);
    set_signed_field(sf1, D1_A1, -2000);
    set_unsigned_field(sf1, D1_AODE, 0);
    EXPECT_EQ(1, dnav_msg.d1_subframe_decoder(sf1));

    auto sf2 = make_d1_subframe(2, 1006);
    set_signed_field(sf2, D1_DELTA_N, -20);
    set_signed_field(sf2, D1_CUC, -30);
    set_signed_field(sf2, D1_M0, -4000);
    set_unsigned_field(sf2, D1_E, 5000);
    set_signed_field(sf2, D1_CUS, 60);
    set_signed_field(sf2, D1_CRC, -70);
    set_signed_field(sf2, D1_CRS, 80);
    set_unsigned_field(sf2, D1_SQRT_A, 5150000);
    set_unsigned_field(sf2, D1_TOE_SF2, 1);
    EXPECT_EQ(2, dnav_msg.d1_subframe_decoder(sf2));

    auto sf3 = make_d1_subframe(3, 1012);
    set_unsigned_field(sf3, D1_TOE_SF3, 2);
    set_signed_field(sf3, D1_I0, -6000);
    set_signed_field(sf3, D1_CIC, 90);
    set_signed_field(sf3, D1_OMEGA_DOT, -100);
    set_signed_field(sf3, D1_CIS, 110);
    set_signed_field(sf3, D1_IDOT, -120);
    set_signed_field(sf3, D1_OMEGA0, 7000);
    set_signed_field(sf3, D1_OMEGA, -8000);
    EXPECT_EQ(3, dnav_msg.d1_subframe_decoder(sf3));

    ASSERT_TRUE(dnav_msg.have_new_ephemeris());
    const auto eph = dnav_msg.get_ephemeris();
    EXPECT_EQ(6U, eph.PRN);
    EXPECT_EQ(1, eph.nav_type);
    EXPECT_EQ(1, eph.sig_type);
    EXPECT_EQ(1000, eph.WN);
    EXPECT_EQ(1, eph.SV_health);
    EXPECT_EQ(4, eph.SV_accuracy);
    EXPECT_EQ(3, eph.AODC);
    EXPECT_EQ(0, eph.AODE);
    EXPECT_EQ(1012.0, eph.tow);
    EXPECT_EQ(static_cast<double>(((1U << 15U) + 2U) * D1_TOE_LSB), eph.toe);
    EXPECT_EQ(123.0 * D1_TOC_LSB, eph.toc);
    EXPECT_NEAR(-4000.0 * D1_M0_LSB, eph.M_0, 1e-15);
    EXPECT_NEAR(-20.0 * D1_DELTA_N_LSB, eph.delta_n, 1e-18);
    EXPECT_NEAR(5000.0 * D1_E_LSB, eph.ecc, 1e-18);
    EXPECT_NEAR(5150000.0 * D1_SQRT_A_LSB, eph.sqrtA, 1e-12);
    EXPECT_NEAR(7000.0 * D1_OMEGA0_LSB, eph.OMEGA_0, 1e-15);
    EXPECT_NEAR(-6000.0 * D1_I0_LSB, eph.i_0, 1e-15);
    EXPECT_NEAR(-8000.0 * D1_OMEGA_LSB, eph.omega, 1e-15);
    EXPECT_NEAR(-100.0 * D1_OMEGA_DOT_LSB, eph.OMEGAdot, 1e-18);
    EXPECT_NEAR(-120.0 * D1_IDOT_LSB, eph.idot, 1e-18);
    EXPECT_NEAR(-30.0 * D1_CUC_LSB, eph.Cuc, 1e-18);
    EXPECT_NEAR(60.0 * D1_CUS_LSB, eph.Cus, 1e-18);
    EXPECT_NEAR(-70.0 * D1_CRC_LSB, eph.Crc, 1e-15);
    EXPECT_NEAR(80.0 * D1_CRS_LSB, eph.Crs, 1e-15);
    EXPECT_NEAR(90.0 * D1_CIC_LSB, eph.Cic, 1e-18);
    EXPECT_NEAR(110.0 * D1_CIS_LSB, eph.Cis, 1e-18);
    EXPECT_NEAR(1000.0 * D1_A0_LSB, eph.af0, 1e-20);
    EXPECT_NEAR(-2000.0 * D1_A1_LSB, eph.af1, 1e-25);
    EXPECT_NEAR(-9.0 * D1_A2_LSB, eph.af2, 1e-30);
    EXPECT_NEAR(-5.0 * D1_TGD1_LSB, eph.TGD1, 1e-20);
    EXPECT_NEAR(6.0 * D1_TGD2_LSB, eph.TGD2, 1e-20);
}


TEST(BeidouDnavNavigationMessageTest, DecodesAllD2EphemerisParameters)
{
    Beidou_Dnav_Navigation_Message dnav_msg;
    dnav_msg.set_satellite_PRN(1);
    dnav_msg.set_signal_type(5);

    const auto unsigned_value = [](int64_t value, int32_t width) {
        return static_cast<uint64_t>(value) & ((1ULL << width) - 1ULL);
    };
    const uint64_t af1 = unsigned_value(-12345, 22);
    const uint64_t cuc = unsigned_value(-1234, 18);
    const uint64_t eccentricity = 0x12345678ULL;
    const uint64_t cic = unsigned_value(-2345, 18);
    const uint64_t i0 = unsigned_value(-12345678, 32);
    const uint64_t omega_dot = unsigned_value(-345678, 24);
    const uint64_t omega = unsigned_value(-23456789, 32);

    auto page1 = make_d2_subframe1_page(1, 600);
    set_unsigned_field(page1, D2_SAT_H1, 1);
    set_unsigned_field(page1, D2_AODC, 2);
    set_unsigned_field(page1, D2_URAI, 3);
    set_unsigned_field(page1, D2_WN, 1001);
    set_unsigned_field(page1, D2_TOC, 44);
    set_signed_field(page1, D2_TGD1, -5);
    set_signed_field(page1, D2_TGD2, 6);
    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(page1));

    auto page2 = make_d2_subframe1_page(2, 603);
    set_signed_field(page2, D2_ALPHA0, -1);
    set_signed_field(page2, D2_ALPHA1, 2);
    set_signed_field(page2, D2_ALPHA2, -3);
    set_signed_field(page2, D2_ALPHA3, 4);
    set_signed_field(page2, D2_BETA0, -5);
    set_signed_field(page2, D2_BETA1, 6);
    set_signed_field(page2, D2_BETA2, -7);
    set_signed_field(page2, D2_BETA3, 8);
    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(page2));

    auto page3 = make_d2_subframe1_page(3, 606);
    set_signed_field(page3, D2_A0, 1000);
    set_unsigned_field(page3, D2_A1_MSB, af1 >> 18U);
    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(page3));

    auto page4 = make_d2_subframe1_page(4, 609);
    set_unsigned_field(page4, D2_A1_LSB, af1 & ((1ULL << 18U) - 1ULL));
    set_signed_field(page4, D2_A2, -9);
    set_unsigned_field(page4, D2_AODE, 0);
    set_signed_field(page4, D2_DELTA_N, -20);
    set_unsigned_field(page4, D2_CUC_MSB, cuc >> 4U);
    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(page4));

    auto page5 = make_d2_subframe1_page(5, 612);
    set_unsigned_field(page5, D2_CUC_LSB, cuc & 0xFULL);
    set_signed_field(page5, D2_M0, -4000);
    set_signed_field(page5, D2_CUS, 60);
    set_unsigned_field(page5, D2_E_MSB, eccentricity >> 22U);
    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(page5));

    auto page6 = make_d2_subframe1_page(6, 615);
    set_unsigned_field(page6, D2_E_LSB, eccentricity & ((1ULL << 22U) - 1ULL));
    set_unsigned_field(page6, D2_SQRT_A, 5150000);
    set_unsigned_field(page6, D2_CIC_MSB, cic >> 8U);
    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(page6));

    auto page7 = make_d2_subframe1_page(7, 618);
    set_unsigned_field(page7, D2_CIC_LSB, cic & 0xFFULL);
    set_signed_field(page7, D2_CIS, 110);
    set_unsigned_field(page7, D2_TOE, 55);
    set_unsigned_field(page7, D2_I0_MSB, i0 >> 11U);
    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(page7));

    auto page8 = make_d2_subframe1_page(8, 621);
    set_unsigned_field(page8, D2_I0_LSB, i0 & ((1ULL << 11U) - 1ULL));
    set_signed_field(page8, D2_CRC, -70);
    set_signed_field(page8, D2_CRS, 80);
    set_unsigned_field(page8, D2_OMEGA_DOT_MSB, omega_dot >> 5U);
    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(page8));

    auto page9 = make_d2_subframe1_page(9, 624);
    set_unsigned_field(page9, D2_OMEGA_DOT_LSB, omega_dot & 0x1FULL);
    set_signed_field(page9, D2_OMEGA0, 7000);
    set_unsigned_field(page9, D2_OMEGA_MSB, omega >> 5U);
    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(page9));

    auto page10 = make_d2_subframe1_page(10, 627);
    set_unsigned_field(page10, D2_OMEGA_LSB, omega & 0x1FULL);
    set_signed_field(page10, D2_IDOT, -120);
    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(page10));

    ASSERT_TRUE(dnav_msg.have_new_ephemeris());
    const auto eph = dnav_msg.get_ephemeris();
    EXPECT_EQ(1U, eph.PRN);
    EXPECT_EQ(2, eph.nav_type);
    EXPECT_EQ(5, eph.sig_type);
    EXPECT_EQ(1001, eph.WN);
    EXPECT_EQ(1, eph.SV_health);
    EXPECT_EQ(3, eph.SV_accuracy);
    EXPECT_EQ(2, eph.AODC);
    EXPECT_EQ(0, eph.AODE);
    EXPECT_EQ(627.0, eph.tow);
    EXPECT_EQ(55.0 * D1_TOE_LSB, eph.toe);
    EXPECT_NEAR(-12345678.0 * D1_I0_LSB, eph.i_0, 1e-15);
    EXPECT_NEAR(-23456789.0 * D1_OMEGA_LSB, eph.omega, 1e-15);
    EXPECT_NEAR(-345678.0 * D1_OMEGA_DOT_LSB, eph.OMEGAdot, 1e-18);
    EXPECT_NEAR(-1234.0 * D1_CUC_LSB, eph.Cuc, 1e-18);
    EXPECT_NEAR(-2345.0 * D1_CIC_LSB, eph.Cic, 1e-18);
    EXPECT_NEAR(static_cast<double>(eccentricity) * D1_E_LSB, eph.ecc, 1e-15);
    EXPECT_NEAR(-12345.0 * D1_A1_LSB, eph.af1, 1e-25);
}


TEST(BeidouDnavNavigationMessageTest, DecodesD1ExpandedAlmanac)
{
    Beidou_Dnav_Navigation_Message dnav_msg;

    auto epoch_page = make_d1_subframe(5, 994, 8);
    set_unsigned_field(epoch_page, D1_WNA, 20);
    EXPECT_EQ(5, dnav_msg.d1_subframe_decoder(epoch_page));

    auto regular_page = make_d1_subframe(4, 1000, 1);
    set_unsigned_field(regular_page, D1_AMEPID, 3);
    EXPECT_EQ(4, dnav_msg.d1_subframe_decoder(regular_page));
    ASSERT_TRUE(dnav_msg.have_new_almanac());

    auto expanded_page = make_d1_subframe(5, 1006, 11);
    set_unsigned_field(expanded_page, D1_SQRT_A_ALMANAC, 4000000);
    set_unsigned_field(expanded_page, D1_AMID, 2);
    EXPECT_EQ(5, dnav_msg.d1_subframe_decoder(expanded_page));

    ASSERT_TRUE(dnav_msg.have_new_almanac());
    const auto almanac = dnav_msg.get_almanac();
    EXPECT_EQ(44U, almanac.PRN);
    EXPECT_EQ(2, almanac.AmID);
    EXPECT_TRUE(almanac.expanded);
}


TEST(BeidouDnavNavigationMessageTest, DecodesD1TimeOffsetsAndUtcParameters)
{
    Beidou_Dnav_Navigation_Message dnav_msg;

    auto offsets = make_d1_subframe(5, 1000, 9);
    set_signed_field(offsets, D1_A0GPS, -11);
    set_signed_field(offsets, D1_A1GPS, 12);
    set_signed_field(offsets, D1_A0GAL, -13);
    set_signed_field(offsets, D1_A1GAL, 14);
    set_signed_field(offsets, D1_A0GLO, -15);
    set_signed_field(offsets, D1_A1GLO, 16);
    EXPECT_EQ(5, dnav_msg.d1_subframe_decoder(offsets));

    auto utc = make_d1_subframe(5, 1006, 10);
    set_signed_field(utc, D1_DELTA_T_LS, -4);
    set_signed_field(utc, D1_DELTA_T_LSF, -3);
    set_unsigned_field(utc, D1_WN_LSF, 200);
    set_signed_field(utc, D1_A0UTC, -1000);
    set_signed_field(utc, D1_A1UTC, 2000);
    set_unsigned_field(utc, D1_DN, 6);
    EXPECT_EQ(5, dnav_msg.d1_subframe_decoder(utc));

    ASSERT_TRUE(dnav_msg.have_new_utc_model());
    const auto model = dnav_msg.get_utc_model();
    EXPECT_TRUE(model.valid);
    EXPECT_EQ(-4, model.DeltaT_LS);
    EXPECT_EQ(-3, model.DeltaT_LSF);
    EXPECT_EQ(200, model.WN_LSF);
    EXPECT_EQ(6, model.DN);
    EXPECT_NEAR(-11.0 * D1_A0GPS_LSB, model.A0_GPS, 1e-20);
    EXPECT_NEAR(2000.0 * D1_A1UTC_LSB, model.A1_UTC, 1e-25);
}


TEST(BeidouDnavNavigationMessageTest, DecodesD2IntegrityAndDifferentialCorrections)
{
    Beidou_Dnav_Navigation_Message dnav_msg;
    dnav_msg.set_satellite_PRN(1);

    auto sf2 = make_d2_subframe(2, 300, 1);
    set_unsigned_field(sf2, D2_SAT_H2, 0);
    set_unsigned_field(sf2, D2_BDID1_30, ((1ULL << 18U) - 1ULL) << 12U);
    set_navigation_data_field(sf2, D2_UDREI1_LOGICAL_FIRST_BIT, 0, 4, 7);
    set_unsigned_field(sf2, D2_RURAI_SF2, 5);
    set_signed_field(sf2, D2_DELTA_T_SF2, -12);
    EXPECT_EQ(2, dnav_msg.d2_subframe_decoder(sf2));

    auto sf3 = make_d2_subframe(3, 300);
    set_unsigned_field(sf3, D2_RURAI_SF3_1, 2);
    set_signed_field(sf3, D2_DELTA_T_SF3_1, 30);
    set_unsigned_field(sf3, D2_RURAI_SF3_2, 3);
    set_signed_field(sf3, D2_DELTA_T_SF3_2, -40);
    EXPECT_EQ(3, dnav_msg.d2_subframe_decoder(sf3));

    auto sf4 = make_d2_subframe(4, 300);
    set_unsigned_field(sf4, D2_BDEPID, 3);
    set_unsigned_field(sf4, D2_BDID31_63, 1ULL << 32U);
    set_navigation_data_field(sf4, D2_UDREI19_LOGICAL_FIRST_BIT, 0, 4, 9);
    set_unsigned_field(sf4, D2_RURAI_SF4, 6);
    set_signed_field(sf4, D2_DELTA_T_SF4, -20);
    EXPECT_EQ(4, dnav_msg.d2_subframe_decoder(sf4));

    ASSERT_TRUE(dnav_msg.have_new_differential_corrections());
    const auto corrections = dnav_msg.get_differential_corrections();
    EXPECT_EQ(1U, corrections.source_PRN);
    EXPECT_EQ(300.0, corrections.SOW);
    EXPECT_EQ(1, corrections.Pnum2);
    EXPECT_EQ(3, corrections.BDEpID);
    EXPECT_TRUE(corrections.range_corrections_valid);
    EXPECT_TRUE(corrections.ionospheric_grid_valid);
    EXPECT_TRUE(corrections.expanded_corrections);

    ASSERT_NE(corrections.corrections.cend(), corrections.corrections.find(1));
    EXPECT_EQ(5, corrections.corrections.at(1).RURAI);
    EXPECT_EQ(7, corrections.corrections.at(1).UDREI);
    EXPECT_NEAR(-1.2, corrections.corrections.at(1).delta_t, 1e-12);
    EXPECT_TRUE(corrections.corrections.at(1).delta_t_available);

    ASSERT_NE(corrections.corrections.cend(), corrections.corrections.find(31));
    EXPECT_EQ(6, corrections.corrections.at(31).RURAI);
    EXPECT_EQ(9, corrections.corrections.at(31).UDREI);
    EXPECT_NEAR(-2.0, corrections.corrections.at(31).delta_t, 1e-12);
}


TEST(BeidouDnavNavigationMessageTest, DecodesD2IonosphericGridPages)
{
    Beidou_Dnav_Navigation_Message dnav_msg;

    auto page1 = make_d2_subframe(5, 300, 1);
    set_navigation_data_field(page1, D2_ION_LOGICAL_FIRST_BIT, 0, 13, (16U << 4U) | 5U);
    set_navigation_data_field(page1, D2_ION_LOGICAL_FIRST_BIT, 13, 13, (510U << 4U) | 14U);
    EXPECT_EQ(5, dnav_msg.d2_subframe_decoder(page1));

    ASSERT_TRUE(dnav_msg.have_new_iono());
    auto iono = dnav_msg.get_iono();
    ASSERT_TRUE(iono.grid_valid);
    ASSERT_NE(iono.grid_points.cend(), iono.grid_points.find(1));
    EXPECT_DOUBLE_EQ(2.0, iono.grid_points.at(1).vertical_delay);
    EXPECT_EQ(5, iono.grid_points.at(1).GIVEI);
    EXPECT_TRUE(iono.grid_points.at(1).monitored);
    EXPECT_TRUE(iono.grid_points.at(1).available);
    EXPECT_FALSE(iono.grid_points.at(2).monitored);
    EXPECT_TRUE(iono.grid_points.at(2).available);

    auto page73 = make_d2_subframe(5, 303, 73);
    set_navigation_data_field(page73, D2_ION_LOGICAL_FIRST_BIT, 0, 13, (511U << 4U) | 15U);
    EXPECT_EQ(5, dnav_msg.d2_subframe_decoder(page73));
    ASSERT_TRUE(dnav_msg.have_new_iono());
    iono = dnav_msg.get_iono();
    ASSERT_NE(iono.grid_points.cend(), iono.grid_points.find(317));
    EXPECT_FALSE(iono.grid_points.at(317).monitored);
    EXPECT_FALSE(iono.grid_points.at(317).available);
}


TEST(BeidouDnavNavigationMessageTest, DecodesD2AlmanacAndUtcPages)
{
    Beidou_Dnav_Navigation_Message dnav_msg;

    auto epoch_page = make_d2_subframe(5, 300, 36);
    set_unsigned_field(epoch_page, D1_WNA, 17);
    set_unsigned_field(epoch_page, D1_TOA2, 4);
    EXPECT_EQ(5, dnav_msg.d2_subframe_decoder(epoch_page));

    auto almanac_page = make_d2_subframe(5, 303, 37);
    set_unsigned_field(almanac_page, D1_SQRT_A_ALMANAC, 4000000);
    set_unsigned_field(almanac_page, D1_AMEPID, 3);
    EXPECT_EQ(5, dnav_msg.d2_subframe_decoder(almanac_page));
    ASSERT_TRUE(dnav_msg.have_new_almanac());
    EXPECT_EQ(1U, dnav_msg.get_almanac().PRN);

    auto expanded_page = make_d2_subframe(5, 306, 103);
    set_unsigned_field(expanded_page, D1_AMID, 2);
    EXPECT_EQ(5, dnav_msg.d2_subframe_decoder(expanded_page));
    ASSERT_TRUE(dnav_msg.have_new_almanac());
    EXPECT_EQ(44U, dnav_msg.get_almanac().PRN);

    auto offsets = make_d2_subframe(5, 309, 101);
    set_signed_field(offsets, D1_A0GPS, -9);
    EXPECT_EQ(5, dnav_msg.d2_subframe_decoder(offsets));

    auto utc = make_d2_subframe(5, 312, 102);
    set_unsigned_field(utc, D1_DN, 5);
    set_unsigned_field(utc, D1_WN_LSF, 201);
    EXPECT_EQ(5, dnav_msg.d2_subframe_decoder(utc));
    ASSERT_TRUE(dnav_msg.have_new_utc_model());
    const auto model = dnav_msg.get_utc_model();
    EXPECT_EQ(5, model.DN);
    EXPECT_EQ(201, model.WN_LSF);
    EXPECT_NEAR(-9.0 * D1_A0GPS_LSB, model.A0_GPS, 1e-20);
}
