/*!
 * \file gps_cnav2_navigation_message_test.cc
 * \brief Tests for the GPS CNAV-2 (L1C) message decoder and its FEC
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
#include "GPS_CNAV2.h"
#include "GPS_L1C.h"
#include "gps_cnav2_ldpc.h"
#include "gps_cnav2_navigation_message.h"
#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace
{
// Box-Muller on raw mt19937 output, so that results do not depend on the
// standard library implementation of std::normal_distribution
class Cnav2Gaussian
{
public:
    explicit Cnav2Gaussian(uint32_t seed) : d_rng(seed) {}
    double operator()()
    {
        const double u1 = (static_cast<double>(d_rng()) + 1.0) / 4294967297.0;
        const double u2 = static_cast<double>(d_rng()) / 4294967296.0;
        return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * GNSS_PI * u2);
    }

private:
    std::mt19937 d_rng;
};


void set_field(std::vector<uint8_t>& bits, const std::vector<std::pair<int32_t, int32_t>>& field, int64_t value)
{
    const int32_t first = field[0].first;
    const int32_t length = field[0].second;
    for (int32_t j = 0; j < length; j++)
        {
            bits[first - 1 + j] = static_cast<uint8_t>((static_cast<uint64_t>(value) >> (length - 1 - j)) & 1U);
        }
}


void append_crc(std::vector<uint8_t>& bits, size_t total_bits)
{
    const uint32_t crc = Gps_CNAV2_Navigation_Message::crc24q(bits, total_bits - 24);
    for (size_t i = 0; i < 24; i++)
        {
            bits[total_bits - 24 + i] = static_cast<uint8_t>((crc >> (23 - i)) & 1U);
        }
}


// Builds the 1800 symbols (as 0/1) of a frame from its subframes
std::vector<uint8_t> build_frame(int32_t toi, const std::vector<uint8_t>& sf2, const std::vector<uint8_t>& sf3)
{
    const Gps_Cnav2_Ldpc code2(2);
    const Gps_Cnav2_Ldpc code3(3);
    std::vector<uint8_t> coded = code2.encode(sf2);
    const std::vector<uint8_t> coded3 = code3.encode(sf3);
    coded.insert(coded.end(), coded3.cbegin(), coded3.cend());
    std::vector<uint8_t> frame = Gps_CNAV2_Navigation_Message::encode_toi(toi);
    std::vector<uint8_t> interleaved(coded.size());
    Gps_CNAV2_Navigation_Message::interleave(coded.data(), interleaved.data());
    frame.insert(frame.end(), interleaved.cbegin(), interleaved.cend());
    return frame;
}


std::vector<float> modulate(const std::vector<uint8_t>& bits, double sigma, uint32_t seed)
{
    Cnav2Gaussian gauss(seed);
    std::vector<float> symbols(bits.size());
    for (size_t i = 0; i < bits.size(); i++)
        {
            symbols[i] = static_cast<float>((bits[i] ? -1.0 : 1.0) + sigma * gauss());
        }
    return symbols;
}
}  // namespace


TEST(GpsCnav2NavigationMessageTest, ToiEncodingRoundTrip)
{
    for (int32_t toi = 0; toi <= GPS_L1C_TOI_MAX; toi++)
        {
            const std::vector<uint8_t> word = Gps_CNAV2_Navigation_Message::encode_toi(toi);
            ASSERT_EQ(word.size(), static_cast<size_t>(GPS_L1C_TOI_SYMBOLS));
            // The ninth bit is the first symbol and is added to all the others
            EXPECT_EQ(word[0], (toi >> 8) & 1);
            std::vector<float> symbols(word.size());
            for (size_t i = 0; i < word.size(); i++)
                {
                    symbols[i] = word[i] ? -1.0F : 1.0F;
                }
            float quality = 0.0F;
            EXPECT_EQ(Gps_CNAV2_Navigation_Message::decode_toi(symbols.data(), quality), toi);
            EXPECT_FLOAT_EQ(quality, 1.0F);
            // The minimum distance of the 52-symbol code is 20: up to 9 hard
            // errors are corrected
            for (int32_t i = 0; i < 9; i++)
                {
                    symbols[(toi + 5 * i) % GPS_L1C_TOI_SYMBOLS] *= -1.0F;
                }
            EXPECT_EQ(Gps_CNAV2_Navigation_Message::decode_toi(symbols.data(), quality), toi) << "TOI " << toi;
        }
}


TEST(GpsCnav2NavigationMessageTest, ToiGeneratorFirstSymbols)
{
    // With the initial conditions loaded MSB first, the first encoded symbol
    // is the MSB of the eight LSBs of TOI (IS-GPS-800J, Figure 3.2-4)
    const std::vector<uint8_t> word = Gps_CNAV2_Navigation_Message::encode_toi(0x80);
    EXPECT_EQ(word[0], 0);
    EXPECT_EQ(word[1], 1);
    EXPECT_EQ(word[2], 0);
    const std::vector<uint8_t> zero = Gps_CNAV2_Navigation_Message::encode_toi(0);
    EXPECT_EQ(std::vector<uint8_t>(zero.size(), 0), zero);
    const std::vector<uint8_t> msb = Gps_CNAV2_Navigation_Message::encode_toi(0x100);
    EXPECT_EQ(std::vector<uint8_t>(msb.size(), 1), msb);
}


TEST(GpsCnav2NavigationMessageTest, InterleaverRoundTrip)
{
    constexpr int32_t n = GPS_L1C_SF2_SYMBOLS + GPS_L1C_SF3_SYMBOLS;
    std::vector<uint8_t> in(n);
    for (int32_t i = 0; i < n; i++)
        {
            in[i] = static_cast<uint8_t>((i * 7 + i / 13) % 2);
        }
    std::vector<uint8_t> out(n);
    Gps_CNAV2_Navigation_Message::interleave(in.data(), out.data());
    // Column-wise read-out of a 38 x 46 row-wise written array
    EXPECT_EQ(out[1], in[GPS_L1C_INTERLEAVER_COLS]);
    EXPECT_EQ(out[GPS_L1C_INTERLEAVER_ROWS], in[1]);
    std::vector<float> soft(n);
    for (int32_t i = 0; i < n; i++)
        {
            soft[i] = static_cast<float>(i);
        }
    std::vector<float> interleaved_index(n);
    std::vector<float> back(n);
    // deinterleave(interleave(x)) == x, tested on symbol indexes
    for (int32_t i = 0; i < n; i++)
        {
            const int32_t row = i % GPS_L1C_INTERLEAVER_ROWS;
            const int32_t col = i / GPS_L1C_INTERLEAVER_ROWS;
            interleaved_index[i] = soft[row * GPS_L1C_INTERLEAVER_COLS + col];
        }
    Gps_CNAV2_Navigation_Message::deinterleave(interleaved_index.data(), back.data());
    EXPECT_EQ(back, soft);
}


TEST(GpsCnav2NavigationMessageTest, Crc24qCheckValue)
{
    // CRC-24Q (CRC-24/LTE-A parameters) check value of "123456789"
    const std::string message = "123456789";
    std::vector<uint8_t> bits;
    for (const char c : message)
        {
            for (int32_t j = 7; j >= 0; j--)
                {
                    bits.push_back(static_cast<uint8_t>((static_cast<uint8_t>(c) >> j) & 1U));
                }
        }
    EXPECT_EQ(Gps_CNAV2_Navigation_Message::crc24q(bits, bits.size()), 0xCDE703U);
}


TEST(GpsCnav2NavigationMessageTest, LdpcEncoderSatisfiesParityChecks)
{
    std::mt19937 rng(42);
    for (const int32_t subframe : {2, 3})
        {
            const Gps_Cnav2_Ldpc code(subframe);
            EXPECT_EQ(code.codeword_bits(), 2 * code.message_bits());
            for (int32_t trial = 0; trial < 20; trial++)
                {
                    std::vector<uint8_t> message(code.message_bits());
                    for (auto& b : message)
                        {
                            b = static_cast<uint8_t>(rng() & 1U);
                        }
                    const std::vector<uint8_t> codeword = code.encode(message);
                    EXPECT_TRUE(code.check(codeword));
                    EXPECT_EQ(std::vector<uint8_t>(codeword.cbegin(), codeword.cbegin() + code.message_bits()), message);
                    std::vector<uint8_t> corrupted = codeword;
                    corrupted[trial * 7] ^= 1U;
                    EXPECT_FALSE(code.check(corrupted));
                }
        }
}


TEST(GpsCnav2NavigationMessageTest, LdpcDecoderInAwgn)
{
    // The belief-propagation waterfall of both codes is around Es/N0 = 1 dB
    // (Eb/N0 = 4 dB for these rate 1/2 codes). At Es/N0 = 2 dB, about 6 % of
    // the symbols are wrong before decoding, and no word error is expected.
    const double sigma = std::pow(10.0, -2.0 / 20.0);
    std::mt19937 rng(7);
    for (const int32_t subframe : {2, 3})
        {
            const Gps_Cnav2_Ldpc code(subframe);
            int32_t failures = 0;
            int32_t raw_errors = 0;
            constexpr int32_t blocks = 50;
            for (int32_t block = 0; block < blocks; block++)
                {
                    std::vector<uint8_t> message(code.message_bits());
                    for (auto& b : message)
                        {
                            b = static_cast<uint8_t>(rng() & 1U);
                        }
                    const std::vector<uint8_t> codeword = code.encode(message);
                    const std::vector<float> received = modulate(codeword, sigma, 1000U + static_cast<uint32_t>(block));
                    std::vector<float> llr(received.size());
                    for (size_t i = 0; i < received.size(); i++)
                        {
                            llr[i] = static_cast<float>(2.0 * received[i] / (sigma * sigma));
                            raw_errors += ((received[i] < 0.0F) != (codeword[i] != 0)) ? 1 : 0;
                        }
                    std::vector<uint8_t> decoded;
                    if (!code.decode(llr, decoded) || decoded != codeword)
                        {
                            failures++;
                        }
                }
            EXPECT_GT(raw_errors, blocks * code.codeword_bits() / 25) << "subframe " << subframe;
            EXPECT_LE(failures, 1) << "subframe " << subframe;
        }
}


TEST(GpsCnav2NavigationMessageTest, FrameRoundTrip)
{
    std::vector<uint8_t> sf2(GPS_L1C_SF2_BITS, 0);
    set_field(sf2, CNAV2_WN, 2398);
    set_field(sf2, CNAV2_ITOW, 47);
    set_field(sf2, CNAV2_TOP, 1100);
    set_field(sf2, CNAV2_L1C_HEALTH, 0);
    set_field(sf2, CNAV2_URA_ED, -3);
    set_field(sf2, CNAV2_TOE, 1128);
    set_field(sf2, CNAV2_DELTA_A, -123456);
    set_field(sf2, CNAV2_A_DOT, 54321);
    set_field(sf2, CNAV2_DELTA_N0, -12345);
    set_field(sf2, CNAV2_DELTA_N0_DOT, 1234);
    set_field(sf2, CNAV2_M0, -3000000000LL);
    set_field(sf2, CNAV2_E_ECCENTRICITY, 123456789);
    set_field(sf2, CNAV2_OMEGA, 2000000000LL);
    set_field(sf2, CNAV2_OMEGA0, -1000000000LL);
    set_field(sf2, CNAV2_I0, 1300000000LL);
    set_field(sf2, CNAV2_DELTA_OMEGA_DOT, -321);
    set_field(sf2, CNAV2_I0_DOT, 77);
    set_field(sf2, CNAV2_CIS, -55);
    set_field(sf2, CNAV2_CIC, 66);
    set_field(sf2, CNAV2_CRS, -777);
    set_field(sf2, CNAV2_CRC, 8888);
    set_field(sf2, CNAV2_CUS, 999);
    set_field(sf2, CNAV2_CUC, -1111);
    set_field(sf2, CNAV2_URA_NED0, -5);
    set_field(sf2, CNAV2_URA_NED1, 3);
    set_field(sf2, CNAV2_URA_NED2, 6);
    set_field(sf2, CNAV2_AF0, -2345678);
    set_field(sf2, CNAV2_AF1, 4567);
    set_field(sf2, CNAV2_AF2, -12);
    set_field(sf2, CNAV2_TGD, -40);
    set_field(sf2, CNAV2_ISCL1CP, 21);
    set_field(sf2, CNAV2_ISCL1CD, -4096);  // not available
    set_field(sf2, CNAV2_ISF, 1);
    set_field(sf2, CNAV2_WNOP, 94);
    append_crc(sf2, GPS_L1C_SF2_BITS);

    std::vector<uint8_t> sf3(GPS_L1C_SF3_BITS, 0);
    set_field(sf3, CNAV2_SF3_PRN, 11);
    set_field(sf3, CNAV2_SF3_PAGE, 1);
    set_field(sf3, CNAV2_A0, -7);
    set_field(sf3, CNAV2_A1, 5);
    set_field(sf3, CNAV2_DELTA_TLS, 18);
    set_field(sf3, CNAV2_TOT, 36000);
    set_field(sf3, CNAV2_WN_OT, 2398);
    set_field(sf3, CNAV2_WN_LSF, 2185);
    set_field(sf3, CNAV2_DN, 7);
    set_field(sf3, CNAV2_DELTA_TLSF, 18);
    set_field(sf3, CNAV2_ALPHA0, 12);
    set_field(sf3, CNAV2_ALPHA1, -3);
    set_field(sf3, CNAV2_BETA0, 98);
    set_field(sf3, CNAV2_BETA3, -2);
    set_field(sf3, CNAV2_ISCL1CA, -9);
    set_field(sf3, CNAV2_ISCL5I5, 13);
    append_crc(sf3, GPS_L1C_SF3_BITS);

    constexpr int32_t toi = 123;
    const std::vector<uint8_t> frame = build_frame(toi, sf2, sf3);
    ASSERT_EQ(frame.size(), static_cast<size_t>(GPS_L1C_FRAME_SYMBOLS));

    // Es/N0 = 2 dB
    const std::vector<float> symbols = modulate(frame, std::pow(10.0, -2.0 / 20.0), 5);
    Gps_CNAV2_Navigation_Message nav;
    nav.set_prn(11);
    ASSERT_TRUE(nav.decode_frame(symbols.data()));
    EXPECT_TRUE(nav.sf2_crc_ok());
    EXPECT_TRUE(nav.sf3_crc_ok());
    EXPECT_EQ(nav.get_toi(), toi);
    EXPECT_EQ(nav.get_sf3_page(), 1);
    EXPECT_EQ(nav.get_frame_end_tow(), 47 * 7200 + toi * 18);

    ASSERT_TRUE(nav.have_new_ephemeris());
    const Gps_CNAV2_Ephemeris eph = nav.get_ephemeris();
    EXPECT_EQ(eph.PRN, 11U);
    EXPECT_EQ(eph.WN, 2398);
    EXPECT_EQ(eph.ITOW, 47);
    EXPECT_EQ(eph.tow, 47 * 7200 + (toi - 1) * 18);
    EXPECT_EQ(eph.top, 1100 * 300);
    EXPECT_EQ(eph.l1c_health, 0);
    EXPECT_EQ(eph.URAED, -3);
    EXPECT_EQ(eph.toe, 1128 * 300);
    EXPECT_EQ(eph.toc, eph.toe);
    EXPECT_DOUBLE_EQ(eph.delta_A, -123456.0 * TWO_N9);
    EXPECT_DOUBLE_EQ(eph.sqrtA, std::sqrt(CNAV_A_REF - 123456.0 * TWO_N9));
    EXPECT_DOUBLE_EQ(eph.Adot, 54321.0 * TWO_N21);
    EXPECT_DOUBLE_EQ(eph.delta_n, -12345.0 * TWO_N44 * GNSS_PI);
    EXPECT_DOUBLE_EQ(eph.delta_ndot, 1234.0 * TWO_N57 * GNSS_PI);
    EXPECT_DOUBLE_EQ(eph.M_0, -3000000000.0 * TWO_N32 * GNSS_PI);
    EXPECT_DOUBLE_EQ(eph.ecc, 123456789.0 * TWO_N34);
    EXPECT_DOUBLE_EQ(eph.omega, 2000000000.0 * TWO_N32 * GNSS_PI);
    EXPECT_DOUBLE_EQ(eph.OMEGA_0, -1000000000.0 * TWO_N32 * GNSS_PI);
    EXPECT_DOUBLE_EQ(eph.i_0, 1300000000.0 * TWO_N32 * GNSS_PI);
    EXPECT_DOUBLE_EQ(eph.OMEGAdot, CNAV_OMEGA_DOT_REF * GNSS_PI - 321.0 * TWO_N44 * GNSS_PI);
    EXPECT_DOUBLE_EQ(eph.idot, 77.0 * TWO_N44 * GNSS_PI);
    EXPECT_DOUBLE_EQ(eph.Cis, -55.0 * TWO_N30);
    EXPECT_DOUBLE_EQ(eph.Cic, 66.0 * TWO_N30);
    EXPECT_DOUBLE_EQ(eph.Crs, -777.0 * TWO_N8);
    EXPECT_DOUBLE_EQ(eph.Crc, 8888.0 * TWO_N8);
    EXPECT_DOUBLE_EQ(eph.Cus, 999.0 * TWO_N30);
    EXPECT_DOUBLE_EQ(eph.Cuc, -1111.0 * TWO_N30);
    EXPECT_EQ(eph.URANED0, -5);
    EXPECT_EQ(eph.URANED1, 3);
    EXPECT_EQ(eph.URANED2, 6);
    EXPECT_DOUBLE_EQ(eph.af0, -2345678.0 * TWO_N35);
    EXPECT_DOUBLE_EQ(eph.af1, 4567.0 * TWO_N48);
    EXPECT_DOUBLE_EQ(eph.af2, -12.0 * TWO_N60);
    EXPECT_DOUBLE_EQ(eph.TGD, -40.0 * TWO_N35);
    EXPECT_DOUBLE_EQ(eph.ISCL1CP, 21.0 * TWO_N35);
    EXPECT_DOUBLE_EQ(eph.ISCL1CD, 0.0);
    EXPECT_TRUE(eph.integrity_status_flag);
    EXPECT_EQ(eph.WNop, 94);
    // ISCs of the other signals, from page 1
    EXPECT_TRUE(eph.sf3_page1_iscs);
    EXPECT_DOUBLE_EQ(eph.ISCL1, -9.0 * TWO_N35);
    EXPECT_DOUBLE_EQ(eph.ISCL5I, 13.0 * TWO_N35);

    ASSERT_TRUE(nav.have_new_utc_model());
    const Gps_CNAV_Utc_Model utc = nav.get_utc_model();
    EXPECT_DOUBLE_EQ(utc.A0, -7.0 * TWO_N35);
    EXPECT_DOUBLE_EQ(utc.A1, 5.0 * TWO_N51);
    EXPECT_EQ(utc.DeltaT_LS, 18);
    EXPECT_EQ(utc.tot, 36000 * 16);
    EXPECT_EQ(utc.WN_T, 2398);
    EXPECT_EQ(utc.WN_LSF, 2185);
    EXPECT_EQ(utc.DN, 7);
    EXPECT_EQ(utc.DeltaT_LSF, 18);
    ASSERT_TRUE(nav.have_new_iono());
    const Gps_CNAV_Iono iono = nav.get_iono();
    EXPECT_DOUBLE_EQ(iono.alpha0, 12.0 * TWO_N30);
    EXPECT_DOUBLE_EQ(iono.alpha1, -3.0 * TWO_N27);
    EXPECT_DOUBLE_EQ(iono.beta0, 98.0 * TWO_P11);
    EXPECT_DOUBLE_EQ(iono.beta3, -2.0 * TWO_P16);

    // The same contents are not reported again, except for a new ITOW
    ASSERT_TRUE(nav.decode_frame(symbols.data()));
    EXPECT_FALSE(nav.have_new_ephemeris());

    // A frame with TOI = 0 does not provide an unambiguous time of week
    const std::vector<uint8_t> frame0 = build_frame(0, sf2, sf3);
    const std::vector<float> symbols0 = modulate(frame0, 0.1, 6);
    ASSERT_TRUE(nav.decode_frame(symbols0.data()));
    EXPECT_EQ(nav.get_toi(), 0);
    EXPECT_EQ(nav.get_frame_end_tow(), -1);

    // Subframe 3 pages from another satellite are ignored
    Gps_CNAV2_Navigation_Message other;
    other.set_prn(12);
    ASSERT_TRUE(other.decode_frame(symbols.data()));
    EXPECT_FALSE(other.have_new_utc_model());
    EXPECT_EQ(other.get_sf3_page(), -1);
}


TEST(GpsCnav2NavigationMessageTest, InvertedFrameFailsCrc)
{
    // The codes are not invariant to inversion: polarity must be resolved
    // before decoding
    std::vector<uint8_t> sf2(GPS_L1C_SF2_BITS, 0);
    set_field(sf2, CNAV2_WN, 2398);
    append_crc(sf2, GPS_L1C_SF2_BITS);
    std::vector<uint8_t> sf3(GPS_L1C_SF3_BITS, 0);
    set_field(sf3, CNAV2_SF3_PAGE, 6);
    append_crc(sf3, GPS_L1C_SF3_BITS);
    const std::vector<uint8_t> frame = build_frame(200, sf2, sf3);
    std::vector<float> symbols = modulate(frame, 0.0, 1);
    for (auto& s : symbols)
        {
            s = -s;
        }
    Gps_CNAV2_Navigation_Message nav;
    EXPECT_FALSE(nav.decode_frame(symbols.data()));
    // The TOI decoder sees the complemented word, with the opposite MSB
    EXPECT_EQ(nav.get_toi(), 200 ^ 0x100);
}


TEST(GpsCnav2NavigationMessageTest, FrameSearchIsSelective)
{
    // Two consecutive frames at Es/N0 = 3 dB: every misaligned 1800-symbol
    // window must fail the CRC, and the cheap candidate test must discard most
    // of them before the LDPC decoder runs
    std::mt19937 rng(3);
    std::vector<uint8_t> sf2(GPS_L1C_SF2_BITS, 0);
    for (int32_t i = 0; i < GPS_L1C_SF2_BITS - 24; i++)
        {
            sf2[i] = static_cast<uint8_t>(rng() & 1U);
        }
    append_crc(sf2, GPS_L1C_SF2_BITS);
    std::vector<uint8_t> sf3(GPS_L1C_SF3_BITS, 0);
    for (int32_t i = 0; i < GPS_L1C_SF3_BITS - 24; i++)
        {
            sf3[i] = static_cast<uint8_t>(rng() & 1U);
        }
    append_crc(sf3, GPS_L1C_SF3_BITS);
    std::vector<uint8_t> stream = build_frame(150, sf2, sf3);
    const std::vector<uint8_t> second = build_frame(151, sf2, sf3);
    stream.insert(stream.end(), second.cbegin(), second.cend());
    const std::vector<float> symbols = modulate(stream, std::pow(10.0, -3.0 / 20.0), 9);

    Gps_CNAV2_Navigation_Message nav;
    int32_t candidates = 0;
    for (int32_t offset = 1; offset < GPS_L1C_FRAME_SYMBOLS; offset++)
        {
            float quality = 0.0F;
            Gps_CNAV2_Navigation_Message::decode_toi(symbols.data() + offset, quality);
            if (quality >= GPS_L1C_TOI_MIN_QUALITY && nav.is_frame_candidate(symbols.data() + offset))
                {
                    candidates++;
                    EXPECT_FALSE(nav.decode_frame(symbols.data() + offset)) << "offset " << offset;
                }
        }
    EXPECT_LT(candidates, GPS_L1C_FRAME_SYMBOLS / 20);
    for (const int32_t offset : {0, GPS_L1C_FRAME_SYMBOLS})
        {
            EXPECT_TRUE(nav.is_frame_candidate(symbols.data() + offset));
            EXPECT_TRUE(nav.decode_frame(symbols.data() + offset));
            EXPECT_TRUE(nav.sf2_crc_ok());
            EXPECT_TRUE(nav.sf3_crc_ok());
            EXPECT_EQ(nav.get_toi(), offset == 0 ? 150 : 151);
        }
}


TEST(GpsCnav2NavigationMessageTest, FramesWithoutDataAreNotDecoded)
{
    // A frame with real data, so that its ITOW would be at hand
    std::vector<uint8_t> sf2(GPS_L1C_SF2_BITS, 0);
    set_field(sf2, CNAV2_WN, 2398);
    set_field(sf2, CNAV2_ITOW, 47);
    append_crc(sf2, GPS_L1C_SF2_BITS);
    std::vector<uint8_t> sf3(GPS_L1C_SF3_BITS, 0);
    set_field(sf3, CNAV2_SF3_PRN, 11);
    set_field(sf3, CNAV2_SF3_PAGE, 1);
    append_crc(sf3, GPS_L1C_SF3_BITS);
    Gps_CNAV2_Navigation_Message nav;
    nav.set_prn(11);
    ASSERT_TRUE(nav.decode_frame(modulate(build_frame(100, sf2, sf3), 0.0, 1).data()));
    EXPECT_FALSE(nav.sf2_has_no_data());
    EXPECT_FALSE(nav.sf3_has_no_data());
    EXPECT_EQ(nav.get_frame_end_tow(), 47 * 7200 + 100 * 18);
    ASSERT_TRUE(nav.have_new_ephemeris());
    ASSERT_TRUE(nav.have_new_iono());
    ASSERT_TRUE(nav.have_new_utc_model());

    // Default message: alternating ones and zeros beginning with one, with a
    // proper CRC (IS-GPS-800J, 3.5.1). Read as data, it would claim an
    // unhealthy L1C signal, ITOW = 85 and a subframe 3 from PRN 170
    std::vector<uint8_t> default_sf2(GPS_L1C_SF2_BITS, 0);
    for (int32_t i = 0; i < GPS_L1C_SF2_BITS - GPS_L1C_CRC_BITS; i++)
        {
            default_sf2[i] = static_cast<uint8_t>((i + 1) % 2);
        }
    append_crc(default_sf2, GPS_L1C_SF2_BITS);
    std::vector<uint8_t> default_sf3(GPS_L1C_SF3_BITS, 0);
    for (int32_t i = 0; i < GPS_L1C_SF3_BITS - GPS_L1C_CRC_BITS; i++)
        {
            default_sf3[i] = static_cast<uint8_t>((i + 1) % 2);
        }
    append_crc(default_sf3, GPS_L1C_SF3_BITS);
    ASSERT_TRUE(nav.decode_frame(modulate(build_frame(101, default_sf2, default_sf3), 0.0, 1).data()));
    EXPECT_TRUE(nav.sf2_crc_ok());
    EXPECT_TRUE(nav.sf3_crc_ok());
    EXPECT_TRUE(nav.sf2_has_no_data());
    EXPECT_TRUE(nav.sf3_has_no_data());
    EXPECT_EQ(nav.get_toi(), 101);
    EXPECT_EQ(nav.get_sf3_page(), -1);
    // Neither the TOW nor the ephemeris may come from it
    EXPECT_EQ(nav.get_frame_end_tow(), -1);
    EXPECT_FALSE(nav.have_new_ephemeris());
    EXPECT_FALSE(nav.have_new_iono());
    EXPECT_FALSE(nav.have_new_utc_model());
    EXPECT_EQ(nav.get_ephemeris().ITOW, 47);

    // An all-zero frame is a valid codeword with a null CRC and TOI = 0
    const std::vector<uint8_t> zero_sf2(GPS_L1C_SF2_BITS, 0);
    const std::vector<uint8_t> zero_sf3(GPS_L1C_SF3_BITS, 0);
    ASSERT_TRUE(nav.decode_frame(modulate(build_frame(0, zero_sf2, zero_sf3), std::pow(10.0, -3.0 / 20.0), 2).data()));
    EXPECT_TRUE(nav.sf2_crc_ok());
    EXPECT_TRUE(nav.sf3_crc_ok());
    EXPECT_TRUE(nav.sf2_has_no_data());
    EXPECT_TRUE(nav.sf3_has_no_data());
    EXPECT_EQ(nav.get_toi(), 0);
    EXPECT_EQ(nav.get_sf3_page(), -1);
    EXPECT_EQ(nav.get_frame_end_tow(), -1);
    EXPECT_FALSE(nav.have_new_ephemeris());
    EXPECT_FALSE(nav.have_new_iono());
    EXPECT_FALSE(nav.have_new_utc_model());
}


TEST(GpsCnav2NavigationMessageTest, Subframe3Page2Layout)
{
    // IS-GPS-800J Figure 3.5-3 (field renamed by IRN-IS-800J-003): the fields
    // follow the page number without gaps, up to the reserved bits 221-250.
    // Unlike CNAV message type 35, the GGTO ID comes before t_GGTO and WN_GGTO
    const std::vector<std::vector<std::pair<int32_t, int32_t>>> fields{
        CNAV2_GGTO_ID, CNAV2_T_GGTO, CNAV2_WN_GGTO, CNAV2_A0_GGTO, CNAV2_A1_GGTO,
        CNAV2_A2_GGTO, CNAV2_T_EOP, CNAV2_PM_X, CNAV2_PM_X_DOT, CNAV2_PM_Y,
        CNAV2_PM_Y_DOT, CNAV2_DELTA_UT1, CNAV2_DELTA_UT1_DOT};
    int32_t next = CNAV2_SF3_PAGE[0].first + CNAV2_SF3_PAGE[0].second;
    for (const auto& field : fields)
        {
            EXPECT_EQ(field[0].first, next);
            next = field[0].first + field[0].second;
        }
    EXPECT_EQ(next, 221);
    EXPECT_EQ(CNAV2_GGTO_ID[0].first, 15);
    EXPECT_EQ(CNAV2_T_GGTO[0].first, 18);
    EXPECT_EQ(CNAV2_WN_GGTO[0].first, 34);
}
