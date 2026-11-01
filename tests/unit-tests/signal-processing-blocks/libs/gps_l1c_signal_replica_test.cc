/*!
 * \file gps_l1c_signal_replica_test.cc
 * \brief Tests for the GPS L1C signal replica generator
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * Expected values from IS-GPS-800J, Tables 3.2-2 and 3.2-3.
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

#include "GPS_L1C.h"
#include "gps_l1c_signal_replica.h"
#include <gtest/gtest.h>
#include <array>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace
{
// Initial and final 24 chips (octal) of the L1CP and L1CD codes, Table 3.2-2
const std::array<std::array<const char*, 4>, GPS_L1C_NUMBER_OF_PRNS> GPS_L1C_ICD_24_CHIPS = {{
    {{"05752067", "20173742", "77001425", "52231646"}},  // PRN 1
    {{"70146401", "35437154", "23342754", "46703351"}},  // PRN 2
    {{"32066222", "00161056", "30523404", "00145161"}},  // PRN 3
    {{"72125121", "71435437", "03777635", "11261273"}},  // PRN 4
    {{"42323273", "15035661", "10505640", "71364603"}},  // PRN 5
    {{"01650642", "32606570", "42134174", "55012662"}},  // PRN 6
    {{"21303446", "03475644", "00471711", "30373701"}},  // PRN 7
    {{"35504263", "11316575", "32237045", "07706523"}},  // PRN 8
    {{"66434311", "23047575", "16004766", "71741157"}},  // PRN 9
    {{"52631623", "07355246", "66234727", "42347523"}},  // PRN 10
    {{"04733076", "15210113", "03755314", "12746122"}},  // PRN 11
    {{"50352603", "72643606", "20604227", "34634113"}},  // PRN 12
    {{"32026612", "63457333", "25477233", "47555063"}},  // PRN 13
    {{"07476042", "46623624", "32025443", "01221116"}},  // PRN 14
    {{"22210746", "35467322", "35503400", "37125437"}},  // PRN 15
    {{"30706376", "70116567", "70504407", "32203664"}},  // PRN 16
    {{"75764610", "62731643", "26163421", "62162634"}},  // PRN 17
    {{"73202225", "14040613", "52176727", "35012616"}},  // PRN 18
    {{"47227426", "07750525", "72557314", "00437232"}},  // PRN 19
    {{"16064126", "37171211", "62043206", "32130365"}},  // PRN 20
    {{"66415734", "01302134", "07151343", "51515733"}},  // PRN 21
    {{"27600270", "37672235", "16027175", "73662313"}},  // PRN 22
    {{"66101627", "32201230", "26267340", "55416712"}},  // PRN 23
    {{"17717055", "37437553", "36272365", "22550142"}},  // PRN 24
    {{"47500232", "23310544", "67707677", "31506062"}},  // PRN 25
    {{"52057615", "07152415", "07760374", "44603344"}},  // PRN 26
    {{"76153566", "02571041", "73633310", "05252052"}},  // PRN 27
    {{"22444670", "52270664", "30401257", "70603616"}},  // PRN 28
    {{"62330044", "61317104", "72606251", "51643216"}},  // PRN 29
    {{"13674337", "43137330", "37370402", "30417163"}},  // PRN 30
    {{"60635146", "20336467", "74255661", "20074570"}},  // PRN 31
    {{"73527653", "40745656", "10171147", "26204176"}},  // PRN 32
    {{"63772350", "50272475", "12242515", "07105451"}},  // PRN 33
    {{"33564215", "75604301", "17426100", "31062227"}},  // PRN 34
    {{"52236055", "52550266", "75647756", "36516016"}},  // PRN 35
    {{"64506521", "15334214", "71265340", "07641474"}},  // PRN 36
    {{"73561133", "53445703", "74355073", "35065520"}},  // PRN 37
    {{"12647121", "71136024", "45253014", "03155010"}},  // PRN 38
    {{"16640265", "01607455", "12452274", "34041736"}},  // PRN 39
    {{"11161337", "73467421", "07011213", "20162561"}},  // PRN 40
    {{"22055260", "54372454", "35143750", "01603755"}},  // PRN 41
    {{"11546064", "11526534", "26442600", "40541055"}},  // PRN 42
    {{"24765004", "16522173", "67214123", "64750626"}},  // PRN 43
    {{"14042504", "74053703", "62274362", "72550016"}},  // PRN 44
    {{"53512265", "52211303", "23371051", "36130364"}},  // PRN 45
    {{"15317006", "72655147", "25121057", "25236175"}},  // PRN 46
    {{"16151224", "01212152", "20362622", "43732204"}},  // PRN 47
    {{"67454561", "10410122", "33050463", "02316015"}},  // PRN 48
    {{"47542743", "22473073", "65334051", "00212370"}},  // PRN 49
    {{"65057230", "63145220", "65523456", "35163655"}},  // PRN 50
    {{"77415771", "65734110", "53741004", "33771603"}},  // PRN 51
    {{"75364651", "25167435", "66360341", "41161255"}},  // PRN 52
    {{"75664330", "17524136", "34421651", "76257261"}},  // PRN 53
    {{"44600202", "47064764", "04530741", "33512503"}},  // PRN 54
    {{"23211425", "14016156", "12621031", "16237466"}},  // PRN 55
    {{"51504740", "11723025", "62330452", "24120336"}},  // PRN 56
    {{"47712554", "76760325", "67510404", "11103121"}},  // PRN 57
    {{"67325233", "04724615", "00726605", "36467526"}},  // PRN 58
    {{"61517015", "72504743", "00200154", "66444010"}},  // PRN 59
    {{"43217554", "51215201", "37533004", "70455364"}},  // PRN 60
    {{"52520062", "00630473", "73771510", "26726105"}},  // PRN 61
    {{"77073716", "71217605", "44071707", "63663333"}},  // PRN 62
    {{"56350460", "50200707", "34665654", "42142704"}}   // PRN 63
}};

// Final 11 bits (octal) of the L1CO overlay codes, Table 3.2-3
const std::array<uint16_t, GPS_L1C_NUMBER_OF_PRNS> GPS_L1CO_ICD_FINAL_BITS = {{0410, 03153, 01767, 02134, 03510, 02260, 02433, 03520,
    02652, 02050, 070, 01605, 01247, 0773, 02377, 01525,
    01531, 03540, 0524, 01035, 03337, 0176, 0244, 01027,
    01753, 03502, 064, 02275, 044, 02777, 0367, 0535,
    03776, 02677, 0102, 02520, 02444, 03770, 01517, 01133,
    03754, 033, 01170, 01567, 03534, 02515, 0104, 03343,
    01510, 02170, 0710, 03375, 02650, 03307, 02262, 02161,
    02076, 01200, 0643, 02000, 03377, 01605, 01552}};


std::string octal_24_chips(const std::vector<int32_t>& chips, size_t first)
{
    uint32_t value = 0;
    for (size_t i = first; i < first + 24; i++)
        {
            value = (value << 1U) | static_cast<uint32_t>(chips[i]);
        }
    char buffer[9];
    snprintf(buffer, sizeof(buffer), "%08o", value);
    return std::string(buffer);
}
}  // namespace


TEST(GpsL1cSignalReplicaTest, RangingCodesMatchIcdTable)
{
    const auto length = static_cast<size_t>(GPS_L1C_CODE_LENGTH_CHIPS);
    std::vector<int32_t> chips(length);
    for (uint32_t prn = 1; prn <= static_cast<uint32_t>(GPS_L1C_NUMBER_OF_PRNS); prn++)
        {
            const auto& expected = GPS_L1C_ICD_24_CHIPS[prn - 1];
            gps_l1c_code_gen_int(chips, true, prn);
            EXPECT_EQ(octal_24_chips(chips, 0), expected[0]) << "L1CP PRN " << prn;
            EXPECT_EQ(octal_24_chips(chips, length - 24), expected[1]) << "L1CP PRN " << prn;
            gps_l1c_code_gen_int(chips, false, prn);
            EXPECT_EQ(octal_24_chips(chips, 0), expected[2]) << "L1CD PRN " << prn;
            EXPECT_EQ(octal_24_chips(chips, length - 24), expected[3]) << "L1CD PRN " << prn;
        }
}


TEST(GpsL1cSignalReplicaTest, ExpansionSequenceIsInsertedBeforeChipP)
{
    // PRN 3 L1CP has insertion index p = 1: the code starts with 0110100
    std::vector<int32_t> chips(static_cast<size_t>(GPS_L1C_CODE_LENGTH_CHIPS));
    gps_l1c_code_gen_int(chips, true, 3);
    const std::vector<int32_t> expansion{0, 1, 1, 0, 1, 0, 0};
    EXPECT_EQ(std::vector<int32_t>(chips.cbegin(), chips.cbegin() + 7), expansion);
}


TEST(GpsL1cSignalReplicaTest, OverlayCodesMatchIcdTable)
{
    for (uint32_t prn = 1; prn <= static_cast<uint32_t>(GPS_L1C_NUMBER_OF_PRNS); prn++)
        {
            const std::string code = gps_l1c_overlay_code(prn);
            ASSERT_EQ(code.size(), static_cast<size_t>(GPS_L1C_OVERLAY_CODE_LENGTH));
            const auto first = static_cast<uint16_t>(std::stoul(code.substr(0, 11), nullptr, 2));
            const auto last = static_cast<uint16_t>(std::stoul(code.substr(code.size() - 11), nullptr, 2));
            EXPECT_EQ(first, GPS_L1CO_INITIAL_BITS[prn - 1]) << "PRN " << prn;
            EXPECT_EQ(last, GPS_L1CO_ICD_FINAL_BITS[prn - 1]) << "PRN " << prn;
        }
    EXPECT_TRUE(gps_l1c_overlay_code(0).empty());
    EXPECT_TRUE(gps_l1c_overlay_code(64).empty());
}


TEST(GpsL1cSignalReplicaTest, TmbocPilotWaveform)
{
    const auto length = static_cast<int32_t>(GPS_L1C_CODE_LENGTH_CHIPS);
    std::vector<int32_t> chips(length);
    gps_l1c_code_gen_int(chips, true, 1);
    std::vector<float> pilot(length * GPS_L1C_TMBOC_SAMPLES_PER_CHIP);
    gps_l1c_code_gen_float(pilot, true, true, 1);
    std::vector<float> data(length * GPS_L1C_TMBOC_SAMPLES_PER_CHIP);
    gps_l1c_code_gen_float(data, false, true, 1);
    int32_t boc61_chips = 0;
    for (int32_t t = 0; t < length; t++)
        {
            const float chip = chips[t] == 0 ? 1.0F : -1.0F;
            const int32_t u = t % 33;
            const bool boc61 = (u == 0 || u == 4 || u == 6 || u == 29);
            boc61_chips += boc61 ? 1 : 0;
            for (int32_t k = 0; k < GPS_L1C_TMBOC_SAMPLES_PER_CHIP; k++)
                {
                    const float boc11 = k < 6 ? chip : -chip;
                    const float expected = boc61 ? ((k % 2 == 0) ? chip : -chip) : boc11;
                    ASSERT_EQ(pilot[t * GPS_L1C_TMBOC_SAMPLES_PER_CHIP + k], expected) << "chip " << t << " sample " << k;
                }
            // The data component is BOC(1,1) only
            const float data_chip = data[t * GPS_L1C_TMBOC_SAMPLES_PER_CHIP] > 0.0F ? 1.0F : -1.0F;
            for (int32_t k = 0; k < GPS_L1C_TMBOC_SAMPLES_PER_CHIP; k++)
                {
                    ASSERT_EQ(data[t * GPS_L1C_TMBOC_SAMPLES_PER_CHIP + k], k < 6 ? data_chip : -data_chip);
                }
        }
    EXPECT_EQ(boc61_chips, 4 * 310);
}


TEST(GpsL1cSignalReplicaTest, Boc11WaveformAndSampledShift)
{
    const auto length = static_cast<int32_t>(GPS_L1C_CODE_LENGTH_CHIPS);
    std::vector<int32_t> chips(length);
    gps_l1c_code_gen_int(chips, false, 7);
    std::vector<float> boc(length * GPS_L1C_BOC_SAMPLES_PER_CHIP);
    gps_l1c_code_gen_float(boc, false, false, 7);
    for (int32_t t = 0; t < length; t++)
        {
            const float chip = chips[t] == 0 ? 1.0F : -1.0F;
            ASSERT_EQ(boc[2 * t], chip);
            ASSERT_EQ(boc[2 * t + 1], -chip);
        }

    // At exactly two samples per chip, the sampled replica is the waveform,
    // and chip_shift delays it by that number of chips
    constexpr int32_t fs = 2046000;
    std::vector<std::complex<float>> sampled(length * 2);
    gps_l1c_code_gen_complex_sampled(sampled, false, false, 7, fs, 0);
    for (int32_t i = 0; i < length * 2; i++)
        {
            ASSERT_EQ(sampled[i].real(), boc[i]);
            ASSERT_EQ(sampled[i].imag(), 0.0F);
        }
    constexpr uint32_t shift = 100;
    gps_l1c_code_gen_complex_sampled(sampled, false, false, 7, fs, shift);
    for (int32_t i = 0; i < length * 2; i++)
        {
            ASSERT_EQ(sampled[(i + 2 * (length - shift)) % (2 * length)].real(), boc[i]);
        }
}
