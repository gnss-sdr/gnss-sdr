/*!
 * \file BEIDOU_B2a.h
 * \brief  Defines system parameters for Beidou B2a signal and CNAV2 data
 * \note Code added as part of GSoC 2018 program
 * \author Dong Kyeong Lee, 2018. dole7890(at)colorado.edu
 * \see <a href="http://m.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf">beidou ICD</a>
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_BEIDOU_B2a_H_
#define GNSS_SDR_BEIDOU_B2a_H_

#include "MATH_CONSTANTS.h"
#include "gnss_frequencies.h"
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>


constexpr double BEIDOU_CNAV2_PI = 3.1415926535898;           //!< BeiDou CNAV2 Pi
constexpr double BEIDOU_CNAV2_PREAMBLE_DURATION_S = 0.120;    //[s]
constexpr int32_t BEIDOU_CNAV2_NBR_SATS = 63;                 // Total number of satellites
constexpr int32_t BEIDOU_CNAV2_PREAMBLE_LENGTH_SYMBOLS = 24;  //[symbols]
constexpr int32_t BEIDOU_CNAV2_PREAMBLE_PERIOD_SYMBOLS = 600;
constexpr int32_t BEIDOU_CNAV2_TELEMETRY_SYMBOLS_PER_BIT = 5;           //spb
constexpr int32_t BEIDOU_CNAV2_TELEMETRY_SYMBOLS_PER_PREAMBLE_BIT = 5;  //spb
constexpr int32_t BEIDOU_CNAV2_FRAME_SYMBOLS = 600;                     //Number of symbols per string in the CNAV2 message
constexpr int32_t BEIDOU_CNAV2_DATA_BITS = 288;
constexpr int32_t BEIDOU_CNAV2_DATA_BYTES = 36;  //Number of bits per string in the CNAV2 message
constexpr int32_t BEIDOU_CNAV2_CODES_PER_SYMBOLS = 5;
constexpr int32_t BEIDOU_CNAV2_CRC_BITS = 24;
constexpr int32_t BEIDOU_CNAV2_CRC_SEED = 0;
constexpr int32_t BEIDOU_CNAV2_CRC_POLY = 0x1864CFB;
constexpr int32_t BEIDOU_CNAV2_BDT2GPST_LEAP_SEC_OFFSET = 14;    //!< Number of leap seconds passed from the start of the GPS epoch up to the start of BeiDou epoch
constexpr int32_t BEIDOU_CNAV2_BDT2GPST_WEEK_NUM_OFFSET = 1356;  //!< Number of weeks passed from the start of the GPS epoch up to the start of BeiDou epoch
constexpr char BEIDOU_CNAV2_PREAMBLE[BEIDOU_CNAV2_PREAMBLE_LENGTH_SYMBOLS + 1] = "111000100100110111101000";

constexpr double BEIDOU_B2a_FREQ_HZ = FREQ5;              //!< BEIDOU B2a carrier frequency [Hz]
constexpr double BEIDOU_B2ad_CODE_PERIOD = 0.001;         //!< BEIDOU B2a C/A code period [seconds]
constexpr double BEIDOU_B2ad_CODE_PERIOD_MS = 1;          //!< BEIDOU B2a C/A code period [mseconds]
constexpr double BEIDOU_B2ad_CODE_RATE_HZ = 10.23e6;      //!< BEIDOU_B2a data code rate [chips/s]
constexpr int32_t BEIDOU_B2ad_CODE_LENGTH_CHIPS = 10230;  //!< BEIDOU_B2a data  code length [chips]
constexpr int32_t BEIDOU_B2ad_SECONDARY_CODE_LENGTH = 5;  //!< Each bit is 1 ms (one primary code sequence)
constexpr char BEIDOU_B2ad_SECONDARY_CODE[6] = "00010";

constexpr double BEIDOU_B2ap_CODE_RATE_HZ = 10.23e6;                       //!< BEIDOU_B2a pilot code rate [chips/s]
constexpr int32_t BEIDOU_B2ap_CODE_LENGTH_CHIPS = 10230;                   //!< BEIDOU_B2a pilot code length [chips]
constexpr int32_t BEIDOU_B2ap_SECONDARY_CODE_LENGTH = 100;                 //!< B2a pilot code is 100 chips long; Each bit is 1 ms (one primary code sequence)
constexpr int32_t BEIDOU_B2ap_SECONDARY_WEIL_CODE_LENGTH = 1021;           //!< B2a pilot Weil code is 1021 chips long; Each bit is 1 ms (one primary code sequence)
constexpr char BEIDOU_B2ap_SECONDARY_CODE[BEIDOU_CNAV2_NBR_SATS][1022] =  //!< B2ap Secondary Codes. See b2ap_sec_code_gen.m
    {
        "0110100001100110001010001001111100001101110111110101111100100000010111101101110101011010010001110111",
        "1010010000110101011011000010000011000111011010101011000110100100010000100011001110101000111111010011",
        "0101100000110010111011010010010001110000001100100011110001010111000101100001000011010100100000111101",
        "0000000000011101101110101100010001101100011111100110110010101111000010000110101001100110111101010101",
        "1000111000011001110010101111110010011001001010001000101110000101011101000001100110110000100101101101",
        "0000110110010111101010111101110010011101111000100010001111111001101000000100101110010000010000100010",
        "1000010010000111101010111111011011000100011110011111011101100011010000110100111001000000111100101000",
        "1000100111110001001011001010010001111100111010010010111111111100111001000100011100010101110111100111",
        "0001100100110010001010010100000100001101011101011010111001100001011110011100100000100011000000111111",
        "0111110001001111011110001011111000100011110110010100100100011110100000010110000110100100010110001111",
        "0111100101000101000110101001100101111101110101001001011011011010000101110110001110011001100100100000",
        "1100101100000001011100111110011110011100000110010010110000000011001001010100000101011010001001010011",
        "1111111000010011011000100100001000101001110001110101100111010010010010011010101110101111011011101010",
        "1000011101011001111110100010100010100010011100111110011110000100101100010011101101111011000111111110",
        "1100110101011010111010100101110101111100000001010100010011111011011110010011000001011010100001100110",
        "0011100000111001001010011101001010000111011100111010100000111010111110101110001111101000000101011001",
        "1011101111010111000110101111110010100100100111001000001100111101001110101000110110110011100100101011",
        "1100101101100001110100101101100110001110011001101000100101011110001110111111011111010100000001101000",
        "0010010110000001110011000111001110011110010110101000010011111000111000000100011010110111011001000001",
        "1001101011101001101110000001110111101101110110010000000101001001011111001111111110110100011000111110",
        "1010011001010111110010000001000100110000010110110001000010000000000110100011100001010011110100011111",
        "1111011010100001111110110111110110011011100000001010100010001010100110110001100111001010110000111011",
        "1011010010001011011111101101000101111000011000001100110001011001111111100101010100110000101100100011",
        "0110010001010000110100110101110001111011010010111101011010110100000110111010000111011100111000110111",
        "1111100000110000101111000110101100100001001011011111000001011110010011001010100001100111000100110010",
        "1100011011111101110011010100010111110010000101000101101100000100100110100110000111101101010100010011",
        "0100010111010111100101110110010100000111001001111011011011001110010100001010001101011000110011110000",
        "0010010110101101100100010000011010011000011111101010100110101100101111011101100011101000111000100001",
        "1111110110001001000101101101111110100110011101010010100110001111000001011010001010101011111110101001",
        "0101101011101010111010100110011011000100000001011110100001010000111101011001011010011110010011100111",
        "0111000010111010101100011111010000101100010101111000111111011010010000001000001100101101000100000110",
        "0110001000001111011101101111110011001110101001010011100110111000101100000111110000000001100001100011",
        "1010100010010110111111000111001111011001000001111001011001100001111110011001110001001001110001000010",
        "1011000011001010100111010110111100100000011010110101000100110010010111110111010000111000010010011110",
        "1100110010000000010001000000100100110000001101100001111010100011111110000011010011100101101010011001",
        "0111010110011111001010100100101110011001110100011110111011010111000011010010001111011101010101111001",
        "0010001110011000000110100000101000100011011110010110011110010010111100101010000111100001111111100001",
        "1000111100000101000100111100100001110110100001000101010011110101110011110110100111100001101101110100",
        "0010111110000001010010011010011111100110011111100001100100100111100110010111011110101101000000100110",
        "1101111001000101101011000001101101000100100100000010111001111100110010010110100001110001101010011000",
        "1000101100100010110000010010111010100010001111100110000100101000100001010000111000010111000100001001",
        "0101011000010111010110101110100111110111001011100001010010001100100000000001111011101010111001000011",
        "1110111001111011110011010000010001001101111111000010011100011011111011011011010000011100100010000101",
        "1100001100000001100010000000001110111000011101000000011101011101001011000011011011100111000000101010",
        "0100100111100100101110011001111101000010000000000011100000110111011010000100111011010001011001111101",
        "1110110111000010111110001010010001001000101000101010111000001010111110101001010001001111101110010100",
        "1111101000010101001100111010101100000100000001101010111101101000111101010010111001001111100110100000",
        "0010001001111011010100100011001100000101111100100101010110000011101011100010111111011011110011000110",
        "0110011101100100111100010100010000110111101010100100100011010001011110111011101010110100101111111010",
        "1110100011101000111000011000010010101110101001001101101110110101100110010110001000001110110110011110",
        "0000111100000001110000111101000101110010000010110101000111001011000001101010110010100100010010101010",
        "0010101110111000100001110001111001001111110010111010000100110100001011110010100111010000101111111110",
        "1101101111001000100111101101100000110001110001000011111110000110010111010010110111000101011111000111",
        "1101100111011001100010110100011100001010111011111011001111010101101101111001001010001000011011111101",
        "1000101110010001001011111001101011011110000101110011101100000001100001000100000001011000100010111110",
        "1110101111001000111101000001110001110001001000110011101001010101100100010000110010010010011111000111",
        "1111101110100001100101011011011010100111110011010011100001101010101000011000000011001001001100101011",
        "1001101101000110101111100001100110010110110011010111101110110010000100110000011100010101000000011111",
        "1010111010101010100011010010100001111001010000000111100010100001011111101001111001101001100010010100",
        "1000101001010111000000100100001010001001010000011001111101010101011001110001011110110010000000000001",
        "0101101100001001111011001010111100110111001101011010011000000100000101010000111000101000010100000110",
        "0111010000101110000100011100100000101001111100100111101000011101000001010100000111011100100110011110",
        "0010100001110110110011110010111101110001001010011001001010001001100001101010011000010110100010001010",
};


//!< Initialization registers for the primary codes for B2a data signal
constexpr int8_t BEIDOU_B2ad_INIT_REG[BEIDOU_CNAV2_NBR_SATS][13] =
    {
        {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0},
        {1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1},
        {1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1},
        {1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
        {1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0},
        {1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0},
        {1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1},
        {1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1},
        {1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0},
        {1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1},
        {1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
        {1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1},
        {1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1},
        {1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0},
        {1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1},
        {1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1},
        {1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0},
        {1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1},
        {1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1},
        {1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1},
        {1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0},
        {1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0},
        {1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0},
        {1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0},
        {1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0},
        {1, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0},
        {1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1},
        {1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0},
        {1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0},
        {1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1},
        {1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1},
        {1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0},
        {1, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1},
        {1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1},
        {1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1},
        {1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1},
        {1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1},
        {1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0},
        {1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1},
        {1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1},
        {1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1},
        {1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0},
        {1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1},
        {1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1},
        {1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1},
        {1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0},
        {1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1},
        {1, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1},
        {1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0},
        {1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0},
        {1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1},
        {1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0},
        {1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0},
        {1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1},
        {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
        {1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1},
        {0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0},


};

constexpr int8_t BEIDOU_B2ap_INIT_REG[BEIDOU_CNAV2_NBR_SATS][13] =
    {
        {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0},
        {1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1},
        {1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1},
        {1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
        {1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0},
        {1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0},
        {1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1},
        {1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1},
        {1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0},
        {1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1},
        {1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
        {1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1},
        {1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1},
        {1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0},
        {1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1},
        {1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1},
        {1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0},
        {1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1},
        {1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1},
        {1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1},
        {1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0},
        {1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0},
        {1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0},
        {1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0},
        {1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0},
        {1, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0},
        {1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1},
        {1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0},
        {1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0},
        {1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1},
        {1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1},
        {1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0},
        {1, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1},
        {1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1},
        {1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1},
        {1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1},
        {1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1},
        {1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0},
        {1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1},
        {1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1},
        {1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1},
        {1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0},
        {1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1},
        {1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1},
        {1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1},
        {1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0},
        {1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1},
        {1, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1},
        {1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0},
        {1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0},
        {1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1},
        {1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0},
        {1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0},
        {1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1},
        {1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0},
        {0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0},
        {0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1},
};

constexpr int32_t BEIDOU_B2ap_SECONDARY_TRUNCATION_POINT[BEIDOU_CNAV2_NBR_SATS] =
    {
        138, 570, 351, 77, 885, 247, 413, 180, 3, 26,
        17, 172, 30, 1008, 646, 158, 170, 99, 53, 179,
        925, 114, 10, 584, 60, 3, 684, 263, 545, 22,
        546, 190, 303, 234, 38, 822, 57, 668, 697, 93,
        18, 66, 318, 133, 98, 70, 132, 26, 354, 58,
        41, 182, 944, 205, 23, 1, 792, 641, 83, 7,
        111, 96, 92};

constexpr int32_t BEIDOU_B2ap_SECONDARY_PHASE_DIFFERENCE[BEIDOU_CNAV2_NBR_SATS] =
    {
        123, 55, 40, 139, 31, 175, 350, 450, 478, 8,
        73, 97, 213, 407, 476, 4, 15, 47, 163, 280,
        322, 353, 375, 510, 332, 7, 13, 16, 18, 25,
        50, 81, 118, 127, 132, 134, 164, 177, 208, 249,
        276, 349, 439, 477, 498, 88, 155, 330, 3, 21,
        84, 111, 128, 153, 197, 199, 214, 256, 265, 291,
        324, 326, 340};

// BEIDOU CNAV2 NAVIGATION MESSAGE STRUCTURE
// NAVIGATION MESSAGE FIELDS POSITIONS

// CRC
const std::vector<std::pair<int32_t, int32_t>> CRC24({{264, 24}});

// Common
const std::vector<std::pair<int32_t, int32_t>> PRN({{1, 6}});
const std::vector<std::pair<int32_t, int32_t>> MesType({{7, 6}});
const std::vector<std::pair<int32_t, int32_t>> SOW({{13, 18}});
const std::vector<std::pair<int32_t, int32_t>> CRC({{265, 24}});

// Type 10 (288 bits)
const std::vector<std::pair<int32_t, int32_t>> WN_10({{31, 13}});
const std::vector<std::pair<int32_t, int32_t>> DIF_10({{44, 1}});
const std::vector<std::pair<int32_t, int32_t>> SIF_10({{45, 1}});
const std::vector<std::pair<int32_t, int32_t>> AIF_10({{46, 1}});
const std::vector<std::pair<int32_t, int32_t>> SISMAI_10({{47, 4}});
const std::vector<std::pair<int32_t, int32_t>> DIF_B1C_10({{51, 1}});
const std::vector<std::pair<int32_t, int32_t>> SIF_B1C_10({{52, 1}});
const std::vector<std::pair<int32_t, int32_t>> AIF_B1C_10({{53, 1}});
const std::vector<std::pair<int32_t, int32_t>> IODE_10({{54, 8}});
// Ephemeris I (203 bits)
const std::vector<std::pair<int32_t, int32_t>> t_oe_10({{62, 11}});
const std::vector<std::pair<int32_t, int32_t>> SatType_10({{73, 2}});
const std::vector<std::pair<int32_t, int32_t>> dA_10({{75, 26}});
const std::vector<std::pair<int32_t, int32_t>> A_dot_10({{101, 25}});
const std::vector<std::pair<int32_t, int32_t>> dn_0_10({{126, 17}});
const std::vector<std::pair<int32_t, int32_t>> dn_0_dot_10({{143, 23}});
const std::vector<std::pair<int32_t, int32_t>> M_0_10({{166, 33}});
const std::vector<std::pair<int32_t, int32_t>> e_10({{199, 33}});
const std::vector<std::pair<int32_t, int32_t>> omega_10({{232, 33}});
// Ephemeris I End

// Type 11 (288 bits)
const std::vector<std::pair<int32_t, int32_t>> HS_11({{31, 2}});
const std::vector<std::pair<int32_t, int32_t>> DIF_11({{33, 1}});
const std::vector<std::pair<int32_t, int32_t>> SIF_11({{34, 1}});
const std::vector<std::pair<int32_t, int32_t>> AIF_11({{35, 1}});
const std::vector<std::pair<int32_t, int32_t>> SISMAI_11({{36, 4}});
const std::vector<std::pair<int32_t, int32_t>> DIF_B1C_11({{40, 1}});
const std::vector<std::pair<int32_t, int32_t>> SIF_B1C_11({{41, 1}});
const std::vector<std::pair<int32_t, int32_t>> AIF_B1C_11({{42, 1}});
// Ephemeris II (222 bits)
const std::vector<std::pair<int32_t, int32_t>> Omega_0_11({{43, 33}});
const std::vector<std::pair<int32_t, int32_t>> i_0_11({{76, 33}});
const std::vector<std::pair<int32_t, int32_t>> Omega_dot_11({{109, 19}});
const std::vector<std::pair<int32_t, int32_t>> i_0_dot_11({{128, 15}});
const std::vector<std::pair<int32_t, int32_t>> C_IS_11({{143, 16}});
const std::vector<std::pair<int32_t, int32_t>> C_IC_11({{159, 16}});
const std::vector<std::pair<int32_t, int32_t>> C_RS_11({{175, 24}});
const std::vector<std::pair<int32_t, int32_t>> C_RC_11({{199, 24}});
const std::vector<std::pair<int32_t, int32_t>> C_US_11({{223, 21}});
const std::vector<std::pair<int32_t, int32_t>> C_UC_11({{244, 21}});
// Ephemeris II End


// Type 30 (288 bits)
const std::vector<std::pair<int32_t, int32_t>> HS_30({{31, 2}});
const std::vector<std::pair<int32_t, int32_t>> DIF_30({{33, 1}});
const std::vector<std::pair<int32_t, int32_t>> SIF_30({{34, 1}});
const std::vector<std::pair<int32_t, int32_t>> AIF_30({{35, 1}});
const std::vector<std::pair<int32_t, int32_t>> SISMAI_30({{36, 4}});
const std::vector<std::pair<int32_t, int32_t>> DIF_B1C_30({{40, 1}});
const std::vector<std::pair<int32_t, int32_t>> SIF_B1C_30({{41, 1}});
const std::vector<std::pair<int32_t, int32_t>> AIF_B1C_30({{42, 1}});
// Clock Correction Parameters (69 bits)
const std::vector<std::pair<int32_t, int32_t>> t_oc_30({{43, 11}});
const std::vector<std::pair<int32_t, int32_t>> a_0_30({{54, 25}});
const std::vector<std::pair<int32_t, int32_t>> a_1_30({{79, 22}});
const std::vector<std::pair<int32_t, int32_t>> a_2_30({{101, 11}});
// Clock Correction Parameters End
const std::vector<std::pair<int32_t, int32_t>> IODC_30({{112, 10}});
const std::vector<std::pair<int32_t, int32_t>> T_GDB2ap_30({{122, 12}});
const std::vector<std::pair<int32_t, int32_t>> ISC_B2ad_30({{134, 12}});
// Ionospheric Delay Correction Model Parameters (74 bits)
const std::vector<std::pair<int32_t, int32_t>> alpha_1_30({{146, 10}});
const std::vector<std::pair<int32_t, int32_t>> alpha_2_30({{156, 8}});
const std::vector<std::pair<int32_t, int32_t>> alpha_3_30({{164, 8}});
const std::vector<std::pair<int32_t, int32_t>> alpha_4_30({{172, 8}});
const std::vector<std::pair<int32_t, int32_t>> alpha_5_30({{180, 8}});
const std::vector<std::pair<int32_t, int32_t>> alpha_6_30({{188, 8}});
const std::vector<std::pair<int32_t, int32_t>> alpha_7_30({{196, 8}});
const std::vector<std::pair<int32_t, int32_t>> alpha_8_30({{204, 8}});
const std::vector<std::pair<int32_t, int32_t>> alpha_9_30({{212, 8}});
// Ionospheric Delay Correction Model Parameters End
const std::vector<std::pair<int32_t, int32_t>> T_GDB1Cp_30({{220, 12}});
const std::vector<std::pair<int32_t, int32_t>> Rev_30({{232, 33}});


// Type 31 (288 bits)
const std::vector<std::pair<int32_t, int32_t>> HS_31({{31, 2}});
const std::vector<std::pair<int32_t, int32_t>> DIF_31({{33, 1}});
const std::vector<std::pair<int32_t, int32_t>> SIF_31({{34, 1}});
const std::vector<std::pair<int32_t, int32_t>> AIF_31({{35, 1}});
const std::vector<std::pair<int32_t, int32_t>> SISMAI_31({{36, 4}});
const std::vector<std::pair<int32_t, int32_t>> DIF_B1C_31({{40, 1}});
const std::vector<std::pair<int32_t, int32_t>> SIF_B1C_31({{41, 1}});
const std::vector<std::pair<int32_t, int32_t>> AIF_B1C_31({{42, 1}});
// Clock Correction Parameters (69 bits)
const std::vector<std::pair<int32_t, int32_t>> t_oc_31({{43, 11}});
const std::vector<std::pair<int32_t, int32_t>> a_0_31({{54, 25}});
const std::vector<std::pair<int32_t, int32_t>> a_1_31({{79, 22}});
const std::vector<std::pair<int32_t, int32_t>> a_2_31({{101, 11}});
// Clock Correction Parameters End
const std::vector<std::pair<int32_t, int32_t>> IODC_31({{112, 10}});
const std::vector<std::pair<int32_t, int32_t>> WN_a_31({{122, 13}});
const std::vector<std::pair<int32_t, int32_t>> t_oa_31({{135, 8}});
// Reduced Almanac Parameters Sat 1(38 bits)
const std::vector<std::pair<int32_t, int32_t>> PRN_a1_31({{143, 6}});
const std::vector<std::pair<int32_t, int32_t>> SatType1_31({{149, 2}});
const std::vector<std::pair<int32_t, int32_t>> delta_A1_31({{151, 8}});
const std::vector<std::pair<int32_t, int32_t>> Omega_01_31({{159, 7}});
const std::vector<std::pair<int32_t, int32_t>> Phi_01_31({{166, 7}});
const std::vector<std::pair<int32_t, int32_t>> Health1_31({{173, 8}});
// Reduced Almanac Parameters End
// Reduced Almanac Parameters Sat 2(38 bits)
const std::vector<std::pair<int32_t, int32_t>> PRN_a2_31({{181, 6}});
const std::vector<std::pair<int32_t, int32_t>> SatType2_31({{187, 2}});
const std::vector<std::pair<int32_t, int32_t>> delta_A2_31({{189, 8}});
const std::vector<std::pair<int32_t, int32_t>> Omega_02_31({{197, 7}});
const std::vector<std::pair<int32_t, int32_t>> Phi_02_31({{204, 7}});
const std::vector<std::pair<int32_t, int32_t>> Health2_31({{211, 8}});
// Reduced Almanac Parameters End
// Reduced Almanac Parameters Sat 3(38 bits)
const std::vector<std::pair<int32_t, int32_t>> PRN_a3_31({{219, 6}});
const std::vector<std::pair<int32_t, int32_t>> SatType3_31({{225, 2}});
const std::vector<std::pair<int32_t, int32_t>> delta_A3_31({{227, 8}});
const std::vector<std::pair<int32_t, int32_t>> Omega_03_31({{235, 7}});
const std::vector<std::pair<int32_t, int32_t>> Phi_03_31({{242, 7}});
const std::vector<std::pair<int32_t, int32_t>> Health3_31({{249, 8}});
// Reduced Almanac Parameters End
const std::vector<std::pair<int32_t, int32_t>> Rev_31({{257, 8}});


// Type 32 (288 bits)
const std::vector<std::pair<int32_t, int32_t>> HS_32({{31, 2}});
const std::vector<std::pair<int32_t, int32_t>> DIF_32({{33, 1}});
const std::vector<std::pair<int32_t, int32_t>> SIF_32({{34, 1}});
const std::vector<std::pair<int32_t, int32_t>> AIF_32({{35, 1}});
const std::vector<std::pair<int32_t, int32_t>> SISMAI_32({{36, 4}});
const std::vector<std::pair<int32_t, int32_t>> DIF_B1C_32({{40, 1}});
const std::vector<std::pair<int32_t, int32_t>> SIF_B1C_32({{41, 1}});
const std::vector<std::pair<int32_t, int32_t>> AIF_B1C_32({{42, 1}});
// Clock Correction Parameters (69 bits)
const std::vector<std::pair<int32_t, int32_t>> t_oc_32({{43, 11}});
const std::vector<std::pair<int32_t, int32_t>> a_0_32({{54, 25}});
const std::vector<std::pair<int32_t, int32_t>> a_1_32({{79, 22}});
const std::vector<std::pair<int32_t, int32_t>> a_2_32({{101, 11}});
// Clock Correction Parameters End
const std::vector<std::pair<int32_t, int32_t>> IODC_32({{112, 10}});
// EOP Parameters (138 bits)
const std::vector<std::pair<int32_t, int32_t>> t_EOP_32({{122, 16}});
const std::vector<std::pair<int32_t, int32_t>> PM_X_32({{138, 21}});
const std::vector<std::pair<int32_t, int32_t>> PM_X_dot_32({{159, 15}});
const std::vector<std::pair<int32_t, int32_t>> PM_Y_32({{174, 21}});
const std::vector<std::pair<int32_t, int32_t>> PM_Y_dot_32({{195, 15}});
const std::vector<std::pair<int32_t, int32_t>> dUT1_32({{210, 31}});
const std::vector<std::pair<int32_t, int32_t>> dUT1_dot_32({{241, 19}});
// EOP Parameters End
const std::vector<std::pair<int32_t, int32_t>> Rev_32({{260, 5}});


// Type 33 (288 bits)
const std::vector<std::pair<int32_t, int32_t>> HS_33({{31, 2}});
const std::vector<std::pair<int32_t, int32_t>> DIF_33({{33, 1}});
const std::vector<std::pair<int32_t, int32_t>> SIF_33({{34, 1}});
const std::vector<std::pair<int32_t, int32_t>> AIF_33({{35, 1}});
const std::vector<std::pair<int32_t, int32_t>> SISMAI_33({{36, 4}});
const std::vector<std::pair<int32_t, int32_t>> DIF_B1C_33({{40, 1}});
const std::vector<std::pair<int32_t, int32_t>> SIF_B1C_33({{41, 1}});
const std::vector<std::pair<int32_t, int32_t>> AIF_B1C_33({{42, 1}});
// Clock Correction Parameters (69 bits)
const std::vector<std::pair<int32_t, int32_t>> t_oc_33({{43, 11}});
const std::vector<std::pair<int32_t, int32_t>> a_0_33({{54, 25}});
const std::vector<std::pair<int32_t, int32_t>> a_1_33({{79, 22}});
const std::vector<std::pair<int32_t, int32_t>> a_2_33({{101, 11}});
// Clock Correction Parameters End
// BGTO Parameters (68 bits)
const std::vector<std::pair<int32_t, int32_t>> GNSS_ID_33({{112, 3}});
const std::vector<std::pair<int32_t, int32_t>> WN_0BGTO_33({{115, 13}});
const std::vector<std::pair<int32_t, int32_t>> t_0BGTO_33({{128, 16}});
const std::vector<std::pair<int32_t, int32_t>> A_0BGTO_33({{144, 16}});
const std::vector<std::pair<int32_t, int32_t>> A_1BGTO_33({{160, 13}});
const std::vector<std::pair<int32_t, int32_t>> A_2BGTO_33({{173, 7}});
// BGTO Parameters End
// Reduced Almanac Parameters (38 bits)
const std::vector<std::pair<int32_t, int32_t>> PRN_ALM_33({{180, 6}});
const std::vector<std::pair<int32_t, int32_t>> SatType_33({{186, 2}});
const std::vector<std::pair<int32_t, int32_t>> delta_A_33({{188, 8}});
const std::vector<std::pair<int32_t, int32_t>> Omega_0_33({{196, 7}});
const std::vector<std::pair<int32_t, int32_t>> Phi_0_33({{203, 7}});
const std::vector<std::pair<int32_t, int32_t>> Health_33({{210, 8}});
// Reduced Almanac Parameters End
const std::vector<std::pair<int32_t, int32_t>> IODC_33({{218, 10}});
const std::vector<std::pair<int32_t, int32_t>> WN_ALM_33({{228, 13}});
const std::vector<std::pair<int32_t, int32_t>> t_oa_33({{241, 8}});
const std::vector<std::pair<int32_t, int32_t>> Rev_33({{249, 16}});


// Type 34 (288 bits)
const std::vector<std::pair<int32_t, int32_t>> HS_34({{31, 2}});
const std::vector<std::pair<int32_t, int32_t>> DIF_34({{33, 1}});
const std::vector<std::pair<int32_t, int32_t>> SIF_34({{34, 1}});
const std::vector<std::pair<int32_t, int32_t>> AIF_34({{35, 1}});
const std::vector<std::pair<int32_t, int32_t>> SISMAI_34({{36, 4}});
const std::vector<std::pair<int32_t, int32_t>> DIF_B1C_34({{40, 1}});
const std::vector<std::pair<int32_t, int32_t>> SIF_B1C_34({{41, 1}});
const std::vector<std::pair<int32_t, int32_t>> AIF_B1C_34({{42, 1}});
// SISAI_OC (22 bits)
const std::vector<std::pair<int32_t, int32_t>> t_op_34({{43, 11}});
const std::vector<std::pair<int32_t, int32_t>> SISAI_ocb_34({{54, 5}});
const std::vector<std::pair<int32_t, int32_t>> SISAI_oc1_34({{59, 3}});
const std::vector<std::pair<int32_t, int32_t>> SISAI_oc2_34({{62, 3}});
// SISAI_OC End
// Clock Correction Parameters (69 bits)
const std::vector<std::pair<int32_t, int32_t>> t_oc_34({{65, 11}});
const std::vector<std::pair<int32_t, int32_t>> a_0_34({{76, 25}});
const std::vector<std::pair<int32_t, int32_t>> a_1_34({{101, 22}});
const std::vector<std::pair<int32_t, int32_t>> a_2_34({{123, 11}});
// Clock Correction Parameters End
const std::vector<std::pair<int32_t, int32_t>> IODC_34({{134, 10}});
// BDT-UTC Time Offset Parameters (97 bits)
const std::vector<std::pair<int32_t, int32_t>> A_0UTC_34({{144, 16}});
const std::vector<std::pair<int32_t, int32_t>> A_1UTC_34({{160, 13}});
const std::vector<std::pair<int32_t, int32_t>> A_2UTC_34({{173, 7}});
const std::vector<std::pair<int32_t, int32_t>> dt_LS_34({{180, 8}});
const std::vector<std::pair<int32_t, int32_t>> t_ot_34({{188, 16}});
const std::vector<std::pair<int32_t, int32_t>> WN_ot_34({{204, 13}});
const std::vector<std::pair<int32_t, int32_t>> WN_LSF_34({{217, 13}});
const std::vector<std::pair<int32_t, int32_t>> DN_34({{230, 3}});
const std::vector<std::pair<int32_t, int32_t>> dt_LSF_34({{233, 8}});
// BDT-UTC Time Offset Parameters End
const std::vector<std::pair<int32_t, int32_t>> Rev_34({{241, 24}});


// Type 40 (288 bits)
const std::vector<std::pair<int32_t, int32_t>> HS_40({{31, 2}});
const std::vector<std::pair<int32_t, int32_t>> DIF_40({{33, 1}});
const std::vector<std::pair<int32_t, int32_t>> SIF_40({{34, 1}});
const std::vector<std::pair<int32_t, int32_t>> AIF_40({{35, 1}});
const std::vector<std::pair<int32_t, int32_t>> SISMAI_40({{36, 4}});
const std::vector<std::pair<int32_t, int32_t>> DIF_B1C_40({{40, 1}});
const std::vector<std::pair<int32_t, int32_t>> SIF_B1C_40({{41, 1}});
const std::vector<std::pair<int32_t, int32_t>> AIF_B1C_40({{42, 1}});
const std::vector<std::pair<int32_t, int32_t>> SISAI_OE_40({{43, 5}});
// SISAI_OC (22 bits)
const std::vector<std::pair<int32_t, int32_t>> t_op_40({{48, 11}});
const std::vector<std::pair<int32_t, int32_t>> SISAI_ocb_40({{59, 5}});
const std::vector<std::pair<int32_t, int32_t>> SISAI_oc1_40({{64, 3}});
const std::vector<std::pair<int32_t, int32_t>> SISAI_oc2_40({{67, 3}});
// SISAI_OC End
// Midi Almanac Parameters (156 bits)
const std::vector<std::pair<int32_t, int32_t>> PRN_a_40({{70, 6}});
const std::vector<std::pair<int32_t, int32_t>> SatType_40({{76, 2}});
const std::vector<std::pair<int32_t, int32_t>> WN_a_40({{78, 13}});
const std::vector<std::pair<int32_t, int32_t>> t_oa_40({{91, 8}});
const std::vector<std::pair<int32_t, int32_t>> e_40({{99, 11}});
const std::vector<std::pair<int32_t, int32_t>> delta_i_40({{110, 11}});
const std::vector<std::pair<int32_t, int32_t>> sqrt_A_40({{121, 17}});
const std::vector<std::pair<int32_t, int32_t>> Omega_0_40({{138, 16}});
const std::vector<std::pair<int32_t, int32_t>> Omega_dot_40({{154, 11}});
const std::vector<std::pair<int32_t, int32_t>> omega_40({{165, 16}});
const std::vector<std::pair<int32_t, int32_t>> M_0_40({{181, 16}});
const std::vector<std::pair<int32_t, int32_t>> a_f0_40({{197, 11}});
const std::vector<std::pair<int32_t, int32_t>> a_f1_40({{208, 10}});
const std::vector<std::pair<int32_t, int32_t>> Health_40({{218, 8}});
// Midi Almanac Parameters End
const std::vector<std::pair<int32_t, int32_t>> Rev_40({{226, 39}});


#endif /* GNSS_SDR_BEIDOU_B2a_H_ */