/*!
 * \file benchmark_osnma.cc
 * \brief Benchmarks for osnma functions
 * \author Carles Fernandez-Prades, 2024. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "Galileo_OSNMA.h"
#include "gnss_crypto.h"
#include "osnma_helper.h"
#include "osnma_msg_receiver.h"
#include <benchmark/benchmark.h>
#include <memory>

void bm_verify_public_key(benchmark::State& state)
{
    osnma_msg_receiver_sptr osnma = osnma_msg_receiver_make(CRTFILE_DEFAULT, MERKLEFILE_DEFAULT);
    Osnma_Helper helper;
    osnma->set_merkle_root(helper.convert_from_hex_string("A10C440F3AA62453526DB4AF76DF8D9410D35D8277397D7053C700D192702B0D"));
    DSM_PKR_message dsm_pkr_message;
    dsm_pkr_message.npkt = 0x01;
    dsm_pkr_message.npktid = 0x2;
    dsm_pkr_message.mid = 0x01;
    std::vector<uint8_t> vec = helper.convert_from_hex_string(
        "7CBE05D9970CFC9E22D0A43A340EF557624453A2E821AADEAC989C405D78BA06"
        "956380BAB0D2C939EC6208151040CCFFCF1FB7156178FD1255BA0AECAAA253F7"
        "407B6C5DD4DF059FF8789474061301E1C34881DB7A367A913A3674300E21EAB1"
        "24EF508389B7D446C3E2ECE8D459FBBD3239A794906F5B1F92469C640164FD87");
    std::copy(vec.begin(), vec.end(), dsm_pkr_message.itn.begin());
    dsm_pkr_message.npk = helper.convert_from_hex_string("0303B2CE64BC207BDD8BC4DF859187FCB686320D63FFA091410FC158FBB77980EA");

    while (state.KeepRunning())
        {
            osnma->verify_dsm_pkr(dsm_pkr_message);
        }
}

void bm_verify_tesla_key(benchmark::State& state)
{
    //    osnma_msg_receiver_sptr osnma = osnma_msg_receiver_make(CRTFILE_DEFAULT, MERKLEFILE_DEFAULT);
    //    Osnma_Helper helper;
    //    osnma->d_tesla_key_verified = false;
    //    osnma->d_osnma_data.d_dsm_kroot_message.kroot = {0x5B, 0xF8, 0xC9, 0xCB, 0xFC, 0xF7, 0x04, 0x22, 0x08, 0x14, 0x75, 0xFD, 0x44, 0x5D, 0xF0, 0xFF};  // Kroot, TOW 345570 GST_0 - 30
    //    osnma->d_osnma_data.d_dsm_kroot_message.ks = 4;                                                                                                    // TABLE 10 --> 128 bits
    //    osnma->d_osnma_data.d_dsm_kroot_message.alpha = 0x610BDF26D77B;
    //    osnma->d_GST_SIS = (1248 & 0x00000FFF) << 20 | (345630 & 0x000FFFFF);
    //    osnma->d_GST_0 = ((1248 & 0x00000FFF) << 20 | (345600 & 0x000FFFFF));                          // applicable time (GST_Kroot + 30)
    //    osnma->d_GST_Sf = osnma->d_GST_0 + 30 * std::floor((osnma->d_GST_SIS - osnma->d_GST_0) / 30);  // Eq. 3 R.G.
    //
    //    osnma->d_tesla_keys.insert((std::pair<uint32_t, std::vector<uint8_t>>(345600, {0xEF, 0xF9, 0x99, 0x04, 0x0E, 0x19, 0xB5, 0x70, 0x83, 0x50, 0x60, 0xBE, 0xBD, 0x23, 0xED, 0x92})));  // K1, not needed, just for reference.
    //    std::vector<uint8_t> key = {0x2D, 0xC3, 0xA3, 0xCD, 0xB1, 0x17, 0xFA, 0xAD, 0xB8, 0x3B, 0x5F, 0x0B, 0x6F, 0xEA, 0x88, 0xEB};                                                        // K2
    //    uint32_t TOW = 345630;
    //
    //    while (state.KeepRunning())
    //        {
    //            osnma->verify_tesla_key(key, TOW);
    //        }
}

void bm_verify_tesla_key_24h(benchmark::State& state)
{
    // TODO - copy of normal tesla verification but with 2800 steps instead of only two (max Kroot time is 1 day as per spec.)
}

void bm_tag_verification(benchmark::State& state)
{
    //    osnma_msg_receiver_sptr osnma = osnma_msg_receiver_make(CRTFILE_DEFAULT, MERKLEFILE_DEFAULT);
    //    Osnma_Helper helper;
    //    uint32_t TOW_Tag0 = 345660;
    //    uint32_t TOW_NavData = TOW_Tag0 - 30;
    //    uint32_t TOW_Key_Tag0 = TOW_Tag0 + 30;
    //    uint32_t WN = 1248;
    //    uint32_t PRNa = 2;
    //    uint8_t CTR = 1;
    //
    //    osnma->d_osnma_data.d_dsm_kroot_message.ts = 9;                                                                                        // 40 bit
    //    osnma->d_tesla_keys[TOW_Key_Tag0] = {0x69, 0xC0, 0x0A, 0xA7, 0x36, 0x42, 0x37, 0xA6, 0x5E, 0xBF, 0x00, 0x6A, 0xD8, 0xDD, 0xBC, 0x73};  // K4
    //    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    //    osnma->d_nav_data_manager->add_navigation_data(
    //        "000011101001011001000100000101000111010110100100100101100000000000"
    //        "011101101011001111101110101010000001010000011011111100000011101011"
    //        "011100101101011010101011011011001001110111101011110110111111001111"
    //        "001000011111101110011000111111110111111010000011101011111111110000"
    //        "110111000000100000001110110000110110001110000100001110101100010100"
    //        "110100010001000110001110011010110000111010000010000000000001101000"
    //        "000000000011100101100100010000000000000110110100110001111100000000"
    //        "000000100110100000000101010010100000001011000010001001100000011111"
    //        "110111111111000000000",
    //        PRNa, TOW_NavData);
    //    osnma->d_osnma_data.d_nma_header.nmas = 0b10;
    //
    //    MACK_tag_and_info MTI;
    //    MTI.tag = static_cast<uint64_t>(0xE37BC4F858);
    //    MTI.tag_info.PRN_d = 0x02;
    //    MTI.tag_info.ADKD = 0x00;
    //    MTI.tag_info.cop = 0x0F;
    //    Tag t0(MTI, TOW_Tag0, WN, PRNa, CTR);
    //
    //    while (state.KeepRunning())
    //        {
    //            osnma->verify_tag(t0);
    //        }
}

void bm_kroot_verification(benchmark::State& state)
{
    // TODO - this is essentially the signature verification, maybe could implement it for comparison purposes
}

BENCHMARK(bm_verify_public_key);
BENCHMARK(bm_verify_tesla_key);
BENCHMARK(bm_verify_tesla_key_24h);
BENCHMARK(bm_tag_verification);
BENCHMARK(bm_kroot_verification);


BENCHMARK_MAIN();