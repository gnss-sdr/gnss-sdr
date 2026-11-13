/*!
 * \file glonass_gnav_telemetry_decoder_test.cc
 * \brief Checks the 180-degree carrier phase lock reported by the GLONASS GNAV
 * telemetry decoder
 * \author Oleksandr Suvorov, 2026. github(at)say.od.ua
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

#include "GLONASS_L1_L2_CA.h"
#include "MATH_CONSTANTS.h"
#include "glonass_gnav_telemetry_decoder_gs.h"
#include "gnss_satellite.h"
#include "gnss_synchro.h"
#include "tlm_conf.h"
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <array>
#include <cstdint>
#include <vector>

#ifdef GR_GREATER_38
#include <gnuradio/blocks/vector_source.h>
#else
#include <gnuradio/blocks/vector_source_b.h>
#endif


class GlonassGnavTelemetryDecoderTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        decoder = glonass_gnav_make_telemetry_decoder_gs(Tlm_Conf{}, 1);
        decoder->set_satellite(Gnss_Satellite("Glonass", 1));
        // Allocate GNU Radio buffers, then drive work directly without scheduler threads.
        auto source = gr::blocks::vector_source_b::make(std::vector<uint8_t>(sizeof(Gnss_Synchro), 0), false, sizeof(Gnss_Synchro));
        auto sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        top_block = gr::make_top_block("glonass_gnav_telemetry_decoder_test");
        top_block->connect(source, 0, decoder, 0);
        top_block->connect(decoder, 0, sink, 0);
        top_block->run();
        decoder->reset();
        symbol.System = 'R';
        symbol.Signal[0] = '1';
        symbol.Signal[1] = 'G';
        symbol.PRN = 1;
        symbol.Flag_valid_symbol_output = true;
        symbol.fs = 6625000;
        symbol.CN0_dB_hz = 40.0;
        symbol.Carrier_phase_rads = CARRIER_PHASE_RADS;
    }

    // Symbols of a time mark followed by a string with the given number, as
    // transmitted: Hamming code, relative code and meander. The decoder takes
    // the time decoded from strings 1, 4 and 5. A string with a damaged
    // Hamming code has two data bits flipped, which the code cannot correct.
    static std::vector<float> string_after_time_mark(uint32_t string_id, bool inverted = false, bool damaged = false)
    {
        std::array<uint8_t, GLONASS_GNAV_STRING_BITS> bits{};  // bits[0] is the first (idle) bit
        set_field(bits, STRING_ID, string_id);
        switch (string_id)
            {
            case 1:
                set_field(bits, T_K_HR, 12);
                break;
            case 4:
                set_field(bits, N_T, 100);
                set_field(bits, N, 1);
                break;
            case 5:
                set_field(bits, N_4, 8);
                break;
            default:
                break;
            }
        add_hamming_code(bits);
        if (damaged)
            {
                bits[10] ^= 1U;
                bits[11] ^= 1U;
            }

        std::vector<float> symbols(GLONASS_GNAV_PREAMBLE_SAMPLES);
        uint8_t relative = 0;
        for (int32_t i = 0; i < GLONASS_GNAV_STRING_BITS; i++)
            {
                if (i > 0)
                    {
                        relative ^= bits[i];
                    }
                symbols.push_back(relative ? 1.0F : -1.0F);
                symbols.push_back(relative ? -1.0F : 1.0F);
            }
        if (inverted)
            {
                for (auto& value : symbols)
                    {
                        value = -value;
                    }
            }
        return symbols;
    }

    // Feeds the symbols and returns the decoder output for the last one
    Gnss_Synchro feed(const std::vector<float>& symbols)
    {
        Gnss_Synchro output{};
        gr_vector_const_void_star inputs{&symbol};
        gr_vector_void_star outputs{&output};
        gr_vector_int available{1};
        for (const auto value : symbols)
            {
                symbol.Prompt_I = value;
                symbol.Tracking_sample_counter += 66250;
                EXPECT_EQ(decoder->general_work(1, available, inputs, outputs), 1);
            }
        return output;
    }

    // Feeds strings and returns the output at the end of each. The decoder
    // first decodes the fifth string, so strings 1 to 5 go from there on;
    // string 5 is not repeated afterwards, as a new one would carry a new time.
    std::vector<Gnss_Synchro> feed_strings(int32_t count, const std::vector<bool>& inverted = {}, int32_t damaged = -1)
    {
        std::vector<Gnss_Synchro> outputs;
        for (int32_t i = 0; i < count; i++)
            {
                uint32_t string_id = 1;
                if (i >= FIRST_DECODED_STRING && i < FIRST_DECODED_STRING + 5)
                    {
                        string_id = static_cast<uint32_t>(i - FIRST_DECODED_STRING + 1);
                    }
                else if (i >= FIRST_DECODED_STRING + 5)
                    {
                        string_id = static_cast<uint32_t>((i - FIRST_DECODED_STRING - 5) % 4 + 1);
                    }
                const bool invert = i < static_cast<int32_t>(inverted.size()) && inverted[i];
                outputs.push_back(feed(string_after_time_mark(string_id, invert, i == damaged)));
            }
        return outputs;
    }

    static constexpr double CARRIER_PHASE_RADS = 1.0;
    static constexpr int32_t FIRST_DECODED_STRING = 4;
    static constexpr int32_t TIME_SET_STRING = FIRST_DECODED_STRING + 4;
    Gnss_Synchro symbol{};

private:
    static void set_field(std::array<uint8_t, GLONASS_GNAV_STRING_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& field, uint32_t value)
    {
        const int32_t start = field[0].first - 1;
        const int32_t length = field[0].second;
        for (int32_t i = 0; i < length; i++)
            {
                bits[start + i] = static_cast<uint8_t>((value >> (length - i - 1)) & 1U);
            }
    }

    // Check bits 1 to 8 (the last eight bits of the string) of the Hamming
    // code. ICD bit k is bits[GLONASS_GNAV_STRING_BITS - k].
    static void add_hamming_code(std::array<uint8_t, GLONASS_GNAV_STRING_BITS>& bits)
    {
        const auto icd_bit = [&bits](int32_t k) -> uint8_t& { return bits[GLONASS_GNAV_STRING_BITS - k]; };
        const auto parity = [&icd_bit](const std::vector<int32_t>& indexes) {
            uint8_t sum = 0;
            for (const auto k : indexes)
                {
                    sum ^= icd_bit(k);
                }
            return sum;
        };
        const std::array<const std::vector<int32_t>*, 7> checks{&GLONASS_GNAV_CRC_I_INDEX, &GLONASS_GNAV_CRC_J_INDEX,
            &GLONASS_GNAV_CRC_K_INDEX, &GLONASS_GNAV_CRC_L_INDEX, &GLONASS_GNAV_CRC_M_INDEX, &GLONASS_GNAV_CRC_N_INDEX,
            &GLONASS_GNAV_CRC_P_INDEX};
        uint8_t check_sum = 0;
        for (int32_t k = 1; k <= 7; k++)
            {
                icd_bit(k) = parity(*checks[k - 1]);
                check_sum ^= icd_bit(k);
            }
        icd_bit(8) = check_sum ^ parity(GLONASS_GNAV_CRC_Q_INDEX);
    }

    glonass_gnav_telemetry_decoder_gs_sptr decoder;
    gr::top_block_sptr top_block;
};


TEST_F(GlonassGnavTelemetryDecoderTest, NormalPolarityClearsTheTrackingFlag)
{
    symbol.Flag_PLL_180_deg_phase_locked = true;
    const auto outputs = feed_strings(TIME_SET_STRING + 2);
    EXPECT_FALSE(outputs[TIME_SET_STRING - 1].Flag_valid_word);
    const auto& output = outputs.back();
    ASSERT_TRUE(output.Flag_valid_word);
    EXPECT_FALSE(output.Flag_PLL_180_deg_phase_locked);
    EXPECT_DOUBLE_EQ(output.Carrier_phase_rads, CARRIER_PHASE_RADS);
}


TEST_F(GlonassGnavTelemetryDecoderTest, InvertedPolaritySetsTheFlagAndAddsHalfACycle)
{
    const auto outputs = feed_strings(TIME_SET_STRING + 2, std::vector<bool>(TIME_SET_STRING + 2, true));
    const auto& output = outputs.back();
    ASSERT_TRUE(output.Flag_valid_word);
    EXPECT_TRUE(output.Flag_PLL_180_deg_phase_locked);
    EXPECT_DOUBLE_EQ(output.Carrier_phase_rads, CARRIER_PHASE_RADS + GNSS_PI);
}


TEST_F(GlonassGnavTelemetryDecoderTest, PolarityChangeIsReportedFromTheNextValidString)
{
    // The string after a time mark is decoded once its last symbol arrives
    std::vector<bool> inverted(TIME_SET_STRING + 3, false);
    inverted[TIME_SET_STRING + 2] = true;
    const auto outputs = feed_strings(TIME_SET_STRING + 3, inverted);
    ASSERT_TRUE(outputs[TIME_SET_STRING + 1].Flag_valid_word);
    EXPECT_FALSE(outputs[TIME_SET_STRING + 1].Flag_PLL_180_deg_phase_locked);
    ASSERT_TRUE(outputs[TIME_SET_STRING + 2].Flag_valid_word);
    EXPECT_TRUE(outputs[TIME_SET_STRING + 2].Flag_PLL_180_deg_phase_locked);
    EXPECT_DOUBLE_EQ(outputs[TIME_SET_STRING + 2].Carrier_phase_rads, CARRIER_PHASE_RADS + GNSS_PI);
}


TEST_F(GlonassGnavTelemetryDecoderTest, StringFailingTheHammingCheckKeepsThePreviousDecision)
{
    std::vector<bool> inverted(TIME_SET_STRING + 4, false);
    inverted[TIME_SET_STRING + 2] = true;
    inverted[TIME_SET_STRING + 3] = true;
    const auto outputs = feed_strings(TIME_SET_STRING + 4, inverted, TIME_SET_STRING + 2);
    ASSERT_TRUE(outputs[TIME_SET_STRING + 2].Flag_valid_word);
    EXPECT_FALSE(outputs[TIME_SET_STRING + 2].Flag_PLL_180_deg_phase_locked);
    EXPECT_DOUBLE_EQ(outputs[TIME_SET_STRING + 2].Carrier_phase_rads, CARRIER_PHASE_RADS);
    ASSERT_TRUE(outputs[TIME_SET_STRING + 3].Flag_valid_word);
    EXPECT_TRUE(outputs[TIME_SET_STRING + 3].Flag_PLL_180_deg_phase_locked);
}
