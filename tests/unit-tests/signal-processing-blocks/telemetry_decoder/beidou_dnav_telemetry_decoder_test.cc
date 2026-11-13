/*!
 * \file beidou_dnav_telemetry_decoder_test.cc
 * \brief Checks the 180-degree carrier phase lock reported by the BeiDou DNAV
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

#include "Beidou_DNAV.h"
#include "MATH_CONSTANTS.h"
#include "beidou_dnav_telemetry_decoder_gs.h"
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


class BeidouDnavTelemetryDecoderTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        decoder = beidou_dnav_make_telemetry_decoder_gs(Tlm_Conf{}, 1);
        decoder->set_satellite(Gnss_Satellite("Beidou", 10));  // D1 (non-GEO) satellite
        // Allocate GNU Radio buffers, then drive work directly without scheduler threads.
        auto source = gr::blocks::vector_source_b::make(std::vector<uint8_t>(sizeof(Gnss_Synchro), 0), false, sizeof(Gnss_Synchro));
        auto sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        top_block = gr::make_top_block("beidou_dnav_telemetry_decoder_test");
        top_block->connect(source, 0, decoder, 0);
        top_block->connect(decoder, 0, sink, 0);
        top_block->run();
        decoder->reset();
        symbol.System = 'C';
        symbol.Signal[0] = 'B';
        symbol.Signal[1] = '1';
        symbol.PRN = 10;
        symbol.Flag_valid_symbol_output = true;
        symbol.fs = 25000000;
        symbol.CN0_dB_hz = 40.0;
        symbol.Carrier_phase_rads = CARRIER_PHASE_RADS;
    }

    // Symbols (+1 for a bit 1) of a D1 subframe 4 with the given SOW and no
    // almanac page, encoded as transmitted: BCH(15,11) and, from word 2 on,
    // the two codewords of each word interleaved bit by bit.
    static std::vector<float> subframe(uint32_t sow, bool inverted = false)
    {
        std::array<uint8_t, BEIDOU_DNAV_SUBFRAME_DATA_BITS> bits{};
        const char* preamble = BEIDOU_DNAV_PREAMBLE;
        for (int32_t i = 0; i < BEIDOU_DNAV_PREAMBLE_LENGTH_BITS; i++)
            {
                bits[i] = static_cast<uint8_t>(preamble[i] == '1');
            }
        set_field(bits, D1_FRAID, 4);
        set_field(bits, D1_SOW, sow);

        std::vector<float> symbols;
        symbols.reserve(BEIDOU_DNAV_SUBFRAME_SYMBOLS);
        // Word 1: 15 bits sent as they are, then one codeword
        for (int32_t i = 0; i < 15; i++)
            {
                symbols.push_back(bits[i] ? 1.0F : -1.0F);
            }
        for (const auto bit : bch_encode(&bits[15]))
            {
                symbols.push_back(bit ? 1.0F : -1.0F);
            }
        // Words 2-10: 22 information bits in two interleaved codewords
        for (uint32_t word = 1; word < BEIDOU_DNAV_WORDS_SUBFRAME; word++)
            {
                const auto first = bch_encode(&bits[word * 30]);
                const auto second = bch_encode(&bits[word * 30 + 11]);
                for (int32_t i = 0; i < 15; i++)
                    {
                        symbols.push_back(first[i] ? 1.0F : -1.0F);
                        symbols.push_back(second[i] ? 1.0F : -1.0F);
                    }
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
                symbol.Tracking_sample_counter += 500000;
                EXPECT_EQ(decoder->general_work(1, available, inputs, outputs), 1);
            }
        return output;
    }

    // Feeds subframes with consecutive SOWs, the given ones inverted, and
    // returns the output at the end of each
    std::vector<Gnss_Synchro> feed_subframes(int32_t count, const std::vector<bool>& inverted = {})
    {
        std::vector<Gnss_Synchro> outputs;
        for (int32_t i = 0; i < count; i++)
            {
                const bool invert = i < static_cast<int32_t>(inverted.size()) && inverted[i];
                outputs.push_back(feed(subframe(FIRST_SOW + static_cast<uint32_t>(i) * 6, invert)));
            }
        return outputs;
    }

    // TOW reported with the last symbol of subframe n: the end of that
    // subframe, which is the SOW of the next one, in GPS time
    static uint32_t tow_at_end_of_subframe(int32_t n)
    {
        return (FIRST_SOW + static_cast<uint32_t>(n + 1) * 6 + BEIDOU_DNAV_BDT2GPST_LEAP_SEC_OFFSET) * 1000;
    }

    static constexpr double CARRIER_PHASE_RADS = 1.0;
    static constexpr uint32_t FIRST_SOW = 345600;
    Gnss_Synchro symbol{};

private:
    template <size_t N>
    static void set_field(std::array<uint8_t, N>& bits, const std::vector<std::pair<int32_t, int32_t>>& field, uint32_t value)
    {
        int32_t length = 0;
        for (const auto& part : field)
            {
                length += part.second;
            }
        for (const auto& part : field)
            {
                for (int32_t i = 0; i < part.second; i++)
                    {
                        length--;
                        bits[part.first - 1 + i] = static_cast<uint8_t>((value >> length) & 1U);
                    }
            }
    }

    // Systematic BCH(15,11) codeword with generator x^4 + x + 1
    static std::array<uint8_t, 15> bch_encode(const uint8_t* info)
    {
        std::array<uint8_t, 15> codeword{};
        std::array<uint8_t, 4> reg{};
        for (int32_t i = 0; i < 11; i++)
            {
                codeword[i] = info[i];
                const uint8_t feedback = info[i] ^ reg[3];
                reg[3] = reg[2];
                reg[2] = reg[1];
                reg[1] = reg[0] ^ feedback;
                reg[0] = feedback;
            }
        for (int32_t i = 0; i < 4; i++)
            {
                codeword[11 + i] = reg[3 - i];
            }
        return codeword;
    }

    beidou_dnav_telemetry_decoder_gs_sptr decoder;
    gr::top_block_sptr top_block;
};


TEST_F(BeidouDnavTelemetryDecoderTest, NormalPolarityClearsTheTrackingFlag)
{
    symbol.Flag_PLL_180_deg_phase_locked = true;
    const auto outputs = feed_subframes(6);
    const auto& output = outputs.back();
    ASSERT_TRUE(output.Flag_valid_word);
    EXPECT_EQ(output.TOW_at_current_symbol_ms, tow_at_end_of_subframe(5));
    EXPECT_FALSE(output.Flag_PLL_180_deg_phase_locked);
    EXPECT_DOUBLE_EQ(output.Carrier_phase_rads, CARRIER_PHASE_RADS);
}


TEST_F(BeidouDnavTelemetryDecoderTest, InvertedPolaritySetsTheFlagAndAddsHalfACycle)
{
    const auto outputs = feed_subframes(6, std::vector<bool>(6, true));
    const auto& output = outputs.back();
    ASSERT_TRUE(output.Flag_valid_word);
    EXPECT_EQ(output.TOW_at_current_symbol_ms, tow_at_end_of_subframe(5));
    EXPECT_TRUE(output.Flag_PLL_180_deg_phase_locked);
    EXPECT_DOUBLE_EQ(output.Carrier_phase_rads, CARRIER_PHASE_RADS + GNSS_PI);
}


TEST_F(BeidouDnavTelemetryDecoderTest, PolarityChangeIsReportedFromTheNextDecodedSubframe)
{
    // The decoder decodes a subframe once it has received the preamble of the
    // next one, so the output at the end of subframe n reflects subframe n - 1.
    const auto outputs = feed_subframes(8, {false, false, false, false, false, true, true, true});
    ASSERT_TRUE(outputs[5].Flag_valid_word);
    EXPECT_FALSE(outputs[5].Flag_PLL_180_deg_phase_locked);
    EXPECT_DOUBLE_EQ(outputs[5].Carrier_phase_rads, CARRIER_PHASE_RADS);
    ASSERT_TRUE(outputs[7].Flag_valid_word);
    EXPECT_EQ(outputs[7].TOW_at_current_symbol_ms, tow_at_end_of_subframe(7));
    EXPECT_TRUE(outputs[7].Flag_PLL_180_deg_phase_locked);
    EXPECT_DOUBLE_EQ(outputs[7].Carrier_phase_rads, CARRIER_PHASE_RADS + GNSS_PI);
}
