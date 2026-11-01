/*!
 * \file gps_l1c_telemetry_decoder_test.cc
 * \brief Checks GPS L1C telemetry time continuity with damaged subframes
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

#include "GPS_CNAV2.h"
#include "GPS_L1C.h"
#include "gnss_satellite.h"
#include "gnss_synchro.h"
#include "gps_cnav2_ldpc.h"
#include "gps_cnav2_navigation_message.h"
#include "gps_l1c_telemetry_decoder_gs.h"
#include "tlm_conf.h"
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <cstdint>
#include <random>
#include <utility>
#include <vector>

#ifdef GR_GREATER_38
#include <gnuradio/blocks/vector_source.h>
#else
#include <gnuradio/blocks/vector_source_b.h>
#endif


class GpsL1cTelemetryDecoderTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        decoder = gps_l1c_make_telemetry_decoder_gs(Tlm_Conf{});
        decoder->set_satellite(Gnss_Satellite("GPS", 11));
        // Allocate GNU Radio buffers, then drive work directly without scheduler threads.
        auto source = gr::blocks::vector_source_b::make(std::vector<uint8_t>(sizeof(Gnss_Synchro), 0), false, sizeof(Gnss_Synchro));
        auto sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        top_block = gr::make_top_block("gps_l1c_telemetry_decoder_test");
        top_block->connect(source, 0, decoder, 0);
        top_block->connect(decoder, 0, sink, 0);
        top_block->run();
        decoder->reset();
        symbol.System = 'G';
        symbol.Signal[0] = '1';
        symbol.Signal[1] = 'S';
        symbol.PRN = 11;
        symbol.Flag_valid_symbol_output = true;
        symbol.fs = 4000000;
        symbol.CN0_dB_hz = 40.0;
    }

    std::vector<float> frame(int32_t toi, int32_t itow = 10, bool valid_sf2 = true, int32_t week = 2400) const
    {
        std::vector<uint8_t> sf2(GPS_L1C_SF2_BITS, 0);
        std::vector<uint8_t> sf3(GPS_L1C_SF3_BITS, 0);
        set_field(sf2, CNAV2_WN, week);
        set_field(sf2, CNAV2_ITOW, itow);
        set_field(sf2, CNAV2_TOE, 240);
        set_field(sf3, CNAV2_SF3_PRN, 11);
        append_crc(sf2);
        append_crc(sf3);
        if (!valid_sf2)
            {
                sf2.back() ^= 1U;  // Valid LDPC codeword with a failing subframe CRC.
            }
        const Gps_Cnav2_Ldpc code2(2);
        const Gps_Cnav2_Ldpc code3(3);
        auto coded = code2.encode(sf2);
        const auto coded3 = code3.encode(sf3);
        coded.insert(coded.end(), coded3.cbegin(), coded3.cend());
        std::vector<uint8_t> interleaved(coded.size());
        Gps_CNAV2_Navigation_Message::interleave(coded.data(), interleaved.data());
        auto bits = Gps_CNAV2_Navigation_Message::encode_toi(toi);
        bits.insert(bits.end(), interleaved.cbegin(), interleaved.cend());
        std::vector<float> symbols(bits.size());
        std::transform(bits.cbegin(), bits.cend(), symbols.begin(), [](uint8_t bit) { return bit ? -1.0F : 1.0F; });
        return symbols;
    }

    void corrupt_toi(std::vector<float>& symbols) const
    {
        std::mt19937 rng(12);
        for (int32_t i = 0; i < GPS_L1C_TOI_SYMBOLS; ++i)
            {
                symbols[i] = (rng() & 1U) ? 1.0F : -1.0F;
            }
        float quality = 0.0F;
        EXPECT_EQ(Gps_CNAV2_Navigation_Message::decode_toi(symbols.data(), quality), 77);
        EXPECT_LT(quality, GPS_L1C_TOI_MIN_QUALITY);
        Gps_CNAV2_Navigation_Message nav;
        ASSERT_TRUE(nav.decode_frame(symbols.data()));
        ASSERT_TRUE(nav.sf2_crc_ok());
        ASSERT_TRUE(nav.sf3_crc_ok());
    }

    void feed(const std::vector<float>& symbols)
    {
        gr_vector_const_void_star inputs{&symbol};
        gr_vector_void_star outputs{&output};
        gr_vector_int available{1};
        for (const auto value : symbols)
            {
                symbol.Prompt_I = value;
                symbol.Tracking_sample_counter += 40000;
                ASSERT_EQ(decoder->general_work(1, available, inputs, outputs), 1);
            }
    }

    Gnss_Synchro output{};

private:
    static void set_field(std::vector<uint8_t>& bits, const std::vector<std::pair<int32_t, int32_t>>& field, uint32_t value)
    {
        const int32_t start = field[0].first - 1;
        const int32_t length = field[0].second;
        for (int32_t i = 0; i < length; ++i)
            {
                bits[start + i] = static_cast<uint8_t>((value >> (length - i - 1)) & 1U);
            }
    }

    static void append_crc(std::vector<uint8_t>& bits)
    {
        const auto payload_size = bits.size() - GPS_L1C_CRC_BITS;
        const uint32_t crc = Gps_CNAV2_Navigation_Message::crc24q(bits, payload_size);
        for (int32_t i = 0; i < GPS_L1C_CRC_BITS; ++i)
            {
                bits[payload_size + i] = static_cast<uint8_t>((crc >> (GPS_L1C_CRC_BITS - i - 1)) & 1U);
            }
    }

    gps_l1c_telemetry_decoder_gs_sptr decoder;
    gr::top_block_sptr top_block;
    Gnss_Synchro symbol{};
};


TEST_F(GpsL1cTelemetryDecoderTest, UnreliableToiDoesNotOverwriteEstablishedTime)
{
    feed(frame(100));
    ASSERT_TRUE(output.Flag_valid_word);
    ASSERT_EQ(output.TOW_at_current_symbol_ms, 73800000U);
    auto damaged = frame(101);
    corrupt_toi(damaged);
    feed(damaged);
    EXPECT_TRUE(output.Flag_valid_word);
    EXPECT_EQ(output.TOW_at_current_symbol_ms, 73818000U);
    feed(frame(102));
    EXPECT_TRUE(output.Flag_valid_word);
    EXPECT_EQ(output.TOW_at_current_symbol_ms, 73836000U);
}


TEST_F(GpsL1cTelemetryDecoderTest, UnreliableToiCannotEstablishTimeAfterSubframe3OnlySync)
{
    feed(frame(100, 10, false));
    ASSERT_FALSE(output.Flag_valid_word);
    auto damaged = frame(101);
    corrupt_toi(damaged);
    feed(damaged);
    EXPECT_FALSE(output.Flag_valid_word);
    feed(frame(102));
    EXPECT_TRUE(output.Flag_valid_word);
    EXPECT_EQ(output.TOW_at_current_symbol_ms, 73836000U);
}


TEST_F(GpsL1cTelemetryDecoderTest, TimeContinuesAcrossWeekRollover)
{
    feed(frame(399, 83));
    ASSERT_TRUE(output.Flag_valid_word);
    ASSERT_EQ(output.TOW_at_current_symbol_ms, 604782000U);
    feed(frame(0, 83));
    EXPECT_TRUE(output.Flag_valid_word);
    EXPECT_EQ(output.TOW_at_current_symbol_ms, 0U);
    feed(frame(1, 0, true, 2401));
    EXPECT_TRUE(output.Flag_valid_word);
    EXPECT_EQ(output.TOW_at_current_symbol_ms, 18000U);
}


TEST_F(GpsL1cTelemetryDecoderTest, TimeContinuesAcrossMissingSubframe2AndPolarityChange)
{
    feed(frame(100));
    ASSERT_TRUE(output.Flag_valid_word);
    feed(frame(101, 10, false));
    EXPECT_TRUE(output.Flag_valid_word);
    EXPECT_EQ(output.TOW_at_current_symbol_ms, 73818000U);
    auto inverted = frame(102);
    for (auto& value : inverted)
        {
            value = -value;
        }
    feed(inverted);
    EXPECT_TRUE(output.Flag_valid_word);
    EXPECT_TRUE(output.Flag_PLL_180_deg_phase_locked);
    EXPECT_EQ(output.TOW_at_current_symbol_ms, 73836000U);
}
