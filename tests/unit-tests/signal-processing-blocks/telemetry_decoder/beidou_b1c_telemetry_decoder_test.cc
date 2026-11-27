/*!
 * \file beidou_b1c_telemetry_decoder_test.cc
 * \brief B1C first-secondary-lock history and subsequent frame timing tests
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * Copyright (C) 2010-2026 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "../../system-parameters/beidou_cnav_test_helpers.h"
#include "Beidou_B1C_codes.h"
#include "Beidou_CNAV1.h"
#include "beidou_b1c_telemetry_decoder_gs.h"
#include "beidou_cnav1_ephemeris.h"
#include "gnss_satellite.h"
#include "gnss_synchro.h"
#include "nav_message_packet.h"
#include "tlm_conf.h"
#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/sync_block.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#if PMT_USES_BOOST_ANY
#include <boost/any.hpp>
#else
#include <any>
#endif

namespace b1c_telemetry_test
{
#if PMT_USES_BOOST_ANY
using boost::any_cast;
#else
using std::any_cast;
#endif

constexpr int n = BEIDOU_CNAV1_FRAME_SYMBOLS;
constexpr int prn = 24;

void bch(std::vector<float>& frame, unsigned message, int length, int bits, const std::vector<int>& taps)
{
    std::vector<int> state(bits, 1);
    for (int i = 0; i < bits; ++i)
        {
            state[bits - 1 - i] = ((message >> (bits - 1 - i)) & 1U) ? -1 : 1;
        }
    for (int i = 0; i < length; ++i)
        {
            frame.push_back(-4.0F * state.back());
            int feedback = 1;
            for (const int tap : taps)
                {
                    feedback *= state[tap - 1];
                }
            for (int j = bits - 1; j > 0; --j)
                {
                    state[j] = state[j - 1];
                }
            state[0] = feedback;
        }
}

// All-zero payloads have zero CRC and are LDPC codewords. For the CRC-negative
// case, encode a nonzero payload without updating its CRC: FEC still succeeds.
std::vector<float> frame(unsigned soh, bool valid_crc = true)
{
    std::array<uint8_t, BEIDOU_CNAV1_SF2_DATA_BITS> sf2{};
    std::array<uint8_t, BEIDOU_CNAV1_SF3_DATA_BITS> sf3{};
    if (!valid_crc)
        {
            sf2[0] = 1U;
        }
    const auto code2 = BeidouCnavTest::encode<BEIDOU_CNAV1_SUBFRAME2_SYMBOLS>(sf2);
    const auto code3 = BeidouCnavTest::encode<BEIDOU_CNAV1_SUBFRAME3_SYMBOLS>(sf3);
    std::array<float, BEIDOU_CNAV1_INTERLEAVED_SYMBOLS> grid{};
    int pos2 = 0;
    int pos3 = 0;
    for (int row = 0; row < BEIDOU_CNAV1_INTERLEAVE_ROWS; ++row)
        {
            for (int col = 0; col < BEIDOU_CNAV1_INTERLEAVE_COLS; ++col)
                {
                    const auto bit = row < 33 && row % 3 == 2 ? code3[pos3++] : code2[pos2++];
                    grid[row * BEIDOU_CNAV1_INTERLEAVE_COLS + col] = bit ? 4.0F : -4.0F;
                }
        }
    std::vector<float> result;
    bch(result, prn, 21, 6, {2, 4, 5, 6});
    bch(result, soh, 51, 8, {1, 4, 5, 6, 7, 8});
    for (int col = 0; col < BEIDOU_CNAV1_INTERLEAVE_COLS; ++col)
        {
            for (int row = 0; row < BEIDOU_CNAV1_INTERLEAVE_ROWS; ++row)
                {
                    result.push_back(grid[row * BEIDOU_CNAV1_INTERLEAVE_COLS + col]);
                }
        }
    return result;
}

std::vector<Gnss_Synchro> stream(bool valid_crc = true, float data_sign = 1.0F, float pilot_sign = 1.0F, int following_frames = 0)
{
    const auto first = frame(1, valid_crc);
    auto symbols = first;
    std::vector<Gnss_Synchro> result;
    for (int t = 0; t <= n * (following_frames + 1); ++t)
        {
            const int chip = t % n;
            const float secondary = BEIDOU_B1C_PILOT_SECONDARY_CODE[prn - 1][chip] == '0' ? 1.0F : -1.0F;
            Gnss_Synchro symbol{};
            symbol.System = 'C';
            symbol.Signal[0] = '1';
            symbol.Signal[1] = 'D';
            symbol.PRN = prn;
            symbol.fs = 8000000;
            symbol.Tracking_sample_counter = 80000ULL * (t + 1);
            symbol.CN0_dB_hz = 45.0;
            symbol.correlation_length_ms = 10;
            symbol.Flag_valid_symbol_output = t >= n;
            if (t >= n && chip == 0)
                {
                    symbols = frame(1 + t / n);
                }
            symbol.Prompt_I = symbols[chip] * data_sign * (t < n ? secondary : 1.0F);
            symbol.Prompt_Q = 8.0F * secondary * pilot_sign;
            result.push_back(symbol);
        }
    return result;
}

// Keep the pilot correlation signs but set the I/Q projector's phase to zero.
// The input amplitudes and power-of-two scaling give exact cancellation of
// the imaginary sum of (I+jQ)^2 over the 1800-symbol candidate.
void balance_pilot_for_raw_decode(std::vector<Gnss_Synchro>& input)
{
    const auto begin = input.begin() + 1;
    const auto end = begin + n;
    const auto positive = std::count_if(begin, end, [](const auto& s) { return s.Prompt_I * s.Prompt_Q > 0.0; });
    const auto negative = std::count_if(begin, end, [](const auto& s) { return s.Prompt_I * s.Prompt_Q < 0.0; });
    for (auto it = begin; it != end; ++it)
        {
            const double product = it->Prompt_I * it->Prompt_Q;
            if (product > 0.0)
                {
                    it->Prompt_Q *= negative;
                }
            else if (product < 0.0)
                {
                    it->Prompt_Q *= positive;
                }
            it->Prompt_Q /= 65536.0;
        }
}

// Erase data samples at negative secondary-code chips. BCH and LDPC can
// recover F from the remaining samples, with or without secondary wiping.
std::vector<Gnss_Synchro> decodable_without_wipe()
{
    auto input = stream(true, 1.0F, 1.0F, 1);
    const auto* secondary = BEIDOU_B1C_PILOT_SECONDARY_CODE[prn - 1];
    for (int i = 0; i < n; ++i)
        {
            if (secondary[i] == '1')
                {
                    input[i].Prompt_I = 0.0;
                }
        }
    balance_pilot_for_raw_decode(input);
    return input;
}

class SynchroSource : public gr::sync_block
{
public:
    explicit SynchroSource(const std::vector<Gnss_Synchro>& symbols)
        : gr::sync_block("b1c_telemetry_test_source",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(1, 1, sizeof(Gnss_Synchro))),
          d_symbols(symbols)
    {
    }

    int work(int noutput_items, gr_vector_const_void_star& /*input_items*/, gr_vector_void_star& output_items) override
    {
        if (d_offset == d_symbols.size())
            {
                return WORK_DONE;
            }
        const auto count = std::min(static_cast<size_t>(noutput_items), d_symbols.size() - d_offset);
        // GNU Radio supplies raw stream storage. Construct each symbol there.
        std::uninitialized_copy_n(d_symbols.data() + d_offset, count, static_cast<Gnss_Synchro*>(output_items[0]));
        d_offset += count;
        return static_cast<int>(count);
    }

private:
    const std::vector<Gnss_Synchro>& d_symbols;
    size_t d_offset{0};
};


class SynchroSink : public gr::sync_block
{
public:
    explicit SynchroSink(std::vector<Gnss_Synchro>& symbols)
        : gr::sync_block("b1c_telemetry_test_sink",
              gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
              gr::io_signature::make(0, 0, 0)),
          d_symbols(symbols)
    {
    }

    int work(int noutput_items, gr_vector_const_void_star& input_items, gr_vector_void_star& /*output_items*/) override
    {
        const auto* in = static_cast<const Gnss_Synchro*>(input_items[0]);
        d_symbols.insert(d_symbols.end(), in, in + noutput_items);
        return noutput_items;
    }

private:
    std::vector<Gnss_Synchro>& d_symbols;
};


struct Result
{
    std::vector<Gnss_Synchro> symbols;
    std::vector<int32_t> frame_tow_ms;
    size_t ephemerides = 0;
};

Result decode(const std::vector<Gnss_Synchro>& input)
{
    Result result;
    const gnss_shared_ptr<SynchroSource> source(new SynchroSource(input));
    const gnss_shared_ptr<SynchroSink> sink(new SynchroSink(result.symbols));
    const auto messages = gr::blocks::message_debug::make();
    const auto nav_messages = gr::blocks::message_debug::make();
    Tlm_Conf conf;
    conf.enable_navdata_monitor = true;
    const auto decoder = beidou_b1c_make_telemetry_decoder_gs(Gnss_Satellite("Beidou", prn), conf);
    auto top = gr::make_top_block("b1c_first_lock_test");
    top->connect(source, 0, decoder, 0);
    top->connect(decoder, 0, sink, 0);
    top->msg_connect(decoder, "telemetry", messages, "store");
    top->msg_connect(decoder, "Nav_msg_from_TLM", nav_messages, "store");
    top->run();
    for (size_t i = 0; i < messages->num_messages(); ++i)
        {
            const auto msg = messages->get_message(i);
            if (pmt::is_any(msg) && pmt::any_ref(msg).type() == typeid(std::shared_ptr<Beidou_Cnav1_Ephemeris>))
                {
                    ++result.ephemerides;
                }
        }
    for (size_t i = 0; i < nav_messages->num_messages(); ++i)
        {
            const auto msg = nav_messages->get_message(i);
            result.frame_tow_ms.push_back(
                any_cast<std::shared_ptr<Nav_Message_Packet>>(pmt::any_ref(msg))->tow_at_current_symbol_ms);
        }
    return result;
}

void expect_rejected(const std::vector<Gnss_Synchro>& input)
{
    const auto result = decode(input);
    EXPECT_EQ(result.ephemerides, 0U);
    EXPECT_TRUE(std::none_of(result.symbols.begin(), result.symbols.end(), [](const Gnss_Synchro& s) { return s.Flag_valid_word; }));
}
}  // namespace b1c_telemetry_test

TEST(BeidouB1cTelemetryDecoderTest, RecoversPreviousFrameAtFirstValidEpoch)
{
    using namespace b1c_telemetry_test;
    auto input = stream();
    input.erase(input.begin());  // The source has F1 through G0, exactly 1800 symbols.
    const auto result = decode(input);
    ASSERT_EQ(result.symbols.size(), static_cast<size_t>(n));
    EXPECT_EQ(result.ephemerides, 1U);
    EXPECT_FALSE(result.symbols[n - 2].Flag_valid_word);
    ASSERT_TRUE(result.symbols.back().Flag_valid_word);
    EXPECT_EQ(result.symbols.back().Tracking_sample_counter, input.back().Tracking_sample_counter);
    EXPECT_EQ(result.symbols.back().TOW_at_current_symbol_ms, 50010U);
}

TEST(BeidouB1cTelemetryDecoderTest, PreservesTimeAndNextFrameForBothDataAndPilotPolarities)
{
    using namespace b1c_telemetry_test;
    for (const float data_sign : {1.0F, -1.0F})
        {
            for (const float pilot_sign : {1.0F, -1.0F})
                {
                    auto input = stream(true, data_sign, pilot_sign, 2);
                    input.pop_back();  // Stop at the third frame's last symbol.
                    const auto result = decode(input);
                    ASSERT_EQ(result.symbols.size(), static_cast<size_t>(3 * n));
                    // Valid TOW can free-run after a missed frame. Require an
                    // ephemeris from each of the three complete frames as well.
                    EXPECT_EQ(result.ephemerides, 3U);
                    EXPECT_EQ(result.frame_tow_ms, (std::vector<int32_t>{50010, 68000, 86000}));
                    for (int t = n; t < 3 * n; ++t)
                        {
                            ASSERT_TRUE(result.symbols[t].Flag_valid_word) << t;
                            EXPECT_EQ(result.symbols[t].TOW_at_current_symbol_ms, 32000U + 10U * (t + 1)) << t;
                            EXPECT_EQ(result.symbols[t].Flag_PLL_180_deg_phase_locked, data_sign < 0.0F);
                        }
                }
        }
}

TEST(BeidouB1cTelemetryDecoderTest, AnchorsWrapEvenWhenRawCandidateIsDecodable)
{
    using namespace b1c_telemetry_test;
    auto input = decodable_without_wipe();
    input.pop_back();  // G1799 is the last available symbol; H0 cannot trigger a decode.
    std::vector<float> raw{static_cast<float>(input[n].Prompt_I)};
    for (int i = 1; i < n; ++i)
        {
            raw.push_back(input[i].Prompt_I);
        }
    Beidou_Cnav1_Navigation_Message nav;
    ASSERT_TRUE(nav.probe_subframe1_prn(raw.data(), n, prn));
    ASSERT_TRUE(nav.decode_frame_symbols(raw.data(), n, prn));
    ASSERT_TRUE(nav.have_new_ephemeris());
    EXPECT_DOUBLE_EQ(nav.get_tow_s(), 18.0);

    const auto result = decode(input);
    ASSERT_EQ(result.symbols.size(), static_cast<size_t>(2 * n));
    EXPECT_EQ(result.ephemerides, 2U);
    EXPECT_EQ(result.frame_tow_ms, (std::vector<int32_t>{50010, 68000}));
    ASSERT_TRUE(result.symbols[n].Flag_valid_word);
    EXPECT_EQ(result.symbols[n].TOW_at_current_symbol_ms, 50010U);
    ASSERT_TRUE(result.symbols.back().Flag_valid_word);
    EXPECT_EQ(result.symbols.back().TOW_at_current_symbol_ms, 68000U);
}

TEST(BeidouB1cTelemetryDecoderTest, DoesNotTreatANearbyPilotLagAsTheFirstLockWrap)
{
    using namespace b1c_telemetry_test;
    auto input = decodable_without_wipe();
    input.resize(n + 1);
    // The Q match is one symbol off, within the loose scan's search span.
    const auto* secondary = BEIDOU_B1C_PILOT_SECONDARY_CODE[prn - 1];
    for (int i = 0; i <= n; ++i)
        {
            input[i].Prompt_Q = secondary[(i + 1) % n] == '0' ? 8.0 : -8.0;
        }
    input[100].Prompt_Q = -input[100].Prompt_Q;  // Make the match loose.
    balance_pilot_for_raw_decode(input);
    expect_rejected(input);
}

TEST(BeidouB1cTelemetryDecoderTest, DecodesNormalPostLockFrame)
{
    using namespace b1c_telemetry_test;
    auto input = stream(true, 1.0F, 1.0F, 1);
    // Unusable pre-lock data forces the existing post-lock decode path. Place
    // lock after history is ready and away from the periodic search epoch.
    for (int i = 0; i < 100; ++i)
        {
            input[i].Prompt_I = 0.0;
        }
    input.insert(input.begin(), 100, Gnss_Synchro{});
    const auto result = decode(input);
    EXPECT_EQ(result.ephemerides, 1U);
    ASSERT_FALSE(result.symbols.empty());
    EXPECT_TRUE(result.symbols.back().Flag_valid_word);
    EXPECT_EQ(result.symbols.back().TOW_at_current_symbol_ms, 68010U);
}

TEST(BeidouB1cTelemetryDecoderTest, RejectsMissingOrRepeatedEpoch)
{
    using namespace b1c_telemetry_test;
    auto missing = stream();
    missing.erase(missing.begin() + 600);
    expect_rejected(missing);
    auto repeated = stream();
    repeated[600].Tracking_sample_counter = repeated[599].Tracking_sample_counter;
    expect_rejected(repeated);
}

TEST(BeidouB1cTelemetryDecoderTest, RejectsGapAndSampleRateChange)
{
    using namespace b1c_telemetry_test;
    auto gap = stream();
    for (size_t i = 600; i < gap.size(); ++i)
        {
            gap[i].Tracking_sample_counter += 80000;
        }
    expect_rejected(gap);
    auto rate = stream();
    rate[600].fs /= 2;
    expect_rejected(rate);
}

TEST(BeidouB1cTelemetryDecoderTest, RejectsSatelliteChangeAndWrongFramePrn)
{
    using namespace b1c_telemetry_test;
    auto changed = stream();
    changed[600].PRN = 25;
    expect_rejected(changed);
    changed = stream();
    changed.back().PRN = 25;
    expect_rejected(changed);
    auto wrong = stream();
    std::vector<float> sf1;
    bch(sf1, 25U, 21, 6, {2, 4, 5, 6});
    for (size_t i = 0; i < sf1.size(); ++i)
        {
            wrong[i].Prompt_I = sf1[i] * (BEIDOU_B1C_PILOT_SECONDARY_CODE[prn - 1][i] == '0' ? 1.0F : -1.0F);
        }
    expect_rejected(wrong);
}

TEST(BeidouB1cTelemetryDecoderTest, RejectsValidLdpcCodewordWithBadCrc)
{
    using namespace b1c_telemetry_test;
    const auto bad = frame(1, false);
    Beidou_Cnav1_Navigation_Message nav;
    int32_t fail_stage = -1;
    EXPECT_TRUE(nav.probe_subframe1_prn(bad.data(), n, prn));
    EXPECT_FALSE(nav.decode_frame_symbols(bad.data(), n, prn, &fail_stage));
    EXPECT_EQ(fail_stage, 6);  // LDPC succeeded, SF2 CRC failed.
    EXPECT_FALSE(nav.have_new_ephemeris());
    expect_rejected(stream(false));
}
