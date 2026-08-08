/*!
 * \file sbas_l1_telemetry_decoder_test.cc
 * \brief Unit tests for SBAS L1 telemetry decoder timestamps, signal flags,
 *  and dump-file session handling.
 * \author Miguel Gómez López, 2026. mgomezl(at)ing.uc3m.es
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

#include "SBAS_L1.h"
#include "gnss_sdr_filesystem.h"
#include "sbas_l1_telemetry_decoder_gs.h"
#include "signal_flag.h"
#include <gtest/gtest.h>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>

namespace sbas_l1_telemetry_decoder_test
{
class ScopedDumpFile
{
public:
    ScopedDumpFile()
    {
        const auto stamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        path = fs::temp_directory_path() / ("sbas_l1_dump_session_" + std::to_string(stamp) + ".ems");
    }

    ~ScopedDumpFile()
    {
        errorlib::error_code error;
        fs::remove(path, error);
    }

    ScopedDumpFile(const ScopedDumpFile &) = delete;
    ScopedDumpFile &operator=(const ScopedDumpFile &) = delete;

    fs::path path;
};


std::string read_file(const fs::path &path)
{
    std::ifstream stream(path.string());
    return std::string(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
}
}  // namespace sbas_l1_telemetry_decoder_test

TEST(SbasL1TelemetryDecoderTest, SignalFlagDoesNotAliasBeidouB1C)
{
    EXPECT_NE(static_cast<uint32_t>(SBAS_S1), static_cast<uint32_t>(BDS_B1C));
}

TEST(SbasL1TelemetryDecoderTest, TimestampsFollowBitsDelayedByViterbiTraceback)
{
    constexpr int32_t bits_per_block = 30;
    constexpr int32_t traceback_depth = 35;
    const double bit_period_s = 1.0 / static_cast<double>(SBAS_L1_DATA_RATE_BPS);
    std::deque<double> pending_stamps;

    const auto make_stamps = [bit_period_s](int32_t first_bit) {
        std::vector<double> stamps(bits_per_block);
        for (int32_t i = 0; i < bits_per_block; ++i)
            {
                stamps[static_cast<size_t>(i)] = static_cast<double>(first_bit + i) * bit_period_s;
            }
        return stamps;
    };

    const auto block_1 = make_stamps(0);
    const auto output_1 = sbas_l1_telemetry_decoder_gs::consume_decoded_timestamps(pending_stamps, block_1, 0);
    EXPECT_TRUE(output_1.empty());
    EXPECT_EQ(pending_stamps.size(), static_cast<size_t>(bits_per_block));

    const auto block_2 = make_stamps(bits_per_block);
    const int32_t startup_decoded_bits = 2 * bits_per_block - traceback_depth;
    const auto output_2 = sbas_l1_telemetry_decoder_gs::consume_decoded_timestamps(pending_stamps, block_2, startup_decoded_bits);
    ASSERT_EQ(output_2.size(), static_cast<size_t>(startup_decoded_bits));
    EXPECT_DOUBLE_EQ(output_2.front(), block_1.front());
    EXPECT_DOUBLE_EQ(output_2.back(), block_1[static_cast<size_t>(startup_decoded_bits - 1)]);
    EXPECT_EQ(pending_stamps.size(), static_cast<size_t>(traceback_depth));

    const auto block_3 = make_stamps(2 * bits_per_block);
    const auto output_3 = sbas_l1_telemetry_decoder_gs::consume_decoded_timestamps(pending_stamps, block_3, bits_per_block);
    ASSERT_EQ(output_3.size(), static_cast<size_t>(bits_per_block));
    EXPECT_DOUBLE_EQ(output_3.front(), block_1[static_cast<size_t>(startup_decoded_bits)]);
    EXPECT_DOUBLE_EQ(output_3.back(), block_2[static_cast<size_t>(startup_decoded_bits - 1)]);
    EXPECT_LT(output_3.front(), block_3.front());
    EXPECT_EQ(pending_stamps.size(), static_cast<size_t>(traceback_depth));
}


TEST(SbasL1TelemetryDecoderTest, DumpSessionAppendsAfterReopenAndTruncatesNextRun)
{
    sbas_l1_telemetry_decoder_test::ScopedDumpFile dump_file;
    const auto filename = dump_file.path.string();
    {
        std::ofstream stale_file(filename);
        stale_file << "stale data\n";
    }

    {
        SbasL1DumpSession session;
        std::ofstream stream;
        EXPECT_TRUE(session.open(stream, filename));
        ASSERT_TRUE(stream.is_open());
        stream << "first message\n";
        stream.close();

        EXPECT_FALSE(session.open(stream, filename));
        ASSERT_TRUE(stream.is_open());
        stream << "second message\n";
        stream.close();
    }
    EXPECT_EQ("first message\nsecond message\n", sbas_l1_telemetry_decoder_test::read_file(dump_file.path));

    {
        SbasL1DumpSession next_run_session;
        std::ofstream stream;
        EXPECT_TRUE(next_run_session.open(stream, filename));
        ASSERT_TRUE(stream.is_open());
        stream << "next run\n";
    }
    EXPECT_EQ("next run\n", sbas_l1_telemetry_decoder_test::read_file(dump_file.path));
}
