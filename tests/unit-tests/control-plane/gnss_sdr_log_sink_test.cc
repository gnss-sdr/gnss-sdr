/*!
 * \file gnss_sdr_log_sink_test.cc
 * \brief Tests for per-run Abseil file logging.
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

#include "gnss_sdr_filesystem.h"
#include "gnss_sdr_log_sink.h"
#include <gtest/gtest.h>
#include <absl/log/log.h>
#include <absl/log/log_sink_registry.h>
#include <absl/time/clock.h>
#include <absl/time/time.h>
#include <atomic>
#include <fstream>
#include <iterator>
#include <regex>
#include <set>
#include <string>
#include <thread>
#include <vector>
#if !defined(_WIN32)
#include <sys/stat.h>
#include <unistd.h>
#endif

class GnssSdrLogSinkTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        do
            {
                directory = fs::temp_directory_path() / ("gnss-sdr-log-sink-test-" + std::to_string(absl::ToUnixNanos(absl::Now())));
            }
        while (!fs::create_directory(directory));
    }

    void TearDown() override
    {
        errorlib::error_code error;
        fs::remove_all(directory, error);
    }

    static std::string Read(const fs::path& path)
    {
        std::ifstream file(path.string(), std::ios::binary);
        return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    }

    // Compare the exact bytes supplied by Abseil, without assuming a timezone
    // or reimplementing its prefix formatting in the test.
    class ForwardingSink : public absl::LogSink
    {
    public:
        explicit ForwardingSink(GnssSdrLogSink& sink) : sink(sink) {}
        void Send(const absl::LogEntry& entry) override
        {
            expected.append(entry.text_message_with_prefix_and_newline().data(), entry.text_message_with_prefix_and_newline().size());
            sink.Send(entry);
        }
        GnssSdrLogSink& sink;
        std::string expected;
    };

    fs::path directory;
};


TEST_F(GnssSdrLogSinkTest, CreatesDirectoriesAndPreservesEveryBasename)
{
    const auto nested = directory / "nested" / "logs";
    for (const auto* basename : {"gnss-sdr", "run_tests", "test", "front_end_cal", "position_test"})
        {
            GnssSdrLogSink sink(nested.string(), basename);
            const fs::path logfile(sink.filename());
            EXPECT_TRUE(fs::is_regular_file(logfile));
            EXPECT_EQ(logfile.parent_path(), nested);
            EXPECT_TRUE(std::regex_match(logfile.filename().string(), std::regex(std::string(basename) + R"(\.[0-9]{8}-[0-9]{6}\.[0-9]+\.log)")));
#if !defined(_WIN32)
            EXPECT_EQ(fs::read_symlink(nested / (std::string(basename) + ".log")), logfile.filename());
#endif
        }
}


TEST_F(GnssSdrLogSinkTest, RepeatedCreationPreservesFilesAndUpdatesRelativeLink)
{
    std::set<std::string> filenames;
    for (int run = 0; run < 20; ++run)
        {
            GnssSdrLogSink sink(directory.string());
            EXPECT_TRUE(filenames.insert(sink.filename()).second);
            LOG(INFO).NoPrefix().ToSinkOnly(&sink) << "run " << run;
#if !defined(_WIN32)
            EXPECT_EQ(fs::read_symlink(directory / "gnss-sdr.log"), fs::path(sink.filename()).filename());
#endif
        }
    for (const auto& filename : filenames)
        {
            EXPECT_EQ(Read(filename).substr(0, 4), "run ");
        }
}


TEST_F(GnssSdrLogSinkTest, PreservesAbseilFormattingAndFlushesErrors)
{
    GnssSdrLogSink sink(directory.string());
    ForwardingSink capture(sink);
    LOG(INFO).AtLocation("log_sink_test.cc", 42).ToSinkOnly(&capture) << "buffered message";
    sink.Flush();
    EXPECT_EQ(Read(sink.filename()), capture.expected);
    EXPECT_NE(capture.expected.find("log_sink_test.cc:42] buffered message\n"), std::string::npos);
    LOG(ERROR).NoPrefix().ToSinkOnly(&capture) << "flush immediately";
    EXPECT_EQ(Read(sink.filename()), capture.expected);
}


TEST_F(GnssSdrLogSinkTest, DestructorFlushesBufferedMessages)
{
    std::string filename;
    {
        GnssSdrLogSink sink(directory.string());
        filename = sink.filename();
        LOG(INFO).NoPrefix().ToSinkOnly(&sink) << "flushed on destruction";
    }
    EXPECT_EQ(Read(filename), "flushed on destruction\n");
}


TEST_F(GnssSdrLogSinkTest, ConcurrentSendAndFlushPreserveWholeMessages)
{
    GnssSdrLogSink sink(directory.string());
    std::atomic<bool> done{false};
    std::thread flusher([&sink, &done]() {
        while (!done.load())
            {
                sink.Flush();
                std::this_thread::yield();
            }
    });
    std::vector<std::thread> writers;
    writers.reserve(8);
    for (int writer = 0; writer < 8; ++writer)
        {
            writers.emplace_back([&sink, writer]() {
                for (int message = 0; message < 250; ++message)
                    {
                        LOG(INFO).NoPrefix().ToSinkOnly(&sink) << writer << ":" << message;
                    }
            });
        }
    for (auto& writer : writers)
        {
            writer.join();
        }
    done.store(true);
    flusher.join();
    sink.Flush();
    std::ifstream file(sink.filename());
    std::set<std::string> messages;
    std::string line;
    while (std::getline(file, line))
        {
            EXPECT_TRUE(messages.insert(line).second);
        }
    EXPECT_EQ(messages.size(), 2000U);
    for (int writer = 0; writer < 8; ++writer)
        {
            for (int message = 0; message < 250; ++message)
                {
                    EXPECT_EQ(messages.count(std::to_string(writer) + ":" + std::to_string(message)), 1U);
                }
        }
}


TEST_F(GnssSdrLogSinkTest, RejectsInvalidDirectoryAndBasename)
{
    const auto file = directory / "file";
    std::ofstream(file.string()) << "keep me";
    EXPECT_THROW(GnssSdrLogSink sink(file.string()), std::exception);
    EXPECT_THROW(GnssSdrLogSink sink((file / "logs").string()), std::exception);
    EXPECT_THROW(GnssSdrLogSink sink(directory.string(), "../escape"), std::invalid_argument);
    EXPECT_EQ(Read(file), "keep me");
}


TEST_F(GnssSdrLogSinkTest, GuardRegistersFlushesAndUnregisters)
{
    GnssSdrLogSinkGuard guard;
    guard.Register(directory.string());
    EXPECT_THROW(guard.Register(directory.string()), std::logic_error);
    LOG(INFO) << "registered message";
    absl::FlushLogSinks();
    fs::path logfile;
    for (const auto& entry : fs::directory_iterator(directory))
        {
            if (!fs::is_symlink(entry.path()) && entry.path().extension() == ".log")
                {
                    logfile = entry.path();
                }
        }
    ASSERT_FALSE(logfile.empty());
    EXPECT_NE(Read(logfile).find("registered message"), std::string::npos);
    guard.Shutdown();
    guard.Shutdown();
    const auto contents = Read(logfile);
    LOG(INFO) << "after unregister";
    EXPECT_EQ(Read(logfile), contents);
}


#if !defined(_WIN32)
TEST_F(GnssSdrLogSinkTest, PreservesLegacyStableLogfile)
{
    std::ofstream((directory / "gnss-sdr.log").string()) << "legacy log";
    GnssSdrLogSink sink(directory.string());
    EXPECT_EQ(Read(sink.filename() + ".previous"), "legacy log");
    EXPECT_EQ(fs::read_symlink(directory / "gnss-sdr.log"), fs::path(sink.filename()).filename());
}


TEST_F(GnssSdrLogSinkTest, SymlinkFailureLeavesLoggingUsableAndCleansTemporaryLink)
{
    fs::create_directory(directory / "gnss-sdr.log");
    GnssSdrLogSink sink(directory.string());
    LOG(ERROR).NoPrefix().ToSinkOnly(&sink) << "still logging";
    EXPECT_EQ(Read(sink.filename()), "still logging\n");
    EXPECT_TRUE(fs::is_directory(directory / "gnss-sdr.log"));
    for (const auto& entry : fs::directory_iterator(directory))
        {
            EXPECT_EQ(entry.path().filename().string().find(".link."), std::string::npos);
        }
}


TEST_F(GnssSdrLogSinkTest, RejectsUnwritableDirectory)
{
    if (geteuid() == 0)
        {
            GTEST_SKIP() << "Root can write despite directory permissions";
        }
    const auto readonly = directory / "readonly";
    fs::create_directory(readonly);
    ASSERT_EQ(chmod(readonly.c_str(), 0500), 0);
    EXPECT_THROW(GnssSdrLogSink sink(readonly.string()), std::exception);
    EXPECT_EQ(chmod(readonly.c_str(), 0700), 0);
}
#endif
