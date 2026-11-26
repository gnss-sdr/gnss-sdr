/*!
 * \file benchmark_labsat_source.cc
 * \brief Benchmark for labsat file processing
 * \author Julien Edmond, 2026. julien@xonaspace.com
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "INIReader.h"
#include "concurrent_queue.h"
#include "gnss_sdr_filesystem.h"
#include "labsat23_source.h"
#include <benchmark/benchmark.h>
#include <gnuradio/blocks/head.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/top_block.h>
#include <pmt/pmt.h>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <streambuf>
#include <string>
#include <vector>

namespace
{

constexpr int kSamplesPerOutput = 8;
constexpr int kRegistersPerOutput = 3;
constexpr int kOutputsPerIteration = 65536;
constexpr int64_t kSamplesPerIteration = static_cast<int64_t>(kSamplesPerOutput) * kOutputsPerIteration;
constexpr int64_t kSourceBytesPerIteration = static_cast<int64_t>(kRegistersPerOutput) * kOutputsPerIteration * sizeof(uint64_t);

// Utility class to silence prints to std::cout
class ScopedCoutSilencer
{
public:
    ScopedCoutSilencer() : previous_buffer_(std::cout.rdbuf(&null_buffer_)) {}

    ScopedCoutSilencer(const ScopedCoutSilencer&) = delete;
    ScopedCoutSilencer(ScopedCoutSilencer&&) = delete;
    ScopedCoutSilencer& operator=(const ScopedCoutSilencer&) = delete;
    ScopedCoutSilencer& operator=(ScopedCoutSilencer&&) = delete;

    ~ScopedCoutSilencer()
    {
        std::cout.rdbuf(previous_buffer_);
    }

private:
    class NullBuffer : public std::streambuf
    {
    protected:
        int overflow(int character) override
        {
            return traits_type::not_eof(character);
        }
    };

    NullBuffer null_buffer_;
    std::streambuf* previous_buffer_;
};

fs::path temporary_path()
{
    const auto stamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    fs::path path = fs::temp_directory_path() / ("gnss_sdr_labsat4_benchmark_" + std::to_string(stamp));
    path.replace_extension(".LS4");
    return path;
}

void write_ls4_capture(const fs::path& path)
{
    std::ofstream output(path.string().c_str(), std::ios::binary);
    const uint64_t registers[kRegistersPerOutput] = {
        0x001001FFF800FFF0ULL, 0x017FF000FFF80000ULL, 0x8007FF0000000000ULL};
    for (int output_index = 0; output_index < kOutputsPerIteration; ++output_index)
        {
            for (const auto register_value : registers)
                {
                    for (int byte_index = 0; byte_index < 8; ++byte_index)
                        {
                            output.put(static_cast<char>((register_value >> (8 * byte_index)) & 0xFFU));
                        }
                }
        }
}

void write_ls4_ini(const fs::path& path)
{
    fs::path ini_path(path);
    ini_path.replace_extension(".ini");
    std::ofstream ini(ini_path.string().c_str());
    ini << "[config]\n"
        << "OSC=OCXO\n"
        << "SMP=20460000\n"
        << "QUA=12\n"
        << "CHN=1\n"
        << "SFT=24\n"
        << "BW_MAX=20000000\n"
        << "BW_DIV_A=2\n"
        << "BW_DIV_B=2\n"
        << "BW_DIV_C=1\n"
        << "[channel A]\n"
        << "CFA=1575420000\n"
        << "BWA=10000000\n"
        << "BUF_SIZE_A=" << kSourceBytesPerIteration << "\n"
        << "[channel B]\n"
        << "CFB=1227600000\n"
        << "BWB=0\n"
        << "BUF_SIZE_B=0\n"
        << "[channel C]\n"
        << "CFC=1176450000\n"
        << "BWC=0\n"
        << "BUF_SIZE_C=0\n";
}

void remove_ls4_fixture(const fs::path& path)
{
    const auto capture_filename = path.string();
    std::remove(capture_filename.c_str());
    fs::path ini_path(path);
    ini_path.replace_extension(".ini");
    const auto ini_filename = ini_path.string();
    std::remove(ini_filename.c_str());
}

std::vector<int> parse_channels(const char* value)
{
    std::vector<int> channels;
    std::stringstream stream(value);
    std::string channel_value;
    while (std::getline(stream, channel_value, ','))
        {
            const size_t first_non_space = channel_value.find_first_not_of(" \t");
            const size_t last_non_space = channel_value.find_last_not_of(" \t");
            if (first_non_space == std::string::npos)
                {
                    return {};
                }
            try
                {
                    size_t parsed_length = 0;
                    const int channel = std::stoi(channel_value.substr(first_non_space, last_non_space - first_non_space + 1), &parsed_length);
                    if (parsed_length != last_non_space - first_non_space + 1 || channel < 1 || channel > 3)
                        {
                            return {};
                        }
                    channels.push_back(channel);
                }
            catch (const std::exception&)
                {
                    return {};
                }
        }
    return channels;
}

std::string channel_list_label(const std::vector<int>& channels)
{
    std::ostringstream label;
    for (size_t index = 0; index < channels.size(); ++index)
        {
            if (index > 0)
                {
                    label << ',';
                }
            label << channels[index];
        }
    return label.str();
}

// Benchmark with a controlled generated file with 1 channel at 20460000sps
void benchmark_labsat4_source(benchmark::State& state)
{
    ScopedCoutSilencer cout_silencer;
    const fs::path capture_path = temporary_path();
    write_ls4_capture(capture_path);
    write_ls4_ini(capture_path);

    for (auto _ : state)
        {
            state.PauseTiming();
            Concurrent_Queue<pmt::pmt_t> queue;
            auto source = labsat23_make_source_sptr(capture_path.string().c_str(), {1}, &queue, false, 0.0);
            auto top_block = gr::make_top_block("benchmark_labsat4_source");
            auto sink = gr::blocks::null_sink::make(sizeof(gr_complex));
            top_block->connect(source, 0, sink, 0);
            state.ResumeTiming();

            top_block->run();
        }

    remove_ls4_fixture(capture_path);
    state.SetItemsProcessed(state.iterations() * kSamplesPerIteration);
    state.SetBytesProcessed(state.iterations() * kSamplesPerIteration * static_cast<int64_t>(sizeof(gr_complex)));
    state.counters["complex_samples/s"] = benchmark::Counter(kSamplesPerIteration, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["source_bytes/s"] = benchmark::Counter(
        static_cast<double>(kSourceBytesPerIteration), benchmark::Counter::kIsIterationInvariantRate);
}

// Benchmark with a file, only enabled if GNSSSDR_LS4_FILE is set in the environment
// It reads the following env variables:
// - GNSSSDR_LS4_FILE: path to the file to benchmark
// - GNSSSDR_LS4_CHANNELS: labsat to gnuradio channel mapping, defined the same way as SignalSource.selected_channel
// - GNSSSDR_LS4_DURATION_S: the duration of the benchmark, defaults to 5s
void benchmark_labsat4_capture(benchmark::State& state)
{
    const char* capture_filename = std::getenv("GNSSSDR_LS4_FILE");
    if (capture_filename == nullptr)
        {
            state.SkipWithMessage("Set GNSSSDR_LS4_FILE to an LS4 capture path.");
            return;
        }

    const char* channels_value = std::getenv("GNSSSDR_LS4_CHANNELS");
    const std::vector<int> channels = parse_channels(channels_value == nullptr ? "1" : channels_value);
    if (channels.empty())
        {
            state.SkipWithError("GNSSSDR_LS4_CHANNELS must be a comma-separated list of channel numbers from 1 through 3.");
            return;
        }

    fs::path ini_path(capture_filename);
    ini_path.replace_extension(".ini");
    INIReader ini_reader(ini_path.string());
    if (ini_reader.ParseError() < 0)
        {
            state.SkipWithError("The LS4 companion INI file could not be opened.");
            return;
        }

    const int64_t sample_rate = ini_reader.GetInteger("config", "SMP", 0);
    const int64_t bandwidth_divisor = ini_reader.GetInteger("config", "BW_DIV_" + std::string(1, static_cast<char>('A' + channels.front() - 1)), 0);
    if (sample_rate <= 0 || bandwidth_divisor <= 0)
        {
            state.SkipWithError("The LS4 INI must define positive SMP and selected-channel BW_DIV values.");
            return;
        }
    for (const int channel : channels)
        {
            const int64_t selected_bandwidth_divisor = ini_reader.GetInteger("config", "BW_DIV_" + std::string(1, static_cast<char>('A' + channel - 1)), 0);
            if (selected_bandwidth_divisor != bandwidth_divisor)
                {
                    state.SkipWithError("All selected LS4 channels must have the same BW_DIV value.");
                    return;
                }
        }

    const char* duration_value = std::getenv("GNSSSDR_LS4_DURATION_S");
    const double duration_seconds = duration_value == nullptr ? 5.0 : std::strtod(duration_value, nullptr);
    if (duration_seconds <= 0.0)
        {
            state.SkipWithError("GNSSSDR_LS4_DURATION_S must be positive.");
            return;
        }

    const int64_t output_rate = sample_rate / bandwidth_divisor;
    auto samples_to_measure = static_cast<int64_t>(output_rate * duration_seconds);
    samples_to_measure -= samples_to_measure % kSamplesPerOutput;
    if (samples_to_measure == 0)
        {
            state.SkipWithError("The requested duration is shorter than one LS4 output block.");
            return;
        }

    ScopedCoutSilencer cout_silencer;
    for (auto _ : state)
        {
            state.PauseTiming();
            Concurrent_Queue<pmt::pmt_t> queue;
            auto source = labsat23_make_source_sptr(capture_filename, channels, &queue, false, 0.0);
            auto top_block = gr::make_top_block("benchmark_labsat4_capture");
            std::vector<gr::blocks::head::sptr> heads;
            std::vector<gr::blocks::null_sink::sptr> sinks;
            heads.reserve(channels.size());
            sinks.reserve(channels.size());
            for (size_t output_index = 0; output_index < channels.size(); ++output_index)
                {
                    heads.push_back(gr::blocks::head::make(sizeof(gr_complex), samples_to_measure));
                    sinks.push_back(gr::blocks::null_sink::make(sizeof(gr_complex)));
                    top_block->connect(source, static_cast<int>(output_index), heads.back(), 0);
                    top_block->connect(heads.back(), 0, sinks.back(), 0);
                }
            state.ResumeTiming();

            top_block->run();
        }

    const int64_t output_samples_per_iteration = samples_to_measure * static_cast<int64_t>(channels.size());
    state.SetItemsProcessed(state.iterations() * output_samples_per_iteration);
    state.SetBytesProcessed(state.iterations() * output_samples_per_iteration * static_cast<int64_t>(sizeof(gr_complex)));
    state.SetLabel("channels " + channel_list_label(channels) + ", " + std::to_string(output_rate) + " samples/s/output");
    state.counters["complex_samples/s"] = benchmark::Counter(output_samples_per_iteration, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["output_channels"] = static_cast<double>(channels.size());
    // Ratio between the expected number of samples and the benchmark run time.
    // The kIsIterationInvariantRate flag is necessary to make it divided by the benchmark run time (even though it misleadingly add a /s at the end of the ratio)
    state.counters["real_time_factor"] = benchmark::Counter(
        static_cast<double>(samples_to_measure) / static_cast<double>(output_rate), benchmark::Counter::kIsIterationInvariantRate);
}

}  // namespace

BENCHMARK(benchmark_labsat4_source)->Unit(benchmark::kMillisecond)->UseRealTime();
BENCHMARK(benchmark_labsat4_capture)->Unit(benchmark::kMillisecond)->UseRealTime()->Iterations(1);

BENCHMARK_MAIN();
