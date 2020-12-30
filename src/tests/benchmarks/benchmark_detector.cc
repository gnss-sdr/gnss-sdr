/*!
 * \file benchmark_detector.cc
 * \brief Benchmark for preamble detection implementations
 * \author Carles Fernandez-Prades, 2020. cfernandez(at)cttc.es
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

#include "GPS_L1_CA.h"
#include <benchmark/benchmark.h>
#include <algorithm>  // for std::generate
#include <array>
#include <cmath>  // std::for signbit
#include <cstdint>
#include <functional>  // for std::plus
#include <numeric>     // for std::accumulate, std::inner_product
#include <random>
#include <vector>
#if COMPILER_HAS_STD_TRANSFORM_REDUCE_WITH_POLICY
#include <execution>
#endif

void bm_forloop(benchmark::State& state)
{
    std::vector<float> d_symbol_history(GPS_CA_PREAMBLE_LENGTH_SYMBOLS, 0.0);
    std::array<int32_t, GPS_CA_PREAMBLE_LENGTH_SYMBOLS> d_preamble_samples{};

    // fill the inputs
    std::random_device rd;
    std::default_random_engine e2(rd());
    std::uniform_real_distribution<> dist(-1.0, 1.0);
    std::generate(d_symbol_history.begin(), d_symbol_history.end(), [&dist, &e2]() { return dist(e2); });

    std::generate(d_preamble_samples.begin(), d_preamble_samples.end(), [n = 0]() mutable { return (GPS_CA_PREAMBLE_SYMBOLS_STR[n++] == '1' ? 1 : -1); });

    while (state.KeepRunning())
        {
            int32_t corr_value = 0;
            for (size_t i = 0; i < d_preamble_samples.size(); i++)
                {
                    if (d_symbol_history[i] < 0.0)
                        {
                            corr_value -= d_preamble_samples[i];
                        }
                    else
                        {
                            corr_value += d_preamble_samples[i];
                        }
                }
        }
}


void bm_accumulate(benchmark::State& state)
{
    std::vector<float> d_symbol_history(GPS_CA_PREAMBLE_LENGTH_SYMBOLS, 0.0);
    std::array<int32_t, GPS_CA_PREAMBLE_LENGTH_SYMBOLS> d_preamble_samples{};

    // fill the inputs
    std::random_device rd;
    std::default_random_engine e2(rd());
    std::uniform_real_distribution<> dist(-1.0, 1.0);
    std::generate(d_symbol_history.begin(), d_symbol_history.end(), [&dist, &e2]() { return dist(e2); });

    std::generate(d_preamble_samples.begin(), d_preamble_samples.end(), [n = 0]() mutable { return (GPS_CA_PREAMBLE_SYMBOLS_STR[n++] == '1' ? 1 : -1); });

    while (state.KeepRunning())
        {
            int32_t corr_value = 0;
            corr_value += std::accumulate(d_symbol_history.begin(),
                d_symbol_history.end(),
                0,
                [&d_preamble_samples, n = 0](float a, float b) mutable { return (b > 0.0 ? a + d_preamble_samples[n++] : a - d_preamble_samples[n++]); });
        }
}


void bm_inner_product(benchmark::State& state)
{
    std::vector<float> d_symbol_history(GPS_CA_PREAMBLE_LENGTH_SYMBOLS, 0.0);
    std::array<int32_t, GPS_CA_PREAMBLE_LENGTH_SYMBOLS> d_preamble_samples{};

    // fill the inputs
    std::random_device rd;
    std::default_random_engine e2(rd());
    std::uniform_real_distribution<> dist(-1.0, 1.0);
    std::generate(d_symbol_history.begin(), d_symbol_history.end(), [&dist, &e2]() { return dist(e2); });

    std::generate(d_preamble_samples.begin(), d_preamble_samples.end(), [n = 0]() mutable { return (GPS_CA_PREAMBLE_SYMBOLS_STR[n++] == '1' ? 1 : -1); });

    while (state.KeepRunning())
        {
            int32_t corr_value = 0;
            corr_value += std::inner_product(d_symbol_history.begin(),
                d_symbol_history.end(),
                d_preamble_samples.begin(),
                0,
#if COMPILER_HAS_STD_PLUS_VOID
                std::plus<>(),
#else
                std::plus<int32_t>(),
#endif
                [](float a, int32_t b) { return (std::signbit(a) ? -b : b); });
        }
}


#if COMPILER_HAS_STD_TRANSFORM_REDUCE
void bm_transform_reduce(benchmark::State& state)
{
    std::vector<float> d_symbol_history(GPS_CA_PREAMBLE_LENGTH_SYMBOLS, 0.0);
    std::array<int32_t, GPS_CA_PREAMBLE_LENGTH_SYMBOLS> d_preamble_samples{};

    // fill the inputs
    std::random_device rd;
    std::default_random_engine e2(rd());
    std::uniform_real_distribution<> dist(-1.0, 1.0);
    std::generate(d_symbol_history.begin(), d_symbol_history.end(), [&dist, &e2]() { return dist(e2); });

    std::generate(d_preamble_samples.begin(), d_preamble_samples.end(), [n = 0]() mutable { return (GPS_CA_PREAMBLE_SYMBOLS_STR[n++] == '1' ? 1 : -1); });

    while (state.KeepRunning())
        {
            int32_t corr_value = 0;
            corr_value += std::transform_reduce(d_symbol_history.begin(),
                d_symbol_history.end(),
                d_preamble_samples.begin(),
                0,
                std::plus<>(),
                [](auto a, auto b) { return (std::signbit(a) ? -b : b); });
        }
}
#endif


#if COMPILER_HAS_STD_TRANSFORM_REDUCE_WITH_POLICY
void bm_transform_reduce_policy(benchmark::State& state)
{
    std::vector<float> d_symbol_history(GPS_CA_PREAMBLE_LENGTH_SYMBOLS, 0.0);
    std::array<int32_t, GPS_CA_PREAMBLE_LENGTH_SYMBOLS> d_preamble_samples{};

    // fill the inputs
    std::random_device rd;
    std::default_random_engine e2(rd());
    std::uniform_real_distribution<> dist(-1.0, 1.0);
    std::generate(d_symbol_history.begin(), d_symbol_history.end(), [&dist, &e2]() { return dist(e2); });

    std::generate(d_preamble_samples.begin(), d_preamble_samples.end(), [n = 0]() mutable { return (GPS_CA_PREAMBLE_SYMBOLS_STR[n++] == '1' ? 1 : -1); });

    while (state.KeepRunning())
        {
            int32_t corr_value = 0;
            corr_value += std::transform_reduce(
                std::execution::par,
                d_symbol_history.begin(),
                d_symbol_history.end(),
                d_preamble_samples.begin(),
                0,
                std::plus<>(),
                [](auto a, auto b) { return (std::signbit(a) ? -b : b); });
        }
}
#endif


BENCHMARK(bm_forloop);
BENCHMARK(bm_accumulate);
BENCHMARK(bm_inner_product);
#if COMPILER_HAS_STD_TRANSFORM_REDUCE
BENCHMARK(bm_transform_reduce);
#endif
#if COMPILER_HAS_STD_TRANSFORM_REDUCE_WITH_POLICY
BENCHMARK(bm_transform_reduce_policy);
#endif
BENCHMARK_MAIN();
