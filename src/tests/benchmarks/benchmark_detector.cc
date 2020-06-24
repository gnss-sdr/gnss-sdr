/*!
 * \file benchmark_detector.cc
 * \brief Benchmark for preamble detection implementations
 * \author Carles Fernandez-Prades, 2020. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#include "GPS_L1_CA.h"
#include <benchmark/benchmark.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <numeric>
#include <random>
#include <vector>


void bm_forloop(benchmark::State& state)
{
    std::vector<float> d_symbol_history(GPS_CA_PREAMBLE_LENGTH_SYMBOLS, 0.0);
    std::array<int32_t, GPS_CA_PREAMBLE_LENGTH_BITS> d_preamble_samples{};

    // fill the inputs
    std::random_device rd;
    std::default_random_engine e2(rd());
    std::uniform_real_distribution<> dist(-1.0, 1.0);
    std::generate(d_symbol_history.begin(), d_symbol_history.end(), [&dist, &e2]() { return dist(e2); });

    std::generate(d_preamble_samples.begin(), d_preamble_samples.end(), [n = 0]() mutable { return (GPS_CA_PREAMBLE[n++] == '1' ? 1 : -1); });

    while (state.KeepRunning())
        {
            int32_t corr_value = 0;
            for (int32_t i = 0; i < GPS_CA_PREAMBLE_LENGTH_BITS; i++)
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
    std::array<int32_t, GPS_CA_PREAMBLE_LENGTH_BITS> d_preamble_samples{};

    // fill the inputs
    std::random_device rd;
    std::default_random_engine e2(rd());
    std::uniform_real_distribution<> dist(-1.0, 1.0);
    std::generate(d_symbol_history.begin(), d_symbol_history.end(), [&dist, &e2]() { return dist(e2); });

    std::generate(d_preamble_samples.begin(), d_preamble_samples.end(), [n = 0]() mutable { return (GPS_CA_PREAMBLE[n++] == '1' ? 1 : -1); });

    while (state.KeepRunning())
        {
            int32_t corr_value = 0;
            corr_value += std::accumulate(d_symbol_history.begin(),
                d_symbol_history.begin() + GPS_CA_PREAMBLE_LENGTH_BITS,
                0,
                [&d_preamble_samples, n = 0](float a, float b) mutable { return (b > 0.0 ? a + d_preamble_samples[n++] : a - d_preamble_samples[n++]); });
        }
}


BENCHMARK(bm_forloop);
BENCHMARK(bm_accumulate);
BENCHMARK_MAIN();
