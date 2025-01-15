/*!
 * \file benchmark_preamble.cc
 * \brief Benchmark for preamble conversion implementations
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
#include <algorithm>
#include <array>

void bm_forloop(benchmark::State& state)
{
    std::array<int32_t, GPS_CA_PREAMBLE_LENGTH_SYMBOLS> d_preamble_samples{};
    volatile int32_t res;  // Prevent the compiler from optimizing the loop away
    while (state.KeepRunning())
        {
            int32_t n = 0;
            for (int32_t i = 0; i < GPS_CA_PREAMBLE_LENGTH_SYMBOLS; i++)
                {
                    if (GPS_CA_PREAMBLE_SYMBOLS_STR[i] == '1')
                        {
                            d_preamble_samples[n] = 1;
                            n++;
                        }
                    else
                        {
                            d_preamble_samples[n] = -1;
                            n++;
                        }
                }
            res = d_preamble_samples[GPS_CA_PREAMBLE_LENGTH_SYMBOLS - 1];
        }
    if (res > 1)
        {
            // Avoid unused-but-set-variable warning
        }
}


void bm_generate(benchmark::State& state)
{
    std::array<int32_t, GPS_CA_PREAMBLE_LENGTH_SYMBOLS> d_preamble_samples{};
    volatile int32_t res;  // Prevent the compiler from optimizing the loop away
    while (state.KeepRunning())
        {
            std::generate(d_preamble_samples.begin(), d_preamble_samples.end(), [n = 0]() mutable { return (GPS_CA_PREAMBLE_SYMBOLS_STR[n++] == '1' ? 1 : -1); });
            res = d_preamble_samples[GPS_CA_PREAMBLE_LENGTH_SYMBOLS - 1];
        }
    if (res > 1)
        {
            // Avoid unused-but-set-variable warning
        }
}


BENCHMARK(bm_forloop);
BENCHMARK(bm_generate);
BENCHMARK_MAIN();
