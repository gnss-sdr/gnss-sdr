/*!
 * \file benchmark_atan2.cc
 * \brief Benchmark for atan2 implementations
 * \author Carles Fernandez-Prades, 2022. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include <benchmark/benchmark.h>
#include <gnuradio/math.h>
#include <cmath>
#include <random>

void bm_std_atan2(benchmark::State& state)
{
    std::random_device rd;
    std::default_random_engine e2(rd());
    std::uniform_real_distribution<> dist(-1.0, 1.0);

    float a = dist(e2);
    float b = dist(e2);
    float c;
    while (state.KeepRunning())
        {
            c = std::atan2(a, b);
        }
    if (c > 1.0)
        {
            // Avoid unused-but-set-variable warning
        }
}


void bm_gr_fast_atan2f(benchmark::State& state)
{
    std::random_device rd;
    std::default_random_engine e2(rd());
    std::uniform_real_distribution<> dist(-1.0, 1.0);

    float a = dist(e2);
    float b = dist(e2);
    float c;
    while (state.KeepRunning())
        {
            c = gr::fast_atan2f(a, b);
        }
    if (c > 1.0)
        {
            // Avoid unused-but-set-variable warning
        }
}

BENCHMARK(bm_std_atan2);
BENCHMARK(bm_gr_fast_atan2f);

BENCHMARK_MAIN();
