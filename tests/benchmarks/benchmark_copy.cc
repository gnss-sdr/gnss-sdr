/*!
 * \file benchmark_copy.cc
 * \brief Benchmark for memory copy implementations
 * \author Carles Fernandez-Prades, 2020. cfernandez(at)cttc.es
 *
 * Based on https://stackoverflow.com/a/40109182
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

#include <benchmark/benchmark.h>
#include <algorithm>
#include <complex>
#include <cstring>
#include <vector>

constexpr int N = 8184;

void bm_memcpy(benchmark::State& state)
{
    std::vector<std::complex<float>> orig(N);
    std::vector<std::complex<float>> dest(N);

    while (state.KeepRunning())
        {
            memcpy(dest.data(), orig.data(), N * sizeof(std::complex<float>));
        }
}


void bm_stdmemcpy(benchmark::State& state)
{
    std::vector<std::complex<float>> orig(N);
    std::vector<std::complex<float>> dest(N);

    while (state.KeepRunning())
        {
            std::memcpy(dest.data(), orig.data(), N * sizeof(std::complex<float>));
        }
}


void bm_stdcopy(benchmark::State& state)
{
    std::vector<std::complex<float>> orig(N);
    std::vector<std::complex<float>> dest(N);

    while (state.KeepRunning())
        {
            std::copy(orig.begin(), orig.end(), dest.begin());
        }
}


void bm_stdcopy_n(benchmark::State& state)
{
    std::vector<std::complex<float>> orig(N);
    std::vector<std::complex<float>> dest(N);

    while (state.KeepRunning())
        {
            std::copy_n(orig.begin(), N, dest.begin());
        }
}

BENCHMARK(bm_memcpy);
BENCHMARK(bm_stdmemcpy);
BENCHMARK(bm_stdcopy);
BENCHMARK(bm_stdcopy_n);

BENCHMARK_MAIN();
