/*!
 * \file benchmark_pcps_grid.cc
 * \brief Benchmark of the PCPS acquisition grid: CPU (volk + gr::fft) baseline
 *        versus the CUDA engine, over a sweep of FFT sizes and Doppler bins.
 * \author Phillip Vu, 2026. phillipvu(at)users.noreply.github.com
 *
 * Run e.g.:
 *   ./benchmark_pcps_grid --benchmark_counters_tabular=true
 *   ./benchmark_pcps_grid --benchmark_filter='cuda' --benchmark_repetitions=5
 *
 * The "grid_cells/s" counter is (fft_size x bins) per second. It is the number of
 * code-phase x Doppler hypotheses are evaluated per second. "dwells/s" is the
 * number of complete search grids per second, which is the figure that decides
 * how many channels can be in acquisition at once in real time.
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

#include "gnss_sdr_fft.h"
#include <benchmark/benchmark.h>
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <memory>
#include <random>
#include <vector>
#if CUDA_GPU_ACCEL
#include "cuda_pcps_engine.h"
#endif

namespace
{
using cvec = volk_gnsssdr::vector<std::complex<float>>;
using fvec = volk_gnsssdr::vector<float>;
constexpr double TWO_PI_D = 6.283185307179586;

// Random complex vectors: the arithmetic cost does not depend on the content.
cvec random_cvec(size_t n, uint32_t seed)
{
    std::mt19937 rng(seed);
    std::normal_distribution<float> dist(0.0F, 1.0F);
    cvec v(n);
    for (auto& x : v)
        {
            x = std::complex<float>(dist(rng), dist(rng));
        }
    return v;
}

std::vector<cvec> make_wipeoffs(uint32_t fft_size, uint32_t bins, double fs)
{
    std::vector<cvec> w(bins, cvec(fft_size));
    for (uint32_t k = 0; k < bins; k++)
        {
            const double doppler = -5000.0 + 10000.0 * static_cast<double>(k) / static_cast<double>(bins);
            const auto step = static_cast<float>(TWO_PI_D * doppler / fs);
            std::array<float, 1> phase{};
            volk_gnsssdr_s32f_sincos_32fc(w[k].data(), -step, phase.data(), fft_size);
        }
    return w;
}

void set_counters(benchmark::State& state, uint32_t fft_size, uint32_t bins)
{
    state.counters["fft_size"] = fft_size;
    state.counters["bins"] = bins;
    state.counters["dwells/s"] = benchmark::Counter(1, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["grid_cells/s"] = benchmark::Counter(static_cast<double>(fft_size) * bins, benchmark::Counter::kIsIterationInvariantRate);
}


// --------------------------------------------------------------------------
// CPU baseline: same operation sequence as pcps_acquisition::doppler_grid_cpu()
// --------------------------------------------------------------------------
void bm_pcps_grid_cpu(benchmark::State& state)
{
    const auto fft_size = static_cast<uint32_t>(state.range(0));
    const auto bins = static_cast<uint32_t>(state.range(1));
    const double fs = 4.0e6;

    auto fft_fwd = gnss_fft_fwd_make_unique(fft_size);
    auto fft_rev = gnss_fft_rev_make_unique(fft_size);
    const cvec in = random_cvec(fft_size, 1U);
    const cvec fft_codes = random_cvec(fft_size, 2U);
    const std::vector<cvec> wipeoffs = make_wipeoffs(fft_size, bins, fs);
    std::vector<fvec> magnitude(bins, fvec(fft_size));

    for (auto _ : state)
        {
            for (uint32_t k = 0; k < bins; k++)
                {
                    volk_32fc_x2_multiply_32fc(fft_fwd->get_inbuf(), in.data(), wipeoffs[k].data(), fft_size);
                    fft_fwd->execute();
                    volk_32fc_x2_multiply_32fc(fft_rev->get_inbuf(), fft_fwd->get_outbuf(), fft_codes.data(), fft_size);
                    fft_rev->execute();
                    volk_32fc_magnitude_squared_32f(magnitude[k].data(), fft_rev->get_outbuf(), fft_size);
                }
            benchmark::DoNotOptimize(magnitude[0].data());
            benchmark::ClobberMemory();
        }
    set_counters(state, fft_size, bins);
}


#if CUDA_GPU_ACCEL
// --------------------------------------------------------------------------
// CUDA engine, including host<->device transfers (what the receiver sees)
// --------------------------------------------------------------------------
void bm_pcps_grid_cuda(benchmark::State& state)
{
    const auto fft_size = static_cast<uint32_t>(state.range(0));
    const auto bins = static_cast<uint32_t>(state.range(1));
    const double fs = 4.0e6;

    CudaPcpsEngine gpu(fft_size, fft_size, bins);
    if (!gpu.is_valid())
        {
            state.SkipWithError(gpu.last_error().c_str());
            return;
        }
    const cvec in = random_cvec(fft_size, 1U);
    const cvec fft_codes = random_cvec(fft_size, 2U);
    const std::vector<cvec> wipeoffs = make_wipeoffs(fft_size, bins, fs);
    std::vector<const std::complex<float>*> wipe_rows(bins);
    for (uint32_t k = 0; k < bins; k++) wipe_rows[k] = wipeoffs[k].data();
    std::vector<fvec> magnitude(bins, fvec(fft_size));
    std::vector<float*> out_rows(bins);
    for (uint32_t k = 0; k < bins; k++) out_rows[k] = magnitude[k].data();

    if (!gpu.set_doppler_wipeoffs(CudaPcpsEngine::MAIN_GRID, wipe_rows.data(), bins) || !gpu.set_fft_codes(fft_codes.data()))
        {
            state.SkipWithError(gpu.last_error().c_str());
            return;
        }
    // Warm-up: cuFFT plan creation and first-launch costs are one-off in the receiver
    if (!gpu.compute_grid(in.data(), CudaPcpsEngine::MAIN_GRID, bins, 0, false, out_rows.data()))
        {
            state.SkipWithError(gpu.last_error().c_str());
            return;
        }

    for (auto _ : state)
        {
            if (!gpu.compute_grid(in.data(), CudaPcpsEngine::MAIN_GRID, bins, 0, false, out_rows.data()))
                {
                    state.SkipWithError(gpu.last_error().c_str());
                    break;
                }
            benchmark::DoNotOptimize(magnitude[0].data());
        }
    set_counters(state, fft_size, bins);
    state.SetLabel(gpu.device_name());
}
#endif
}  // namespace


// fft_size x bins. 1 ms at 2/4/8/16/20 Msps, then 4 ms at 4 Msps (16000) and
// 2 ms at 20 Msps (40000). Bins: +/-5 kHz at 500/250/125 Hz steps.
const std::vector<std::vector<int64_t>> grid_args{
    {2000, 4000, 8000, 16000, 20000, 40000},
    {21, 41, 81}};

BENCHMARK(bm_pcps_grid_cpu)->ArgsProduct(grid_args)->Unit(benchmark::kMicrosecond)->UseRealTime();
#if CUDA_GPU_ACCEL
BENCHMARK(bm_pcps_grid_cuda)->ArgsProduct(grid_args)->Unit(benchmark::kMicrosecond)->UseRealTime();
#endif

BENCHMARK_MAIN();
