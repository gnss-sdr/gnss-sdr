/*!
 * \file cuda_pcps_engine_test.cc
 * \brief Checks that the CUDA PCPS grid matches the CPU (volk + gr::fft) grid.
 * \author Phillip Vu, 2026. phillipvu(at)users.noreply.github.com
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

#include "GPS_L1_CA.h"
#include "MATH_CONSTANTS.h"
#include "cuda_pcps_engine.h"
#include "gnss_sdr_fft.h"
#include "gps_sdr_signal_replica.h"
#include <gtest/gtest.h>
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <iostream>
#include <random>
#include <vector>


namespace
{
using cvec = volk_gnsssdr::vector<std::complex<float>>;
using fvec = volk_gnsssdr::vector<float>;

struct PcpsScenario
{
    int64_t fs_in{4000000};
    uint32_t sampled_ms{1};
    bool bit_transition{false};
    int32_t doppler_max{5000};
    int32_t doppler_step{250};
    uint32_t prn{1};
    uint32_t delay_samples{524};
    float doppler_hz{1680.0F};
    float noise_sigma{0.5F};

    uint32_t consumed_samples() const { return static_cast<uint32_t>(fs_in / 1000) * sampled_ms * (bit_transition ? 2U : 1U); }
    uint32_t fft_size() const { return consumed_samples(); }
    uint32_t effective_fft_size() const { return bit_transition ? fft_size() / 2U : fft_size(); }
    uint32_t num_bins() const { return static_cast<uint32_t>(std::ceil(2.0 * doppler_max / doppler_step)); }
    uint32_t offset() const { return bit_transition ? effective_fft_size() : 0U; }
};


// Reference implementation: identical operation sequence to
// pcps_acquisition::doppler_grid_cpu(), but standalone.
class CpuPcpsReference
{
public:
    explicit CpuPcpsReference(const PcpsScenario& sc)
        : sc_(sc),
          N_(sc.fft_size()),
          E_(sc.effective_fft_size()),
          fft_fwd_(gnss_fft_fwd_make_unique(N_)),
          fft_rev_(gnss_fft_rev_make_unique(N_)),
          fft_codes_(N_),
          tmp_(E_)
    {
        // Doppler wipe-off grid
        wipeoffs_.resize(sc.num_bins(), cvec(N_));
        for (uint32_t k = 0; k < sc.num_bins(); k++)
            {
                const int32_t doppler = -sc.doppler_max + sc.doppler_step * static_cast<int32_t>(k);
                const float phase_step_rad = static_cast<float>(TWO_PI) * static_cast<float>(doppler) / static_cast<float>(sc.fs_in);
                std::array<float, 1> phase{};
                volk_gnsssdr_s32f_sincos_32fc(wipeoffs_[k].data(), -phase_step_rad, phase.data(), N_);
            }
        // conj(FFT(code)), zero padded exactly like pcps_acquisition::set_local_code.
        // The adapter tiles one code period sampled_ms times before handing it over.
        const uint32_t samples_per_code = static_cast<uint32_t>(sc.fs_in / 1000);
        cvec period(samples_per_code);
        gps_l1_ca_code_gen_complex_sampled(own::span<std::complex<float>>(period.data(), period.size()), sc.prn, static_cast<int32_t>(sc.fs_in), 0);
        cvec code(samples_per_code * sc.sampled_ms);
        for (uint32_t r = 0; r < sc.sampled_ms; r++)
            {
                std::copy(period.begin(), period.end(), code.begin() + static_cast<std::ptrdiff_t>(r * samples_per_code));
            }
        if (sc.bit_transition)
            {
                const uint32_t half = N_ / 2;
                std::fill_n(fft_fwd_->get_inbuf(), half, std::complex<float>(0.0F, 0.0F));
                std::copy(code.data(), code.data() + half, fft_fwd_->get_inbuf() + half);
            }
        else
            {
                std::copy(code.data(), code.data() + N_, fft_fwd_->get_inbuf());
            }
        fft_fwd_->execute();
        volk_32fc_conjugate_32fc(fft_codes_.data(), fft_fwd_->get_outbuf(), N_);
        magnitude_.resize(sc.num_bins(), fvec(E_));
    }

    const cvec& fft_codes() const { return fft_codes_; }
    const std::vector<cvec>& wipeoffs() const { return wipeoffs_; }
    std::vector<fvec>& magnitude() { return magnitude_; }

    void doppler_grid(const std::complex<float>* in, bool accumulate)
    {
        for (uint32_t k = 0; k < sc_.num_bins(); k++)
            {
                volk_32fc_x2_multiply_32fc(fft_fwd_->get_inbuf(), in, wipeoffs_[k].data(), N_);
                fft_fwd_->execute();
                volk_32fc_x2_multiply_32fc(fft_rev_->get_inbuf(), fft_fwd_->get_outbuf(), fft_codes_.data(), N_);
                fft_rev_->execute();
                if (!accumulate)
                    {
                        volk_32fc_magnitude_squared_32f(magnitude_[k].data(), fft_rev_->get_outbuf() + sc_.offset(), E_);
                    }
                else
                    {
                        volk_32fc_magnitude_squared_32f(tmp_.data(), fft_rev_->get_outbuf() + sc_.offset(), E_);
                        volk_32f_x2_add_32f(magnitude_[k].data(), magnitude_[k].data(), tmp_.data(), E_);
                    }
            }
    }

private:
    PcpsScenario sc_;
    uint32_t N_;
    uint32_t E_;
    std::unique_ptr<gnss_fft_complex_fwd> fft_fwd_;
    std::unique_ptr<gnss_fft_complex_rev> fft_rev_;
    cvec fft_codes_;
    fvec tmp_;
    std::vector<cvec> wipeoffs_;
    std::vector<fvec> magnitude_;
};


// Synthetic GPS L1 C/A signal: delayed, Doppler-shifted code + AWGN
cvec make_signal(const PcpsScenario& sc, uint32_t seed)
{
    const uint32_t N = sc.fft_size();
    const uint32_t samples_per_code = static_cast<uint32_t>(sc.fs_in / 1000);
    cvec code(samples_per_code);
    gps_l1_ca_code_gen_complex_sampled(own::span<std::complex<float>>(code.data(), code.size()), sc.prn, static_cast<int32_t>(sc.fs_in), 0);

    std::mt19937 rng(seed);
    std::normal_distribution<float> noise(0.0F, sc.noise_sigma);
    cvec sig(N);
    const float phase_step = static_cast<float>(TWO_PI) * sc.doppler_hz / static_cast<float>(sc.fs_in);
    for (uint32_t n = 0; n < N; n++)
        {
            const uint32_t idx = (n + samples_per_code - (sc.delay_samples % samples_per_code)) % samples_per_code;
            const std::complex<float> carrier(std::cos(phase_step * static_cast<float>(n)), std::sin(phase_step * static_cast<float>(n)));
            sig[n] = code[idx] * carrier + std::complex<float>(noise(rng), noise(rng));
        }
    return sig;
}


struct GridPeak
{
    uint32_t bin{0};
    uint32_t index{0};
    float value{0.0F};
};

GridPeak find_peak(const std::vector<fvec>& grid, uint32_t bins, uint32_t E)
{
    GridPeak p;
    for (uint32_t k = 0; k < bins; k++)
        {
            for (uint32_t i = 0; i < E; i++)
                {
                    if (grid[k][i] > p.value)
                        {
                            p.value = grid[k][i];
                            p.bin = k;
                            p.index = i;
                        }
                }
        }
    return p;
}

// Max |gpu - cpu| over the grid, normalized by the grid maximum
double max_relative_error(const std::vector<fvec>& a, const std::vector<fvec>& b, uint32_t bins, uint32_t E)
{
    double max_abs = 0.0;
    double max_val = 0.0;
    for (uint32_t k = 0; k < bins; k++)
        {
            for (uint32_t i = 0; i < E; i++)
                {
                    max_abs = std::max(max_abs, static_cast<double>(std::fabs(a[k][i] - b[k][i])));
                    max_val = std::max(max_val, static_cast<double>(std::fabs(a[k][i])));
                }
        }
    return max_val > 0.0 ? max_abs / max_val : max_abs;
}


void run_parity_case(const PcpsScenario& sc, uint32_t dwells)
{
    const uint32_t N = sc.fft_size();
    const uint32_t E = sc.effective_fft_size();
    const uint32_t bins = sc.num_bins();

    CpuPcpsReference cpu(sc);
    CudaPcpsEngine gpu(N, E, bins);
    ASSERT_TRUE(gpu.is_valid()) << gpu.last_error();

    std::vector<const std::complex<float>*> wipe_rows(bins);
    for (uint32_t k = 0; k < bins; k++)
        {
            wipe_rows[k] = cpu.wipeoffs()[k].data();
        }
    ASSERT_TRUE(gpu.set_doppler_wipeoffs(CudaPcpsEngine::MAIN_GRID, wipe_rows.data(), bins)) << gpu.last_error();
    ASSERT_TRUE(gpu.set_fft_codes(cpu.fft_codes().data())) << gpu.last_error();

    std::vector<fvec> gpu_grid(bins, fvec(E));
    std::vector<float*> out_rows(bins);
    for (uint32_t k = 0; k < bins; k++)
        {
            out_rows[k] = gpu_grid[k].data();
        }

    for (uint32_t d = 0; d < dwells; d++)
        {
            const cvec sig = make_signal(sc, 1234U + d);
            const bool accumulate = (d != 0);
            cpu.doppler_grid(sig.data(), accumulate);
            ASSERT_TRUE(gpu.compute_grid(sig.data(), CudaPcpsEngine::MAIN_GRID, bins, sc.offset(), accumulate, out_rows.data())) << gpu.last_error();
        }

    const double rel_err = max_relative_error(cpu.magnitude(), gpu_grid, bins, E);
    EXPECT_LT(rel_err, 2e-4) << "GPU grid deviates from the CPU grid";

    const GridPeak pc = find_peak(cpu.magnitude(), bins, E);
    const GridPeak pg = find_peak(gpu_grid, bins, E);
    EXPECT_EQ(pc.bin, pg.bin) << "Doppler bin of the peak differs";
    EXPECT_EQ(pc.index, pg.index) << "Code phase of the peak differs";

    // And the peak must sit where the synthetic signal was put
    const int32_t expected_bin = (sc.doppler_max + static_cast<int32_t>(std::lround(sc.doppler_hz))) / sc.doppler_step;
    EXPECT_NEAR(static_cast<double>(pg.bin), static_cast<double>(expected_bin), 1.0);
    const uint32_t samples_per_code = static_cast<uint32_t>(sc.fs_in / 1000);
    EXPECT_EQ(pg.index % samples_per_code, sc.delay_samples % samples_per_code);

    std::cout << "  fft_size=" << N << " bins=" << bins << " dwells=" << dwells
              << " max rel. error=" << rel_err << " peak@(bin " << pg.bin << ", idx " << pg.index << ")\n";
}
}  // namespace


TEST(CudaPcpsEngineTest, MatchesCpuReferenceSingleDwell)
{
    PcpsScenario sc;
    run_parity_case(sc, 1);
}


TEST(CudaPcpsEngineTest, MatchesCpuReferenceNonCoherentAccumulation)
{
    PcpsScenario sc;
    sc.noise_sigma = 2.0F;
    run_parity_case(sc, 4);
}


TEST(CudaPcpsEngineTest, MatchesCpuReferenceBitTransitionMode)
{
    PcpsScenario sc;
    sc.bit_transition = true;
    run_parity_case(sc, 1);
}


TEST(CudaPcpsEngineTest, MatchesCpuReferenceLongCoherentIntegration)
{
    PcpsScenario sc;
    sc.sampled_ms = 4;
    sc.doppler_step = 100;
    run_parity_case(sc, 1);
}


TEST(CudaPcpsEngineTest, ReportsInvalidUsage)
{
    PcpsScenario sc;
    CudaPcpsEngine gpu(sc.fft_size(), sc.effective_fft_size(), sc.num_bins());
    ASSERT_TRUE(gpu.is_valid()) << gpu.last_error();
    cvec sig = make_signal(sc, 7U);
    fvec row(sc.effective_fft_size());
    float* rows[1] = {row.data()};
    // No wipe-offs uploaded yet
    EXPECT_FALSE(gpu.compute_grid(sig.data(), CudaPcpsEngine::MAIN_GRID, 1, 0, false, rows));
    EXPECT_FALSE(gpu.last_error().empty());
}


TEST(CudaPcpsEngineTest, MeasureExecutionTime)
{
    // Quick CPU vs GPU timing on the default scenario. The Google Benchmark
    // target benchmark_pcps_grid gives the full sweep.
    PcpsScenario sc;
    const uint32_t N = sc.fft_size();
    const uint32_t E = sc.effective_fft_size();
    const uint32_t bins = sc.num_bins();
    const int iterations = 200;

    CpuPcpsReference cpu(sc);
    CudaPcpsEngine gpu(N, E, bins);
    ASSERT_TRUE(gpu.is_valid()) << gpu.last_error();
    std::vector<const std::complex<float>*> wipe_rows(bins);
    for (uint32_t k = 0; k < bins; k++) wipe_rows[k] = cpu.wipeoffs()[k].data();
    ASSERT_TRUE(gpu.set_doppler_wipeoffs(CudaPcpsEngine::MAIN_GRID, wipe_rows.data(), bins));
    ASSERT_TRUE(gpu.set_fft_codes(cpu.fft_codes().data()));
    std::vector<fvec> gpu_grid(bins, fvec(E));
    std::vector<float*> out_rows(bins);
    for (uint32_t k = 0; k < bins; k++) out_rows[k] = gpu_grid[k].data();
    const cvec sig = make_signal(sc, 99U);

    // warm-up (cuFFT plan creation, first-launch overhead)
    ASSERT_TRUE(gpu.compute_grid(sig.data(), CudaPcpsEngine::MAIN_GRID, bins, sc.offset(), false, out_rows.data()));
    cpu.doppler_grid(sig.data(), false);

    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < iterations; i++)
        {
            cpu.doppler_grid(sig.data(), false);
        }
    auto t1 = std::chrono::steady_clock::now();
    for (int i = 0; i < iterations; i++)
        {
            gpu.compute_grid(sig.data(), CudaPcpsEngine::MAIN_GRID, bins, sc.offset(), false, out_rows.data());
        }
    auto t2 = std::chrono::steady_clock::now();

    const double cpu_us = std::chrono::duration<double, std::micro>(t1 - t0).count() / iterations;
    const double gpu_us = std::chrono::duration<double, std::micro>(t2 - t1).count() / iterations;
    std::cout << "PCPS grid, fft_size=" << N << ", " << bins << " Doppler bins: CPU " << cpu_us
              << " us/dwell, GPU (" << gpu.device_name() << ") " << gpu_us << " us/dwell, speedup x"
              << cpu_us / gpu_us << "\n";
}
