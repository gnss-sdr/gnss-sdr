/*!
 * \file cuda_pcps_engine.cu
 * \brief GPU (CUDA + cuFFT) implementation of the Parallel Code Phase Search grid
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

#include "cuda_pcps_engine.h"
#include <algorithm>
#include <array>
#include <cstring>
#include <cuda_runtime.h>
#include <cufft.h>
#include <sstream>


// ---------------------------------------------------------------------------
// Kernels
// ---------------------------------------------------------------------------

namespace
{
constexpr unsigned int THREADS_PER_BLOCK = 256U;

__device__ __forceinline__ float2 cmul(float2 a, float2 b)
{
    return make_float2(a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x);
}

// out[k*N + n] = in[n] * wipe[k*N + n]   for k in [0, bins), n in [0, N)
__global__ void k_wipeoff(const float2* __restrict__ in,
    const float2* __restrict__ wipe,
    float2* __restrict__ out,
    unsigned int fft_size,
    unsigned int total)
{
    for (unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x; idx < total; idx += gridDim.x * blockDim.x)
        {
            out[idx] = cmul(in[idx % fft_size], wipe[idx]);
        }
}

// batch[k*N + n] *= codes[n]
__global__ void k_mult_codes(float2* __restrict__ batch,
    const float2* __restrict__ codes,
    unsigned int fft_size,
    unsigned int total)
{
    for (unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x; idx < total; idx += gridDim.x * blockDim.x)
        {
            batch[idx] = cmul(batch[idx], codes[idx % fft_size]);
        }
}

// mag[k*E + i] (+)= |batch[k*N + offset + i]|^2
__global__ void k_magnitude(const float2* __restrict__ batch,
    float* __restrict__ mag,
    unsigned int fft_size,
    unsigned int effective_fft_size,
    unsigned int offset,
    unsigned int total,
    int accumulate)
{
    for (unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x; idx < total; idx += gridDim.x * blockDim.x)
        {
            const unsigned int k = idx / effective_fft_size;
            const unsigned int i = idx - k * effective_fft_size;
            const float2 v = batch[k * fft_size + offset + i];
            const float m = v.x * v.x + v.y * v.y;
            mag[idx] = accumulate ? (mag[idx] + m) : m;
        }
}

inline unsigned int grid_for(unsigned int total, int multiprocessors)
{
    const unsigned int needed = (total + THREADS_PER_BLOCK - 1U) / THREADS_PER_BLOCK;
    // Enough blocks to fill the device several times over, but bounded so
    // that very long FFTs (e.g. 20 ms at 20 Msps) do not launch huge grids.
    const unsigned int cap = static_cast<unsigned int>(std::max(1, multiprocessors)) * 8U;
    return std::max(1U, std::min(needed, cap));
}

const char* cufft_error_string(cufftResult r)
{
    // Only codes present in every cuFFT release since 10.x: newer toolkits
    // (cuFFT >= 12, CUDA 13) removed some of the older enumerators.
    switch (r)
        {
        case CUFFT_SUCCESS:
            return "CUFFT_SUCCESS";
        case CUFFT_INVALID_PLAN:
            return "CUFFT_INVALID_PLAN";
        case CUFFT_ALLOC_FAILED:
            return "CUFFT_ALLOC_FAILED";
        case CUFFT_INVALID_TYPE:
            return "CUFFT_INVALID_TYPE";
        case CUFFT_INVALID_VALUE:
            return "CUFFT_INVALID_VALUE";
        case CUFFT_INTERNAL_ERROR:
            return "CUFFT_INTERNAL_ERROR";
        case CUFFT_EXEC_FAILED:
            return "CUFFT_EXEC_FAILED";
        case CUFFT_SETUP_FAILED:
            return "CUFFT_SETUP_FAILED";
        case CUFFT_INVALID_SIZE:
            return "CUFFT_INVALID_SIZE";
        case CUFFT_UNALIGNED_DATA:
            return "CUFFT_UNALIGNED_DATA";
        case CUFFT_INVALID_DEVICE:
            return "CUFFT_INVALID_DEVICE";
        case CUFFT_NO_WORKSPACE:
            return "CUFFT_NO_WORKSPACE";
        case CUFFT_NOT_IMPLEMENTED:
            return "CUFFT_NOT_IMPLEMENTED";
        case CUFFT_NOT_SUPPORTED:
            return "CUFFT_NOT_SUPPORTED";
        default:
            return "CUFFT_ERROR (see cufft.h for the numeric code)";
        }
}
}  // namespace


// ---------------------------------------------------------------------------
// Private state
// ---------------------------------------------------------------------------

struct CudaPcpsEngine::Impl
{
    uint32_t fft_size{0};
    uint32_t effective_fft_size{0};
    uint32_t max_bins{0};
    int device{0};
    int multiprocessors{1};
    bool valid{false};
    std::string error;
    std::string dev_name;
    size_t dev_bytes{0};

    cudaStream_t stream{nullptr};

    float2* d_in{nullptr};                                                      // fft_size
    float2* d_codes{nullptr};                                                   // fft_size
    std::array<float2*, CudaPcpsEngine::NUM_GRIDS> d_wipe{{nullptr, nullptr}};  // max_bins * fft_size each
    std::array<uint32_t, CudaPcpsEngine::NUM_GRIDS> wipe_bins{{0U, 0U}};        // bins uploaded per grid
    float2* d_batch{nullptr};                                                   // max_bins * fft_size (in-place FFT)
    float* d_mag{nullptr};                                                      // max_bins * effective_fft_size

    float2* h_in{nullptr};  // pinned staging, fft_size
    float* h_mag{nullptr};  // pinned staging, max_bins * effective_fft_size

    // One cuFFT plan per grid; (re)created when the batch size changes.
    std::array<cufftHandle, CudaPcpsEngine::NUM_GRIDS> plan{{0, 0}};
    std::array<uint32_t, CudaPcpsEngine::NUM_GRIDS> plan_batch{{0U, 0U}};

    bool fail(const char* what, cudaError_t e)
    {
        std::ostringstream ss;
        ss << what << ": " << cudaGetErrorName(e) << " (" << cudaGetErrorString(e) << ")";
        error = ss.str();
        return false;
    }

    bool fail(const char* what, cufftResult r)
    {
        std::ostringstream ss;
        ss << what << ": " << cufft_error_string(r) << " [" << static_cast<int>(r) << "]";
        error = ss.str();
        return false;
    }

    template <typename T>
    bool dev_alloc(T** ptr, size_t count, const char* what)
    {
        const size_t bytes = count * sizeof(T);
        const cudaError_t e = cudaMalloc(reinterpret_cast<void**>(ptr), bytes);
        if (e != cudaSuccess)
            {
                return fail(what, e);
            }
        dev_bytes += bytes;
        return true;
    }

    bool ensure_plan(int grid, uint32_t bins)
    {
        if (plan_batch[grid] == bins && plan[grid] != 0)
            {
                return true;
            }
        if (plan[grid] != 0)
            {
                cufftDestroy(plan[grid]);
                plan[grid] = 0;
                plan_batch[grid] = 0U;
            }
        cufftResult r = cufftPlan1d(&plan[grid], static_cast<int>(fft_size), CUFFT_C2C, static_cast<int>(bins));
        if (r != CUFFT_SUCCESS)
            {
                plan[grid] = 0;
                return fail("cufftPlan1d", r);
            }
        r = cufftSetStream(plan[grid], stream);
        if (r != CUFFT_SUCCESS)
            {
                return fail("cufftSetStream", r);
            }
        plan_batch[grid] = bins;
        return true;
    }

    void release()
    {
        cudaSetDevice(device);
        if (stream != nullptr)
            {
                cudaStreamSynchronize(stream);
            }
        for (int g = 0; g < CudaPcpsEngine::NUM_GRIDS; g++)
            {
                if (plan[g] != 0)
                    {
                        cufftDestroy(plan[g]);
                        plan[g] = 0;
                    }
                if (d_wipe[g] != nullptr)
                    {
                        cudaFree(d_wipe[g]);
                        d_wipe[g] = nullptr;
                    }
            }
        if (d_in != nullptr) cudaFree(d_in);
        if (d_codes != nullptr) cudaFree(d_codes);
        if (d_batch != nullptr) cudaFree(d_batch);
        if (d_mag != nullptr) cudaFree(d_mag);
        if (h_in != nullptr) cudaFreeHost(h_in);
        if (h_mag != nullptr) cudaFreeHost(h_mag);
        d_in = d_codes = d_batch = nullptr;
        d_mag = nullptr;
        h_in = nullptr;
        h_mag = nullptr;
        if (stream != nullptr)
            {
                cudaStreamDestroy(stream);
                stream = nullptr;
            }
        valid = false;
    }
};


// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

CudaPcpsEngine::CudaPcpsEngine(uint32_t fft_size, uint32_t effective_fft_size, uint32_t max_doppler_bins, int device)
    : p(new Impl())
{
    p->fft_size = fft_size;
    p->effective_fft_size = effective_fft_size;
    p->max_bins = std::max(1U, max_doppler_bins);

    if (fft_size == 0U || effective_fft_size == 0U || effective_fft_size > fft_size)
        {
            p->error = "invalid FFT geometry";
            return;
        }

    int count = 0;
    cudaError_t e = cudaGetDeviceCount(&count);
    if (e != cudaSuccess || count == 0)
        {
            p->fail("cudaGetDeviceCount", e == cudaSuccess ? cudaErrorNoDevice : e);
            return;
        }
    if (device < 0)
        {
            e = cudaGetDevice(&p->device);
            if (e != cudaSuccess)
                {
                    p->fail("cudaGetDevice", e);
                    return;
                }
        }
    else
        {
            if (device >= count)
                {
                    p->error = "requested CUDA device does not exist";
                    return;
                }
            p->device = device;
        }
    e = cudaSetDevice(p->device);
    if (e != cudaSuccess)
        {
            p->fail("cudaSetDevice", e);
            return;
        }

    cudaDeviceProp prop{};
    if (cudaGetDeviceProperties(&prop, p->device) == cudaSuccess)
        {
            p->dev_name = prop.name;
            p->multiprocessors = prop.multiProcessorCount;
        }

    e = cudaStreamCreateWithFlags(&p->stream, cudaStreamNonBlocking);
    if (e != cudaSuccess)
        {
            p->fail("cudaStreamCreate", e);
            return;
        }

    const size_t n = static_cast<size_t>(p->max_bins) * fft_size;
    const size_t m = static_cast<size_t>(p->max_bins) * effective_fft_size;
    if (!p->dev_alloc(&p->d_in, fft_size, "cudaMalloc(d_in)") ||
        !p->dev_alloc(&p->d_codes, fft_size, "cudaMalloc(d_codes)") ||
        !p->dev_alloc(&p->d_batch, n, "cudaMalloc(d_batch)") ||
        !p->dev_alloc(&p->d_mag, m, "cudaMalloc(d_mag)"))
        {
            p->release();
            return;
        }
    // The wipe-off grids are allocated on first upload so that a block which
    // never runs the two-step search does not pay for the second grid.

    e = cudaHostAlloc(reinterpret_cast<void**>(&p->h_in), fft_size * sizeof(float2), cudaHostAllocDefault);
    if (e != cudaSuccess)
        {
            p->fail("cudaHostAlloc(h_in)", e);
            p->release();
            return;
        }
    e = cudaHostAlloc(reinterpret_cast<void**>(&p->h_mag), m * sizeof(float), cudaHostAllocDefault);
    if (e != cudaSuccess)
        {
            p->fail("cudaHostAlloc(h_mag)", e);
            p->release();
            return;
        }

    e = cudaMemsetAsync(p->d_codes, 0, fft_size * sizeof(float2), p->stream);
    if (e == cudaSuccess) e = cudaMemsetAsync(p->d_mag, 0, m * sizeof(float), p->stream);
    if (e == cudaSuccess) e = cudaStreamSynchronize(p->stream);
    if (e != cudaSuccess)
        {
            p->fail("cudaMemset", e);
            p->release();
            return;
        }

    p->valid = true;
}


CudaPcpsEngine::~CudaPcpsEngine()
{
    if (p)
        {
            p->release();
        }
}


bool CudaPcpsEngine::is_valid() const
{
    return p->valid;
}


const std::string& CudaPcpsEngine::last_error() const
{
    return p->error;
}


const std::string& CudaPcpsEngine::device_name() const
{
    return p->dev_name;
}


size_t CudaPcpsEngine::device_bytes() const
{
    return p->dev_bytes;
}


bool CudaPcpsEngine::set_fft_codes(const std::complex<float>* fft_codes)
{
    if (!p->valid)
        {
            return false;
        }
    cudaSetDevice(p->device);
    // Order uploads with kernels on the nonblocking stream, then wait so the
    // caller may reuse its host buffer immediately.
    cudaError_t e = cudaMemcpyAsync(p->d_codes, fft_codes, p->fft_size * sizeof(float2), cudaMemcpyHostToDevice, p->stream);
    if (e != cudaSuccess)
        {
            return p->fail("cudaMemcpyAsync(codes)", e);
        }
    e = cudaStreamSynchronize(p->stream);
    if (e != cudaSuccess)
        {
            return p->fail("cudaStreamSynchronize(codes)", e);
        }
    return true;
}


bool CudaPcpsEngine::set_doppler_wipeoffs(GridId grid, const std::complex<float>* const* wipeoffs, uint32_t bins)
{
    if (!p->valid || grid < 0 || grid >= NUM_GRIDS)
        {
            return false;
        }
    if (bins == 0U || bins > p->max_bins)
        {
            p->error = "set_doppler_wipeoffs: bins out of range";
            return false;
        }
    cudaSetDevice(p->device);
    if (p->d_wipe[grid] == nullptr)
        {
            if (!p->dev_alloc(&p->d_wipe[grid], static_cast<size_t>(p->max_bins) * p->fft_size, "cudaMalloc(d_wipe)"))
                {
                    return false;
                }
        }
    // Rows live in separate host allocations; copy them one by one. This is
    // only done when the Doppler grid changes (PRN change on FDMA, assisted
    // Doppler center, or the two-step refinement), not per dwell.
    const size_t row_bytes = p->fft_size * sizeof(float2);
    cudaError_t upload_error = cudaSuccess;
    for (uint32_t k = 0; k < bins; k++)
        {
            upload_error = cudaMemcpyAsync(p->d_wipe[grid] + static_cast<size_t>(k) * p->fft_size, wipeoffs[k], row_bytes, cudaMemcpyHostToDevice, p->stream);
            if (upload_error != cudaSuccess)
                {
                    break;
                }
        }
    // Drain queued uploads even if a later row failed, before the caller can
    // reuse its host buffers or plan creation can return an error.
    const cudaError_t sync_error = cudaStreamSynchronize(p->stream);
    if (upload_error != cudaSuccess)
        {
            return p->fail("cudaMemcpyAsync(wipeoffs)", upload_error);
        }
    if (sync_error != cudaSuccess)
        {
            return p->fail("cudaStreamSynchronize(wipeoffs)", sync_error);
        }
    p->wipe_bins[grid] = bins;

    // Build the cuFFT plan and run the whole pipeline once on the current
    // (zero) input. Plan creation and first kernel launches can take tens of
    // milliseconds on Jetson; doing them here instead of on the first real
    // dwell keeps acquisition latency deterministic when the block runs in
    // non-blocking mode and samples flow past while the worker is busy.
    if (!p->ensure_plan(grid, bins))
        {
            return false;
        }
    return run_pipeline(grid, bins, 0U, false);
}


bool CudaPcpsEngine::run_pipeline(int grid, uint32_t bins, uint32_t offset, bool accumulate)
{
    const uint32_t N = p->fft_size;
    const uint32_t E = p->effective_fft_size;
    const unsigned int total_c = bins * N;
    const unsigned int total_m = bins * E;
    cudaStream_t s = p->stream;

    // Carrier wipe-off for every Doppler bin at once
    k_wipeoff<<<grid_for(total_c, p->multiprocessors), THREADS_PER_BLOCK, 0, s>>>(p->d_in, p->d_wipe[grid], p->d_batch, N, total_c);
    cudaError_t e = cudaGetLastError();
    if (e != cudaSuccess) return p->fail("k_wipeoff", e);

    // Batched forward FFT (in place)
    cufftResult r = cufftExecC2C(p->plan[grid], reinterpret_cast<cufftComplex*>(p->d_batch), reinterpret_cast<cufftComplex*>(p->d_batch), CUFFT_FORWARD);
    if (r != CUFFT_SUCCESS) return p->fail("cufftExecC2C(forward)", r);

    // Multiply by conj(FFT(code))
    k_mult_codes<<<grid_for(total_c, p->multiprocessors), THREADS_PER_BLOCK, 0, s>>>(p->d_batch, p->d_codes, N, total_c);
    e = cudaGetLastError();
    if (e != cudaSuccess) return p->fail("k_mult_codes", e);

    // Batched inverse FFT (in place, unnormalized like FFTW/gr::fft)
    r = cufftExecC2C(p->plan[grid], reinterpret_cast<cufftComplex*>(p->d_batch), reinterpret_cast<cufftComplex*>(p->d_batch), CUFFT_INVERSE);
    if (r != CUFFT_SUCCESS) return p->fail("cufftExecC2C(inverse)", r);

    // Squared magnitude (+ non-coherent accumulation) into the device grid
    k_magnitude<<<grid_for(total_m, p->multiprocessors), THREADS_PER_BLOCK, 0, s>>>(p->d_batch, p->d_mag, N, E, offset, total_m, accumulate ? 1 : 0);
    e = cudaGetLastError();
    if (e != cudaSuccess) return p->fail("k_magnitude", e);

    e = cudaStreamSynchronize(s);
    if (e != cudaSuccess) return p->fail("cudaStreamSynchronize", e);
    return true;
}


bool CudaPcpsEngine::compute_grid(const std::complex<float>* in, GridId grid, uint32_t bins, uint32_t offset,
    bool accumulate, float* const* magnitude_out)
{
    if (!p->valid || grid < 0 || grid >= NUM_GRIDS)
        {
            return false;
        }
    if (bins == 0U || bins > p->wipe_bins[grid])
        {
            p->error = "compute_grid: bins exceed the uploaded Doppler grid";
            return false;
        }
    if (offset + p->effective_fft_size > p->fft_size)
        {
            p->error = "compute_grid: offset out of range";
            return false;
        }
    cudaSetDevice(p->device);
    if (!p->ensure_plan(grid, bins))
        {
            return false;
        }

    const uint32_t N = p->fft_size;
    const uint32_t E = p->effective_fft_size;
    const size_t in_bytes = static_cast<size_t>(N) * sizeof(float2);
    const size_t mag_bytes = static_cast<size_t>(bins) * E * sizeof(float);
    cudaStream_t s = p->stream;

    // Input: pageable -> pinned staging -> device
    std::memcpy(p->h_in, in, in_bytes);
    cudaError_t e = cudaMemcpyAsync(p->d_in, p->h_in, in_bytes, cudaMemcpyHostToDevice, s);
    if (e != cudaSuccess) return p->fail("cudaMemcpyAsync(in)", e);

    // Wipe-off, FFT, code multiply, IFFT, magnitude (synchronizes the stream)
    if (!run_pipeline(grid, bins, offset, accumulate))
        {
            return false;
        }

    // Device -> pinned staging -> caller's per-bin rows
    e = cudaMemcpyAsync(p->h_mag, p->d_mag, mag_bytes, cudaMemcpyDeviceToHost, s);
    if (e != cudaSuccess) return p->fail("cudaMemcpyAsync(mag)", e);
    e = cudaStreamSynchronize(s);
    if (e != cudaSuccess) return p->fail("cudaStreamSynchronize", e);

    const size_t row_bytes = static_cast<size_t>(E) * sizeof(float);
    for (uint32_t k = 0; k < bins; k++)
        {
            std::memcpy(magnitude_out[k], p->h_mag + static_cast<size_t>(k) * E, row_bytes);
        }
    return true;
}
