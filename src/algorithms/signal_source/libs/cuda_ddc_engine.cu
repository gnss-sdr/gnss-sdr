/*!
 * \file cuda_ddc_engine.cu
 * \brief CUDA implementation of CudaDdcEngine, see cuda_ddc_engine.h
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

#include "cuda_ddc_engine.h"
#include <algorithm>
#include <cstring>
#include <cuda_runtime.h>

namespace
{
// One thread per output sample; each does its own ntaps-length dot product.
__global__ void fir_ddc_kernel(const float2* __restrict__ taps, int ntaps,
    const float* __restrict__ data, int decimation,
    float2* __restrict__ out, int n_out)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n_out)
        {
            return;
        }
    const float* window = data + static_cast<size_t>(i) * decimation;
    float acc_re = 0.0F, acc_im = 0.0F;
    for (int k = 0; k < ntaps; k++)
        {
            const float2 t = taps[k];
            const float d = window[k];
            acc_re += t.x * d;
            acc_im += t.y * d;
        }
    out[i] = make_float2(acc_re, acc_im);
}
}  // namespace

struct CudaDdcEngine::Impl
{
    bool valid = false;
    std::string error;
    std::string device_name;
    int device = -1;
    cudaStream_t stream = nullptr;

    int ntaps = 0;
    float2* d_taps = nullptr;

    size_t data_capacity = 0;
    float* d_data = nullptr;
    float* h_data = nullptr;  // pinned staging buffer

    size_t out_capacity = 0;
    float2* d_out = nullptr;
    float2* h_out = nullptr;  // pinned staging buffer

    ~Impl()
    {
        if (d_taps) cudaFree(d_taps);
        if (d_data) cudaFree(d_data);
        if (d_out) cudaFree(d_out);
        if (h_data) cudaFreeHost(h_data);
        if (h_out) cudaFreeHost(h_out);
        if (stream) cudaStreamDestroy(stream);
    }

    bool fail(const char* what, cudaError_t err)
    {
        error = std::string(what) + ": " + cudaGetErrorString(err);
        return false;
    }

    bool ensure_data_capacity(size_t n)
    {
        if (n <= data_capacity)
            {
                return true;
            }
        if (d_data) cudaFree(d_data);
        if (h_data) cudaFreeHost(h_data);
        d_data = nullptr;
        h_data = nullptr;
        data_capacity = 0;
        cudaError_t err = cudaMalloc(&d_data, n * sizeof(float));
        if (err != cudaSuccess)
            {
                return fail("cudaMalloc(data)", err);
            }
        err = cudaHostAlloc(&h_data, n * sizeof(float), cudaHostAllocDefault);
        if (err != cudaSuccess)
            {
                return fail("cudaHostAlloc(data)", err);
            }
        data_capacity = n;
        return true;
    }

    bool ensure_out_capacity(size_t n)
    {
        if (n <= out_capacity)
            {
                return true;
            }
        if (d_out) cudaFree(d_out);
        if (h_out) cudaFreeHost(h_out);
        d_out = nullptr;
        h_out = nullptr;
        out_capacity = 0;
        cudaError_t err = cudaMalloc(&d_out, n * sizeof(float2));
        if (err != cudaSuccess)
            {
                return fail("cudaMalloc(out)", err);
            }
        err = cudaHostAlloc(&h_out, n * sizeof(float2), cudaHostAllocDefault);
        if (err != cudaSuccess)
            {
                return fail("cudaHostAlloc(out)", err);
            }
        out_capacity = n;
        return true;
    }
};

CudaDdcEngine::CudaDdcEngine(int device) : d_impl(new Impl)
{
    if (device >= 0)
        {
            cudaError_t err = cudaSetDevice(device);
            if (err != cudaSuccess)
                {
                    d_impl->fail("cudaSetDevice", err);
                    return;
                }
        }
    d_impl->device = device;

    cudaDeviceProp prop{};
    int active_device = 0;
    if (cudaGetDevice(&active_device) == cudaSuccess && cudaGetDeviceProperties(&prop, active_device) == cudaSuccess)
        {
            d_impl->device_name = prop.name;
        }

    cudaError_t err = cudaStreamCreate(&d_impl->stream);
    if (err != cudaSuccess)
        {
            d_impl->fail("cudaStreamCreate", err);
            return;
        }
    d_impl->valid = true;
}

CudaDdcEngine::~CudaDdcEngine()
{
    delete d_impl;
}

bool CudaDdcEngine::is_valid() const
{
    return d_impl->valid;
}

const std::string& CudaDdcEngine::last_error() const
{
    return d_impl->error;
}

const std::string& CudaDdcEngine::device_name() const
{
    return d_impl->device_name;
}

bool CudaDdcEngine::set_taps(const std::complex<float>* taps, int ntaps)
{
    if (!d_impl->valid)
        {
            return false;
        }
    if (d_impl->d_taps == nullptr || ntaps != d_impl->ntaps)
        {
            if (d_impl->d_taps) cudaFree(d_impl->d_taps);
            d_impl->d_taps = nullptr;
            cudaError_t err = cudaMalloc(&d_impl->d_taps, static_cast<size_t>(ntaps) * sizeof(float2));
            if (err != cudaSuccess)
                {
                    return d_impl->fail("cudaMalloc(taps)", err);
                }
            d_impl->ntaps = ntaps;
        }
    cudaError_t err = cudaMemcpyAsync(d_impl->d_taps, taps, static_cast<size_t>(ntaps) * sizeof(float2), cudaMemcpyHostToDevice, d_impl->stream);
    if (err != cudaSuccess)
        {
            return d_impl->fail("cudaMemcpyAsync(taps)", err);
        }
    err = cudaStreamSynchronize(d_impl->stream);
    if (err != cudaSuccess)
        {
            return d_impl->fail("cudaStreamSynchronize(set_taps)", err);
        }
    return true;
}

bool CudaDdcEngine::compute(const float* data, int n_data, int decimation, std::complex<float>* out, int n_out)
{
    if (!d_impl->valid || d_impl->d_taps == nullptr)
        {
            return false;
        }
    if (!d_impl->ensure_data_capacity(static_cast<size_t>(n_data)))
        {
            return false;
        }
    if (!d_impl->ensure_out_capacity(static_cast<size_t>(n_out)))
        {
            return false;
        }

    std::memcpy(d_impl->h_data, data, static_cast<size_t>(n_data) * sizeof(float));

    cudaError_t err = cudaMemcpyAsync(d_impl->d_data, d_impl->h_data, static_cast<size_t>(n_data) * sizeof(float), cudaMemcpyHostToDevice, d_impl->stream);
    if (err != cudaSuccess)
        {
            return d_impl->fail("cudaMemcpyAsync(data)", err);
        }

    const int threads = 256;
    const int blocks = (n_out + threads - 1) / threads;
    fir_ddc_kernel<<<blocks, threads, 0, d_impl->stream>>>(d_impl->d_taps, d_impl->ntaps, d_impl->d_data, decimation, d_impl->d_out, n_out);

    err = cudaMemcpyAsync(d_impl->h_out, d_impl->d_out, static_cast<size_t>(n_out) * sizeof(float2), cudaMemcpyDeviceToHost, d_impl->stream);
    if (err != cudaSuccess)
        {
            return d_impl->fail("cudaMemcpyAsync(out)", err);
        }

    err = cudaStreamSynchronize(d_impl->stream);
    if (err != cudaSuccess)
        {
            return d_impl->fail("cudaStreamSynchronize(compute)", err);
        }

    std::memcpy(out, d_impl->h_out, static_cast<size_t>(n_out) * sizeof(float2));
    return true;
}
