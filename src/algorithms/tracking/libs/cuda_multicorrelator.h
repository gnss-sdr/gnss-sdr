/*!
 * \file cuda_multicorrelator.h
 * \brief Highly optimized CUDA GPU vector multiTAP correlator class
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          </ul>
 *
 * Class that implements a highly optimized vector multiTAP correlator class for NVIDIA CUDA GPUs
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_CUDA_MULTICORRELATOR_H
#define GNSS_SDR_CUDA_MULTICORRELATOR_H


#ifdef __CUDACC__
#define CUDA_CALLABLE_MEMBER_GLOBAL __global__
#define CUDA_CALLABLE_MEMBER_DEVICE __device__
#else
#define CUDA_CALLABLE_MEMBER_GLOBAL
#define CUDA_CALLABLE_MEMBER_DEVICE
#endif

#include <complex>
#include <cuda.h>
#include <cuda_runtime.h>

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_libs
 * \{ */


// GPU new internal data types for complex numbers

struct GPU_Complex
{
    float r;
    float i;
    CUDA_CALLABLE_MEMBER_DEVICE GPU_Complex(){};
    CUDA_CALLABLE_MEMBER_DEVICE GPU_Complex(float a, float b) : r(a), i(b) {}
    CUDA_CALLABLE_MEMBER_DEVICE float magnitude2(void) { return r * r + i * i; }
    CUDA_CALLABLE_MEMBER_DEVICE GPU_Complex operator*(const GPU_Complex& a)
    {
#ifdef __CUDACC__
        return GPU_Complex(__fmul_rn(r, a.r) - __fmul_rn(i, a.i), __fmul_rn(i, a.r) + __fmul_rn(r, a.i));
#else
        return GPU_Complex(r * a.r - i * a.i, i * a.r + r * a.i);
#endif
    }
    CUDA_CALLABLE_MEMBER_DEVICE GPU_Complex operator+(const GPU_Complex& a)
    {
        return GPU_Complex(r + a.r, i + a.i);
    }
    CUDA_CALLABLE_MEMBER_DEVICE void operator+=(const GPU_Complex& a)
    {
        r += a.r;
        i += a.i;
    }
    CUDA_CALLABLE_MEMBER_DEVICE void multiply_acc(const GPU_Complex& a, const GPU_Complex& b)
    {
        // c=a*b+c
        // real part
        // c.r=(a.r*b.r - a.i*b.i)+c.r
#ifdef __CUDACC__
        r = __fmaf_rn(a.r, b.r, r);
        r = __fmaf_rn(-a.i, b.i, r);
        // imag part
        i = __fmaf_rn(a.i, b.r, i);
        i = __fmaf_rn(a.r, b.i, i);
#else
        r = (a.r * b.r - a.i * b.i) + r;
        i = (a.i * b.r - a.r * b.i) + i;
#endif
    }
};


struct GPU_Complex_Short
{
    float r;
    float i;
    CUDA_CALLABLE_MEMBER_DEVICE GPU_Complex_Short(short int a, short int b) : r(a), i(b) {}
    CUDA_CALLABLE_MEMBER_DEVICE float magnitude2(void)
    {
        return r * r + i * i;
    }
    CUDA_CALLABLE_MEMBER_DEVICE GPU_Complex_Short operator*(const GPU_Complex_Short& a)
    {
        return GPU_Complex_Short(r * a.r - i * a.i, i * a.r + r * a.i);
    }
    CUDA_CALLABLE_MEMBER_DEVICE GPU_Complex_Short operator+(const GPU_Complex_Short& a)
    {
        return GPU_Complex_Short(r + a.r, i + a.i);
    }
};


/*!
 * \brief Class that implements carrier wipe-off and correlators using NVIDIA CUDA GPU accelerators.
 */
class cuda_multicorrelator
{
public:
    cuda_multicorrelator();
    bool init_cuda_integrated_resampler(
        int signal_length_samples,
        int code_length_chips,
        int n_correlators);
    bool set_local_code_and_taps(
        int code_length_chips,
        const std::complex<float>* local_codes_in,
        float* shifts_chips,
        int n_correlators);
    bool set_input_output_vectors(
        std::complex<float>* corr_out,
        std::complex<float>* sig_in);

    bool free_cuda();
    bool Carrier_wipeoff_multicorrelator_resampler_cuda(
        float rem_carrier_phase_in_rad,
        float phase_step_rad,
        float code_phase_step_chips,
        float rem_code_phase_chips,
        int signal_length_samples,
        int n_correlators);

private:
    cudaStream_t stream1;
    // cudaStream_t stream2;

    // Allocate the device input vectors
    GPU_Complex* d_sig_in;
    GPU_Complex* d_nco_in;
    GPU_Complex* d_sig_doppler_wiped;
    GPU_Complex* d_local_codes_in;
    GPU_Complex* d_corr_out;

    std::complex<float>* d_sig_in_cpu;
    std::complex<float>* d_corr_out_cpu;

    float* d_shifts_chips;
    int* d_shifts_samples;
    int d_code_length_chips;

    int selected_gps_device;
    int threadsPerBlock;
    int blocksPerGrid;

    int num_gpu_devices;
    int selected_device;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_CUDA_MULTICORRELATOR_H
