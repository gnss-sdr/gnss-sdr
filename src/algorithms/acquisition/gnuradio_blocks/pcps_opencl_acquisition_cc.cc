/*!
 * \file pcps_opencl_acquisition_cc.cc
 * \brief This class implements a Parallel Code Phase Search Acquisition
 * using OpenCL to offload some functions to the GPU.
 *
 *  Acquisition strategy (Kay Borre book + CFAR threshold).
 *  <ol>
 *  <li> Compute the input signal power estimation
 *  <li> Doppler serial search loop
 *  <li> Perform the FFT-based circular convolution (parallel time search)
 *  <li> Record the maximum peak and the associated synchronization parameters
 *  <li> Compute the test statistics and compare to the threshold
 *  <li> Declare positive or negative acquisition using a message port
 *  </ol>
 *
 * Kay Borre book: K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * "A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach", Birkhauser, 2007. pp 81-84
 *
 * \authors <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          <li> Marc Molina, 2013. marc.molina.pena@gmail.com
 *          </ul>
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

#include "pcps_opencl_acquisition_cc.h"
#include "MATH_CONSTANTS.h"  // TWO_PI
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <algorithm>
#include <array>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>

// Khronos OpenCL C++ bindings: CL/opencl.hpp from the system or, if not
// found, from src/algorithms/libs/opencl/include
#define CL_HPP_ENABLE_EXCEPTIONS
#define CL_HPP_TARGET_OPENCL_VERSION 120
#define CL_HPP_MINIMUM_OPENCL_VERSION 120
#ifndef CL_SILENCE_DEPRECATION
#define CL_SILENCE_DEPRECATION  // OpenCL is deprecated, but still available, in macOS
#endif
#include "opencl/clFFT.h"
#if defined(__APPLE__) || defined(__MACOSX)
#include <OpenCL/opencl.h>
#else
#include <CL/opencl.h>
#endif
// The bindings compare the CL_KHR_COMMAND_BUFFER_*_EXTENSION_VERSION macros of
// the C headers of their own release, which older C headers (e.g., those of
// Ubuntu 22.04 and 24.04) do not define. Command buffers are not used here:
// hiding them lets the bundled bindings build with any C headers
#undef cl_khr_command_buffer
#undef cl_khr_command_buffer_mutable_dispatch
#include <CL/opencl.hpp>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


namespace
{
// The kernels work on a batch of Doppler bins: dimension 1 of the global work
// size is the bin, and each bin has an FFT buffer of stride samples
const char *const math_kernel_source = R"CL(
// Carrier wipe-off of the input, into the first n samples of the FFT buffer of
// each bin (the rest of it stays zero)
__kernel void wipeoff_batch(__global const float2* in, __global const float2* wipeoffs,
    __global float2* out, const uint n, const uint stride)
{
    const uint i = get_global_id(0);
    const size_t bin = get_global_id(1);
    const float2 a = in[i];
    const float2 b = wipeoffs[bin * n + i];
    out[bin * stride + i] = (float2)(a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x);
}

// x = x * codes, in the FFT buffer of each bin
__kernel void code_product_batch(__global float2* x, __global const float2* codes, const uint stride)
{
    const uint k = get_global_id(0);
    const size_t j = get_global_id(1) * stride + k;
    const float2 a = x[j];
    const float2 b = codes[k];
    x[j] = (float2)(a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x);
}

// Maximum of |x|^2 over the first n lags of each bin, its first lag, and the
// sum of |x|^2 over those lags. One work-group per bin, with a power of two of
// work-items
__kernel void max_magnitude_batch(__global const float2* x, const uint n, const uint stride,
    __global float* max_value, __global uint* max_index, __global float* power_sum,
    __local float* lmax, __local uint* lidx, __local float* lsum)
{
    const uint lid = get_local_id(0);
    const uint lsize = get_local_size(0);
    const size_t bin = get_group_id(1);
    __global const float2* corr = x + bin * stride;
    float best = -1.0f;
    uint best_i = 0;
    float sum = 0.0f;
    for (uint i = lid; i < n; i += lsize)
        {
            const float2 v = corr[i];
            const float m = v.x * v.x + v.y * v.y;
            sum += m;
            if (m > best)
                {
                    best = m;
                    best_i = i;
                }
        }
    lmax[lid] = best;
    lidx[lid] = best_i;
    lsum[lid] = sum;
    barrier(CLK_LOCAL_MEM_FENCE);
    for (uint s = lsize / 2; s > 0; s >>= 1)
        {
            if (lid < s)
                {
                    lsum[lid] += lsum[lid + s];
                    const float m = lmax[lid + s];
                    const uint i = lidx[lid + s];
                    if (m > lmax[lid] || (m == lmax[lid] && i < lidx[lid]))
                        {
                            lmax[lid] = m;
                            lidx[lid] = i;
                        }
                }
            barrier(CLK_LOCAL_MEM_FENCE);
        }
    if (lid == 0)
        {
            max_value[bin] = lmax[0];
            max_index[bin] = lidx[0];
            power_sum[bin] = lsum[0];
        }
}
)CL";

// Upper limit of each batch buffer: larger grids are searched in several batches
constexpr size_t max_batch_buffer_bytes = 64 * 1024 * 1024;


uint32_t next_power_of_two(uint32_t n)
{
    uint32_t p = 1;
    while (p < n)
        {
            p <<= 1U;
        }
    return p;
}
}  // namespace


/*
 * The OpenCL objects of the block.
 *
 * The circular correlation over d_fft_size lags is computed with FFTs of
 * d_fft_size_pow2 >= 2 * d_fft_size samples: the input, zero-padded, against
 * the local code laid out as [code, zeros, code]. The Doppler bins are searched
 * in batches of batch_bins, each with a single blocking read of the maximum of
 * every bin. The kernel arguments are set once, except the Doppler wipe-offs of
 * the batch.
 */
struct pcps_opencl_acquisition_cc::OpenClState
{
    struct FftPlanDeleter
    {
        void operator()(void *plan) const
        {
            clFFT_DestroyPlan(plan);
        }
    };

    // Batched FFTs, always of batch_bins: the plan keeps its temporary buffer
    void fft(clFFT_Direction dir, const cl::Buffer &input, const cl::Buffer &output)
    {
        const cl_int err = clFFT_ExecuteInterleaved(queue(), fft_plan.get(), static_cast<cl_int>(batch_bins), dir, input(), output(), 0, nullptr, nullptr);
        if (err != CL_SUCCESS)
            {
                throw cl::Error(err, "clFFT_ExecuteInterleaved");
            }
    }

    cl::Platform platform;
    cl::Device device;
    cl::Context context;
    cl::CommandQueue queue;
    cl::Program program;
    size_t batch_bins{1};                            // Doppler bins searched at once
    size_t reduction_size{1};                        // work-items per bin in the maximum search
    cl::Buffer in;                                   // input dwell, d_fft_size samples
    std::vector<cl::Buffer> wipeoffs;                // per batch: batch_bins x d_fft_size samples
    cl::Buffer fft_in;                               // per bin: wiped-off input, zero-padded to d_fft_size_pow2 samples
    cl::Buffer fft_out;                              // per bin: its FFT, times the code FFT, and its inverse FFT
    cl::Buffer fft_codes;                            // conjugated FFT of the code layout, d_fft_size_pow2 samples
    cl::Buffer max_value;                            // per bin: maximum |correlation|^2
    cl::Buffer max_index;                            // per bin: its lag
    cl::Buffer power_sum;                            // per bin: sum of |correlation|^2
    cl::Kernel wipeoff;                              // fft_in = in * wipeoffs
    cl::Kernel code_product;                         // fft_out = fft_out * fft_codes
    cl::Kernel max_magnitude;                        // max_value, max_index, power_sum of |fft_out|^2
    std::unique_ptr<gnss_fft_complex_fwd> code_fft;  // FFT of the code layout, on the CPU
    std::vector<gr_complex> codes;                   // conjugated FFT of the code layout, d_fft_size_pow2 samples
    std::unique_ptr<void, FftPlanDeleter> fft_plan;
};


pcps_opencl_acquisition_cc_sptr pcps_make_opencl_acquisition_cc(const Acq_Conf &conf, uint32_t max_dwells)
{
    return pcps_opencl_acquisition_cc_sptr(new pcps_opencl_acquisition_cc(conf, max_dwells));
}


pcps_opencl_acquisition_cc::pcps_opencl_acquisition_cc(const Acq_Conf &conf, uint32_t max_dwells)
    : acquisition_impl_interface("pcps_opencl_acquisition_cc",
          gr::io_signature::make(1, 1, static_cast<int>(sizeof(gr_complex) * conf.sampled_ms * conf.samples_per_ms)),
          gr::io_signature::make(0, 1, sizeof(Gnss_Synchro))),
      d_acq_params(conf),
      d_gnss_synchro(nullptr),
      d_sample_counter(0ULL),
      d_mag(0.0),
      d_input_power(0.0),
      d_test_statistics(0.0),
      d_state(0),
      d_max_dwells(max_dwells),
      d_well_count(0),
      d_fft_size(static_cast<uint32_t>(conf.sampled_ms * conf.samples_per_ms)),
      d_fft_size_pow2(next_power_of_two(2 * d_fft_size)),
      d_num_doppler_bins(0),
      d_channel(0),
      d_in_dwell_count(0),
      d_active(false),
      d_core_working(false),
      d_use_opencl(false),
      d_fft_if(gnss_fft_fwd_make_unique(d_fft_size)),
      d_ifft(gnss_fft_rev_make_unique(d_fft_size)),
      d_in_buffer(d_max_dwells, std::vector<gr_complex>(d_fft_size)),
      d_fft_codes(d_fft_size),
      d_magnitude(d_fft_size)
{
    // A search needs its dwells beyond its first sample (GNSS-SDR.deterministic=true)
    d_det.require_span(static_cast<uint64_t>(d_fft_size) * std::max(1U, d_max_dwells));
    // Filled while an acquisition core reads it: never reallocated
    d_sample_counter_buffer.reserve(d_max_dwells);

    this->message_port_register_out(pmt::mp("events"));

    // Count the number of bins
    for (int doppler = -d_acq_params.doppler_max; doppler <= d_acq_params.doppler_max; doppler += d_acq_params.doppler_step)
        {
            d_num_doppler_bins++;
        }

    d_bin_peak = std::vector<float>(d_num_doppler_bins);
    d_bin_index = std::vector<uint32_t>(d_num_doppler_bins);
    d_bin_power = std::vector<float>(d_num_doppler_bins);
    d_doppler_bin_power = std::vector<float>(d_num_doppler_bins);

    // Create the carrier Doppler wipeoff signals
    d_grid_doppler_wipeoffs = std::vector<std::vector<gr_complex>>(d_num_doppler_bins, std::vector<gr_complex>(d_fft_size));
    for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            int doppler = -d_acq_params.doppler_max + d_acq_params.doppler_step * doppler_index;
            float phase_step_rad = static_cast<float>(TWO_PI) * doppler / static_cast<float>(d_acq_params.fs_in);
            std::array<float, 1> _phase{};
            volk_gnsssdr_s32f_sincos_32fc(d_grid_doppler_wipeoffs[doppler_index].data(), -phase_step_rad, _phase.data(), d_fft_size);
        }

    init_opencl();
}


pcps_opencl_acquisition_cc::~pcps_opencl_acquisition_cc()
{
    try
        {
            {
                std::lock_guard<std::mutex> lock(d_state_mutex);
                d_state = 0;
                d_active = false;
            }
            join_core();
        }
    catch (const std::exception &e)
        {
            std::cerr << "Error while waiting for the OpenCL acquisition core: " << e.what() << '\n';
        }

    try
        {
            if (d_acq_params.dump)
                {
                    d_dump_file.close();
                }
        }
    catch (const std::ofstream::failure &e)
        {
            std::cerr << "Problem closing Acquisition dump file: " << d_acq_params.dump_filename << '\n';
        }
    catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
}


void pcps_opencl_acquisition_cc::init_opencl()
{
    try
        {
            auto cl_state = std::unique_ptr<OpenClState>(new OpenClState());

            // The first GPU device of any platform: the FFT runs on GPU devices only
            std::vector<cl::Platform> platforms;
            cl::Platform::get(&platforms);
            for (const auto &platform : platforms)
                {
                    std::vector<cl::Device> devices;
                    try
                        {
                            platform.getDevices(CL_DEVICE_TYPE_GPU, &devices);
                        }
                    catch (const cl::Error &e)
                        {
                            LOG(WARNING) << "Cannot list the OpenCL devices of platform " << platform.getInfo<CL_PLATFORM_NAME>()
                                         << ": " << e.what() << " (" << e.err() << ")";
                        }
                    if (!devices.empty())
                        {
                            cl_state->platform = platform;
                            cl_state->device = devices[0];
                            break;
                        }
                }
            if (cl_state->device() == nullptr)
                {
                    LOG(WARNING) << "No OpenCL GPU device found. The OpenCL acquisition runs on the CPU";
                    return;
                }

            cl_state->context = cl::Context(cl_state->device);
            cl_state->queue = cl::CommandQueue(cl_state->context, cl_state->device);
            cl_state->program = cl::Program(cl_state->context, std::string(math_kernel_source));
            cl_state->program.build(std::vector<cl::Device>{cl_state->device});

            const size_t n = d_fft_size;
            const size_t n2 = d_fft_size_pow2;
            const size_t bins = d_num_doppler_bins;
            const size_t bin_bytes = sizeof(gr_complex) * n2;
            const auto max_alloc = static_cast<size_t>(cl_state->device.getInfo<CL_DEVICE_MAX_MEM_ALLOC_SIZE>());
            cl_state->batch_bins = std::max<size_t>(1, std::min<size_t>(bins, std::min(max_alloc, max_batch_buffer_bytes) / bin_bytes));
            const size_t batch = cl_state->batch_bins;
            const size_t num_batches = (bins + batch - 1) / batch;

            cl_state->in = cl::Buffer(cl_state->context, CL_MEM_READ_ONLY, sizeof(gr_complex) * n);
            std::vector<gr_complex> batch_wipeoffs(batch * n);
            for (size_t b = 0; b < num_batches; b++)
                {
                    const size_t first = b * batch;
                    const size_t count = std::min(batch, bins - first);
                    for (size_t k = 0; k < count; k++)
                        {
                            std::copy(d_grid_doppler_wipeoffs[first + k].begin(), d_grid_doppler_wipeoffs[first + k].end(), batch_wipeoffs.begin() + k * n);
                        }
                    std::fill(batch_wipeoffs.begin() + count * n, batch_wipeoffs.end(), gr_complex(0.0, 0.0));  // unused bins of the last batch
                    cl_state->wipeoffs.emplace_back(cl_state->context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(gr_complex) * batch * n, batch_wipeoffs.data());
                }
            {
                // The zero padding is never written again. Not with
                // enqueueFillBuffer(): it leaves old contents in macOS
                const std::vector<gr_complex> zeros(n2 * batch, gr_complex(0.0, 0.0));
                cl_state->fft_in = cl::Buffer(cl_state->context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, bin_bytes * batch, const_cast<gr_complex *>(zeros.data()));
            }
            cl_state->fft_out = cl::Buffer(cl_state->context, CL_MEM_READ_WRITE, bin_bytes * batch);
            cl_state->fft_codes = cl::Buffer(cl_state->context, CL_MEM_READ_ONLY, bin_bytes);
            cl_state->max_value = cl::Buffer(cl_state->context, CL_MEM_WRITE_ONLY, sizeof(cl_float) * batch);
            cl_state->max_index = cl::Buffer(cl_state->context, CL_MEM_WRITE_ONLY, sizeof(cl_uint) * batch);
            cl_state->power_sum = cl::Buffer(cl_state->context, CL_MEM_WRITE_ONLY, sizeof(cl_float) * batch);

            const auto n_arg = static_cast<cl_uint>(n);
            const auto stride_arg = static_cast<cl_uint>(n2);
            cl_state->wipeoff = cl::Kernel(cl_state->program, "wipeoff_batch");
            cl_state->wipeoff.setArg(0, cl_state->in);
            cl_state->wipeoff.setArg(2, cl_state->fft_in);  // argument 1: the wipe-offs of the batch
            cl_state->wipeoff.setArg(3, n_arg);
            cl_state->wipeoff.setArg(4, stride_arg);
            cl_state->code_product = cl::Kernel(cl_state->program, "code_product_batch");
            cl_state->code_product.setArg(0, cl_state->fft_out);
            cl_state->code_product.setArg(1, cl_state->fft_codes);
            cl_state->code_product.setArg(2, stride_arg);
            cl_state->max_magnitude = cl::Kernel(cl_state->program, "max_magnitude_batch");
            const size_t max_work_items = std::min({static_cast<size_t>(256),
                cl_state->max_magnitude.getWorkGroupInfo<CL_KERNEL_WORK_GROUP_SIZE>(cl_state->device),
                cl_state->device.getInfo<CL_DEVICE_MAX_WORK_ITEM_SIZES>()[0]});
            while (cl_state->reduction_size * 2 <= max_work_items)
                {
                    cl_state->reduction_size *= 2;
                }
            cl_state->max_magnitude.setArg(0, cl_state->fft_out);
            cl_state->max_magnitude.setArg(1, n_arg);
            cl_state->max_magnitude.setArg(2, stride_arg);
            cl_state->max_magnitude.setArg(3, cl_state->max_value);
            cl_state->max_magnitude.setArg(4, cl_state->max_index);
            cl_state->max_magnitude.setArg(5, cl_state->power_sum);
            cl_state->max_magnitude.setArg(6, cl::Local(sizeof(cl_float) * cl_state->reduction_size));
            cl_state->max_magnitude.setArg(7, cl::Local(sizeof(cl_uint) * cl_state->reduction_size));
            cl_state->max_magnitude.setArg(8, cl::Local(sizeof(cl_float) * cl_state->reduction_size));

            cl_state->code_fft = gnss_fft_fwd_make_unique(d_fft_size_pow2);
            cl_state->codes = std::vector<gr_complex>(n2);

            cl_int err = CL_SUCCESS;
            const clFFT_Dim3 dim = {d_fft_size_pow2, 1, 1};
            cl_state->fft_plan.reset(clFFT_CreatePlan(cl_state->context(), dim, clFFT_1D, clFFT_InterleavedComplexFormat, &err));
            if (err != CL_SUCCESS || !cl_state->fft_plan)
                {
                    throw cl::Error(err, "clFFT_CreatePlan");
                }

            LOG(INFO) << "OpenCL acquisition on device " << cl_state->device.getInfo<CL_DEVICE_NAME>()
                      << " (" << cl_state->device.getInfo<CL_DEVICE_VERSION>() << ") of platform "
                      << cl_state->platform.getInfo<CL_PLATFORM_NAME>() << ", " << bins << " Doppler bins in "
                      << num_batches << " batch(es) of " << batch;
            d_cl = std::move(cl_state);
            d_use_opencl = true;
        }
    catch (const cl::BuildError &e)
        {
            for (const auto &log : e.getBuildLog())
                {
                    LOG(WARNING) << "OpenCL build log: " << log.second;
                }
            LOG(WARNING) << "Cannot build the OpenCL kernels (" << e.err() << "). The OpenCL acquisition runs on the CPU";
        }
    catch (const cl::Error &e)
        {
            LOG(WARNING) << "OpenCL error in " << e.what() << " (" << e.err() << "). The OpenCL acquisition runs on the CPU";
        }
}


void pcps_opencl_acquisition_cc::set_local_code(std::complex<float> *code)
{
    // Not while an acquisition core uses the FFT engines or the code
    std::lock_guard<std::mutex> core_lock(d_core_mutex);

    // Conjugated FFT of the local code, for the CPU implementation and its fallback
    std::copy(code, code + d_fft_size, d_fft_if->get_inbuf());
    d_fft_if->execute();
    volk_32fc_conjugate_32fc(d_fft_codes.data(), d_fft_if->get_outbuf(), d_fft_size);

    if (d_use_opencl)
        {
            try
                {
                    // [code, zeros, code], transformed on the CPU: the FFT plan of the
                    // device runs batches of Doppler bins
                    gr_complex *layout = d_cl->code_fft->get_inbuf();
                    std::fill(layout, layout + d_fft_size_pow2, gr_complex(0.0, 0.0));
                    std::copy(code, code + d_fft_size, layout);
                    std::copy(code, code + d_fft_size, layout + (d_fft_size_pow2 - d_fft_size));
                    d_cl->code_fft->execute();
                    volk_32fc_conjugate_32fc(d_cl->codes.data(), d_cl->code_fft->get_outbuf(), d_fft_size_pow2);
                    d_cl->queue.enqueueWriteBuffer(d_cl->fft_codes, CL_TRUE, 0, sizeof(gr_complex) * d_fft_size_pow2, d_cl->codes.data());
                }
            catch (const cl::Error &e)
                {
                    LOG(ERROR) << "OpenCL error in " << e.what() << " (" << e.err() << ") in channel " << d_channel
                               << ". The OpenCL acquisition continues on the CPU";
                    d_use_opencl = false;
                }
        }
}


void pcps_opencl_acquisition_cc::search_opencl(const gr_complex *in)
{
    OpenClState &cl_state = *d_cl;
    const size_t batch = cl_state.batch_bins;
    // Not modified until the blocking read below
    cl_state.queue.enqueueWriteBuffer(cl_state.in, CL_FALSE, 0, sizeof(gr_complex) * d_fft_size, in);
    for (size_t b = 0; b < cl_state.wipeoffs.size(); b++)
        {
            const size_t first = b * batch;
            const size_t count = std::min<size_t>(batch, d_num_doppler_bins - first);
            cl_state.wipeoff.setArg(1, cl_state.wipeoffs[b]);
            cl_state.queue.enqueueNDRangeKernel(cl_state.wipeoff, cl::NullRange, cl::NDRange(d_fft_size, batch), cl::NullRange);
            cl_state.fft(clFFT_Forward, cl_state.fft_in, cl_state.fft_out);
            cl_state.queue.enqueueNDRangeKernel(cl_state.code_product, cl::NullRange, cl::NDRange(d_fft_size_pow2, batch), cl::NullRange);
            cl_state.fft(clFFT_Inverse, cl_state.fft_out, cl_state.fft_out);
            cl_state.queue.enqueueNDRangeKernel(cl_state.max_magnitude, cl::NullRange,
                cl::NDRange(cl_state.reduction_size, batch), cl::NDRange(cl_state.reduction_size, 1));

            // The blocking read waits for all the commands above
            cl_state.queue.enqueueReadBuffer(cl_state.max_value, CL_FALSE, 0, sizeof(float) * count, &d_bin_peak[first]);
            cl_state.queue.enqueueReadBuffer(cl_state.power_sum, CL_FALSE, 0, sizeof(float) * count, &d_bin_power[first]);
            cl_state.queue.enqueueReadBuffer(cl_state.max_index, CL_TRUE, 0, sizeof(uint32_t) * count, &d_bin_index[first]);

            if (d_acq_params.dump)
                {
                    // In the output buffer of the CPU inverse FFT, and with its scale
                    gr_complex *corr = d_ifft->get_outbuf();
                    const float scale = static_cast<float>(d_fft_size) / static_cast<float>(d_fft_size_pow2);
                    for (size_t k = 0; k < count; k++)
                        {
                            cl_state.queue.enqueueReadBuffer(cl_state.fft_out, CL_TRUE, sizeof(gr_complex) * k * d_fft_size_pow2, sizeof(gr_complex) * d_fft_size, corr);
                            std::transform(corr, corr + d_fft_size, corr, [scale](const gr_complex &v) { return v * scale; });
                            dump_correlation(-d_acq_params.doppler_max + d_acq_params.doppler_step * static_cast<int>(first + k));
                        }
                }
        }
}


void pcps_opencl_acquisition_cc::search_volk(const gr_complex *in)
{
    for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            correlate_volk(in, doppler_index);
            volk_gnsssdr_32f_index_max_32u(&d_bin_index[doppler_index], d_magnitude.data(), d_fft_size);
            d_bin_peak[doppler_index] = d_magnitude[d_bin_index[doppler_index]];
            volk_32f_accumulator_s32f(&d_bin_power[doppler_index], d_magnitude.data(), d_fft_size);
            if (d_acq_params.dump)
                {
                    dump_correlation(-d_acq_params.doppler_max + d_acq_params.doppler_step * static_cast<int>(doppler_index));
                }
        }
}


void pcps_opencl_acquisition_cc::correlate_volk(const gr_complex *in, uint32_t doppler_index)
{
    volk_32fc_x2_multiply_32fc(d_fft_if->get_inbuf(), in,
        d_grid_doppler_wipeoffs[doppler_index].data(), d_fft_size);

    // Compute the FFT of the carrier wiped--off incoming signal
    d_fft_if->execute();

    // Multiply carrier wiped--off, Fourier transformed incoming signal
    // with the local FFT'd code reference using SIMD operations with VOLK library
    volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(),
        d_fft_if->get_outbuf(), d_fft_codes.data(), d_fft_size);

    // compute the inverse FFT
    d_ifft->execute();

    volk_32fc_magnitude_squared_32f(d_magnitude.data(), d_ifft->get_outbuf(), d_fft_size);
}


void pcps_opencl_acquisition_cc::dump_correlation(int doppler)
{
    std::stringstream filename;
    std::streamsize n = 2 * sizeof(float) * (d_fft_size);  // complex file write
    filename << "./test_statistics_" << d_gnss_synchro->System
             << "_" << d_gnss_synchro->Signal[0] << d_gnss_synchro->Signal[1] << "_sat_"
             << d_gnss_synchro->PRN << "_doppler_" << doppler << ".dat";
    d_dump_file.open(filename.str().c_str(), std::ios::out | std::ios::binary);
    d_dump_file.write(reinterpret_cast<char *>(d_ifft->get_outbuf()), n);  // write directly |abs(x)|^2 in this Doppler bin?
    d_dump_file.close();
}


void pcps_opencl_acquisition_cc::acquisition_core()
{
    // Not while set_local_code() changes the code or uses the FFT engines
    std::lock_guard<std::mutex> core_lock(d_core_mutex);

    // initialize acquisition algorithm
    uint64_t samplestamp = d_sample_counter_buffer[d_well_count];
    const gr_complex *in = d_in_buffer[d_well_count].data();

    d_input_power = 0.0;
    d_mag = 0.0;

    d_well_count++;

    DLOG(INFO) << "Channel: " << d_channel
               << " , doing acquisition of satellite: " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
               << " ,sample stamp: " << d_sample_counter
               << ", threshold: " << d_acq_params.threshold
               << ", doppler_max: " << d_acq_params.doppler_max
               << ", doppler_step: " << d_acq_params.doppler_step;

    // 1- Compute the input signal power estimation
    volk_32fc_magnitude_squared_32f(d_magnitude.data(), in, d_fft_size);
    volk_32f_accumulator_s32f(&d_input_power, d_magnitude.data(), d_fft_size);
    d_input_power /= static_cast<float>(d_fft_size);

    // 2- Doppler frequency search: in each bin, the maximum of the FFT-based
    // circular correlation (parallel code phase search). The inverse FFTs are
    // not normalized: they scale it by their size
    bool searched = false;
    auto ifft_size = static_cast<float>(d_fft_size);
    if (d_use_opencl)
        {
            try
                {
                    search_opencl(in);
                    ifft_size = static_cast<float>(d_fft_size_pow2);
                    searched = true;
                }
            catch (const cl::Error &e)
                {
                    LOG(ERROR) << "OpenCL error in " << e.what() << " (" << e.err() << ") in channel " << d_channel
                               << ". The OpenCL acquisition continues on the CPU";
                    d_use_opencl = false;
                }
        }
    if (!searched)
        {
            search_volk(in);
        }

    // 3- Normalize the maximum values and the mean power of the bins to correct
    // the scale factor introduced by the inverse FFT, to |correlation|^2 /
    // d_fft_size^2, and record the maximum peak of the dwell
    const float fft_normalization_factor = ifft_size * static_cast<float>(d_fft_size);
    const float scale = 1.0F / (fft_normalization_factor * fft_normalization_factor);
    for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            const float magt = d_bin_peak[doppler_index] * scale;
            d_doppler_bin_power[doppler_index] = d_bin_power[doppler_index] * scale / static_cast<float>(d_fft_size);
            if (doppler_index == 0 || d_mag < magt)
                {
                    d_mag = magt;
                    d_peak_code_phase = d_bin_index[doppler_index];
                    d_peak_doppler_index = doppler_index;
                }
        }

    // 4- Compute the test statistics and compare to the threshold
    compute_test_statistics(samplestamp);
    conclude_dwell(samplestamp);
}


uint32_t pcps_opencl_acquisition_cc::reference_doppler_index(uint32_t peak_doppler_index) const
{
    // As in pcps_acquisition: the bin opposite to the peak in the wrapped
    // grid, if it is beyond the reference_bin_min_sidelobes Doppler sidelobes
    // of the peak, and otherwise the edge of the grid farthest from it
    const uint32_t separation_bins = d_num_doppler_bins / 2U;
    const float separation_hz = static_cast<float>(separation_bins) * static_cast<float>(d_acq_params.doppler_step);
    const float min_separation_hz = (static_cast<float>(d_acq_params.reference_bin_min_sidelobes) + 0.5F) / (static_cast<float>(d_acq_params.sampled_ms) / 1000.0F);
    if (separation_hz >= min_separation_hz)
        {
            return (peak_doppler_index + separation_bins) % d_num_doppler_bins;
        }
    const int peak_doppler = -d_acq_params.doppler_max + d_acq_params.doppler_step * static_cast<int>(peak_doppler_index);
    return peak_doppler >= 0 ? 0U : d_num_doppler_bins - 1U;
}


void pcps_opencl_acquisition_cc::compute_test_statistics(uint64_t samplestamp)
{
    if (d_acq_params.use_CFAR_algorithm_flag)
        {
            // CFAR, as in pcps_acquisition and for its threshold: the peak
            // against half the mean |correlation|^2 of a Doppler bin clear of
            // the peak and its sidelobes
            d_input_power = d_doppler_bin_power[reference_doppler_index(d_peak_doppler_index)] / 2.0F;
        }
    // else, the peak against the input signal power, for a fixed threshold

    const float test_statistics = d_input_power > 0.0F ? d_mag / d_input_power : 0.0F;

    // In case that d_bit_transition_flag = true, we compare the new test
    // statistics with the value in d_test_statistics. When the second dwell
    // is being processed, its test statistics could be lower than
    // d_test_statistics (i.e, the maximum test statistics in the previous
    // dwell is greater). Note that d_test_statistics is not restarted between
    // consecutive dwells in multidwell operation.
    if (d_test_statistics < test_statistics || !d_acq_params.bit_transition_flag)
        {
            const int doppler = -d_acq_params.doppler_max + d_acq_params.doppler_step * static_cast<int>(d_peak_doppler_index);
            d_gnss_synchro->Acq_delay_samples = static_cast<double>(d_peak_code_phase % static_cast<int32_t>(d_acq_params.samples_per_code));
            d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
            d_gnss_synchro->Acq_samplestamp_samples = samplestamp;
            d_gnss_synchro->Acq_doppler_step = d_acq_params.doppler_step;
            d_test_statistics = test_statistics;
        }
}


void pcps_opencl_acquisition_cc::conclude_dwell(uint64_t samplestamp)
{
    int result = 1;  // keep searching
    if (!d_acq_params.bit_transition_flag)
        {
            if (d_test_statistics > d_acq_params.threshold)
                {
                    result = 2;  // Positive acquisition
                }
            else if (d_well_count == d_max_dwells)
                {
                    result = 3;  // Negative acquisition
                }
        }
    else
        {
            if (d_well_count == d_max_dwells)  // d_max_dwells = 2
                {
                    result = d_test_statistics > d_acq_params.threshold ? 2 : 3;  // Positive or negative acquisition
                }
        }

    bool concluded = false;
    if (result != 1)
        {
            std::lock_guard<std::mutex> lock(d_state_mutex);
            if (d_state == 1)  // not stopped meanwhile
                {
                    if (d_det.enabled())
                        {
                            // GNSS-SDR.deterministic=true: stamped with the end of the dwell that decided it
                            d_det.raise(result == 2 ? DeterministicClock::ACQ_POSITIVE : DeterministicClock::ACQ_NEGATIVE, samplestamp);
                        }
                    d_state = result;
                    d_active = false;
                    concluded = true;
                }
        }
    if (concluded && d_det.enabled())
        {
            // Released here: the stream may be held back until the result is
            // out, so general_work() may not run again before
            d_det.dispatch(d_channel, [this](int event) { this->message_port_pub(pmt::mp("events"), pmt::from_long(event)); });
            std::lock_guard<std::mutex> lock(d_state_mutex);
            d_det.publish(d_sample_counter, d_active, d_channel);
        }
}


void pcps_opencl_acquisition_cc::run_acquisition_cores()
{
    // Processes the buffered dwells in order, until a decision or until it
    // runs out of them (then general_work() starts it again for the next one)
    do
        {
            acquisition_core();
        }
    while (next_dwell_ready());
}


bool pcps_opencl_acquisition_cc::next_dwell_ready()
{
    std::lock_guard<std::mutex> lock(d_state_mutex);
    if (d_state == 1 && d_well_count < d_in_dwell_count)
        {
            return true;
        }
    d_core_working = false;  // the last access of the core thread to the block
    return false;
}


void pcps_opencl_acquisition_cc::join_core()
{
    if (d_core_thread.joinable())
        {
            d_core_thread.join();
        }
}


bool pcps_opencl_acquisition_cc::stop()
{
    try
        {
            {
                std::lock_guard<std::mutex> lock(d_state_mutex);
                d_state = 0;
                d_active = false;
            }
            join_core();
        }
    catch (const std::exception &e)
        {
            LOG(WARNING) << "Exception while waiting for the OpenCL acquisition core to terminate: " << e.what();
        }
    return acquisition_impl_interface::stop();
}


int pcps_opencl_acquisition_cc::general_work(int noutput_items,
    gr_vector_int &ninput_items, gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    const int produced = process_input(noutput_items, ninput_items, input_items, output_items);
    if (d_det.enabled())
        {
            // GNSS-SDR.deterministic=true: results reach the control queue
            // before the progress passes their stamp
            d_det.dispatch(d_channel, [this](int event) { this->message_port_pub(pmt::mp("events"), pmt::from_long(event)); });
            std::lock_guard<std::mutex> lock(d_state_mutex);
            d_det.publish(d_sample_counter, d_active, d_channel);
        }
    return produced;
}


int pcps_opencl_acquisition_cc::process_input(int noutput_items,
    gr_vector_int &ninput_items, gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    int acquisition_message = -1;  // 0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL

    if (d_det.enabled() && d_state == 1 && d_in_dwell_count == 0)
        {
            // GNSS-SDR.deterministic=true: the first dwell starts at its stamped sample
            int n_items = 0;
            if (!d_det.ready_to_start(d_sample_counter, 0, d_fft_size, ninput_items[0], n_items, d_channel))
                {
                    d_sample_counter += static_cast<uint64_t>(d_fft_size) * static_cast<uint64_t>(n_items);
                    consume_each(n_items);
                    return 0;
                }
        }

    // GNSS-SDR.deterministic=true: a requested search keeps the input for its
    // first dwell, which starts at its stamped sample
    bool keep_input = false;
    switch (d_state.load())  // C++11 cannot switch on std::atomic
        {
        case 0:
            {
                // A new search waits for the core of the previous one
                if (d_active && !d_core_working)
                    {
                        // restart acquisition variables
                        d_gnss_synchro->Acq_delay_samples = 0.0;
                        d_gnss_synchro->Acq_doppler_hz = 0.0;
                        d_gnss_synchro->Acq_samplestamp_samples = 0ULL;
                        d_gnss_synchro->Acq_doppler_step = 0U;
                        d_well_count = 0;
                        d_mag = 0.0;
                        d_input_power = 0.0;
                        d_test_statistics = 0.0;
                        d_in_dwell_count = 0;
                        d_sample_counter_buffer.clear();

                        d_state = 1;
                    }
                keep_input = d_det.enabled() && d_active;
                if (!keep_input)
                    {
                        d_sample_counter += static_cast<uint64_t>(d_fft_size) * ninput_items[0];  // sample counter
                    }

                break;
            }

        case 1:
            {
                if (d_in_dwell_count < d_max_dwells)
                    {
                        // Fill internal buffer with d_max_dwells signal blocks. This step ensures that
                        // consecutive signal blocks will be processed in multi-dwell operation. This is
                        // essential when d_bit_transition_flag = true.
                        uint32_t num_dwells = std::min(static_cast<int>(d_max_dwells - d_in_dwell_count), ninput_items[0]);
                        for (uint32_t i = 0; i < num_dwells; i++)
                            {
                                const auto *in = reinterpret_cast<const gr_complex *>(input_items[0]) + static_cast<size_t>(i) * d_fft_size;
                                std::copy(in, in + d_fft_size, d_in_buffer[d_in_dwell_count].data());
                                d_sample_counter += static_cast<uint64_t>(d_fft_size);
                                d_sample_counter_buffer.push_back(d_sample_counter);
                                d_in_dwell_count++;  // complete: an acquisition core may process it
                            }

                        if (ninput_items[0] > static_cast<int>(num_dwells))
                            {
                                d_sample_counter += static_cast<uint64_t>(d_fft_size) * (ninput_items[0] - num_dwells);
                            }
                    }
                else
                    {
                        // We already have d_max_dwells consecutive blocks in the internal buffer,
                        // just skip input blocks.
                        d_sample_counter += static_cast<uint64_t>(d_fft_size) * ninput_items[0];
                    }

                // We create a new thread to process next block if the following
                // conditions are fulfilled:
                //   1. There are new blocks in d_in_buffer that have not been processed yet
                //      (d_well_count < d_in_dwell_count).
                //   2. No other acquisition_core thead is working (!d_core_working).
                //   3. d_state==1. We need to check again d_state because it can be modified at any
                //      moment by the external thread (may have changed since checked in the switch()).
                //      If the external thread has already declared positive (d_state=2) or negative
                //      (d_state=3) acquisition, we don't have to process next block!!
                {
                    // A running core checks for the next dwell under the same lock
                    std::lock_guard<std::mutex> lock(d_state_mutex);
                    if (!d_core_working && (d_well_count < d_in_dwell_count) && d_state == 1)
                        {
                            d_core_working = true;
                            // The previous core thread, if any, has already
                            // left the block (see next_dwell_ready())
                            join_core();
                            d_core_thread = std::thread(&pcps_opencl_acquisition_cc::run_acquisition_cores, this);
                        }
                }

                break;
            }

        case 2:
            {
                // Declare positive acquisition using a message port
                DLOG(INFO) << "positive acquisition";
                DLOG(INFO) << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
                DLOG(INFO) << "sample_stamp " << d_sample_counter;
                DLOG(INFO) << "test statistics value " << d_test_statistics;
                DLOG(INFO) << "test statistics threshold " << d_acq_params.threshold;
                DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
                DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
                DLOG(INFO) << "magnitude " << d_mag;
                DLOG(INFO) << "input signal power " << d_input_power;

                d_state = 0;  // the acquisition core has set d_active = false

                // A new search may already be requested (e.g., the channel repeats a failed one)
                keep_input = d_det.enabled() && d_active;
                if (!keep_input)
                    {
                        d_sample_counter += static_cast<uint64_t>(d_fft_size) * ninput_items[0];  // sample counter
                    }

                acquisition_message = 1;
                if (!d_det.enabled())  // else raised by the acquisition core
                    {
                        this->message_port_pub(pmt::mp("events"), pmt::from_long(acquisition_message));
                    }
                LOG(INFO) << "Successful acquisition in channel " << d_channel
                          << " for satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;

                // Copy and push current Gnss_Synchro to monitor queue
                if (d_acq_params.enable_monitor_output)
                    {
                        auto **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);
                        Gnss_Synchro current_synchro_data = Gnss_Synchro();
                        current_synchro_data = *d_gnss_synchro;
                        *out[0] = std::move(current_synchro_data);
                        noutput_items = 1;  // Number of Gnss_Synchro objects produced
                    }

                break;
            }

        case 3:
            {
                // Declare negative acquisition using a message port
                DLOG(INFO) << "negative acquisition";
                DLOG(INFO) << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
                DLOG(INFO) << "sample_stamp " << d_sample_counter;
                DLOG(INFO) << "test statistics value " << d_test_statistics;
                DLOG(INFO) << "test statistics threshold " << d_acq_params.threshold;
                DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
                DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
                DLOG(INFO) << "magnitude " << d_mag;
                DLOG(INFO) << "input signal power " << d_input_power;

                d_state = 0;  // the acquisition core has set d_active = false

                // A new search may already be requested (e.g., the channel repeats a failed one)
                keep_input = d_det.enabled() && d_active;
                if (!keep_input)
                    {
                        d_sample_counter += static_cast<uint64_t>(d_fft_size) * ninput_items[0];  // sample counter
                    }

                acquisition_message = 2;
                if (!d_det.enabled())  // else raised by the acquisition core
                    {
                        this->message_port_pub(pmt::mp("events"), pmt::from_long(acquisition_message));
                    }

                break;
            }
        }

    if (!keep_input)
        {
            consume_each(ninput_items[0]);
        }

    return noutput_items;
}
