/*!
 * \file fpga_acquisition.cc
 * \brief Highly optimized FPGA vector correlator class
 * \authors <ul>
 *          <li> Marc Majoral, 2019. mmajoral(at)cttc.cat
 *          </ul>
 *
 * Class that controls and executes a highly optimized acquisition HW
 * accelerator in the FPGA
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

#include "fpga_acquisition.h"
#include "MATH_CONSTANTS.h"  // for TWO_PI
#include <cmath>             // for log2
#include <fcntl.h>           // libraries used by the GIPO
#include <iostream>          // for operator<<
#include <sys/mman.h>        // libraries used by the GIPO
#include <unistd.h>          // for write, close, read, ssize_t

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

#ifndef TEMP_FAILURE_RETRY
#define TEMP_FAILURE_RETRY(exp)              \
    ({                                       \
        decltype(exp) _rc;                   \
        do                                   \
            {                                \
                _rc = (exp);                 \
            }                                \
        while (_rc == -1 && errno == EINTR); \
        _rc;                                 \
    })
#endif

Fpga_Acquisition::Fpga_Acquisition(std::string device_name,
    uint32_t select_queue,
    std::vector<std::pair<uint32_t, uint32_t>> &downsampling_filter_specs,
    uint32_t &max_FFT_size) : d_device_name(device_name),
                              d_resampled_fs(0),
                              d_map_base(nullptr),  // driver memory map
                              d_all_fft_codes(nullptr),
                              d_fd(0),  // driver descriptor
                              d_fft_size(0),
                              d_excludelimit(0),
                              d_nsamples(0),  // number of samples not including padding
                              d_filter_num(0),
                              d_downsampling_factor(1),
                              d_downsampling_filter_delay(0),
                              d_select_queue(select_queue),
                              d_doppler_max(0),
                              d_doppler_step(0),
                              d_PRN(0),
                              d_IP_core_version(0)
{
    Fpga_Acquisition::open_device();
    Fpga_Acquisition::reset_acquisition();
    Fpga_Acquisition::fpga_acquisition_test_register();
    Fpga_Acquisition::read_ipcore_info(downsampling_filter_specs, max_FFT_size);
    Fpga_Acquisition::close_device();
    DLOG(INFO) << "Acquisition FPGA class created";
}

void Fpga_Acquisition::init(uint32_t nsamples, uint32_t doppler_max, uint32_t fft_size,
    int64_t resampled_fs, uint32_t downsampling_filter_num, uint32_t excludelimit, uint32_t *all_fft_codes)
{
    d_resampled_fs = resampled_fs;
    d_all_fft_codes = all_fft_codes;
    d_fft_size = fft_size;
    d_excludelimit = excludelimit;
    d_nsamples = nsamples;
    d_filter_num = downsampling_filter_num;
    if (d_filter_num > 0)
        {
            d_downsampling_factor = d_downsampling_filter_specs[d_filter_num - 1].first;
            d_downsampling_filter_delay = d_downsampling_filter_specs[d_filter_num - 1].second;
        }
    d_doppler_max = doppler_max;
}


bool Fpga_Acquisition::set_local_code(uint32_t PRN)
{
    // select the code with the chosen PRN
    d_PRN = PRN;
    return true;
}


void Fpga_Acquisition::write_local_code()
{
    d_map_base[CLEAR_MEM_REG_ADDR] = LOCAL_CODE_CLEAR_MEM;
    // write local code
    for (uint32_t k = 0; k < d_fft_size; k++)
        {
            d_map_base[PROG_MEM_ADDR] = d_all_fft_codes[d_fft_size * (d_PRN - 1) + k];
        }
}


void Fpga_Acquisition::open_device()
{
    // open communication with HW accelerator
    if ((d_fd = open(d_device_name.c_str(), O_RDWR | O_SYNC)) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio" << d_device_name;
            std::cout << "Acq: cannot open deviceio" << d_device_name << '\n';
        }
    d_map_base = reinterpret_cast<volatile uint32_t *>(mmap(nullptr, FPGA_PAGE_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, d_fd, 0));

    if (d_map_base == reinterpret_cast<void *>(-1))
        {
            LOG(WARNING) << "Cannot map the FPGA acquisition module into user memory";
            std::cout << "Acq: cannot map deviceio" << d_device_name << '\n';
        }
}


void Fpga_Acquisition::fpga_acquisition_test_register()
{
    // write value to test register
    d_map_base[TEST_REG_ADDR] = TEST_REG_SANITY_CHECK;
    // read value from test register
    const uint32_t readval = d_map_base[TEST_REG_ADDR];

    if (readval != TEST_REG_SANITY_CHECK)
        {
            LOG(WARNING) << "Acquisition test register sanity check failed";
        }
    else
        {
            LOG(INFO) << "Acquisition test register sanity check success!";
        }
}

void Fpga_Acquisition::read_ipcore_info(std::vector<std::pair<uint32_t, uint32_t>> &downsampling_filter_specs, uint32_t &max_FFT_size)
{
    d_IP_core_version = d_map_base[FPGA_IP_CORE_VERSION_REG_ADDR];
    if (d_IP_core_version == FPGA_ACQ_IP_VERSION_1)
        {
            // FPGA acquisition IP core version FPGA_ACQ_IP_VERSION_1
            max_FFT_size = d_map_base[MAX_FFT_SIZE_REG_ADDR];
            if (d_select_queue == ACQ_BUFF_0)
                {
                    // Check if the requested downsampling filter is available on the FPGA and read its implemented latency.
                    uint32_t downsampling_filter_dec_factors = d_map_base[DOWNSAMPLING_FILTER_DEC_FACTORS_REG_ADDR];
                    uint32_t downsampling_filter_latencies = d_map_base[DOWNSAMPLING_FILTER_LATENCIES_REG_ADDR];
                    for (uint32_t filt_num = 1; filt_num <= MAX_FILTERS_AVAILABLE; filt_num++)
                        {
                            // The information about the instantiated downsampling filters in the FPGA is ordered such that the largest downsampling factor is stored in the least significant bits (LSBs), while the smallest downsampling factor is stored in the most significant bits (MSBs).
                            uint32_t dec_factor = downsampling_filter_dec_factors & BIT_MASK_4;
                            downsampling_filter_dec_factors = downsampling_filter_dec_factors >> RSHIFT_4_BITS;

                            uint32_t filter_latency = downsampling_filter_latencies & BIT_MASK_8;
                            downsampling_filter_latencies = downsampling_filter_latencies >> RSHIFT_8_BITS;
                            if (dec_factor != 0)
                                {
                                    downsampling_filter_specs.emplace_back(dec_factor, filter_latency);
                                }
                            else
                                {
                                    break;
                                }
                        }
                }
        }
    else
        {
            // FPGA Acquisition IP core versions earlier than FPGA_ACQ_IP_VERSION_1
            max_FFT_size = DEFAULT_MAX_FFT_SIZE;
            if (d_select_queue == ACQ_BUFF_0)
                {
                    // An acquisition resampler with a downsampling factor of DEFAULT_DOWNSAMPLING_FACTOR is implemented in the L1/E1 frequency band
                    downsampling_filter_specs.emplace_back(DEFAULT_DOWNSAMPLING_FACTOR, DEFAULT_DOWNSAMPLING_FILTER_DELAY);
                }
        }

    d_downsampling_filter_specs = downsampling_filter_specs;
}

void Fpga_Acquisition::run_acquisition()
{
    // enable interrupts
    int32_t reenable = 1;
    // int32_t disable_int = 0;
    const ssize_t nbytes = TEMP_FAILURE_RETRY(write(d_fd, reinterpret_cast<void *>(&reenable), sizeof(int32_t)));
    if (nbytes != sizeof(int32_t))
        {
            std::cerr << "Error enabling run in the FPGA.\n";
        }

    // launch the acquisition process
    d_map_base[ACQ_COMMAND_FLAGS_REG_ADDR] = LAUNCH_ACQUISITION;  // writing a 1 to reg 8 launches the acquisition process
    int32_t irq_count;

    // wait for interrupt
    const ssize_t nb = read(d_fd, &irq_count, sizeof(irq_count));
    if (nb != sizeof(irq_count))
        {
            std::cout << "acquisition module Read failed to retrieve 4 bytes!\n";
            std::cout << "acquisition module Interrupt number " << irq_count << '\n';
        }
}


void Fpga_Acquisition::set_block_exp(uint32_t total_block_exp)
{
    d_map_base[MAX_FFT_SCALING_FACTOR_REG_ADDR] = total_block_exp;
}


void Fpga_Acquisition::set_doppler_sweep(uint32_t num_sweeps, uint32_t doppler_step, int32_t doppler_min)
{
    // The doppler step can never be outside the range -pi to +pi, otherwise there would be aliasing
    // The FPGA expects phase_step_rad between -1 (-pi) to +1 (+pi)
    float phase_step_rad_real = 2.0F * (doppler_min) / static_cast<float>(d_resampled_fs);
    d_map_base[DOPPLER_MIN_REG_ADDR] = static_cast<int32_t>(phase_step_rad_real * (POW_2_31));

    // repeat the calculation with the doppler step
    phase_step_rad_real = 2.0F * (doppler_step) / static_cast<float>(d_resampled_fs);
    d_map_base[DOPPLER_STEP_REG_ADDR] = static_cast<int32_t>(phase_step_rad_real * (POW_2_31));  // * 2^29 (in total it makes x2^31 in two steps to avoid the warnings

    // write number of doppler sweeps
    d_map_base[NUM_DOPPLER_SEARCH_STEPS_REG_ADDR] = num_sweeps;
}


void Fpga_Acquisition::configure_acquisition()
{
    // Fpga_Acquisition::();
    d_map_base[FREQ_BAND_DOWNSAMPLE_REG_ADDR] = d_select_queue | (d_filter_num << 4);
    d_map_base[FFT_LENGTH_REG_ADDR] = d_fft_size;
    d_map_base[CORR_NSAMPLES_REG_ADDR] = d_nsamples;
    d_map_base[LOG2_FFT_LENGTH_REG_ADDR] = static_cast<int32_t>(std::log2(static_cast<float>(d_fft_size)));  // log2 FFTlength
    d_map_base[EXCL_LIM_REG_ADDR] = d_excludelimit;
}


void Fpga_Acquisition::read_acquisition_results(uint32_t *max_index,
    float *firstpeak, float *secondpeak, uint64_t *initial_sample, float *power_sum, uint32_t *doppler_index, uint32_t *total_blk_exp)
{
    uint64_t initial_sample_tmp = (static_cast<uint64_t>(d_map_base[SAMPLESTAMP_MSW_REG_ADDR]) << 32) + (static_cast<uint64_t>(d_map_base[SAMPLESTAMP_LSW_REG_ADDR]));
    uint32_t max_index_tmp = d_map_base[ACQ_DELAY_SAMPLES_REG_ADDR];  // read max index position
    if (d_downsampling_factor > 1)
        {
            initial_sample_tmp *= static_cast<uint64_t>(d_downsampling_factor);        // take into account the downsampling factor for the sample stamp
            initial_sample_tmp -= static_cast<uint64_t>(d_downsampling_filter_delay);  // take into account the downsampling filter delay
            max_index_tmp *= d_downsampling_factor;                                    // take into account the downsampling factor for the acq. delay
        }
    *initial_sample = initial_sample_tmp;
    *max_index = max_index_tmp;
    *firstpeak = d_map_base[MAG_SQ_FIRST_PEAK_REG_ADDR];       // read first peak value
    *secondpeak = d_map_base[MAG_SQ_SECOND_PEAK_REG_ADDR];     // read second peak value
    *power_sum = 0;                                            // The FPGA does not currently support power sum.
    *doppler_index = d_map_base[DOPPLER_INDEX_REG_ADDR];       // read doppler index -- this read releases the interrupt line
    *total_blk_exp = d_map_base[FFT_SCALING_FACTOR_REG_ADDR];  // read FFT block exponent
}


void Fpga_Acquisition::close_device()
{
    auto *aux = const_cast<uint32_t *>(d_map_base);
    if (munmap(static_cast<void *>(aux), FPGA_PAGE_SIZE) == -1)
        {
            std::cout << "Failed to unmap memory uio\n";
        }
    close(d_fd);
}


void Fpga_Acquisition::reset_acquisition()
{
    d_map_base[ACQ_COMMAND_FLAGS_REG_ADDR] = RESET_ACQUISITION;  // setting bit 2 resets the acquisition. This causes a reset of all
                                                                 // the FPGA HW modules including the multicorrelators
}


void Fpga_Acquisition::stop_acquisition()
{
    d_map_base[ACQ_COMMAND_FLAGS_REG_ADDR] = STOP_ACQUISITION;  // setting bit 3 stops the acquisition module. This stops all
                                                                // the FPGA HW modules including the multicorrelators
}
