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
#include <glog/logging.h>    // for LOG
#include <cmath>             // for log2
#include <fcntl.h>           // libraries used by the GIPO
#include <iostream>          // for operator<<
#include <sys/mman.h>        // libraries used by the GIPO
#include <unistd.h>          // for write, close, read, ssize_t
#include <utility>           // for move


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
    uint32_t nsamples,
    uint32_t doppler_max,
    uint32_t nsamples_total,
    int64_t fs_in,
    uint32_t select_queue,
    uint32_t *all_fft_codes,
    uint32_t excludelimit)
{
    const uint32_t vector_length = nsamples_total;

    // initial values
    d_device_name = std::move(device_name);
    d_fs_in = fs_in;
    d_vector_length = vector_length;
    d_excludelimit = excludelimit;
    d_nsamples = nsamples;  // number of samples not including padding
    d_select_queue = select_queue;
    d_nsamples_total = nsamples_total;
    d_doppler_max = doppler_max;
    d_doppler_step = 0;
    d_fd = 0;              // driver descriptor
    d_map_base = nullptr;  // driver memory map
    d_all_fft_codes = all_fft_codes;
    Fpga_Acquisition::open_device();
    Fpga_Acquisition::reset_acquisition();
    Fpga_Acquisition::fpga_acquisition_test_register();
    Fpga_Acquisition::close_device();
    d_PRN = 0;
    DLOG(INFO) << "Acquisition FPGA class created";
}


bool Fpga_Acquisition::set_local_code(uint32_t PRN)
{
    // select the code with the chosen PRN
    d_PRN = PRN;
    return true;
}


void Fpga_Acquisition::write_local_code()
{
    d_map_base[9] = LOCAL_CODE_CLEAR_MEM;
    // write local code
    for (uint32_t k = 0; k < d_vector_length; k++)
        {
            d_map_base[6] = d_all_fft_codes[d_nsamples_total * (d_PRN - 1) + k];
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
    d_map_base = reinterpret_cast<volatile uint32_t *>(mmap(nullptr, PAGE_SIZE_DEFAULT,
        PROT_READ | PROT_WRITE, MAP_SHARED, d_fd, 0));

    if (d_map_base == reinterpret_cast<void *>(-1))
        {
            LOG(WARNING) << "Cannot map the FPGA acquisition module into user memory";
            std::cout << "Acq: cannot map deviceio" << d_device_name << '\n';
        }
}


void Fpga_Acquisition::fpga_acquisition_test_register()
{
    // sanity check : check test register
    const uint32_t writeval = TEST_REG_SANITY_CHECK;

    // write value to test register
    d_map_base[15] = writeval;
    // read value from test register
    const uint32_t readval = d_map_base[15];

    if (writeval != readval)
        {
            LOG(WARNING) << "Acquisition test register sanity check failed";
        }
    else
        {
            LOG(INFO) << "Acquisition test register sanity check success!";
        }
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
    d_map_base[8] = LAUNCH_ACQUISITION;  // writing a 1 to reg 8 launches the acquisition process
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
    d_map_base[11] = total_block_exp;
}


void Fpga_Acquisition::set_doppler_sweep(uint32_t num_sweeps, uint32_t doppler_step, int32_t doppler_min)
{
    // The doppler step can never be outside the range -pi to +pi, otherwise there would be aliasing
    // The FPGA expects phase_step_rad between -1 (-pi) to +1 (+pi)
    float phase_step_rad_real = 2.0F * (doppler_min) / static_cast<float>(d_fs_in);
    auto phase_step_rad_int = static_cast<int32_t>(phase_step_rad_real * (POW_2_31));
    d_map_base[3] = phase_step_rad_int;

    // repeat the calculation with the doppler step
    phase_step_rad_real = 2.0F * (doppler_step) / static_cast<float>(d_fs_in);
    phase_step_rad_int = static_cast<int32_t>(phase_step_rad_real * (POW_2_31));  // * 2^29 (in total it makes x2^31 in two steps to avoid the warnings
    d_map_base[4] = phase_step_rad_int;

    // write number of doppler sweeps
    d_map_base[5] = num_sweeps;
}


void Fpga_Acquisition::configure_acquisition()
{
    // Fpga_Acquisition::();
    d_map_base[0] = d_select_queue;
    d_map_base[1] = d_vector_length;
    d_map_base[2] = d_nsamples;
    d_map_base[7] = static_cast<int32_t>(std::log2(static_cast<float>(d_vector_length)));  // log2 FFTlength
    d_map_base[12] = d_excludelimit;
}


void Fpga_Acquisition::read_acquisition_results(uint32_t *max_index,
    float *firstpeak, float *secondpeak, uint64_t *initial_sample, float *power_sum, uint32_t *doppler_index, uint32_t *total_blk_exp)
{
    uint32_t readval = d_map_base[1];  // read sample counter (LSW)
    auto initial_sample_tmp = static_cast<uint64_t>(readval);

    uint64_t readval_long = d_map_base[2];               // read sample counter (MSW)
    uint64_t readval_long_shifted = readval_long << 32;  // 2^32

    initial_sample_tmp += readval_long_shifted;  // 2^32
    *initial_sample = initial_sample_tmp;

    readval = d_map_base[3];  // read first peak value
    *firstpeak = static_cast<float>(readval);

    readval = d_map_base[4];  // read second peak value
    *secondpeak = static_cast<float>(readval);

    readval = d_map_base[5];  // read max index position
    *max_index = readval;

    *power_sum = 0;  // power sum is not used

    readval = d_map_base[7];  // read doppler index -- this read releases the interrupt line
    *doppler_index = readval;

    readval = d_map_base[8];  // read FFT block exponent
    *total_blk_exp = readval;
}


void Fpga_Acquisition::close_device()
{
    auto *aux = const_cast<uint32_t *>(d_map_base);
    if (munmap(static_cast<void *>(aux), PAGE_SIZE_DEFAULT) == -1)
        {
            std::cout << "Failed to unmap memory uio\n";
        }
    close(d_fd);
}


void Fpga_Acquisition::reset_acquisition()
{
    // printf("============ resetting the hw now from the acquisition ===============");
    d_map_base[8] = RESET_ACQUISITION;  // writing a 2 to d_map_base[8] resets the acquisition. This causes a reset of all
                                        // the FPGA HW modules including the multicorrelators
}


// this function is only used for the unit tests
void Fpga_Acquisition::read_fpga_total_scale_factor(uint32_t *total_scale_factor, uint32_t *fw_scale_factor)
{
    uint32_t readval = d_map_base[8];
    *total_scale_factor = readval;
    // only the total scale factor is used for the tests (fw scale factor to be removed)
    *fw_scale_factor = 0;
}


void Fpga_Acquisition::read_result_valid(uint32_t *result_valid)
{
    uint32_t readval = d_map_base[0];
    *result_valid = readval;
}
