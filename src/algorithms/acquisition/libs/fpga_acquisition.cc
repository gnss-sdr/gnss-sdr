/*!
 * \file fpga_acquisition.cc
 * \brief High optimized FPGA vector correlator class
 * \authors <ul>
 *          <li> Marc Majoral, 2018. mmajoral(at)cttc.cat
 *          </ul>
 *
 * Class that controls and executes a high optimized acquisition HW
 * accelerator in the FPGA
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "fpga_acquisition.h"
#include "GPS_L1_CA.h"
#include "gps_sdr_signal_processing.h"
#include <glog/logging.h>
#include <fcntl.h>     // libraries used by the GIPO
#include <sys/mman.h>  // libraries used by the GIPO


#define PAGE_SIZE 0x10000                     // default page size for the multicorrelator memory map
#define MAX_PHASE_STEP_RAD 0.999999999534339  // 1 - pow(2,-31);
#define RESET_ACQUISITION 2                   // command to reset the multicorrelator
#define LAUNCH_ACQUISITION 1                  // command to launch the multicorrelator
#define TEST_REG_SANITY_CHECK 0x55AA          // value to check the presence of the test register (to detect the hw)
#define LOCAL_CODE_CLEAR_MEM 0x10000000       // command to clear the internal memory of the multicorrelator
#define MEM_LOCAL_CODE_WR_ENABLE 0x0C000000   // command to enable the ENA and WR pins of the internal memory of the multicorrelator
#define POW_2_2 4                             // 2^2 (used for the conversion of floating point numbers to integers)
#define POW_2_29 536870912                    // 2^29 (used for the conversion of floating point numbers to integers)
#define SELECT_LSB 0x00FF                     // value to select the least significant byte
#define SELECT_MSB 0XFF00                     // value to select the most significant byte
#define SELECT_16_BITS 0xFFFF                 // value to select 16 bits
#define SHL_8_BITS 256                        // value used to shift a value 8 bits to the left


bool fpga_acquisition::init()
{
    // configure the acquisition with the main initialization values
    fpga_acquisition::configure_acquisition();
    return true;
}


bool fpga_acquisition::set_local_code(uint32_t PRN)
{
    // select the code with the chosen PRN
    fpga_acquisition::fpga_configure_acquisition_local_code(
        &d_all_fft_codes[d_nsamples_total * (PRN - 1)]);
    return true;
}


fpga_acquisition::fpga_acquisition(std::string device_name,
    uint32_t nsamples,
    uint32_t doppler_max,
    uint32_t nsamples_total, int64_t fs_in,
    uint32_t sampled_ms, uint32_t select_queue,
    lv_16sc_t *all_fft_codes)
{
    uint32_t vector_length = nsamples_total * sampled_ms;
    // initial values
    d_device_name = device_name;
    d_fs_in = fs_in;
    d_vector_length = vector_length;
    d_nsamples = nsamples;  // number of samples not including padding
    d_select_queue = select_queue;
    d_nsamples_total = nsamples_total;
    d_doppler_max = doppler_max;
    d_doppler_step = 0;
    d_fd = 0;              // driver descriptor
    d_map_base = nullptr;  // driver memory map
    d_all_fft_codes = all_fft_codes;

    // open communication with HW accelerator
    if ((d_fd = open(d_device_name.c_str(), O_RDWR | O_SYNC)) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio" << d_device_name;
        }
    d_map_base = reinterpret_cast<volatile uint32_t *>(mmap(NULL, PAGE_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, d_fd, 0));

    if (d_map_base == reinterpret_cast<void *>(-1))
        {
            LOG(WARNING) << "Cannot map the FPGA acquisition module into user memory";
        }

    // sanity check : check test register
    uint32_t writeval = TEST_REG_SANITY_CHECK;
    uint32_t readval;
    readval = fpga_acquisition::fpga_acquisition_test_register(writeval);
    if (writeval != readval)
        {
            LOG(WARNING) << "Acquisition test register sanity check failed";
        }
    else
        {
            LOG(INFO) << "Acquisition test register sanity check success!";
        }
    fpga_acquisition::reset_acquisition();
    DLOG(INFO) << "Acquisition FPGA class created";
}


fpga_acquisition::~fpga_acquisition()
{
    close_device();
}


bool fpga_acquisition::free()
{
    return true;
}


uint32_t fpga_acquisition::fpga_acquisition_test_register(uint32_t writeval)
{
    uint32_t readval;
    // write value to test register
    d_map_base[15] = writeval;
    // read value from test register
    readval = d_map_base[15];
    // return read value
    return readval;
}


void fpga_acquisition::fpga_configure_acquisition_local_code(lv_16sc_t fft_local_code[])
{
    uint16_t local_code;
    uint32_t k, tmp, tmp2;
    uint32_t fft_data;
    // clear memory address counter
    d_map_base[4] = LOCAL_CODE_CLEAR_MEM;
    // write local code
    for (k = 0; k < d_vector_length; k++)
        {
            tmp = fft_local_code[k].real();
            tmp2 = fft_local_code[k].imag();
            local_code = (tmp & SELECT_LSB) | ((tmp2 * SHL_8_BITS) & SELECT_MSB);  // put together the real part and the imaginary part
            fft_data = MEM_LOCAL_CODE_WR_ENABLE | (local_code & SELECT_16_BITS);
            d_map_base[4] = fft_data;
        }
}


void fpga_acquisition::run_acquisition(void)
{
    // enable interrupts
    int32_t reenable = 1;
    write(d_fd, reinterpret_cast<void *>(&reenable), sizeof(int32_t));
    // launch the acquisition process
    d_map_base[6] = LAUNCH_ACQUISITION;  // writing anything to reg 6 launches the acquisition process

    int32_t irq_count;
    ssize_t nb;
    // wait for interrupt
    nb = read(d_fd, &irq_count, sizeof(irq_count));
    if (nb != sizeof(irq_count))
        {
            printf("acquisition module Read failed to retrieve 4 bytes!\n");
            printf("acquisition module Interrupt number %d\n", irq_count);
        }
}


void fpga_acquisition::configure_acquisition()
{
    d_map_base[0] = d_select_queue;
    d_map_base[1] = d_vector_length;
    d_map_base[2] = d_nsamples;
    d_map_base[5] = static_cast<int32_t>(log2(static_cast<float>(d_vector_length)));  // log2 FFTlength
}


void fpga_acquisition::set_phase_step(uint32_t doppler_index)
{
    float phase_step_rad_real;
    float phase_step_rad_int_temp;
    int32_t phase_step_rad_int;
    int32_t doppler = static_cast<int>(-d_doppler_max) + d_doppler_step * doppler_index;
    float phase_step_rad = GPS_TWO_PI * doppler / static_cast<float>(d_fs_in);
    // The doppler step can never be outside the range -pi to +pi, otherwise there would be aliasing
    // The FPGA expects phase_step_rad between -1 (-pi) to +1 (+pi)
    // The FPGA also expects the phase to be negative since it produces cos(x) -j*sin(x)
    // while the gnss-sdr software (volk_gnsssdr_s32f_sincos_32fc) generates cos(x) + j*sin(x)
    phase_step_rad_real = phase_step_rad / (GPS_TWO_PI / 2);
    // avoid saturation of the fixed point representation in the fpga
    // (only the positive value can saturate due to the 2's complement representation)
    if (phase_step_rad_real >= 1.0)
        {
            phase_step_rad_real = MAX_PHASE_STEP_RAD;
        }
    phase_step_rad_int_temp = phase_step_rad_real * POW_2_2;               // * 2^2
    phase_step_rad_int = (int32_t)(phase_step_rad_int_temp * (POW_2_29));  // * 2^29 (in total it makes x2^31 in two steps to avoid the warnings
    d_map_base[3] = phase_step_rad_int;
}


void fpga_acquisition::read_acquisition_results(uint32_t *max_index,
    float *max_magnitude, uint32_t *initial_sample, float *power_sum)
{
    uint32_t readval = 0;
    readval = d_map_base[1];
    *initial_sample = readval;
    readval = d_map_base[2];
    *max_magnitude = static_cast<float>(readval);
    readval = d_map_base[4];
    *power_sum = static_cast<float>(readval);
    readval = d_map_base[3];
    *max_index = readval;
}


void fpga_acquisition::block_samples()
{
    d_map_base[14] = 1;  // block the samples
}


void fpga_acquisition::unblock_samples()
{
    d_map_base[14] = 0;  // unblock the samples
}


void fpga_acquisition::close_device()
{
    uint32_t *aux = const_cast<uint32_t *>(d_map_base);
    if (munmap(static_cast<void *>(aux), PAGE_SIZE) == -1)
        {
            printf("Failed to unmap memory uio\n");
        }
    close(d_fd);
}


void fpga_acquisition::reset_acquisition(void)
{
    d_map_base[6] = RESET_ACQUISITION;  // writing a 2 to d_map_base[6] resets the multicorrelator
}
