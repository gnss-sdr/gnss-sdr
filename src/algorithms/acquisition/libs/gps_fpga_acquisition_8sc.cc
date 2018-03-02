/*!
 * \file gps_fpga_acquisition_8sc.cc
 * \brief High optimized FPGA vector correlator class
 * \authors <ul>
 *    <li> Marc Majoral, 2017. mmajoral(at)cttc.cat
 * </ul>
 *
 * Class that controls and executes a high optimized vector correlator
 * class in the FPGA
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "gps_fpga_acquisition_8sc.h"
#include "gps_sdr_signal_processing.h"
#include <cmath>

// allocate memory dynamically
#include <new>

// libraries used by DMA test code and GIPO test code
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

// libraries used by DMA test code
#include <sys/stat.h>
#include <stdint.h>
#include <unistd.h>
#include <assert.h>

// libraries used by GPIO test code
#include <stdlib.h>
#include <signal.h>
#include <sys/mman.h>

// logging
#include <glog/logging.h>

// volk
#include <volk/volk.h>

// GPS L1
#include "GPS_L1_CA.h"

#define PAGE_SIZE 0x10000
#define MAX_PHASE_STEP_RAD 0.999999999534339 // 1 - pow(2,-31);
#define NUM_PRNs 32
#define TEST_REGISTER_ACQ_WRITEVAL 0x55AA

bool gps_fpga_acquisition_8sc::init()
{
    // configure the acquisition with the main initialization values
    gps_fpga_acquisition_8sc::configure_acquisition();
    return true;
}

bool gps_fpga_acquisition_8sc::set_local_code(unsigned int PRN)
{
    // select the code with the chosen PRN
    gps_fpga_acquisition_8sc::fpga_configure_acquisition_local_code(
            &d_all_fft_codes[d_nsamples_total * (PRN - 1)]);
    return true;
}

gps_fpga_acquisition_8sc::gps_fpga_acquisition_8sc(std::string device_name,
        unsigned int vector_length, unsigned int nsamples,
        unsigned int doppler_max,
        unsigned int nsamples_total, long fs_in, long freq,
        unsigned int sampled_ms, unsigned select_queue)
{
    // initial values
    d_device_name = device_name;
    d_freq = freq;
    d_fs_in = fs_in;
    d_vector_length = vector_length;
    d_nsamples = nsamples; // number of samples not including padding
    d_select_queue = select_queue;
    d_nsamples_total = nsamples_total;
    d_doppler_max = doppler_max;
    d_doppler_step = 0;
    d_fd = 0; // driver descriptor
    d_map_base = nullptr; // driver memory map
    // Direct FFT
    d_fft_if = new gr::fft::fft_complex(vector_length, true);
    // allocate memory to compute all the PRNs
    // and compute all the possible codes
    std::complex<float>* code = new std::complex<float>[nsamples_total]; // buffer for the local code
    gr_complex* d_fft_codes_padded = static_cast<gr_complex*>(volk_gnsssdr_malloc(nsamples_total * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_all_fft_codes = new lv_16sc_t[nsamples_total * NUM_PRNs]; // memory containing all the possible fft codes for PRN 0 to 32
    float max; // temporary maxima search
    for (unsigned int PRN = 1; PRN <= NUM_PRNs; PRN++)
        {
            gps_l1_ca_code_gen_complex_sampled(code, PRN, fs_in, 0); // generate PRN code
            // fill in zero padding
            for (int s=nsamples;s<nsamples_total;s++)
                {
                    code[s] = 0;
                }
            int offset = 0;
            memcpy(d_fft_if->get_inbuf() + offset, code, sizeof(gr_complex) * nsamples_total); // copy to FFT buffer
            d_fft_if->execute(); // Run the FFT of local code
            volk_32fc_conjugate_32fc(d_fft_codes_padded, d_fft_if->get_outbuf(), nsamples_total); // conjugate values
            max = 0; // initialize maximum value
            for (unsigned int i = 0; i < nsamples_total; i++) // search for maxima
                {
                    if (std::abs(d_fft_codes_padded[i].real()) > max)
                        {
                            max = std::abs(d_fft_codes_padded[i].real());
                        }
                    if (std::abs(d_fft_codes_padded[i].imag()) > max)
                        {
                            max = std::abs(d_fft_codes_padded[i].imag());
                        }
                }
            for (unsigned int i = 0; i < nsamples_total; i++) // map the FFT to the dynamic range of the fixed point values an copy to buffer containing all FFTs
                {
                    d_all_fft_codes[i + nsamples_total * (PRN -1)] = lv_16sc_t(static_cast<int>(d_fft_codes_padded[i].real() * (pow(2, 7) - 1) / max),
                            static_cast<int>(d_fft_codes_padded[i].imag() * (pow(2, 7) - 1) / max));

                }
            }
    // open communication with HW accelerator
    //printf("opening device %s\n", d_device_name.c_str());
    if ((d_fd = open(d_device_name.c_str(), O_RDWR | O_SYNC)) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio" << d_device_name;
            //std::cout << "acquisition cannot open deviceio";
        }
    d_map_base = reinterpret_cast<volatile unsigned *>(mmap(NULL, PAGE_SIZE,
            PROT_READ | PROT_WRITE, MAP_SHARED, d_fd, 0));
    if (d_map_base == reinterpret_cast<void*>(-1))
        {
            LOG(WARNING) << "Cannot map the FPGA acquisition module into user memory";
            //std::cout << "acquisition : could not map the fpga registers to the driver" << std::endl;
        }
    // sanity check : check test register
    // we only nee to do this when the class is created
    // but the device is not opened yet when the class is create
    // because we need to open and close the device every time we run an acquisition
    // since the same device may be used by more than one class (gps acquisition, galileo
    // acquisition, etc ..)
    unsigned writeval = TEST_REGISTER_ACQ_WRITEVAL;
    unsigned readval;
    readval = gps_fpga_acquisition_8sc::fpga_acquisition_test_register(writeval);
    if (writeval != readval)
        {
            LOG(WARNING) << "Acquisition test register sanity check failed";
            //std:: cout << "Acquisition test register sanity check failed" << std::endl;
        }
    else
        {
            //std::cout << "Acquisition test register sanity check success !" << std::endl;
            LOG(INFO) << "Acquisition test register sanity check success !";
        }
    gps_fpga_acquisition_8sc::reset_acquisition();
    DLOG(INFO) << "Acquisition FPGA class created";
    // temporary buffers that we can delete
    delete[] code;
    delete d_fft_if;
    delete[] d_fft_codes_padded;
}

gps_fpga_acquisition_8sc::~gps_fpga_acquisition_8sc()
{
    close_device();
    delete[] d_all_fft_codes;
}

bool gps_fpga_acquisition_8sc::free()
{
    return true;
}

unsigned gps_fpga_acquisition_8sc::fpga_acquisition_test_register(unsigned writeval)
{
    unsigned readval;
    // write value to test register
    d_map_base[15] = writeval;
    // read value from test register
    readval = d_map_base[15];
    // return read value
    return readval;
}

void gps_fpga_acquisition_8sc::fpga_configure_acquisition_local_code(lv_16sc_t fft_local_code[])
{
    unsigned short local_code;
    unsigned int k, tmp, tmp2;
    unsigned int fft_data;
    // clear memory address counter
    d_map_base[4] = 0x10000000;
    // write local code
    for (k = 0; k < d_vector_length; k++)
        {
            tmp = fft_local_code[k].real();
            tmp2 = fft_local_code[k].imag();
            local_code = (tmp & 0xFF) | ((tmp2 * 256) & 0xFF00); // put together the real part and the imaginary part
            fft_data = 0x0C000000 | (local_code & 0xFFFF);
            d_map_base[4] = fft_data;
        }
}

void gps_fpga_acquisition_8sc::run_acquisition(void)
{
    // enable interrupts
    int reenable = 1;
    write(d_fd, reinterpret_cast<void*>(&reenable), sizeof(int));
    // launch the acquisition process
    d_map_base[6] = 1; // writing anything to reg 6 launches the acquisition process

    int irq_count;
    ssize_t nb;
    // wait for interrupt
    nb = read(d_fd, &irq_count, sizeof(irq_count));
    if (nb != sizeof(irq_count))
        {
            printf("acquisition module Read failed to retrieve 4 bytes!\n");
            printf("acquisition module Interrupt number %d\n", irq_count);
        }
}

void gps_fpga_acquisition_8sc::configure_acquisition()
{
    d_map_base[0] = d_select_queue;
    d_map_base[1] = d_vector_length;
    d_map_base[2] = d_nsamples;
    d_map_base[5] = (int) log2((float) d_vector_length); // log2 FFTlength
}

void gps_fpga_acquisition_8sc::set_phase_step(unsigned int doppler_index)
{
    float phase_step_rad_real;
    float phase_step_rad_int_temp;
    int32_t phase_step_rad_int;
    int doppler = static_cast<int>(-d_doppler_max) + d_doppler_step * doppler_index;
    float phase_step_rad = GPS_TWO_PI * (d_freq + doppler) / static_cast<float>(d_fs_in);
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
    phase_step_rad_int_temp = phase_step_rad_real * 4; // * 2^2
    phase_step_rad_int = (int32_t) (phase_step_rad_int_temp * (536870912)); // * 2^29 (in total it makes x2^31 in two steps to avoid the warnings
    d_map_base[3] = phase_step_rad_int;
}

void gps_fpga_acquisition_8sc::read_acquisition_results(uint32_t* max_index,
        float* max_magnitude, unsigned *initial_sample, float *power_sum)
{
    unsigned readval = 0;
    readval = d_map_base[1];
    *initial_sample = readval;
    readval = d_map_base[2];
    *max_magnitude = static_cast<float>(readval);
    readval = d_map_base[4];
    *power_sum = static_cast<float>(readval);
    readval = d_map_base[3];
    *max_index = readval;
}

void gps_fpga_acquisition_8sc::block_samples()
{
    d_map_base[14] = 1; // block the samples
}


void gps_fpga_acquisition_8sc::unblock_samples()
{
    d_map_base[14] = 0; // unblock the samples
}

void gps_fpga_acquisition_8sc::close_device()
{
    unsigned * aux = const_cast<unsigned*>(d_map_base);
    if (munmap(static_cast<void*>(aux), PAGE_SIZE) == -1)
        {
            printf("Failed to unmap memory uio\n");
        }
    close(d_fd);
}

void gps_fpga_acquisition_8sc::reset_acquisition(void)
{
    d_map_base[6] = 2; // writing a 2 to d_map_base[6] resets the multicorrelator
}
