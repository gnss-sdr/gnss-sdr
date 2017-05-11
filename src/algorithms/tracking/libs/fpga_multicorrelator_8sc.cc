/*!
 * \file fpga_multicorrelator_8sc.cc
 * \brief High optimized FPGA vector correlator class
 * \authors <ul>
 *    <li> Marc Majoral, 2017. mmajoral(at)cttc.cat
 *    <li> Javier Arribas, 2015. jarribas(at)cttc.es
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

#include "fpga_multicorrelator_8sc.h"
#include <cmath>

// FPGA stuff
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

#define PAGE_SIZE 0x10000
#define MAX_LENGTH_DEVICEIO_NAME 50
#define CODE_RESAMPLER_NUM_BITS_PRECISION 20
#define CODE_PHASE_STEP_CHIPS_NUM_NBITS CODE_RESAMPLER_NUM_BITS_PRECISION
#define pwrtwo(x) (1 << (x))
#define MAX_CODE_RESAMPLER_COUNTER pwrtwo(CODE_PHASE_STEP_CHIPS_NUM_NBITS) // 2^CODE_PHASE_STEP_CHIPS_NUM_NBITS
#define PHASE_CARR_NBITS 32
#define PHASE_CARR_NBITS_INT 1
#define PHASE_CARR_NBITS_FRAC PHASE_CARR_NBITS - PHASE_CARR_NBITS_INT



bool fpga_multicorrelator_8sc::init(int n_correlators)
{
    d_n_correlators = n_correlators;

    // instantiate variable length vectors
    d_initial_index = static_cast<unsigned*>(volk_gnsssdr_malloc(n_correlators * sizeof(unsigned), volk_gnsssdr_get_alignment()));
    d_initial_interp_counter = static_cast<unsigned*>(volk_gnsssdr_malloc(n_correlators * sizeof(unsigned), volk_gnsssdr_get_alignment()));

    return true;
}


void fpga_multicorrelator_8sc::set_initial_sample(int samples_offset)
{
    d_initial_sample_counter = samples_offset;
}


bool fpga_multicorrelator_8sc::set_local_code_and_taps(
        int code_length_chips,
        const lv_16sc_t* local_code_in,
        float *shifts_chips)
{
    d_local_code_in = local_code_in;
    d_shifts_chips = shifts_chips;
    d_code_length_chips = code_length_chips;

    fpga_multicorrelator_8sc::fpga_configure_tracking_gps_local_code();

    return true;
}


bool fpga_multicorrelator_8sc::set_output_vectors(lv_16sc_t* corr_out)
{
    // Save CPU pointers
    d_corr_out = corr_out;

    return true;
}


void fpga_multicorrelator_8sc::update_local_code(float rem_code_phase_chips)
{
    d_rem_code_phase_chips = rem_code_phase_chips;

    fpga_multicorrelator_8sc::fpga_compute_code_shift_parameters();
    fpga_multicorrelator_8sc::fpga_configure_code_parameters_in_fpga();
}


bool fpga_multicorrelator_8sc::Carrier_wipeoff_multicorrelator_resampler(
        float rem_carrier_phase_in_rad,
        float phase_step_rad,
        float rem_code_phase_chips,
        float code_phase_step_chips,
        int signal_length_samples)
{
    update_local_code(rem_code_phase_chips);

    d_rem_carrier_phase_in_rad = rem_carrier_phase_in_rad;
    d_code_phase_step_chips = code_phase_step_chips;
    d_phase_step_rad = phase_step_rad;
    d_correlator_length_samples = signal_length_samples;

    fpga_multicorrelator_8sc::fpga_compute_signal_parameters_in_fpga();
    fpga_multicorrelator_8sc::fpga_configure_signal_parameters_in_fpga();
    fpga_multicorrelator_8sc::fpga_launch_multicorrelator_fpga();

    int irq_count;
    ssize_t nb;
    // wait for interrupt
    nb=read(d_fd, &irq_count, sizeof(irq_count));
    if (nb != sizeof(irq_count))
        {
            printf("Tracking_module Read failed to retrive 4 bytes!\n");
            printf("Tracking_module Interrupt number %d\n", irq_count);
        }

    fpga_multicorrelator_8sc::read_tracking_gps_results();

    return true;
}


fpga_multicorrelator_8sc::fpga_multicorrelator_8sc()
{
    d_local_code_in = nullptr;
    d_shifts_chips = nullptr;
    d_corr_out = nullptr;
    d_code_length_chips = 0;
    d_n_correlators = 0;
}


fpga_multicorrelator_8sc::~fpga_multicorrelator_8sc()
{
    close(d_fd);
}


bool fpga_multicorrelator_8sc::free()
{
    // unlock the hardware
    fpga_multicorrelator_8sc::unlock_channel(); // unlock the channel

    // free the FPGA dynamically created variables
    if (d_initial_index != nullptr)
        {
            volk_gnsssdr_free(d_initial_index);
            d_initial_index = nullptr;
        }

    if (d_initial_interp_counter != nullptr)
        {
            volk_gnsssdr_free(d_initial_interp_counter);
            d_initial_interp_counter = nullptr;
        }

    return true;
}


void fpga_multicorrelator_8sc::set_channel(unsigned int channel)
{
    d_channel = channel;

    snprintf(d_device_io_name, MAX_LENGTH_DEVICEIO_NAME, "/dev/uio%d",d_channel);
    printf("Opening Device Name : %s\n", d_device_io_name);

    if ((d_fd = open(d_device_io_name, O_RDWR | O_SYNC )) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio" << d_device_io_name;
        }
    d_map_base = (volatile unsigned *)mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, d_fd,0);

    if (d_map_base == (void *) -1)
        {
            LOG(WARNING) << "Cannot map the FPGA tracking module " << d_channel << "into user memory";
        }

    // sanity check : check test register
    unsigned writeval = 0x55AA;
    unsigned readval;
    readval = fpga_multicorrelator_8sc::fpga_acquisition_test_register(writeval);
    if (writeval != readval)
        {
            LOG(WARNING) << "Test register sanity check failed";
        }
    else
        {
            LOG(INFO) << "Test register sanity check success !";
        }

}


unsigned fpga_multicorrelator_8sc::fpga_acquisition_test_register(unsigned writeval)
{
    unsigned readval;
    // write value to test register
    d_map_base[15] = writeval;
    // read value from test register
    readval = d_map_base[15];
    // return read value
    return readval;
}


void fpga_multicorrelator_8sc::fpga_configure_tracking_gps_local_code(void)
{
    int k,s;
    unsigned temp;
    unsigned *ena_write_signals;
    ena_write_signals = new unsigned[d_n_correlators];
    ena_write_signals[0] = 0x00000000;
    ena_write_signals[1] = 0x20000000;
    for (s = 2; s < d_n_correlators; s++)
        {
            ena_write_signals[s]= ena_write_signals[s-1]*2; //0x40000000;
        }

    for (s = 0; s < d_n_correlators; s++)
        {
            // clear memory address counter
            d_map_base[11] = 0x10000000;
            // write correlator 0
            for (k = 0; k < d_code_length_chips; k++)
                {
                    if (lv_creal(d_local_code_in[k]) == 1)
                        {
                            temp = 1;
                        }
                    else
                        {
                            temp = 0;
                        }
                    d_map_base[11] = 0x0C000000 | (temp & 0xFFFF) | ena_write_signals[s];
                }
        }

    delete [] ena_write_signals;
}


void fpga_multicorrelator_8sc::fpga_compute_code_shift_parameters(void)
{
    float tempvalues[3];
    float tempvalues2[3];
    float tempvalues3[3];
    int i;

    for (i = 0; i < d_n_correlators; i++)
        {
            // initial index calculation
            tempvalues[i] = floor(d_shifts_chips[i] + d_rem_code_phase_chips);
            if (tempvalues[i] < 0)
                {
                    tempvalues2[i] = tempvalues[i] + d_code_length_chips; // % operator does not work as in Matlab with negative numbers
                }
            else
                {
                    tempvalues2[i] = tempvalues[i];
                }
            d_initial_index[i] = (unsigned) ((int) tempvalues2[i]) % d_code_length_chips;

            // initial interpolator counter calculation
            tempvalues3[i] = fmod(d_shifts_chips[i]+ d_rem_code_phase_chips,1.0);
            if (tempvalues3[i] < 0)
                {
                    tempvalues3[i] = tempvalues3[i] + 1.0; // fmod operator does not work as in Matlab with negative numbers
                }
            d_initial_interp_counter[i] = (unsigned) floor(MAX_CODE_RESAMPLER_COUNTER * tempvalues3[i]);
        }
}


void fpga_multicorrelator_8sc::fpga_configure_code_parameters_in_fpga(void)
{
    int i;
    for (i = 0; i < d_n_correlators; i++)
        {
            d_map_base[1+i] = d_initial_index[i];
            d_map_base[1 + d_n_correlators + i] = d_initial_interp_counter[i];
        }
    d_map_base[8] = d_code_length_chips - 1; // number of samples - 1
}


void fpga_multicorrelator_8sc::fpga_compute_signal_parameters_in_fpga(void)
{
    float d_rem_carrier_phase_in_rad_temp;

    d_code_phase_step_chips_num = (unsigned) roundf(MAX_CODE_RESAMPLER_COUNTER * d_code_phase_step_chips);

    if (d_rem_carrier_phase_in_rad > M_PI)
        {
            d_rem_carrier_phase_in_rad_temp = -2*M_PI + d_rem_carrier_phase_in_rad;
        }
    else if (d_rem_carrier_phase_in_rad < - M_PI)
        {
            d_rem_carrier_phase_in_rad_temp = 2*M_PI + d_rem_carrier_phase_in_rad;
        }
    else
        {
            d_rem_carrier_phase_in_rad_temp = d_rem_carrier_phase_in_rad;
        }

    d_rem_carr_phase_rad_int = (int) roundf((fabs(d_rem_carrier_phase_in_rad_temp)/M_PI)*pow(2, PHASE_CARR_NBITS_FRAC));

    if (d_rem_carrier_phase_in_rad_temp < 0)
        {
            d_rem_carr_phase_rad_int = -d_rem_carr_phase_rad_int;
        }
    d_phase_step_rad_int = (int) roundf((fabs(d_phase_step_rad)/M_PI)*pow(2, PHASE_CARR_NBITS_FRAC)); // the FPGA accepts a range for the phase step between -pi and +pi

    if (d_phase_step_rad < 0)
        {
            d_phase_step_rad_int = -d_phase_step_rad_int;
        }
}


void fpga_multicorrelator_8sc::fpga_configure_signal_parameters_in_fpga(void)
{
    d_map_base[0] = d_code_phase_step_chips_num;
    d_map_base[7] = d_correlator_length_samples - 1;
    d_map_base[9] = d_rem_carr_phase_rad_int;
    d_map_base[10] = d_phase_step_rad_int;
    d_map_base[12] = 0; // lock the channel
    d_map_base[13] = d_initial_sample_counter;
}


void fpga_multicorrelator_8sc::fpga_launch_multicorrelator_fpga(void)
{
    // enable interrupts
    int reenable = 1;
    write(d_fd, (void *)&reenable, sizeof(int));

    d_map_base[14] = 0; // writing anything to reg 14 launches the tracking
}


void fpga_multicorrelator_8sc::read_tracking_gps_results(void)
{
    int *readval_real;
    int *readval_imag;
    int k;
    readval_real = new int[d_n_correlators];
    readval_imag = new int[d_n_correlators];

    for (k =0 ; k < d_n_correlators; k++)
        {
            readval_real[k] = d_map_base[1 + k];
            if (readval_real[k] >= 1048576)  // 0x100000 (21 bits two's complement)
                {
                    readval_real[k] = -2097152 + readval_real[k];
                }
            readval_real[k] = readval_real[k] * 2; // the results are shifted two bits to the left due to the complex multiplier in the FPGA

        }
    for (k = 0; k < d_n_correlators; k++)
        {
            readval_imag[k] = d_map_base[1 + d_n_correlators + k];
            if (readval_imag[k] >= 1048576) // 0x100000 (21 bits two's complement)
                {
                    readval_imag[k] = -2097152 + readval_imag[k];
                }
            readval_imag[k] = readval_imag[k] * 2; // the results are shifted two bits to the left due to the complex multiplier in the FPGA
        }

    for (k = 0; k < d_n_correlators; k++)
        {
            d_corr_out[k] = lv_cmake(readval_real[k], readval_imag[k]);
        }

    delete[] readval_real;
    delete[] readval_imag;
}


void fpga_multicorrelator_8sc::unlock_channel(void)
{
    // unlock the channel to let the next samples go through
    d_map_base[12] = 1; // unlock the channel
}
