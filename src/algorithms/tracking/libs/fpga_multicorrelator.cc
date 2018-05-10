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

#include "fpga_multicorrelator.h"

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

// string manipulation
#include <string>

// constants
#include "GPS_L1_CA.h"

//#include "gps_sdr_signal_processing.h"

#define NUM_PRNs 32
#define PAGE_SIZE 0x10000
#define MAX_LENGTH_DEVICEIO_NAME 50
#define CODE_RESAMPLER_NUM_BITS_PRECISION 20
#define CODE_PHASE_STEP_CHIPS_NUM_NBITS CODE_RESAMPLER_NUM_BITS_PRECISION
#define pwrtwo(x) (1 << (x))
#define MAX_CODE_RESAMPLER_COUNTER pwrtwo(CODE_PHASE_STEP_CHIPS_NUM_NBITS) // 2^CODE_PHASE_STEP_CHIPS_NUM_NBITS
#define PHASE_CARR_NBITS 32
#define PHASE_CARR_NBITS_INT 1
#define PHASE_CARR_NBITS_FRAC PHASE_CARR_NBITS - PHASE_CARR_NBITS_INT
#define LOCAL_CODE_FPGA_CORRELATOR_SELECT_COUNT 0x20000000
#define LOCAL_CODE_FPGA_CLEAR_ADDRESS_COUNTER 0x10000000
#define LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY 0x0C000000
#define TEST_REGISTER_TRACK_WRITEVAL 0x55AA

int fpga_multicorrelator_8sc::read_sample_counter()
{
	return d_map_base[7];
}

void fpga_multicorrelator_8sc::set_initial_sample(int samples_offset)
{
    d_initial_sample_counter = samples_offset;
    d_map_base[13] = d_initial_sample_counter;
}
       
void fpga_multicorrelator_8sc::set_local_code_and_taps(int code_length_chips,
        float *shifts_chips, int PRN)             
{

    d_shifts_chips = shifts_chips;
    d_code_length_chips = code_length_chips;
    fpga_multicorrelator_8sc::fpga_configure_tracking_gps_local_code(PRN);
}

void fpga_multicorrelator_8sc::set_output_vectors(gr_complex* corr_out)
{
    d_corr_out = corr_out;
}

void fpga_multicorrelator_8sc::update_local_code(float rem_code_phase_chips)
{
    d_rem_code_phase_chips = rem_code_phase_chips;
    fpga_multicorrelator_8sc::fpga_compute_code_shift_parameters();
    fpga_multicorrelator_8sc::fpga_configure_code_parameters_in_fpga();
}


void fpga_multicorrelator_8sc::Carrier_wipeoff_multicorrelator_resampler(
        float rem_carrier_phase_in_rad, float phase_step_rad,
        float rem_code_phase_chips, float code_phase_step_chips,
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
    nb = read(d_device_descriptor, &irq_count, sizeof(irq_count));
    if (nb != sizeof(irq_count))
        {
            printf("Tracking_module Read failed to retrieve 4 bytes!\n");
            printf("Tracking_module Interrupt number %d\n", irq_count);
        }
    fpga_multicorrelator_8sc::read_tracking_gps_results();
}

fpga_multicorrelator_8sc::fpga_multicorrelator_8sc(int n_correlators,
        std::string device_name, unsigned int device_base, int *ca_codes, unsigned int code_length)
{
    d_n_correlators = n_correlators;
    d_device_name = device_name;
    d_device_base = device_base;
    d_device_descriptor = 0;
    d_map_base = nullptr;

    // instantiate variable length vectors
    d_initial_index = static_cast<unsigned*>(volk_gnsssdr_malloc(
            n_correlators * sizeof(unsigned), volk_gnsssdr_get_alignment()));
    d_initial_interp_counter = static_cast<unsigned*>(volk_gnsssdr_malloc(
            n_correlators * sizeof(unsigned), volk_gnsssdr_get_alignment()));

    //d_local_code_in = nullptr;
    d_shifts_chips = nullptr;
    d_corr_out = nullptr;
    d_code_length_chips = 0;
    d_rem_code_phase_chips = 0;
    d_code_phase_step_chips = 0;
    d_rem_carrier_phase_in_rad = 0;
    d_phase_step_rad = 0;
    d_rem_carr_phase_rad_int = 0;
    d_phase_step_rad_int = 0;
    d_initial_sample_counter = 0;
    d_channel = 0;
    d_correlator_length_samples = 0,
    d_code_length = code_length;
    
    // pre-compute all the codes
//    d_ca_codes = static_cast<int*>(volk_gnsssdr_malloc(static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS*NUM_PRNs) * sizeof(int), volk_gnsssdr_get_alignment()));
//    for (unsigned int PRN = 1; PRN <= NUM_PRNs; PRN++)
//    {
//		gps_l1_ca_code_gen_int(&d_ca_codes[(int(GPS_L1_CA_CODE_LENGTH_CHIPS)) * (PRN - 1)], PRN, 0);
//    }
    d_ca_codes = ca_codes;
    DLOG(INFO) << "TRACKING FPGA CLASS CREATED";
    
}


fpga_multicorrelator_8sc::~fpga_multicorrelator_8sc()
{
	delete[] d_ca_codes;
    close_device();
}


bool fpga_multicorrelator_8sc::free()
{
    // unlock the channel
    fpga_multicorrelator_8sc::unlock_channel(); 

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
    char device_io_name[MAX_LENGTH_DEVICEIO_NAME]; // driver io name
    d_channel = channel;

    // open the device corresponding to the assigned channel
    std::string mergedname;
    std::stringstream devicebasetemp;
    int numdevice = d_device_base + d_channel;
    devicebasetemp << numdevice;
    mergedname = d_device_name + devicebasetemp.str();
    strcpy(device_io_name, mergedname.c_str());
    if ((d_device_descriptor = open(device_io_name, O_RDWR | O_SYNC)) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio" << device_io_name;
        }
    d_map_base = reinterpret_cast<volatile unsigned *>(mmap(NULL, PAGE_SIZE,
            PROT_READ | PROT_WRITE, MAP_SHARED, d_device_descriptor, 0));

    if (d_map_base == reinterpret_cast<void*>(-1))
        {
            LOG(WARNING) << "Cannot map the FPGA tracking module "
                    << d_channel << "into user memory";
        }

    // sanity check : check test register
    unsigned writeval = TEST_REGISTER_TRACK_WRITEVAL;
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


unsigned fpga_multicorrelator_8sc::fpga_acquisition_test_register(
        unsigned writeval)
{
    unsigned readval;
    // write value to test register
    d_map_base[15] = writeval;
    // read value from test register
    readval = d_map_base[15];
    // return read value
    return readval;
}


void fpga_multicorrelator_8sc::fpga_configure_tracking_gps_local_code(int PRN)
{
    int k, s;
    unsigned code_chip;
    unsigned select_fpga_correlator;
    select_fpga_correlator = 0;

    for (s = 0; s < d_n_correlators; s++)
        {
            d_map_base[11] = LOCAL_CODE_FPGA_CLEAR_ADDRESS_COUNTER;
            for (k = 0; k < d_code_length_chips; k++)
                {
                    //if (d_local_code_in[k] == 1)
                    if (d_ca_codes[((int(d_code_length)) * (PRN - 1)) + k] == 1)
                        {
                            code_chip = 1;
                        }
                    else
                        {
                            code_chip = 0;
                        }
                    // copy the local code to the FPGA memory one by one
                    d_map_base[11] = LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY
                            | code_chip | select_fpga_correlator;
                }
            select_fpga_correlator = select_fpga_correlator
                    + LOCAL_CODE_FPGA_CORRELATOR_SELECT_COUNT;
        }
}


void fpga_multicorrelator_8sc::fpga_compute_code_shift_parameters(void)
{
    float temp_calculation;
    int i;

    for (i = 0; i < d_n_correlators; i++)
        {
            temp_calculation = floor(
                    d_shifts_chips[i] - d_rem_code_phase_chips);
                                        
            if (temp_calculation < 0)
                {
                    temp_calculation = temp_calculation + d_code_length_chips; // % operator does not work as in Matlab with negative numbers
                }
            d_initial_index[i] = static_cast<unsigned>( (static_cast<int>(temp_calculation)) % d_code_length_chips);
            temp_calculation = fmod(d_shifts_chips[i] - d_rem_code_phase_chips,
                    1.0);                    
            if (temp_calculation < 0)
                {
                    temp_calculation = temp_calculation + 1.0; // fmod operator does not work as in Matlab with negative numbers
                }
            d_initial_interp_counter[i] = static_cast<unsigned>( floor( MAX_CODE_RESAMPLER_COUNTER * temp_calculation));
        }
}


void fpga_multicorrelator_8sc::fpga_configure_code_parameters_in_fpga(void)
{
    int i;
    for (i = 0; i < d_n_correlators; i++)
        {
            d_map_base[1 + i] = d_initial_index[i];
            d_map_base[1 + d_n_correlators + i] = d_initial_interp_counter[i];
        }
    d_map_base[8] = d_code_length_chips - 1; // number of samples - 1
}


void fpga_multicorrelator_8sc::fpga_compute_signal_parameters_in_fpga(void)
{
    float d_rem_carrier_phase_in_rad_temp;

    d_code_phase_step_chips_num = static_cast<unsigned>( roundf(MAX_CODE_RESAMPLER_COUNTER * d_code_phase_step_chips));
    if (d_rem_carrier_phase_in_rad > M_PI)
        {
            d_rem_carrier_phase_in_rad_temp = -2 * M_PI
                    + d_rem_carrier_phase_in_rad;
        }
    else if (d_rem_carrier_phase_in_rad < -M_PI)
        {
            d_rem_carrier_phase_in_rad_temp = 2 * M_PI
                    + d_rem_carrier_phase_in_rad;
        }
    else
        {
            d_rem_carrier_phase_in_rad_temp = d_rem_carrier_phase_in_rad;
        }
    d_rem_carr_phase_rad_int = static_cast<int>( roundf(
            (fabs(d_rem_carrier_phase_in_rad_temp) / M_PI)
                    * pow(2, PHASE_CARR_NBITS_FRAC)));
    if (d_rem_carrier_phase_in_rad_temp < 0)
        {
            d_rem_carr_phase_rad_int = -d_rem_carr_phase_rad_int;
        }
    d_phase_step_rad_int = static_cast<int>( roundf(
            (fabs(d_phase_step_rad) / M_PI) * pow(2, PHASE_CARR_NBITS_FRAC))); // the FPGA accepts a range for the phase step between -pi and +pi

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
}


void fpga_multicorrelator_8sc::fpga_launch_multicorrelator_fpga(void)
{
    // enable interrupts
    int reenable = 1;
    write(d_device_descriptor, reinterpret_cast<void*>(&reenable), sizeof(int));

	// writing 1 to reg 14 launches the tracking
    d_map_base[14] = 1; 
}


void fpga_multicorrelator_8sc::read_tracking_gps_results(void)
{
    int readval_real;
    int readval_imag;
    int k;

    for (k = 0; k < d_n_correlators; k++)
        {
            readval_real = d_map_base[1 + k];
            if (readval_real >= 1048576) // 0x100000 (21 bits two's complement)
                {
                    readval_real = -2097152 + readval_real;
                }

            readval_imag = d_map_base[1 + d_n_correlators + k];
            if (readval_imag >= 1048576) // 0x100000 (21 bits two's complement)
                {
                    readval_imag = -2097152 + readval_imag;
                }
            d_corr_out[k] = gr_complex(readval_real,readval_imag);
        }
}


void fpga_multicorrelator_8sc::unlock_channel(void)
{
    // unlock the channel to let the next samples go through
    d_map_base[12] = 1; // unlock the channel
}

void fpga_multicorrelator_8sc::close_device()
{
    unsigned * aux = const_cast<unsigned*>(d_map_base);
    if (munmap(static_cast<void*>(aux), PAGE_SIZE) == -1)
        {
            printf("Failed to unmap memory uio\n");
        }
/*    else
        {
            printf("memory uio unmapped\n");
        } */
    close(d_device_descriptor);
}
    

void fpga_multicorrelator_8sc::lock_channel(void)
{
    // lock the channel for processing
    d_map_base[12] = 0; // lock the channel
}

void fpga_multicorrelator_8sc::read_sample_counters(int *sample_counter, int *secondary_sample_counter, int *counter_corr_0_in, int *counter_corr_0_out)
{
	*sample_counter = d_map_base[11];
	*secondary_sample_counter = d_map_base[8];
	*counter_corr_0_in = d_map_base[10];
	*counter_corr_0_out = d_map_base[9];
	
}

void fpga_multicorrelator_8sc::reset_multicorrelator(void)
{
	d_map_base[14] = 2; // writing a 2 to d_map_base[14] resets the multicorrelator   
}
