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
#define MAX_CODE_RESAMPLER_COUNTER pwrtwo(CODE_PHASE_STEP_CHIPS_NUM_NBITS)  // 2^CODE_PHASE_STEP_CHIPS_NUM_NBITS
#define PHASE_CARR_NBITS 32
#define PHASE_CARR_NBITS_INT 1
#define PHASE_CARR_NBITS_FRAC PHASE_CARR_NBITS - PHASE_CARR_NBITS_INT
#define LOCAL_CODE_FPGA_CORRELATOR_SELECT_COUNT 0x20000000
#define LOCAL_CODE_FPGA_CLEAR_ADDRESS_COUNTER 0x10000000
#define LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY 0x0C000000
#define TEST_REGISTER_TRACK_WRITEVAL 0x55AA

uint64_t fpga_multicorrelator_8sc::read_sample_counter()
{
    uint64_t sample_counter_tmp, sample_counter_msw_tmp;
    sample_counter_tmp = d_map_base[d_SAMPLE_COUNTER_REG_ADDR_LSW];
    sample_counter_msw_tmp = d_map_base[d_SAMPLE_COUNTER_REG_ADDR_MSW];
    sample_counter_msw_tmp = sample_counter_msw_tmp << 32;
    sample_counter_tmp = sample_counter_tmp + sample_counter_msw_tmp;  // 2^32
    //return d_map_base[d_SAMPLE_COUNTER_REG_ADDR];
    return sample_counter_tmp;
}

void fpga_multicorrelator_8sc::set_initial_sample(uint64_t samples_offset)
{
    d_initial_sample_counter = samples_offset;
    //printf("www writing d map base %d = d_initial_sample_counter = %d\n", d_INITIAL_COUNTER_VALUE_REG_ADDR, d_initial_sample_counter);
    d_map_base[d_INITIAL_COUNTER_VALUE_REG_ADDR_LSW] = (d_initial_sample_counter & 0xFFFFFFFF);
    d_map_base[d_INITIAL_COUNTER_VALUE_REG_ADDR_MSW] = (d_initial_sample_counter >> 32) & 0xFFFFFFFF;
}

//void fpga_multicorrelator_8sc::set_local_code_and_taps(int32_t code_length_chips,
//        float *shifts_chips, int32_t PRN)

void fpga_multicorrelator_8sc::set_local_code_and_taps(float *shifts_chips, float *prompt_data_shift, int32_t PRN)
{
    d_shifts_chips = shifts_chips;
    d_prompt_data_shift = prompt_data_shift;
    //d_code_length_chips = code_length_chips;
    fpga_multicorrelator_8sc::fpga_configure_tracking_gps_local_code(PRN);
}

void fpga_multicorrelator_8sc::set_output_vectors(gr_complex *corr_out, gr_complex *Prompt_Data)
{
    d_corr_out = corr_out;
    d_Prompt_Data = Prompt_Data;
}

void fpga_multicorrelator_8sc::update_local_code(float rem_code_phase_chips)
{
    d_rem_code_phase_chips = rem_code_phase_chips;
    //printf("uuuuu d_rem_code_phase_chips = %f\n", d_rem_code_phase_chips);
    fpga_multicorrelator_8sc::fpga_compute_code_shift_parameters();
    fpga_multicorrelator_8sc::fpga_configure_code_parameters_in_fpga();
}


void fpga_multicorrelator_8sc::Carrier_wipeoff_multicorrelator_resampler(
    float rem_carrier_phase_in_rad, float phase_step_rad,
    float rem_code_phase_chips, float code_phase_step_chips,
    int32_t signal_length_samples)
{
    update_local_code(rem_code_phase_chips);
    d_rem_carrier_phase_in_rad = rem_carrier_phase_in_rad;
    d_code_phase_step_chips = code_phase_step_chips;
    d_phase_step_rad = phase_step_rad;
    d_correlator_length_samples = signal_length_samples;
    fpga_multicorrelator_8sc::fpga_compute_signal_parameters_in_fpga();
    fpga_multicorrelator_8sc::fpga_configure_signal_parameters_in_fpga();
    fpga_multicorrelator_8sc::fpga_launch_multicorrelator_fpga();
    int32_t irq_count;
    ssize_t nb;
    //printf("$$$$$ waiting for interrupt ... \n");
    nb = read(d_device_descriptor, &irq_count, sizeof(irq_count));
    //printf("$$$$$ interrupt received ... \n");
    if (nb != sizeof(irq_count))
        {
            printf("Tracking_module Read failed to retrieve 4 bytes!\n");
            printf("Tracking_module Interrupt number %d\n", irq_count);
        }
    fpga_multicorrelator_8sc::read_tracking_gps_results();
}

fpga_multicorrelator_8sc::fpga_multicorrelator_8sc(int32_t n_correlators,
    std::string device_name, uint32_t device_base, int32_t *ca_codes, int32_t *data_codes, uint32_t code_length_chips, bool track_pilot,
    uint32_t multicorr_type, uint32_t code_samples_per_chip)
{
    //printf("tracking fpga class created\n");
    d_n_correlators = n_correlators;
    d_device_name = device_name;
    d_device_base = device_base;
    d_track_pilot = track_pilot;
    d_device_descriptor = 0;
    d_map_base = nullptr;

    // instantiate variable length vectors
    if (d_track_pilot)
        {
            d_initial_index = static_cast<uint32_t *>(volk_gnsssdr_malloc(
                (n_correlators + 1) * sizeof(uint32_t), volk_gnsssdr_get_alignment()));
            d_initial_interp_counter = static_cast<uint32_t *>(volk_gnsssdr_malloc(
                (n_correlators + 1) * sizeof(uint32_t), volk_gnsssdr_get_alignment()));
        }
    else
        {
            d_initial_index = static_cast<uint32_t *>(volk_gnsssdr_malloc(
                n_correlators * sizeof(uint32_t), volk_gnsssdr_get_alignment()));
            d_initial_interp_counter = static_cast<uint32_t *>(volk_gnsssdr_malloc(
                n_correlators * sizeof(uint32_t), volk_gnsssdr_get_alignment()));
        }
    d_shifts_chips = nullptr;
    d_prompt_data_shift = nullptr;
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
    //d_code_length = code_length;
        d_code_length_chips = code_length_chips;
    d_ca_codes = ca_codes;
    d_data_codes = data_codes;
    d_multicorr_type = multicorr_type;

    d_code_samples_per_chip = code_samples_per_chip;
    // set up register mapping

    // write-only registers
    d_CODE_PHASE_STEP_CHIPS_NUM_REG_ADDR = 0;
    d_INITIAL_INDEX_REG_BASE_ADDR = 1;
    //    if (d_multicorr_type == 0)
    //        {
    //            // multicorrelator with 3 correlators (16 registers only)
    //            d_INITIAL_INTERP_COUNTER_REG_BASE_ADDR = 4;
    //            d_NSAMPLES_MINUS_1_REG_ADDR = 7;
    //            d_CODE_LENGTH_MINUS_1_REG_ADDR = 8;
    //            d_REM_CARR_PHASE_RAD_REG_ADDR = 9;
    //            d_PHASE_STEP_RAD_REG_ADDR = 10;
    //            d_PROG_MEMS_ADDR = 11;
    //            d_DROP_SAMPLES_REG_ADDR = 12;
    //            d_INITIAL_COUNTER_VALUE_REG_ADDR = 13;
    //            d_START_FLAG_ADDR = 14;
    //        }
    //    else
    //        {
    // other types of multicorrelators (32 registers)
    d_INITIAL_INTERP_COUNTER_REG_BASE_ADDR = 7;
    d_NSAMPLES_MINUS_1_REG_ADDR = 13;
    d_CODE_LENGTH_MINUS_1_REG_ADDR = 14;
    d_REM_CARR_PHASE_RAD_REG_ADDR = 15;
    d_PHASE_STEP_RAD_REG_ADDR = 16;
    d_PROG_MEMS_ADDR = 17;
    d_DROP_SAMPLES_REG_ADDR = 18;
    d_INITIAL_COUNTER_VALUE_REG_ADDR_LSW = 19;
    d_INITIAL_COUNTER_VALUE_REG_ADDR_MSW = 20;
    d_START_FLAG_ADDR = 30;
    //        }

    //printf("d_n_correlators = %d\n", d_n_correlators);
    //printf("d_multicorr_type = %d\n", d_multicorr_type);
    // read-write registers
    //    if (d_multicorr_type == 0)
    //        {
    //            // multicorrelator with 3 correlators (16 registers only)
    //            d_TEST_REG_ADDR = 15;
    //        }
    //    else
    //        {
    // other types of multicorrelators (32 registers)
    d_TEST_REG_ADDR = 31;
    //       }

    // result 2's complement saturation value
    //    if (d_multicorr_type == 0)
    //        {
    //            // multicorrelator with 3 correlators (16 registers only)
    //            d_result_SAT_value = 1048576; // 21 bits 2's complement -> 2^20
    //        }
    //    else
    //        {
    //            // other types of multicorrelators (32 registers)
    //            d_result_SAT_value = 4194304; // 23 bits 2's complement -> 2^22
    //        }

    // read only registers
    d_RESULT_REG_REAL_BASE_ADDR = 1;
    //    if (d_multicorr_type == 0)
    //        {
    //            // multicorrelator with 3 correlators (16 registers only)
    //            d_RESULT_REG_IMAG_BASE_ADDR = 4;
    //            d_RESULT_REG_DATA_REAL_BASE_ADDR = 0; // no pilot tracking
    //            d_RESULT_REG_DATA_IMAG_BASE_ADDR = 0;
    //            d_SAMPLE_COUNTER_REG_ADDR = 7;
    //
    //        }
    //    else
    //        {
    // other types of multicorrelators (32 registers)
    d_RESULT_REG_IMAG_BASE_ADDR = 7;
    d_RESULT_REG_DATA_REAL_BASE_ADDR = 6;  // no pilot tracking
    d_RESULT_REG_DATA_IMAG_BASE_ADDR = 12;
    d_SAMPLE_COUNTER_REG_ADDR_LSW = 13;
    d_SAMPLE_COUNTER_REG_ADDR_MSW = 14;

    //        }

    //printf("d_SAMPLE_COUNTER_REG_ADDR = %d\n", d_SAMPLE_COUNTER_REG_ADDR);
    //printf("mmmmmmmmmmmmm d_n_correlators = %d\n", d_n_correlators);
    DLOG(INFO) << "TRACKING FPGA CLASS CREATED";
}


fpga_multicorrelator_8sc::~fpga_multicorrelator_8sc()
{
    //delete[] d_ca_codes;
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


void fpga_multicorrelator_8sc::set_channel(uint32_t channel)
{
    //printf("www trk set channel\n");
    char device_io_name[MAX_LENGTH_DEVICEIO_NAME];  // driver io name
    d_channel = channel;

    // open the device corresponding to the assigned channel
    std::string mergedname;
    std::stringstream devicebasetemp;
    int32_t numdevice = d_device_base + d_channel;
    devicebasetemp << numdevice;
    mergedname = d_device_name + devicebasetemp.str();
    strcpy(device_io_name, mergedname.c_str());

    //printf("ppps opening device %s\n", device_io_name);

    if ((d_device_descriptor = open(device_io_name, O_RDWR | O_SYNC)) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio" << device_io_name;
            std::cout << "Cannot open deviceio" << device_io_name << std::endl;

            //printf("error opening device\n");
        }
    //    else
    //        {
    //            std::cout << "deviceio" << device_io_name << " opened successfully" << std::endl;
    //
    //        }
    d_map_base = reinterpret_cast<volatile uint32_t *>(mmap(NULL, PAGE_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, d_device_descriptor, 0));

    if (d_map_base == reinterpret_cast<void *>(-1))
        {
            LOG(WARNING) << "Cannot map the FPGA tracking module "
                         << d_channel << "into user memory";
            std::cout << "Cannot map deviceio" << device_io_name << std::endl;
            //printf("error mapping registers\n");
        }
    //    else
    //        {
    //            std::cout << "deviceio" << device_io_name << "mapped successfully" << std::endl;
    //        }
    //    else
    //        {
    //            printf("trk mapping registers succes\n"); // this is for debug -- remove !
    //        }

    // sanity check : check test register
    uint32_t writeval = TEST_REGISTER_TRACK_WRITEVAL;
    uint32_t readval;
    readval = fpga_multicorrelator_8sc::fpga_acquisition_test_register(writeval);
    if (writeval != readval)
        {
            LOG(WARNING) << "Test register sanity check failed";
            printf("tracking test register sanity check failed\n");

            //printf("lslslls test sanity check reg failure\n");
        }
    else
        {
            LOG(INFO) << "Test register sanity check success !";
            //printf("tracking test register sanity check success\n");
            //printf("lslslls test sanity check reg success\n");
        }
}


uint32_t fpga_multicorrelator_8sc::fpga_acquisition_test_register(
    uint32_t writeval)
{
    //printf("d_TEST_REG_ADDR = %d\n", d_TEST_REG_ADDR);

    uint32_t readval = 0;
    // write value to test register
    d_map_base[d_TEST_REG_ADDR] = writeval;
    // read value from test register
    readval = d_map_base[d_TEST_REG_ADDR];
    // return read value
    return readval;
}


void fpga_multicorrelator_8sc::fpga_configure_tracking_gps_local_code(int32_t PRN)
{
    uint32_t k;
    uint32_t code_chip;
    uint32_t select_pilot_corelator = LOCAL_CODE_FPGA_CORRELATOR_SELECT_COUNT;
    //    select_fpga_correlator = 0;

    //printf("kkk d_n_correlators = %x\n", d_n_correlators);
    //printf("kkk d_code_length_chips = %d\n", d_code_length_chips);
    //printf("programming mems d map base %d\n", d_PROG_MEMS_ADDR);

    //FILE *fp;
    //char str[80];
    //sprintf(str, "generated_code_PRN%d", PRN);
    //fp = fopen(str,"w");
    //    for (s = 0; s < d_n_correlators; s++)
    //        {

    //printf("kkk select_fpga_correlator = %x\n", select_fpga_correlator);

    d_map_base[d_PROG_MEMS_ADDR] = LOCAL_CODE_FPGA_CLEAR_ADDRESS_COUNTER;
    for (k = 0; k < d_code_length_chips * d_code_samples_per_chip; k++)
        {
            //if (d_local_code_in[k] == 1)
            //printf("kkk d_ca_codes %d = %d\n", k, d_ca_codes[((int(d_code_length)) * (PRN - 1)) + k]);
            //fprintf(fp, "%d\n", d_ca_codes[((int(d_code_length_chips)) * d_code_samples_per_chip * (PRN - 1)) + k]);
            if (d_ca_codes[((int(d_code_length_chips)) * d_code_samples_per_chip * (PRN - 1)) + k] == 1)
                {
                    code_chip = 1;
                }
            else
                {
                    code_chip = 0;
                }

            // copy the local code to the FPGA memory one by one
            d_map_base[d_PROG_MEMS_ADDR] = LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY | code_chip;  // | select_fpga_correlator;
        }
    //            select_fpga_correlator = select_fpga_correlator
    //                    + LOCAL_CODE_FPGA_CORRELATOR_SELECT_COUNT;
    //        }
    //fclose(fp);
    //printf("kkk d_track_pilot = %d\n", d_track_pilot);
    if (d_track_pilot)
        {
            //printf("kkk select_fpga_correlator = %x\n", select_fpga_correlator);

            d_map_base[d_PROG_MEMS_ADDR] = LOCAL_CODE_FPGA_CLEAR_ADDRESS_COUNTER;
            for (k = 0; k < d_code_length_chips * d_code_samples_per_chip; k++)
                {
                    //if (d_local_code_in[k] == 1)
                    if (d_data_codes[((int(d_code_length_chips)) * d_code_samples_per_chip * (PRN - 1)) + k] == 1)
                        {
                            code_chip = 1;
                        }
                    else
                        {
                            code_chip = 0;
                        }
                    //printf("%d %d | ", d_data_codes, code_chip);
                    // copy the local code to the FPGA memory one by one
                    d_map_base[d_PROG_MEMS_ADDR] = LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY | code_chip | select_pilot_corelator;
                }
        }
    //printf("\n");
}


void fpga_multicorrelator_8sc::fpga_compute_code_shift_parameters(void)
{
    float temp_calculation;
    int32_t i;

    //printf("ppp d_rem_code_phase_chips = %f\n", d_rem_code_phase_chips);
    for (i = 0; i < d_n_correlators; i++)
        {
            //printf("ppp d_shifts_chips %d = %f\n", i, d_shifts_chips[i]);
            //printf("ppp d_code_samples_per_chip = %d\n", d_code_samples_per_chip);
            temp_calculation = floor(
                d_shifts_chips[i] - d_rem_code_phase_chips);

            //printf("ppp d_rem_code_phase_chips = %f\n", d_rem_code_phase_chips);
            //printf("ppp temp calculation %d = %f ================================ \n", i, temp_calculation);
            if (temp_calculation < 0)
                {
                    temp_calculation = temp_calculation + (d_code_length_chips * d_code_samples_per_chip);  // % operator does not work as in Matlab with negative numbers
                }
            //printf("ppp d_rem_code_phase_chips = %f\n", d_rem_code_phase_chips);
            //printf("ppp temp calculation %d = %f ================================ \n", i, temp_calculation);
            d_initial_index[i] = static_cast<uint32_t>((static_cast<int32_t>(temp_calculation)) % (d_code_length_chips * d_code_samples_per_chip));
            //printf("ppp d_initial_index %d = %d\n", i, d_initial_index[i]);
            temp_calculation = fmod(d_shifts_chips[i] - d_rem_code_phase_chips,
                1.0);
            //printf("ppp fmod %d = fmod(%f, 1) = %f\n", i, d_shifts_chips[i] - d_rem_code_phase_chips, temp_calculation);
            if (temp_calculation < 0)
                {
                    temp_calculation = temp_calculation + 1.0;  // fmod operator does not work as in Matlab with negative numbers
                }

            d_initial_interp_counter[i] = static_cast<uint32_t>(floor(MAX_CODE_RESAMPLER_COUNTER * temp_calculation));
            //printf("ppp d_initial_interp_counter %d = %d\n", i, d_initial_interp_counter[i]);
            //printf("MAX_CODE_RESAMPLER_COUNTER = %d\n", MAX_CODE_RESAMPLER_COUNTER);
        }
    if (d_track_pilot)
        {
            //printf("tracking pilot !!!!!!!!!!!!!!!!\n");
            temp_calculation = floor(
                d_prompt_data_shift[0] - d_rem_code_phase_chips);

            if (temp_calculation < 0)
                {
                    temp_calculation = temp_calculation + (d_code_length_chips * d_code_samples_per_chip);  // % operator does not work as in Matlab with negative numbers
                }
            d_initial_index[d_n_correlators] = static_cast<uint32_t>((static_cast<int32_t>(temp_calculation)) % (d_code_length_chips * d_code_samples_per_chip));
            temp_calculation = fmod(d_prompt_data_shift[0] - d_rem_code_phase_chips,
                1.0);
            if (temp_calculation < 0)
                {
                    temp_calculation = temp_calculation + 1.0;  // fmod operator does not work as in Matlab with negative numbers
                }
            d_initial_interp_counter[d_n_correlators] = static_cast<uint32_t>(floor(MAX_CODE_RESAMPLER_COUNTER * temp_calculation));
        }
    //while(1);
}


void fpga_multicorrelator_8sc::fpga_configure_code_parameters_in_fpga(void)
{
    int32_t i;
    for (i = 0; i < d_n_correlators; i++)
        {
            //printf("www writing d map base %d = d_initial_index %d  = %d\n", d_INITIAL_INDEX_REG_BASE_ADDR + i, i, d_initial_index[i]);
            d_map_base[d_INITIAL_INDEX_REG_BASE_ADDR + i] = d_initial_index[i];
            //d_map_base[1 + d_n_correlators + i] = d_initial_interp_counter[i];
            //printf("www writing d map base %d = d_initial_interp_counter %d  = %d\n", d_INITIAL_INTERP_COUNTER_REG_BASE_ADDR + i, i, d_initial_interp_counter[i]);
            d_map_base[d_INITIAL_INTERP_COUNTER_REG_BASE_ADDR + i] = d_initial_interp_counter[i];
        }
    if (d_track_pilot)
        {
            //printf("www writing d map base %d = d_initial_index %d  = %d\n", d_INITIAL_INDEX_REG_BASE_ADDR + d_n_correlators, d_n_correlators, d_initial_index[d_n_correlators]);
            d_map_base[d_INITIAL_INDEX_REG_BASE_ADDR + d_n_correlators] = d_initial_index[d_n_correlators];
            //d_map_base[1 + d_n_correlators + i] = d_initial_interp_counter[i];
            //printf("www writing d map base %d = d_initial_interp_counter %d  = %d\n", d_INITIAL_INTERP_COUNTER_REG_BASE_ADDR + d_n_correlators, d_n_correlators, d_initial_interp_counter[d_n_correlators]);
            d_map_base[d_INITIAL_INTERP_COUNTER_REG_BASE_ADDR + d_n_correlators] = d_initial_interp_counter[d_n_correlators];
        }

    //printf("www writing d map base %d = d_code_length_chips*d_code_samples_per_chip - 1  = %d\n", d_CODE_LENGTH_MINUS_1_REG_ADDR, (d_code_length_chips*d_code_samples_per_chip) - 1);
    d_map_base[d_CODE_LENGTH_MINUS_1_REG_ADDR] = (d_code_length_chips * d_code_samples_per_chip) - 1;  // number of samples - 1
}


void fpga_multicorrelator_8sc::fpga_compute_signal_parameters_in_fpga(void)
{
    float d_rem_carrier_phase_in_rad_temp;

    d_code_phase_step_chips_num = static_cast<uint32_t>(roundf(MAX_CODE_RESAMPLER_COUNTER * d_code_phase_step_chips));
    if (d_code_phase_step_chips > 1.0)
        {
            printf("Warning : d_code_phase_step_chips = %f cannot be bigger than one\n", d_code_phase_step_chips);
        }

    //printf("d_rem_carrier_phase_in_rad = %f\n", d_rem_carrier_phase_in_rad);

    if (d_rem_carrier_phase_in_rad > M_PI)
        {
            d_rem_carrier_phase_in_rad_temp = -2 * M_PI + d_rem_carrier_phase_in_rad;
        }
    else if (d_rem_carrier_phase_in_rad < -M_PI)
        {
            d_rem_carrier_phase_in_rad_temp = 2 * M_PI + d_rem_carrier_phase_in_rad;
        }
    else
        {
            d_rem_carrier_phase_in_rad_temp = d_rem_carrier_phase_in_rad;
        }
    d_rem_carr_phase_rad_int = static_cast<int32_t>(roundf(
        (fabs(d_rem_carrier_phase_in_rad_temp) / M_PI) * pow(2, PHASE_CARR_NBITS_FRAC)));
    if (d_rem_carrier_phase_in_rad_temp < 0)
        {
            d_rem_carr_phase_rad_int = -d_rem_carr_phase_rad_int;
        }
    d_phase_step_rad_int = static_cast<int32_t>(roundf(
        (fabs(d_phase_step_rad) / M_PI) * pow(2, PHASE_CARR_NBITS_FRAC)));  // the FPGA accepts a range for the phase step between -pi and +pi

    //printf("d_phase_step_rad_int = %d\n", d_phase_step_rad_int);
    if (d_phase_step_rad < 0)
        {
            d_phase_step_rad_int = -d_phase_step_rad_int;
        }

    //printf("d_phase_step_rad_int = %d\n", d_phase_step_rad_int);
}


void fpga_multicorrelator_8sc::fpga_configure_signal_parameters_in_fpga(void)
{
    //printf("www d map base %d = d_code_phase_step_chips_num = %d\n", d_CODE_PHASE_STEP_CHIPS_NUM_REG_ADDR, d_code_phase_step_chips_num);
    d_map_base[d_CODE_PHASE_STEP_CHIPS_NUM_REG_ADDR] = d_code_phase_step_chips_num;

    //printf("www d map base %d = d_correlator_length_samples - 1 = %d\n", d_NSAMPLES_MINUS_1_REG_ADDR, d_correlator_length_samples - 1);
    d_map_base[d_NSAMPLES_MINUS_1_REG_ADDR] = d_correlator_length_samples - 1;

    //printf("www d map base %d = d_rem_carr_phase_rad_int = %d\n", d_REM_CARR_PHASE_RAD_REG_ADDR, d_rem_carr_phase_rad_int);
    d_map_base[d_REM_CARR_PHASE_RAD_REG_ADDR] = d_rem_carr_phase_rad_int;

    //printf("www d map base %d = d_phase_step_rad_int = %d\n", d_PHASE_STEP_RAD_REG_ADDR, d_phase_step_rad_int);
    d_map_base[d_PHASE_STEP_RAD_REG_ADDR] = d_phase_step_rad_int;
}


void fpga_multicorrelator_8sc::fpga_launch_multicorrelator_fpga(void)
{
    // enable interrupts
    int32_t reenable = 1;
    write(d_device_descriptor, reinterpret_cast<void *>(&reenable), sizeof(int32_t));

    // writing 1 to reg 14 launches the tracking
    //printf("www writing 1 to d map base %d = start flag\n", d_START_FLAG_ADDR);
    d_map_base[d_START_FLAG_ADDR] = 1;
    //while(1);
}


void fpga_multicorrelator_8sc::read_tracking_gps_results(void)
{
    int32_t readval_real;
    int32_t readval_imag;
    int32_t k;

    //printf("www reading trk results\n");
    for (k = 0; k < d_n_correlators; k++)
        {
            readval_real = d_map_base[d_RESULT_REG_REAL_BASE_ADDR + k];
            //printf("read real before checking d map base %d = %d\n", d_RESULT_REG_BASE_ADDR + k, readval_real);
            ////            if (readval_real > debug_max_readval_real[k])
            ////                {
            ////                    debug_max_readval_real[k] = readval_real;
            ////                }
            //            if (readval_real >= d_result_SAT_value) // 0x100000 (21 bits two's complement)
            //                {
            //                    readval_real = -2*d_result_SAT_value + readval_real;
            //                }
            ////            if (readval_real > debug_max_readval_real_after_check[k])
            ////                {
            ////                    debug_max_readval_real_after_check[k] = readval_real;
            ////                }
            //printf("read real d map base %d = %d\n", d_RESULT_REG_BASE_ADDR + k, readval_real);
            readval_imag = d_map_base[d_RESULT_REG_IMAG_BASE_ADDR + k];
            //printf("read imag before checking d map base %d = %d\n", d_RESULT_REG_BASE_ADDR + k, readval_imag);
            ////            if (readval_imag > debug_max_readval_imag[k])
            ////                {
            ////                    debug_max_readval_imag[k] = readval_imag;
            ////                }
            //
            //            if (readval_imag >= d_result_SAT_value) // 0x100000 (21 bits two's complement)
            //                {
            //                    readval_imag = -2*d_result_SAT_value + readval_imag;
            //                }
            ////            if (readval_imag > debug_max_readval_imag_after_check[k])
            ////                {
            ////                    debug_max_readval_imag_after_check[k] = readval_real;
            ////                }
            //printf("read imag d map base %d = %d\n", d_RESULT_REG_BASE_ADDR + k, readval_imag);
            d_corr_out[k] = gr_complex(readval_real, readval_imag);

            //      if (printcounter > 100)
            //          {
            //              printcounter = 0;
            //              for (int32_t ll=0;ll<d_n_correlators;ll++)
            //                  {
            //                      printf("debug_max_readval_real %d = %d\n", ll, debug_max_readval_real[ll]);
            //                      printf("debug_max_readval_imag %d = %d\n", ll, debug_max_readval_imag[ll]);
            //                      printf("debug_max_readval_real_%d after_check = %d\n", ll, debug_max_readval_real_after_check[ll]);
            //                      printf("debug_max_readval_imag_%d after_check = %d\n", ll, debug_max_readval_imag_after_check[ll]);
            //                  }
            //
            //                }
            //            else
            //                {
            //                    printcounter = printcounter + 1;
            //                }
        }
    if (d_track_pilot)
        {
            //printf("reading pilot !!!\n");
            readval_real = d_map_base[d_RESULT_REG_DATA_REAL_BASE_ADDR];
            //            if (readval_real >= d_result_SAT_value) // 0x100000 (21 bits two's complement)
            //                {
            //                    readval_real = -2*d_result_SAT_value + readval_real;
            //                }

            readval_imag = d_map_base[d_RESULT_REG_DATA_IMAG_BASE_ADDR];
            //            if (readval_imag >= d_result_SAT_value) // 0x100000 (21 bits two's complement)
            //                {
            //                    readval_imag = -2*d_result_SAT_value + readval_imag;
            //                }
            d_Prompt_Data[0] = gr_complex(readval_real, readval_imag);
        }
}


void fpga_multicorrelator_8sc::unlock_channel(void)
{
    // unlock the channel to let the next samples go through
    //printf("www writing 1 to d map base %d = drop samples\n", d_DROP_SAMPLES_REG_ADDR);
    d_map_base[d_DROP_SAMPLES_REG_ADDR] = 1;  // unlock the channel
}

void fpga_multicorrelator_8sc::close_device()
{
    uint32_t *aux = const_cast<uint32_t *>(d_map_base);
    if (munmap(static_cast<void *>(aux), PAGE_SIZE) == -1)
        {
            printf("Failed to unmap memory uio\n");
        }
    close(d_device_descriptor);
}


void fpga_multicorrelator_8sc::lock_channel(void)
{
    // lock the channel for processing
    //printf("www writing 0 to d map base %d = drop samples\n", d_DROP_SAMPLES_REG_ADDR);
    d_map_base[d_DROP_SAMPLES_REG_ADDR] = 0;  // lock the channel
}

//void fpga_multicorrelator_8sc::read_sample_counters(int32_t *sample_counter, int32_t *secondary_sample_counter, int32_t *counter_corr_0_in, int32_t *counter_corr_0_out)
//{
//	*sample_counter = d_map_base[11];
//	*secondary_sample_counter = d_map_base[8];
//	*counter_corr_0_in = d_map_base[10];
//	*counter_corr_0_out = d_map_base[9];
//
//}

//void fpga_multicorrelator_8sc::reset_multicorrelator(void)
//{
//	d_map_base[14] = 2; // writing a 2 to d_map_base[14] resets the multicorrelator
//}
