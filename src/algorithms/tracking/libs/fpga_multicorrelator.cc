/*!
 * \file fpga_multicorrelator.cc
 * \brief FPGA vector correlator class
 * \authors <ul>
 *    <li> Marc Majoral, 2019. mmajoral(at)cttc.cat
 *    <li> Javier Arribas, 2019. jarribas(at)cttc.es
 * </ul>
 *
 * Class that controls and executes a highly optimized vector correlator
 * class in the FPGA
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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
#include <glog/logging.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <cmath>
#include <cstdio>
#include <fcntl.h>  // for O_RDWR, O_RSYNC
#include <string>
#include <sys/mman.h>  // for PROT_READ, PROT_WRITE, MAP_SHARED
#include <utility>

// FPGA register access constants
#define PAGE_SIZE 0x10000
#define MAX_LENGTH_DEVICEIO_NAME 50
#define CODE_RESAMPLER_NUM_BITS_PRECISION 20
#define CODE_PHASE_STEP_CHIPS_NUM_NBITS CODE_RESAMPLER_NUM_BITS_PRECISION
#define pwrtwo(x) (1 << (x))
#define MAX_CODE_RESAMPLER_COUNTER pwrtwo(CODE_PHASE_STEP_CHIPS_NUM_NBITS)  // 2^CODE_PHASE_STEP_CHIPS_NUM_NBITS
#define PHASE_CARR_MAX 2147483648                                           // 2^(31) The phase is represented as a 32-bit vector in 1.31 format
#define PHASE_CARR_MAX_div_PI 683565275.5764316                             // 2^(31)/pi
#define TWO_PI 6.283185307179586
#define LOCAL_CODE_FPGA_CORRELATOR_SELECT_COUNT 0x20000000
#define LOCAL_CODE_FPGA_CLEAR_ADDRESS_COUNTER 0x10000000
#define LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY 0x0C000000
#define TEST_REGISTER_TRACK_WRITEVAL 0x55AA
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

Fpga_Multicorrelator_8sc::Fpga_Multicorrelator_8sc(int32_t n_correlators,
    std::string device_name, uint32_t device_base, int32_t *ca_codes, int32_t *data_codes, uint32_t code_length_chips, bool track_pilot,
    uint32_t multicorr_type, uint32_t code_samples_per_chip)
{
    d_n_correlators = n_correlators;
    d_device_name = std::move(device_name);
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
    d_Prompt_Data = nullptr;
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
    d_correlator_length_samples = 0;
    d_code_phase_step_chips_num = 0;
    d_code_length_chips = code_length_chips;
    d_ca_codes = ca_codes;
    d_data_codes = data_codes;
    d_multicorr_type = multicorr_type;
    d_code_samples_per_chip = code_samples_per_chip;
    d_code_length_samples = d_code_length_chips * d_code_samples_per_chip;

    DLOG(INFO) << "TRACKING FPGA CLASS CREATED";
}


Fpga_Multicorrelator_8sc::~Fpga_Multicorrelator_8sc()
{
    close_device();
    if (d_initial_index != nullptr)
        {
            volk_gnsssdr_free(d_initial_index);
        }
    if (d_initial_interp_counter != nullptr)
        {
            volk_gnsssdr_free(d_initial_interp_counter);
        }
}


uint64_t Fpga_Multicorrelator_8sc::read_sample_counter()
{
    uint64_t sample_counter_tmp, sample_counter_msw_tmp;
    sample_counter_tmp = d_map_base[SAMPLE_COUNTER_REG_ADDR_LSW];
    sample_counter_msw_tmp = d_map_base[SAMPLE_COUNTER_REG_ADDR_MSW];
    sample_counter_msw_tmp = sample_counter_msw_tmp << 32;
    sample_counter_tmp = sample_counter_tmp + sample_counter_msw_tmp;  // 2^32
    return sample_counter_tmp;
}


void Fpga_Multicorrelator_8sc::set_initial_sample(uint64_t samples_offset)
{
    d_initial_sample_counter = samples_offset;
    d_map_base[INITIAL_COUNTER_VALUE_REG_ADDR_LSW] = (d_initial_sample_counter & 0xFFFFFFFF);
    d_map_base[INITIAL_COUNTER_VALUE_REG_ADDR_MSW] = (d_initial_sample_counter >> 32) & 0xFFFFFFFF;
}


void Fpga_Multicorrelator_8sc::set_local_code_and_taps(float *shifts_chips, float *prompt_data_shift, int32_t PRN)
{
    d_shifts_chips = shifts_chips;
    d_prompt_data_shift = prompt_data_shift;
    Fpga_Multicorrelator_8sc::fpga_configure_tracking_gps_local_code(PRN);
}


void Fpga_Multicorrelator_8sc::set_output_vectors(gr_complex *corr_out, gr_complex *Prompt_Data)
{
    d_corr_out = corr_out;
    d_Prompt_Data = Prompt_Data;
}


void Fpga_Multicorrelator_8sc::update_local_code()
{
    Fpga_Multicorrelator_8sc::fpga_compute_code_shift_parameters();
    Fpga_Multicorrelator_8sc::fpga_configure_code_parameters_in_fpga();
}


void Fpga_Multicorrelator_8sc::Carrier_wipeoff_multicorrelator_resampler(
    float rem_carrier_phase_in_rad,                             // nco phase initial position
    float phase_step_rad,                                       // nco phase step
    float carrier_phase_rate_step_rad __attribute__((unused)),  // nco phase step rate change
    float rem_code_phase_chips,                                 // code resampler initial position
    float code_phase_step_chips __attribute__((unused)),        // code resampler step
    float code_phase_rate_step_chips __attribute__((unused)),   // code resampler step rate
    int32_t signal_length_samples)                              // number of samples
{
    d_rem_carrier_phase_in_rad = rem_carrier_phase_in_rad;        // nco phase initial position
    d_phase_step_rad = phase_step_rad;                            // nco phase step
    d_carrier_phase_rate_step_rad = carrier_phase_rate_step_rad;  // nco phase step rate
    d_rem_code_phase_chips = rem_code_phase_chips;                // code resampler initial position
    d_code_phase_step_chips = code_phase_step_chips;              // code resampler step
    d_code_phase_rate_step_chips = code_phase_rate_step_chips;    // code resampler step rate
    d_correlator_length_samples = signal_length_samples;          // number of samples
    Fpga_Multicorrelator_8sc::update_local_code();
    Fpga_Multicorrelator_8sc::fpga_compute_signal_parameters_in_fpga();
    Fpga_Multicorrelator_8sc::fpga_configure_signal_parameters_in_fpga();
    Fpga_Multicorrelator_8sc::fpga_launch_multicorrelator_fpga();
    int32_t irq_count;
    ssize_t nb;
    nb = read(d_device_descriptor, &irq_count, sizeof(irq_count));
    if (nb != sizeof(irq_count))
        {
            std::cout << "Tracking_module Read failed to retrieve 4 bytes!" << std::endl;
            std::cout << "Tracking_module Interrupt number " << irq_count << std::endl;
        }
    Fpga_Multicorrelator_8sc::read_tracking_gps_results();
}


bool Fpga_Multicorrelator_8sc::free()
{
    // unlock the channel
    Fpga_Multicorrelator_8sc::unlock_channel();

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


void Fpga_Multicorrelator_8sc::set_channel(uint32_t channel)
{
    char device_io_name[MAX_LENGTH_DEVICEIO_NAME];  // driver io name
    d_channel = channel;

    // open the device corresponding to the assigned channel
    std::string mergedname;
    std::stringstream devicebasetemp;
    int32_t numdevice = d_device_base + d_channel;
    devicebasetemp << numdevice;
    mergedname = d_device_name + devicebasetemp.str();

    if (mergedname.size() > MAX_LENGTH_DEVICEIO_NAME)
        {
            mergedname = mergedname.substr(0, MAX_LENGTH_DEVICEIO_NAME);
        }

    mergedname.copy(device_io_name, mergedname.size() + 1);
    device_io_name[mergedname.size()] = '\0';
    std::cout << "trk device_io_name = " << device_io_name << std::endl;

    if ((d_device_descriptor = open(device_io_name, O_RDWR | O_SYNC)) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio" << device_io_name;
            std::cout << "Cannot open deviceio" << device_io_name << std::endl;
        }
    d_map_base = reinterpret_cast<volatile uint32_t *>(mmap(nullptr, PAGE_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, d_device_descriptor, 0));

    if (d_map_base == reinterpret_cast<void *>(-1))
        {
            LOG(WARNING) << "Cannot map the FPGA tracking module "
                         << d_channel << "into user memory";
            std::cout << "Cannot map deviceio" << device_io_name << std::endl;
        }

    // sanity check: check test register
    uint32_t writeval = TEST_REGISTER_TRACK_WRITEVAL;
    uint32_t readval;
    readval = Fpga_Multicorrelator_8sc::fpga_acquisition_test_register(writeval);
    if (writeval != readval)
        {
            LOG(WARNING) << "Test register sanity check failed";
            std::cout << "Tracking test register sanity check failed" << std::endl;
        }
    else
        {
            LOG(INFO) << "Test register sanity check success !";
        }
}


uint32_t Fpga_Multicorrelator_8sc::fpga_acquisition_test_register(
    uint32_t writeval)
{
    uint32_t readval = 0;
    // write value to test register
    d_map_base[TEST_REG_ADDR] = writeval;
    // read value from test register
    readval = d_map_base[TEST_REG_ADDR];
    // return read value
    return readval;
}


void Fpga_Multicorrelator_8sc::fpga_configure_tracking_gps_local_code(int32_t PRN)
{
    uint32_t k;

    d_map_base[PROG_MEMS_ADDR] = LOCAL_CODE_FPGA_CLEAR_ADDRESS_COUNTER;
    for (k = 0; k < d_code_length_samples; k++)
        {
            d_map_base[PROG_MEMS_ADDR] = d_ca_codes[(d_code_length_samples * (PRN - 1)) + k];
        }
    if (d_track_pilot)
        {
            d_map_base[PROG_MEMS_ADDR] = LOCAL_CODE_FPGA_CLEAR_ADDRESS_COUNTER;
            for (k = 0; k < d_code_length_samples; k++)
                {
                    d_map_base[PROG_MEMS_ADDR] = d_data_codes[(d_code_length_samples * (PRN - 1)) + k];
                }
        }

    d_map_base[CODE_LENGTH_MINUS_1_REG_ADDR] = (d_code_length_samples)-1;  // number of samples - 1
}


void Fpga_Multicorrelator_8sc::fpga_compute_code_shift_parameters(void)
{
    float frac_part;   // decimal part
    int32_t dec_part;  // fractional part

    for (uint32_t i = 0; i < d_n_correlators; i++)
        {
            dec_part = floor(d_shifts_chips[i] - d_rem_code_phase_chips);

            if (dec_part < 0)
                {
                    dec_part = dec_part + d_code_length_samples;  // % operator does not work as in Matlab with negative numbers
                }
            d_initial_index[i] = dec_part;


            frac_part = fmod(d_shifts_chips[i] - d_rem_code_phase_chips, 1.0);
            if (frac_part < 0)
                {
                    frac_part = frac_part + 1.0;  // fmod operator does not work as in Matlab with negative numbers
                }

            d_initial_interp_counter[i] = static_cast<uint32_t>(floor(MAX_CODE_RESAMPLER_COUNTER * frac_part));
        }
    if (d_track_pilot)
        {
            dec_part = floor(d_prompt_data_shift[0] - d_rem_code_phase_chips);

            if (dec_part < 0)
                {
                    dec_part = dec_part + d_code_length_samples;  // % operator does not work as in Matlab with negative numbers
                }
            d_initial_index[d_n_correlators] = dec_part;

            frac_part = fmod(d_prompt_data_shift[0] - d_rem_code_phase_chips, 1.0);
            if (frac_part < 0)
                {
                    frac_part = frac_part + 1.0;  // fmod operator does not work as in Matlab with negative numbers
                }
            d_initial_interp_counter[d_n_correlators] = static_cast<uint32_t>(floor(MAX_CODE_RESAMPLER_COUNTER * frac_part));
        }
}


void Fpga_Multicorrelator_8sc::fpga_configure_code_parameters_in_fpga(void)
{
    for (uint32_t i = 0; i < d_n_correlators; i++)
        {
            d_map_base[INITIAL_INDEX_REG_BASE_ADDR + i] = d_initial_index[i];
            d_map_base[INITIAL_INTERP_COUNTER_REG_BASE_ADDR + i] = d_initial_interp_counter[i];
        }
    if (d_track_pilot)
        {
            d_map_base[INITIAL_INDEX_REG_BASE_ADDR + d_n_correlators] = d_initial_index[d_n_correlators];
            d_map_base[INITIAL_INTERP_COUNTER_REG_BASE_ADDR + d_n_correlators] = d_initial_interp_counter[d_n_correlators];
        }
}


void Fpga_Multicorrelator_8sc::fpga_compute_signal_parameters_in_fpga(void)
{
    float d_rem_carrier_phase_in_rad_temp;

    d_code_phase_step_chips_num = static_cast<uint32_t>(roundf(MAX_CODE_RESAMPLER_COUNTER * d_code_phase_step_chips));
    d_code_phase_rate_step_chips_num = static_cast<uint32_t>(roundf(MAX_CODE_RESAMPLER_COUNTER * d_code_phase_rate_step_chips));

    if (d_rem_carrier_phase_in_rad > M_PI)
        {
            d_rem_carrier_phase_in_rad_temp = -TWO_PI + d_rem_carrier_phase_in_rad;
        }
    else if (d_rem_carrier_phase_in_rad < -M_PI)
        {
            d_rem_carrier_phase_in_rad_temp = TWO_PI + d_rem_carrier_phase_in_rad;
        }
    else
        {
            d_rem_carrier_phase_in_rad_temp = d_rem_carrier_phase_in_rad;
        }

    d_rem_carr_phase_rad_int = static_cast<int32_t>(roundf((d_rem_carrier_phase_in_rad_temp)*PHASE_CARR_MAX_div_PI));
    d_phase_step_rad_int = static_cast<int32_t>(roundf((d_phase_step_rad)*PHASE_CARR_MAX_div_PI));  // the FPGA accepts a range for the phase step between -pi and +pi
    d_carrier_phase_rate_step_rad_int = static_cast<int32_t>(roundf((d_carrier_phase_rate_step_rad)*PHASE_CARR_MAX_div_PI));
}


void Fpga_Multicorrelator_8sc::fpga_configure_signal_parameters_in_fpga(void)
{
    d_map_base[CODE_PHASE_STEP_CHIPS_NUM_REG_ADDR] = d_code_phase_step_chips_num;  // code phase step

    d_map_base[CODE_PHASE_STEP_CHIPS_RATE] = d_code_phase_rate_step_chips_num;  // code phase step rate

    d_map_base[NSAMPLES_MINUS_1_REG_ADDR] = d_correlator_length_samples - 1;  // number of samples

    d_map_base[REM_CARR_PHASE_RAD_REG_ADDR] = d_rem_carr_phase_rad_int;  // initial nco phase

    d_map_base[PHASE_STEP_RAD_REG_ADDR] = d_phase_step_rad_int;  // nco phase step

    d_map_base[PHASE_STEP_RATE_REG_ADDR] = d_carrier_phase_rate_step_rad_int;  // nco phase step rate
}


void Fpga_Multicorrelator_8sc::fpga_launch_multicorrelator_fpga(void)
{
    // enable interrupts
    int32_t reenable = 1;
    ssize_t nbytes = TEMP_FAILURE_RETRY(write(d_device_descriptor, reinterpret_cast<void *>(&reenable), sizeof(int32_t)));
    if (nbytes != sizeof(int32_t))
        {
            std::cerr << "Error launching the FPGA multicorrelator" << std::endl;
        }
    // writing 1 to reg 14 launches the tracking
    d_map_base[START_FLAG_ADDR] = 1;
}


void Fpga_Multicorrelator_8sc::read_tracking_gps_results(void)
{
    int32_t readval_real;
    int32_t readval_imag;

    for (uint32_t k = 0; k < d_n_correlators; k++)
        {
            readval_real = d_map_base[RESULT_REG_REAL_BASE_ADDR + k];
            readval_imag = d_map_base[RESULT_REG_IMAG_BASE_ADDR + k];
            d_corr_out[k] = gr_complex(readval_real, readval_imag);
        }
    if (d_track_pilot)
        {
            readval_real = d_map_base[RESULT_REG_REAL_BASE_ADDR + d_n_correlators];
            readval_imag = d_map_base[RESULT_REG_IMAG_BASE_ADDR + d_n_correlators];
            d_Prompt_Data[0] = gr_complex(readval_real, readval_imag);
        }
}


void Fpga_Multicorrelator_8sc::unlock_channel(void)
{
    // unlock the channel to let the next samples go through
    d_map_base[DROP_SAMPLES_REG_ADDR] = 1;   // unlock the channel
    d_map_base[STOP_TRACKING_REG_ADDR] = 1;  // set the tracking module back to idle
}


void Fpga_Multicorrelator_8sc::close_device()
{
    auto *aux = const_cast<uint32_t *>(d_map_base);
    if (munmap(static_cast<void *>(aux), PAGE_SIZE) == -1)
        {
            std::cout << "Failed to unmap memory uio" << std::endl;
        }
    close(d_device_descriptor);
}


void Fpga_Multicorrelator_8sc::lock_channel(void)
{
    // lock the channel for processing
    d_map_base[DROP_SAMPLES_REG_ADDR] = 0;  // lock the channel
}
