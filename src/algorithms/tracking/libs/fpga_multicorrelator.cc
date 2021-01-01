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

#include "fpga_multicorrelator.h"
#include <glog/logging.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <cmath>
#include <fcntl.h>  // for O_RDWR, O_RSYNC
#include <string>
#include <sys/mman.h>  // for PROT_READ, PROT_WRITE, MAP_SHARED
#include <utility>

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

// floating point math constants related to the parameters that are written in the FPGA
const float PHASE_CARR_MAX_DIV_PI = 683565275.5764316;  // 2^(31)/pi
const float TWO_PI = 6.283185307179586;

Fpga_Multicorrelator_8sc::Fpga_Multicorrelator_8sc(int32_t n_correlators,
    int32_t *ca_codes,
    int32_t *data_codes,
    uint32_t code_length_chips,
    bool track_pilot,
    uint32_t code_samples_per_chip)
{
    d_n_correlators = n_correlators;
    d_track_pilot = track_pilot;
    d_device_descriptor = 0;
    d_map_base = nullptr;

    // instantiate variable length vectors
    if (d_track_pilot)
        {
            d_initial_index.reserve(n_correlators + 1);
            d_initial_interp_counter.reserve(n_correlators + 1);
        }
    else
        {
            d_initial_index.reserve(n_correlators);
            d_initial_interp_counter.reserve(n_correlators);
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
    d_correlator_length_samples = 0;
    d_code_phase_step_chips_num = 0;
    d_code_length_chips = code_length_chips;
    d_ca_codes = ca_codes;
    d_data_codes = data_codes;
    d_code_samples_per_chip = code_samples_per_chip;
    d_code_length_samples = d_code_length_chips * d_code_samples_per_chip;
    d_secondary_code_enabled = false;

    DLOG(INFO) << "TRACKING FPGA CLASS CREATED";
}


Fpga_Multicorrelator_8sc::~Fpga_Multicorrelator_8sc()
{
    close_device();
}


uint64_t Fpga_Multicorrelator_8sc::read_sample_counter()
{
    uint64_t sample_counter_tmp;
    uint64_t sample_counter_msw_tmp;
    sample_counter_tmp = d_map_base[sample_counter_reg_addr_lsw];
    sample_counter_msw_tmp = d_map_base[sample_counter_reg_addr_msw];
    sample_counter_msw_tmp = sample_counter_msw_tmp << 32;
    sample_counter_tmp = sample_counter_tmp + sample_counter_msw_tmp;  // 2^32
    return sample_counter_tmp;
}


void Fpga_Multicorrelator_8sc::set_initial_sample(uint64_t samples_offset)
{
    d_initial_sample_counter = samples_offset;
    d_map_base[initial_counter_value_reg_addr_lsw] = (d_initial_sample_counter & 0xFFFFFFFF);
    d_map_base[initial_counter_value_reg_addr_msw] = (d_initial_sample_counter >> 32) & 0xFFFFFFFF;
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
            std::cout << "Tracking_module Read failed to retrieve 4 bytes!\n";
            std::cout << "Tracking_module Interrupt number " << irq_count << '\n';
        }

    // release secondary code indices, keep channel locked
    if (d_secondary_code_enabled == true)
        {
            d_map_base[drop_samples_reg_addr] = enable_secondary_code;  // keep secondary code enabled
        }
    else
        {
            d_map_base[drop_samples_reg_addr] = 0;  // block samples
        }
    Fpga_Multicorrelator_8sc::read_tracking_gps_results();
}


bool Fpga_Multicorrelator_8sc::free()
{
    // unlock the channel
    Fpga_Multicorrelator_8sc::unlock_channel();
    return true;
}


void Fpga_Multicorrelator_8sc::open_channel(std::string device_io_name, uint32_t channel)
{
    std::cout << "trk device_io_name = " << device_io_name << '\n';

    if ((d_device_descriptor = open(device_io_name.c_str(), O_RDWR | O_SYNC)) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio" << device_io_name;
            std::cout << "Cannot open deviceio" << device_io_name << '\n';
        }
    d_map_base = reinterpret_cast<volatile uint32_t *>(mmap(nullptr, page_size,
        PROT_READ | PROT_WRITE, MAP_SHARED, d_device_descriptor, 0));

    if (d_map_base == reinterpret_cast<void *>(-1))
        {
            LOG(WARNING) << "Cannot map the FPGA tracking module "
                         << channel << "into user memory";
            std::cout << "Cannot map deviceio" << device_io_name << '\n';
        }

    // sanity check: check test register
    uint32_t writeval = test_register_track_writeval;
    uint32_t readval;
    readval = Fpga_Multicorrelator_8sc::fpga_acquisition_test_register(writeval);
    if (writeval != readval)
        {
            LOG(WARNING) << "Test register sanity check failed";
            std::cout << "Tracking test register sanity check failed\n";
        }
    else
        {
            LOG(INFO) << "Test register sanity check success !";
        }
}


uint32_t Fpga_Multicorrelator_8sc::fpga_acquisition_test_register(uint32_t writeval)
{
    uint32_t readval = 0;
    // write value to test register
    d_map_base[test_reg_addr] = writeval;
    // read value from test register
    readval = d_map_base[test_reg_addr];
    // return read value
    return readval;
}


void Fpga_Multicorrelator_8sc::fpga_configure_tracking_gps_local_code(int32_t PRN)
{
    uint32_t k;
    d_map_base[prog_mems_addr] = local_code_fpga_clear_address_counter;
    for (k = 0; k < d_code_length_samples; k++)
        {
            d_map_base[prog_mems_addr] = d_ca_codes[(d_code_length_samples * (PRN - 1)) + k];
        }
    if (d_track_pilot)
        {
            d_map_base[prog_mems_addr] = local_code_fpga_clear_address_counter;
            for (k = 0; k < d_code_length_samples; k++)
                {
                    d_map_base[prog_mems_addr] = d_data_codes[(d_code_length_samples * (PRN - 1)) + k];
                }
        }

    d_map_base[code_length_minus_1_reg_addr] = d_code_length_samples - 1;  // number of samples - 1
}


void Fpga_Multicorrelator_8sc::fpga_compute_code_shift_parameters()
{
    float frac_part;   // decimal part
    int32_t dec_part;  // fractional part

    for (uint32_t i = 0; i < d_n_correlators; i++)
        {
            dec_part = std::floor(d_shifts_chips[i] - d_rem_code_phase_chips);

            if (dec_part < 0)
                {
                    dec_part = dec_part + d_code_length_samples;  // % operator does not work as in Matlab with negative numbers
                }
            d_initial_index[i] = dec_part;

            frac_part = fmod(d_shifts_chips[i] - d_rem_code_phase_chips, 1.0);
            if (frac_part < 0)
                {
                    frac_part = frac_part + 1.0F;  // fmod operator does not work as in Matlab with negative numbers
                }

            d_initial_interp_counter[i] = static_cast<uint32_t>(std::floor(max_code_resampler_counter * frac_part));
        }
    if (d_track_pilot)
        {
            dec_part = std::floor(d_prompt_data_shift[0] - d_rem_code_phase_chips);

            if (dec_part < 0)
                {
                    dec_part = dec_part + d_code_length_samples;  // % operator does not work as in Matlab with negative numbers
                }
            d_initial_index[d_n_correlators] = dec_part;

            frac_part = fmod(d_prompt_data_shift[0] - d_rem_code_phase_chips, 1.0);
            if (frac_part < 0)
                {
                    frac_part = frac_part + 1.0F;  // fmod operator does not work as in Matlab with negative numbers
                }
            d_initial_interp_counter[d_n_correlators] = static_cast<uint32_t>(std::floor(max_code_resampler_counter * frac_part));
        }
}


void Fpga_Multicorrelator_8sc::fpga_configure_code_parameters_in_fpga()
{
    for (uint32_t i = 0; i < d_n_correlators; i++)
        {
            d_map_base[initial_index_reg_base_addr + i] = d_initial_index[i];
            d_map_base[initial_interp_counter_reg_base_addr + i] = d_initial_interp_counter[i];
        }
    if (d_track_pilot)
        {
            d_map_base[initial_index_reg_base_addr + d_n_correlators] = d_initial_index[d_n_correlators];
            d_map_base[initial_interp_counter_reg_base_addr + d_n_correlators] = d_initial_interp_counter[d_n_correlators];
        }
}


void Fpga_Multicorrelator_8sc::fpga_compute_signal_parameters_in_fpga()
{
    float d_rem_carrier_phase_in_rad_temp;

    d_code_phase_step_chips_num = static_cast<uint32_t>(std::roundf(max_code_resampler_counter * d_code_phase_step_chips));
    d_code_phase_rate_step_chips_num = static_cast<uint32_t>(std::roundf(max_code_resampler_counter * d_code_phase_rate_step_chips));

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

    d_rem_carr_phase_rad_int = static_cast<int32_t>(std::roundf(d_rem_carrier_phase_in_rad_temp * PHASE_CARR_MAX_DIV_PI));
    d_phase_step_rad_int = static_cast<int32_t>(std::roundf(d_phase_step_rad * PHASE_CARR_MAX_DIV_PI));  // the FPGA accepts a range for the phase step between -pi and +pi
    d_carrier_phase_rate_step_rad_int = static_cast<int32_t>(std::roundf(d_carrier_phase_rate_step_rad * PHASE_CARR_MAX_DIV_PI));
}


void Fpga_Multicorrelator_8sc::fpga_configure_signal_parameters_in_fpga()
{
    d_map_base[code_phase_step_chips_num_reg_addr] = d_code_phase_step_chips_num;  // code phase step

    d_map_base[code_phase_step_chips_rate_reg_addr] = d_code_phase_rate_step_chips_num;  // code phase step rate

    d_map_base[nsamples_minus_1_reg_addr] = d_correlator_length_samples - 1;  // number of samples

    d_map_base[rem_carr_phase_rad_reg_addr] = d_rem_carr_phase_rad_int;  // initial nco phase

    d_map_base[phase_step_rad_reg_addr] = d_phase_step_rad_int;  // nco phase step

    d_map_base[phase_step_rate_reg_addr] = d_carrier_phase_rate_step_rad_int;  // nco phase step rate
}


void Fpga_Multicorrelator_8sc::fpga_launch_multicorrelator_fpga()
{
    // enable interrupts
    int32_t reenable = 1;
    ssize_t nbytes = TEMP_FAILURE_RETRY(write(d_device_descriptor, reinterpret_cast<void *>(&reenable), sizeof(int32_t)));
    if (nbytes != sizeof(int32_t))
        {
            std::cerr << "Error launching the FPGA multicorrelator\n";
        }
    // writing 1 to reg 14 launches the tracking
    d_map_base[start_flag_addr] = 1;
}


void Fpga_Multicorrelator_8sc::read_tracking_gps_results()
{
    int32_t readval_real;
    int32_t readval_imag;

    for (uint32_t k = 0; k < d_n_correlators; k++)
        {
            readval_real = d_map_base[result_reg_real_base_addr + k];
            readval_imag = d_map_base[result_reg_imag_base_addr + k];
            d_corr_out[k] = gr_complex(readval_real, readval_imag);
        }
    if (d_track_pilot)
        {
            readval_real = d_map_base[result_reg_real_base_addr + d_n_correlators];
            readval_imag = d_map_base[result_reg_imag_base_addr + d_n_correlators];
            d_Prompt_Data[0] = gr_complex(readval_real, readval_imag);
        }
}


void Fpga_Multicorrelator_8sc::unlock_channel()
{
    // unlock the channel to let the next samples go through
    d_map_base[drop_samples_reg_addr] = drop_samples;  // unlock the channel and disable secondary codes
    d_map_base[stop_tracking_reg_addr] = 1;            // set the tracking module back to idle
    d_secondary_code_enabled = false;
}


void Fpga_Multicorrelator_8sc::close_device()
{
    auto *aux = const_cast<uint32_t *>(d_map_base);
    if (munmap(static_cast<void *>(aux), page_size) == -1)
        {
            std::cout << "Failed to unmap memory uio\n";
        }
    close(d_device_descriptor);
}


void Fpga_Multicorrelator_8sc::lock_channel()
{
    // lock the channel for processing
    d_map_base[drop_samples_reg_addr] = 0;  // lock the channel
}


void Fpga_Multicorrelator_8sc::set_secondary_code_lengths(uint32_t secondary_code_0_length, uint32_t secondary_code_1_length)
{
    d_secondary_code_0_length = secondary_code_0_length;
    d_secondary_code_1_length = secondary_code_1_length;

    uint32_t secondary_code_length_0_minus_1 = d_secondary_code_0_length - 1;
    uint32_t secondary_code_length_1_minus_1 = d_secondary_code_1_length - 1;

    d_map_base[secondary_code_lengths_reg_addr] = secondary_code_length_1_minus_1 * 256 + secondary_code_length_0_minus_1;
}


void Fpga_Multicorrelator_8sc::update_prn_code_length(uint32_t first_prn_length, uint32_t next_prn_length)
{
    d_map_base[first_prn_length_minus_1_reg_addr] = first_prn_length - 1;
    d_map_base[next_prn_length_minus_1_reg_addr] = next_prn_length - 1;
}


void Fpga_Multicorrelator_8sc::initialize_secondary_code(uint32_t secondary_code, std::string *secondary_code_string)
{
    uint32_t secondary_code_length;
    uint32_t reg_addr;
    if (secondary_code == 0)
        {
            secondary_code_length = d_secondary_code_0_length;
            reg_addr = prog_secondary_code_0_data_reg_addr;
        }
    else
        {
            secondary_code_length = d_secondary_code_1_length;
            reg_addr = prog_secondary_code_1_data_reg_addr;
        }
    Fpga_Multicorrelator_8sc::write_secondary_code(secondary_code_length, secondary_code_string, reg_addr);
}


void Fpga_Multicorrelator_8sc::write_secondary_code(uint32_t secondary_code_length, std::string *secondary_code_string, uint32_t reg_addr)
{
    uint32_t num_words = std::ceil(static_cast<float>(secondary_code_length) / secondary_code_word_size);
    uint32_t last_word_size = secondary_code_length % secondary_code_word_size;

    if (last_word_size == 0)
        {
            last_word_size = secondary_code_word_size;
        }

    uint32_t write_val = 0U;
    uint32_t pow_k;
    uint32_t mem_addr;
    if (num_words > 1)
        {
            for (mem_addr = 0; mem_addr < num_words - 1; mem_addr++)
                {
                    write_val = 0U;
                    pow_k = 1;
                    for (unsigned int k = 0; k < secondary_code_word_size; k++)
                        {
                            std::string string_tmp(1, (*secondary_code_string)[mem_addr * secondary_code_word_size + k]);
                            write_val = write_val | std::stoi(string_tmp) * pow_k;
                            pow_k = pow_k * 2;
                        }
                    write_val = write_val | mem_addr * secondary_code_addr_bits | secondary_code_wr_strobe;
                    d_map_base[reg_addr] = write_val;
                }
        }
    write_val = 0U;
    pow_k = 1;
    mem_addr = num_words - 1;

    for (unsigned int k = 0; k < last_word_size; k++)
        {
            std::string string_tmp(1, (*secondary_code_string)[mem_addr * secondary_code_word_size + k]);
            write_val = write_val | std::stoi(string_tmp) * pow_k;
            pow_k = pow_k * 2;
        }

    write_val = write_val | (mem_addr * secondary_code_addr_bits) | (secondary_code_wr_strobe);
    d_map_base[reg_addr] = write_val;
}


void Fpga_Multicorrelator_8sc::enable_secondary_codes()
{
    d_map_base[drop_samples_reg_addr] = init_secondary_code_addresses | enable_secondary_code;  // enable secondary codes and clear secondary code indices
    d_secondary_code_enabled = true;
}


void Fpga_Multicorrelator_8sc::disable_secondary_codes()
{
    // this function is to be called before starting the tracking process in order to disable the secondary codes by default
    d_map_base[drop_samples_reg_addr] = drop_samples;
}
