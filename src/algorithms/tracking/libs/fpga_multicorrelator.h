/*!
 * \file fpga_multicorrelator.h
 * \brief FPGA vector correlator class
 * \authors <ul>
 * 			<li> Marc Majoral, 2019. mmajoral(at)cttc.cat
 *          <li> Javier Arribas, 2019. jarribas(at)cttc.es
 *          </ul>
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_FPGA_MULTICORRELATOR_8SC_H_
#define GNSS_SDR_FPGA_MULTICORRELATOR_8SC_H_

#include <gnuradio/block.h>
#include <cstdint>

// FPGA register addresses

// write addresses
#define CODE_PHASE_STEP_CHIPS_NUM_REG_ADDR 0
#define INITIAL_INDEX_REG_BASE_ADDR 1
#define INITIAL_INTERP_COUNTER_REG_BASE_ADDR 7
#define NSAMPLES_MINUS_1_REG_ADDR 13
#define CODE_LENGTH_MINUS_1_REG_ADDR 14
#define REM_CARR_PHASE_RAD_REG_ADDR 15
#define PHASE_STEP_RAD_REG_ADDR 16
#define PROG_MEMS_ADDR 17
#define DROP_SAMPLES_REG_ADDR 18
#define INITIAL_COUNTER_VALUE_REG_ADDR_LSW 19
#define INITIAL_COUNTER_VALUE_REG_ADDR_MSW 20
#define STOP_TRACKING_REG_ADDR 23
#define START_FLAG_ADDR 30
// read-write addresses
#define TEST_REG_ADDR 31
// read addresses
#define RESULT_REG_REAL_BASE_ADDR 1
#define RESULT_REG_IMAG_BASE_ADDR 7
#define SAMPLE_COUNTER_REG_ADDR_LSW 13
#define SAMPLE_COUNTER_REG_ADDR_MSW 14


/*!
 * \brief Class that implements carrier wipe-off and correlators.
 */
class Fpga_Multicorrelator_8sc
{
public:
    Fpga_Multicorrelator_8sc(int32_t n_correlators, std::string device_name,
        uint32_t device_base, int32_t *ca_codes, int32_t *data_codes, uint32_t code_length_chips, bool track_pilot, uint32_t multicorr_type, uint32_t code_samples_per_chip);
    ~Fpga_Multicorrelator_8sc();
    void set_output_vectors(gr_complex *corr_out, gr_complex *Prompt_Data);
    void set_local_code_and_taps(
        float *shifts_chips, float *prompt_data_shift, int32_t PRN);
    void update_local_code();
    void Carrier_wipeoff_multicorrelator_resampler(
        float rem_carrier_phase_in_rad, float phase_step_rad,
        float carrier_phase_rate_step_rad,
        float rem_code_phase_chips, float code_phase_step_chips,
        float code_phase_rate_step_chips,
        int32_t signal_length_samples);
    bool free();
    void set_channel(uint32_t channel);
    void set_initial_sample(uint64_t samples_offset);
    uint64_t read_sample_counter();
    void lock_channel(void);
    void unlock_channel(void);

private:
    gr_complex *d_corr_out;
    gr_complex *d_Prompt_Data;
    float *d_shifts_chips;
    float *d_prompt_data_shift;
    uint32_t d_code_length_chips;
    uint32_t d_code_length_samples;
    uint32_t d_n_correlators;  // number of correlators

    // data related to the hardware module and the driver
    int32_t d_device_descriptor;    // driver descriptor
    volatile uint32_t *d_map_base;  // driver memory map

    // configuration data received from the interface
    uint32_t d_channel;  // channel number
    uint32_t d_correlator_length_samples;
    float d_rem_code_phase_chips;
    float d_code_phase_step_chips;
    float d_rem_carrier_phase_in_rad;
    float d_phase_step_rad;

    // configuration data computed in the format that the FPGA expects
    uint32_t *d_initial_index;
    uint32_t *d_initial_interp_counter;
    uint32_t d_code_phase_step_chips_num;
    int32_t d_rem_carr_phase_rad_int;
    int32_t d_phase_step_rad_int;
    uint64_t d_initial_sample_counter;

    // driver
    std::string d_device_name;
    uint32_t d_device_base;

    int32_t *d_ca_codes;
    int32_t *d_data_codes;

    uint32_t d_code_samples_per_chip;
    bool d_track_pilot;

    uint32_t d_multicorr_type;

    // private functions
    uint32_t fpga_acquisition_test_register(uint32_t writeval);
    void fpga_configure_tracking_gps_local_code(int32_t PRN);
    void fpga_compute_code_shift_parameters(void);
    void fpga_configure_code_parameters_in_fpga(void);
    void fpga_compute_signal_parameters_in_fpga(void);
    void fpga_configure_signal_parameters_in_fpga(void);
    void fpga_launch_multicorrelator_fpga(void);
    void read_tracking_gps_results(void);
    void close_device(void);
};

#endif /* GNSS_SDR_FPGA_MULTICORRELATOR_H_ */
