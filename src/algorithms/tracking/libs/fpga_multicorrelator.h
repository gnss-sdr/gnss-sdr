/*!
 * \file fpga_multicorrelator_8sc.h
 * \brief High optimized FPGA vector correlator class for lv_16sc_t (short int32_t complex)
 * \authors <ul>
 * 			<li> Marc Majoral, 2017. mmajoral(at)cttc.cat
 *          <li> Javier Arribas, 2016. jarribas(at)cttc.es
 *          </ul>
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

#ifndef GNSS_SDR_FPGA_MULTICORRELATOR_8SC_H_
#define GNSS_SDR_FPGA_MULTICORRELATOR_8SC_H_

#include <gnuradio/block.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <cstdint>

#define MAX_LENGTH_DEVICEIO_NAME 50

/*!
 * \brief Class that implements carrier wipe-off and correlators.
 */
class fpga_multicorrelator_8sc
{
public:
    fpga_multicorrelator_8sc(int32_t n_correlators, std::string device_name,
        uint32_t device_base, int32_t *ca_codes, int32_t *data_codes, uint32_t code_length_chips, bool track_pilot, uint32_t multicorr_type, uint32_t code_samples_per_chip);
    ~fpga_multicorrelator_8sc();
    //bool set_output_vectors(gr_complex* corr_out);
    void set_output_vectors(gr_complex *corr_out, gr_complex *Prompt_Data);
    //    bool set_local_code_and_taps(
    //            int32_t code_length_chips, const int* local_code_in,
    //            float *shifts_chips, int32_t PRN);
    //bool set_local_code_and_taps(
    void set_local_code_and_taps(
        //            int32_t code_length_chips,
        float *shifts_chips, float *prompt_data_shift, int32_t PRN);
    //bool set_output_vectors(lv_16sc_t* corr_out);
    void update_local_code(float rem_code_phase_chips);
    //bool Carrier_wipeoff_multicorrelator_resampler(
    void Carrier_wipeoff_multicorrelator_resampler(
        float rem_carrier_phase_in_rad, float phase_step_rad,
        float rem_code_phase_chips, float code_phase_step_chips,
        int32_t signal_length_samples);
    bool free();
    void set_channel(uint32_t channel);
    void set_initial_sample(uint64_t samples_offset);
    uint64_t read_sample_counter();
    void lock_channel(void);
    void unlock_channel(void);
    //void read_sample_counters(int32_t *sample_counter, int32_t *secondary_sample_counter, int32_t *counter_corr_0_in, int32_t *counter_corr_0_out); // debug

private:
    //const int32_t *d_local_code_in;
    gr_complex *d_corr_out;
    gr_complex *d_Prompt_Data;
    float *d_shifts_chips;
    float *d_prompt_data_shift;
    int32_t d_code_length_chips;
    int32_t d_n_correlators;

    // data related to the hardware module and the driver
    int32_t d_device_descriptor;    // driver descriptor
    volatile uint32_t *d_map_base;  // driver memory map

    // configuration data received from the interface
    uint32_t d_channel;       // channel number
    uint32_t d_ncorrelators;  // number of correlators
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

    //uint32_t d_code_length; // nominal number of chips

    uint32_t d_code_samples_per_chip;
    bool d_track_pilot;

    uint32_t d_multicorr_type;

    // register addresses
    // write-only regs
    uint32_t d_CODE_PHASE_STEP_CHIPS_NUM_REG_ADDR;
    uint32_t d_INITIAL_INDEX_REG_BASE_ADDR;
    uint32_t d_INITIAL_INTERP_COUNTER_REG_BASE_ADDR;
    uint32_t d_NSAMPLES_MINUS_1_REG_ADDR;
    uint32_t d_CODE_LENGTH_MINUS_1_REG_ADDR;
    uint32_t d_REM_CARR_PHASE_RAD_REG_ADDR;
    uint32_t d_PHASE_STEP_RAD_REG_ADDR;
    uint32_t d_PROG_MEMS_ADDR;
    uint32_t d_DROP_SAMPLES_REG_ADDR;
    uint32_t d_INITIAL_COUNTER_VALUE_REG_ADDR_LSW;
    uint32_t d_INITIAL_COUNTER_VALUE_REG_ADDR_MSW;
    uint32_t d_START_FLAG_ADDR;
    // read-write regs
    uint32_t d_TEST_REG_ADDR;
    // read-only regs
    uint32_t d_RESULT_REG_REAL_BASE_ADDR;
    uint32_t d_RESULT_REG_IMAG_BASE_ADDR;
    uint32_t d_RESULT_REG_DATA_REAL_BASE_ADDR;
    uint32_t d_RESULT_REG_DATA_IMAG_BASE_ADDR;
    uint32_t d_SAMPLE_COUNTER_REG_ADDR_LSW;
    uint32_t d_SAMPLE_COUNTER_REG_ADDR_MSW;

    // private functions
    uint32_t fpga_acquisition_test_register(uint32_t writeval);
    void fpga_configure_tracking_gps_local_code(int32_t PRN);
    void fpga_compute_code_shift_parameters(void);
    void fpga_configure_code_parameters_in_fpga(void);
    void fpga_compute_signal_parameters_in_fpga(void);
    void fpga_configure_signal_parameters_in_fpga(void);
    void fpga_launch_multicorrelator_fpga(void);
    void read_tracking_gps_results(void);
    //void reset_multicorrelator(void);
    void close_device(void);

    uint32_t d_result_SAT_value;

    int32_t debug_max_readval_real[5] = {0, 0, 0, 0, 0};
    int32_t debug_max_readval_imag[5] = {0, 0, 0, 0, 0};

    int32_t debug_max_readval_real_after_check[5] = {0, 0, 0, 0, 0};
    int32_t debug_max_readval_imag_after_check[5] = {0, 0, 0, 0, 0};
    int32_t printcounter = 0;
};

#endif /* GNSS_SDR_FPGA_MULTICORRELATOR_H_ */
