/*!
 * \file fpga_multicorrelator_8sc.h
 * \brief High optimized FPGA vector correlator class for lv_16sc_t (short int complex)
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

#include <volk_gnsssdr/volk_gnsssdr.h>


#define MAX_LENGTH_DEVICEIO_NAME 50

/*!
 * \brief Class that implements carrier wipe-off and correlators.
 */
class fpga_multicorrelator_8sc
{
public:
    fpga_multicorrelator_8sc();
    ~fpga_multicorrelator_8sc();
    bool init(int n_correlators);
    bool set_local_code_and_taps(int code_length_chips, const lv_16sc_t* local_code_in, float *shifts_chips);
    bool set_output_vectors(lv_16sc_t* corr_out);
    void update_local_code(float rem_code_phase_chips);
    bool Carrier_wipeoff_multicorrelator_resampler(float rem_carrier_phase_in_rad, float phase_step_rad, float rem_code_phase_chips, float code_phase_step_chips, int signal_length_samples);
    bool free();

    void set_channel(unsigned int channel);
    void set_initial_sample(int samples_offset);

private:
    const lv_16sc_t *d_local_code_in;
    lv_16sc_t *d_corr_out;
    float *d_shifts_chips;
    int d_code_length_chips;
    int d_n_correlators;

    // data related to the hardware module and the driver
    char d_device_io_name[MAX_LENGTH_DEVICEIO_NAME];  // driver io name
    int d_fd;                                         // driver descriptor
    volatile unsigned *d_map_base;                    // driver memory map

    // configuration data received from the interface
    unsigned int d_channel;    // channel number
    unsigned d_ncorrelators;   // number of correlators
    unsigned d_correlator_length_samples;
    float d_rem_code_phase_chips;
    float d_code_phase_step_chips;
    float d_rem_carrier_phase_in_rad;
    float d_phase_step_rad;
    
    // configuration data computed in the format that the FPGA expects
    unsigned *d_initial_index; 
    unsigned *d_initial_interp_counter; 
    unsigned d_code_phase_step_chips_num;
    int d_rem_carr_phase_rad_int;
    int d_phase_step_rad_int;
    unsigned d_initial_sample_counter;

    // FPGA private functions
    unsigned fpga_acquisition_test_register(unsigned writeval);
    void fpga_configure_tracking_gps_local_code(void); 
    void fpga_compute_code_shift_parameters(void);
    void fpga_configure_code_parameters_in_fpga(void);
    void fpga_compute_signal_parameters_in_fpga(void);
    void fpga_configure_signal_parameters_in_fpga(void);
    void fpga_launch_multicorrelator_fpga(void);
    void read_tracking_gps_results(void);
    void unlock_channel(void);
};


#endif /* GNSS_SDR_FPGA_MULTICORRELATOR_H_ */
