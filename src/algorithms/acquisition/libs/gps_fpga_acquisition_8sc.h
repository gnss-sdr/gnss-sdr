/*!
 * \file fpga_acquisition_8sc.h
 * \brief High optimized FPGA vector correlator class for lv_16sc_t (short int complex).
 * \authors <ul>
 * 			<li> Marc Majoral, 2017. mmajoral(at)cttc.cat
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

#ifndef GNSS_SDR_FPGA_ACQUISITION_8SC_H_
#define GNSS_SDR_FPGA_ACQUISITION_8SC_H_

#include <volk_gnsssdr/volk_gnsssdr.h>

#include <gnuradio/block.h>


/*!
 * \brief Class that implements carrier wipe-off and correlators.
 */
class gps_fpga_acquisition_8sc
{
public:
    gps_fpga_acquisition_8sc();
    ~gps_fpga_acquisition_8sc();
    bool init(unsigned int fft_size, unsigned int nsamples_total, long d_freq, unsigned int doppler_max, unsigned int doppler_step, int num_doppler_bins, long fs_in, unsigned select_queue);
    bool set_local_code(gr_complex* fft_codes); //int code_length_chips, const lv_16sc_t* local_code_in, float *shifts_chips);
    bool free();
    void run_acquisition(void);
    void set_phase_step(unsigned int doppler_index);
    void read_acquisition_results(uint32_t* max_index, float* max_magnitude, unsigned *initial_sample, float *power_sum);
    void block_samples();
    void unblock_samples();
private:
    const lv_16sc_t *d_local_code_in;
    lv_16sc_t *d_corr_out;
    float *d_shifts_chips;
    int d_code_length_chips;
    int d_n_correlators;

    // data related to the hardware module and the driver
    char d_device_io_name[11] = "/dev/uio13";  // driver io name
    int d_fd;                                         // driver descriptor
    volatile unsigned *d_map_base;                    // driver memory map

    // configuration data received from the interface
    lv_16sc_t *d_fft_codes = nullptr;
    float *d_phase_step_rad_vector = nullptr;

    unsigned int d_nsamples_total; // total number of samples in the fft including padding
    unsigned int d_nsamples; // number of samples not including padding
    unsigned int d_select_queue =0; // queue selection

    // FPGA private functions
    unsigned fpga_acquisition_test_register(unsigned writeval);
    void fpga_configure_acquisition_local_code(lv_16sc_t fft_local_code[]);
    void configure_acquisition();

};


#endif /* GNSS_SDR_FPGA_MULTICORRELATOR_H_ */
