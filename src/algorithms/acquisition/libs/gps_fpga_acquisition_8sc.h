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
#include <gnuradio/fft/fft.h>

/*!
 * \brief Class that implements carrier wipe-off and correlators.
 */
class gps_fpga_acquisition_8sc
{
public:
    gps_fpga_acquisition_8sc(std::string device_name,
        unsigned int vector_length, unsigned int nsamples,
        unsigned int nsamples_total, long fs_in, long freq,
        unsigned int sampled_ms, unsigned select_queue);
    ~gps_fpga_acquisition_8sc();
    bool init();
    bool set_local_code(
        unsigned int PRN);  //int code_length_chips, const lv_16sc_t* local_code_in, float *shifts_chips);
    bool free();
    void run_acquisition(void);
    void set_phase_step(unsigned int doppler_index);
    void read_acquisition_results(uint32_t *max_index, float *max_magnitude,
        unsigned *initial_sample, float *power_sum);
    void block_samples();
    void unblock_samples();
    void open_device();
    void close_device();

    /*!
     * \brief Set maximum Doppler grid search
     * \param doppler_max - Maximum Doppler shift considered in the grid search [Hz].
     */
    void set_doppler_max(unsigned int doppler_max)
    {
        d_doppler_max = doppler_max;
    }

    /*!
     * \brief Set Doppler steps for the grid search
     * \param doppler_step - Frequency bin of the search grid [Hz].
     */
    void set_doppler_step(unsigned int doppler_step)
    {
        d_doppler_step = doppler_step;
    }

private:
    long d_freq;
    long d_fs_in;
    gr::fft::fft_complex *d_fft_if;  // function used to run the fft of the local codes

    // data related to the hardware module and the driver
    int d_fd;                       // driver descriptor
    volatile unsigned *d_map_base;  // driver memory map
    lv_16sc_t *d_all_fft_codes;     // memory that contains all the code ffts
    unsigned int d_vector_length;   // number of samples incluing padding and number of ms
    unsigned int d_nsamples;        // number of samples not including padding
    unsigned int d_select_queue;    // queue selection
    std::string d_device_name;      // HW device name
    unsigned int d_doppler_max;     // max doppler
    unsigned int d_doppler_step;    // doppler step

    // FPGA private functions
    unsigned fpga_acquisition_test_register(unsigned writeval);
    void fpga_configure_acquisition_local_code(lv_16sc_t fft_local_code[]);
    void configure_acquisition();
};

#endif /* GNSS_SDR_FPGA_MULTICORRELATOR_H_ */
