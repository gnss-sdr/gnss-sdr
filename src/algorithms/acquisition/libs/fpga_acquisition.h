/*!
 * \file fpga_acquisition.h
 * \brief High optimized FPGA vector correlator class
 * \authors <ul>
 *          <li> Marc Majoral, 2018. mmajoral(at)cttc.cat
 *          </ul>
 *
 * Class that controls and executes a high optimized acquisition HW
 * accelerator in the FPGA
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

#ifndef GNSS_SDR_FPGA_ACQUISITION_H_
#define GNSS_SDR_FPGA_ACQUISITION_H_

#include <gnuradio/fft/fft.h>
#include <volk/volk.h>
#include <cstdint>

/*!
 * \brief Class that implements carrier wipe-off and correlators.
 */
class fpga_acquisition
{
public:
    fpga_acquisition(std::string device_name,
        uint32_t nsamples,
        uint32_t doppler_max,
        uint32_t nsamples_total,
        int64_t fs_in,
        uint32_t sampled_ms,
        uint32_t select_queue,
        lv_16sc_t *all_fft_codes);

    ~fpga_acquisition();
    bool init();
    bool set_local_code(uint32_t PRN);
    bool free();
    void run_acquisition(void);
    void set_phase_step(uint32_t doppler_index);
    void read_acquisition_results(uint32_t *max_index, float *max_magnitude,
        uint32_t *initial_sample, float *power_sum);
    void block_samples();
    void unblock_samples();

    /*!
     * \brief Set maximum Doppler grid search
     * \param doppler_max - Maximum Doppler shift considered in the grid search [Hz].
     */
    void set_doppler_max(uint32_t doppler_max)
    {
        d_doppler_max = doppler_max;
    }

    /*!
     * \brief Set Doppler steps for the grid search
     * \param doppler_step - Frequency bin of the search grid [Hz].
     */
    void set_doppler_step(uint32_t doppler_step)
    {
        d_doppler_step = doppler_step;
    }

private:
    int64_t d_fs_in;
    // data related to the hardware module and the driver
    int32_t d_fd;                   // driver descriptor
    volatile uint32_t *d_map_base;  // driver memory map
    lv_16sc_t *d_all_fft_codes;     // memory that contains all the code ffts
    uint32_t d_vector_length;       // number of samples incluing padding and number of ms
    uint32_t d_nsamples_total;      // number of samples including padding
    uint32_t d_nsamples;            // number of samples not including padding
    uint32_t d_select_queue;        // queue selection
    std::string d_device_name;      // HW device name
    uint32_t d_doppler_max;         // max doppler
    uint32_t d_doppler_step;        // doppler step
    // FPGA private functions
    uint32_t fpga_acquisition_test_register(uint32_t writeval);
    void fpga_configure_acquisition_local_code(lv_16sc_t fft_local_code[]);
    void configure_acquisition();
    void reset_acquisition(void);
    void close_device();
};

#endif /* GNSS_SDR_FPGA_ACQUISITION_H_ */
