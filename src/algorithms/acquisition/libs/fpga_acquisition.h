/*!
 * \file fpga_acquisition.h
 * \brief Highly optimized FPGA vector correlator class
 * \authors <ul>
 *          <li> Marc Majoral, 2019. mmajoral(at)cttc.cat
 *          </ul>
 *
 * Class that controls and executes a highly optimized acquisition HW
 * accelerator in the FPGA
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

#ifndef GNSS_SDR_FPGA_ACQUISITION_H_
#define GNSS_SDR_FPGA_ACQUISITION_H_

#include <cstdint>
#include <string>

/*!
 * \brief Class that implements carrier wipe-off and correlators.
 */
class Fpga_Acquisition
{
public:
    Fpga_Acquisition(
        std::string device_name,
        uint32_t nsamples,
        uint32_t doppler_max,
        uint32_t nsamples_total,
        int64_t fs_in,
        uint32_t sampled_ms,
        uint32_t select_queue,
        uint32_t *all_fft_codes,
        uint32_t excludelimit);

    ~Fpga_Acquisition();

    bool set_local_code(uint32_t PRN);

    void set_doppler_sweep(uint32_t num_sweeps, uint32_t doppler_step, int32_t doppler_min);

    void run_acquisition(void);

    void read_acquisition_results(
        uint32_t *max_index,
        float *firstpeak,
        float *secondpeak,
        uint64_t *initial_sample,
        float *power_sum,
        uint32_t *doppler_index,
        uint32_t *total_blk_exp);

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

    /*!
     * \brief Reset the FPGA PL.
     */
    void reset_acquisition(void);

    /*!
     * \brief read the scaling factor that has been used by the FFT-IFFT
     */
    void read_fpga_total_scale_factor(uint32_t *total_scale_factor, uint32_t *fw_scale_factor);

    void set_block_exp(uint32_t total_block_exp);

    void write_local_code(void);

    void configure_acquisition(void);

    void open_device();

    void close_device();

private:
    int64_t d_fs_in;
    // data related to the hardware module and the driver
    int32_t d_fd;                   // driver descriptor
    volatile uint32_t *d_map_base;  // driver memory map
    uint32_t *d_all_fft_codes;      // memory that contains all the code ffts
    uint32_t d_vector_length;       // number of samples including padding and number of ms
    uint32_t d_excludelimit;
    uint32_t d_nsamples_total;  // number of samples including padding
    uint32_t d_nsamples;        // number of samples not including padding
    uint32_t d_select_queue;    // queue selection
    std::string d_device_name;  // HW device name
    uint32_t d_doppler_max;     // max doppler
    uint32_t d_doppler_step;    // doppler step
    uint32_t d_PRN;             // PRN
    // FPGA private functions
    void fpga_acquisition_test_register(void);
    void read_result_valid(uint32_t *result_valid);
};

#endif /* GNSS_SDR_FPGA_ACQUISITION_H_ */
