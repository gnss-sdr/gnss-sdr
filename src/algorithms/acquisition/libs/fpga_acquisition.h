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

#ifndef GNSS_SDR_FPGA_ACQUISITION_H
#define GNSS_SDR_FPGA_ACQUISITION_H

#include <cstdint>
#include <string>

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup acquisition_libs
 * \{ */


/*!
 * \brief Class that implements carrier wipe-off and correlators.
 */
class Fpga_Acquisition
{
public:
    /*!
     * \brief Constructor
     */
    Fpga_Acquisition(
        std::string device_name,
        uint32_t nsamples,
        uint32_t doppler_max,
        uint32_t nsamples_total,
        int64_t fs_in,
        uint32_t select_queue,
        uint32_t *all_fft_codes,
        uint32_t excludelimit);

    /*!
     * \brief Destructor
     */
    ~Fpga_Acquisition() = default;

    /*!
     * \brief Select the code with the chosen PRN
     */
    bool set_local_code(uint32_t PRN);

    /*!
     * \brief Configure the doppler sweep parameters in the FPGA
     */
    void set_doppler_sweep(uint32_t num_sweeps, uint32_t doppler_step, int32_t doppler_min);

    /*!
     * \brief Run the acquisition process in the FPGA
     */
    void run_acquisition();

    /*!
     * \brief Read the results of the acquisition process
     */
    void read_acquisition_results(
        uint32_t *max_index,
        float *firstpeak,
        float *secondpeak,
        uint64_t *initial_sample,
        float *power_sum,
        uint32_t *doppler_index,
        uint32_t *total_blk_exp);

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
    void reset_acquisition();

    /*!
     * \brief stop the acquisition and the FPGA modules.
     */
    void stop_acquisition();

    /*!
     * \brief Read the scaling factor that has been used by the FFT-IFFT
     */
    void read_fpga_total_scale_factor(uint32_t *total_scale_factor, uint32_t *fw_scale_factor);

    /*!
     * \brief Set the block exponent of the FFT in the FPGA.
     */
    void set_block_exp(uint32_t total_block_exp);

    /*!
     * \brief Write the PRN code in the FPGA
     */
    void write_local_code(void);

    /*!
     * \brief Write the acquisition parameters into the FPGA
     */
    void configure_acquisition(void);

    /*!
     * \brief Open the device driver
     */
    void open_device();

    /*!
     * \brief Close the device driver
     */
    void close_device();

private:
    // FPGA register parameters
    static const uint32_t FPGA_PAGE_SIZE = 0x1000;                // default page size for the multicorrelator memory map
    static const uint32_t LAUNCH_ACQUISITION = 1;                 // command to launch the acquisition process
    static const uint32_t RESET_ACQUISITION = 2;                  // command to reset the acquisition and the FPGA Modules
    static const uint32_t STOP_ACQUISITION = 4;                   // command to stop the acquisition and the FPGA modules
    static const uint32_t TEST_REG_SANITY_CHECK = 0x55AA;         // value to check the presence of the test register (to detect the hw)
    static const uint32_t LOCAL_CODE_CLEAR_MEM = 0x10000000;      // command to clear the internal memory of the multicorrelator
    static const uint32_t MEM_LOCAL_CODE_WR_ENABLE = 0x0C000000;  // command to enable the ENA and WR pins of the internal memory of the multicorrelator
    static const uint32_t POW_2_2 = 4;                            // 2^2 (used for the conversion of floating point numbers to integers)
    static const uint32_t POW_2_31 = 2147483648;                  // 2^31 (used for the conversion of floating point numbers to integers)

    static const uint32_t SELECT_LSBits = 0x0000FFFF;         // Select the 10 LSbits out of a 20-bit word
    static const uint32_t SELECT_MSBbits = 0xFFFF0000;        // Select the 10 MSbits out of a 20-bit word
    static const uint32_t SELECT_ALL_CODE_BITS = 0xFFFFFFFF;  // Select a 20 bit word
    static const uint32_t SHL_CODE_BITS = 65536;              // shift left by 10 bits

    // FPGA private functions
    void fpga_acquisition_test_register(void);
    void read_result_valid(uint32_t *result_valid);

    std::string d_device_name;  // HW device name

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
    uint32_t d_doppler_max;     // max doppler
    uint32_t d_doppler_step;    // doppler step
    uint32_t d_PRN;             // PRN
};


/** \} */
/** \} */
#endif  // GNSS_SDR_FPGA_ACQUISITION_H
