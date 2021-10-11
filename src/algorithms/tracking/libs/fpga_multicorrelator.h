/*!
 * \file fpga_multicorrelator.h
 * \brief FPGA vector correlator class
 * \authors <ul>
 *          <li> Marc Majoral, 2019. mmajoral(at)cttc.cat
 *          <li> Javier Arribas, 2019. jarribas(at)cttc.es
 *          </ul>
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

#ifndef GNSS_SDR_FPGA_MULTICORRELATOR_H
#define GNSS_SDR_FPGA_MULTICORRELATOR_H

#include <gnuradio/block.h>
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <cstdint>
#include <string>

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_libs
 * \{ */


/*!
 * \brief Class that implements carrier wipe-off and correlators.
 */
class Fpga_Multicorrelator_8sc
{
public:
    /*!
     * \brief Constructor
     */
    Fpga_Multicorrelator_8sc(int32_t n_correlators,
        int32_t *ca_codes,
        int32_t *data_codes,
        uint32_t code_length_chips,
        bool track_pilot,
        uint32_t code_samples_per_chip);

    /*!
     * \brief Destructor
     */
    ~Fpga_Multicorrelator_8sc();

    /*!
     * \brief Configure pointers to the FPGA multicorrelator results
     */
    void set_output_vectors(gr_complex *corr_out, gr_complex *Prompt_Data);

    /*!
     * \brief Configure the local code in the FPGA multicorrelator
     */
    void set_local_code_and_taps(
        float *shifts_chips, float *prompt_data_shift, int32_t PRN);

    /*!
     * \brief Configure code phase and code rate parameters in the FPGA
     */
    void update_local_code();

    /*!
     * \brief Perform a multicorrelation
     */
    void Carrier_wipeoff_multicorrelator_resampler(
        float rem_carrier_phase_in_rad,
        float phase_step_rad,
        float carrier_phase_rate_step_rad,
        float rem_code_phase_chips,
        float code_phase_step_chips,
        float code_phase_rate_step_chips,
        int32_t signal_length_samples);

    /*!
     * \brief Stop the correlation process in the FPGA and free code phase and code rate parameters
     */
    bool free();

    /*!
     * \brief Open the FPGA device driver
     */
    void open_channel(const std::string &device_io_name, uint32_t channel);

    /*!
     * \brief Set the initial sample number where the tracking process begins
     */
    void set_initial_sample(uint64_t samples_offset);

    /*!
     * \brief Read the sample counter in the FPGA
     */
    uint64_t read_sample_counter();

    /*!
     * \brief Start the tracking process in the FPGA
     */
    void lock_channel();

    /*!
     * \brief finish the tracking process in the FPGA
     */
    void unlock_channel();

    /*!
     * \brief Set the secondary code length in the FPGA. This is only used when extended coherent integration
     * is enabled in the FPGA. If tracking the pilot is enabled then secondary_code_0_length is the length of the pilot
     * secondary code and secondary_code_1_length is the length of the data secondary code. If tracking the pilot is disabled
     * then secondary_code_0_length is the length of the data secondary code, and secondary_code_1_length must be set to zero.
     */
    void set_secondary_code_lengths(uint32_t secondary_code_0_length, uint32_t secondary_code_1_length);

    /*!
     * \brief Initialize the secondary code in the FPGA. If tracking the pilot is enabled then the pilot secondary code is
     * configured when secondary_code = 0 and the data secondary code is configured when secondary_code = 1. If tracking the
     * pilot is disabled then the data secondary code is configured when secondary code = 0.
     */
    void initialize_secondary_code(uint32_t secondary_code, std::string *secondary_code_string);

    /*!
     * \brief Set the PRN length in the FPGA in number of samples. This function is only used then extended coherent integration is enabled in the
     * FPGA. The FPGA allows for the configuration of two PRN lengths. When the length of the extended coherent integration is bigger than the
     * length of the PRN code, the FPGA uses the first_length_secondary_code as the length of the PRN code immediately following the beginning
     * of the extended coherent integration, and the next_length_secondary_code as the length of the remaining PRN codes.
     * The purpose of this is to have the option to allow the FPGA to compensate for a possible deviation between the nominal value of the PRN
     * code length and the measured PRN code length in the PRN immediately following the start of the coherent integration only.
     * If this option is not used then write the same value to first_length_secondary_code  and next_length_secondary_code.
     */
    void update_prn_code_length(uint32_t first_prn_length, uint32_t next_prn_length);

    /*!
     * \brief Enable the use of secondary codes in the FPGA
     */
    void enable_secondary_codes();

    /*!
     * \brief Disable the use of secondary codes in the FPGA
     */
    void disable_secondary_codes();

private:
    // FPGA register addresses
    // write addresses
    static const uint32_t code_phase_step_chips_num_reg_addr = 0;
    static const uint32_t initial_index_reg_base_addr = 1;
    static const uint32_t initial_interp_counter_reg_base_addr = 7;
    static const uint32_t nsamples_minus_1_reg_addr = 13;
    static const uint32_t code_length_minus_1_reg_addr = 14;
    static const uint32_t rem_carr_phase_rad_reg_addr = 15;
    static const uint32_t phase_step_rad_reg_addr = 16;
    static const uint32_t prog_mems_addr = 17;
    static const uint32_t drop_samples_reg_addr = 18;
    static const uint32_t initial_counter_value_reg_addr_lsw = 19;
    static const uint32_t initial_counter_value_reg_addr_msw = 20;
    static const uint32_t code_phase_step_chips_rate_reg_addr = 21;
    static const uint32_t phase_step_rate_reg_addr = 22;
    static const uint32_t stop_tracking_reg_addr = 23;
    static const uint32_t secondary_code_lengths_reg_addr = 25;
    static const uint32_t prog_secondary_code_0_data_reg_addr = 26;
    static const uint32_t prog_secondary_code_1_data_reg_addr = 27;
    static const uint32_t first_prn_length_minus_1_reg_addr = 28;
    static const uint32_t next_prn_length_minus_1_reg_addr = 29;
    static const uint32_t start_flag_addr = 30;
    // read-write addresses
    static const uint32_t test_reg_addr = 31;
    // read addresses
    static const uint32_t result_reg_real_base_addr = 1;
    static const uint32_t result_reg_imag_base_addr = 7;
    static const uint32_t sample_counter_reg_addr_lsw = 13;
    static const uint32_t sample_counter_reg_addr_msw = 14;
    // FPGA-related constants
    static const uint32_t secondary_code_word_size = 20;        // the secondary codes are written in to the FPGA in words of secondary_code_word_size bits
    static const uint32_t secondary_code_wr_strobe = 0x800000;  // write strobe position in the secondary code write register
    static const uint32_t secondary_code_addr_bits = 0x100000;  // memory address position in the secondary code write register
    static const uint32_t drop_samples = 1;                     // bit 0 of drop_samples_reg_addr
    static const uint32_t enable_secondary_code = 2;            // bit 1 of drop_samples_reg_addr
    static const uint32_t init_secondary_code_addresses = 4;    // bit 2 of drop_samples_reg_addr
    static const uint32_t page_size = 0x10000;
    static const uint32_t max_code_resampler_counter = 1 << 31;  // 2^(number of bits of precision of the code resampler)
    static const uint32_t local_code_fpga_clear_address_counter = 0x10000000;
    static const uint32_t test_register_track_writeval = 0x55AA;

    // private functions
    uint32_t fpga_acquisition_test_register(uint32_t writeval);
    void fpga_configure_tracking_gps_local_code(int32_t PRN);
    void fpga_compute_code_shift_parameters();
    void fpga_configure_code_parameters_in_fpga();
    void fpga_compute_signal_parameters_in_fpga();
    void fpga_configure_signal_parameters_in_fpga();
    void fpga_launch_multicorrelator_fpga();
    void read_tracking_gps_results();
    void close_device(void);
    void write_secondary_code(uint32_t secondary_code_length, std::string *secondary_code_string, uint32_t reg_addr);

    volk_gnsssdr::vector<uint32_t> d_initial_index;
    volk_gnsssdr::vector<uint32_t> d_initial_interp_counter;

    uint64_t d_initial_sample_counter;

    gr_complex *d_corr_out;
    gr_complex *d_Prompt_Data;

    float *d_shifts_chips;
    float *d_prompt_data_shift;

    float d_rem_code_phase_chips;
    float d_code_phase_step_chips;
    float d_code_phase_rate_step_chips;
    float d_rem_carrier_phase_in_rad;
    float d_phase_step_rad;
    float d_carrier_phase_rate_step_rad;

    uint32_t d_code_length_samples;
    uint32_t d_n_correlators;  // number of correlators

    // data related to the hardware module and the driver
    int32_t d_device_descriptor;    // driver descriptor
    volatile uint32_t *d_map_base;  // driver memory map

    // configuration data received from the interface
    uint32_t d_correlator_length_samples;

    uint32_t d_code_phase_step_chips_num;
    uint32_t d_code_phase_rate_step_chips_num;
    int32_t d_rem_carr_phase_rad_int;
    int32_t d_phase_step_rad_int;
    int32_t d_carrier_phase_rate_step_rad_int;

    // PRN codes
    int32_t *d_ca_codes;
    int32_t *d_data_codes;

    // secondary code configuration
    uint32_t d_secondary_code_0_length;
    uint32_t d_secondary_code_1_length;

    bool d_track_pilot;
    bool d_secondary_code_enabled;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_FPGA_MULTICORRELATOR_H
