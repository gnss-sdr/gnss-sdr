/*!
 * \file fpga_buffer_monitor.h
 * \brief Check receiver buffer overflow and monitor the status of the receiver
 * buffers.
 * \authors
 * <ul>
 *    <li> Marc Majoral, 2021. mmajoral(at)cttc.es
 * </ul>
 *
 * Class that checks the receiver buffer overflow flags and monitors the status
 * of the receiver buffers.
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_FPGA_BUFFER_MONITOR_H
#define GNSS_SDR_FPGA_BUFFER_MONITOR_H

#include <cstdint>  // for int32_t
#include <fstream>  // for std::ofstream
#include <string>   // for std::string

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_libs
 * \{ */


/*!
 * \brief Class that checks the receiver buffer overflow flags and monitors the
 * status of the receiver buffers.
 */
class Fpga_buffer_monitor
{
public:
    /*!
     * \brief Constructor
     */
    explicit Fpga_buffer_monitor(uint32_t num_freq_bands,
        bool dump,
        std::string dump_filename);
    //    explicit Fpga_buffer_monitor(const std::string& device_name,
    //        uint32_t num_freq_bands,
    //        bool dump,
    //        std::string dump_filename);

    /*!
     * \brief Destructor
     */
    ~Fpga_buffer_monitor();

    /*!
     * \brief This function checks buffer overflow and monitors the FPGA buffer status
     */
    void check_buffer_overflow_and_monitor_buffer_status();

private:
    const std::string BUFFER_MONITOR_DEVICE_NAME = std::string("buffer_monitor");  // buffer monitor device name
    static const size_t FPGA_PAGE_SIZE = 0x1000;
    static const uint32_t test_register_writeval = 0x55AA;
    static const uint32_t num_sapmples_per_buffer_element = 2;
    // write addresses
    static const uint32_t reset_overflow_flags_and_max_buff_size_reg_addr = 0;
    // read-write addresses
    static const uint32_t test_reg_addr = 7;
    // read addresses
    static const uint32_t current_buff_occ_freq_band_0_reg_addr = 0;
    static const uint32_t current_buff_occ_freq_band_1_reg_addr = 1;
    static const uint32_t max_buff_occ_freq_band_0_reg_addr = 2;
    static const uint32_t max_buff_occ_freq_band_1_reg_addr = 3;
    static const uint32_t overflow_flags_reg_addr = 4;
    // FPGA-related constants
    static const uint32_t overflow_freq_band_0_bit_pos = 1;
    static const uint32_t overflow_freq_band_1_bit_pos = 2;

    int32_t buffer_monitor_test_register();
    void close_device();

    std::string d_dump_filename;
    std::ofstream d_dump_file;

    volatile unsigned* d_map_base;  // driver memory map corresponding to the FPGA buffer monitor
    int d_device_descriptor;        // driver descriptor corresponding to the FPGA buffer monitor

    uint32_t d_num_freq_bands;

    uint32_t d_max_buff_occ_freq_band_0;
    uint32_t d_max_buff_occ_freq_band_1;

    bool d_dump;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_FPGA_BUFFER_MONITOR_H
