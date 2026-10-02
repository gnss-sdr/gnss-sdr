/*!
 * \file fpga_dynamic_bit_selection.h
 * \brief Dynamic bit selection in the received signal.
 * \authors <ul>
 *          <li> Marc Majoral, 2020. mmajoral(at)cttc.es
 *          </ul>
 *
 * Class that controls the Dynamic Bit Selection in the FPGA.
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_FPGA_DYNAMIC_BIT_SELECTION_H
#define GNSS_SDR_FPGA_DYNAMIC_BIT_SELECTION_H

#include <cstddef>
#include <cstdint>
#include <string>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_libs
 * \{ */


/*!
 * \brief Controls dynamic bit selection in the FPGA.
 */
class Fpga_dynamic_bit_selection
{
public:
    /*!
     * \brief Constructor
     */
    explicit Fpga_dynamic_bit_selection(bool enable_rx1_band, bool enable_rx2_band);

    /*!
     * \brief Destructor
     */
    ~Fpga_dynamic_bit_selection();

    // Prevent copying objects that own FPGA mappings and file descriptors.
    Fpga_dynamic_bit_selection(const Fpga_dynamic_bit_selection &) = delete;
    Fpga_dynamic_bit_selection &operator=(const Fpga_dynamic_bit_selection &) = delete;


    /*!
     * \brief Adjusts the bit shift for each enabled frequency band based on signal power.
     */
    void bit_selection(void);

private:
    // IP core device name
    const std::string DYN_BIT_SEL_DEV_NAME = std::string("dynamic_bits_selector");  // Dynamic bit selection device name

    // IP Core version
    const uint32_t FPGA_DYN_BIT_SEL_IP_VERSION_1_2 = 0x0002;  // Dynamic bit selection IP core version 1.2

    // page size
    static const size_t FPGA_PAGE_SIZE = 0x1000;

    // write-only registers
    static const uint32_t SOBITS_REG_ADDR = 0;  // number of shift bits

    // read-only registers
    static const uint32_t FPGA_IP_CORE_VERSION_REG_ADDR = 0;  // IP core version register address
    static const uint32_t SIGPOW_REG_ADDR = 1;                // rx signal power
    static const uint32_t MAX_BIT_SHIFT_REG_ADDR = 2;         // maximum number of shift bits

    // dynamic bit selection parameters
    static const uint32_t SELECT_FREQ_BAND_1 = 1;          // selection for frequency band 1
    static const uint32_t SELECT_FREQ_BAND_2 = 2;          // selection for frequency band 2
    static const uint32_t SHIFT_OUT_BITS_MAX_DEFAULT = 8;  // take the most significant bits by default
    static const uint32_t SHIFT_OUT_BITS_MIN = 0;          // minimum possible value for the bit selection
    static const uint32_t POWER_THRESHOLD_HIGH = 9000;
    static const uint32_t POWER_THRESHOLD_LOW = 3000;

    bool open_device(volatile unsigned **d_map_base, int &d_dev_descr, int freq_band);
    void initialize_device(volatile unsigned *d_map_base, uint32_t &shift_out_bits, uint32_t &shift_out_bit_max);
    void bit_selection_per_rf_band(volatile unsigned *d_map_base, uint32_t &shift_out_bits, uint32_t shift_out_bit_max);
    void close_device(volatile unsigned *d_map_base, int &d_dev_descr);

    volatile unsigned *d_map_base_freq_band_1;
    volatile unsigned *d_map_base_freq_band_2;
    int d_dev_descr_freq_band_1;
    int d_dev_descr_freq_band_2;
    uint32_t d_shift_out_bits_freq_band_1;
    uint32_t d_shift_out_bits_freq_band_2;
    uint32_t d_shift_out_bit_max_band_1;
    uint32_t d_shift_out_bit_max_band_2;
    bool d_enable_rx1_band;
    bool d_enable_rx2_band;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_FPGA_DYNAMIC_BIT_SELECTION_H
