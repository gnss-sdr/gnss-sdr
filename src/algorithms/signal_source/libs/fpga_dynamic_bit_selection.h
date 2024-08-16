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
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_FPGA_DYNAMIC_BIT_SELECTION_H
#define GNSS_SDR_FPGA_DYNAMIC_BIT_SELECTION_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_libs
 * \{ */


/*!
 * \brief Class that controls the switch in the FPGA, which connects the FPGA acquisition and multicorrelator modules to
 * either the DMA or the Analog Front-End.
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

    /*!
     * \brief This function configures the switch in th eFPGA
     */
    void bit_selection(void);

private:
    const std::string dyn_bit_sel_device_name = std::string("dynamic_bits_selector");  // Switch dhnamic bit selector device name
    static const size_t FPGA_PAGE_SIZE = 0x1000;
    static const uint32_t Num_bits_ADC = 12;                                      // Number of bits in the ADC
    static const uint32_t Num_bits_FPGA = 4;                                      // Number of bits after the bit selection
    static const uint32_t shift_out_bits_default = Num_bits_ADC - Num_bits_FPGA;  // take the most significant bits by default
    static const uint32_t shift_out_bits_min = 0;                                 // minimum possible value for the bit selection
    static const uint32_t shift_out_bit_max = Num_bits_ADC - Num_bits_FPGA;       // maximum possible value for the bit selection
    // received signal power thresholds for the bit selection
    // the received signal power is estimated as the averaged squared absolute value of the received signal samples
    static const uint32_t Power_Threshold_High = 9000;
    static const uint32_t Power_Threshold_Low = 3000;

    void open_device(volatile unsigned **d_map_base, int &d_dev_descr, int freq_band);
    void bit_selection_per_rf_band(volatile unsigned *d_map_base, uint32_t shift_out_bits);
    void close_device(volatile unsigned *d_map_base, int &d_dev_descr);

    volatile unsigned *d_map_base_freq_band_1;
    volatile unsigned *d_map_base_freq_band_2;
    int d_dev_descr_freq_band_1;
    int d_dev_descr_freq_band_2;
    uint32_t d_shift_out_bits_freq_band_1;
    uint32_t d_shift_out_bits_freq_band_2;
    bool d_enable_rx1_band;
    bool d_enable_rx2_band;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_FPGA_DYNAMIC_BIT_SELECTION_H
