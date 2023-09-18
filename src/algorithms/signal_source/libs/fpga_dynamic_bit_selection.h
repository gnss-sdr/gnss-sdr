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
    explicit Fpga_dynamic_bit_selection(uint32_t num_freq_bands);

    /*!
     * \brief Destructor
     */
    ~Fpga_dynamic_bit_selection();

    /*!
     * \brief This function configures the switch in th eFPGA
     */
    void bit_selection(void);

private:
    const std::string switch_device_name = std::string("AXIS_Switch_v1_0_0");          // Switch UIO device name
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

    void close_devices(void);

    std::vector<volatile unsigned*> d_map_base;
    std::vector<int> d_device_descriptors;
    std::vector<uint32_t> d_shift_out_bits;

    uint32_t d_num_freq_bands;  // number of frequency bands
};


/** \} */
/** \} */
#endif  // GNSS_SDR_FPGA_DYNAMIC_BIT_SELECTION_H
