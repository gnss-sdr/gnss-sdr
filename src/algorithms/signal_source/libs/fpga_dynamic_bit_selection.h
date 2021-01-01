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
    explicit Fpga_dynamic_bit_selection(const std::string& device_name1, const std::string& device_name2);

    /*!
     * \brief Destructor
     */
    ~Fpga_dynamic_bit_selection();

    /*!
     * \brief This function configures the switch in th eFPGA
     */
    //    void set_switch_position(int32_t switch_position);
    void bit_selection(void);

private:
    static const size_t FPGA_PAGE_SIZE = 0x10000;

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

    uint32_t shift_out_bits_band1;  // number of bits to shift for frequency band 1
    uint32_t shift_out_bits_band2;  // number of bits to shift for frequency band 2

    volatile unsigned* d_map_base1;  // driver memory map corresponding to frequency band 1
    int d_device_descriptor1;        // driver descriptor corresponding to frequency band 1

    volatile unsigned* d_map_base2;  // driver memory map corresponding to frequency band 2
    int d_device_descriptor2;        // driver descriptor corresponding to frequency band 2
};


/** \} */
/** \} */
#endif  // GNSS_SDR_FPGA_DYNAMIC_BIT_SELECTION_H
