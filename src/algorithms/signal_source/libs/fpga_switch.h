/*!
 * \file fpga_switch.h
 * \brief Switch that connects the HW accelerator queues to the analog front end or the DMA.
 * \authors <ul>
 *          <li> Marc Majoral, 2019. mmajoral(at)cttc.cat
 *          <li> Javier Arribas, 2016. jarribas(at)cttc.es
 *          </ul>
 *
 * Class that controls a switch in the FPGA
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

#ifndef GNSS_SDR_FPGA_SWITCH_H
#define GNSS_SDR_FPGA_SWITCH_H

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
class Fpga_Switch
{
public:
    /*!
     * \brief Constructor
     */
    Fpga_Switch(void);
    /*!
     * \brief Destructor
     */
    ~Fpga_Switch();

    /*!
     * \brief This function configures the switch in th eFPGA
     */
    void set_switch_position(int32_t switch_position);

private:
    const std::string SWITCH_DEVICE_NAME = std::string("AXIS_Switch_v1_0_0");  // Switch UIO device name
    static const size_t FPGA_PAGE_SIZE = 0x1000;
    static const uint32_t TEST_REGISTER_TRACK_WRITEVAL = 0x55AA;
    static const uint32_t MAX_LENGTH_DEVICEIO_NAME = 50;

    // private functions
    unsigned fpga_switch_test_register(unsigned writeval);
    void close_device(void);

    volatile unsigned* d_map_base;  // driver memory map
    int d_device_descriptor;        // driver descriptor
};


/** \} */
/** \} */
#endif  // GNSS_SDR_FPGA_SWITCH_H
