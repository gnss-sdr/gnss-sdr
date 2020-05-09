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
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_FPGA_SWITCH_H
#define GNSS_SDR_FPGA_SWITCH_H

#include <string>

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
    explicit Fpga_Switch(const std::string& device_name);

    /*!
     * \brief Destructor
     */
    ~Fpga_Switch();

    /*!
     * \brief This function configures the switch in th eFPGA
     */
    void set_switch_position(int32_t switch_position);

private:
    static const size_t FPGA_PAGE_SIZE = 0x10000;
    static const uint32_t TEST_REGISTER_TRACK_WRITEVAL = 0x55AA;
    static const uint32_t MAX_LENGTH_DEVICEIO_NAME = 50;

    int d_device_descriptor;        // driver descriptor
    volatile unsigned* d_map_base;  // driver memory map

    // private functions
    unsigned fpga_switch_test_register(unsigned writeval);
    void close_device(void);
};

#endif  // GNSS_SDR_FPGA_SWITCH_H
