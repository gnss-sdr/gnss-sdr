/*!
 * \file fpga_spidev.h
 * \brief FPGA SPI control.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_FPGA_SPIDEV_H
#define GNSS_SDR_FPGA_SPIDEV_H

#include <string>

class Fpga_spidev
{
public:
    /*!
     * \brief Default constructor.
     */
    Fpga_spidev() = default;

    /*!
     * \brief Default destructor.
     */
    ~Fpga_spidev() = default;

    /*!
     * \brief write a register through the SPI.
     */
    int write_reg32(char addr, uint32_t data);

    /*!
     * \brief read a register through the SPI.
     */
    int read_reg32(uint8_t addr, uint32_t* copy_to);
    /*!
     * \brief Open the SPI device driver.
     */
    int SPI_open(void);

    /*!
     * \brief Close the SPI device driver
     */
    int SPI_close(void) const;

private:
    static const uint32_t SPI_SPEED = 250000;
    const std::string SPI_DEVICE_NAME = std::string("/dev/spidev2.0");  // Switch UIO device name

    int d_fd;
};


#endif  // GNSS_SDR_FPGA_SPIDEV_H
