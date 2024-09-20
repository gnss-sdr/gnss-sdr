/*!
 * \file fpga_spidev.cc
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

#include "fpga_spidev.h"
#include <cstring>             // for memset()
#include <fcntl.h>             // for open(), O_RDWR
#include <iostream>            // for std::cerr
#include <linux/spi/spidev.h>  // spidev driver
#include <sys/ioctl.h>         // for ioctl()
#include <unistd.h>            // for close()


int Fpga_spidev::SPI_open()
{
    if ((d_fd = open(SPI_DEVICE_NAME.c_str(), O_RDWR)) < 0)
        {
            std::cerr << "Failed to open the " << SPI_DEVICE_NAME << " device file \n";
            return -1;
        }

    int ret;
    int32_t mode = 0;

    ret = ioctl(d_fd, SPI_IOC_WR_MODE32, &mode);
    if (ret == -1)
        {
            std::cerr << "can't set spi mode\n";
            return -1;
        }

    ret = ioctl(d_fd, SPI_IOC_RD_MODE32, &mode);  // le digo al spi "algo"
    if (ret == -1)
        {
            std::cerr << "can't set spi mode\n";
            return -1;
        }

    return 0;
}

int Fpga_spidev::write_reg32(char addr, uint32_t data)
{
    uint8_t data_buffer[2];
    uint8_t recv_buffer[4];
    int res = 0;
    struct spi_ioc_transfer xfer[2];
    memset(xfer, 0, sizeof(xfer));
    xfer[0].bits_per_word = 8;
    xfer[0].speed_hz = SPI_SPEED;
    xfer[1].bits_per_word = 8;
    xfer[1].speed_hz = SPI_SPEED;

    memset(&recv_buffer, 0, sizeof(recv_buffer));
    memset(&data_buffer, 0, sizeof(data_buffer));

    data_buffer[1] = addr << 4 | 0 << 3;
    xfer[0].tx_buf = (unsigned long)data_buffer;
    xfer[0].len = 2;

    // Would use memcpy but 'data' is in little endian
    ((char*)recv_buffer)[0] = ((char*)&data)[3];
    ((char*)recv_buffer)[1] = ((char*)&data)[2];
    ((char*)recv_buffer)[2] = ((char*)&data)[1];
    ((char*)recv_buffer)[3] = ((char*)&data)[0];

    xfer[1].tx_buf = (unsigned long)recv_buffer;
    xfer[1].len = 4;
    res = ioctl(d_fd, SPI_IOC_MESSAGE(2), xfer);
    if (res < 0)
        {
            std::cout << "Error sending SPI message\n";
            return res;
        }
    return 0;
}

int Fpga_spidev::read_reg32(uint8_t addr, uint32_t* copy_to)
{
    uint8_t data_buffer[2];
    uint8_t recv_buffer[4];
    int res;
    struct spi_ioc_transfer xfer[2];
    memset(xfer, 0, sizeof(xfer));
    xfer[0].bits_per_word = 8;
    xfer[0].speed_hz = SPI_SPEED;
    xfer[1].bits_per_word = 8;
    xfer[1].speed_hz = SPI_SPEED;

    memset(&recv_buffer, 0, sizeof(recv_buffer));
    memset(&data_buffer, 0, sizeof(data_buffer));

    data_buffer[1] = addr << 4 | 1 << 3;
    xfer[0].tx_buf = (unsigned long)data_buffer;
    xfer[0].len = 2;

    xfer[1].rx_buf = (unsigned long)recv_buffer;
    xfer[1].len = 4;
    res = ioctl(d_fd, SPI_IOC_MESSAGE(2), xfer);
    if (res < 0)
        {
            std::cout << "Error sending SPI message\n";
            return res;
        }

    // the register data is received in the reverse order
    uint32_t tmp_result = 0;
    for (uint32_t k = 0; k < 4; ++k)
        {
            tmp_result = tmp_result + ((recv_buffer[3 - k]) << 8 * k);
        }
    *copy_to = tmp_result;

    return 0;
}

int Fpga_spidev::SPI_close() const
{
    return close(d_fd);
}
