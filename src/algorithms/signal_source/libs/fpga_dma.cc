/*!
 * \file fpga_dma.cc
 * \brief FPGA DMA control. This code is based in the Xilinx DMA proxy test application:
 * https://github.com/Xilinx-Wiki-Projects/software-prototypes/tree/master/linux-user-space-dma/Software
 * \author Marc Majoral, mmajoral(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "fpga_dma.h"
#include <fcntl.h>
#include <iostream>     // for std::cerr
#include <sys/ioctl.h>  // for ioctl()
#include <sys/mman.h>   // libraries used by the GIPO
#include <unistd.h>

int Fpga_DMA::DMA_open()
{
#if INTPTR_MAX == INT64_MAX  // 64-bit processor architecture
    tx_channel.fd = open("/dev/dma_proxy_tx", O_RDWR);
    if (tx_channel.fd < 1)
        {
            return tx_channel.fd;
        }

    tx_channel.buf_ptr = (struct channel_buffer *)mmap(NULL, sizeof(struct channel_buffer) * TX_BUFFER_COUNT,
        PROT_READ | PROT_WRITE, MAP_SHARED, tx_channel.fd, 0);
    if (tx_channel.buf_ptr == MAP_FAILED)
        {
            std::cerr << "Failed to mmap DMA tx channel\n"
                      << std::endl;
            return -1;
        }

#else  // 32-bit processor architecture
    tx_fd = open("/dev/loop_tx", O_WRONLY);
    if (tx_fd < 1)
        {
            return tx_fd;
        }
    // note: a problem was identified with the DMA: when switching from tx to rx or rx to tx mode
    // the DMA transmission may hang. This problem will be fixed soon.
    // for the moment this problem can be avoided by closing and opening the DMA a second time
    if (close(tx_fd) < 0)
        {
            std::cerr << "Error closing loop device " << '\n';
            return -1;
        }
    // open the DMA  a second time
    tx_fd = open("/dev/loop_tx", O_WRONLY);
    if (tx_fd < 1)
        {
            std::cerr << "Cannot open loop device\n";
            // stop the receiver
            return tx_fd;
        }
#endif

    return 0;
}


std::array<int8_t, BUFFER_SIZE> *Fpga_DMA::get_buffer_address(void)
{
#if INTPTR_MAX == INT64_MAX  // 64-bit processor architecture
    return &tx_channel.buf_ptr[0].buffer;
#else  // 32-bit processor architecture
    return &buffer;
#endif
}


int Fpga_DMA::DMA_write(int nbytes)
{
#if INTPTR_MAX == INT64_MAX  // 64-bit processor architecture

    int buffer_id = 0;

    tx_channel.buf_ptr[0].length = nbytes;

    // start DMA transfer
    if (ioctl(tx_channel.fd, START_XFER, &buffer_id))
        {
            std::cerr << "Error starting tx DMA transfer " << '\n';
            return -1;
        }

    // wait for completion of DMA transfer
    if (ioctl(tx_channel.fd, FINISH_XFER, &buffer_id))
        {
            std::cerr << "Error detecting end of DMA transfer " << '\n';
            return -1;
        }

    if (tx_channel.buf_ptr[buffer_id].status)
        {
            std::cerr << "Proxy DMA Tx transfer error " << '\n';
            return -1;
        }

#else  // 32-bit processor architecture

    const int num_bytes_sent = write(tx_fd, dma_buffer, nread_elements * 2);
    if (num_bytes_sent != num_transferred_bytes)
        {
            return -1
        }

#endif

    return 0;
}

int Fpga_DMA::DMA_close()
{
#if INTPTR_MAX == INT64_MAX  // 64-bit processor architecture
    if (munmap(tx_channel.buf_ptr, sizeof(struct channel_buffer)))
        {
            std::cerr << "Failed to unmap DMA tx channel " << '\n';
            return -1;
        }
    return close(tx_channel.fd);
#else  // 32-bit processor architecture
    return close(tx_fd);
#endif
}
