/*!
 * \file fpga_dma-proxy.cc
 * \brief FPGA DMA control. This code is based in the Xilinx DMA proxy test application:
 * https://github.com/Xilinx-Wiki-Projects/software-prototypes/tree/master/linux-user-space-dma/Software
 * \author Marc Majoral, mmajoral(at)cttc.es
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

#include "fpga_dma-proxy.h"
#include <fcntl.h>
#include <iostream>     // for std::cerr
#include <sys/ioctl.h>  // for ioctl()
#include <sys/mman.h>   // libraries used by the GIPO
#include <unistd.h>

Fpga_DMA::~Fpga_DMA()
{
    DMA_close();
}

int Fpga_DMA::DMA_open()
{
    if (tx_channel.fd >= 0 || tx_channel.buf_ptr != nullptr)
        {
            std::cerr << "DMA device is already open or has an active mapping\n";
            return -1;
        }

    tx_channel.fd = open("/dev/dma_proxy_tx", O_RDWR);
    if (tx_channel.fd < 0)
        {
            return -1;
        }

    void *mapping = mmap(nullptr, sizeof(channel_buffer) * TX_BUFFER_COUNT,
        PROT_READ | PROT_WRITE, MAP_SHARED, tx_channel.fd, 0);
    if (mapping == MAP_FAILED)
        {
            std::cerr << "Failed to mmap DMA tx channel\n";
            close(tx_channel.fd);
            tx_channel.fd = -1;
            return -1;
        }

    tx_channel.buf_ptr = static_cast<channel_buffer *>(mapping);
    return 0;
}

int8_t *Fpga_DMA::get_buffer_address() const
{
    if (tx_channel.fd < 0 || tx_channel.buf_ptr == nullptr)
        {
            std::cerr << "DMA device is not open\n";
            return nullptr;
        }
    return tx_channel.buf_ptr[0].buffer;
}

uint32_t Fpga_DMA::get_buffer_size() const
{
    return DMA_MAX_BUFFER_SIZE;
}

int Fpga_DMA::DMA_write(int nbytes) const
{
    if (tx_channel.fd < 0 || tx_channel.buf_ptr == nullptr)
        {
            std::cerr << "DMA device is not open\n";
            return -1;
        }
    if (nbytes <= 0 || static_cast<uint32_t>(nbytes) > DMA_MAX_BUFFER_SIZE)
        {
            std::cerr << "Invalid DMA transfer size\n";
            return -1;
        }

    int buffer_id = 0;
    tx_channel.buf_ptr[0].length = static_cast<unsigned int>(nbytes);

    // Start the transfer. These ioctl definitions must match the driver.
    if (ioctl(tx_channel.fd, _IOW('a', 'b', int32_t *), &buffer_id))
        {
            std::cerr << "Error starting tx DMA transfer\n";
            return -1;
        }

    // Wait for completion.
    if (ioctl(tx_channel.fd, _IOW('a', 'a', int32_t *), &buffer_id))
        {
            std::cerr << "Error detecting end of DMA transfer\n";
            return -1;
        }

    if (buffer_id < 0 || static_cast<uint32_t>(buffer_id) >= TX_BUFFER_COUNT)
        {
            std::cerr << "Invalid DMA buffer ID\n";
            return -1;
        }
    if (tx_channel.buf_ptr[buffer_id].status != channel_buffer::PROXY_NO_ERROR)
        {
            std::cerr << "Proxy DMA Tx transfer error\n";
            return -1;
        }
    return 0;
}

int Fpga_DMA::DMA_close()
{
    int result = 0;
    if (tx_channel.buf_ptr != nullptr)
        {
            if (munmap(tx_channel.buf_ptr, sizeof(channel_buffer) * TX_BUFFER_COUNT))
                {
                    std::cerr << "Failed to unmap DMA tx channel\n";
                    result = -1;
                }
            else
                {
                    tx_channel.buf_ptr = nullptr;
                }
        }

    // Attempt descriptor cleanup even if unmapping failed.
    if (tx_channel.fd >= 0)
        {
            const int fd = tx_channel.fd;
            tx_channel.fd = -1;
            // Do not retry close(): on Linux the descriptor may already be released.
            if (close(fd))
                {
                    std::cerr << "Failed to close DMA tx channel\n";
                    result = -1;
                }
        }
    return result;
}
