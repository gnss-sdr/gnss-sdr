/*!
 * \file fpga_dma.cc
 * \brief FPGA DMA control.
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
#include <errno.h>
#include <fcntl.h>
#include <iostream>  // for operator<<
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

//Fpga_DMA::Fpga_DMA()
//{
//
//}


int Fpga_DMA::DMA_open()
{
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


    //buffer = std::vector<int8_t>(BUFFER_SIZE);
    //tx_channel.buf_ptr[0].buffer2 = buffer;
    //tx_channel.buf_ptr[0].buffer = &buffer;

    return 0;
}


int8_t *Fpga_DMA::get_buffer_address(void)
{
    return tx_channel.buf_ptr[0].buffer;
}


int Fpga_DMA::DMA_write(int nbytes)
{
    int buffer_id = 0;

    tx_channel.buf_ptr[0].length = nbytes;
    //std::cout << "transmitting " << nbytes << " bytes" << std::endl;

    // debug write values to buffer
    //for (uint k= 0; k < 512; k++)
    //{
    //	tx_channel.buf_ptr[0].buffer[k] = k;
    //}

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
    return 0;
}

int Fpga_DMA::DMA_close()
{
    if (munmap(tx_channel.buf_ptr, sizeof(struct channel_buffer)))
        {
            std::cerr << "Failed to unmap DMA tx channel " << '\n';
            return -1;
        }
    return close(tx_channel.fd);
}
