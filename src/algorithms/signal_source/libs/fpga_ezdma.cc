/*!
 * \file fpga_edma.cc
 * \brief FPGA DMA control using the ezdma (See https://github.com/jeremytrimble/ezdma).
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
#include "fpga_ezdma.h"
#include <fcntl.h>
#include <iostream>  // for std::cerr
#include <unistd.h>

int Fpga_DMA::DMA_open()
{
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
    return 0;
}


int8_t *Fpga_DMA::get_buffer_address()
{
    return buffer;
}


int Fpga_DMA::DMA_write(int nbytes) const
{
    const int num_bytes_sent = write(tx_fd, buffer, nbytes);
    if (num_bytes_sent != nbytes)
        {
            return -1;
        }
    return 0;
}


int Fpga_DMA::DMA_close() const
{
    return close(tx_fd);
}
