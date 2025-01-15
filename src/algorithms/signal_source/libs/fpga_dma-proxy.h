/*!
 * \file fpga_dma-proxy.h
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

#ifndef GNSS_SDR_FPGA_DMA_PROXY_H
#define GNSS_SDR_FPGA_DMA_PROXY_H

#include <cstdint>  // for std::int8_t

/*!
 * \brief Class that controls the switch DMA in the FPGA
 */
class Fpga_DMA
{
public:
    /*!
     * \brief Default constructor.
     */
    Fpga_DMA() = default;

    /*!
     * \brief Default destructor.
     */
    ~Fpga_DMA() = default;

    /*!
     * \brief Open the DMA device driver.
     */
    int DMA_open(void);

    /*!
     * \brief Obtain DMA buffer address.
     */
    int8_t *get_buffer_address(void);  // NOLINT(readability-make-member-function-const)

    /*!
     * \brief Transfer DMA data
     */
    int DMA_write(int nbytes) const;

    /*!
     * \brief Close the DMA device driver
     */
    int DMA_close(void) const;

private:
    static const uint32_t DMA_MAX_BUFFER_SIZE = (128 * 1024); /* must match driver exactly */
    static const uint32_t TX_BUFFER_COUNT = 1;

    // channel buffer structure
    struct channel_buffer
    {
        int8_t buffer[DMA_MAX_BUFFER_SIZE];
        enum proxy_status
        {
            PROXY_NO_ERROR = 0,
            PROXY_BUSY = 1,
            PROXY_TIMEOUT = 2,
            PROXY_ERROR = 3
        } status;
        unsigned int length;
    } __attribute__((aligned(1024))); /* 64 byte alignment required for DMA, but 1024 handy for viewing memory */

    // internal DMA channel data structure
    struct channel
    {
        struct channel_buffer *buf_ptr;
        int fd;
    };

    channel tx_channel;
};
#endif  // GNSS_SDR_FPGA_DMA_PROXY_H
