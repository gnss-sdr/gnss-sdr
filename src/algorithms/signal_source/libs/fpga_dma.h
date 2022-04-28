/*!
 * \file fpga_dma.h
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


#ifndef GNSS_SDR_FPGA_DMA_H
#define GNSS_SDR_FPGA_DMA_H

#include <cstdint>  // for int8_t

#define BUFFER_SIZE (128 * 1024) /* must match driver exactly */

#if INTPTR_MAX == INT64_MAX  // 64-bit processor architecture

#define TX_BUFFER_COUNT 1 /* app only, must be <= to the number in the driver */

#define FINISH_XFER _IOW('a', 'a', int32_t *)
#define START_XFER _IOW('a', 'b', int32_t *)

// channel buffer structure
struct channel_buffer
{
    int8_t buffer[BUFFER_SIZE];
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

#endif

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
    int8_t *get_buffer_address(void);
    /*!
     * \brief Transfer DMA data
     */
    int DMA_write(int nbytes);

    /*!
     * \brief Close the DMA device driver
     */
    int DMA_close(void);

private:
#if INTPTR_MAX == INT64_MAX  // 64-bit processor architecture
    channel tx_channel;
    int8_t buffer[BUFFER_SIZE];
#else  // 32-bit processor architecture
    int tx_fd;
#endif
};
#endif  // GNSS_SDR_FPGA_DMA_H
