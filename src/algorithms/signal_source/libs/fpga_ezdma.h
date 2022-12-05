/*!
 * \file fpga_ezdma.h
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


#ifndef GNSS_SDR_FPGA_EDMA_H
#define GNSS_SDR_FPGA_EDMA_H

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
    static const uint32_t DMA_MAX_BUFFER_SIZE = 4 * 16384;  // 4-channel 16384-sample buffers
    int8_t buffer[DMA_MAX_BUFFER_SIZE];
    int tx_fd;
};
#endif  // GNSS_SDR_FPGA_EDMA_H
