/*!
 * \file uio_fpga.h
 * \brief This library contains functions to determine the uio device driver
 * file that corresponds to a hardware accelerator device name in the FPGA.
 * \author Marc Majoral, 2020. mmajoral(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_UIO_FPGA_H
#define GNSS_SDR_UIO_FPGA_H

#include <cstdint>
#include <string>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */

const std::string uio_dir("/sys/class/uio/");
const std::string uio_filename("uio");
const std::string uio_subdir_name("/name");

/*!
 * \brief This function finds the uio device driver device file name out of the
 * device name and the device number.
 */
int32_t find_uio_dev_file_name(std::string &device_file_name,
    const std::string &device_name,
    uint32_t device_num);


/** \} */
/** \} */
#endif  // GNSS_SDR_UIO_FPGA_H
