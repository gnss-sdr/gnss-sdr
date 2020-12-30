/*!
 * \file fpga_dynamic_bit_selection.cc
 * \brief Dynamic Bit Selection in the received signal.
 * \authors <ul>
 *    <li> Marc Majoral, 2020. mmajoral(at)cttc.es
 * </ul>
 *
 * Class that controls the Dynamic Bit Selection in the FPGA.
 *
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

#include "fpga_dynamic_bit_selection.h"
#include <glog/logging.h>
#include <fcntl.h>     // for open, O_RDWR, O_SYNC
#include <iostream>    // for cout
#include <sys/mman.h>  // for mmap

Fpga_dynamic_bit_selection::Fpga_dynamic_bit_selection(const std::string &device_name1, const std::string &device_name2)
{
    // dynamic bits selection corresponding to frequency band 1
    if ((d_device_descriptor1 = open(device_name1.c_str(), O_RDWR | O_SYNC)) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio" << device_name1;
        }
    d_map_base1 = reinterpret_cast<volatile unsigned *>(mmap(nullptr, FPGA_PAGE_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, d_device_descriptor1, 0));

    if (d_map_base1 == reinterpret_cast<void *>(-1))
        {
            LOG(WARNING) << "Cannot map the FPGA dynamic bit selection module in frequency band 1 into tracking memory";
            std::cout << "Could not map dynamic bit selection memory corresponding to frequency band 1.\n";
        }

    // dynamic bits selection corresponding to frequency band 2
    if ((d_device_descriptor2 = open(device_name2.c_str(), O_RDWR | O_SYNC)) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio" << device_name2;
        }
    d_map_base2 = reinterpret_cast<volatile unsigned *>(mmap(nullptr, FPGA_PAGE_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, d_device_descriptor2, 0));

    if (d_map_base2 == reinterpret_cast<void *>(-1))
        {
            LOG(WARNING) << "Cannot map the FPGA dynamic bit selection module in frequency band 2 into tracking memory";
            std::cout << "Could not map dynamic bit selection memory corresponding to frequency band 2.\n";
        }

    // initialize default bit selection
    shift_out_bits_band1 = shift_out_bits_default;
    shift_out_bits_band2 = shift_out_bits_default;

    DLOG(INFO) << "Dynamic bit selection FPGA class created";
}


Fpga_dynamic_bit_selection::~Fpga_dynamic_bit_selection()
{
    close_devices();
}


void Fpga_dynamic_bit_selection::bit_selection(void)
{
    // estimated signal power corresponding to frequency band 1
    uint32_t rx_signal_power1 = d_map_base1[1];
    // estimated signal power corresponding to frequency band 2
    uint32_t rx_signal_power2 = d_map_base2[1];

    // dynamic bit selection corresponding to frequency band 1
    if (rx_signal_power1 > Power_Threshold_High)
        {
            if (shift_out_bits_band1 < shift_out_bit_max)
                {
                    shift_out_bits_band1 = shift_out_bits_band1 + 1;
                }
        }
    else if (rx_signal_power1 < Power_Threshold_Low)
        {
            if (shift_out_bits_band1 > shift_out_bits_min)
                {
                    shift_out_bits_band1 = shift_out_bits_band1 - 1;
                }
        }

    // dynamic bit selection corresponding to frequency band 2
    if (rx_signal_power2 > Power_Threshold_High)
        {
            if (shift_out_bits_band2 < shift_out_bit_max)
                {
                    shift_out_bits_band2 = shift_out_bits_band2 + 1;
                }
        }
    else if (rx_signal_power2 < Power_Threshold_Low)
        {
            if (shift_out_bits_band2 > shift_out_bits_min)
                {
                    shift_out_bits_band2 = shift_out_bits_band2 - 1;
                }
        }

    // update bit selection corresopnding to frequency band 1
    d_map_base1[0] = shift_out_bits_band1;

    // udpate bit selection corresponding to frequency band 2
    d_map_base2[0] = shift_out_bits_band2;
}


void Fpga_dynamic_bit_selection::close_devices()
{
    auto *aux = const_cast<unsigned *>(d_map_base1);
    if (munmap(static_cast<void *>(aux), FPGA_PAGE_SIZE) == -1)
        {
            std::cout << "Failed to unmap memory uio\n";
        }

    aux = const_cast<unsigned *>(d_map_base2);
    if (munmap(static_cast<void *>(aux), FPGA_PAGE_SIZE) == -1)
        {
            std::cout << "Failed to unmap memory uio\n";
        }

    close(d_device_descriptor1);
    close(d_device_descriptor2);
}
