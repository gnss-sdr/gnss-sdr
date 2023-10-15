/*!
 * \file fpga_dynamic_bit_selection.cc
 * \brief Dynamic Bit Selection in the received signal.
 * \authors <ul>
 *    <li> Marc Majoral, 2023. mmajoral(at)cttc.es
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
 * Copyright (C) 2010-2023  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "fpga_dynamic_bit_selection.h"
#include "uio_fpga.h"
#include <glog/logging.h>
#include <fcntl.h>     // for open, O_RDWR, O_SYNC
#include <iostream>    // for cout
#include <sys/mman.h>  // for mmap

Fpga_dynamic_bit_selection::Fpga_dynamic_bit_selection(uint32_t num_freq_bands)
    : d_num_freq_bands(num_freq_bands)
{
    d_map_base = std::vector<volatile unsigned *>(d_num_freq_bands);
    d_device_descriptors = std::vector<int>(d_num_freq_bands);
    d_shift_out_bits = std::vector<uint32_t>(d_num_freq_bands);
    for (uint32_t k = 0; k < d_num_freq_bands; k++)
        {
            // find the uio device file corresponding to the dynamic bit selector 0 module.
            std::string device_name;
            if (find_uio_dev_file_name(device_name, dyn_bit_sel_device_name, 0) < 0)
                {
                    std::cerr << "Cannot find the FPGA uio device file corresponding to device name " << dyn_bit_sel_device_name << '\n';
                    return;
                }
            // dynamic bits selection corresponding to frequency band 1
            if ((d_device_descriptors[k] = open(device_name.c_str(), O_RDWR | O_SYNC)) == -1)
                {
                    LOG(WARNING) << "Cannot open deviceio" << device_name;
                }
            d_map_base[k] = reinterpret_cast<volatile unsigned *>(mmap(nullptr, FPGA_PAGE_SIZE,
                PROT_READ | PROT_WRITE, MAP_SHARED, d_device_descriptors[k], 0));

            if (d_map_base[k] == reinterpret_cast<void *>(-1))
                {
                    LOG(WARNING) << "Cannot map the FPGA dynamic bit selection module in frequency band 1 into tracking memory";
                    std::cout << "Could not map dynamic bit selection memory corresponding to frequency band 1.\n";
                }

            // init bit selection corresopnding to frequency band 1
            d_shift_out_bits[k] = shift_out_bits_default;
            d_map_base[k][0] = d_shift_out_bits[k];
        }
    DLOG(INFO) << "Dynamic bit selection FPGA class created";
}


Fpga_dynamic_bit_selection::~Fpga_dynamic_bit_selection()
{
    close_devices();
}


void Fpga_dynamic_bit_selection::bit_selection()
{
    for (uint32_t k = 0; k < d_num_freq_bands; k++)
        {
            // estimated signal power
            uint32_t rx_signal_power = d_map_base[k][1];

            // dynamic bit selection
            if (rx_signal_power > Power_Threshold_High)
                {
                    if (d_shift_out_bits[k] < shift_out_bit_max)
                        {
                            d_shift_out_bits[k] = d_shift_out_bits[k] + 1;
                        }
                }
            else if (rx_signal_power < Power_Threshold_Low)
                {
                    if (d_shift_out_bits[k] > shift_out_bits_min)
                        {
                            d_shift_out_bits[k] = d_shift_out_bits[k] - 1;
                        }
                }

            // update bit selection corresopnding to frequency band 1
            d_map_base[k][0] = d_shift_out_bits[k];
        }
}


void Fpga_dynamic_bit_selection::close_devices()
{
    for (uint32_t k = 0; k < d_num_freq_bands; k++)
        {
            auto *aux = const_cast<unsigned *>(d_map_base[k]);
            if (munmap(static_cast<void *>(aux), FPGA_PAGE_SIZE) == -1)
                {
                    std::cout << "Failed to unmap memory uio\n";
                }
            close(d_device_descriptors[k]);
        }
}
