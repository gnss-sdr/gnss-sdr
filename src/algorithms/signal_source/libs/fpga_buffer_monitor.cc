/*!
 * \file fpga_buffer_monitor.cc
 * \brief Check receiver buffer overflow and monitor the status of the receiver buffers.
 * \authors <ul>
 *    <li> Marc Majoral, 2020. mmajoral(at)cttc.es
 * </ul>
 *
 * Class that checks the receiver buffer overflow flags and monitors the status of the receiver buffers.
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

#include "fpga_buffer_monitor.h"
#include <glog/logging.h>
#include <fcntl.h>     // for open, O_RDWR, O_SYNC
#include <iostream>    // for cout
#include <sys/mman.h>  // for mmap

Fpga_buffer_monitor::Fpga_buffer_monitor(const std::string &device_name, uint32_t num_freq_bands)
{
    d_num_freq_bands = num_freq_bands;

    // open device descriptor
    if ((d_device_descriptor = open(device_name.c_str(), O_RDWR | O_SYNC)) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio" << device_name;
        }

    // device memory map
    d_map_base = reinterpret_cast<volatile unsigned *>(mmap(nullptr, FPGA_PAGE_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, d_device_descriptor, 0));

    if (d_map_base == reinterpret_cast<void *>(-1))
        {
            LOG(WARNING) << "Cannot map the FPGA buffer monitor module";
            std::cout << "Could not map the FPGA buffer monitor \n";
        }

    // sanity check: check test register
    if (buffer_monitor_test_register() < 0)
        {
            LOG(WARNING) << "FPGA buffer monitor test register sanity check failed";
            std::cout << "FPGA buffer monitor test register sanity check failed\n";
        }
    else
        {
            LOG(INFO) << "FPGA buffer monitor test register sanity check success !";
        }

    DLOG(INFO) << "FPGA buffer monitor class created";

    // initialize maximum buffer occupancy in case buffer monitoring is enabled
    max_buff_occ_freq_band_0 = 0;
    max_buff_occ_freq_band_1 = 0;
}


Fpga_buffer_monitor::~Fpga_buffer_monitor()
{
    close_device();
}


void Fpga_buffer_monitor::check_buffer_overflow()
{
    // check buffer overflow flags
    uint32_t buffer_overflow_status = d_map_base[overflow_flags_reg_addr];

    if ((buffer_overflow_status & overflow_freq_band_0_bit_pos) != 0)
        {
            std::cout << "Buffer overflow in frequency band 0" << std::endl;
            throw std::exception();
        }

    if (d_num_freq_bands > 1)
        {
            if ((buffer_overflow_status & overflow_freq_band_1_bit_pos) != 0)
                {
                    std::cout << "Buffer overflow in frequency band 1" << std::endl;
                    throw std::exception();
                }
        }
}


void Fpga_buffer_monitor::monitor_buffer_status(void)
{
    uint32_t current_buff_occ_freq_band_0 = d_map_base[current_buff_occ_freq_band_0_reg_addr] * num_sapmples_per_buffer_element;
    uint32_t temp_max_buff_occ_freq_band_0 = d_map_base[max_buff_occ_freq_band_0_reg_addr] * num_sapmples_per_buffer_element;
    if (temp_max_buff_occ_freq_band_0 > max_buff_occ_freq_band_0)
        {
            max_buff_occ_freq_band_0 = temp_max_buff_occ_freq_band_0;
        }
    std::cout << "current buffer occupancy frequency band 0 = " << current_buff_occ_freq_band_0 << " samples " << std::endl;
    std::cout << "temporary maximum buffer occupancy frequency band 0 = " << temp_max_buff_occ_freq_band_0 << " samples " << std::endl;
    std::cout << "maximum buffer occupancy frequency band 0 = " << max_buff_occ_freq_band_0 << " samples " << std::endl;

    if (d_num_freq_bands > 1)
        {
            uint32_t current_buff_occ_freq_band_1 = d_map_base[current_buff_occ_freq_band_1_reg_addr] * num_sapmples_per_buffer_element;
            uint32_t temp_max_buff_occ_freq_band_1 = d_map_base[max_buff_occ_freq_band_1_reg_addr] * num_sapmples_per_buffer_element;
            if (temp_max_buff_occ_freq_band_1 > max_buff_occ_freq_band_1)
                {
                    max_buff_occ_freq_band_1 = temp_max_buff_occ_freq_band_1;
                }
            std::cout << "current buffer occupancy frequency band 1 = " << current_buff_occ_freq_band_1 << " samples " << std::endl;
            std::cout << "temporary maximum buffer occupancy frequency band 1 = " << temp_max_buff_occ_freq_band_1 << " samples " << std::endl;
            std::cout << "maximum buffer occupancy frequency band 1 = " << max_buff_occ_freq_band_1 << " samples " << std::endl;
        }
}


int32_t Fpga_buffer_monitor::buffer_monitor_test_register(void)
{
    // write value to test register
    d_map_base[test_reg_addr] = test_register_writeval;
    // read value from test register
    uint32_t readval = d_map_base[test_reg_addr];

    if (test_register_writeval != readval)
        {
            return -1;
        }

    return 0;
}


void Fpga_buffer_monitor::close_device()
{
    auto *aux = const_cast<unsigned *>(d_map_base);
    if (munmap(static_cast<void *>(aux), FPGA_PAGE_SIZE) == -1)
        {
            std::cout << "Failed to unmap memory uio\n";
        }

    close(d_device_descriptor);
}
