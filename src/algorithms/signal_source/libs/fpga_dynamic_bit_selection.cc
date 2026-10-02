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
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "fpga_dynamic_bit_selection.h"
#include "uio_fpga.h"
#include <fcntl.h>     // for open, O_RDWR, O_SYNC
#include <iostream>    // for cout
#include <sys/mman.h>  // for mmap
#include <unistd.h>    // for close

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

Fpga_dynamic_bit_selection::Fpga_dynamic_bit_selection(bool enable_rx1_band, bool enable_rx2_band)
    : d_map_base_freq_band_1(nullptr),
      d_map_base_freq_band_2(nullptr),
      d_dev_descr_freq_band_1(-1),
      d_dev_descr_freq_band_2(-1),
      d_shift_out_bits_freq_band_1(SHIFT_OUT_BITS_MAX_DEFAULT),
      d_shift_out_bits_freq_band_2(SHIFT_OUT_BITS_MAX_DEFAULT),
      d_shift_out_bit_max_band_1(SHIFT_OUT_BITS_MAX_DEFAULT),
      d_shift_out_bit_max_band_2(SHIFT_OUT_BITS_MAX_DEFAULT),
      d_enable_rx1_band(enable_rx1_band),
      d_enable_rx2_band(enable_rx2_band)
{
    if (d_enable_rx1_band)
        {
            if (open_device(&d_map_base_freq_band_1, d_dev_descr_freq_band_1, SELECT_FREQ_BAND_1))
                {
                    // Read the maximum supported bit shift for frequency band 1
                    initialize_device(d_map_base_freq_band_1, d_shift_out_bits_freq_band_1, d_shift_out_bit_max_band_1);
                }
            else
                {
                    LOG(FATAL) << "Cannot initialize dynamic bit selection in frequency band 1";
                }
        }
    if (d_enable_rx2_band)
        {
            if (open_device(&d_map_base_freq_band_2, d_dev_descr_freq_band_2, SELECT_FREQ_BAND_2))
                {
                    // Read the maximum supported bit shift for frequency band 2
                    initialize_device(d_map_base_freq_band_2, d_shift_out_bits_freq_band_2, d_shift_out_bit_max_band_2);
                }
            else
                {
                    LOG(FATAL) << "Cannot initialize dynamic bit selection in frequency band 2";
                }
        }
    DLOG(INFO) << "Dynamic bit selection FPGA class created";
}


Fpga_dynamic_bit_selection::~Fpga_dynamic_bit_selection()
{
    if (d_enable_rx1_band)
        {
            close_device(d_map_base_freq_band_1, d_dev_descr_freq_band_1);
        }
    if (d_enable_rx2_band)
        {
            close_device(d_map_base_freq_band_2, d_dev_descr_freq_band_2);
        }
}


void Fpga_dynamic_bit_selection::bit_selection()
{
    if (d_enable_rx1_band)
        {
            bit_selection_per_rf_band(d_map_base_freq_band_1, d_shift_out_bits_freq_band_1, d_shift_out_bit_max_band_1);
        }

    if (d_enable_rx2_band)
        {
            bit_selection_per_rf_band(d_map_base_freq_band_2, d_shift_out_bits_freq_band_2, d_shift_out_bit_max_band_2);
        }
}


bool Fpga_dynamic_bit_selection::open_device(volatile unsigned **d_map_base, int &d_dev_descr, int freq_band)
{
    // Find the UIO device for the selected frequency band.
    std::string device_name;
    const int device_num = freq_band - 1;
    if (find_uio_dev_file_name(device_name, DYN_BIT_SEL_DEV_NAME, device_num) < 0)
        {
            std::cerr << "Cannot find the FPGA uio device file corresponding to device name " << DYN_BIT_SEL_DEV_NAME << " in frequency band " << freq_band << '\n';
            return false;
        }
    // Open the dynamic bit selection device.
    if ((d_dev_descr = open(device_name.c_str(), O_RDWR | O_SYNC)) == -1)
        {
            std::cerr << "Cannot open deviceio " << device_name << std::endl;
            return false;
        }
    volatile void *map_base = reinterpret_cast<volatile unsigned *>(mmap(nullptr, FPGA_PAGE_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, d_dev_descr, 0));

    if (map_base == MAP_FAILED)
        {
            std::cerr << "Could not map dynamic bit selection memory corresponding to frequency band " << freq_band << ".\n";
            close(d_dev_descr);
            d_dev_descr = -1;
            return false;
        }
    *d_map_base = reinterpret_cast<volatile unsigned *>(map_base);

    return true;
}

void Fpga_dynamic_bit_selection::initialize_device(volatile unsigned *d_map_base, uint32_t &shift_out_bits, uint32_t &shift_out_bit_max)
{
    // Read the IP core version
    uint32_t IP_core_version = d_map_base[FPGA_IP_CORE_VERSION_REG_ADDR];

    if (IP_core_version == FPGA_DYN_BIT_SEL_IP_VERSION_1_2)
        {
            // Read the maximum supported bit shift.
            // Previous versions of the IP core are initialized to SHIFT_OUT_BITS_MAX_DEFAULT
            shift_out_bit_max = static_cast<uint32_t>(d_map_base[MAX_BIT_SHIFT_REG_ADDR]);
            shift_out_bits = shift_out_bit_max;
        }
    // Initialize dynamic bit selection to the maximum supported shift.
    d_map_base[SOBITS_REG_ADDR] = shift_out_bits;
}

void Fpga_dynamic_bit_selection::bit_selection_per_rf_band(volatile unsigned *d_map_base, uint32_t &shift_out_bits, uint32_t shift_out_bit_max)
{
    // estimated signal power
    uint32_t rx_signal_power = d_map_base[SIGPOW_REG_ADDR];

    // dynamic bit selection
    if (rx_signal_power > POWER_THRESHOLD_HIGH)
        {
            if (shift_out_bits < shift_out_bit_max)
                {
                    shift_out_bits = shift_out_bits + 1;
                }
        }
    else if (rx_signal_power < POWER_THRESHOLD_LOW)
        {
            if (shift_out_bits > SHIFT_OUT_BITS_MIN)
                {
                    shift_out_bits = shift_out_bits - 1;
                }
        }

    // Update bit selection for the selected frequency band.
    d_map_base[SOBITS_REG_ADDR] = shift_out_bits;
}


void Fpga_dynamic_bit_selection::close_device(volatile unsigned *d_map_base, int &d_dev_descr)
{
    auto *aux = const_cast<unsigned *>(d_map_base);
    if (munmap(static_cast<void *>(aux), FPGA_PAGE_SIZE) == -1)
        {
            std::cout << "Failed to unmap memory uio\n";
        }
    close(d_dev_descr);
}
