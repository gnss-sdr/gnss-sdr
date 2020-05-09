/*!
 * \file fpga_switch.cc
 * \brief Switch that connects the HW accelerator queues to the analog front end or the DMA.
 * \authors <ul>
 *    <li> Marc Majoral, 2019. mmajoral(at)cttc.cat
 *    <li> Javier Arribas, 2015. jarribas(at)cttc.es
 * </ul>
 *
 * Class that controls a switch in the FPGA
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#include "fpga_switch.h"
#include <glog/logging.h>
#include <fcntl.h>     // for open, O_RDWR, O_SYNC
#include <iostream>    // for cout, endl
#include <sys/mman.h>  // for mmap

Fpga_Switch::Fpga_Switch(const std::string &device_name)
{
    if ((d_device_descriptor = open(device_name.c_str(), O_RDWR | O_SYNC)) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio" << device_name;
        }
    d_map_base = reinterpret_cast<volatile unsigned *>(mmap(nullptr, FPGA_PAGE_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, d_device_descriptor, 0));

    if (d_map_base == reinterpret_cast<void *>(-1))
        {
            LOG(WARNING) << "Cannot map the FPGA switch module into tracking memory";
            std::cout << "Could not map switch memory." << std::endl;
        }
    else
        {
            std::cout << "Switch memory successfully mapped." << std::endl;
        }

    // sanity check : check test register
    unsigned writeval = TEST_REGISTER_TRACK_WRITEVAL;
    unsigned readval;
    readval = Fpga_Switch::fpga_switch_test_register(writeval);
    if (writeval != readval)
        {
            LOG(WARNING) << "Test register sanity check failed";
        }
    else
        {
            LOG(INFO) << "Test register sanity check success !";
        }

    DLOG(INFO) << "Switch FPGA class created";
}


Fpga_Switch::~Fpga_Switch()
{
    close_device();
}


void Fpga_Switch::set_switch_position(int32_t switch_position)
{
    d_map_base[0] = switch_position;
}


unsigned Fpga_Switch::fpga_switch_test_register(
    unsigned writeval)
{
    unsigned readval;
    // write value to test register
    d_map_base[3] = writeval;
    // read value from test register
    readval = d_map_base[3];
    // return read value
    return readval;
}


void Fpga_Switch::close_device()
{
    auto *aux = const_cast<unsigned *>(d_map_base);
    if (munmap(static_cast<void *>(aux), FPGA_PAGE_SIZE) == -1)
        {
            std::cout << "Failed to unmap memory uio" << std::endl;
        }

    close(d_device_descriptor);
}
