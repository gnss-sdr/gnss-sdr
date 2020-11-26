/*!
 * \file uio_fpga.cc
 * \brief This library contains functions to determine the uio device driver file that
 * corresponds to a hardware accelerator device name in the FPGA
 * \author Marc Majoral, 2020. mmajoral(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "uio_fpga.h"
#include <dirent.h>  // scandir()
#include <fstream>   // ifstream
#include <iostream>  // cout
#include <sstream>   // std::stringstream

int32_t get_uio_num(std::string uio_name)
{
    size_t i = 0;
    // search first digit
    for (; i < uio_name.length(); i++)
        {
            if (isdigit(uio_name[i])) break;
        }

    // remove the first chars, which aren't digits
    std::string uio_num = uio_name.substr(i, uio_name.length() - i);

    // convert the remaining text to an integer
    return atoi(uio_num.c_str());
}

void get_uio_name(uint32_t uio_num, std::string &uio_name)
{
    std::stringstream filename;
    filename << uio_dir << uio_filename << uio_num << uio_subdir_name;

    std::ifstream infile;
    infile.open(filename.str());
    if (infile.is_open())
        {
            getline(infile, uio_name);
            if (infile.bad())
                {
                    std::cout << "Could not read the FPGA uio device information file" << std::endl;
                    throw std::exception();
                }
            infile.close();
        }
    else
        {
            std::cout << "Could not open the FPGA uio device information file" << std::endl;
            throw std::exception();
        }
}

int32_t find_uio_num(const std::string &device_name, uint32_t device_num)
{
    struct dirent **files;
    int32_t uio_num;

    // get the number of directory entries in the uio driver description system directories
    uint32_t num_dir_entries = scandir(uio_dir.c_str(), &files, 0, versionsort);

    uint32_t device_count = 0, uio_count = 0;

    // search for the requested device driver
    while (device_count < num_dir_entries)
        {
            // get the uio number corresponding to directory entry n
            uio_num = get_uio_num(files[device_count]->d_name);
            if (uio_num >= 0)  // valid uio number
                {
                    std::string nametemp;
                    get_uio_name(uio_num, nametemp);
                    if (device_name.compare(nametemp) == 0)
                        {
                            if (uio_count == device_num)
                                {
                                    return uio_num;
                                }
                            else
                                {
                                    uio_count++;
                                }
                        }
                }
            device_count++;
        }
    return -1;  // uio number not found
}

int32_t find_uio_dev_file_name(std::string &device_file_name, const std::string &device_name, uint32_t device_num)
{
    int32_t uio_num = find_uio_num(device_name, device_num);
    if (uio_num >= 0)
        {
            std::stringstream device_file_name_tmp;
            device_file_name_tmp << "/dev/uio" << uio_num;
            device_file_name = device_file_name_tmp.str();
            return 0;
        }

    return -1;
}
