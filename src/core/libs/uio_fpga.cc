/*!
 * \file uio_fpga.cc
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

#include "uio_fpga.h"
#include <algorithm>  // sort
#include <cstdlib>    // atoi, size_t
#include <fstream>    // ifstream
#include <iostream>   // cout
#include <locale>     // isdigit
#include <sstream>    // std::stringstream
#include <vector>

#if HAS_STD_FILESYSTEM
#if HAS_STD_FILESYSTEM_EXPERIMENTAL
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif
#else
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
namespace fs = boost::filesystem;
#endif

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
            std::getline(infile, uio_name);
            if (infile.bad())
                {
                    std::cout << "Could not read the FPGA uio device information file\n";
                    throw std::exception();
                }
            infile.close();
        }
    else
        {
            std::cout << "Could not open the FPGA uio device information file\n";
            throw std::exception();
        }
}


int my_strverscmp(const char *s1, const char *s2)
{
    // from https://code.woboq.org/userspace/glibc/string/strverscmp.c.html
    const uint8_t S_N = 0x0;
    const uint8_t S_I = 0x3;
    const uint8_t S_F = 0x6;
    const uint8_t S_Z = 0x9;

    const int8_t CMP = 2;
    const int8_t LEN = 3;

    const unsigned char *p1 = (const unsigned char *)s1;
    const unsigned char *p2 = (const unsigned char *)s2;
    /* Symbol(s)    0       [1-9]   others
     Transition   (10) 0  (01) d  (00) x   */
    static const uint8_t next_state[] =
        {
            /* state    x    d    0  */
            /* S_N */ S_N, S_I, S_Z,
            /* S_I */ S_N, S_I, S_I,
            /* S_F */ S_N, S_F, S_F,
            /* S_Z */ S_N, S_F, S_Z};
    static const int8_t result_type[] =
        {
            /* state   x/x  x/d  x/0  d/x  d/d  d/0  0/x  0/d  0/0  */
            /* S_N */ CMP, CMP, CMP, CMP, LEN, CMP, CMP, CMP, CMP,
            /* S_I */ CMP, -1, -1, +1, LEN, LEN, +1, LEN, LEN,
            /* S_F */ CMP, CMP, CMP, CMP, CMP, CMP, CMP, CMP, CMP,
            /* S_Z */ CMP, +1, +1, -1, CMP, CMP, -1, CMP, CMP};
    if (p1 == p2)
        {
            return 0;
        }
    unsigned char c1 = *p1++;
    unsigned char c2 = *p2++;
    /* Hint: '0' is a digit too.  */
    int state = S_N + ((c1 == '0') + (isdigit(c1) != 0));
    int diff;
    while ((diff = c1 - c2) == 0)
        {
            if (c1 == '\0')
                {
                    return diff;
                }
            state = next_state[state];
            c1 = *p1++;
            c2 = *p2++;
            state += (c1 == '0') + (isdigit(c1) != 0);
        }
    state = result_type[state * 3 + (((c2 == '0') + (isdigit(c2) != 0)))];
    switch (state)
        {
        case CMP:
            return diff;
        case LEN:
            while (isdigit(*p1++))
                {
                    if (!isdigit(*p2++))
                        {
                            return 1;
                        }
                }
            return isdigit(*p2) ? -1 : diff;
        default:
            return state;
        }
}


bool sort_directories(fs::directory_entry a, fs::directory_entry b)
{
    int cmp = my_strverscmp(a.path().string().c_str(), b.path().string().c_str());
    return (cmp < 0);
}


int32_t find_uio_num(const std::string &device_name, uint32_t device_num)
{
    int32_t uio_num;
    uint32_t uio_count = 0;

    // search for the requested device driver
    fs::path path(uio_dir);
    std::vector<fs::directory_entry> dirs;
    for (const auto &p : fs::directory_iterator(path))
        {
            dirs.push_back(p);
        }
    std::sort(dirs.begin(), dirs.end(), sort_directories);

    for (auto &p : dirs)
        {
            // get the uio number corresponding to directory entry n
            uio_num = get_uio_num(p.path().string());
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
