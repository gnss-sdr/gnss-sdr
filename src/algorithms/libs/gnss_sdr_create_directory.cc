/*!
 * \file gnss_sdr_create_directory.cc
 * \brief Create a directory
 * \author Carles Fernandez-Prades, 2018. cfernandez(at)cttc.es
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

#include "gnss_sdr_create_directory.h"
#include <exception>  // for exception
#include <fstream>    // for ofstream

// clang-format off
#if HAS_STD_FILESYSTEM
#include <system_error>
namespace errorlib = std;
#if HAS_STD_FILESYSTEM_EXPERIMENTAL
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif
#else
#include <boost/filesystem/operations.hpp>   // for create_directories, exists
#include <boost/filesystem/path.hpp>         // for path, operator<<
#include <boost/filesystem/path_traits.hpp>  // for filesystem
#include <boost/system/error_code.hpp>       // for error_code
namespace fs = boost::filesystem;
namespace errorlib = boost::system;
#endif
// clang-format on

bool gnss_sdr_create_directory(const std::string& foldername)
{
    std::string new_folder;
    for (const auto& folder : fs::path(foldername))
        {
            new_folder += folder.string();
            errorlib::error_code ec;
            if (!fs::exists(new_folder))
                {
                    if (!fs::create_directory(new_folder, ec))
                        {
                            return false;
                        }

                    if (static_cast<bool>(ec))
                        {
                            return false;
                        }
                }
            new_folder += fs::path::preferred_separator;
        }

    // Check if we have writing permissions
    std::string test_file = foldername + "/test_file.txt";
    std::ofstream os_test_file;
    os_test_file.open(test_file.c_str(), std::ios::out | std::ios::binary);

    if (os_test_file.is_open())
        {
            errorlib::error_code ec;
            os_test_file.close();

            if (!fs::remove(test_file, ec))
                {
                    return false;
                }
            if (static_cast<bool>(ec))
                {
                    return false;
                }
            return true;
        }

    return false;
}
