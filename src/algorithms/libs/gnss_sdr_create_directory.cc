/*!
 * \file gnss_sdr_create_directory.cc
 * \brief Create a directory
 * \author Carles Fernandez-Prades, 2018. cfernandez(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include "gnss_sdr_create_directory.h"
#include <boost/filesystem/operations.hpp>   // for create_directories, exists
#include <boost/filesystem/path.hpp>         // for path, operator<<
#include <boost/filesystem/path_traits.hpp>  // for filesystem
#include <fstream>


bool gnss_sdr_create_directory(const std::string& foldername)
{
    std::string new_folder;
    for (auto& folder : boost::filesystem::path(foldername))
        {
            new_folder += folder.string();
            boost::system::error_code ec;
            if (!boost::filesystem::exists(new_folder))
                {
                    try
                        {
                            if (!boost::filesystem::create_directory(new_folder, ec))
                                {
                                    return false;
                                }
                        }
                    catch (std::exception& e)
                        {
                            return false;
                        }
                }
            new_folder += boost::filesystem::path::preferred_separator;
        }

    // Check if we have writing permissions
    std::string test_file = foldername + "/test_file.txt";
    std::ofstream os_test_file;
    os_test_file.open(test_file.c_str(), std::ios::out | std::ios::binary);

    if (os_test_file.is_open())
        {
            boost::system::error_code ec;
            os_test_file.close();
            try
                {
                    boost::filesystem::remove(test_file, ec);
                }
            catch (std::exception& e)
                {
                    return false;
                }
            return true;
        }
    else
        {
            os_test_file.close();
            return false;
        }
}
