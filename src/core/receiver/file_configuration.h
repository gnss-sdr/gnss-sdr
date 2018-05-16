/*!
 * \file file_configuration.h
 * \brief A ConfigurationInterface that reads the configuration from a file.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 * This implementation has a text file as the source for the values of the parameters.
 * The file is in the INI format, containing sections and pairs of names and values.
 * For more information about the INI format, see http://en.wikipedia.org/wiki/INI_file
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


#ifndef GNSS_SDR_FILE_CONFIGURATION_H_
#define GNSS_SDR_FILE_CONFIGURATION_H_

#include "configuration_interface.h"
#include <memory>
#include <string>

class INIReader;
class StringConverter;
class InMemoryConfiguration;

/*!
 * \brief This class is an implementation of the interface ConfigurationInterface
 *
 * Derived from ConfigurationInterface, this class implements an interface
 * to a configuration file. This implementation has a text file as the source
 * for the values of the parameters.
 * The file is in the INI format, containing sections and pairs of names and values.
 * For more information about the INI format, see http://en.wikipedia.org/wiki/INI_file
 */
class FileConfiguration : public ConfigurationInterface
{
public:
    FileConfiguration(std::string filename);
    FileConfiguration();
    //! Virtual destructor
    ~FileConfiguration();
    std::string property(std::string property_name, std::string default_value);
    bool property(std::string property_name, bool default_value);
    long property(std::string property_name, long default_value);
    int property(std::string property_name, int default_value);
    unsigned int property(std::string property_name, unsigned int default_value);
    unsigned short property(std::string property_name, unsigned short default_value);
    float property(std::string property_name, float default_value);
    double property(std::string property_name, double default_value);
    void set_property(std::string property_name, std::string value);

private:
    void init();
    std::string filename_;
    std::shared_ptr<INIReader> ini_reader_;
    std::shared_ptr<InMemoryConfiguration> overrided_;
    std::unique_ptr<StringConverter> converter_;
    int error_;
};

#endif /*GNSS_SDR_FILE_CONFIGURATION_H_*/
