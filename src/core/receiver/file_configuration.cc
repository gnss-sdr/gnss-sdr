/*!
 * \file file_configuration.cc
 * \brief Implementation of the interface ConfigurationInterface that reads the
 * configuration from a file.
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

#include "file_configuration.h"
#include "in_memory_configuration.h"
#include "INIReader.h"
#include "string_converter.h"
#include <glog/logging.h>
#include <string>


using google::LogMessage;

FileConfiguration::FileConfiguration(std::string filename)
{
    filename_ = filename;
    init();
}


FileConfiguration::FileConfiguration()
{
    filename_ = "./default_config_file.txt";
    init();
}


FileConfiguration::~FileConfiguration()
{
    LOG(INFO) << "Destructor called";
}


std::string FileConfiguration::property(std::string property_name, std::string default_value)
{
    if (overrided_->is_present(property_name))
        {
            return overrided_->property(property_name, default_value);
        }
    else
        {
            return ini_reader_->Get("GNSS-SDR", property_name, default_value);
        }
}


bool FileConfiguration::property(std::string property_name, bool default_value)
{
    if (overrided_->is_present(property_name))
        {
            return overrided_->property(property_name, default_value);
        }
    else
        {
            std::string empty = "";
            return converter_->convert(property(property_name, empty), default_value);
        }
}


long FileConfiguration::property(std::string property_name, long default_value)
{
    if (overrided_->is_present(property_name))
        {
            return overrided_->property(property_name, default_value);
        }
    else
        {
            std::string empty = "";
            return converter_->convert(property(property_name, empty), default_value);
        }
}


int FileConfiguration::property(std::string property_name, int default_value)
{
    if (overrided_->is_present(property_name))
        {
            return overrided_->property(property_name, default_value);
        }
    else
        {
            std::string empty = "";
            return converter_->convert(property(property_name, empty), default_value);
        }
}


unsigned int FileConfiguration::property(std::string property_name, unsigned int default_value)
{
    if (overrided_->is_present(property_name))
        {
            return overrided_->property(property_name, default_value);
        }
    else
        {
            std::string empty = "";
            return converter_->convert(property(property_name, empty), default_value);
        }
}


unsigned short FileConfiguration::property(std::string property_name, unsigned short default_value)
{
    if (overrided_->is_present(property_name))
        {
            return overrided_->property(property_name, default_value);
        }
    else
        {
            std::string empty = "";
            return converter_->convert(property(property_name, empty), default_value);
        }
}


float FileConfiguration::property(std::string property_name, float default_value)
{
    if (overrided_->is_present(property_name))
        {
            return overrided_->property(property_name, default_value);
        }
    else
        {
            std::string empty = "";
            return converter_->convert(property(property_name, empty), default_value);
        }
}


double FileConfiguration::property(std::string property_name, double default_value)
{
    if (overrided_->is_present(property_name))
        {
            return overrided_->property(property_name, default_value);
        }
    else
        {
            std::string empty = "";
            return converter_->convert(property(property_name, empty), default_value);
        }
}


void FileConfiguration::set_property(std::string property_name, std::string value)
{
    overrided_->set_property(property_name, value);
}


void FileConfiguration::init()
{
    std::unique_ptr<StringConverter> converter_(new StringConverter);
    overrided_ = std::make_shared<InMemoryConfiguration>();
    ini_reader_ = std::make_shared<INIReader>(filename_);
    error_ = ini_reader_->ParseError();
    if (error_ == 0)
        {
            DLOG(INFO) << "Configuration file " << filename_ << " opened with no errors";
        }
    else if (error_ > 0)
        {
            LOG(WARNING) << "Configuration file " << filename_ << " contains errors in line " << error_;
        }
    else
        {
            LOG(WARNING) << "Unable to open configuration file " << filename_;
        }
}
