/*!
 * \file in_memory_configuration.cc
 * \brief This implementation accepts configuration parameters upon instantiation and
 * it is intended to be used in unit testing.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
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


#include "in_memory_configuration.h"
#include "string_converter.h"
#include <memory>
#include <utility>


InMemoryConfiguration::InMemoryConfiguration()
{
    std::unique_ptr<StringConverter> converter_(new StringConverter);
}


InMemoryConfiguration::~InMemoryConfiguration()
{
    properties_.clear();
}


std::string InMemoryConfiguration::property(std::string property_name, std::string default_value)
{
    std::map<std::string, std::string>::iterator iter = properties_.find(property_name);
    if (iter != properties_.end())
        {
            return iter->second;
        }
    else
        {
            return default_value;
        }
}


bool InMemoryConfiguration::property(std::string property_name, bool default_value)
{
    std::string empty = "";
    return converter_->convert(property(property_name, empty), default_value);
}


long InMemoryConfiguration::property(std::string property_name, long default_value)
{
    std::string empty = "";
    return converter_->convert(property(property_name, empty), default_value);
}


int InMemoryConfiguration::property(std::string property_name, int default_value)
{
    std::string empty = "";
    return converter_->convert(property(property_name, empty), default_value);
}


unsigned int InMemoryConfiguration::property(std::string property_name, unsigned int default_value)
{
    std::string empty = "";
    return converter_->convert(property(property_name, empty), default_value);
}


unsigned short InMemoryConfiguration::property(std::string property_name, unsigned short default_value)
{
    std::string empty = "";
    return converter_->convert(property(property_name, empty), default_value);
}


float InMemoryConfiguration::property(std::string property_name, float default_value)
{
    std::string empty = "";
    return converter_->convert(property(property_name, empty), default_value);
}


double InMemoryConfiguration::property(std::string property_name, double default_value)
{
    std::string empty = "";
    return converter_->convert(property(property_name, empty), default_value);
}


void InMemoryConfiguration::set_property(std::string property_name, std::string value)
{
    properties_.insert(std::make_pair(property_name, value));
}


bool InMemoryConfiguration::is_present(std::string property_name)
{
    return (properties_.find(property_name) != properties_.end());
}
