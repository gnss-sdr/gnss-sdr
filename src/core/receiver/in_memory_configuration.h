/*!
 * \file in_memory_configuration.h
 * \brief  A ConfigurationInterface for testing purposes.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 * This implementation accepts configuration parameters upon instantiation and
 * it is intended to be used in unit testing.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_IN_MEMORY_CONFIGURATION_H_
#define GNSS_SDR_IN_MEMORY_CONFIGURATION_H_

#include "configuration_interface.h"
#include <map>
#include <memory>
#include <string>


class StringConverter;

/*!
 * \brief  This class is an implementation of the interface ConfigurationInterface.
 *
 * This implementation accepts configuration parameters upon instantiation and
 * it is intended to be used in unit testing.
 */
class InMemoryConfiguration : public ConfigurationInterface
{
public:
    InMemoryConfiguration();
    virtual ~InMemoryConfiguration();
    std::string property(std::string property_name, std::string default_value);
    bool property(std::string property_name, bool default_value);
    long property(std::string property_name, long default_value);
    int property(std::string property_name, int default_value);
    unsigned int property(std::string property_name, unsigned int default_value);
    unsigned short property(std::string property_name, unsigned short default_value);
    float property(std::string property_name, float default_value);
    double property(std::string property_name, double default_value);
    void set_property(std::string property_name, std::string value);
    bool is_present(std::string property_name);

private:
    std::map<std::string, std::string> properties_;
    std::unique_ptr<StringConverter> converter_;
};

#endif /*GNSS_SDR_IN_MEMORY_CONFIGURATION_H_*/
