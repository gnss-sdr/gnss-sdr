/*!
 * \file configuration_interface.h
 * \brief This class represents an interface to configuration parameters.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 * The interface defines an accessor method that gets a parameter name as input
 * and returns the value of this parameter, a string, as output.
 * Property names are defined here. This is an abstract class for interfaces.
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

#ifndef GNSS_SDR_CONFIGURATION_INTERFACE_H_
#define GNSS_SDR_CONFIGURATION_INTERFACE_H_

#include <cstdint>
#include <string>

/*!
 * \brief This abstract class represents an interface to configuration parameters.
 *
 * The interface defines an accessor method that gets a parameter name as input
 * and returns the value of this parameter, a string, as output.
 * Property names are defined here. This is an abstract class for interfaces.
 * Since all its methods are virtual,
 * this class cannot be instantiated directly, and a subclass can only be
 * instantiated directly if all inherited pure virtual methods have been
 * implemented by that class or a parent class.
 */
class ConfigurationInterface
{
public:
    virtual ~ConfigurationInterface() = default;
    virtual std::string property(std::string property_name, std::string default_value) = 0;
    virtual bool property(std::string property_name, bool default_value) = 0;
    virtual int64_t property(std::string property_name, int64_t default_value) = 0;
    virtual uint64_t property(std::string property_name, uint64_t default_value) = 0;
    virtual int32_t property(std::string property_name, int32_t default_value) = 0;
    virtual uint32_t property(std::string property_name, uint32_t default_value) = 0;
    virtual int16_t property(std::string property_name, int16_t default_value) = 0;
    virtual uint16_t property(std::string property_name, uint16_t default_value) = 0;
    virtual float property(std::string property_name, float default_value) = 0;
    virtual double property(std::string property_name, double default_value) = 0;
    virtual void set_property(std::string property_name, std::string value) = 0;
};

#endif /*GNSS_SDR_CONFIGURATION_INTERFACE_H_*/
