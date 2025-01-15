/*!
 * \file configuration_interface.h
 * \brief This class represents an interface to configuration parameters.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 * The interface defines an accessor method that gets a parameter name as input
 * and returns the value of this parameter, a string, as output.
 * Property names are defined here. This is an abstract class for interfaces.
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

#ifndef GNSS_SDR_CONFIGURATION_INTERFACE_H
#define GNSS_SDR_CONFIGURATION_INTERFACE_H

#include <cstdint>
#include <string>

/** \addtogroup Core
 * \{ */
/** \addtogroup GNSS_Block_Interfaces
 * \{ */


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
    virtual std::string property(std::string property_name, std::string default_value) const = 0;
    virtual bool property(std::string property_name, bool default_value) const = 0;
    virtual int64_t property(std::string property_name, int64_t default_value) const = 0;
    virtual uint64_t property(std::string property_name, uint64_t default_value) const = 0;
    virtual int32_t property(std::string property_name, int32_t default_value) const = 0;
    virtual uint32_t property(std::string property_name, uint32_t default_value) const = 0;
    virtual int16_t property(std::string property_name, int16_t default_value) const = 0;
    virtual uint16_t property(std::string property_name, uint16_t default_value) const = 0;
    virtual float property(std::string property_name, float default_value) const = 0;
    virtual double property(std::string property_name, double default_value) const = 0;
    virtual void set_property(std::string property_name, std::string value) = 0;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_CONFIGURATION_INTERFACE_H
