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

    /*!
     * \brief Retrieves a configuration parameter as a string.
     * \param[in] property_name Name of the configuration parameter.
     * \param[in] default_value Value returned if the property is not found.
     * \return The property value as a string, or default_value if absent.
     */
    virtual std::string property(std::string property_name, std::string default_value) const = 0;

    /*!
     * \brief Retrieves a configuration parameter as a bool.
     * \param[in] property_name Name of the configuration parameter.
     * \param[in] default_value Value returned if the property is not found.
     * \return The property value as a bool, or default_value if absent.
     */
    virtual bool property(std::string property_name, bool default_value) const = 0;

    /*!
     * \brief Retrieves a configuration parameter as a signed 64-bit integer.
     * \param[in] property_name Name of the configuration parameter.
     * \param[in] default_value Value returned if the property is not found.
     * \return The property value as an int64_t, or default_value if absent.
     */
    virtual int64_t property(std::string property_name, int64_t default_value) const = 0;

    /*!
     * \brief Retrieves a configuration parameter as an unsigned 64-bit integer.
     * \param[in] property_name Name of the configuration parameter.
     * \param[in] default_value Value returned if the property is not found.
     * \return The property value as a uint64_t, or default_value if absent.
     */
    virtual uint64_t property(std::string property_name, uint64_t default_value) const = 0;

    /*!
     * \brief Retrieves a configuration parameter as a signed 32-bit integer.
     * \param[in] property_name Name of the configuration parameter.
     * \param[in] default_value Value returned if the property is not found.
     * \return The property value as an int32_t, or default_value if absent.
     */
    virtual int32_t property(std::string property_name, int32_t default_value) const = 0;

    /*!
     * \brief Retrieves a configuration parameter as an unsigned 32-bit integer.
     * \param[in] property_name Name of the configuration parameter.
     * \param[in] default_value Value returned if the property is not found.
     * \return The property value as a uint32_t, or default_value if absent.
     */
    virtual uint32_t property(std::string property_name, uint32_t default_value) const = 0;

    /*!
     * \brief Retrieves a configuration parameter as a signed 16-bit integer.
     * \param[in] property_name Name of the configuration parameter.
     * \param[in] default_value Value returned if the property is not found.
     * \return The property value as an int16_t, or default_value if absent.
     */
    virtual int16_t property(std::string property_name, int16_t default_value) const = 0;

    /*!
     * \brief Retrieves a configuration parameter as an unsigned 16-bit integer.
     * \param[in] property_name Name of the configuration parameter.
     * \param[in] default_value Value returned if the property is not found.
     * \return The property value as a uint16_t, or default_value if absent.
     */
    virtual uint16_t property(std::string property_name, uint16_t default_value) const = 0;

    /*!
     * \brief Retrieves a configuration parameter as a single-precision float.
     * \param[in] property_name Name of the configuration parameter.
     * \param[in] default_value Value returned if the property is not found.
     * \return The property value as a float, or default_value if absent.
     */
    virtual float property(std::string property_name, float default_value) const = 0;

    /*!
     * \brief Retrieves a configuration parameter as a double-precision float.
     * \param[in] property_name Name of the configuration parameter.
     * \param[in] default_value Value returned if the property is not found.
     * \return The property value as a double, or default_value if absent.
     */
    virtual double property(std::string property_name, double default_value) const = 0;

    /*!
     * \brief Sets a configuration parameter value.
     * \param[in] property_name Name of the configuration parameter.
     * \param[in] value The value to assign.
     */
    virtual void set_property(std::string property_name, std::string value) = 0;

    /*!
     * \brief Checks whether a configuration parameter exists.
     * \param[in] property_name Name of the configuration parameter.
     * \return True if the property is present, false otherwise.
     */
    virtual bool is_present(const std::string& property_name) const = 0;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_CONFIGURATION_INTERFACE_H
