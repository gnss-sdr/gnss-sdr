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
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_IN_MEMORY_CONFIGURATION_H
#define GNSS_SDR_IN_MEMORY_CONFIGURATION_H

#include "configuration_interface.h"
#include "string_converter.h"
#include <cstdint>
#include <map>
#include <memory>
#include <string>


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
    ~InMemoryConfiguration();
    std::string property(std::string property_name, std::string default_value);
    bool property(std::string property_name, bool default_value);
    int64_t property(std::string property_name, int64_t default_value);
    uint64_t property(std::string property_name, uint64_t default_value);
    int32_t property(std::string property_name, int32_t default_value);
    uint32_t property(std::string property_name, uint32_t default_value);
    int16_t property(std::string property_name, int16_t default_value);
    uint16_t property(std::string property_name, uint16_t default_value);
    float property(std::string property_name, float default_value);
    double property(std::string property_name, double default_value);
    void set_property(std::string property_name, std::string value);
    void supersede_property(const std::string& property_name, const std::string& value);
    bool is_present(const std::string& property_name);

private:
    std::map<std::string, std::string> properties_;
    std::unique_ptr<StringConverter> converter_;
};

#endif  // GNSS_SDR_IN_MEMORY_CONFIGURATION_H
