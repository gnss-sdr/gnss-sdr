/*!
 * \file in_memory_configuration.cc
 * \brief This implementation accepts configuration parameters upon instantiation and
 * it is intended to be used in unit testing.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
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


#include "in_memory_configuration.h"
#include "string_converter.h"
#include <utility>


InMemoryConfiguration::InMemoryConfiguration()
{
    std::unique_ptr<StringConverter> converter_aux(new StringConverter);
    converter_ = std::move(converter_aux);
}


InMemoryConfiguration::~InMemoryConfiguration()
{
    properties_.clear();
}


std::string InMemoryConfiguration::property(std::string property_name, std::string default_value)
{
    auto iter = properties_.find(property_name);
    if (iter != properties_.end())
        {
            return iter->second;
        }
    return default_value;
}


bool InMemoryConfiguration::property(std::string property_name, bool default_value)
{
    std::string empty;
    return converter_->convert(property(property_name, empty), default_value);
}


int64_t InMemoryConfiguration::property(std::string property_name, int64_t default_value)
{
    std::string empty;
    return converter_->convert(property(property_name, empty), default_value);
}


uint64_t InMemoryConfiguration::property(std::string property_name, uint64_t default_value)
{
    std::string empty;
    return converter_->convert(property(property_name, empty), default_value);
}


int32_t InMemoryConfiguration::property(std::string property_name, int32_t default_value)
{
    std::string empty;
    return converter_->convert(property(property_name, empty), default_value);
}


uint32_t InMemoryConfiguration::property(std::string property_name, uint32_t default_value)
{
    std::string empty;
    return converter_->convert(property(property_name, empty), default_value);
}


uint16_t InMemoryConfiguration::property(std::string property_name, uint16_t default_value)
{
    std::string empty;
    return converter_->convert(property(property_name, empty), default_value);
}


int16_t InMemoryConfiguration::property(std::string property_name, int16_t default_value)
{
    std::string empty;
    return converter_->convert(property(property_name, empty), default_value);
}


float InMemoryConfiguration::property(std::string property_name, float default_value)
{
    std::string empty;
    return converter_->convert(property(property_name, empty), default_value);
}


double InMemoryConfiguration::property(std::string property_name, double default_value)
{
    std::string empty;
    return converter_->convert(property(property_name, empty), default_value);
}


void InMemoryConfiguration::set_property(std::string property_name, std::string value)
{
    properties_.insert(std::make_pair(property_name, value));
}


void InMemoryConfiguration::supersede_property(const std::string& property_name, const std::string& value)
{
    properties_.erase(property_name);
    properties_.insert(std::make_pair(property_name, value));
}


bool InMemoryConfiguration::is_present(const std::string& property_name)
{
    return (properties_.find(property_name) != properties_.end());
}
