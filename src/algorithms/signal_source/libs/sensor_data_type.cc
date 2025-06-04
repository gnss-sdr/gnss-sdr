/*!
 * \file sensor_data_type.cc
 * \brief
 * \author Victor Castillo, 2025. victorcastilloaguero(at).gmail.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "sensor_data_type.h"
#include <cstdint>
#include <pmt/pmt.h>
#include <stdexcept>
#include <string>

SensorDataType::value_type SensorDataType::from_string(const std::string& s)
{
    if (s == "float" or s == "FLOAT")
        {
            return SensorDataType::FLOAT;
        }
    throw std::runtime_error{"Unknown sensor data type: " + s};
}

std::string SensorDataType::to_string(const SensorDataType::value_type& v)
{
    switch (v)
        {
        case SensorDataType::FLOAT:
            return "float";
        default:
            return "UNKNOWN SENSOR";
        }
}

uint64_t SensorDataType::get_size(const SensorDataType::value_type& v)
{
    switch (v)
        {
        case SensorDataType::FLOAT:
            return sizeof(float);
        default:
            return 0UL;
        }
}

pmt::pmt_t SensorDataType::make_value(const SensorDataType::value_type& v, void* value)
{
    switch (v)
        {
        case SensorDataType::FLOAT:
            return pmt::from_float(*static_cast<float*>(value));
        default:
            throw std::runtime_error{"Unknown sensor data type: " + to_string(v)};
        }
}
