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
#include <pmt/pmt.h>
#include <cstdint>
#include <stdexcept>
#include <string>

SensorDataType::value_type SensorDataType::from_string(const std::string& s)
{
    std::string str = s;
    for (char& c : str)
        {
            c = toupper(c);
        }

    if (str == "UINT64")
        {
            return SensorDataType::UINT64;
        }
    else if (str == "FLOAT")
        {
            return SensorDataType::FLOAT;
        }
    throw std::runtime_error{"Unknown sensor data type: " + s};
}

std::string SensorDataType::to_string(const SensorDataType::value_type& v)
{
    switch (v)
        {
        case SensorDataType::UINT64:
            return "UINT64";
        case SensorDataType::FLOAT:
            return "FLOAT";
        default:
            return "UNKNOWN SENSOR";
        }
}

uint64_t SensorDataType::get_size(const SensorDataType::value_type& v)
{
    switch (v)
        {
        case SensorDataType::UINT64:
            return sizeof(uint64_t);
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
        case SensorDataType::UINT64:
            return pmt::from_uint64(*static_cast<uint64_t*>(value));
        case SensorDataType::FLOAT:
            return pmt::from_float(*static_cast<float*>(value));
        default:
            throw std::runtime_error{"Unknown sensor data type: " + to_string(v)};
        }
}
