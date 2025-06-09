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
    else if (str == "F32")
        {
            return SensorDataType::F32;
        }
    else if (str == "F64")
        {
            return SensorDataType::F64;
        }
    else if (str == "I32")
        {
            return SensorDataType::I32;
        }
    else if (str == "I64")
        {
            return SensorDataType::I64;
        }
    throw std::runtime_error{"Unknown sensor data type: " + s};
}

std::string SensorDataType::to_string(const SensorDataType::value_type& v)
{
    switch (v)
        {
        case SensorDataType::UINT64:
            return "UINT64";
        case SensorDataType::F32:
            return "F32";
        case SensorDataType::F64:
            return "F64";
        case SensorDataType::I32:
            return "I32";
        case SensorDataType::I64:
            return "I64";
        default:
            return "UNKNOWN TYPE";
        }
}

uint64_t SensorDataType::get_size(const SensorDataType::value_type& v)
{
    switch (v)
        {
        case SensorDataType::UINT64:
            return sizeof(uint64_t);
        case SensorDataType::F32:
            return sizeof(float);
        case SensorDataType::F64:
            return sizeof(double);
        case SensorDataType::I32:
            return sizeof(int);
        case SensorDataType::I64:
            return sizeof(long);
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
        case SensorDataType::F32:
            return pmt::from_float(*static_cast<float*>(value));
        case SensorDataType::F64:
            return pmt::from_double(*static_cast<double*>(value));
        case SensorDataType::I32:
            return pmt::from_long(long{*static_cast<int*>(value)});
        case SensorDataType::I64:
            return pmt::from_long(*static_cast<long*>(value));
        default:
            throw std::runtime_error{"Unknown sensor data type: " + to_string(v)};
        }
}
