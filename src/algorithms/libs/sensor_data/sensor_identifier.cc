/*!
 * \file sensor_identifier.cc
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

#include "sensor_identifier.h"
#include "sensor_data_type.h"
#include <iostream>
#include <stdexcept>
#include <string>


struct ConversionEntry
{
    SensorDataType::value_type from;
    SensorDataType::value_type to;
    pmt::pmt_t (*conversion_fun)(const pmt::pmt_t&);
};

static const ConversionEntry conversion_table[] = {
    {SensorDataType::F32, SensorDataType::F32, [](const pmt::pmt_t& val) { return val; }},
    {SensorDataType::F64, SensorDataType::F32, [](const pmt::pmt_t& val) { return pmt::from_float(pmt::to_double(val)); }},
    {SensorDataType::I32, SensorDataType::F32, [](const pmt::pmt_t& val) { return pmt::from_float(pmt::to_long(val)); }},
    {SensorDataType::I64, SensorDataType::F32, [](const pmt::pmt_t& val) { return pmt::from_float(pmt::to_long(val)); }},
};

const ConversionEntry* lookup_conversion(SensorDataType::value_type from_type, SensorDataType::value_type to_type)
{
    for (const auto& conversion_entry : conversion_table)
        {
            if (conversion_entry.from == from_type && conversion_entry.to == to_type)
                {
                    return &conversion_entry;
                }
        }
    return nullptr;
}


SensorIdentifier::value_type SensorIdentifier::from_string(const std::string& s)
{
    if (s == "SAMPLE_STAMP")
        {
            return SAMPLE_STAMP;
        }
    else if (s == "CHUNK_COUNT")
        {
            return CHUNK_COUNT;
        }
    else if (s == "IMU_VEL_X")
        {
            return IMU_VEL_X;
        }
    else if (s == "IMU_VEL_Y")
        {
            return IMU_VEL_Y;
        }
    else if (s == "IMU_VEL_Z")
        {
            return IMU_VEL_Z;
        }
    else if (s == "IMU_ACC_X")
        {
            return IMU_ACC_X;
        }
    else if (s == "IMU_ACC_Y")
        {
            return IMU_ACC_Y;
        }
    else if (s == "IMU_ACC_Z")
        {
            return IMU_ACC_Z;
        }
    else if (s == "IMU_ANG_VEL_X")
        {
            return IMU_ANG_VEL_X;
        }
    else if (s == "IMU_ANG_VEL_Y")
        {
            return IMU_ANG_VEL_Y;
        }
    else if (s == "IMU_ANG_VEL_Z")
        {
            return IMU_ANG_VEL_Z;
        }
    else if (s == "IMU_ANG_ACC_X")
        {
            return IMU_ANG_ACC_X;
        }
    else if (s == "IMU_ANG_ACC_Y")
        {
            return IMU_ANG_ACC_Y;
        }
    else if (s == "IMU_ANG_ACC_Z")
        {
            return IMU_ANG_ACC_Z;
        }
    throw std::runtime_error{"Unknown sensor identifier: " + s};
}

std::string SensorIdentifier::to_string(SensorIdentifier::value_type v)
{
    switch (v)
        {
        case SAMPLE_STAMP:
            return "SAMPLE_STAMP";
        case CHUNK_COUNT:
            return "CHUNK_COUNT";
        case IMU_VEL_X:
            return "IMU_VEL_X";
        case IMU_VEL_Y:
            return "IMU_VEL_Y";
        case IMU_VEL_Z:
            return "IMU_VEL_Z";
        case IMU_ACC_X:
            return "IMU_ACC_X";
        case IMU_ACC_Y:
            return "IMU_ACC_Y";
        case IMU_ACC_Z:
            return "IMU_ACC_Z";
        case IMU_ANG_VEL_X:
            return "IMU_ANG_VEL_X";
        case IMU_ANG_VEL_Y:
            return "IMU_ANG_VEL_Y";
        case IMU_ANG_VEL_Z:
            return "IMU_ANG_VEL_Z";
        case IMU_ANG_ACC_X:
            return "IMU_ANG_ACC_X";
        case IMU_ANG_ACC_Y:
            return "IMU_ANG_ACC_Y";
        case IMU_ANG_ACC_Z:
            return "IMU_ANG_ACC_Z";
        default:
            return "UNKNOWN SENSOR";
        }
}


bool SensorIdentifier::is_valid_type(SensorIdentifier::value_type sensor_id, SensorDataType::value_type type)
{
    return nullptr != lookup_conversion(type, get_internal_type(sensor_id));
}


SensorDataType::value_type SensorIdentifier::get_internal_type(value_type sensor_id)
{
    switch (sensor_id)
        {
        case SAMPLE_STAMP:
        case CHUNK_COUNT:
            return SensorDataType::UINT64;
        case IMU_VEL_X:
        case IMU_VEL_Y:
        case IMU_VEL_Z:
        case IMU_ACC_X:
        case IMU_ACC_Y:
        case IMU_ACC_Z:
        case IMU_ANG_VEL_X:
        case IMU_ANG_VEL_Y:
        case IMU_ANG_VEL_Z:
        case IMU_ANG_ACC_X:
        case IMU_ANG_ACC_Y:
        case IMU_ANG_ACC_Z:
            return SensorDataType::F32;
        default:
            return SensorDataType::F32;
        }
}


pmt::pmt_t SensorIdentifier::convert_to_internal_type(value_type sensor_id, SensorDataType::value_type original_type, const pmt::pmt_t& value)
{
    const ConversionEntry* conversion = lookup_conversion(original_type, get_internal_type(sensor_id));
    if (conversion == nullptr)
        {
            throw std::runtime_error("Could not convert sensor value to internal type");
        }

    return conversion->conversion_fun(value);
}
