/*!
 * \file string_converter.cc
 * \brief Implementation of a class that interprets the contents of a string
 * and converts it into different types.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
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


#include "string_converter.h"
#include <sstream>


bool StringConverter::convert(const std::string& value, bool default_value)
{
    if (value == "true")
        {
            return true;
        }
    if (value == "false")
        {
            return false;
        }

    return default_value;
}


int64_t StringConverter::convert(const std::string& value, int64_t default_value)
{
    std::stringstream stream(value);

    int64_t result;
    stream >> result;

    if (stream.fail())
        {
            return default_value;
        }

    return result;
}


uint64_t StringConverter::convert(const std::string& value, uint64_t default_value)
{
    std::stringstream stream(value);

    uint64_t result;
    stream >> result;

    if (stream.fail())
        {
            return default_value;
        }

    return result;
}


int32_t StringConverter::convert(const std::string& value, int32_t default_value)
{
    std::stringstream stream(value);

    int result;
    stream >> result;

    if (stream.fail())
        {
            return default_value;
        }

    return result;
}


uint32_t StringConverter::convert(const std::string& value, uint32_t default_value)
{
    std::stringstream stream(value);

    uint32_t result;
    stream >> result;

    if (stream.fail())
        {
            return default_value;
        }

    return result;
}


uint16_t StringConverter::convert(const std::string& value, uint16_t default_value)
{
    std::stringstream stream(value);

    uint16_t result;
    stream >> result;

    if (stream.fail())
        {
            return default_value;
        }

    return result;
}


int16_t StringConverter::convert(const std::string& value, int16_t default_value)
{
    std::stringstream stream(value);

    int16_t result;
    stream >> result;

    if (stream.fail())
        {
            return default_value;
        }

    return result;
}


float StringConverter::convert(const std::string& value, float default_value)
{
    std::stringstream stream(value);

    float result;
    stream >> result;

    if (stream.fail())
        {
            return default_value;
        }

    return result;
}


double StringConverter::convert(const std::string& value, double default_value)
{
    std::stringstream stream(value);

    double result;
    stream >> result;

    if (stream.fail())
        {
            return default_value;
        }

    return result;
}
