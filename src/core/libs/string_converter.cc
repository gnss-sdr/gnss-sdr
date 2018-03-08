/*!
 * \file string_converter.cc
 * \brief Implementation of a class that interprets the contents of a string
 * and converts it into different types.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include "string_converter.h"
#include <sstream>


StringConverter::StringConverter() {}

StringConverter::~StringConverter() {}

bool StringConverter::convert(const std::string& value, bool default_value)
{
    if (value.compare("true") == 0)
        {
            return true;
        }
    else if (value.compare("false") == 0)
        {
            return false;
        }
    else
        {
            return default_value;
        }
}


long StringConverter::convert(const std::string& value, long default_value)
{
    std::stringstream stream(value);

    long result;
    stream >> result;

    if (stream.fail())
        {
            return default_value;
        }
    else
        {
            return result;
        }
}


int StringConverter::convert(const std::string& value, int default_value)
{
    std::stringstream stream(value);

    int result;
    stream >> result;

    if (stream.fail())
        {
            return default_value;
        }
    else
        {
            return result;
        }
}


unsigned int StringConverter::convert(const std::string& value, unsigned int default_value)
{
    std::stringstream stream(value);

    unsigned int result;
    stream >> result;

    if (stream.fail())
        {
            return default_value;
        }
    else
        {
            return result;
        }
}


unsigned short StringConverter::convert(const std::string& value, unsigned short default_value)
{
    std::stringstream stream(value);

    unsigned short result;
    stream >> result;

    if (stream.fail())
        {
            return default_value;
        }
    else
        {
            return result;
        }
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
    else
        {
            return result;
        }
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
    else
        {
            return result;
        }
}
