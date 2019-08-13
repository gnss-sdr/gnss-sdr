/*!
 * \file string_converter.h
 * \brief Interface of a class that interprets the contents of a string
 * and converts it into different types.
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_STRING_CONVERTER_H_
#define GNSS_SDR_STRING_CONVERTER_H_

#include <cstdint>
#include <string>

/*!
 * \brief Class that interprets the contents of a string
 * and converts it into different types.
 */
class StringConverter
{
public:
    StringConverter() = default;
    ~StringConverter() = default;

    bool convert(const std::string& value, bool default_value);
    int64_t convert(const std::string& value, int64_t default_value);
    uint64_t convert(const std::string& value, uint64_t default_value);
    int32_t convert(const std::string& value, int32_t default_value);
    uint32_t convert(const std::string& value, uint32_t default_value);
    int16_t convert(const std::string& value, int16_t default_value);
    uint16_t convert(const std::string& value, uint16_t default_value);
    float convert(const std::string& value, float default_value);
    double convert(const std::string& value, double default_value);
};

#endif /*GNSS_SDR_STRING_CONVERTER_H_*/
