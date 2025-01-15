/*!
 * \file string_converter.h
 * \brief Interface of a class that interprets the contents of a string
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


#ifndef GNSS_SDR_STRING_CONVERTER_H
#define GNSS_SDR_STRING_CONVERTER_H

#include <cstdint>
#include <string>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */


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


/** \} */
/** \} */
#endif  // GNSS_SDR_STRING_CONVERTER_H
