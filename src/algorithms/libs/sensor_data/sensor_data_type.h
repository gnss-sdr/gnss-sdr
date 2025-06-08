/*!
 * \file sensor_data_type.h
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


#ifndef GNSS_SDR_SENSOR_DATA_TYPE_H
#define GNSS_SDR_SENSOR_DATA_TYPE_H

#include <cstdint>
#include <pmt/pmt.h>
#include <string>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_libs
 * \{ */

struct SensorDataType
{
    SensorDataType() = delete;
    enum value_type
    {
        UINT64,
        FLOAT
    };

    static value_type from_string(const std::string& s);

    static std::string to_string(const value_type& v);

    static uint64_t get_size(const value_type& v);

    static pmt::pmt_t make_value(const value_type& v, void* value);
};


/** \} */
/** \} */
#endif  // GNSS_SDR_SENSOR_DATA_TYPE_H
