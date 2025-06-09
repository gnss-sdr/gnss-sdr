/*!
 * \file sensor_identifier.h
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


#ifndef GNSS_SDR_SENSOR_IDENTIFIER_H
#define GNSS_SDR_SENSOR_IDENTIFIER_H

#include "sensor_data_type.h"
#include <string>

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */

struct SensorIdentifier
{
    SensorIdentifier() = delete;
    enum value_type : unsigned short
    {
        SAMPLE_STAMP = 0,  // Used internally
        CHUNK_COUNT,       // Used internally
        IMU_VEL_X,
        IMU_VEL_Y,
        IMU_VEL_Z,
        IMU_ACC_X,
        IMU_ACC_Y,
        IMU_ACC_Z,
        IMU_ANG_VEL_X,
        IMU_ANG_VEL_Y,
        IMU_ANG_VEL_Z,
        IMU_ANG_ACC_X,
        IMU_ANG_ACC_Y,
        IMU_ANG_ACC_Z,
    };

    static value_type from_string(const std::string& s);

    static std::string to_string(value_type v);

    static bool is_valid_type(value_type sensor_id, SensorDataType::value_type type);

    static SensorDataType::value_type get_internal_type(value_type sensor_id);

    static pmt::pmt_t convert_to_internal_type(value_type sensor_id, SensorDataType::value_type original_type, const pmt::pmt_t& value);
};

/** \} */
/** \} */
#endif  // GNSS_SDR_SENSOR_IDENTIFIER_H
