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

#include <string>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_libs
 * \{ */

struct SensorIdentifier
{
    SensorIdentifier() = delete;
    enum value_type
    {
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

    static std::string to_string(const value_type& v);
};

/** \} */
/** \} */
#endif  // GNSS_SDR_SENSOR_IDENTIFIER_H
