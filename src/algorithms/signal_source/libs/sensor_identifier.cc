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
#include <stdexcept>
#include <string>

SensorIdentifier::value_type SensorIdentifier::from_string(const std::string& s)
{
    if (s == "IMU_VEL_X")
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

std::string SensorIdentifier::to_string(const SensorIdentifier::value_type& v)
{
    switch (v)
        {
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
