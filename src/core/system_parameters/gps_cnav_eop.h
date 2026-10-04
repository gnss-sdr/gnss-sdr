/*!
 * \file gps_cnav_eop.h
 * \brief Storage for GPS/QZSS CNAV Earth orientation parameters
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GPS_CNAV_EOP_H
#define GNSS_SDR_GPS_CNAV_EOP_H


#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief Storage for the Earth orientation parameters broadcast in GPS/QZSS
 * CNAV Message Type 32, as described in IS-GPS-200M.
 */
class Gps_CNAV_Eop
{
public:
    uint32_t PRN{};              //!< Transmitting satellite PRN
    double tow{};                //!< Message transmission time [s of GPS week]
    double t_eop{};              //!< EOP reference time [s of GPS week]
    double pm_x{};               //!< X-axis polar motion [arc-seconds]
    double pm_x_dot{};           //!< X-axis polar motion rate [arc-seconds/day]
    double pm_y{};               //!< Y-axis polar motion [arc-seconds]
    double pm_y_dot{};           //!< Y-axis polar motion rate [arc-seconds/day]
    double delta_ut1_gps{};      //!< UT1-GPS difference [s]
    double delta_ut1_gps_dot{};  //!< UT1-GPS difference rate [s/day]
    bool valid{};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_CNAV_EOP_H
