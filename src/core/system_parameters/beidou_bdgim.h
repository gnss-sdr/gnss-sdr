/*!
 * \file beidou_bdgim.h
 * \brief BeiDou Global Ionospheric delay Model (BDGIM), BDS-SIS-ICD-B1C-1.0 §7.8
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef GNSS_SDR_BEIDOU_BDGIM_H
#define GNSS_SDR_BEIDOU_BDGIM_H

#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

/*!
 * \brief Compute BDGIM slant ionospheric delay [m] at carrier frequency freq_hz.
 * \param time_mjd Modified Julian Date (days) of the computation epoch
 * \param lat_rad User geodetic latitude [rad]
 * \param lon_rad User geodetic longitude [rad]
 * \param az_rad Satellite azimuth [rad]
 * \param el_rad Satellite elevation [rad]
 * \param alpha Broadcast BDGIM parameters alpha1..alpha9 [TECu]
 * \param freq_hz Carrier frequency [Hz]
 * \return Slant ionospheric delay [m], or 0 if inputs are invalid
 */
double beidou_bdgim_delay_m(
    double time_mjd,
    double lat_rad,
    double lon_rad,
    double az_rad,
    double el_rad,
    const double alpha[9],
    double freq_hz);

/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_BDGIM_H
