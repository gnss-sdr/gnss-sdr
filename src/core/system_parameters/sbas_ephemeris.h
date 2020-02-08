/*!
 * \file sbas_ephemeris.h
 * \brief  Interface of a SBAS REFERENCE LOCATION storage
 * \author Daniel Fehr, 2013. daniel.co(at)bluewin.ch
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
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_SBAS_EPHEMERIS_H_
#define GNSS_SDR_SBAS_EPHEMERIS_H_

#include <ostream>

/*!
 * \brief This class stores SBAS SV ephemeris data
 *
 */
class Sbas_Ephemeris
{
public:
    void print(std::ostream &out);
    int i_prn;             //!< PRN number
    int i_t0;              //!< Reference epoch time (GPST)
    double d_tof;          //!< Time of message frame (GPST)
    int i_sv_ura;          //!< SV accuracy (URA index), not standardized
    bool b_sv_do_not_use;  //!< Health status (false:do not use / true:usable)
    double d_pos[3];       //!< Satellite position (m) (ECEF)
    double d_vel[3];       //!< Satellite velocity (m/s) (ECEF)
    double d_acc[3];       //!< Satellite acceleration (m/s^2) (ECEF)
    double d_af0;          //!< Satellite clock-offset (s)
    double d_af1;          //!< Satellite drift (s/s)
};


#endif  // GNSS_SDR_SBAS_EPHEMERIS_H_
