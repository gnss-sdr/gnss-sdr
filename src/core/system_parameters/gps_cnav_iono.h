/*!
 * \file gps_cnav_iono.h
 * \brief  Interface of a GPS CNAV IONOSPHERIC MODEL storage
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_GPS_CNAV_IONO_H
#define GNSS_SDR_GPS_CNAV_IONO_H

#include "gps_iono.h"

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the GPS IONOSPHERIC data as described in IS-GPS-200M
 *
 * See https://www.gps.gov/technical/icwg/IS-GPS-200M.pdf Appendix III
 */
class Gps_CNAV_Iono : public Gps_Iono
{
public:
    Gps_CNAV_Iono() = default;  //!< Default constructor
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_CNAV_IONO_H
