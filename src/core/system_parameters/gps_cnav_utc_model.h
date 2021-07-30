/*!
 * \file gps_cnav_utc_model.h
 * \brief  Interface of a GPS CNAV UTC MODEL storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_GPS_CNAV_UTC_MODEL_H
#define GNSS_SDR_GPS_CNAV_UTC_MODEL_H

#include "gps_utc_model.h"

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the GPS UTC MODEL data as described in in IS-GPS-200M
 *
 * See https://www.gps.gov/technical/icwg/IS-GPS-200M.pdf Appendix III
 */
class Gps_CNAV_Utc_Model : public Gps_Utc_Model
{
public:
    Gps_CNAV_Utc_Model() = default;  //!< Default constructor
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_CNAV_UTC_MODEL_H
