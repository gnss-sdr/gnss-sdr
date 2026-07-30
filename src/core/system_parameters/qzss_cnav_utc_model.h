/*!
 * \file qzss_cnav_utc_model.h
 * \brief Interface of a QZSS CNAV UTC model storage
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_QZSS_CNAV_UTC_MODEL_H
#define GNSS_SDR_QZSS_CNAV_UTC_MODEL_H

#include "gps_cnav_utc_model.h"

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the QZSS CNAV UTC model data as
 * described in IS-QZSS-PNT-006
 *
 * The parameters share the GPS CNAV layout, but the broadcast offset refers
 * to UTC(NICT) instead of UTC(USNO). See https://qzss.go.jp/en/technical/ps-is-qzss/
 */
class Qzss_CNAV_Utc_Model : public Gps_CNAV_Utc_Model
{
public:
    Qzss_CNAV_Utc_Model() = default;  //!< Default constructor
    explicit Qzss_CNAV_Utc_Model(const Gps_CNAV_Utc_Model& utc_model) : Gps_CNAV_Utc_Model(utc_model) {}
};


/** \} */
/** \} */
#endif  // GNSS_SDR_QZSS_CNAV_UTC_MODEL_H
