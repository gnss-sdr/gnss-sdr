/*!
 * \file qzss_cnav_iono.h
 * \brief Interface of a QZSS CNAV ionospheric model storage
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

#ifndef GNSS_SDR_QZSS_CNAV_IONO_H
#define GNSS_SDR_QZSS_CNAV_IONO_H

#include "gps_cnav_iono.h"

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the QZSS CNAV Klobuchar ionospheric
 * model data as described in IS-QZSS-PNT-006
 *
 * Message Type 30 broadcasts the Wide Area coefficient set (the Japan area
 * set is broadcast in Message Type 61). See https://qzss.go.jp/en/technical/ps-is-qzss/
 */
class Qzss_CNAV_Iono : public Gps_CNAV_Iono
{
public:
    Qzss_CNAV_Iono() = default;  //!< Default constructor
    explicit Qzss_CNAV_Iono(const Gps_CNAV_Iono& iono) : Gps_CNAV_Iono(iono) {}
};


/** \} */
/** \} */
#endif  // GNSS_SDR_QZSS_CNAV_IONO_H
