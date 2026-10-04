/*!
 * \file qzss_iono.h
 * \brief Interface of a QZSS ionospheric model storage
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

#ifndef GNSS_SDR_QZSS_IONO_H
#define GNSS_SDR_QZSS_IONO_H

#include "gps_iono.h"

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the QZSS Klobuchar ionospheric model
 * data as described in IS-QZSS-PNT-006
 *
 * The coefficients share the GPS LNAV layout, but they are independently
 * determined by QZSS. See https://qzss.go.jp/en/technical/ps-is-qzss/
 */
class Qzss_Iono : public Gps_Iono
{
public:
    Qzss_Iono() = default;  //!< Default constructor
    explicit Qzss_Iono(const Gps_Iono& iono) : Gps_Iono(iono) {}
};


/** \} */
/** \} */
#endif  // GNSS_SDR_QZSS_IONO_H
