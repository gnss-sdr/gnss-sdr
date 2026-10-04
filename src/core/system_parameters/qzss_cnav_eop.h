/*!
 * \file qzss_cnav_eop.h
 * \brief Storage for QZSS CNAV Earth orientation parameters
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


#ifndef GNSS_SDR_QZSS_CNAV_EOP_H
#define GNSS_SDR_QZSS_CNAV_EOP_H


#include "gps_cnav_eop.h"

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


class Qzss_CNAV_Eop : public Gps_CNAV_Eop
{
public:
    Qzss_CNAV_Eop() = default;
    explicit Qzss_CNAV_Eop(const Gps_CNAV_Eop& eop) : Gps_CNAV_Eop(eop) {}
};


/** \} */
/** \} */
#endif  // GNSS_SDR_QZSS_CNAV_EOP_H
