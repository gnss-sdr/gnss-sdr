/*!
 * \file beidou_dnav_differential_corrections.h
 * \brief Storage for BeiDou D2 integrity and differential corrections
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

#ifndef GNSS_SDR_BEIDOU_DNAV_DIFFERENTIAL_CORRECTIONS_H
#define GNSS_SDR_BEIDOU_DNAV_DIFFERENTIAL_CORRECTIONS_H

#include <cstdint>
#include <map>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

struct Beidou_Dnav_Differential_Correction
{
    uint32_t PRN{};
    int RURAI{};               //!< Regional User Range Accuracy Index
    int UDREI{};               //!< User Differential Range Error Index
    double delta_t{};          //!< Equivalent clock correction [m]
    bool delta_t_available{};  //!< False when the 13-bit value is -4096
};


class Beidou_Dnav_Differential_Corrections
{
public:
    uint32_t source_PRN{};  //!< GEO satellite broadcasting this data set
    double SOW{};
    int Pnum2{};
    int SatH2{};
    int BDEpID{};
    bool range_corrections_valid{};
    bool ionospheric_grid_valid{};
    bool expanded_corrections{};
    std::map<int, Beidou_Dnav_Differential_Correction> corrections;  //!< Keyed by corrected SV ID
};

/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_DNAV_DIFFERENTIAL_CORRECTIONS_H
