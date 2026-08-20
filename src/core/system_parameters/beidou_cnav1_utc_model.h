/*!
 * \file beidou_cnav1_utc_model.h
 * \brief BeiDou B-CNAV1 UTC model (BDS-SIS-ICD-B1C-1.0 §7.12)
 * \author Wenhao Ou, 2026. ouwh(at)mail2.sysu.edu.cn
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
#ifndef GNSS_SDR_BEIDOU_CNAV1_UTC_MODEL_H
#define GNSS_SDR_BEIDOU_CNAV1_UTC_MODEL_H

#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

class Beidou_Cnav1_Utc_Model
{
public:
    double A0{};
    double A1{};
    double A2{};
    int32_t tot{};
    int32_t WN_t{};
    int32_t WN_LSF{};
    int32_t DN{};
    int32_t delta_t_LS{};   //!< Table 7-20: leap seconds before new leap effective [s]
    int32_t delta_t_LSF{};  //!< Table 7-20: leap seconds after new leap effective [s]
    bool valid{};
};

/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_CNAV1_UTC_MODEL_H
