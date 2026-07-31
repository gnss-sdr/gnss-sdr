/*!
 * \file beidou_cnav1_utc_model.h
 * \brief BeiDou B-CNAV1 UTC model (§7.12)
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef GNSS_SDR_BEIDOU_CNAV1_UTC_MODEL_H
#define GNSS_SDR_BEIDOU_CNAV1_UTC_MODEL_H

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
    int32_t delta_t_LSF{};
    bool valid{};
};

#endif
