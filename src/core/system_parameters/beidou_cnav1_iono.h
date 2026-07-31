/*!
 * \file beidou_cnav1_iono.h
 * \brief BeiDou B-CNAV1 ionospheric model (§7.8)
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef GNSS_SDR_BEIDOU_CNAV1_IONO_H
#define GNSS_SDR_BEIDOU_CNAV1_IONO_H

class Beidou_Cnav1_Iono
{
public:
    double alpha1{};
    double alpha2{};
    double alpha3{};
    double alpha4{};
    double alpha5{};
    double alpha6{};
    double alpha7{};
    double alpha8{};
    double alpha9{};
    bool valid{};
};

#endif
