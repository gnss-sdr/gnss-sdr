/*!
 * \file Galileo_FNAV.h
 * \brief Galileo FNAV mesage constants
 * \author Carles Fernandez, 2020. cfernandez(at)cttc.es
 *
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

#ifndef GNSS_SDR_GALILEO_FNAV_H
#define GNSS_SDR_GALILEO_FNAV_H

#include "MATH_CONSTANTS.h"
#include <cstdint>
#include <utility>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


const std::vector<std::pair<int32_t, int32_t>> FNAV_PAGE_TYPE_BIT({{1, 6}});

/* WORD 1 iono corrections. FNAV (Galileo E5a message)*/
const std::vector<std::pair<int32_t, int32_t>> FNAV_SV_ID_PRN_1_BIT({{7, 6}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_IO_DNAV_1_BIT({{13, 10}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_T0C_1_BIT({{23, 14}});
constexpr int32_t FNAV_T0C_1_LSB = 60;
const std::vector<std::pair<int32_t, int32_t>> FNAV_AF0_1_BIT({{37, 31}});
constexpr double FNAV_AF0_1_LSB = TWO_N34;
const std::vector<std::pair<int32_t, int32_t>> FNAV_AF1_1_BIT({{68, 21}});
constexpr double FNAV_AF1_1_LSB = TWO_N46;
const std::vector<std::pair<int32_t, int32_t>> FNAV_AF2_1_BIT({{89, 6}});
constexpr double FNAV_AF2_1_LSB = TWO_N59;
const std::vector<std::pair<int32_t, int32_t>> FNAV_SISA_1_BIT({{95, 8}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_AI0_1_BIT({{103, 11}});
constexpr double FNAV_AI0_1_LSB = TWO_N2;
const std::vector<std::pair<int32_t, int32_t>> FNAV_AI1_1_BIT({{114, 11}});
constexpr double FNAV_AI1_1_LSB = TWO_N8;
const std::vector<std::pair<int32_t, int32_t>> FNAV_AI2_1_BIT({{125, 14}});
constexpr double FNAV_AI2_1_LSB = TWO_N15;
const std::vector<std::pair<int32_t, int32_t>> FNAV_REGION1_1_BIT({{139, 1}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_REGION2_1_BIT({{140, 1}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_REGION3_1_BIT({{141, 1}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_REGION4_1_BIT({{142, 1}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_REGION5_1_BIT({{143, 1}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_BGD_1_BIT({{144, 10}});
constexpr double FNAV_BGD_1_LSB = TWO_N32;
const std::vector<std::pair<int32_t, int32_t>> FNAV_E5AHS_1_BIT({{154, 2}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_WN_1_BIT({{156, 12}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_TOW_1_BIT({{168, 20}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_E5ADVS_1_BIT({{188, 1}});

// WORD 2 Ephemeris (1/3)
const std::vector<std::pair<int32_t, int32_t>> FNAV_IO_DNAV_2_BIT({{7, 10}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_M0_2_BIT({{17, 32}});
constexpr double FNAV_M0_2_LSB = PI_TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> FNAV_OMEGADOT_2_BIT({{49, 24}});
constexpr double FNAV_OMEGADOT_2_LSB = PI_TWO_N43;
const std::vector<std::pair<int32_t, int32_t>> FNAV_E_2_BIT({{73, 32}});
constexpr double FNAV_E_2_LSB = TWO_N33;
const std::vector<std::pair<int32_t, int32_t>> FNAV_A12_2_BIT({{105, 32}});
constexpr double FNAV_A12_2_LSB = TWO_N19;
const std::vector<std::pair<int32_t, int32_t>> FNAV_OMEGA0_2_BIT({{137, 32}});
constexpr double FNAV_OMEGA0_2_LSB = PI_TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> FNAV_IDOT_2_BIT({{169, 14}});
constexpr double FNAV_IDOT_2_LSB = PI_TWO_N43;
const std::vector<std::pair<int32_t, int32_t>> FNAV_WN_2_BIT({{183, 12}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_TOW_2_BIT({{195, 20}});

// WORD 3 Ephemeris (2/3)
const std::vector<std::pair<int32_t, int32_t>> FNAV_IO_DNAV_3_BIT({{7, 10}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_I0_3_BIT({{17, 32}});
constexpr double FNAV_I0_3_LSB = PI_TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> FNAV_W_3_BIT({{49, 32}});
constexpr double FNAV_W_3_LSB = PI_TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> FNAV_DELTAN_3_BIT({{81, 16}});
constexpr double FNAV_DELTAN_3_LSB = PI_TWO_N43;
const std::vector<std::pair<int32_t, int32_t>> FNAV_CUC_3_BIT({{97, 16}});
constexpr double FNAV_CUC_3_LSB = TWO_N29;
const std::vector<std::pair<int32_t, int32_t>> FNAV_CUS_3_BIT({{113, 16}});
constexpr double FNAV_CUS_3_LSB = TWO_N29;
const std::vector<std::pair<int32_t, int32_t>> FNAV_CRC_3_BIT({{129, 16}});
constexpr double FNAV_CRC_3_LSB = TWO_N5;
const std::vector<std::pair<int32_t, int32_t>> FNAV_CRS_3_BIT({{145, 16}});
constexpr double FNAV_CRS_3_LSB = TWO_N5;
const std::vector<std::pair<int32_t, int32_t>> FNAV_T0E_3_BIT({{161, 14}});
constexpr int32_t FNAV_T0E_3_LSB = 60;
const std::vector<std::pair<int32_t, int32_t>> FNAV_WN_3_BIT({{175, 12}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_TOW_3_BIT({{187, 20}});

// WORD 4 Ephemeris (3/3)
const std::vector<std::pair<int32_t, int32_t>> FNAV_IO_DNAV_4_BIT({{7, 10}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_CIC_4_BIT({{17, 16}});
constexpr double FNAV_CIC_4_LSB = TWO_N29;
const std::vector<std::pair<int32_t, int32_t>> FNAV_CIS_4_BIT({{33, 16}});
constexpr double FNAV_CIS_4_LSB = TWO_N29;
const std::vector<std::pair<int32_t, int32_t>> FNAV_A0_4_BIT({{49, 32}});
constexpr double FNAV_A0_4_LSB = TWO_N30;
const std::vector<std::pair<int32_t, int32_t>> FNAV_A1_4_BIT({{81, 24}});
constexpr double FNAV_A1_4_LSB = TWO_N50;
const std::vector<std::pair<int32_t, int32_t>> FNAV_DELTATLS_4_BIT({{105, 8}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_T0T_4_BIT({{113, 8}});
constexpr int32_t FNAV_T0T_4_LSB = 3600;
const std::vector<std::pair<int32_t, int32_t>> FNAV_W_NOT_4_BIT({{121, 8}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_W_NLSF_4_BIT({{129, 8}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_DN_4_BIT({{137, 3}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_DELTATLSF_4_BIT({{140, 8}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_T0G_4_BIT({{148, 8}});
constexpr int32_t FNAV_T0G_4_LSB = 3600;
const std::vector<std::pair<int32_t, int32_t>> FNAV_A0G_4_BIT({{156, 16}});
constexpr double FNAV_A0G_4_LSB = TWO_N35;
const std::vector<std::pair<int32_t, int32_t>> FNAV_A1G_4_BIT({{172, 12}});
constexpr double FNAV_A1G_4_LSB = TWO_N51;
const std::vector<std::pair<int32_t, int32_t>> FNAV_W_N0G_4_BIT({{184, 6}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_TOW_4_BIT({{190, 20}});

// WORD 5 Almanac SVID1 SVID2(1/2)
const std::vector<std::pair<int32_t, int32_t>> FNAV_IO_DA_5_BIT({{7, 4}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_W_NA_5_BIT({{11, 2}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_T0A_5_BIT({{13, 10}});
constexpr int32_t FNAV_T0A_5_LSB = 600;
const std::vector<std::pair<int32_t, int32_t>> FNAV_SVI_D1_5_BIT({{23, 6}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_DELTAA12_1_5_BIT({{29, 13}});
constexpr double FNAV_DELTAA12_5_LSB = TWO_N9;
const std::vector<std::pair<int32_t, int32_t>> FNAV_E_1_5_BIT({{42, 11}});
constexpr double FNAV_E_5_LSB = TWO_N16;
const std::vector<std::pair<int32_t, int32_t>> FNAV_W_1_5_BIT({{53, 16}});
constexpr double FNAV_W_5_LSB = TWO_N15;
const std::vector<std::pair<int32_t, int32_t>> FNAV_DELTAI_1_5_BIT({{69, 11}});
constexpr double FNAV_DELTAI_5_LSB = TWO_N14;
const std::vector<std::pair<int32_t, int32_t>> FNAV_OMEGA0_1_5_BIT({{80, 16}});
constexpr double FNAV_OMEGA0_5_LSB = TWO_N15;
const std::vector<std::pair<int32_t, int32_t>> FNAV_OMEGADOT_1_5_BIT({{96, 11}});
constexpr double FNAV_OMEGADOT_5_LSB = TWO_N33;
const std::vector<std::pair<int32_t, int32_t>> FNAV_M0_1_5_BIT({{107, 16}});
constexpr double FNAV_M0_5_LSB = TWO_N15;
const std::vector<std::pair<int32_t, int32_t>> FNAV_AF0_1_5_BIT({{123, 16}});
constexpr double FNAV_AF0_5_LSB = TWO_N19;
const std::vector<std::pair<int32_t, int32_t>> FNAV_AF1_1_5_BIT({{139, 13}});
constexpr double FNAV_AF1_5_LSB = TWO_N38;
const std::vector<std::pair<int32_t, int32_t>> FNAV_E5AHS_1_5_BIT({{152, 2}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_SVI_D2_5_BIT({{154, 6}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_DELTAA12_2_5_BIT({{160, 13}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_E_2_5_BIT({{173, 11}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_W_2_5_BIT({{184, 16}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_DELTAI_2_5_BIT({{200, 11}});
// const std::vector<std::pair<int,int>> FNAV_Omega012_2_5_bit({{210,4}});

// WORD 6 Almanac SVID2(1/2) SVID3
const std::vector<std::pair<int32_t, int32_t>> FNAV_IO_DA_6_BIT({{7, 4}});
// const std::vector<std::pair<int,int>> FNAV_Omega022_2_6_bit({{10,12}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_OMEGADOT_2_6_BIT({{23, 11}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_M0_2_6_BIT({{34, 16}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_AF0_2_6_BIT({{50, 16}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_AF1_2_6_BIT({{66, 13}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_E5AHS_2_6_BIT({{79, 2}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_SVI_D3_6_BIT({{81, 6}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_DELTAA12_3_6_BIT({{87, 13}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_E_3_6_BIT({{100, 11}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_W_3_6_BIT({{111, 16}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_DELTAI_3_6_BIT({{127, 11}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_OMEGA0_3_6_BIT({{138, 16}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_OMEGADOT_3_6_BIT({{154, 11}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_M0_3_6_BIT({{165, 16}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_AF0_3_6_BIT({{181, 16}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_AF1_3_6_BIT({{197, 13}});
const std::vector<std::pair<int32_t, int32_t>> FNAV_E5AHS_3_6_BIT({{210, 2}});


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_FNAV_H
