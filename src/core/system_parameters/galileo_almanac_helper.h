/*!
 * \file galileo_almanac_helper.h
 * \brief  Interface of a Galileo ALMANAC storage helper
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
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

#ifndef GNSS_SDR_GALILEO_ALMANAC_HELPER_H
#define GNSS_SDR_GALILEO_ALMANAC_HELPER_H

#include "galileo_almanac.h"
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the GALILEO ALMANAC data as described in GALILEO ICD
 *
 * See https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo-OS-SIS-ICD.pdf paragraph 5.1.10
 */
class Galileo_Almanac_Helper
{
public:
    Galileo_Almanac_Helper() = default;  //!< Default constructor

    Galileo_Almanac get_almanac(int i) const;

    // Word type 7: Almanac for SVID1 (1/2), almanac reference time and almanac reference week number
    int32_t IOD_a_7{};
    int32_t WN_a_7{};
    int32_t t0a_7{};
    int32_t SVID1_7{};
    double DELTA_A_7{};
    double e_7{};
    double omega_7{};
    double delta_i_7{};
    double Omega0_7{};
    double Omega_dot_7{};
    double M0_7{};

    // Word type 8: Almanac for SVID1 (2/2) and SVID2 (1/2)
    int32_t IOD_a_8{};
    double af0_8{};
    double af1_8{};
    int32_t E5b_HS_8{};
    int32_t E1B_HS_8{};
    int32_t E5a_HS_8{};
    int32_t SVID2_8{};
    double DELTA_A_8{};
    double e_8{};
    double omega_8{};
    double delta_i_8{};
    double Omega0_8{};
    double Omega_dot_8{};

    // Word type 9: Almanac for SVID2 (2/2) and SVID3 (1/2)
    int32_t IOD_a_9{};
    int32_t WN_a_9{};
    int32_t t0a_9{};
    double M0_9{};
    double af0_9{};
    double af1_9{};
    int32_t E5b_HS_9{};
    int32_t E1B_HS_9{};
    int32_t E5a_HS_9{};
    int32_t SVID3_9{};
    double DELTA_A_9{};
    double e_9{};
    double omega_9{};
    double delta_i_9{};

    // Word type 10: Almanac for SVID3 (2/2)
    int32_t IOD_a_10{};
    double Omega0_10{};
    double Omega_dot_10{};
    double M0_10{};
    double af0_10{};
    double af1_10{};
    int32_t E5b_HS_10{};
    int32_t E1B_HS_10{};
    int32_t E5a_HS_10{};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_ALMANAC_HELPER_H
