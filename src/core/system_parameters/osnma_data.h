/*!
 * \file osnma_data.h
 * \brief Class for Galileo OSNMA data storage
 * \author Carles Fernandez-Prades, 2020-2023 cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2023  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_OSNMA_DATA_H
#define GNSS_SDR_OSNMA_DATA_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

struct nma_header
{
    uint8_t nmas;
    uint8_t cid;
    uint8_t cpks;
    bool reserved;
};

struct dsm_header
{
    uint8_t dsm_id;
    uint8_t dsm_block_id;
};


/*!
 * \brief This class handles ONSMA data
 * See https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_OSNMA_User_ICD_for_Test_Phase_v1.0.pdf
 */
class OSNMA_data
{
public:
    OSNMA_data() = default;

    std::string itn;  // bitset<1024>
    std::string npk;
    std::string p_dp;
    nma_header d_nma_header;
    dsm_header d_dsm_header;
    uint8_t nb_dp;
    uint8_t mid;
    uint8_t npkt;
    uint8_t npktid;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_OSNMA_DATA_H
