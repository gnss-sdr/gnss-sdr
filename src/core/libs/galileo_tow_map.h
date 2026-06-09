/*!
 * \file galileo_tow_map.h
 * \brief GNU Radio block that stores TOW for Galileo channels
 * \author Carles Fernandez-Prades, 2022. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GALILEO_TOW_MAP_H
#define GNSS_SDR_GALILEO_TOW_MAP_H

#include "gnss_block_interface.h"  // for gnss_shared_ptr
#include <gnuradio/block.h>        // for gr::block
#include <pmt/pmt.h>               // for pmt::pmt_t
#include <cstdint>
#include <limits>
#include <map>
#include <utility>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */

class galileo_tow_map;

using galileo_tow_map_sptr = gnss_shared_ptr<galileo_tow_map>;

galileo_tow_map_sptr galileo_tow_map_make();

constexpr uint32_t GALILEO_TOW_MAP_WEEK_MS = 604800000U;
constexpr uint32_t GALILEO_TOW_MAP_INVALID_WEEK = std::numeric_limits<uint32_t>::max();
constexpr uint32_t GALILEO_TOW_MAP_INVALID_TOW_MS = std::numeric_limits<uint32_t>::max();
constexpr uint64_t GALILEO_TOW_MAP_INVALID_SAMPLE_COUNTER = std::numeric_limits<uint64_t>::max();

struct GalileoTowMapEntry
{
    uint32_t week{GALILEO_TOW_MAP_INVALID_WEEK};
    uint32_t tow_ms{GALILEO_TOW_MAP_INVALID_TOW_MS};
    uint64_t sample_counter{GALILEO_TOW_MAP_INVALID_SAMPLE_COUNTER};
};

using GalileoTowMap = std::map<uint32_t, GalileoTowMapEntry>;
using GalileoTowMapMessage = std::pair<uint32_t, GalileoTowMapEntry>;

class galileo_tow_map : public gr::block
{
public:
    ~galileo_tow_map() = default;  //!< Default destructor

private:
    friend galileo_tow_map_sptr galileo_tow_map_make();
    galileo_tow_map();

    void msg_handler_galileo_tow_map(const pmt::pmt_t& msg);

    GalileoTowMap d_galileo_tow;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_TOW_MAP_H
