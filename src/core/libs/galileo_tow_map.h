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
#include <mutex>
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
    GalileoTowMapEntry() = default;
    GalileoTowMapEntry(uint32_t entry_week, uint32_t entry_tow_ms, uint64_t entry_sample_counter)
        : week(entry_week),
          tow_ms(entry_tow_ms),
          sample_counter(entry_sample_counter)
    {
    }

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


/*!
 * \brief Deterministic counterpart of galileo_tow_map. The Galileo E1, E5a
 * and E5b telemetry decoders record their time of week at the sample stamp of
 * the symbol that sets it, and the E6 decoders look it up as of a sample that
 * all the decoders have already gone past, so that what they get does not
 * depend on thread timing.
 */
class GalileoTowHistory
{
public:
    static GalileoTowHistory& instance();

    GalileoTowHistory(const GalileoTowHistory&) = delete;
    GalileoTowHistory& operator=(const GalileoTowHistory&) = delete;

    /*!
     * \brief Forgets all the records. Lookups will not ask for samples more
     * than \p retention samples older than the newest record of a channel.
     */
    void reset(uint64_t retention = std::numeric_limits<uint64_t>::max());

    /*!
     * \brief Records the \p entry that \p channel publishes for satellite
     * \p prn at sample \p stamp. An invalid entry withdraws the earlier
     * entries of that channel for that satellite, and also an entry with the
     * same stamp, whatever the order in which they are recorded.
     */
    void record(uint32_t prn, int32_t channel, uint64_t stamp, const GalileoTowMapEntry& entry);

    /*!
     * \brief Newest valid entry for satellite \p prn recorded by any channel
     * at or before sample \p sample (the lowest channel on ties), or an
     * invalid entry if there is none.
     */
    GalileoTowMapEntry lookup(uint32_t prn, uint64_t sample) const;

private:
    GalileoTowHistory();

    using Timeline = std::map<uint64_t, GalileoTowMapEntry>;  // by sample stamp

    mutable std::mutex d_mutex;
    std::map<uint32_t, std::map<int32_t, Timeline>> d_records;  // by PRN and channel
    uint64_t d_retention;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_TOW_MAP_H
