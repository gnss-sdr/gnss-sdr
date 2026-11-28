/*!
 * \file osnma_nav_data_manager.h
 * \brief Class for Galileo OSNMA navigation data management
 * \author Cesare Ghionoiu-Martinez, 2020-2023 cesare.martinez(at)proton.me
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

#ifndef GNSS_SDR_OSNMA_NAV_DATA_MANAGER_H
#define GNSS_SDR_OSNMA_NAV_DATA_MANAGER_H

#include "osnma_data.h"  // for OSNMA_NavData, Tag
#include <cstdint>       // for uint32_t
#include <map>
#include <string>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */

/**
 * @class OSNMA_NavDataManager
 * @brief Class for managing OSNMA navigation data
 */
class OSNMA_NavDataManager
{
public:
    OSNMA_NavDataManager() = default;

    void log_status() const;
    bool have_nav_data(const Tag& t) const;
    std::string get_navigation_data(const Tag& t) const;

    void add_navigation_data(const std::string& nav_bits, uint32_t PRNd, uint32_t WN, uint32_t TOW);
    void update_nav_data(const std::multimap<uint32_t, Tag>& tags_verified, uint8_t tag_size);
    void reset_tag_accumulations();
    void prune_old_navigation_data(uint32_t WN, uint32_t TOW);
    bool have_nav_data(const std::string& nav_bits, uint32_t PRNd, uint32_t WN, uint32_t TOW);
    std::vector<OSNMA_NavData> get_verified_data();

private:
    using Nav_Data_Map = std::map<uint32_t, std::map<uint64_t, OSNMA_NavData>>;  // NavData sorted by [PRNd][GST_start_seconds]

    Nav_Data_Map* nav_data_map_for_adkd(uint8_t adkd);
    const Nav_Data_Map* nav_data_map_for_adkd(uint8_t adkd) const;
    Nav_Data_Map* nav_data_map_for_bits(const std::string& nav_bits);

    // The two data sets are stored apart, so that a verified tag only
    // accounts for the bits it was computed over
    Nav_Data_Map d_satellite_nav_data{};     // I/NAV Words 1 to 5, authenticated by ADKD 0 and 12
    Nav_Data_Map d_satellite_timing_data{};  // I/NAV Words 6 and 10, authenticated by ADKD 4
    const uint32_t L_t_min{40};
    const uint16_t EPH_SIZE{549};
    const uint16_t UTC_SIZE{141};
};

/** \} */
/** \} */
#endif  // GNSS_SDR_OSNMA_NAV_DATA_MANAGER_H
