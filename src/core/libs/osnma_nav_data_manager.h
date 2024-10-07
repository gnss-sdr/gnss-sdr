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
    bool have_nav_data(uint32_t PRNd, uint32_t TOW, uint8_t ADKD) const;
    std::string get_navigation_data(const Tag& t) const;

    void add_navigation_data(const std::string& nav_bits, uint32_t PRNd, uint32_t TOW);
    void update_nav_data(const std::multimap<uint32_t, Tag>& tags_verified, uint8_t tag_size);
    bool have_nav_data(const std::string& nav_bits, uint32_t PRNd, uint32_t TOW);
    std::vector<OSNMA_NavData> get_verified_data();

private:
    std::map<uint32_t, std::map<uint32_t, OSNMA_NavData>> d_satellite_nav_data{};  // NavData sorted by [PRNd][TOW_start]
    const uint32_t L_t_min{40};
    const uint16_t EPH_SIZE{549};
    const uint16_t UTC_SIZE{141};
};

/** \} */
/** \} */
#endif  // GNSS_SDR_OSNMA_NAV_DATA_MANAGER_H
