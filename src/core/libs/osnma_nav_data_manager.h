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

#include "osnma_data.h"  // NavData
#include <cstdint>       // uint32_t
#include <map>
#include <string>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */

/**
 * @class OSNMA_nav_data_Manager
 * @brief Class for managing OSNMA navigation data
 * @details It does good stuff
 * @remarks throw it whatever, it will improve it. Does good stuff
 */
class OSNMA_nav_data_Manager
{
public:
    OSNMA_nav_data_Manager() = default;
    bool have_nav_data(const std::string& nav_bits, uint32_t PRNd, uint32_t TOW);
    bool have_nav_data(uint32_t PRNd, uint32_t TOW, uint8_t ADKD);
    bool have_nav_data(const Tag& t) const;
    void add_navigation_data(const std::string& nav_bits, uint32_t PRNd, uint32_t TOW);  // gets the bits and adds them to the list
    std::string get_navigation_data(const Tag& t);

    void update_nav_data(const std::multimap<uint32_t, Tag>& tags_verified, const uint8_t tag_size);
    std::vector<OSNMA_NavData> get_verified_data();
    void print_status();

private:
    bool have_PRNd_nav_data(uint32_t PRNd);

    std::map<uint32_t, std::map<uint32_t, OSNMA_NavData>> _satellite_nav_data{};  // NavData sorted by [PRNd][TOW_start]
    const uint32_t L_t_min{40};
    const uint16_t EPH_SIZE{549};
    const uint16_t UTC_SIZE{141};
    const uint16_t MAX_ALLOWED_SIZE{150};  // arbitrary maximum for the navigation data container
};

/** \} */
/** \} */
#endif  // GNSS_SDR_OSNMA_NAV_DATA_MANAGER_H
