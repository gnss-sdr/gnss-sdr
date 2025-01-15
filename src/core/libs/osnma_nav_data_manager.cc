/*!
 * \file osnma_nav_data_manager.cc
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

#include "osnma_nav_data_manager.h"
#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>  // for DLOG
#else
#include <absl/log/log.h>
#endif

/**
 * @brief Adds the navigation data bits to the container holding OSNMA_NavData objects.
 *
 * @param nav_bits The navigation bits.
 * @param PRNd The satellite ID.
 * @param TOW The TOW of the received data.
 */
void OSNMA_NavDataManager::add_navigation_data(const std::string& nav_bits, uint32_t PRNd, uint32_t TOW)
{
    if (not have_nav_data(nav_bits, PRNd, TOW))
        {
            d_satellite_nav_data[PRNd][TOW].add_nav_data(nav_bits);
            d_satellite_nav_data[PRNd][TOW].set_prn_d(PRNd);
            d_satellite_nav_data[PRNd][TOW].set_tow_sf0(TOW);
            d_satellite_nav_data[PRNd][TOW].set_last_received_TOW(TOW);
        }
}


/**
 * @brief loops over the verified tags and updates the navigation data tag length
 */
void OSNMA_NavDataManager::update_nav_data(const std::multimap<uint32_t, Tag>& tags_verified, uint8_t tag_size)
{
    if (d_satellite_nav_data.empty())
        {
            return;
        }
    // loop through all tags
    for (const auto& tag : tags_verified)
        {
            // if tag status is verified, look for corresponding OSNMA_NavData and add increase verified tag bits.
            if (tag.second.status == Tag::e_verification_status::SUCCESS)
                {
                    auto sat_it = d_satellite_nav_data.find(tag.second.PRN_d);
                    if (sat_it == d_satellite_nav_data.end())
                        {
                            continue;
                        }
                    auto& tow_map = sat_it->second;
                    for (auto& tow_it : tow_map)  // note: starts with smallest (i.e. oldest) navigation dataset
                        {
                            std::string nav_data;
                            if (tag.second.ADKD == 0 || tag.second.ADKD == 12)
                                {
                                    nav_data = tow_it.second.get_ephemeris_data();
                                }
                            else if (tag.second.ADKD == 4)
                                {
                                    nav_data = tow_it.second.get_utc_data();
                                }
                            // find associated OSNMA_NavData
                            if (tag.second.nav_data == nav_data)
                                {
                                    d_satellite_nav_data[tag.second.PRN_d][tow_it.first].set_update_verified_bits(tag_size);
                                }
                        }
                }
        }
}


std::vector<OSNMA_NavData> OSNMA_NavDataManager::get_verified_data()
{
    std::vector<OSNMA_NavData> result;
    for (const auto& prna : d_satellite_nav_data)
        {
            for (const auto& tow_navdata : prna.second)
                {
                    if (tow_navdata.second.get_verified_bits() >= L_t_min)
                        {
                            result.push_back(tow_navdata.second);
                            d_satellite_nav_data[prna.first][tow_navdata.first].set_verified_status(true);
                        }
                }
        }
    return result;
}


bool OSNMA_NavDataManager::have_nav_data(uint32_t PRNd, uint32_t TOW, uint8_t ADKD) const
{
    const auto sat_it = d_satellite_nav_data.find(PRNd);
    if (sat_it == d_satellite_nav_data.cend())
        {
            return false;
        }

    const auto tow_it = sat_it->second.find(TOW);
    if (tow_it == sat_it->second.cend())
        {
            return false;
        }

    switch (ADKD)
        {
        case 0:
        case 12:
            return !tow_it->second.get_ephemeris_data().empty();
        case 4:
            return !tow_it->second.get_utc_data().empty();
        default:
            return false;
        }
}


std::string OSNMA_NavDataManager::get_navigation_data(const Tag& tag) const
{
    // Check if Dummy Tag, navData is all zeros
    if (tag.cop == 0)
        {
            if (tag.ADKD == 0 || tag.ADKD == 12)
                {
                    return {std::string(549, '0')};
                }
            else if (tag.ADKD == 4)
                {
                    return {std::string(141, '0')};
                }
        }
    auto prn_it = d_satellite_nav_data.find(tag.PRN_d);
    if (prn_it == d_satellite_nav_data.end())
        {
            return "";
        }
    // satellite was found, check if TOW exists in inner map
    auto nav_data = prn_it->second.find(tag.TOW - 30);
    if (nav_data != prn_it->second.end())
        {
            if (tag.ADKD == 0 || tag.ADKD == 12)
                {
                    if (!nav_data->second.get_ephemeris_data().empty())
                        {
                            return nav_data->second.get_ephemeris_data();
                        }
                }
            else if (tag.ADKD == 4)
                {
                    if (!nav_data->second.get_utc_data().empty())
                        {
                            return nav_data->second.get_utc_data();
                        }
                }
        }
    else
        {
            for (auto rev_it = prn_it->second.rbegin(); rev_it != prn_it->second.rend(); ++rev_it)  // NOLINT(modernize-loop-convert)
                {
                    // note: starts with largest (i.e. newest) navigation dataset
                    // Check if current key (TOW) fulfills condition
                    if ((tag.TOW - 30 * tag.cop <= rev_it->first || tag.TOW - 30 * tag.cop <= rev_it->second.get_last_received_TOW()) && rev_it->first < tag.TOW)
                        {
                            if (tag.ADKD == 0 || tag.ADKD == 12)
                                {
                                    if (!rev_it->second.get_ephemeris_data().empty())
                                        {
                                            return rev_it->second.get_ephemeris_data();
                                        }
                                }
                            else if (tag.ADKD == 4)
                                {
                                    if (!rev_it->second.get_utc_data().empty())
                                        {
                                            return rev_it->second.get_utc_data();
                                        }
                                }
                        }
                }
        }
    return "";
}


/**
 * @brief Checks if the OSNMA_NavData bits are already present. In case affirmative, it updates the OSNMA_NavData 'last received' timestamp
 * @remarks e.g.: a SV may repeat the bits over several subframes. In that case, need to save them only once.
 * @param nav_bits
 * @param PRNd
 * @return
 */
bool OSNMA_NavDataManager::have_nav_data(const std::string& nav_bits, uint32_t PRNd, uint32_t TOW)
{
    if (d_satellite_nav_data.find(PRNd) != d_satellite_nav_data.end())
        {
            for (auto& data_timestamp : d_satellite_nav_data[PRNd])
                {
                    if (nav_bits.size() == EPH_SIZE)
                        {
                            if (data_timestamp.second.get_ephemeris_data() == nav_bits)
                                {
                                    data_timestamp.second.set_last_received_TOW(TOW);
                                    return true;
                                }
                        }
                    else if (nav_bits.size() == UTC_SIZE)
                        {
                            if (data_timestamp.second.get_utc_data() == nav_bits)
                                {
                                    data_timestamp.second.set_last_received_TOW(TOW);
                                    return true;
                                }
                        }
                }
        }
    return false;
}


/**
 * @brief Checks if there is a OSNMA_NavData element within the COP time interval for a Tag t
 * @param t Tag object
 * @return True if the needed navigation data for the tag is available (oldest possible OSNMA_NavData available)
 */
bool OSNMA_NavDataManager::have_nav_data(const Tag& t) const
{
    if (t.cop == 0)
        {
            return true;
        }
    auto prn_it = d_satellite_nav_data.find(t.PRN_d);
    if (prn_it == d_satellite_nav_data.end())
        {
            return false;
        }
    // satellite was found, check if TOW exists in inner map
    // try find target TOW directly first
    auto nav_data = prn_it->second.find(t.TOW - 30);
    if (nav_data != prn_it->second.end())
        {
            if (t.ADKD == 0 || t.ADKD == 12)
                {
                    if (!nav_data->second.get_ephemeris_data().empty())
                        {
                            return true;
                        }
                }
            else if (t.ADKD == 4)
                {
                    if (!nav_data->second.get_utc_data().empty())
                        {
                            return true;
                        }
                }
        }
    else
        {
            // iterate in reverse order to find matching TOW with Tag's COP value
            std::map<uint32_t, OSNMA_NavData> tow_map = prn_it->second;
            for (auto rev_it = tow_map.rbegin(); rev_it != tow_map.rend(); ++rev_it)  // NOLINT(modernize-loop-convert)
                {
                    // note: starts with largest (i.e. newest) navigation dataset
                    // Check if current key (TOW) fulfills cut-off point  and is not received after the tag
                    if ((t.TOW - 30 * t.cop <= rev_it->first || t.TOW - 30 * t.cop <= rev_it->second.get_last_received_TOW()) && rev_it->first < t.TOW)
                        {
                            if (t.ADKD == 0 || t.ADKD == 12)
                                {
                                    if (!rev_it->second.get_ephemeris_data().empty())
                                        {
                                            return true;
                                        }
                                }
                            else if (t.ADKD == 4)
                                {
                                    if (!rev_it->second.get_utc_data().empty())
                                        {
                                            return true;
                                        }
                                }
                        }
                }
        }
    return false;
}


void OSNMA_NavDataManager::log_status() const
{
    for (const auto& satellite : d_satellite_nav_data)
        {
            LOG(INFO) << "Galileo OSNMA: NavData status :: SVID=" << satellite.first;
            const auto& tow_data = satellite.second;
            for (const auto& nav_data : tow_data)
                {
                    LOG(INFO) << "Galileo OSNMA: IOD_nav=0b" << std::uppercase
                              << std::bitset<10>(nav_data.second.get_IOD_nav())
                              << ", TOW_start="
                              << nav_data.second.get_tow_sf0()
                              << ", TOW_last="
                              << nav_data.second.get_last_received_TOW()
                              << ", l_t="
                              << nav_data.second.get_verified_bits()
                              << ", PRNd="
                              << nav_data.second.get_prn_d()
                              << ", verified="
                              << nav_data.second.get_verified_status();
                }
        }
}
