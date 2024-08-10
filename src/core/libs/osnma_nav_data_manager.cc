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
void OSNMA_nav_data_Manager::add_navigation_data(const std::string& nav_bits, uint32_t PRNd, uint32_t TOW)
{
    if (not have_nav_data(nav_bits, PRNd, TOW))
        {
            _satellite_nav_data[PRNd][TOW].add_nav_data(nav_bits);
            _satellite_nav_data[PRNd][TOW].PRNd = PRNd;
            _satellite_nav_data[PRNd][TOW].set_tow_sf0(TOW);
        }
}


/**
 * @brief loops over the verified tags and updates the navigation data tag length
 */
void OSNMA_nav_data_Manager::update_nav_data(const std::multimap<uint32_t, Tag>& tags_verified, const uint8_t tag_size)
{
    // loop through all tags
    for (const auto& tag : tags_verified)
        {
            // if tag status is verified, look for corresponding OSNMA_NavData and add increase verified tag bits.
            if (tag.second.status == Tag::e_verification_status::SUCCESS)
                {
                    if (have_PRNd_nav_data(tag.second.PRN_d))
                        {
                            std::map<uint32_t, OSNMA_NavData> tow_map = _satellite_nav_data.find(tag.second.PRN_d)->second;
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
                                            _satellite_nav_data[tag.second.PRN_d][tow_it.first].verified_bits += tag_size;
                                        }
                                }
                        }
                }
        }
}


bool OSNMA_nav_data_Manager::have_PRNd_nav_data(uint32_t PRNd)
{
    // check if have data from PRNd in _satellite_nav_data
    return _satellite_nav_data.find(PRNd) != _satellite_nav_data.end();
}


std::vector<OSNMA_NavData> OSNMA_nav_data_Manager::get_verified_data()
{
    std::vector<OSNMA_NavData> result;
    for (const auto& prna : _satellite_nav_data)
        {
            for (const auto& tow_navdata : prna.second)
                {
                    if (tow_navdata.second.verified_bits >= L_t_min)
                        {
                            result.push_back(tow_navdata.second);
                            _satellite_nav_data[prna.first][tow_navdata.first].verified = true;
                        }
                }
        }
    return result;
}


bool OSNMA_nav_data_Manager::have_nav_data(uint32_t PRNd, uint32_t TOW, uint8_t ADKD)
{
    if (ADKD == 0 || ADKD == 12)
        {
            const auto it = _satellite_nav_data.find(PRNd);
            if (it != _satellite_nav_data.cend())
                {
                    const auto it2 = it->second.find(TOW);
                    if (it2 != it->second.cend() && !it->second[TOW].get_ephemeris_data().empty())
                        {
                            return true;
                        }
                }
        }
    else if (ADKD == 4)
        {
            const auto it = _satellite_nav_data.find(PRNd);
            if (it != _satellite_nav_data.cend())
                {
                    const auto it2 = it->second.find(TOW);
                    if (it2 != it->second.cend() && !it->second[TOW].get_utc_data().empty())
                        {
                            return true;
                        }
                }
        }
    return false;
}


/**
 * @brief returns OSNMA_NavData object.
 * @remarks assumes it exists (called have_nav_data before), otherwise undefined behavior
 * TODO - maybe add const promise and use find() instead? this is kinda sensitive topic.
 */
std::string OSNMA_nav_data_Manager::get_navigation_data(const Tag& tag)
{
    auto prn_it = _satellite_nav_data.find(tag.PRN_d);
    if (prn_it == _satellite_nav_data.end())
        {
            return "";
        }

    // satellite was found, check if TOW exists in inner map
    std::map<uint32_t, OSNMA_NavData> tow_map = prn_it->second;
    for (auto& tow_it : tow_map)  // note: starts with smallest (i.e. oldest) navigation dataset
        {
            // Check if current key (TOW) fulfills condition
            if ((tag.TOW - 30 * tag.cop) <= tow_it.first && tow_it.first <= tag.TOW - 30)
                {
                    if (tag.ADKD == 0 || tag.ADKD == 12)
                        {
                            if (!tow_it.second.get_ephemeris_data().empty())
                                {
                                    return tow_it.second.get_ephemeris_data();
                                }
                        }
                    else if (tag.ADKD == 4)
                        {
                            if (!tow_it.second.get_utc_data().empty())
                                {
                                    return tow_it.second.get_utc_data();
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
bool OSNMA_nav_data_Manager::have_nav_data(const std::string& nav_bits, uint32_t PRNd, uint32_t TOW)
{
    if (_satellite_nav_data.find(PRNd) != _satellite_nav_data.end())
        {
            for (auto& data_timestamp : _satellite_nav_data[PRNd])
                {
                    if (nav_bits.size() == EPH_SIZE)
                        {
                            if (data_timestamp.second.get_ephemeris_data() == nav_bits)
                                {
                                    data_timestamp.second.update_last_received_timestamp(TOW);
                                    return true;
                                }
                        }
                    else if (nav_bits.size() == UTC_SIZE)
                        {
                            if (data_timestamp.second.get_utc_data() == nav_bits)
                                {
                                    data_timestamp.second.update_last_received_timestamp(TOW);
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
bool OSNMA_nav_data_Manager::have_nav_data(const Tag& t) const
{
    auto prn_it = _satellite_nav_data.find(t.PRN_d);
    if (prn_it == _satellite_nav_data.end())
        {
            return false;
        }
    // satellite was found, check if TOW exists in inner map
    std::map<uint32_t, OSNMA_NavData> tow_map = prn_it->second;
    for (auto& tow_it : tow_map)  // note: starts with smallest (i.e. oldest) navigation dataset
        {
            // Check if current key (TOW) fulfills condition
            if (t.TOW - 30 * t.cop <= tow_it.first && tow_it.first <= t.TOW - 30)
                {
                    if (t.ADKD == 0 || t.ADKD == 12)
                        {
                            if (!tow_it.second.get_ephemeris_data().empty())
                                {
                                    return true;
                                }
                        }
                    else if (t.ADKD == 4)
                        {
                            if (!tow_it.second.get_utc_data().empty())
                                {
                                    return true;
                                }
                        }
                }
        }
    return false;
}


void OSNMA_nav_data_Manager::print_status()
{
    for (const auto& satellite : _satellite_nav_data)
        {
            LOG(INFO) << "Galileo OSNMA: NavData status :: SVID=" << satellite.first;
            const auto& tow_data = satellite.second;
            for (const auto& nav_data : tow_data)
                {
                    LOG(INFO) << "Galileo OSNMA: IOD_nav=0b" << std::uppercase
                              << std::bitset<10>(nav_data.second.IOD_nav)
                              << ", TOW_start="
                              << nav_data.second.get_tow_sf0()
                              << ", TOW_last="
                              << nav_data.second.last_received_TOW
                              << ", l_t="
                              << nav_data.second.verified_bits
                              << ", PRNd="
                              << nav_data.second.PRNd
                              << ", verified="
                              << nav_data.second.verified;
                }
        }
}
