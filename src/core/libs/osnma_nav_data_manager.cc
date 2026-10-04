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

namespace
{
constexpr int64_t seconds_per_week = 604800;
constexpr int64_t nav_data_retention_s = 4 * 3600 + 600;


uint64_t gst_seconds(uint32_t WN, uint32_t TOW)
{
    return static_cast<uint64_t>(static_cast<int64_t>(WN) * seconds_per_week + static_cast<int64_t>(TOW));
}


uint64_t gst_seconds_with_offset(uint32_t WN, uint32_t TOW, int32_t offset_seconds)
{
    int64_t absolute_seconds = static_cast<int64_t>(WN) * seconds_per_week + static_cast<int64_t>(TOW) + offset_seconds;
    if (absolute_seconds < 0)
        {
            absolute_seconds = 0;
        }
    return static_cast<uint64_t>(absolute_seconds);
}


int64_t tag_seconds_minus_cop(const Tag& tag)
{
    return static_cast<int64_t>(gst_seconds(tag.WN, tag.TOW)) - static_cast<int64_t>(30) * tag.cop;
}


bool nav_data_is_before_tag(uint64_t nav_gst, const Tag& tag)
{
    return static_cast<int64_t>(nav_gst) < static_cast<int64_t>(gst_seconds(tag.WN, tag.TOW));
}


bool nav_data_is_in_cop_window(const OSNMA_NavData& nav_data, uint64_t nav_gst, const Tag& tag)
{
    const int64_t oldest_gst = tag_seconds_minus_cop(tag);
    const auto last_received_gst = static_cast<int64_t>(gst_seconds(nav_data.get_last_received_WN(), nav_data.get_last_received_TOW()));
    return (oldest_gst <= static_cast<int64_t>(nav_gst) || oldest_gst <= last_received_gst) &&
           nav_data_is_before_tag(nav_gst, tag);
}


std::string nav_data_for_tag(const OSNMA_NavData& nav_data, const Tag& tag)
{
    if (tag.ADKD == 0 || tag.ADKD == 12)
        {
            return nav_data.get_ephemeris_data();
        }
    if (tag.ADKD == 4)
        {
            return nav_data.get_utc_data();
        }
    return "";
}


bool tag_gst_is_coherent_with_accumulation(const OSNMA_NavData& nav_data, const Tag& tag)
{
    if (!nav_data.has_tag_accumulation())
        {
            return true;
        }
    const uint64_t tag_gst = gst_seconds(tag.WN, tag.TOW);
    const uint64_t first_tag_gst = nav_data.get_first_accumulated_tag_gst();
    const uint64_t last_tag_gst = nav_data.get_last_accumulated_tag_gst();
    if (tag_gst < last_tag_gst)
        {
            return false;
        }
    if (((tag_gst - last_tag_gst) % 30) != 0)
        {
            return false;
        }
    return tag_seconds_minus_cop(tag) <= static_cast<int64_t>(first_tag_gst);
}
}  // namespace

/**
 * @brief Adds the navigation data bits to the container holding OSNMA_NavData objects.
 *
 * @param nav_bits The navigation bits.
 * @param PRNd The satellite ID.
 * @param WN The GST week number of the received data.
 * @param TOW The GST time-of-week of the received data.
 */
void OSNMA_NavDataManager::add_navigation_data(const std::string& nav_bits, uint32_t PRNd, uint32_t WN, uint32_t TOW)
{
    if (!have_nav_data(nav_bits, PRNd, WN, TOW))
        {
            const uint64_t nav_gst = gst_seconds(WN, TOW);
            auto& nav_data = d_satellite_nav_data[PRNd][nav_gst];
            nav_data.add_nav_data(nav_bits);
            nav_data.set_prn_d(PRNd);
            nav_data.set_wn_sf0(WN);
            nav_data.set_tow_sf0(TOW);
            nav_data.set_last_received_WN(WN);
            nav_data.set_last_received_TOW(TOW);
        }
    prune_old_navigation_data(WN, TOW);
}


/**
 * @brief loops over the verified tags and updates the navigation data tag length
 */
void OSNMA_NavDataManager::update_nav_data(const std::multimap<uint32_t, Tag>& tags_verified, uint8_t tag_size)
{
    if (d_satellite_nav_data.empty() || tag_size == 0)
        {
            return;
        }
    // loop through all tags
    for (const auto& tag_entry : tags_verified)
        {
            const auto& tag = tag_entry.second;
            if (tag.cop == 0 ||
                (tag.status != Tag::e_verification_status::SUCCESS && tag.status != Tag::e_verification_status::FAIL))
                {
                    continue;
                }

            auto sat_it = d_satellite_nav_data.find(tag.PRN_d);
            if (sat_it == d_satellite_nav_data.end())
                {
                    continue;
                }

            auto& tow_map = sat_it->second;
            for (auto& tow_it : tow_map)  // note: starts with smallest (i.e. oldest) navigation dataset
                {
                    const std::string nav_data = nav_data_for_tag(tow_it.second, tag);
                    if (nav_data.empty() ||
                        tag.nav_data != nav_data ||
                        !nav_data_is_in_cop_window(tow_it.second, tow_it.first, tag))
                        {
                            continue;
                        }

                    if (tag.status == Tag::e_verification_status::FAIL)
                        {
                            if (tag_size < L_t_min &&
                                tow_it.second.has_tag_accumulation() &&
                                tow_it.second.get_accumulated_tag_adkd() == tag.ADKD &&
                                !tow_it.second.get_verified_status())
                                {
                                    tow_it.second.reset_tag_accumulation();
                                }
                            continue;
                        }

                    if (tag_size >= L_t_min)
                        {
                            tow_it.second.set_update_verified_bits(tag_size);
                            continue;
                        }

                    const uint64_t tag_gst = gst_seconds(tag.WN, tag.TOW);
                    if (!tow_it.second.has_tag_accumulation() ||
                        tow_it.second.get_accumulated_tag_adkd() != tag.ADKD ||
                        !tag_gst_is_coherent_with_accumulation(tow_it.second, tag))
                        {
                            tow_it.second.start_tag_accumulation(tag_size, tag.ADKD, tag_gst);
                        }
                    else
                        {
                            tow_it.second.continue_tag_accumulation(tag_size, tag_gst);
                        }
                }
        }
}


void OSNMA_NavDataManager::reset_tag_accumulations()
{
    for (auto& satellite : d_satellite_nav_data)
        {
            for (auto& tow_navdata : satellite.second)
                {
                    if (tow_navdata.second.get_verified_status())
                        {
                            tow_navdata.second.clear_tag_accumulation_metadata();
                        }
                    else
                        {
                            tow_navdata.second.reset_tag_accumulation();
                        }
                }
        }
}


void OSNMA_NavDataManager::prune_old_navigation_data(uint32_t WN, uint32_t TOW)
{
    const auto current_gst = static_cast<int64_t>(gst_seconds(WN, TOW));
    for (auto sat_it = d_satellite_nav_data.begin(); sat_it != d_satellite_nav_data.end();)
        {
            auto& tow_map = sat_it->second;
            for (auto nav_it = tow_map.begin(); nav_it != tow_map.end();)
                {
                    const auto last_received_gst = static_cast<int64_t>(gst_seconds(nav_it->second.get_last_received_WN(), nav_it->second.get_last_received_TOW()));
                    if (current_gst > last_received_gst &&
                        current_gst - last_received_gst > nav_data_retention_s)
                        {
                            nav_it = tow_map.erase(nav_it);
                        }
                    else
                        {
                            ++nav_it;
                        }
                }
            if (tow_map.empty())
                {
                    sat_it = d_satellite_nav_data.erase(sat_it);
                }
            else
                {
                    ++sat_it;
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
                    if (tow_navdata.second.get_verified_bits() >= L_t_min && !tow_navdata.second.get_verified_status())
                        {
                            result.push_back(tow_navdata.second);
                            d_satellite_nav_data[prna.first][tow_navdata.first].set_verified_status(true);
                        }
                }
        }
    return result;
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
    auto nav_data = prn_it->second.find(gst_seconds_with_offset(tag.WN, tag.TOW, -30));
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
    for (auto rev_it = prn_it->second.rbegin(); rev_it != prn_it->second.rend(); ++rev_it)  // NOLINT(modernize-loop-convert)
        {
            // note: starts with largest (i.e. newest) navigation dataset
            // Check if current key (TOW) fulfills condition
            if (nav_data_is_in_cop_window(rev_it->second, rev_it->first, tag))
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
    return "";
}


/**
 * @brief Checks if the OSNMA_NavData bits are already present. In case affirmative, it updates the OSNMA_NavData 'last received' timestamp
 * @remarks e.g.: a SV may repeat the bits over several subframes. In that case, need to save them only once.
 * @param nav_bits
 * @param PRNd
 * @param WN
 * @param TOW
 * @return
 */
bool OSNMA_NavDataManager::have_nav_data(const std::string& nav_bits, uint32_t PRNd, uint32_t WN, uint32_t TOW)
{
    if (d_satellite_nav_data.find(PRNd) != d_satellite_nav_data.end())
        {
            const uint64_t nav_gst = gst_seconds(WN, TOW);
            for (auto& data_timestamp : d_satellite_nav_data[PRNd])
                {
                    if (nav_gst >= data_timestamp.first + seconds_per_week ||
                        data_timestamp.first >= nav_gst + seconds_per_week)
                        {
                            continue;
                        }
                    if (nav_bits.size() == EPH_SIZE)
                        {
                            if (data_timestamp.second.get_ephemeris_data() == nav_bits)
                                {
                                    data_timestamp.second.set_last_received_WN(WN);
                                    data_timestamp.second.set_last_received_TOW(TOW);
                                    return true;
                                }
                        }
                    else if (nav_bits.size() == UTC_SIZE)
                        {
                            if (data_timestamp.second.get_utc_data() == nav_bits)
                                {
                                    data_timestamp.second.set_last_received_WN(WN);
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
    auto nav_data = prn_it->second.find(gst_seconds_with_offset(t.WN, t.TOW, -30));
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
    // iterate in reverse order to find matching TOW with Tag's COP value
    for (auto rev_it = prn_it->second.rbegin(); rev_it != prn_it->second.rend(); ++rev_it)  // NOLINT(modernize-loop-convert)
        {
            // note: starts with largest (i.e. newest) navigation dataset
            // Check if current key (TOW) fulfills cut-off point  and is not received after the tag
            if (nav_data_is_in_cop_window(rev_it->second, rev_it->first, t))
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
                              << ", WN_start="
                              << nav_data.second.get_wn_sf0()
                              << ", TOW_last="
                              << nav_data.second.get_last_received_TOW()
                              << ", WN_last="
                              << nav_data.second.get_last_received_WN()
                              << ", l_t="
                              << nav_data.second.get_verified_bits()
                              << ", PRNd="
                              << nav_data.second.get_prn_d()
                              << ", verified="
                              << nav_data.second.get_verified_status();
                }
        }
}
