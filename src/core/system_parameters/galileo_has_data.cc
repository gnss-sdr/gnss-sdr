/*!
 * \file galileo_has_data.cc
 * \brief Class for Galileo HAS message type 1 data storage
 * \author Carles Fernandez-Prades, 2020-2022 cfernandez(at)cttc.es
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

#include "galileo_has_data.h"
#include "Galileo_CNAV.h"
#include <algorithm>
#include <bitset>
#include <iterator>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>


std::vector<std::string> Galileo_HAS_data::get_signals_in_mask(uint8_t nsys) const
{
    if (signal_mask.size() <= nsys)
        {
            return {};
        }

    std::vector<std::string> signals_in_mask;
    uint16_t sig = signal_mask[nsys];
    uint8_t system = gnss_id_mask[nsys];
    std::bitset<HAS_MSG_NUMBER_SIGNAL_MASKS> bits(sig);

    for (int32_t k = 0; k < HAS_MSG_NUMBER_SIGNAL_MASKS; k++)
        {
            if ((bits[HAS_MSG_NUMBER_SIGNAL_MASKS - k - 1]))
                {
                    try
                        {
                            signals_in_mask.emplace_back(HAS_SIGNAL_INDEX_TABLE.at(system).at(k));
                        }
                    catch (const std::out_of_range& e)
                        {
                            signals_in_mask.emplace_back("Unknown");
                        }
                }
        }
    return signals_in_mask;
}


std::vector<std::string> Galileo_HAS_data::get_signals_in_mask(const std::string& system) const
{
    auto systems = this->get_systems_string();
    auto sys_it = std::find(systems.begin(), systems.end(), system);
    if (sys_it == systems.end())
        {
            return {};
        }
    auto system_index = std::distance(systems.begin(), sys_it);
    return this->get_signals_in_mask(system_index);
}


std::vector<std::string> Galileo_HAS_data::get_systems_string() const
{
    if (this->gnss_id_mask.empty())
        {
            return {};
        }

    std::vector<std::string> systems;
    systems.reserve(this->gnss_id_mask.size());

    for (uint8_t i = 0; i < this->Nsys; i++)
        {
            switch (this->gnss_id_mask[i])
                {
                case 0:
                    systems.emplace_back("GPS");
                    break;
                case 2:
                    systems.emplace_back("Galileo");
                    break;
                default:
                    systems.emplace_back("Reserved");
                }
        }

    return systems;
}


std::vector<std::string> Galileo_HAS_data::get_systems_subset_string() const
{
    if (this->gnss_id_clock_subset.empty())
        {
            return {};
        }

    std::vector<std::string> systems;
    systems.reserve(this->gnss_id_clock_subset.size());

    for (uint8_t i = 0; i < this->Nsys_sub; i++)
        {
            switch (this->gnss_id_clock_subset[i])
                {
                case 0:
                    systems.emplace_back("GPS");
                    break;
                case 2:
                    systems.emplace_back("Galileo");
                    break;
                default:
                    systems.emplace_back("Reserved");
                }
        }

    return systems;
}


std::vector<std::vector<float>> Galileo_HAS_data::get_code_bias_m() const
{
    if (code_bias.empty())
        {
            return {};
        }
    const size_t outer_size = this->code_bias.size();
    const size_t inner_size = this->code_bias[0].size();
    std::vector<std::vector<float>> code_bias_m(outer_size, std::vector<float>(inner_size));
    for (size_t i = 0; i < outer_size; i++)
        {
            for (size_t j = 0; j < inner_size; j++)
                {
                    code_bias_m[i][j] = static_cast<float>(this->code_bias[i][j]) * HAS_MSG_CODE_BIAS_SCALE_FACTOR;
                }
        }
    return code_bias_m;
}


std::vector<std::vector<float>> Galileo_HAS_data::get_phase_bias_cycle() const
{
    if (phase_bias.empty())
        {
            return {};
        }
    const size_t outer_size = this->phase_bias.size();
    const size_t inner_size = this->phase_bias[0].size();
    std::vector<std::vector<float>> phase_bias_cycle(outer_size, std::vector<float>(inner_size));
    for (size_t i = 0; i < outer_size; i++)
        {
            for (size_t j = 0; j < inner_size; j++)
                {
                    phase_bias_cycle[i][j] = static_cast<float>(this->phase_bias[i][j]) * HAS_MSG_PHASE_BIAS_SCALE_FACTOR;
                }
        }
    return phase_bias_cycle;
}


std::vector<std::vector<float>> Galileo_HAS_data::get_delta_clock_subset_correction_m() const
{
    if (this->delta_clock_correction_clock_subset.empty())
        {
            return {};
        }

    std::vector<std::vector<float>> delta_clock_subset_correction_m;
    delta_clock_subset_correction_m.reserve(this->Nsys_sub);

    for (const auto& correction_subset : this->delta_clock_correction_clock_subset)
        {
            std::vector<float> correction_m;
            correction_m.reserve(correction_subset.size());
            for (const auto& d : correction_subset)
                {
                    correction_m.push_back(static_cast<float>(d) * HAS_MSG_DELTA_CLOCK_SCALE_FACTOR);
                }
            delta_clock_subset_correction_m.emplace_back(std::move(correction_m));
        }

    return delta_clock_subset_correction_m;
}


std::vector<float> Galileo_HAS_data::get_delta_radial_m() const
{
    std::vector<float> delta_radial_m;
    delta_radial_m.reserve(this->delta_radial.size());
    for (const auto& d : this->delta_radial)
        {
            delta_radial_m.push_back(static_cast<float>(d) * HAS_MSG_DELTA_RADIAL_SCALE_FACTOR);
        }
    return delta_radial_m;
}


std::vector<float> Galileo_HAS_data::get_delta_radial_m(uint8_t nsys) const
{
    if (nsys >= this->Nsys)
        {
            return {};
        }
    std::vector<float> delta_orbit_radial_m_v = this->get_delta_radial_m();
    std::vector<uint8_t> num_satellites = this->get_num_satellites();
    std::vector<float> delta_orbit_radial_m_aux;
    uint8_t num_sats_in_this_system = num_satellites[nsys];
    delta_orbit_radial_m_aux.reserve(num_sats_in_this_system);

    size_t index = 0;
    for (uint8_t sys = 0; sys <= nsys; sys++)
        {
            uint8_t num_sats_in_system = num_satellites[sys];
            if (sys != nsys)
                {
                    index += num_sats_in_system;
                }
            else
                {
                    for (uint8_t sat = 0; sat < num_sats_in_system; sat++)
                        {
                            delta_orbit_radial_m_aux.push_back(delta_orbit_radial_m_v[index]);
                            index++;
                        }
                }
        }
    return delta_orbit_radial_m_aux;
}


std::vector<float> Galileo_HAS_data::get_delta_in_track_m() const
{
    std::vector<float> delta_in_track_m;
    delta_in_track_m.reserve(this->delta_in_track.size());
    for (const auto& d : this->delta_in_track)
        {
            delta_in_track_m.push_back(static_cast<float>(d) * HAS_MSG_DELTA_IN_TRACK_SCALE_FACTOR);
        }
    return delta_in_track_m;
}


std::vector<float> Galileo_HAS_data::get_delta_in_track_m(uint8_t nsys) const
{
    if (nsys >= this->Nsys)
        {
            return {};
        }
    std::vector<float> delta_in_track_m_v = this->get_delta_in_track_m();
    std::vector<uint8_t> num_satellites = this->get_num_satellites();
    std::vector<float> delta_in_track_m_aux;
    uint8_t num_sats_in_this_system = num_satellites[nsys];
    delta_in_track_m_aux.reserve(num_sats_in_this_system);

    size_t index = 0;
    for (uint8_t sys = 0; sys <= nsys; sys++)
        {
            uint8_t num_sats_in_system = num_satellites[sys];
            if (sys != nsys)
                {
                    index += num_sats_in_system;
                }
            else
                {
                    for (uint8_t sat = 0; sat < num_sats_in_system; sat++)
                        {
                            delta_in_track_m_aux.push_back(delta_in_track_m_v[index]);
                            index++;
                        }
                }
        }
    return delta_in_track_m_aux;
}


std::vector<float> Galileo_HAS_data::get_delta_cross_track_m() const
{
    std::vector<float> delta_cross_track_m;
    delta_cross_track_m.reserve(this->delta_cross_track.size());
    for (const auto& d : this->delta_cross_track)
        {
            delta_cross_track_m.push_back(static_cast<float>(d) * HAS_MSG_DELTA_CROSS_TRACK_SCALE_FACTOR);
        }
    return delta_cross_track_m;
}


std::vector<float> Galileo_HAS_data::get_delta_cross_track_m(uint8_t nsys) const
{
    if (nsys >= this->Nsys)
        {
            return {};
        }
    std::vector<float> delta_cross_track_m_v = this->get_delta_cross_track_m();
    std::vector<float> delta_cross_track_m_aux;
    std::vector<uint8_t> num_satellites = this->get_num_satellites();
    uint8_t num_sats_in_this_system = num_satellites[nsys];
    delta_cross_track_m_aux.reserve(num_sats_in_this_system);

    size_t index = 0;
    for (uint8_t sys = 0; sys <= nsys; sys++)
        {
            uint8_t num_sats_in_system = num_satellites[sys];
            if (sys != nsys)
                {
                    index += num_sats_in_system;
                }
            else
                {
                    for (uint8_t sat = 0; sat < num_sats_in_system; sat++)
                        {
                            delta_cross_track_m_aux.push_back(delta_cross_track_m_v[index]);
                            index++;
                        }
                }
        }
    return delta_cross_track_m_aux;
}


std::vector<float> Galileo_HAS_data::get_delta_clock_correction_m() const
{
    std::vector<float> delta_clock_correction_m;
    delta_clock_correction_m.reserve(this->delta_clock_correction.size());
    for (const auto& d : this->delta_clock_correction)
        {
            delta_clock_correction_m.push_back(static_cast<float>(d) * HAS_MSG_DELTA_CLOCK_SCALE_FACTOR);
        }
    return delta_clock_correction_m;
}


std::vector<float> Galileo_HAS_data::get_delta_clock_correction_m(uint8_t nsys) const
{
    if (nsys >= this->Nsys)
        {
            return {};
        }
    std::vector<float> delta_clock_correction_m_v = this->get_delta_clock_correction_m();
    std::vector<float> delta_clock_correction_m_aux;
    std::vector<uint8_t> num_satellites = this->get_num_satellites();
    uint8_t num_sats_in_this_system = num_satellites[nsys];
    delta_clock_correction_m_aux.reserve(num_sats_in_this_system);

    size_t index = 0;
    for (uint8_t sys = 0; sys <= nsys; sys++)
        {
            uint8_t num_sats_in_system = num_satellites[sys];
            if (sys != nsys)
                {
                    index += num_sats_in_system;
                }
            else
                {
                    for (uint8_t sat = 0; sat < num_sats_in_system; sat++)
                        {
                            delta_clock_correction_m_aux.push_back(delta_clock_correction_m_v[index]);
                            index++;
                        }
                }
        }
    return delta_clock_correction_m_aux;
}


std::vector<float> Galileo_HAS_data::get_delta_clock_subset_correction_m(uint8_t nsys) const
{
    if (nsys >= this->Nsys)
        {
            return {};
        }

    // get equivalence of nsys in mask with nsys_sub_index in submask
    auto gnss_id_in_mask = this->gnss_id_mask[nsys];
    auto sys_it = std::find(gnss_id_clock_subset.begin(), gnss_id_clock_subset.end(), gnss_id_in_mask);
    if (sys_it == gnss_id_clock_subset.end())
        {
            return {};
        }
    uint64_t nsys_sub_index = std::distance(gnss_id_clock_subset.begin(), sys_it);
    if (nsys_sub_index >= satellite_submask.size())
        {
            return {};
        }

    auto subset_corr_matrix = this->get_delta_clock_subset_correction_m();
    std::vector<float> delta_clock_subset_correction_m_v = subset_corr_matrix[nsys_sub_index];
    std::vector<float> delta_clock_subset_correction_m_aux;
    std::vector<uint8_t> num_satellites_subset = this->get_num_subset_satellites();
    uint8_t num_sats_in_this_system_subset = num_satellites_subset[nsys_sub_index];
    delta_clock_subset_correction_m_aux.reserve(num_sats_in_this_system_subset);
    size_t index = 0;
    for (uint8_t sys = 0; sys <= nsys; sys++)
        {
            uint8_t num_sats_in_subset = num_satellites_subset[sys];
            if (sys != nsys)
                {
                    index += num_sats_in_subset;
                }
            else
                {
                    for (uint8_t sat = 0; sat < num_sats_in_subset; sat++)
                        {
                            delta_clock_subset_correction_m_aux.push_back(delta_clock_subset_correction_m_v[index]);
                            index++;
                        }
                }
        }
    return delta_clock_subset_correction_m_aux;
}


std::vector<int> Galileo_HAS_data::get_PRNs_in_mask(uint8_t nsys) const
{
    if (nsys >= satellite_mask.size())
        {
            return {};
        }
    std::vector<int> prn;
    auto sat_mask = satellite_mask[nsys];
    std::bitset<HAS_MSG_NUMBER_SATELLITE_IDS> bits(sat_mask);

    for (int32_t i = 0; i < HAS_MSG_NUMBER_SATELLITE_IDS; i++)
        {
            if ((bits[HAS_MSG_NUMBER_SATELLITE_IDS - i - 1]))
                {
                    prn.push_back(i + 1);
                }
        }
    return prn;
}


std::vector<int> Galileo_HAS_data::get_PRNs_in_mask(const std::string& system) const
{
    auto systems = this->get_systems_string();
    auto sys_it = std::find(systems.begin(), systems.end(), system);
    if (sys_it == systems.end())
        {
            return {};  // system not found
        }
    auto system_index = std::distance(systems.begin(), sys_it);
    return this->get_PRNs_in_mask(system_index);
}


std::vector<int> Galileo_HAS_data::get_PRNs_in_submask(uint8_t nsys) const
{
    // nsys refers to the system index in the main mask
    if (nsys >= this->Nsys)
        {
            return {};
        }

    // get equivalence of nsys in mask with nsys_sub_index in submask
    auto gnss_id_in_mask = this->gnss_id_mask[nsys];
    auto sys_it = std::find(gnss_id_clock_subset.begin(), gnss_id_clock_subset.end(), gnss_id_in_mask);
    if (sys_it == gnss_id_clock_subset.end())
        {
            return {};
        }
    uint64_t nsys_sub_index = std::distance(gnss_id_clock_subset.begin(), sys_it);
    if (nsys_sub_index >= satellite_submask.size())
        {
            return {};
        }

    auto mask_prns = this->get_PRNs_in_mask(nsys);

    std::vector<int> prn_in_submask;
    auto sat_submask = this->satellite_submask[nsys_sub_index];
    int count = 0;
    while (sat_submask)
        {
            if (sat_submask & 1)
                {
                    prn_in_submask.push_back(mask_prns[count]);
                    count++;
                }
            sat_submask >>= 1;
        }

    return prn_in_submask;
}


std::vector<uint16_t> Galileo_HAS_data::get_gnss_iod(uint8_t nsys) const
{
    if (nsys >= this->Nsys)
        {
            return {};
        }

    std::vector<uint16_t> gnss_iod_aux;
    std::vector<uint8_t> num_satellites = this->get_num_satellites();
    uint8_t num_sats_in_this_system = num_satellites[nsys];
    gnss_iod_aux.reserve(num_sats_in_this_system);

    size_t index = 0;
    for (uint8_t sys = 0; sys <= nsys; sys++)
        {
            uint8_t num_sats_in_system = num_satellites[sys];
            if (sys != nsys)
                {
                    index += num_sats_in_system;
                }
            else
                {
                    for (uint8_t sat = 0; sat < num_sats_in_system; sat++)
                        {
                            gnss_iod_aux.push_back(this->gnss_iod[index]);
                            index++;
                        }
                }
        }
    return gnss_iod_aux;
}


std::vector<uint8_t> Galileo_HAS_data::get_num_satellites() const
{
    if (this->Nsys == 0)
        {
            return {};
        }
    std::vector<uint8_t> num_satellites;
    num_satellites.reserve(this->Nsys);
    for (uint8_t i = 0; i < this->Nsys; i++)
        {
            uint64_t sat_mask = this->satellite_mask[i];
            int count = 0;
            while (sat_mask)
                {
                    count += sat_mask & 1;
                    sat_mask >>= 1;
                }
            num_satellites.push_back(count);
        }

    return num_satellites;
}


std::vector<uint8_t> Galileo_HAS_data::get_num_subset_satellites() const
{
    if (this->Nsys_sub == 0)
        {
            return {};
        }
    std::vector<uint8_t> num_satellites_subset;
    num_satellites_subset.reserve(this->Nsys_sub);

    for (uint8_t i = 0; i < this->Nsys_sub; i++)
        {
            uint64_t sat_submask = this->satellite_submask[i];
            int count = 0;
            while (sat_submask)
                {
                    count += sat_submask & 1;
                    sat_submask >>= 1;
                }
            num_satellites_subset.push_back(count);
        }
    return num_satellites_subset;
}


float Galileo_HAS_data::get_code_bias_m(const std::string& signal, int PRN) const
{
    float code_bias_correction_m = 0.0;
    const size_t outer_size = this->code_bias.size();
    if (outer_size == 0)
        {
            return code_bias_correction_m;
        }
    const size_t inner_size = this->code_bias[0].size();
    if (inner_size == 0)
        {
            return code_bias_correction_m;
        }

    std::string targeted_system;
    if ((signal == "L1 C/A") || (signal == "L1C(D)") || (signal == "L1C(P)" || (signal == "L1C(D+P)") || (signal == "L2 CM") || (signal == "L2 CL") || (signal == "L2 CM+CL") || (signal == "L2 P") || (signal == "L5 I")) ||
        (signal == "L5 Q") || (signal == "L5 I + L5 Q"))
        {
            targeted_system = std::string("GPS");
        }
    if ((signal == "E1-B I/NAV OS") || (signal == "E1-C") || (signal == "E1-B + E1-C") || (signal == "E5a-I F/NAV OS") || (signal == "E5a-Q") || (signal == "E5a-I+E5a-Q") || (signal == "E5b-I I/NAV OS") ||
        (signal == "E5b-Q") || (signal == "E5b-I+E5b-Q") || (signal == "E5-I") || (signal == "E5-Q") || (signal == "E5-I + E5-Q") || (signal == "E6-B C/NAV HAS") || (signal == "E6-C") || (signal == "E6-B + E6-C"))
        {
            targeted_system = std::string("Galileo");
        }

    if (!code_bias.empty() && !targeted_system.empty())
        {
            std::vector<std::string> systems = this->get_systems_string();
            auto Nsys = systems.size();
            auto nsys_index = std::distance(systems.cbegin(), std::find(systems.cbegin(), systems.cend(), targeted_system));

            if (static_cast<size_t>(nsys_index) < Nsys)
                {
                    std::vector<std::string> signals = get_signals_in_mask(static_cast<uint8_t>(nsys_index));
                    auto sig_index = std::distance(signals.cbegin(), std::find(signals.cbegin(), signals.cend(), signal));

                    if (static_cast<size_t>(sig_index) < signals.size())
                        {
                            int sat_index = 0;
                            bool index_found = false;
                            for (int s = 0; s < static_cast<int>(Nsys); s++)
                                {
                                    std::vector<int> PRNs_in_mask = get_PRNs_in_mask(s);
                                    if (s == nsys_index)
                                        {
                                            if (index_found == false)
                                                {
                                                    auto sati = std::distance(PRNs_in_mask.cbegin(), std::find(PRNs_in_mask.cbegin(), PRNs_in_mask.cend(), PRN));
                                                    if (static_cast<size_t>(sati) < PRNs_in_mask.size())
                                                        {
                                                            sat_index += sati;
                                                            index_found = true;
                                                        }
                                                }
                                        }
                                    else
                                        {
                                            if (index_found == false)
                                                {
                                                    sat_index += PRNs_in_mask.size();
                                                }
                                        }
                                }
                            if (index_found == true)
                                {
                                    code_bias_correction_m = static_cast<float>(this->code_bias[sat_index][sig_index]) * HAS_MSG_CODE_BIAS_SCALE_FACTOR;
                                }
                        }
                }
        }

    return code_bias_correction_m;
}


float Galileo_HAS_data::get_phase_bias_cycle(const std::string& signal, int PRN) const
{
    float phase_bias_correction_cycles = 0.0;
    const size_t outer_size = this->phase_bias.size();
    if (outer_size == 0)
        {
            return phase_bias_correction_cycles;
        }
    const size_t inner_size = this->phase_bias[0].size();
    if (inner_size == 0)
        {
            return phase_bias_correction_cycles;
        }

    std::string targeted_system;
    if ((signal == "L1 C/A") || (signal == "L1C(D)") || (signal == "L1C(P)" || (signal == "L1C(D+P)") || (signal == "L2 CM") || (signal == "L2 CL") || (signal == "L2 CM+CL") || (signal == "L2 P") || (signal == "L5 I")) ||
        (signal == "L5 Q") || (signal == "L5 I + L5 Q"))
        {
            targeted_system = std::string("GPS");
        }
    if ((signal == "E1-B I/NAV OS") || (signal == "E1-C") || (signal == "E1-B + E1-C") || (signal == "E5a-I F/NAV OS") || (signal == "E5a-Q") || (signal == "E5a-I+E5a-Q") || (signal == "E5b-I I/NAV OS") ||
        (signal == "E5b-Q") || (signal == "E5b-I+E5b-Q") || (signal == "E5-I") || (signal == "E5-Q") || (signal == "E5-I + E5-Q") || (signal == "E6-B C/NAV HAS") || (signal == "E6-C") || (signal == "E6-B + E6-C"))
        {
            targeted_system = std::string("Galileo");
        }

    if (!phase_bias.empty() && !targeted_system.empty())
        {
            std::vector<std::string> systems = this->get_systems_string();
            auto Nsys = systems.size();
            auto nsys_index = std::distance(systems.cbegin(), std::find(systems.cbegin(), systems.cend(), targeted_system));

            if (static_cast<size_t>(nsys_index) < Nsys)
                {
                    std::vector<std::string> signals = get_signals_in_mask(static_cast<uint8_t>(nsys_index));
                    auto sig_index = std::distance(signals.cbegin(), std::find(signals.cbegin(), signals.cend(), signal));

                    if (static_cast<size_t>(sig_index) < signals.size())
                        {
                            int sat_index = 0;
                            bool index_found = false;
                            for (int s = 0; s < static_cast<int>(Nsys); s++)
                                {
                                    std::vector<int> PRNs_in_mask = get_PRNs_in_mask(s);
                                    if (s == nsys_index)
                                        {
                                            if (index_found == false)
                                                {
                                                    auto sati = std::distance(PRNs_in_mask.cbegin(), std::find(PRNs_in_mask.cbegin(), PRNs_in_mask.cend(), PRN));
                                                    if (static_cast<size_t>(sati) < PRNs_in_mask.size())
                                                        {
                                                            sat_index += sati;
                                                            index_found = true;
                                                        }
                                                }
                                        }
                                    else
                                        {
                                            if (index_found == false)
                                                {
                                                    sat_index += PRNs_in_mask.size();
                                                }
                                        }
                                }
                            if (index_found == true)
                                {
                                    phase_bias_correction_cycles = static_cast<float>(this->phase_bias[sat_index][sig_index]) * HAS_MSG_PHASE_BIAS_SCALE_FACTOR;
                                }
                        }
                }
        }

    return phase_bias_correction_cycles;
}


float Galileo_HAS_data::get_delta_radial_m(const std::string& system, int prn) const
{
    if (this->delta_radial.empty())
        {
            return 0.0;
        }
    auto systems = this->get_systems_string();
    auto sys_it = std::find(systems.begin(), systems.end(), system);
    if (sys_it == systems.end())
        {
            return 0.0;  // system not found
        }
    auto system_index = std::distance(systems.begin(), sys_it);

    auto prns = this->get_PRNs_in_mask(system_index);
    auto it = std::find(prns.begin(), prns.end(), prn);
    if (it == prns.end())
        {
            return 0.0;  // PRN not found
        }

    auto radial_m_v = this->get_delta_radial_m(system_index);
    return radial_m_v[std::distance(prns.begin(), it)];
}


float Galileo_HAS_data::get_delta_in_track_m(const std::string& system, int prn) const
{
    if (this->delta_in_track.empty())
        {
            return 0.0;
        }
    auto systems = this->get_systems_string();
    auto sys_it = std::find(systems.begin(), systems.end(), system);
    if (sys_it == systems.end())
        {
            return 0.0;  // system not found
        }
    auto system_index = std::distance(systems.begin(), sys_it);

    auto prns = this->get_PRNs_in_mask(system_index);
    auto it = std::find(prns.begin(), prns.end(), prn);
    if (it == prns.end())
        {
            return 0.0;  // PRN not found
        }

    auto in_track_m_v = this->get_delta_in_track_m(system_index);
    return in_track_m_v[std::distance(prns.begin(), it)];
}


float Galileo_HAS_data::get_delta_cross_track_m(const std::string& system, int prn) const
{
    if (this->delta_cross_track.empty())
        {
            return 0.0;
        }
    auto systems = this->get_systems_string();
    auto sys_it = std::find(systems.begin(), systems.end(), system);
    if (sys_it == systems.end())
        {
            return 0.0;  // system not found
        }
    auto system_index = std::distance(systems.begin(), sys_it);

    auto prns = this->get_PRNs_in_mask(system_index);
    auto it = std::find(prns.begin(), prns.end(), prn);
    if (it == prns.end())
        {
            return 0.0;  // PRN not found
        }

    auto cross_track_m_v = this->get_delta_cross_track_m(system_index);
    return cross_track_m_v[std::distance(prns.begin(), it)];
}


float Galileo_HAS_data::get_clock_correction_mult_m(const std::string& system, int prn) const
{
    if (this->delta_clock_correction.empty())
        {
            return 0.0;
        }
    auto systems = this->get_systems_string();
    auto sys_it = std::find(systems.begin(), systems.end(), system);
    if (sys_it == systems.end())
        {
            return 0.0;  // system not found
        }
    auto system_index = std::distance(systems.begin(), sys_it);

    auto prns = this->get_PRNs_in_mask(system_index);
    auto it = std::find(prns.begin(), prns.end(), prn);
    if (it == prns.end())
        {
            return 0.0;  // PRN not found
        }

    auto index = std::distance(prns.begin(), it);
    auto clock_correction_m_v = this->get_delta_clock_correction_m(system_index);
    auto dcm = static_cast<float>(this->delta_clock_multiplier[system_index]);
    return clock_correction_m_v[index] * dcm;
}


float Galileo_HAS_data::get_clock_subset_correction_mult_m(const std::string& system, int prn) const
{
    if (this->gnss_id_clock_subset.empty())
        {
            return 0.0;
        }

    auto systems = this->get_systems_string();
    auto sys_it = std::find(systems.begin(), systems.end(), system);
    if (sys_it == systems.end())
        {
            return 0.0;  // system not found
        }
    auto system_index = std::distance(systems.begin(), sys_it);

    auto prns_subset = this->get_PRNs_in_submask(system_index);
    auto it = std::find(prns_subset.begin(), prns_subset.end(), prn);
    if (it == prns_subset.end())
        {
            return 0.0;  // PRN not found
        }

    auto index_subset = std::distance(prns_subset.begin(), it);
    auto clock_correction_m_v = this->get_delta_clock_subset_correction_m(system_index);
    auto dcm = static_cast<float>(this->delta_clock_multiplier_clock_subset[system_index]);
    if (!clock_correction_m_v.empty())
        {
            return clock_correction_m_v[index_subset] * dcm;
        }
    else
        {
            return 0.0;
        }
}


uint16_t Galileo_HAS_data::get_nsat() const
{
    std::vector<uint8_t> num_satellites = this->get_num_satellites();
    uint16_t total_number_of_sats = std::accumulate(num_satellites.begin(), num_satellites.end(), 0);
    return total_number_of_sats;
}


uint16_t Galileo_HAS_data::get_nsat_sub() const
{
    auto Nsat_sub = static_cast<uint16_t>(this->delta_clock_correction_clock_subset.size());
    return Nsat_sub;
}


uint16_t Galileo_HAS_data::get_validity_interval_s(uint8_t validity_interval_index) const
{
    const auto it = HAS_VALIDITY_INTERVALS.find(validity_interval_index);
    if (it == HAS_VALIDITY_INTERVALS.cend())
        {
            return 0;
        }
    return it->second;
}


uint16_t Galileo_HAS_data::get_gnss_iod(const std::string& system, int prn) const
{
    if (this->gnss_iod.empty())
        {
            return 65535;  // not found
        }
    auto systems = this->get_systems_string();
    auto sys_it = std::find(systems.begin(), systems.end(), system);
    if (sys_it == systems.end())
        {
            return 65535;  // not found
        }
    auto system_index = std::distance(systems.begin(), sys_it);

    auto prns = this->get_PRNs_in_mask(system_index);
    auto it = std::find(prns.begin(), prns.end(), prn);
    if (it == prns.end())
        {
            return 65535;  // PRN not found
        }

    auto gnss_iod_v = this->get_gnss_iod(system_index);
    return gnss_iod_v[std::distance(prns.begin(), it)];
}


uint8_t Galileo_HAS_data::get_gnss_id(int nsat) const
{
    int number_sats = 0;
    for (uint8_t i = 0; i < Nsys; i++)
        {
            auto prns = get_PRNs_in_mask(i);
            number_sats += static_cast<int>(prns.size());
            if (nsat < number_sats)
                {
                    return gnss_id_mask[i];
                }
        }

    return HAS_MSG_WRONG_SYSTEM;
}
