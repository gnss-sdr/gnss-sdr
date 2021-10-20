/*!
 * \file galileo_has_data.cc
 * \brief Class for Galileo HAS message type 1 data storage
 * \author Carles Fernandez-Prades, 2020-2021 cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "galileo_has_data.h"
#include "Galileo_CNAV.h"
#include <algorithm>
#include <bitset>
#include <numeric>
#include <sstream>


std::vector<int> Galileo_HAS_data::get_PRNs_in_mask(uint8_t nsys) const
{
    std::vector<int> prn;
    if (nsys < satellite_mask.size())
        {
            uint64_t sat_mask = satellite_mask[nsys];
            std::bitset<HAS_MSG_NUMBER_SATELLITE_IDS> bits(sat_mask);
            std::string bit_str = bits.to_string();
            for (int32_t i = 0; i < HAS_MSG_NUMBER_SATELLITE_IDS; i++)
                {
                    if (bit_str[i] == '1')
                        {
                            prn.push_back(i + 1);
                        }
                }
        }
    return prn;
}


std::vector<int> Galileo_HAS_data::get_PRNs_in_submask(uint8_t nsys) const
{
    std::vector<int> prn;
    std::vector<int> prn_in_submask;
    if (nsys < satellite_submask.size())
        {
            auto it = std::find(gnss_id_mask.begin(), gnss_id_mask.end(), gnss_id_clock_subset[nsys]);
            int index = 0;
            if (it != gnss_id_mask.end())
                {
                    index = it - gnss_id_mask.begin();
                }
            else
                {
                    return prn_in_submask;
                }

            uint8_t nsys_index_in_mask = gnss_id_mask[index];
            uint64_t sat_mask = satellite_mask[nsys_index_in_mask];

            std::bitset<HAS_MSG_NUMBER_SATELLITE_IDS> bits(sat_mask);
            std::string mask_bit_str = bits.to_string();
            for (int i = 0; i < HAS_MSG_NUMBER_SATELLITE_IDS; i++)
                {
                    if (mask_bit_str[i] == '1')
                        {
                            prn.push_back(i + 1);
                        }
                }

            int number_sats_this_gnss_id = std::count(mask_bit_str.begin(), mask_bit_str.end(), '1');
            uint64_t sat_submask = satellite_submask[nsys];
            // convert into string
            std::string sat_submask_str("");
            uint64_t aux = 1;
            for (int k = 0; k < number_sats_this_gnss_id - 1; k++)
                {
                    if ((aux & sat_submask) >= 1)
                        {
                            sat_submask_str.insert(0, "1");
                        }
                    else
                        {
                            sat_submask_str.insert(0, "0");
                        }
                    aux <<= 1;
                }

            for (int i = 0; i < number_sats_this_gnss_id; i++)
                {
                    if (sat_submask_str[i] == '1')
                        {
                            prn_in_submask.push_back(prn[i]);
                        }
                }
        }

    return prn_in_submask;
}


std::vector<std::string> Galileo_HAS_data::get_signals_in_mask(uint8_t nsys) const
{
    std::vector<std::string> signals_in_mask;
    if (signal_mask.size() > nsys)
        {
            uint16_t sig = signal_mask[nsys];
            std::bitset<HAS_MSG_NUMBER_SIGNAL_MASKS> bits(sig);
            std::string bits_str = bits.to_string();
            for (int32_t k = 0; k < HAS_MSG_NUMBER_SIGNAL_MASKS; k++)
                {
                    if (bits_str[k] == '1')
                        {
                            uint8_t system = gnss_id_mask[nsys];
                            std::string signal;
                            switch (k)
                                {
                                case 0:
                                    if (system == 0)
                                        {
                                            // GPS
                                            signal = "L1 C/A";
                                        }
                                    else if (system == 2)
                                        {
                                            // Galileo
                                            signal = "E1-B I/NAV OS";
                                        }
                                    else
                                        {
                                            signal = "Unknown";
                                        }
                                    break;
                                case 1:
                                    if (system == 0)
                                        {
                                            // GPS
                                            signal = "Reserved";
                                        }
                                    else if (system == 2)
                                        {
                                            // Galileo
                                            signal = "E1-C";
                                        }
                                    else
                                        {
                                            signal = "Unknown";
                                        }
                                    break;
                                case 2:
                                    if (system == 0)
                                        {
                                            // GPS
                                            signal = "Reserved";
                                        }
                                    else if (system == 2)
                                        {
                                            // Galileo
                                            signal = "E1-B + E1-C";
                                        }
                                    else
                                        {
                                            signal = "Unknown";
                                        }
                                    break;
                                case 3:
                                    if (system == 0)
                                        {
                                            // GPS
                                            signal = "L1 L1C(D)";
                                        }
                                    else if (system == 2)
                                        {
                                            // Galileo
                                            signal = "E5a-I F/NAV OS";
                                        }
                                    else
                                        {
                                            signal = "Unknown";
                                        }
                                    break;
                                case 4:
                                    if (system == 0)
                                        {
                                            // GPS
                                            signal = "L1 L1C(P)";
                                        }
                                    else if (system == 2)
                                        {
                                            // Galileo
                                            signal = "E5a-Q";
                                        }
                                    else
                                        {
                                            signal = "Unknown";
                                        }
                                    break;
                                case 5:
                                    if (system == 0)
                                        {
                                            // GPS
                                            signal = "L1 L1C(D+P)";
                                        }
                                    else if (system == 2)
                                        {
                                            // Galileo
                                            signal = "E5a-I+E5a-Q";
                                        }
                                    else
                                        {
                                            signal = "Unknown";
                                        }
                                    break;
                                case 6:
                                    if (system == 0)
                                        {
                                            // GPS
                                            signal = "L2 L2C(M)";
                                        }
                                    else if (system == 2)
                                        {
                                            // Galileo
                                            signal = "E5b-I I/NAV OS";
                                        }
                                    else
                                        {
                                            signal = "Unknown";
                                        }
                                    break;
                                case 7:
                                    if (system == 0)
                                        {
                                            // GPS
                                            signal = "L2 L2C(L)";
                                        }
                                    else if (system == 2)
                                        {
                                            // Galileo
                                            signal = "E5b-Q";
                                        }
                                    else
                                        {
                                            signal = "Unknown";
                                        }
                                    break;
                                case 8:
                                    if (system == 0)
                                        {
                                            // GPS
                                            signal = "L2 L2C(M+L)";
                                        }
                                    else if (system == 2)
                                        {
                                            // Galileo
                                            signal = "E5b-I+E5b-Q";
                                        }
                                    else
                                        {
                                            signal = "Unknown";
                                        }
                                    break;
                                case 9:
                                    if (system == 0)
                                        {
                                            // GPS
                                            signal = "L2 P";
                                        }
                                    else if (system == 2)
                                        {
                                            // Galileo
                                            signal = "E5-I";
                                        }
                                    else
                                        {
                                            signal = "Unknown";
                                        }
                                    break;
                                case 10:
                                    if (system == 0)
                                        {
                                            // GPS
                                            signal = "Reserved";
                                        }
                                    else if (system == 2)
                                        {
                                            // Galileo
                                            signal = "E5-Q";
                                        }
                                    else
                                        {
                                            signal = "Unknown";
                                        }
                                    break;
                                case 11:
                                    if (system == 0)
                                        {
                                            // GPS
                                            signal = "L5 I";
                                        }
                                    else if (system == 2)
                                        {
                                            // Galileo
                                            signal = "E5-I + E5-Q";
                                        }
                                    else
                                        {
                                            signal = "Unknown";
                                        }
                                    break;
                                case 12:
                                    if (system == 0)
                                        {
                                            // GPS
                                            signal = "L5 Q";
                                        }
                                    else if (system == 2)
                                        {
                                            // Galileo
                                            signal = "E6-B C/NAV HAS";
                                        }
                                    else
                                        {
                                            signal = "Unknown";
                                        }
                                    break;
                                case 13:
                                    if (system == 0)
                                        {
                                            // GPS
                                            signal = "L5 I + L5 Q";
                                        }
                                    else if (system == 2)
                                        {
                                            // Galileo
                                            signal = "E6-C";
                                        }
                                    else
                                        {
                                            signal = "Unknown";
                                        }
                                    break;
                                case 14:
                                    if (system == 0)
                                        {
                                            // GPS
                                            signal = "Reserved";
                                        }
                                    else if (system == 2)
                                        {
                                            // Galileo
                                            signal = "E6-B + E6-C";
                                        }
                                    else
                                        {
                                            signal = "Unknown";
                                        }
                                    break;
                                case 15:
                                    if (system == 0)
                                        {
                                            // GPS
                                            signal = "Reserved";
                                        }
                                    else if (system == 2)
                                        {
                                            // Galileo
                                            signal = "Reserved";
                                        }
                                    else
                                        {
                                            signal = "Unknown";
                                        }
                                    break;
                                default:
                                    signal = "Unknown";
                                }
                            signals_in_mask.push_back(signal);
                        }
                }
        }
    return signals_in_mask;
}


uint8_t Galileo_HAS_data::get_gnss_id(int nsat) const
{
    int number_sats = 0;
    for (uint8_t i = 0; i < Nsys; i++)
        {
            number_sats += static_cast<int>(get_PRNs_in_mask(i).size());
            if (nsat < number_sats)
                {
                    return gnss_id_mask[i];
                }
        }

    return HAS_MSG_WRONG_SYSTEM;
}


uint16_t Galileo_HAS_data::get_validity_interval_s(uint8_t validity_interval_index) const
{
    uint16_t validity_interval;
    switch (validity_interval_index)
        {
        case 0:
            validity_interval = 5;
            break;
        case 1:
            validity_interval = 10;
            break;
        case 2:
            validity_interval = 15;
            break;
        case 3:
            validity_interval = 20;
            break;
        case 4:
            validity_interval = 30;
            break;
        case 5:
            validity_interval = 60;
            break;
        case 6:
            validity_interval = 90;
            break;
        case 7:
            validity_interval = 120;
            break;
        case 8:
            validity_interval = 180;
            break;
        case 9:
            validity_interval = 240;
            break;
        case 10:
            validity_interval = 300;
            break;
        case 11:
            validity_interval = 600;
            break;
        case 12:
            validity_interval = 900;
            break;
        case 13:
            validity_interval = 1800;
            break;
        case 14:
            validity_interval = 3600;
            break;
        default:  // reserved
            validity_interval = 0;
        }
    return validity_interval;
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
    std::vector<float> delta_orbit_radial_m = this->get_delta_radial_m();
    if (nsys >= this->Nsys)
        {
            return delta_orbit_radial_m;
        }
    std::vector<float> delta_orbit_radial_m_aux;
    uint8_t num_sats_in_this_system = this->get_num_satellites()[nsys];
    delta_orbit_radial_m_aux.reserve(num_sats_in_this_system);

    size_t index = 0;
    for (uint8_t sys = 0; sys <= nsys; sys++)
        {
            uint8_t num_sats_in_system = this->get_num_satellites()[sys];
            if (sys != nsys)
                {
                    index += num_sats_in_system;
                }
            else
                {
                    for (uint8_t sat = 0; sat < num_sats_in_system; sat++)
                        {
                            delta_orbit_radial_m_aux.push_back(delta_orbit_radial_m[index]);
                            index++;
                        }
                }
        }
    return delta_orbit_radial_m_aux;
}


std::vector<float> Galileo_HAS_data::get_delta_along_track_m() const
{
    std::vector<float> delta_along_track_m;
    delta_along_track_m.reserve(this->delta_along_track.size());
    for (const auto& d : this->delta_along_track)
        {
            delta_along_track_m.push_back(static_cast<float>(d) * HAS_MSG_DELTA_ALONG_TRACK_SCALE_FACTOR);
        }
    return delta_along_track_m;
}


std::vector<float> Galileo_HAS_data::get_delta_along_track_m(uint8_t nsys) const
{
    std::vector<float> delta_along_track_m = this->get_delta_along_track_m();
    if (nsys >= this->Nsys)
        {
            return delta_along_track_m;
        }
    std::vector<float> delta_along_track_m_aux;
    uint8_t num_sats_in_this_system = this->get_num_satellites()[nsys];
    delta_along_track_m_aux.reserve(num_sats_in_this_system);

    size_t index = 0;
    for (uint8_t sys = 0; sys <= nsys; sys++)
        {
            uint8_t num_sats_in_system = this->get_num_satellites()[sys];
            if (sys != nsys)
                {
                    index += num_sats_in_system;
                }
            else
                {
                    for (uint8_t sat = 0; sat < num_sats_in_system; sat++)
                        {
                            delta_along_track_m_aux.push_back(delta_along_track_m[index]);
                            index++;
                        }
                }
        }
    return delta_along_track_m_aux;
}


std::vector<float> Galileo_HAS_data::get_delta_cross_track_m() const
{
    std::vector<float> delta_cross_track_m;
    delta_cross_track_m.reserve(this->delta_cross_track.size());
    for (const auto& d : this->delta_along_track)
        {
            delta_cross_track_m.push_back(static_cast<float>(d) * HAS_MSG_DELTA_CROSS_TRACK_SCALE_FACTOR);
        }
    return delta_cross_track_m;
}


std::vector<float> Galileo_HAS_data::get_delta_cross_track_m(uint8_t nsys) const
{
    std::vector<float> delta_cross_track_m = this->get_delta_cross_track_m();
    if (nsys >= this->Nsys)
        {
            return delta_cross_track_m;
        }
    std::vector<float> delta_cross_track_m_aux;
    uint8_t num_sats_in_this_system = this->get_num_satellites()[nsys];
    delta_cross_track_m_aux.reserve(num_sats_in_this_system);

    size_t index = 0;
    for (uint8_t sys = 0; sys <= nsys; sys++)
        {
            uint8_t num_sats_in_system = this->get_num_satellites()[sys];
            if (sys != nsys)
                {
                    index += num_sats_in_system;
                }
            else
                {
                    for (uint8_t sat = 0; sat < num_sats_in_system; sat++)
                        {
                            delta_cross_track_m_aux.push_back(delta_cross_track_m[index]);
                            index++;
                        }
                }
        }
    return delta_cross_track_m_aux;
}


std::vector<float> Galileo_HAS_data::get_delta_clock_c0_m() const
{
    std::vector<float> delta_clock_c0_m;
    delta_clock_c0_m.reserve(this->delta_clock_c0.size());
    for (const auto& d : this->delta_clock_c0)
        {
            delta_clock_c0_m.push_back(static_cast<float>(d) * HAS_MSG_DELTA_CLOCK_SCALE_FACTOR);
        }
    return delta_clock_c0_m;
}


std::vector<float> Galileo_HAS_data::get_delta_clock_c0_m(uint8_t nsys) const
{
    std::vector<float> delta_clock_c0_m = this->get_delta_clock_c0_m();
    if (nsys >= this->Nsys)
        {
            return delta_clock_c0_m;
        }
    std::vector<float> delta_clock_c0_m_aux;
    uint8_t num_sats_in_this_system = this->get_num_satellites()[nsys];
    delta_clock_c0_m_aux.reserve(num_sats_in_this_system);

    size_t index = 0;
    for (uint8_t sys = 0; sys <= nsys; sys++)
        {
            uint8_t num_sats_in_system = this->get_num_satellites()[sys];
            if (sys != nsys)
                {
                    index += num_sats_in_system;
                }
            else
                {
                    for (uint8_t sat = 0; sat < num_sats_in_system; sat++)
                        {
                            delta_clock_c0_m_aux.push_back(delta_clock_c0_m[index]);
                            index++;
                        }
                }
        }
    return delta_clock_c0_m_aux;
}


std::vector<std::vector<float>> Galileo_HAS_data::get_code_bias_m() const
{
    std::vector<std::vector<float>> code_bias_m;
    const size_t outer_size = this->code_bias.size();
    if (outer_size == 0)
        {
            return code_bias_m;
        }
    const size_t inner_size = this->code_bias[0].size();
    code_bias_m = std::vector<std::vector<float>>(outer_size, std::vector<float>(inner_size));
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
    std::vector<std::vector<float>> phase_bias_cycle;
    const size_t outer_size = this->phase_bias.size();
    if (outer_size == 0)
        {
            return phase_bias_cycle;
        }
    const size_t inner_size = this->phase_bias[0].size();
    phase_bias_cycle = std::vector<std::vector<float>>(outer_size, std::vector<float>(inner_size));
    for (size_t i = 0; i < outer_size; i++)
        {
            for (size_t j = 0; j < inner_size; j++)
                {
                    phase_bias_cycle[i][j] = static_cast<float>(this->phase_bias[i][j]) * HAS_MSG_PHASE_BIAS_SCALE_FACTOR;
                }
        }
    return phase_bias_cycle;
}


std::vector<uint8_t> Galileo_HAS_data::get_num_satellites() const
{
    std::vector<uint8_t> num_satellites;
    if (this->Nsys == 0)
        {
            return num_satellites;
        }
    num_satellites.reserve(this->Nsys);
    for (uint8_t i = 0; i < this->Nsys; i++)
        {
            std::stringstream ss;
            std::bitset<HAS_MSG_SATELLITE_MASK_LENGTH> bits(this->satellite_mask[i]);
            ss << bits.to_string();
            std::string sat_mask = ss.str();
            num_satellites.push_back(static_cast<int8_t>(std::count(sat_mask.begin(), sat_mask.end(), '1')));
        }

    return num_satellites;
}


std::vector<uint16_t> Galileo_HAS_data::get_gnss_iod(uint8_t nsys) const
{
    std::vector<uint16_t> gnss_iod_v = this->gnss_iod;

    if (nsys >= this->Nsys)
        {
            return gnss_iod_v;
        }
    std::vector<uint16_t> gnss_iod_aux;
    uint8_t num_sats_in_this_system = this->get_num_satellites()[nsys];
    gnss_iod_aux.reserve(num_sats_in_this_system);

    size_t index = 0;
    for (uint8_t sys = 0; sys <= nsys; sys++)
        {
            uint8_t num_sats_in_system = this->get_num_satellites()[sys];
            if (sys != nsys)
                {
                    index += num_sats_in_system;
                }
            else
                {
                    for (uint8_t sat = 0; sat < num_sats_in_system; sat++)
                        {
                            gnss_iod_aux.push_back(gnss_iod_v[index]);
                            index++;
                        }
                }
        }
    return gnss_iod_aux;
}


uint16_t Galileo_HAS_data::get_nsat() const
{
    std::vector<uint8_t> num_satellites = this->get_num_satellites();
    uint16_t total_number_of_sats = std::accumulate(num_satellites.begin(), num_satellites.end(), 0);
    return total_number_of_sats;
}


std::vector<std::string> Galileo_HAS_data::get_systems_string() const
{
    const size_t nsys = this->gnss_id_mask.size();
    std::vector<std::string> systems(nsys, std::string(""));
    for (uint8_t i = 0; i < this->Nsys; i++)
        {
            std::string system("Reserved");
            if (this->gnss_id_mask[i] == 0)
                {
                    system = "GPS";
                }
            if (this->gnss_id_mask[i] == 2)
                {
                    system = "Galileo";
                }
            systems[i] = system;
        }
    return systems;
}


uint16_t Galileo_HAS_data::get_nsatprime() const
{
    auto Nsatprime = static_cast<uint16_t>(this->delta_clock_c0_clock_subset.size());
    return Nsatprime;
}
