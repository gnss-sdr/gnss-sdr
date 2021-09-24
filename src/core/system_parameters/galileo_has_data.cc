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
