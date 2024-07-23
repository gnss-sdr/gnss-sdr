/*!
* \file osnma_data.cc
* \brief Class for Galileo OSNMA data storage
* \author Carles Fernandez-Prades, 2020-2023 cfernandez(at)cttc.es
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

#include "osnma_data.h"
#include <cstring>
#include <iostream>

/**
 * @brief Constructs a NavData object with the given osnma_msg.
 * \details Packs the ephemeris, iono and utc data from the current subframe into the NavData structure. It also gets the PRNa and the GST.
 * @param osnma_msg The shared pointer to the OSNMA_msg object.
 */

uint32_t Tag::id_counter = 0;
uint32_t NavData::id_counter = 0;

bool NavData::add_nav_data(std::string nav_data)
{
    if (nav_data.size() == 549)
        {
            ephemeris_iono_vector_2 = nav_data;
            std::bitset<10> bits(nav_data.substr(0,10));
            IOD_nav = static_cast<uint8_t>(bits.to_ulong());
            return true;
        }
    else if (nav_data.size() == 141)
        {
            utc_vector_2 = nav_data;
            return true;
        }
    return false;
}
std::string NavData::get_utc_data() const
{
    return utc_vector_2;
}
std::string NavData::get_ephemeris_data() const
{
    return ephemeris_iono_vector_2;
}
/**
 * Updates the last TOW the NavData bits were received.
 * @param TOW
 */
void NavData::update_last_received_timestamp(uint32_t TOW)
{
    last_received_TOW = TOW;
}
