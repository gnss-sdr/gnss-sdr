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

uint32_t Tag::id_counter = 0;
uint32_t OSNMA_NavData::id_counter = 0;

bool OSNMA_NavData::add_nav_data(const std::string& nav_data)
{
    if (nav_data.size() == 549)
        {
            d_ephemeris_iono = nav_data;
            std::bitset<10> bits(nav_data.substr(0, 10));
            IOD_nav = static_cast<uint8_t>(bits.to_ulong());
            return true;
        }
    else if (nav_data.size() == 141)
        {
            d_utc = nav_data;
            return true;
        }
    return false;
}


std::string OSNMA_NavData::get_utc_data() const
{
    return d_utc;
}


std::string OSNMA_NavData::get_ephemeris_data() const
{
    return d_ephemeris_iono;
}


/**
 * Updates the last TOW the NavData bits were received.
 * @param TOW
 */
void OSNMA_NavData::update_last_received_timestamp(uint32_t TOW)
{
    d_last_received_TOW = TOW;
}
