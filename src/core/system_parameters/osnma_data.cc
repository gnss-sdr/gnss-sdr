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


void OSNMA_NavData::init(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    d_TOW_sf0 = osnma_msg->TOW_sf0;
}


std::string OSNMA_NavData::get_ephemeris_iono_data() const
{
    return d_ephemeris_iono;
}


std::string OSNMA_NavData::get_utc_data() const
{
    return d_utc;
}


uint32_t OSNMA_NavData::get_tow_sf0() const
{
    return d_TOW_sf0;
}


void OSNMA_NavData::set_ephemeris_iono_data(const std::string& iono_data)
{
    d_ephemeris_iono = iono_data;
}


void OSNMA_NavData::set_utc_data(const std::string& utc_data)
{
    d_utc = utc_data;
}
