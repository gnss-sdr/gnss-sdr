/*!
 * \file galileo_ism.cc
 * \brief Interface of a Galileo Integrity Support Message
 * \author Carles Fernandez, 2024. cfernandez(at)cttc.cat
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "galileo_ism.h"
#include <algorithm>

void Galileo_ISM::set_ism_constellation_id(uint8_t const_id)
{
    this->ism_constellation_id = const_id;
}


void Galileo_ISM::set_ism_service_level_id(uint8_t sl_id)
{
    this->ism_service_level_id = sl_id;
}


void Galileo_ISM::set_ism_wn(uint16_t wn_ism)
{
    this->ism_wn = wn_ism;
}


void Galileo_ISM::set_ism_t0(uint16_t t0)
{
    this->ism_t0 = t0;
}


void Galileo_ISM::set_ism_mask_msb(bool mask_msb)
{
    this->ism_mask_msb = mask_msb;
}


void Galileo_ISM::set_ism_mask(uint32_t mask)
{
    this->ism_mask = mask;
}


void Galileo_ISM::set_ism_pconst(uint8_t pconst)
{
    this->ism_pconst = pconst;
}


void Galileo_ISM::set_ism_psat(uint8_t psat)
{
    this->ism_psat = psat;
}


void Galileo_ISM::set_ism_ura(uint8_t ura)
{
    this->ism_ura = ura;
}


void Galileo_ISM::set_ism_ure(uint8_t ure)
{
    this->ism_ure = ure;
}


void Galileo_ISM::set_ism_bnom(uint8_t bnom)
{
    this->ism_bnom = bnom;
}


void Galileo_ISM::set_ism_Tvalidity(uint8_t tvalidity)
{
    this->ism_Tvalidity = tvalidity;
}


void Galileo_ISM::set_ism_crc(uint32_t crc)
{
    this->ism_crc = crc;
}


uint16_t Galileo_ISM::get_WN_ISM() const
{
    return this->ism_wn;
}


uint16_t Galileo_ISM::get_t0_ISM() const
{
    return (this->ism_t0 * 1800);
}


double Galileo_ISM::get_pconst_value() const
{
    auto it = ISM_PCONST_MAP.find(this->ism_pconst);
    if (it == ISM_PCONST_MAP.end())
        {
            return 0.0;
        }
    return it->second;
}


double Galileo_ISM::get_psat_value() const
{
    auto it = ISM_PSAT_MAP.find(this->ism_psat);
    if (it == ISM_PSAT_MAP.end())
        {
            return 0.0;
        }
    return it->second;
}


float Galileo_ISM::get_ura_m() const
{
    auto it = ISM_URA_MAP.find(this->ism_ura);
    if (it == ISM_URA_MAP.end())
        {
            return 0.0;
        }
    return it->second;
}


float Galileo_ISM::get_ure_m() const
{
    auto it = ISM_URE_MAP.find(this->ism_ure);
    if (it == ISM_URE_MAP.end())
        {
            return 0.0;
        }
    return it->second;
}


float Galileo_ISM::get_bnom_m() const
{
    auto it = ISM_BNOM_MAP.find(this->ism_bnom);
    if (it == ISM_BNOM_MAP.end())
        {
            return 5.0;  //
        }
    return it->second;
}


uint16_t Galileo_ISM::get_Tvalidity_hours() const
{
    auto it = ISM_TVALIDITY_MAP.find(this->ism_Tvalidity);
    if (it == ISM_TVALIDITY_MAP.end())
        {
            return 0.0;
        }
    return it->second;
}


bool Galileo_ISM::check_ism_crc(const std::bitset<GALILEO_DATA_JK_BITS>& bits)
{
    std::bitset<96> extracted;
    for (int32_t i = 0; i < GALILEO_ISM_CRC_DATA_BITS; ++i)
        {
            extracted[i] = bits[i + 32];
        }

    std::vector<uint8_t> data_bytes((extracted.size() + 7) / 8);

    for (size_t i = 0; i < extracted.size(); i += 8)
        {
            uint8_t byte = 0;
            for (size_t j = 0; j < 8 && i + j < extracted.size(); ++j)
                {
                    byte |= (extracted[i + j] << j);
                }
            data_bytes[i / 8] = byte;
        }

    std::reverse(data_bytes.begin(), data_bytes.end());
    const uint32_t crc_computed = this->compute_crc(data_bytes);

    if (this->ism_crc == crc_computed)
        {
            return true;
        }

    return false;
}


uint32_t Galileo_ISM::compute_crc(const std::vector<uint8_t>& data)
{
    crc32_ism.process_bytes(data.data(), data.size());
    uint32_t result = crc32_ism.checksum();
    return result;
}
