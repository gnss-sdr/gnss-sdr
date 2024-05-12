/*!
* \file osnma_helper.h
* \brief Class for auxiliary osnma functions
* \author Carles Fernandez-Prades, 2024 cfernandez(at)cttc.es
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

#include "osnma_helper.h"

uint32_t Osnma_Helper::compute_gst(uint32_t WN, uint32_t TOW) const
{
    uint32_t GST = (WN & 0x00000FFF) << 20 | (TOW & 0x000FFFFF);
    return GST;
}

std::vector<uint8_t> Osnma_Helper::gst_to_uint8(uint32_t GST) const
{
    std::vector<uint8_t> res(4);

    res[1] = static_cast<uint8_t>((GST & 0xFF000000) >> 24);
    res[2] = static_cast<uint8_t>((GST & 0x00FF0000) >> 16);
    res[3] = static_cast<uint8_t>((GST & 0x0000FF00) >> 8);
    res[4] = static_cast<uint8_t>(GST & 0x000000FF);
    return res;
}
