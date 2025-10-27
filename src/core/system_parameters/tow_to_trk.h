/*!
 * \file tow_to_trk.h
 * \brief Class to inform about TOW from Telemetry to Tracking blocks
 * \author Carles Fernandez, 2025. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2025  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_TOW_TO_TRK_H
#define GNSS_SDR_TOW_TO_TRK_H

#include <cstdint>
#include <string>


/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

class TOW_to_trk
{
public:
    TOW_to_trk() = default;

    // Constructor with all parameters
    TOW_to_trk(const std::string& sig, int32_t ch, uint32_t t, uint64_t stamp, int32_t w, uint32_t p)
        : signal(sig), channel(ch), tow(t), sample_stamp(stamp), wn(w), prn(p) {}

    std::string signal;
    int32_t channel{0};
    uint32_t tow{0};
    uint64_t sample_stamp{0};
    int32_t wn{0};
    uint32_t prn{0};
};

/** \} */
/** \} */
#endif  // GNSS_SDR_TOW_TO_TRK_H