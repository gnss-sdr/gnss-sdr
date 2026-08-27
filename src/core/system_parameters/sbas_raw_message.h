/*!
 * \file sbas_raw_message.h
 * \brief Container for a CRC-validated raw SBAS L1 message
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_SBAS_RAW_MESSAGE_H
#define GNSS_SDR_SBAS_RAW_MESSAGE_H

#include "SBAS_L1.h"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

/*!
 * \brief A CRC-validated 250-bit SBAS L1 frame and its receiver timestamp.
 *
 * The frame is stored MSB first as 32 bytes. Its six least-significant padding
 * bits are zero. The timestamp identifies the first bit of the preamble in the
 * receiver sample-clock time base.
 */
class Sbas_Raw_Message
{
public:
    Sbas_Raw_Message() = default;

    Sbas_Raw_Message(double sample_stamp_s, uint32_t prn, const std::vector<uint8_t>& frame)
        : d_sample_stamp_s(sample_stamp_s),
          d_prn(prn)
    {
        const size_t bytes_to_copy = std::min(frame.size(), d_frame.size());
        std::copy_n(frame.cbegin(), bytes_to_copy, d_frame.begin());
        d_frame.back() &= 0xC0U;
    }

    double sample_stamp_s() const { return d_sample_stamp_s; }
    uint32_t prn() const { return d_prn; }
    uint32_t message_type() const { return (static_cast<uint32_t>(d_frame[1]) >> 2U) & 0x3FU; }
    const std::array<uint8_t, SBAS_L1_MSG_LENGTH_BYTES>& frame() const { return d_frame; }

private:
    double d_sample_stamp_s{};
    uint32_t d_prn{};
    std::array<uint8_t, SBAS_L1_MSG_LENGTH_BYTES> d_frame{};
};

/** \} */
/** \} */

#endif  // GNSS_SDR_SBAS_RAW_MESSAGE_H
