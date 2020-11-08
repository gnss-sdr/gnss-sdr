/*!
 * \file galileo_cnav_message.h
 * \brief  Implementation of a Galileo CNAV Data message as described in
 * Galileo High Accuracy Service E6-B Signal-In-Space Message Specification v1.2
 * (April 2020)
 * \author Carles Fernandez-Prades, 2020 cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GALILEO_CNAV_MESSAGE_H
#define GNSS_SDR_GALILEO_CNAV_MESSAGE_H

#include "Galileo_CNAV.h"
#include <bitset>
#include <cstdint>
#include <string>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class handles the Galileo CNAV Data message, as described in the
 * Galileo High Accuracy Service E6-B Signal-In-Space Message Specification v1.2
 * (April 2020)
 */
class Galileo_Cnav_Message
{
public:
    Galileo_Cnav_Message() = default;

    void read_HAS_page(const std::string& page_string);

    inline bool get_flag_CRC_test() const
    {
        return flag_CRC_test;
    }

private:
    bool CRC_test(std::bitset<GALILEO_CNAV_BITS_FOR_CRC> bits, uint32_t checksum) const;
    bool flag_CRC_test{};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_CNAV_MESSAGE_H
