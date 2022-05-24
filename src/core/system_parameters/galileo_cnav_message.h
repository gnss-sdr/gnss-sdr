/*!
 * \file galileo_cnav_message.h
 * \brief  Implementation of a Galileo CNAV Data message as described in
 * Galileo High Accuracy Service Signal-In-Space Interface Control Document
 * (HAS SIS ICD) Issue 1.0, May 2022
 * \author Carles Fernandez-Prades, 2020-2022 cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GALILEO_CNAV_MESSAGE_H
#define GNSS_SDR_GALILEO_CNAV_MESSAGE_H

#include "Galileo_CNAV.h"
#include "galileo_has_page.h"
#include <bitset>
#include <cstdint>
#include <string>
#include <utility>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class handles the Galileo CNAV Data message, as described in the
 * Galileo High Accuracy Service Signal-In-Space Interface Control Document
 * (HAS SIS ICD) Issue 1.0, May 2022
 */
class Galileo_Cnav_Message
{
public:
    Galileo_Cnav_Message() = default;

    void read_HAS_page(const std::string& page_string);

    inline bool is_HAS_in_test_mode() const
    {
        return d_test_mode;
    }

    inline bool is_HAS_page_dummy() const
    {
        return d_page_dummy;
    }

    inline bool have_new_HAS_page() const
    {
        return d_new_HAS_page;
    }

    inline Galileo_HAS_page get_HAS_encoded_page() const
    {
        return has_page;
    }

    inline bool get_flag_CRC_test() const
    {
        return d_flag_CRC_test;
    }

    inline void set_time_stamp(uint64_t time_stamp)
    {
        has_page.time_stamp = time_stamp;
    }

    inline void set_tow(uint32_t tow)
    {
        has_page.tow = tow;
    }

private:
    uint8_t read_has_page_header_parameter(const std::bitset<GALILEO_CNAV_PAGE_HEADER_BITS>& bits, const std::pair<int32_t, int32_t>& parameter) const;
    bool CRC_test(const std::bitset<GALILEO_CNAV_BITS_FOR_CRC>& bits, uint32_t checksum) const;
    void read_HAS_page_header(const std::string& page_string);

    Galileo_HAS_page has_page{};

    uint8_t d_has_page_status{};
    uint8_t d_has_reserved{};
    uint8_t d_received_message_page_id{};
    uint8_t d_received_message_type{};
    uint8_t d_received_message_id{};
    uint8_t d_received_message_size{};

    bool d_test_mode{};
    bool d_flag_CRC_test{};
    bool d_page_dummy{};
    bool d_new_HAS_page{};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_CNAV_MESSAGE_H
