/*!
 * \file galileo_cnav_message.h
 * \brief  Implementation of a Galileo CNAV Data message as described in
 * Galileo High Accuracy Service E6-B Signal-In-Space Message Specification v1.2
 * (April 2020)
 * \author Carles Fernandez-Prades, 2020 cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GALILEO_CNAV_MESSAGE_H
#define GNSS_SDR_GALILEO_CNAV_MESSAGE_H

#include "Galileo_CNAV.h"
#include "galileo_has_data.h"
#include <bitset>
#include <cstdint>
#include <list>
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

    inline bool have_new_HAS_message()
    {
        return d_new_message;
    }

    inline bool is_HAS_in_test_mode() const
    {
        return d_test_mode;
    }

    inline bool is_HAS_message_dummy() const
    {
        return d_page_dummy;
    }

    inline Galileo_HAS_data get_HAS_data() const
    {
        return d_HAS_data;
    }

private:
    bool CRC_test(std::bitset<GALILEO_CNAV_BITS_FOR_CRC> bits, uint32_t checksum) const;
    void read_HAS_page_header(const std::string& page_string);
    void process_HAS_page(const std::string& page_string);
    void decode_message_type1();
    void read_HAS_message_type1(const std::string& message_string);
    void read_MT1_header(const std::string& message_string);
    void read_MT1_body(const std::string& message_string);

    uint8_t read_has_page_header_parameter(std::bitset<GALILEO_CNAV_PAGE_HEADER_BITS> bits, const std::pair<int32_t, int32_t>& parameter) const;
    uint8_t read_has_message_header_parameter_uint8(std::bitset<GALILEO_CNAV_MT1_HEADER_BITS> bits, const std::pair<int32_t, int32_t>& parameter) const;
    uint16_t read_has_message_header_parameter_uint16(std::bitset<GALILEO_CNAV_MT1_HEADER_BITS> bits, const std::pair<int32_t, int32_t>& parameter) const;
    bool read_has_message_header_parameter_bool(std::bitset<GALILEO_CNAV_MT1_HEADER_BITS> bits, const std::pair<int32_t, int32_t>& parameter) const;
    uint8_t read_has_message_body_uint8(const std::string& bits) const;
    uint16_t read_has_message_body_uint16(const std::string& bits) const;
    uint64_t read_has_message_body_uint64(const std::string& bits) const;
    int16_t read_has_message_body_int16(const std::string& bits) const;

    Galileo_HAS_data d_HAS_data{};

    std::string d_encoded_message_type_1;
    std::list<uint8_t> d_list_pid;

    uint8_t d_has_page_status{};
    uint8_t d_current_message_id{};
    uint8_t d_current_message_size{};

    uint8_t d_received_message_page_id{};
    uint8_t d_received_message_type{};
    uint8_t d_received_message_id{};
    uint8_t d_received_encoded_messages{};
    uint8_t d_received_message_size{};

    bool d_test_mode{};
    bool d_new_message{};
    bool d_flag_CRC_test{};
    bool d_page_dummy{};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_CNAV_MESSAGE_H
