/*!
 * \file nav_message_packet.h
 * \brief Class for storage of decoded navigation messages
 * \author Carles Fernandez-Prades, 2021. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_NAV_MESSAGE_PACKET_H
#define GNSS_SDR_NAV_MESSAGE_PACKET_H

#include <cstdint>
#include <string>
#include <utility>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */

class Nav_Message_Packet
{
public:
    Nav_Message_Packet() = default;  //!< Default constructor

    ~Nav_Message_Packet() = default;  //!< Default destructor

    std::string system;                //!< GNSS constellation: "G" for GPS, "R" for Glonass, "S" for SBAS, "E" for Galileo and "C" for Beidou.
    std::string signal;                //!< GNSS signal: "1C" for GPS L1 C/A, "1B" for Galileo E1b/c, "1G" for Glonass L1 C/A, "2S" for GPS L2 L2C(M), "2G" for Glonass L2 C/A, "L5" for GPS L5 and "5X" for Galileo E5a
    int32_t prn;                       //!< SV ID
    int32_t tow_at_current_symbol_ms;  //!< Time of week of the current symbol, in ms
    std::string nav_message;           //!< Content of the navigation page

    /// Copy constructor
    Nav_Message_Packet(const Nav_Message_Packet& other) noexcept
    {
        *this = other;
    };

    /// Copy assignment operator
    Nav_Message_Packet& operator=(const Nav_Message_Packet& rhs) noexcept
    {
        // Only do assignment if RHS is a different object from this.
        if (this != &rhs)
            {
                this->system = rhs.system;
                this->signal = rhs.signal;
                this->prn = rhs.prn;
                this->tow_at_current_symbol_ms = rhs.tow_at_current_symbol_ms;
                this->nav_message = rhs.nav_message;
            }
        return *this;
    };

    /// Move constructor
    Nav_Message_Packet(Nav_Message_Packet&& other) noexcept
    {
        *this = std::move(other);
    };

    /// Move assignment operator
    Nav_Message_Packet& operator=(Nav_Message_Packet&& other) noexcept
    {
        if (this != &other)
            {
                this->system = other.system;
                this->signal = other.signal;
                this->prn = other.prn;
                this->tow_at_current_symbol_ms = other.tow_at_current_symbol_ms;
                this->nav_message = other.nav_message;
            }
        return *this;
    };
};

/** \} */
/** \} */
#endif  // GNSS_SDR_NAV_MESSAGE_PACKET_H
