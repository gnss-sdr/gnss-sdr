/*!
 * \file serdes_nav_message.h
 * \brief Serialization / Deserialization of Nav_Message_Packet objects using
 * Protocol Buffers
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

#ifndef GNSS_SDR_SERDES_NAV_MESSAGE_H
#define GNSS_SDR_SERDES_NAV_MESSAGE_H

#include "nav_message.pb.h"  // file created by Protocol Buffers at compile time
#include "nav_message_packet.h"
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_libs
 * \{ */


/*!
 * \brief This class implements serialization and deserialization of
 * Nav_Message_Packet objects using Protocol Buffers.
 */
class Serdes_Nav_Message
{
public:
    Serdes_Nav_Message()
    {
        // Verify that the version of the library that we linked against is
        // compatible with the version of the headers we compiled against.
        GOOGLE_PROTOBUF_VERIFY_VERSION;
    }

    ~Serdes_Nav_Message()
    {
        // google::protobuf::ShutdownProtobufLibrary();
    }

    inline Serdes_Nav_Message(const Serdes_Nav_Message& other) noexcept : navmsg_(other.navmsg_)  //!< Copy constructor
    {
    }

    inline Serdes_Nav_Message& operator=(const Serdes_Nav_Message& rhs) noexcept  //!< Copy assignment operator
    {
        if (this != &rhs)
            {
                this->navmsg_.CopyFrom(rhs.navmsg_);
            }
        return *this;
    }

    inline Serdes_Nav_Message(Serdes_Nav_Message&& other) noexcept : navmsg_(std::move(other.navmsg_))  //!< Move constructor
    {
        // Set the other object's navmsg_ to a default-constructed state
        other.navmsg_ = gnss_sdr::navMsg{};
    }

    inline Serdes_Nav_Message& operator=(Serdes_Nav_Message&& other) noexcept  //!< Move assignment operator
    {
        if (this != &other)
            {
                navmsg_ = std::move(other.navmsg_);
                other.navmsg_ = gnss_sdr::navMsg{};
            }
        return *this;
    }

    inline std::string createProtobuffer(const std::shared_ptr<Nav_Message_Packet> nav_msg_packet)  //!< Serialization into a string
    {
        navmsg_.Clear();
        std::string data;

        navmsg_.set_system(nav_msg_packet->system);
        navmsg_.set_signal(nav_msg_packet->signal);
        navmsg_.set_prn(nav_msg_packet->prn);
        navmsg_.set_tow_at_current_symbol_ms(nav_msg_packet->tow_at_current_symbol_ms);
        navmsg_.set_nav_message(nav_msg_packet->nav_message);

        navmsg_.SerializeToString(&data);

        return data;
    }

    inline Nav_Message_Packet readProtobuffer(const gnss_sdr::navMsg& msg) const  //!< Deserialization
    {
        Nav_Message_Packet navmsg;

        navmsg.system = msg.system();
        navmsg.signal = msg.signal();
        navmsg.prn = msg.prn();
        navmsg.tow_at_current_symbol_ms = msg.tow_at_current_symbol_ms();
        navmsg.nav_message = msg.nav_message();

        return navmsg;
    }

private:
    gnss_sdr::navMsg navmsg_{};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_SERDES_NAV_MESSAGE_H
