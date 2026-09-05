/*!
 * \file protobuf_cleanup_manager.h
 * \brief Singleton class that implements the shared resource cleaner pattern
 * for the Protocol Buffers library
 * \author Vladislav P, 2026. vladisslav2011(at)gmail.com
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

#ifndef GNSS_SDR_PROTOBUF_CLEANUP_MANAGER_H
#define GNSS_SDR_PROTOBUF_CLEANUP_MANAGER_H

#include <google/protobuf/message_lite.h>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */

/*!
 * \brief This class implements the Protocol Buffers library shared memory cleanup manager.
 *
 * google::protobuf::ShutdownProtobufLibrary() must be called only once, at the end
 * of the program execution: it is not safe to use any other part of the library
 * after that call. Calling it earlier (e.g., from the destructor of a class that
 * uses protobuf) results in use-after-free memory corruption and random crashes.
 * Call Protobuf_Cleanup_Manager::get() in the constructor of every class that
 * uses the protobuf library, so that google::protobuf::ShutdownProtobufLibrary()
 * is called at program shutdown, when the singleton is destroyed.
 */
class Protobuf_Cleanup_Manager
{
public:
    static Protobuf_Cleanup_Manager& get()
    {
        static Protobuf_Cleanup_Manager instance{};
        return instance;
    }

    Protobuf_Cleanup_Manager(const Protobuf_Cleanup_Manager&) = delete;
    Protobuf_Cleanup_Manager& operator=(const Protobuf_Cleanup_Manager&) = delete;
    Protobuf_Cleanup_Manager(Protobuf_Cleanup_Manager&&) = delete;
    Protobuf_Cleanup_Manager& operator=(Protobuf_Cleanup_Manager&&) = delete;

private:
    Protobuf_Cleanup_Manager() = default;
    ~Protobuf_Cleanup_Manager()
    {
        google::protobuf::ShutdownProtobufLibrary();
    }
};

/** \} */
/** \} */
#endif  // GNSS_SDR_PROTOBUF_CLEANUP_MANAGER_H
