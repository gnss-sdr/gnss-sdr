/*!
 * \file protobuf_cleanup_manager.h
 * \brief Singleton class, that implements shared resourse  cleaner pattern for
 * Protocol Buffers library
 * \author Vladislav P, 2026. vladisslav2011(at)gmail.com
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

#ifndef PROTOBUF_CLEANUP_MANAGER_H
#define PROTOBUF_CLEANUP_MANAGER_H

#include <google/protobuf/message_lite.h>
#include <iostream>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */

/*!
 * \brief This class implements protobuf library shared memory cleanup manager
 * google::protobuf::ShutdownProtobufLibrary() should be called only once at the end of the program execution
 * Calling google::protobuf::ShutdownProtobufLibrary() during tests execution results in a weird crash due to use-after-free
 * Create an instance of this class in a constructor of every class, that uses protobuf library
 * to make sure that google::protobuf::ShutdownProtobufLibrary() will be called at program shutdown.
 */
class protobuf_cleanup_manager
{
public:
    explicit protobuf_cleanup_manager()
    {
    }
    ~protobuf_cleanup_manager()
    {
        google::protobuf::ShutdownProtobufLibrary();
    }
    static protobuf_cleanup_manager& get()
    {
        static protobuf_cleanup_manager instance{};
        return instance;
    }
};

/** \} */
/** \} */
#endif  // PROTOBUF_CLEANUP_MANAGER_H
