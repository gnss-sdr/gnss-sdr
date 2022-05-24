/*!
 * \file galileo_has_page.h
 * \brief Class for Galileo HAS message page storage
 * \author Carles Fernandez-Prades, 2021 cfernandez(at)cttc.es
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


#ifndef GNSS_SDR_GALILEO_HAS_PAGE_H
#define GNSS_SDR_GALILEO_HAS_PAGE_H

#include <cstdint>
#include <string>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for Galileo HAS message page, as defined in
 * Galileo High Accuracy Service Signal-In-Space Interface Control Document
 * (HAS SIS ICD) Issue 1.0, May 2022
 */
class Galileo_HAS_page
{
public:
    Galileo_HAS_page() = default;

    std::string has_message_string;  //!< HAS message content
    uint64_t time_stamp{};           //!< HAS page time stamp, in [s]
    uint32_t tow{};                  //!< HAS page time of week, in [s]

    // HAS page header
    uint8_t has_status{};       //!< HAS status
    uint8_t reserved{};         //!< HAS reserved field
    uint8_t message_type{};     //!< HAS message type (MT)
    uint8_t message_id{};       //!< HAS message ID (MID)
    uint8_t message_size{};     //!< HAS message size (MS)
    uint8_t message_page_id{};  //!< HAS message page ID (PID)
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_HAS_PAGE_H
