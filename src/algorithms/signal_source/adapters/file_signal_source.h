/*!
 * \file file_signal_source.h
 * \brief Interface of a class that reads signals samples from a file
 * and adapts it to a SignalSourceInterface
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 * This class represents a file signal source. Internally it uses a GNU Radio's
 * gr_file_source as a connector to the data.
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

#ifndef GNSS_SDR_FILE_SIGNAL_SOURCE_H
#define GNSS_SDR_FILE_SIGNAL_SOURCE_H

#include "file_source_base.h"
#include <string>

/** \addtogroup Signal_Source Signal Source
 * Classes for Signal Source management.
 * \{ */
/** \addtogroup Signal_Source_adapters signal_source_adapters
 * Classes that wrap GNU Radio signal sources with a GNSSBlockInterface
 * \{ */


class ConfigurationInterface;

/*!
 * \brief Class that reads signals samples from a file
 * and adapts it to a SignalSourceInterface
 */
class FileSignalSource : public FileSourceBase
{
public:
    FileSignalSource(ConfigurationInterface const* configuration, std::string const& role,
        unsigned int in_streams, unsigned int out_streams,
        Concurrent_Queue<pmt::pmt_t>* queue);

    ~FileSignalSource() = default;

private:
};


/** \} */
/** \} */
#endif  // GNSS_SDR_FILE_SIGNAL_SOURCE_H
