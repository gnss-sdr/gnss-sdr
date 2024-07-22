/*!
 * \file ion_gsms_metadata_handler.h
 * \brief Build instances of IONGSMSFileSource as needed given a list of stream ids
 * \author Víctor Castillo Agüero, 2024. victorcastilloaguero(at)gmail.com
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

#ifndef ION_GSMS_METADATA_HANDLER_H
#define ION_GSMS_METADATA_HANDLER_H

#include "GnssMetadata.h"
#include <gnuradio/block.h>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

class IONGSMSMetadataHandler
{
public:
    explicit IONGSMSMetadataHandler(const std::string& metadata_filepath);

    std::vector<IONGSMSFileSource::sptr> make_stream_sources(const std::vector<std::string>& stream_ids) const;

public:  // Getters
    const std::string& metadata_filepath() const;

private: // Private methods
    void load_metadata();

private: // State
    std::string metadata_filepath_;
    GnssMetadata::Metadata metadata_;
};

#endif //ION_GSMS_METADATA_HANDLER_H
