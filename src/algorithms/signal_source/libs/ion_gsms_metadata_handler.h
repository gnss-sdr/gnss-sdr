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
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef ION_GSMS_METADATA_HANDLER_H
#define ION_GSMS_METADATA_HANDLER_H

#include "GnssMetadata.h"
#include "configuration_interface.h"
#include <gnuradio/block.h>
#include <string>
#include <vector>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_libs
 * \{ */


class IONGSMSMetadataHandler
{
public:
    explicit IONGSMSMetadataHandler(const std::string& metadata_filepath);

    std::vector<IONGSMSFileSource::sptr> make_stream_sources(const ConfigurationInterface* configuration, const std::string& role, const std::vector<std::string>& stream_ids) const;
    const std::string& metadata_filepath() const;

private:
    void load_metadata();

    // State
    std::string metadata_filepath_;
    GnssMetadata::Metadata metadata_;
};


/** \} */
/** \} */
#endif  // ION_GSMS_METADATA_HANDLER_H
