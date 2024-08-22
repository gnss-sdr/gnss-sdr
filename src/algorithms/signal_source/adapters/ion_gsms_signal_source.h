/*!
 * \file ion_gsms_signal_source.h
 * \brief GNSS-SDR Signal Source that reads sample streams following ION's GNSS-SDR metadata standard
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


#ifndef GNSS_SDR_ION_GSMS_SIGNAL_SOURCE_H
#define GNSS_SDR_ION_GSMS_SIGNAL_SOURCE_H

#include "configuration_interface.h"
#include "file_source_base.h"
#include "gnss_sdr_timestamp.h"
#include "ion_gsms.h"
#include <GnssMetadata.h>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */

/*!
 * \brief Class that reads signals samples from a file
 * and adapts it to a SignalSourceInterface
 */
class IONGSMSSignalSource : public SignalSourceBase
{
public:
    IONGSMSSignalSource(const ConfigurationInterface* configuration, const std::string& role,
        unsigned int in_streams, unsigned int out_streams,
        Concurrent_Queue<pmt::pmt_t>* queue);

    ~IONGSMSSignalSource() override = default;

protected:
    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;

    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;
    gr::basic_block_sptr get_right_block(int RF_channel) override;

    inline size_t item_size() override
    {
        return (*sources_.begin())->output_stream_item_size(0);
    }

private:
    std::vector<IONGSMSFileSource::sptr> make_stream_sources(const std::vector<std::string>& stream_ids) const;

    void load_metadata();

    std::vector<std::string> stream_ids_;
    std::vector<IONGSMSFileSource::sptr> sources_;
    std::vector<gnss_shared_ptr<gr::block>> copy_blocks_;
    std::vector<gnss_shared_ptr<gr::block>> valves_;

    std::string metadata_filepath_;
    std::shared_ptr<GnssMetadata::Metadata> metadata_;

    gnss_shared_ptr<Gnss_Sdr_Timestamp> timestamp_block_;
    std::string timestamp_file_;

    uint32_t in_streams_;
    uint32_t out_streams_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_ION_GSMS_SIGNAL_SOURCE_H
