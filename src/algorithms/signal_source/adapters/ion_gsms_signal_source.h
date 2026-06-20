/*!
 * \file ion_gsms_signal_source.h
 * \brief GNSS-SDR Signal Source that reads sample streams following ION's GNSS-SDR metadata standard
 * \author Víctor Castillo Agüero, 2024. victorcastilloaguero(at)gmail.com
 * \author Carles Fernandez, 2026 carles.fernandez(at)cttc.es
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
        if (sources_.empty())
            {
                return 0;
            }
        return (*sources_.begin())->output_stream_item_size(0);
    }

private:
    static constexpr double kMinimumTailSeconds = 0.2;

    static std::vector<std::string> parse_comma_list(const std::string& str);
    static bool block_contains_stream(const GnssMetadata::Block& block, const std::vector<std::string>& stream_ids);
    static std::size_t chunk_cycle_bytes(const GnssMetadata::Block& block);
    static std::size_t infer_block_cycles(
        const fs::path& data_filepath,
        const GnssMetadata::Block& block,
        std::size_t block_start_offset);
    static std::size_t block_storage_bytes(
        const fs::path& data_filepath,
        const GnssMetadata::Block& block,
        std::size_t block_start_offset);

    std::vector<IONGSMSFileSource::sptr> make_stream_sources(const std::vector<std::string>& stream_ids) const;

    void load_metadata();
    std::uint64_t valve_sample_count(std::uint64_t total_sample_count) const;

    std::vector<std::string> stream_ids_;
    std::vector<IONGSMSFileSource::sptr> sources_;
    std::vector<gnss_shared_ptr<gr::block>> copy_blocks_;
    std::vector<gnss_shared_ptr<gr::block>> valves_;

    std::string metadata_filepath_;
    std::shared_ptr<GnssMetadata::Metadata> metadata_;

    gnss_shared_ptr<Gnss_Sdr_Timestamp> timestamp_block_;
    std::string timestamp_file_;
    double minimum_tail_s_;
    int64_t sampling_frequency_;

    uint32_t in_streams_;
    uint32_t out_streams_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_ION_GSMS_SIGNAL_SOURCE_H
