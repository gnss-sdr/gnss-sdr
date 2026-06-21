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
#include "gnss_sdr_filesystem.h"
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
    size_t getRfChannels() const override;

    inline size_t item_size() override
    {
        if (sources_.empty())
            {
                return 0;
            }
        return sources_.begin()->source->output_stream_item_size(0);
    }

private:
    static constexpr double kMinimumTailSeconds = 0.2;

    struct StreamSourceData
    {
        IONGSMSFileSource::sptr source;
        std::int64_t sampling_frequency = 0;
    };

    struct MetadataFileData
    {
        const GnssMetadata::File* file = nullptr;
        fs::path metadata_directory;
    };

    static std::vector<std::string> parse_comma_list(const std::string& str);
    static bool block_contains_stream(const GnssMetadata::Block& block, const std::vector<std::string>& stream_ids);
    static std::size_t chunk_cycle_bytes(const GnssMetadata::Block& block);
    static std::size_t infer_block_cycles(
        const fs::path& data_filepath,
        const GnssMetadata::Block& block,
        std::size_t block_start_offset,
        bool block_extends_to_eof);
    static std::size_t block_storage_bytes(
        const fs::path& data_filepath,
        const GnssMetadata::Block& block,
        std::size_t block_start_offset,
        bool block_extends_to_eof);
    static const char* file_uri_scheme();
    static bool starts_with(const std::string& value, const std::string& prefix);
    static fs::path absolute_path_key(const fs::path& path);
    static fs::path resolve_local_metadata_uri(const fs::path& metadata_path, const std::string& uri);

    std::vector<StreamSourceData> make_stream_sources(const std::vector<std::string>& stream_ids) const;
    std::vector<MetadataFileData> ordered_metadata_files() const;
    const GnssMetadata::System* resolve_system(const GnssMetadata::System& system) const;
    double lane_base_frequency_hz(const GnssMetadata::Lane& lane) const;
    std::int64_t stream_sampling_frequency_hz(
        const GnssMetadata::Lane& lane,
        const GnssMetadata::Block& block,
        const std::string& stream_id) const;
    static std::int64_t reconcile_sampling_frequency(
        std::int64_t current_frequency,
        std::int64_t candidate_frequency,
        const std::string& stream_id);

    void load_metadata();
    void load_metadata_file(
        const fs::path& metadata_path,
        GnssMetadata::Metadata& metadata,
        std::vector<std::string>& include_stack);
    std::uint64_t valve_sample_count(std::uint64_t total_sample_count, std::int64_t sampling_frequency) const;

    std::vector<std::string> stream_ids_;
    std::vector<StreamSourceData> sources_;
    std::vector<gnss_shared_ptr<gr::block>> copy_blocks_;
    std::vector<gnss_shared_ptr<gr::block>> valves_;

    std::string metadata_filepath_;
    std::shared_ptr<GnssMetadata::Metadata> metadata_;
    std::vector<MetadataFileData> metadata_files_;

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
