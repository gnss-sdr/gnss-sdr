/*!
 * \file sensor_data_source.cc
 * \brief  GNURadio block that adds extra data to the sample stream.
 * \author Victor Castillo, 2024. victorcastilloaguero(at).gmail.es
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

#include "sensor_data/sensor_data_source.h"
#include "sensor_data/sensor_data_file.h"
#include <pmt/pmt.h>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

using namespace std::string_literals;

SensorDataSource::SensorDataSource(
    const SensorDataSourceConfiguration& configuration,
    const gr::io_signature::sptr& io_signature)
    : gr::sync_block("Sensor Data Source",
          io_signature, io_signature),
      sensor_data_files_({}),
      item_size_(io_signature->sizeof_stream_item(0)),
      items_per_sample_(configuration.get_items_per_sample())
{
    if (not configuration.validate())
        {
            DLOG(ERROR) << "Failed to validate sensor data configuration";
            throw std::runtime_error("Failed to validate sensor data configuration");
        }

    // Open needed data files
    for (const auto& file_pair : configuration.files())
        {
            const auto& id = file_pair.first;
            const auto& file = file_pair.second;

            std::size_t s_offset = file.sample_offset;
            std::size_t s_period = file.sample_period;
            if (items_per_sample_ != 1)
                {
                    s_offset *= items_per_sample_;
                    s_period *= items_per_sample_;
                }

            sensor_data_files_.emplace(id, gnss_make_shared<SensorDataFile>(file.filename, s_offset, s_period, file.file_offset, file.chunk_size, file.repeat));

            if (sensor_config_map_.find(id) == sensor_config_map_.end())
                {
                    sensor_config_map_[id] = {};
                }
        }

    // Populate sensor map (groups sensors by file ID)
    for (const auto& sensor : configuration.sensors())
        {
            sensor_config_map_.at(sensor.file_id).emplace_back(sensor);
        }

    // Sort lists in sensor map by byte offset within chunk
    for (auto& it : sensor_config_map_)
        {
            auto& sensors_in_file = it.second;
            std::sort(sensors_in_file.begin(), sensors_in_file.end(), [](const SensorDataConfiguration& lhs, const SensorDataConfiguration& rhs) -> bool {
                return lhs.offset < rhs.offset;
            });
        }

    // Validate IO signature
    if (io_signature->min_streams() != 1 and io_signature->max_streams() != 1)
        {
            std::cout << "ERROR: This block only supports adding data to a single stream." << "\n";
        }
}

int SensorDataSource::work(int noutput_items,
    gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items)
{
    static pmt::pmt_t TAG_KEY = pmt::mp("sensor_data");
    static pmt::pmt_t CHUNK_COUNT_KEY = pmt::mp("CHUNK_COUNT");
    static pmt::pmt_t SAMPLE_STAMP_KEY = pmt::mp("SAMPLE_STAMP");

    std::memcpy(output_items[0], input_items[0], noutput_items * item_size_);

    const uint64_t total_items_written = nitems_written(0) + noutput_items;

    std::size_t sample_stamp;
    std::vector<uint8_t> chunk{};
    for (auto& file_pair : sensor_data_files_)
        {
            const auto& file_id = file_pair.first;
            auto& data_file = file_pair.second;
            while (data_file->read_until_sample(total_items_written, sample_stamp, chunk))
                {
                    pmt::pmt_t data_tag = pmt::make_dict();
                    data_tag = pmt::dict_add(data_tag, SAMPLE_STAMP_KEY, pmt::from_uint64(sample_stamp / items_per_sample_));
                    data_tag = pmt::dict_add(data_tag, CHUNK_COUNT_KEY, pmt::from_long(data_file->get_chunks_read()));
                    for (const auto& sensor : sensor_config_map_.at(file_id))
                        {
                            pmt::pmt_t raw_value = SensorDataType::make_value(sensor.type, &chunk[sensor.offset]);
                            pmt::pmt_t value = SensorIdentifier::convert_to_internal_type(sensor.identifier, sensor.type, raw_value);
                            data_tag = pmt::dict_add(data_tag, sensor.tag_key, value);
                        }
                    add_item_tag(0, sample_stamp, TAG_KEY, data_tag);
                }
        }

    return noutput_items;
}
