/*!
 * \file sensor_data_aggregator.cc
 * \brief  Aggregates sensor samples from gnu radio stream tags into typed lists for easy access
 * \author Victor Castillo, 2024. victorcastilloaguero(at)gmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2024-2025  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "sensor_data_aggregator.h"
#include <sstream>
#include <string>
#include <utility>

SensorDataAggregator::SensorDataAggregator(const SensorDataSourceConfiguration& configuration, const std::vector<SensorIdentifier::value_type>& required_sensors)
{
    std::vector<SensorIdentifier::value_type> missing_sensors{};
    for (const auto& required_sensor : required_sensors)
        {
            if (not configuration.is_sensor_provided(required_sensor))
                {
                    // Sensor was not provided in the configuration file
                    missing_sensors.push_back(required_sensor);
                }
            else
                {
                    // Populate sensor sample maps
                    switch (SensorIdentifier::get_internal_type(required_sensor))
                        {
                        case SensorDataType::F32:
                            f32_data_[required_sensor] = {};
                            break;

                            // More maps to be populated in the future for different types
                            // For now, all supported sensors are represented as f32

                        default:
                            break;
                        }
                }
        }

    if (not missing_sensors.empty())
        {
            // TODO - Throw error if not all ok
            std::stringstream ss;
            ss << "ERROR: Required sensors were not provided: ";
            ss << SensorIdentifier::to_string(missing_sensors[0]);
            for (std::size_t i = 1; i < missing_sensors.size(); ++i)
                {
                    ss << ", ";
                    ss << SensorIdentifier::to_string(missing_sensors[i]);
                }
            throw std::runtime_error(ss.str());
        }
}


void SensorDataAggregator::update(const std::vector<gr::tag_t>& tags)
{
    // Delete all data except last sample for each sensor
    for (auto& sensor_data : f32_data_)
        {
            std::vector<SensorDataSample<float>>& sensor_samples = sensor_data.second;
            if (not sensor_samples.empty())
                {
                    SensorDataSample<float> last_sample = sensor_samples.back();
                    sensor_samples.clear();
                    sensor_samples.emplace_back(last_sample);
                }
        }
    // More maps to be cleared in the future for different types
    // For now, all supported sensors are represented as f32

    // Append new data
    for (const auto& sensor_tag : tags)
        {
            if (pmt::is_dict(sensor_tag.value))
                {
                    append_data(sensor_tag.value);
                }
        }
}


const std::vector<SensorDataSample<float>>& SensorDataAggregator::get_f32(SensorIdentifier::value_type sensor_id) const
{
    // The map is populated on construction with empty vectors for each provided sensor.
    // If a required sensor is not provided, the error is handled on construction.
    return f32_data_.at(sensor_id);
}


SensorDataSample<float> SensorDataAggregator::get_last_f32(SensorIdentifier::value_type sensor_id) const
{
    // The map is populated on construction with empty vectors for each provided sensor.
    // If a required sensor is not provided, the error is handled on construction.
    const std::vector<SensorDataSample<float>> samples = f32_data_.at(sensor_id);
    if (samples.empty())
        {
            return {0, 0};
        }
    return samples.back();
}


void SensorDataAggregator::append_data(const pmt::pmt_t& data_dict)
{
    static pmt::pmt_t SAMPLE_STAMP_KEY = pmt::mp(SensorIdentifier::to_string(SensorIdentifier::SAMPLE_STAMP));
    pmt::pmt_t data_list = pmt::dict_items(data_dict);

    uint64_t sample_stamp = pmt::to_uint64(pmt::dict_ref(data_dict, SAMPLE_STAMP_KEY, pmt::from_uint64(0)));
    while (not pmt::is_null(data_list))
        {
            pmt::pmt_t pair = pmt::car(data_list);
            pmt::pmt_t key = pmt::car(pair);
            pmt::pmt_t val = pmt::cdr(pair);

            std::string key_str = pmt::write_string(std::move(key));
            SensorIdentifier::value_type sensor_id = SensorIdentifier::from_string(key_str);

            if (sensor_id != SensorIdentifier::SAMPLE_STAMP and sensor_id != SensorIdentifier::CHUNK_COUNT)
                {
                    switch (SensorIdentifier::get_internal_type(sensor_id))
                        {
                        case SensorDataType::F32:
                            if (f32_data_.find(sensor_id) != f32_data_.end())
                                {
                                    f32_data_.at(sensor_id).emplace_back(sample_stamp, pmt::to_float(std::move(val)));
                                }
                            break;

                            // More types to be handled in the future for different types
                            // For now, all supported sensors are represented as f32

                        default:
                            break;
                        }
                }

            data_list = pmt::cdr(data_list);
        }
}
