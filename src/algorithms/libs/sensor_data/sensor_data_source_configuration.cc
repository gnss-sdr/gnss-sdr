/*!
 * \file sensor_data_source_configuration.cc
 * \brief
 * \author Victor Castillo, 2025. victorcastilloaguero(at).gmail.es
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

#include "sensor_data_source_configuration.h"

SensorDataSourceConfiguration::SensorDataSourceConfiguration(const ConfigurationInterface* configuration)
    : enabled_(configuration->property("SensorData.enabled"s, false)), items_per_sample_(1)
{
    if (enabled_)
        {
            configure_files(configuration);
            configure_sensors(configuration);
        }
}

bool SensorDataSourceConfiguration::validate() const
{
    return validate_files() and validate_sensors();
}

bool SensorDataSourceConfiguration::is_enabled() const
{
    return enabled_;
}

bool SensorDataSourceConfiguration::is_sensor_provided(SensorIdentifier::value_type sensor_id) const
{
    if (not enabled_)
        {
            return false;
        }

    bool sensor_found = false;
    for (const auto& sensor : sensors_)
        {
            if (sensor_id == sensor.identifier)
                {
                    sensor_found = true;
                }
        }
    return sensor_found;
}


void SensorDataSourceConfiguration::set_items_per_sample(uint64_t items_per_sample)
{
    items_per_sample_ = items_per_sample;
}

uint64_t SensorDataSourceConfiguration::get_items_per_sample() const
{
    return items_per_sample_;
}


const std::unordered_map<uint64_t, SensorDataFileConfiguration>& SensorDataSourceConfiguration::files() const
{
    return files_;
}

const std::vector<SensorDataConfiguration>& SensorDataSourceConfiguration::sensors() const
{
    return sensors_;
}

void SensorDataSourceConfiguration::configure_files(const ConfigurationInterface* configuration)
{
    for (uint64_t id = 0;; ++id)
        {
            std::string role = CONFIGURATION_ROLE + ".file" + std::to_string(id);

            // Find out which sensor this is
            const std::string filename = configuration->property(role + ".filename"s, std::string{""});

            if (filename.empty())
                {
                    // No more files
                    break;
                }

            files_.emplace(
                id,
                SensorDataFileConfiguration{
                    .id = id,
                    .filename = filename,
                    .repeat = configuration->property(role + ".repeat"s, false),
                    .chunk_size = configuration->property(role + ".chunk_size"s, 0UL),
                    .file_offset = configuration->property(role + ".file_offset"s, 0UL),
                    .sample_offset = configuration->property(role + ".sample_offset"s, 0UL),
                    .sample_period = configuration->property(role + ".sample_period"s, 0UL)});
        }
}

void SensorDataSourceConfiguration::configure_sensors(const ConfigurationInterface* configuration)
{
    for (uint64_t id = 0;; ++id)
        {
            std::string role = CONFIGURATION_ROLE + ".sensor" + std::to_string(id);

            // Find out which sensor this is
            const std::string sensor_identifier = configuration->property(role + ".data"s, std::string{"UNDEFINED"});

            if (sensor_identifier == "UNDEFINED")
                {
                    // No more sensors
                    break;
                }

            // Configure sensor data type, default to same data type as previous sensor
            SensorDataType::value_type data_type = SensorDataType::FLOAT;
            if (id > 0 and not configuration->is_present(role + ".type"))
                {
                    data_type = sensors_[id - 1].type;
                }
            else
                {
                    data_type = SensorDataType::from_string(configuration->property(role + ".type"s, std::string{"UNKNOWN"}));
                }

            // Configure offset, default to previous sensor offset plus previous sensor size
            uint64_t offset = 0UL;
            if (id > 0 and not configuration->is_present(role + ".offset"))
                {
                    offset = sensors_[id - 1].offset + SensorDataType::get_size(sensors_[id - 1].type);
                }
            else
                {
                    offset = configuration->property(role + ".offset"s, 0UL);
                }

            // Configure file_id, default to previous sensor file_id
            uint64_t file_id = 0UL;
            if (id > 0 and not configuration->is_present(role + ".file"))
                {
                    file_id = sensors_[id - 1].file_id;
                }
            else
                {
                    file_id = configuration->property(role + ".file"s, 0UL);
                }

            if (sensor_identifier != "UNDEFINED")
                {
                    sensors_.emplace_back(SensorDataConfiguration{
                        .id = id,
                        .file_id = file_id,
                        .identifier = SensorIdentifier::from_string(sensor_identifier),
                        .type = data_type,
                        .offset = offset,
                        .tag_key = pmt::mp(sensor_identifier)});
                }
        }
}


bool SensorDataSourceConfiguration::validate_files() const
{
    for (const auto& file : files_)
        {
            // TODO - Implement
        }
    return true;
}

bool SensorDataSourceConfiguration::validate_sensors() const
{
    for (const auto& sensor : sensors_)
        {
            // TODO - Implement
        }
    return true;
}
