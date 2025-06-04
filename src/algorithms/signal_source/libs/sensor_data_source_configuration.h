/*!
 * \file sensor_data_source_configuration.h
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


#ifndef GNSS_SDR_SENSOR_DATA_SOURCE_CONFIGURATION_H
#define GNSS_SDR_SENSOR_DATA_SOURCE_CONFIGURATION_H

#include "configuration_interface.h"
#include "sensor_data_type.h"
#include "sensor_identifier.h"
#include <string>
#include <unordered_map>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_gnuradio_blocks
 * \{ */

using namespace std::string_literals;

static std::string CONFIGURATION_ROLE = "SensorData";

class ConfigurationInterface;

struct SensorDataFileConfiguration
{
    uint64_t id;
    std::string filename;
    bool repeat;
    uint64_t chunk_size;
    uint64_t file_offset;
    uint64_t sample_offset;
    uint64_t sample_period;
};

struct SensorDataConfiguration
{
    uint64_t id;
    uint64_t file_id;
    SensorIdentifier::value_type identifier;
    SensorDataType::value_type type;
    uint64_t offset;

    pmt::pmt_t tag_key;
};

class SensorDataSourceConfiguration
{
public:
    explicit SensorDataSourceConfiguration(const ConfigurationInterface* configuration, bool enabled);

    bool validate() const;

    bool is_enabled() const;

    const std::unordered_map<uint64_t, SensorDataFileConfiguration>& files() const;

    const std::vector<SensorDataConfiguration>& sensors() const;

    void set_items_per_sample(uint64_t items_per_sample);

    uint64_t get_items_per_sample() const;

private:
    void configure_files(const ConfigurationInterface* configuration);

    void configure_sensors(const ConfigurationInterface* configuration);


    bool validate_files() const;

    bool validate_sensors() const;

private:
    bool enabled_;
    std::unordered_map<uint64_t, SensorDataFileConfiguration> files_;
    std::vector<SensorDataConfiguration> sensors_;
    uint64_t items_per_sample_;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_SENSOR_DATA_SOURCE_CONFIGURATION_H
