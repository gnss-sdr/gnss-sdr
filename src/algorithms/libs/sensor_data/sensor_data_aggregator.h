/*!
 * \file sensor_data_aggregator.h
 * \brief  Aggregates sensor samples from gnu radio stream tags into typed lists for easy access
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


#ifndef GNSS_SDR_SENSOR_DATA_AGGREGATOR_H
#define GNSS_SDR_SENSOR_DATA_AGGREGATOR_H

#include "sensor_data_source_configuration.h"
#include "sensor_identifier.h"
#include <gnss_block_interface.h>
#include <string>
#include <vector>

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */

template <typename DataType>
struct SensorDataSample
{
    uint64_t rf_sample_stamp{};
    DataType value{};
};

class SensorDataAggregator
{
public:
    const pmt::pmt_t SENSOR_DATA_TAG = pmt::mp("sensor_data");

    explicit SensorDataAggregator(const SensorDataSourceConfiguration& configuration, const std::vector<SensorIdentifier::value_type>& required_sensors);

    void update(const std::vector<gr::tag_t>& tags);

    const std::vector<SensorDataSample<float>>& get_f32(SensorIdentifier::value_type sensor_id) const;
    SensorDataSample<float> get_last_f32(SensorIdentifier::value_type sensor_id) const;

    // More getters to be added in the future for different types
    // For now, all supported sensors are represented as f32
private:
    void append_data(const pmt::pmt_t& data_dict);

    std::unordered_map<SensorIdentifier::value_type, std::vector<SensorDataSample<float>>> f32_data_{};
    // More maps to be added in the future for different types
    // For now, all supported sensors are represented as f32
};


/** \} */
/** \} */
#endif  // GNSS_SDR_SENSOR_DATA_AGGREGATOR_H
