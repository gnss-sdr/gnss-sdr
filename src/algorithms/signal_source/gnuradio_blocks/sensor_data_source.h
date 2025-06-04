/*!
 * \file sensor_data_source.h
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


#ifndef GNSS_SDR_SENSOR_DATA_SOURCE_H
#define GNSS_SDR_SENSOR_DATA_SOURCE_H

#include "gnss_block_interface.h"
#include "sensor_data_file.h"
#include "sensor_data_source_configuration.h"
#include <gnuradio/sync_block.h>  // for sync_block
#include <gnuradio/types.h>       // for gr_vector_const_void_star
#include <cstddef>                // for size_t
#include <cstdint>
#include <string>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_gnuradio_blocks
 * \{ */

class SensorDataSource : public gr::sync_block
{
public:
    using sptr = gnss_shared_ptr<SensorDataSource>;

    SensorDataSource(
        const SensorDataSourceConfiguration& configuration,
        const gr::io_signature::sptr& io_signature);

    int work(int noutput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items) override;

private:
    std::unordered_map<SensorDataFile::id_type, SensorDataFile::sptr> sensor_data_files_;
    std::unordered_map<SensorDataFile::id_type, std::vector<SensorDataConfiguration>> sensor_config_map_;
    std::size_t item_size_;
    std::size_t items_per_sample_;
};

class SensorDataAggregator
{
public:
    explicit SensorDataAggregator(std::vector<gr::tag_t> tags)
    {
        for (const auto& sensor_tag : tags)
            {
                if (sensor_tag.value->is_dict())
                    {
                        append_data(sensor_tag.value);
                    }
            }
    }

    auto get(SensorIdentifier::value_type sensor_id) const
    {
        if (data_.contains(sensor_id))
            {
                return data_.at(sensor_id);
            }
        else
            {
                return {};
            }
    }

private:
    void append_data(const pmt::pmt_t& data)
    {
        pmt::pmt_t data_list = pmt::dict_items(data);
        while (not pmt::is_null(data_list))
            {
                pmt::pmt_t pair = pmt::car(data_list);
                pmt::pmt_t key = pmt::car(pair);
                pmt::pmt_t val = pmt::cdr(pair);

                std::string key_str = pmt::write_string(key);
                SensorIdentifier::value_type sensor_id = SensorIdentifier::from_string(key_str);

                if (not data_.contains(sensor_id))
                    {
                        data_[sensor_id] = {};
                    }
                data_[sensor_id].emplace_back(val);
            }
    }

    std::unordered_map<SensorIdentifier::value_type, std::vector<pmt::pmt_t>> data_{};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_SENSOR_DATA_SOURCE_H
