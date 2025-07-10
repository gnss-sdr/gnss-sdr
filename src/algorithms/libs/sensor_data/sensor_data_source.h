/*!
 * \file sensor_data_source.h
 * \brief  GNURadio block that adds extra data to the sample stream.
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


#ifndef GNSS_SDR_SENSOR_DATA_SOURCE_H
#define GNSS_SDR_SENSOR_DATA_SOURCE_H

#include "gnss_block_interface.h"
#include "sensor_data/sensor_data_file.h"
#include "sensor_data/sensor_data_source_configuration.h"
#include <gnuradio/sync_block.h>  // for sync_block
#include <gnuradio/types.h>       // for gr_vector_const_void_star
#include <cstddef>                // for size_t

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
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

/** \} */
/** \} */
#endif  // GNSS_SDR_SENSOR_DATA_SOURCE_H
