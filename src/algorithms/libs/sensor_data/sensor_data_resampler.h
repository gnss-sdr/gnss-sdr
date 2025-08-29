/*!
 * \file sensor_data_resampler.h
 * \brief  Updates timestamp within sensor data tags. To be used in resampler blocks.
 * \author Victor Castillo, 2024. victorcastilloaguero(at)gmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2025  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_SENSOR_DATA_RESAMPLER_H
#define GNSS_SDR_SENSOR_DATA_RESAMPLER_H

#include <gnuradio/tags.h>
#include <vector>

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */


/** \brief Updates timestamp within sensor data tags. To be used in resampler blocks.
 *
 *  \param tags Stream tags as retrieved by `get_tags_in_window` or `get_tags_in_range`.
 *  \param freq_in Input RF sample rate.
 *  \param freq_out Output RF sample rate.
 *  \return Stream tags to be added with `add_item_tag`.
 */
std::vector<gr::tag_t> resample_sensor_data_tags(const std::vector<gr::tag_t>& tags, double freq_in, double freq_out);


/** \} */
/** \} */
#endif  // GNSS_SDR_SENSOR_DATA_RESAMPLER_H
