/*!
 * \file sensor_data_resampler.cc
 * \brief  Updates timestamp within sensor data tags. To be used within resampler blocks.
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

#include "sensor_data_resampler.h"
#include "sensor_identifier.h"

std::vector<gr::tag_t> resample_sensor_data_tags(const std::vector<gr::tag_t>& tags, double freq_in, double freq_out)
{
    static pmt::pmt_t SAMPLE_STAMP_KEY = pmt::mp(SensorIdentifier::to_string(SensorIdentifier::SAMPLE_STAMP));
    std::vector<gr::tag_t> new_tags{};
    for (auto& tag : tags)
        {
            if (pmt::dict_has_key(tag.value, SAMPLE_STAMP_KEY))
                {
                    auto& new_tag = new_tags.emplace_back();
                    uint64_t sample_stamp = pmt::to_uint64(pmt::dict_ref(tag.value, SAMPLE_STAMP_KEY, pmt::from_uint64(0)));
                    sample_stamp = sample_stamp * freq_out / freq_in;
                    new_tag.offset = tag.offset * freq_out / freq_in;
                    new_tag.key = tag.key;
                    new_tag.value = pmt::dict_add(tag.value, SAMPLE_STAMP_KEY, pmt::from_uint64(sample_stamp));
                }
        }
    return new_tags;
}
