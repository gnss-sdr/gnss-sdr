/*!
 * \file signal_enabled_flags.cc
 * \brief Class to check the enabled signals
 * \author Mathieu Favreau, 2025. favreau.mathieu(at)hotmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "signal_enabled_flags.h"
#include "configuration_interface.h"  // for ConfigurationInterface
#include <vector>                     // for vector

namespace
{
uint32_t flags_from_config(const ConfigurationInterface* configuration)
{
    uint32_t flags = 0;

    const std::vector<std::pair<uint32_t, std::string>> signal_flag_to_prop = {
        {GPS_1C, "Channels_1C.count"},
        {GPS_2S, "Channels_2S.count"},
        {GPS_L5, "Channels_L5.count"},
        {GAL_1B, "Channels_1B.count"},
        {GAL_E5a, "Channels_5X.count"},
        {GAL_E5b, "Channels_7X.count"},
        {GAL_E6, "Channels_E6.count"},
        {GLO_1G, "Channels_1G.count"},
        {GLO_2G, "Channels_2G.count"},
        {BDS_B1, "Channels_B1.count"},
        {BDS_B3, "Channels_B3.count"}};

    for (const auto& pair_aux : signal_flag_to_prop)
        {
            auto flag = pair_aux.first;
            const auto& prop = pair_aux.second;
            const auto enabled = configuration->property(prop, 0) > 0;

            if (enabled)
                {
                    flags |= flag;
                }
        }

    return flags;
}
}  // namespace


Signal_Enabled_Flags::Signal_Enabled_Flags(const ConfigurationInterface* configuration) : Signal_Enabled_Flags(flags_from_config(configuration))
{
}


Signal_Enabled_Flags::Signal_Enabled_Flags(uint32_t flags_) : flags(flags_),
                                                              has_gps(check_any_enabled(GPS_1C, GPS_2S, GPS_L5)),
                                                              has_galileo(check_any_enabled(GAL_1B, GAL_E5a, GAL_E5b, GAL_E6)),
                                                              has_glonass(check_any_enabled(GLO_1G, GLO_2G)),
                                                              has_beidou(check_any_enabled(BDS_B1, BDS_B3)),
                                                              only_gps(has_gps && !(has_galileo || has_glonass || has_beidou)),
                                                              only_galileo(has_galileo && !(has_gps || has_glonass || has_beidou)),
                                                              only_glonass(has_glonass && !(has_gps || has_galileo || has_beidou)),
                                                              only_beidou(has_beidou && !(has_gps || has_galileo || has_glonass))
{
}
