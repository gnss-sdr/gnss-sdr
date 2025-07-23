/*!
 * \file receiver_type.cc
 * \brief Helper function to get the receiver type
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

#include "receiver_type.h"
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
            auto prop = pair_aux.second;
            const auto enabled = configuration->property(prop, 0) > 0;

            if (enabled)
                {
                    flags |= flag;
                }
        }

    return flags;
}
}  // namespace


Signal_Enabled_Flags::Signal_Enabled_Flags(const ConfigurationInterface* configuration) : flags(flags_from_config(configuration))
{
}


Signal_Enabled_Flags::Signal_Enabled_Flags(uint32_t flags_) : flags(flags_)
{
}


uint32_t get_type_of_receiver(const Signal_Enabled_Flags& signal_enabled_flags)
{
    if (signal_enabled_flags.check_only_enabled(GPS_1C))
        {
            return 1;  // GPS L1 C/A
        }
    if (signal_enabled_flags.check_only_enabled(GPS_2S))
        {
            return 2;  // GPS L2C
        }
    if (signal_enabled_flags.check_only_enabled(GPS_L5))
        {
            return 3;  // L5
        }
    if (signal_enabled_flags.check_only_enabled(GAL_1B))
        {
            return 4;  // E1
        }
    if (signal_enabled_flags.check_only_enabled(GAL_E5a))
        {
            return 5;  // E5a
        }
    if (signal_enabled_flags.check_only_enabled(GAL_E5b))
        {
            return 6;  // E5b
        }
    if (signal_enabled_flags.check_only_enabled(GPS_1C, GPS_2S))
        {
            return 7;  // GPS L1 C/A + GPS L2C
        }
    if (signal_enabled_flags.check_only_enabled(GPS_1C, GPS_L5))
        {
            return 8;  // L1+L5
        }
    if (signal_enabled_flags.check_only_enabled(GPS_1C, GAL_1B))
        {
            return 9;  // L1+E1
        }
    if (signal_enabled_flags.check_only_enabled(GPS_1C, GAL_E5a))
        {
            return 10;  // GPS L1 C/A + Galileo E5a
        }
    if (signal_enabled_flags.check_only_enabled(GPS_1C, GAL_E5b))
        {
            return 11;  // GPS L1 C/A + Galileo E5b
        }
    if (signal_enabled_flags.check_only_enabled(GPS_2S, GAL_1B))
        {
            return 12;  // Galileo E1B + GPS L2C
        }
    if (signal_enabled_flags.check_only_enabled(GPS_L5, GAL_E5a))
        {
            return 13;  // L5+E5a
        }
    if (signal_enabled_flags.check_only_enabled(GAL_1B, GAL_E5a))
        {
            return 14;  // Galileo E1B + Galileo E5a
        }
    if (signal_enabled_flags.check_only_enabled(GAL_1B, GAL_E5b))
        {
            return 15;  // Galileo E1B + Galileo E5b
        }
    if (signal_enabled_flags.check_only_enabled(GPS_2S, GPS_L5))
        {
            return 16;  // GPS L2C + GPS L5
        }
    if (signal_enabled_flags.check_only_enabled(GPS_2S, GAL_E5a))
        {
            return 17;  // GPS L2C + Galileo E5a
        }
    if (signal_enabled_flags.check_only_enabled(GPS_2S, GAL_E5b))
        {
            return 18;  // GPS L2C + Galileo E5b
        }
    if (signal_enabled_flags.check_only_enabled(GAL_E5a, GAL_E5b))
        {
            return 19;  // Galileo E5a + Galileo E5b
        }
    if (signal_enabled_flags.check_only_enabled(GPS_L5, GAL_E5b))
        {
            return 20;  // GPS L5 + Galileo E5b
        }
    if (signal_enabled_flags.check_only_enabled(GPS_1C, GAL_1B, GPS_2S))
        {
            return 21;  // GPS L1 C/A + Galileo E1B + GPS L2C
        }
    if (signal_enabled_flags.check_only_enabled(GPS_1C, GAL_1B, GPS_L5))
        {
            return 22;  // GPS L1 C/A + Galileo E1B + GPS L5
        }
    if (signal_enabled_flags.check_only_enabled(GLO_1G))
        {
            return 23;  // GLONASS L1 C/A
        }
    if (signal_enabled_flags.check_only_enabled(GLO_2G))
        {
            return 24;  // GLONASS L2 C/A
        }
    if (signal_enabled_flags.check_only_enabled(GLO_1G, GLO_2G))
        {
            return 25;  // GLONASS L1 C/A + GLONASS L2 C/A
        }
    if (signal_enabled_flags.check_only_enabled(GPS_1C, GLO_1G))
        {
            return 26;  // GPS L1 C/A + GLONASS L1 C/A
        }
    if (signal_enabled_flags.check_only_enabled(GAL_1B, GLO_1G))
        {
            return 27;  // Galileo E1B + GLONASS L1 C/A
        }
    if (signal_enabled_flags.check_only_enabled(GPS_2S, GLO_1G))
        {
            return 28;  // GPS L2C + GLONASS L1 C/A
        }
    if (signal_enabled_flags.check_only_enabled(GPS_1C, GLO_2G))
        {
            return 29;  // GPS L1 C/A + GLONASS L2 C/A
        }
    if (signal_enabled_flags.check_only_enabled(GAL_1B, GLO_2G))
        {
            return 30;  // Galileo E1B + GLONASS L2 C/A
        }
    if (signal_enabled_flags.check_only_enabled(GPS_2S, GLO_2G))
        {
            return 31;  // GPS L2C + GLONASS L2 C/A
        }
    if (signal_enabled_flags.check_only_enabled(GPS_1C, GAL_1B, GPS_L5, GAL_E5a))
        {
            return 32;  // L1+E1+L5+E5a
        }
    if (signal_enabled_flags.check_only_enabled(GPS_1C, GAL_1B, GAL_E5a))
        {
            return 33;  // L1+E1+E5a
        }
    // Galileo E6
    if (signal_enabled_flags.check_only_enabled(GAL_E6))
        {
            return 100;  // Galileo E6B
        }
    if (signal_enabled_flags.check_only_enabled(GAL_1B, GAL_E6))
        {
            return 101;  // Galileo E1B + Galileo E6B
        }
    if (signal_enabled_flags.check_only_enabled(GAL_E5a, GAL_E6))
        {
            return 102;  // Galileo E5a + Galileo E6B
        }
    if (signal_enabled_flags.check_only_enabled(GAL_E5b, GAL_E6))
        {
            return 103;  // Galileo E5b + Galileo E6B
        }
    if (signal_enabled_flags.check_only_enabled(GAL_1B, GAL_E5a, GAL_E6))
        {
            return 104;  // Galileo E1B + Galileo E5a + Galileo E6B
        }
    if (signal_enabled_flags.check_only_enabled(GAL_1B, GAL_E5b, GAL_E6))
        {
            return 105;  // Galileo E1B + Galileo E5b + Galileo E6B
        }
    if (signal_enabled_flags.check_only_enabled(GPS_1C, GAL_1B, GAL_E6))
        {
            return 106;  // GPS L1 C/A + Galileo E1B + Galileo E6B
        }
    if (signal_enabled_flags.check_only_enabled(GPS_1C, GAL_E6))
        {
            return 107;  // GPS L1 C/A + Galileo E6B
        }
    if (signal_enabled_flags.check_only_enabled(GPS_1C, GAL_1B, GPS_L5, GAL_E5a, GAL_E6))
        {
            return 108;  // GPS L1 C/A + Galileo E1B + GPS L5 + Galileo E5a + Galileo E6B
        }
    // BeiDou B1I Receiver
    if (signal_enabled_flags.check_only_enabled(BDS_B1))
        {
            return 500;  // Beidou B1I
        }
    if (signal_enabled_flags.check_only_enabled(BDS_B1, GPS_1C))
        {
            return 501;  // Beidou B1I + GPS L1 C/A
        }
    if (signal_enabled_flags.check_only_enabled(BDS_B1, GAL_1B))
        {
            return 502;  // Beidou B1I + Galileo E1B
        }
    if (signal_enabled_flags.check_only_enabled(BDS_B1, GLO_1G))
        {
            return 503;  // Beidou B1I + GLONASS L1 C/A
        }
    if (signal_enabled_flags.check_only_enabled(BDS_B1, GPS_1C, GAL_1B))
        {
            return 504;  // Beidou B1I + GPS L1 C/A + Galileo E1B
        }
    if (signal_enabled_flags.check_only_enabled(BDS_B1, GPS_1C, GLO_1G, GAL_1B))
        {
            return 505;  // Beidou B1I + GPS L1 C/A + GLONASS L1 C/A + Galileo E1B
        }
    if (signal_enabled_flags.check_only_enabled(BDS_B1, BDS_B3))
        {
            return 506;  // Beidou B1I + Beidou B3I
        }
    // BeiDou B3I Receiver
    if (signal_enabled_flags.check_only_enabled(BDS_B3))
        {
            return 600;  // Beidou B3I
        }
    if (signal_enabled_flags.check_only_enabled(BDS_B3, GPS_2S))
        {
            return 601;  // Beidou B3I + GPS L2C
        }
    if (signal_enabled_flags.check_only_enabled(BDS_B3, GLO_2G))
        {
            return 602;  // Beidou B3I + GLONASS L2 C/A
        }
    if (signal_enabled_flags.check_only_enabled(BDS_B3, GPS_2S, GLO_2G))
        {
            return 603;  // Beidou B3I + GPS L2C + GLONASS L2 C/A
        }
    if (signal_enabled_flags.check_only_enabled(GPS_1C, GPS_2S, GPS_L5))
        {
            return 1000;  // GPS L1 + GPS L2C + GPS L5
        }
    if (signal_enabled_flags.check_only_enabled(GPS_1C, GAL_1B, GPS_2S, GPS_L5, GAL_E5a))
        {
            return 1001;  // GPS L1 + Galileo E1B + GPS L2C + GPS L5 + Galileo E5a
        }

    return 0;
}
