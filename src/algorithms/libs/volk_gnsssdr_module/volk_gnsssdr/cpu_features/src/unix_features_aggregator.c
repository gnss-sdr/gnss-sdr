// SPDX-FileCopyrightText: 2017 Google Inc.
// SPDX-License-Identifier: Apache-2.0


#include "internal/unix_features_aggregator.h"
#include "internal/string_view.h"

void CpuFeatures_SetFromFlags(const size_t configs_size,
    const CapabilityConfig* configs,
    const StringView flags_line,
    void* const features)
{
    size_t i = 0;
    for (; i < configs_size; ++i)
        {
            const CapabilityConfig config = configs[i];
            config.set_bit(features, CpuFeatures_StringView_HasWord(
                                         flags_line, config.proc_cpuinfo_flag));
        }
}

static bool IsSet(const uint32_t mask, const uint32_t value)
{
    if (mask == 0) return false;
    return (value & mask) == mask;
}

static bool IsHwCapsSet(const HardwareCapabilities hwcaps_mask,
    const HardwareCapabilities hwcaps)
{
    return IsSet(hwcaps_mask.hwcaps, hwcaps.hwcaps) ||
           IsSet(hwcaps_mask.hwcaps2, hwcaps.hwcaps2);
}

void CpuFeatures_OverrideFromHwCaps(const size_t configs_size,
    const CapabilityConfig* configs,
    const HardwareCapabilities hwcaps,
    void* const features)
{
    size_t i = 0;
    for (; i < configs_size; ++i)
        {
            const CapabilityConfig* config = &configs[i];
            if (IsHwCapsSet(config->hwcaps_mask, hwcaps))
                {
                    config->set_bit(features, true);
                }
        }
}
