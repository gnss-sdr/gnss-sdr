// SPDX-FileCopyrightText: 2017 Google LLC
// SPDX-License-Identifier: Apache-2.0

#include "internal/hwcaps.h"
#include <stdbool.h>
#include <stdint.h>

static bool IsSet(const uint64_t mask, const uint64_t value)
{
    if (mask == 0) return false;
    return (value & mask) == mask;
}

bool CpuFeatures_IsHwCapsSet(const HardwareCapabilities hwcaps_mask,
    const HardwareCapabilities hwcaps)
{
    return IsSet(hwcaps_mask.hwcaps, hwcaps.hwcaps) ||
           IsSet(hwcaps_mask.hwcaps2, hwcaps.hwcaps2);
}
