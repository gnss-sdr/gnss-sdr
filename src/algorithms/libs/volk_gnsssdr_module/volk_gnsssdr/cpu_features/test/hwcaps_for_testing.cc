// SPDX-FileCopyrightText: 2017 Google LLC
// SPDX-License-Identifier: Apache-2.0

#include "hwcaps_for_testing.h"
#include "internal/string_view.h"
#include <string.h>

namespace cpu_features
{
namespace
{
static auto* const g_hardware_capabilities = new HardwareCapabilities();
static auto* const g_platform_types = new PlatformType();
}  // namespace

void SetHardwareCapabilities(uint32_t hwcaps, uint32_t hwcaps2)
{
    g_hardware_capabilities->hwcaps = hwcaps;
    g_hardware_capabilities->hwcaps2 = hwcaps2;
}

HardwareCapabilities CpuFeatures_GetHardwareCapabilities(void)
{
    return *g_hardware_capabilities;
}

void SetPlatformTypes(const char* platform, const char* base_platform)
{
    CpuFeatures_StringView_CopyString(str(platform), g_platform_types->platform,
        sizeof(g_platform_types->platform));
    CpuFeatures_StringView_CopyString(str(base_platform),
        g_platform_types->base_platform,
        sizeof(g_platform_types->base_platform));
}

PlatformType CpuFeatures_GetPlatformType(void) { return *g_platform_types; }
}  // namespace cpu_features
