// SPDX-FileCopyrightText: 2023 Google LLC
// SPDX-License-Identifier: Apache-2.0

#include "cpu_features_macros.h"

#ifdef CPU_FEATURES_ARCH_AARCH64
#ifdef CPU_FEATURES_OS_FREEBSD

#include "cpuinfo_aarch64.h"
#include "internal/cpuid_aarch64.h"
#include "internal/hwcaps.h"
#include "impl_aarch64__base_implementation.inl"

static const Aarch64Info kEmptyAarch64Info;

Aarch64Info GetAarch64Info(void)
{
    Aarch64Info info = kEmptyAarch64Info;
    const HardwareCapabilities hwcaps = CpuFeatures_GetHardwareCapabilities();
    for (size_t i = 0; i < AARCH64_LAST_; ++i)
        {
            if (CpuFeatures_IsHwCapsSet(kHardwareCapabilities[i], hwcaps))
                {
                    kSetters[i](&info.features, true);
                }
        }
    if (info.features.cpuid)
        {
            const uint64_t midr_el1 = GetMidrEl1();
            info.implementer = (int)ExtractBitRange(midr_el1, 31, 24);
            info.variant = (int)ExtractBitRange(midr_el1, 23, 20);
            info.part = (int)ExtractBitRange(midr_el1, 15, 4);
            info.revision = (int)ExtractBitRange(midr_el1, 3, 0);
        }
    return info;
}

#endif  // CPU_FEATURES_OS_FREEBSD
#endif  // CPU_FEATURES_ARCH_AARCH64
