// SPDX-FileCopyrightText: 2023 Google LLC
// SPDX-License-Identifier: Apache-2.0

#include "cpu_features_macros.h"

#ifdef CPU_FEATURES_ARCH_AARCH64
#ifdef CPU_FEATURES_OS_WINDOWS

#include "cpuinfo_aarch64.h"
#include "internal/windows_utils.h"
#include "impl_aarch64__base_implementation.inl"
#include <stdbool.h>

#ifdef CPU_FEATURES_MOCK_CPUID_AARCH64
extern bool GetWindowsIsProcessorFeaturePresent(DWORD);
extern WORD GetWindowsNativeSystemInfoProcessorRevision();
#else  // CPU_FEATURES_MOCK_CPUID_AARCH64
static bool GetWindowsIsProcessorFeaturePresent(DWORD dwProcessorFeature)
{
    return IsProcessorFeaturePresent(dwProcessorFeature);
}

static WORD GetWindowsNativeSystemInfoProcessorRevision()
{
    SYSTEM_INFO system_info;
    GetNativeSystemInfo(&system_info);
    return system_info.wProcessorRevision;
}
#endif

static const Aarch64Info kEmptyAarch64Info;

Aarch64Info GetAarch64Info(void)
{
    Aarch64Info info = kEmptyAarch64Info;
    info.revision = GetWindowsNativeSystemInfoProcessorRevision();
    info.features.fp =
        GetWindowsIsProcessorFeaturePresent(PF_ARM_VFP_32_REGISTERS_AVAILABLE);
    info.features.asimd =
        GetWindowsIsProcessorFeaturePresent(PF_ARM_NEON_INSTRUCTIONS_AVAILABLE);
    info.features.crc32 = GetWindowsIsProcessorFeaturePresent(
        PF_ARM_V8_CRC32_INSTRUCTIONS_AVAILABLE);
    info.features.asimddp =
        GetWindowsIsProcessorFeaturePresent(PF_ARM_V82_DP_INSTRUCTIONS_AVAILABLE);
    info.features.jscvt = GetWindowsIsProcessorFeaturePresent(
        PF_ARM_V83_JSCVT_INSTRUCTIONS_AVAILABLE);
    info.features.lrcpc = GetWindowsIsProcessorFeaturePresent(
        PF_ARM_V83_LRCPC_INSTRUCTIONS_AVAILABLE);
    info.features.atomics = GetWindowsIsProcessorFeaturePresent(
        PF_ARM_V81_ATOMIC_INSTRUCTIONS_AVAILABLE);

    bool is_crypto_available = GetWindowsIsProcessorFeaturePresent(
        PF_ARM_V8_CRYPTO_INSTRUCTIONS_AVAILABLE);
    info.features.aes = is_crypto_available;
    info.features.sha1 = is_crypto_available;
    info.features.sha2 = is_crypto_available;
    info.features.pmull = is_crypto_available;
    return info;
}

#endif  // CPU_FEATURES_OS_WINDOWS
#endif  // CPU_FEATURES_ARCH_AARCH64