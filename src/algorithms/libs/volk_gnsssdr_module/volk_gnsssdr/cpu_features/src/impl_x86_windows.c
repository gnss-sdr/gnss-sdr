// SPDX-FileCopyrightText: 2017 Google LLC
// SPDX-License-Identifier: Apache-2.0

#include "cpu_features_macros.h"

#ifdef CPU_FEATURES_ARCH_X86
#ifdef CPU_FEATURES_OS_WINDOWS

#include "impl_x86__base_implementation.inl"

static void OverrideOsPreserves(OsPreserves* os_preserves)
{
    (void)os_preserves;
    // No override
}

#include "internal/windows_utils.h"

#if defined(CPU_FEATURES_MOCK_CPUID_X86)
extern bool GetWindowsIsProcessorFeaturePresent(DWORD);
#else  // CPU_FEATURES_MOCK_CPUID_X86
static bool GetWindowsIsProcessorFeaturePresent(DWORD ProcessorFeature)
{
    return IsProcessorFeaturePresent(ProcessorFeature);
}
#endif

static void DetectFeaturesFromOs(X86Info* info, X86Features* features)
{
    // Handling Windows platform through IsProcessorFeaturePresent.
    // https://docs.microsoft.com/en-us/windows/win32/api/processthreadsapi/nf-processthreadsapi-isprocessorfeaturepresent
    features->sse =
        GetWindowsIsProcessorFeaturePresent(PF_XMMI_INSTRUCTIONS_AVAILABLE);
    features->sse2 =
        GetWindowsIsProcessorFeaturePresent(PF_XMMI64_INSTRUCTIONS_AVAILABLE);
    features->sse3 =
        GetWindowsIsProcessorFeaturePresent(PF_SSE3_INSTRUCTIONS_AVAILABLE);
    features->ssse3 =
        GetWindowsIsProcessorFeaturePresent(PF_SSSE3_INSTRUCTIONS_AVAILABLE);
    features->sse4_1 =
        GetWindowsIsProcessorFeaturePresent(PF_SSE4_1_INSTRUCTIONS_AVAILABLE);
    features->sse4_2 =
        GetWindowsIsProcessorFeaturePresent(PF_SSE4_2_INSTRUCTIONS_AVAILABLE);

    // Do not bother checking PF_AVX* : AVX enabled processor will have their XCR0
    // register exposed and this function will be skipped altogether.
}

#endif  // CPU_FEATURES_OS_WINDOWS
#endif  // CPU_FEATURES_ARCH_X86