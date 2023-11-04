// SPDX-FileCopyrightText: 2023 Google LLC
// SPDX-License-Identifier: Apache-2.0

#include "cpu_features_macros.h"

#ifdef CPU_FEATURES_OS_FREEBSD

#include "internal/hwcaps.h"

#ifdef CPU_FEATURES_TEST
// In test mode, hwcaps_for_testing will define the following functions.
HardwareCapabilities CpuFeatures_GetHardwareCapabilities(void);
const char* CpuFeatures_GetPlatformPointer(void);
const char* CpuFeatures_GetBasePlatformPointer(void);
#else

#ifdef HAVE_STRONG_ELF_AUX_INFO
#include <stddef.h>
#include <sys/auxv.h>

static unsigned long GetElfHwcapFromElfAuxInfo(int hwcap_type)
{
    unsigned long hwcap;
    elf_aux_info(hwcap_type, &hwcap, sizeof(hwcap));
    return hwcap;
}

HardwareCapabilities CpuFeatures_GetHardwareCapabilities(void)
{
    HardwareCapabilities capabilities;
    capabilities.hwcaps = GetElfHwcapFromElfAuxInfo(AT_HWCAP);
    capabilities.hwcaps2 = GetElfHwcapFromElfAuxInfo(AT_HWCAP2);
    return capabilities;
}

const char *CpuFeatures_GetPlatformPointer(void) { return NULL; }

const char *CpuFeatures_GetBasePlatformPointer(void) { return NULL; }

#else
#error "FreeBSD needs support for elf_aux_info"
#endif  // HAVE_STRONG_ELF_AUX_INFO

#endif  // CPU_FEATURES_TEST
#endif  // CPU_FEATURES_OS_FREEBSD