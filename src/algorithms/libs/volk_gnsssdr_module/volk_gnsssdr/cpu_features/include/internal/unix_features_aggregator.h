// SPDX-FileCopyrightText: 2017 Google Inc.
// SPDX-License-Identifier: Apache-2.0


// CapabilityConfig provides a way to map cpu features to hardware caps and
// /proc/cpuinfo flags. We then provide functions to update capabilities from
// either source.
#ifndef CPU_FEATURES_INCLUDE_INTERNAL_LINUX_FEATURES_AGGREGATOR_H_
#define CPU_FEATURES_INCLUDE_INTERNAL_LINUX_FEATURES_AGGREGATOR_H_

#include "cpu_features_macros.h"
#include "internal/hwcaps.h"
#include "internal/string_view.h"
#include <ctype.h>
#include <stdint.h>

CPU_FEATURES_START_CPP_NAMESPACE

// Use the following macro to declare setter functions to be used in
// CapabilityConfig.
#define DECLARE_SETTER(FeatureType, FeatureName)                    \
    static void set_##FeatureName(void* const features, bool value) \
    {                                                               \
        ((FeatureType*)features)->FeatureName = value;              \
    }

// Use the following macro to declare getter functions to be used in
// CapabilityConfig.
#define DECLARE_GETTER(FeatureType, FeatureName)       \
    static int get_##FeatureName(void* const features) \
    {                                                  \
        return ((FeatureType*)features)->FeatureName;  \
    }

#define DECLARE_SETTER_AND_GETTER(FeatureType, FeatureName) \
    DECLARE_SETTER(FeatureType, FeatureName)                \
    DECLARE_GETTER(FeatureType, FeatureName)

// Describes the relationship between hardware caps and /proc/cpuinfo flags.
typedef struct
{
    const HardwareCapabilities hwcaps_mask;
    const char* const proc_cpuinfo_flag;
    void (*set_bit)(void* const, bool);  // setter for the corresponding bit.
    int (*get_bit)(void* const);         // getter for the corresponding bit.
} CapabilityConfig;

// For every config, looks into flags_line for the presence of the
// corresponding proc_cpuinfo_flag, calls `set_bit` accordingly.
// Note: features is a pointer to the underlying Feature struct.
void CpuFeatures_SetFromFlags(const size_t configs_size,
    const CapabilityConfig* configs,
    const StringView flags_line,
    void* const features);

// For every config, looks into hwcaps for the presence of the feature. Calls
// `set_bit` with true if the hardware capability is found.
// Note: features is a pointer to the underlying Feature struct.
void CpuFeatures_OverrideFromHwCaps(const size_t configs_size,
    const CapabilityConfig* configs,
    const HardwareCapabilities hwcaps,
    void* const features);

CPU_FEATURES_END_CPP_NAMESPACE
#endif  // CPU_FEATURES_INCLUDE_INTERNAL_LINUX_FEATURES_AGGREGATOR_H_
