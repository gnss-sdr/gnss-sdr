// SPDX-FileCopyrightText: 2017 Google LLC
// SPDX-License-Identifier: Apache-2.0

// An interface for the filesystem that allows mocking the filesystem in
// unittests.
#ifndef CPU_FEATURES_INCLUDE_INTERNAL_FILESYSTEM_H_
#define CPU_FEATURES_INCLUDE_INTERNAL_FILESYSTEM_H_

#include "cpu_features_macros.h"
#include <stddef.h>
#include <stdint.h>

CPU_FEATURES_START_CPP_NAMESPACE

// Same as linux "open(filename, O_RDONLY)", retries automatically on EINTR.
int CpuFeatures_OpenFile(const char* filename);

// Same as linux "read(file_descriptor, buffer, buffer_size)", retries
// automatically on EINTR.
int CpuFeatures_ReadFile(int file_descriptor, void* buffer, size_t buffer_size);

// Same as linux "close(file_descriptor)".
void CpuFeatures_CloseFile(int file_descriptor);

CPU_FEATURES_END_CPP_NAMESPACE

#endif  // CPU_FEATURES_INCLUDE_INTERNAL_FILESYSTEM_H_
