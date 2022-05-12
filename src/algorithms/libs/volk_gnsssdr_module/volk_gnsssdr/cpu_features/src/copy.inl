// SPDX-FileCopyrightText: 2021 Google LLC
// SPDX-License-Identifier: Apache-2.0

#include <stddef.h>

static void copy(char *__restrict dst, const char *src, size_t count)
{
    for (size_t i = 0; i < count; ++i) dst[i] = src[i];
}
