// SPDX-FileCopyrightText: 2021 Google LLC
// SPDX-License-Identifier: Apache-2.0

#include <stdbool.h>
#include <stddef.h>

static bool equals(const char *lhs, const char *rhs, size_t count)
{
    for (size_t i = 0; i < count; ++i)
        if (lhs[i] != rhs[i]) return false;
    return true;
}
