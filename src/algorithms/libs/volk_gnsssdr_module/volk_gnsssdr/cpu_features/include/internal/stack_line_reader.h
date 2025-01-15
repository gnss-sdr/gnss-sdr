// SPDX-FileCopyrightText: 2017 Google LLC
// SPDX-License-Identifier: Apache-2.0

// Reads a file line by line and stores the data on the stack. This allows
// parsing files in one go without allocating.
#ifndef CPU_FEATURES_INCLUDE_INTERNAL_STACK_LINE_READER_H_
#define CPU_FEATURES_INCLUDE_INTERNAL_STACK_LINE_READER_H_

#include "cpu_features_macros.h"
#include "internal/string_view.h"
#include <stdbool.h>

CPU_FEATURES_START_CPP_NAMESPACE

typedef struct
{
    char buffer[STACK_LINE_READER_BUFFER_SIZE];
    StringView view;
    int fd;
    bool skip_mode;
} StackLineReader;

// Initializes a StackLineReader.
void StackLineReader_Initialize(StackLineReader* reader, int fd);

typedef struct
{
    StringView line;  // A view of the line.
    bool eof;         // Nothing more to read, we reached EOF.
    bool full_line;   // If false the line was truncated to
                      // STACK_LINE_READER_BUFFER_SIZE.
} LineResult;

// Reads the file pointed to by fd and tries to read a full line.
LineResult StackLineReader_NextLine(StackLineReader* reader);

CPU_FEATURES_END_CPP_NAMESPACE

#endif  // CPU_FEATURES_INCLUDE_INTERNAL_STACK_LINE_READER_H_
