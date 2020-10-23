// SPDX-FileCopyrightText: 2017 Google LLC
// SPDX-License-Identifier: Apache-2.0

// Implements a fake filesystem, useful for tests.
#ifndef CPU_FEATURES_TEST_FILESYSTEM_FOR_TESTING_H_
#define CPU_FEATURES_TEST_FILESYSTEM_FOR_TESTING_H_

#include "internal/filesystem.h"
#include <memory>
#include <string>
#include <unordered_map>

namespace cpu_features
{
class FakeFile
{
public:
    explicit FakeFile(int file_descriptor, const char* content);
    ~FakeFile();

    void Open();
    void Close();
    int Read(int fd, void* buf, size_t count);

    int GetFileDescriptor() const { return file_descriptor_; }

private:
    const int file_descriptor_;
    const std::string content_;
    bool opened_ = false;
    size_t head_index_ = 0;
};

class FakeFilesystem
{
public:
    void Reset();
    FakeFile* CreateFile(const std::string& filename, const char* content);
    FakeFile* FindFileOrDie(const int file_descriptor) const;
    FakeFile* FindFileOrNull(const std::string& filename) const;

private:
    int next_file_descriptor_ = 0;
    std::unordered_map<std::string, std::unique_ptr<FakeFile>> files_;
};

FakeFilesystem& GetEmptyFilesystem();

}  // namespace cpu_features

#endif  // CPU_FEATURES_TEST_FILESYSTEM_FOR_TESTING_H_
