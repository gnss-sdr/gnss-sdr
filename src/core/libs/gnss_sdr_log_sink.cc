/*!
 * \file gnss_sdr_log_sink.cc
 * \brief Buffered, per-run file logging for Abseil.
 * \author Carles Fernandez-Prades, 2026 <carles.fernandez@cttc.es>
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "gnss_sdr_log_sink.h"
#include "gnss_sdr_filesystem.h"
#include <absl/base/log_severity.h>
#include <absl/log/log_entry.h>
#include <absl/log/log_sink_registry.h>
#include <absl/time/clock.h>
#include <absl/time/time.h>
#include <cerrno>
#include <fcntl.h>
#include <stdexcept>
#include <system_error>
#include <utility>
#if defined(_WIN32)
#include <io.h>
#include <process.h>
#include <sys/stat.h>
#else
#include <unistd.h>
#endif

GnssSdrLogSink::GnssSdrLogSink(const std::string& directory, const std::string& basename)
{
    if (basename.empty() || basename == "." || basename == ".." || basename.find_first_of("/\\:") != std::string::npos)
        {
            throw std::invalid_argument("Invalid log basename: " + basename);
        }
    const fs::path log_directory = fs::absolute(directory.empty() ? fs::temp_directory_path() : fs::path(directory));
    fs::create_directories(log_directory);
#if defined(_WIN32)
    const auto pid = _getpid();
#else
    const auto pid = getpid();
#endif
    const std::string stem = basename + "." + absl::FormatTime("%Y%m%d-%H%M%S", absl::Now(), absl::LocalTimeZone()) + "." + std::to_string(pid);
    for (unsigned int attempt = 0;; ++attempt)
        {
            const std::string suffix = attempt == 0 ? "" : "." + std::to_string(attempt);
            const fs::path path = log_directory / (stem + suffix + ".log");
            d_filename = path.string();
            // C++17 ofstream cannot create exclusively. Keep the descriptor open
            // through fdopen so there is no close/reopen race or truncation.
#if defined(_WIN32)
            const int fd = _wopen(path.c_str(), _O_WRONLY | _O_CREAT | _O_EXCL | _O_BINARY | _O_NOINHERIT, _S_IREAD | _S_IWRITE);
#else
            // Same default mode as glog's --logfile_mode; the process umask still applies.
            const int fd = open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0664);
#endif
            if (fd < 0)
                {
                    const int error = errno;
                    if (error == EEXIST)
                        {
                            continue;
                        }
                    throw std::system_error(error, std::generic_category(), "Cannot create logfile " + d_filename);
                }
#if defined(_WIN32)
            d_logfile.reset(_fdopen(fd, "wb"));
#else
            d_logfile.reset(fdopen(fd, "wb"));
#endif
            if (!d_logfile)
                {
                    const int error = errno;
#if defined(_WIN32)
                    _close(fd);
#else
                    close(fd);
#endif
                    throw std::system_error(error, std::generic_category(), "Cannot open logfile stream " + d_filename);
                }
            break;
        }
    try
        {
            UpdateLatestLink(basename);
        }
    catch (const std::exception& ex)
        {
            std::fprintf(stderr, "Cannot update latest logfile link for %s: %s\n", d_filename.c_str(), ex.what());
        }
}


GnssSdrLogSink::~GnssSdrLogSink()
{
    Flush();
    if (std::fclose(d_logfile.release()) != 0)
        {
            ReportWriteError();
        }
}


void GnssSdrLogSink::UpdateLatestLink(const std::string& basename) const
{
#if defined(_WIN32)
    // Creating symlinks can require privileges on Windows. The per-run file
    // remains available without a stable link.
    (void)basename;
#else
    const fs::path path(d_filename);
    const fs::path latest = path.parent_path() / (basename + ".log");
    // Preserve a regular logfile left by versions predating this sink. A hard
    // link keeps its contents available even when the stable name is replaced.
    if (fs::is_regular_file(fs::symlink_status(latest)))
        {
            fs::create_hard_link(latest, fs::path(d_filename + ".previous"));
        }
    fs::path temporary;
    for (unsigned int attempt = 0;; ++attempt)
        {
            temporary = fs::path(d_filename + ".link." + std::to_string(attempt));
            errorlib::error_code error;
            fs::create_symlink(path.filename(), temporary, error);
            if (!error)
                {
                    break;
                }
            if (error != errorlib::errc::file_exists)
                {
                    throw fs::filesystem_error("Cannot create latest logfile symlink", temporary, error);
                }
        }
    errorlib::error_code error;
    fs::rename(temporary, latest, error);
    if (error)
        {
            errorlib::error_code ignored;
            fs::remove(temporary, ignored);
            throw fs::filesystem_error("Cannot replace latest logfile symlink", latest, error);
        }
#endif
}


void GnssSdrLogSink::Send(const absl::LogEntry& entry) noexcept
{
    try
        {
            std::lock_guard<std::mutex> lock(d_mutex);
            const auto text = entry.text_message_with_prefix_and_newline();
            if (std::fwrite(text.data(), 1, text.size(), d_logfile.get()) != text.size())
                {
                    ReportWriteError();
                }
            if (entry.log_severity() >= absl::LogSeverity::kError && std::fflush(d_logfile.get()) != 0)
                {
                    ReportWriteError();
                }
        }
    catch (...)
        {
            ReportWriteError();
        }
}


void GnssSdrLogSink::Flush() noexcept
{
    try
        {
            std::lock_guard<std::mutex> lock(d_mutex);
            if (std::fflush(d_logfile.get()) != 0)
                {
                    ReportWriteError();
                }
        }
    catch (...)
        {
            ReportWriteError();
        }
}


void GnssSdrLogSink::ReportWriteError() noexcept
{
    if (!d_error_reported.test_and_set())
        {
            std::fprintf(stderr, "Failed to write or flush logfile %s\n", d_filename.c_str());
        }
}


GnssSdrLogSinkGuard::~GnssSdrLogSinkGuard()
{
    Shutdown();
}


void GnssSdrLogSinkGuard::Register(const std::string& directory, const std::string& basename)
{
    if (d_sink)
        {
            throw std::logic_error("Log sink is already registered");
        }
    auto sink = std::make_unique<GnssSdrLogSink>(directory, basename);
    absl::AddLogSink(sink.get());
    d_sink = std::move(sink);
}


void GnssSdrLogSinkGuard::Shutdown() noexcept
{
    if (d_sink)
        {
            // Stop callbacks before the destructor flushes and closes the file.
            absl::RemoveLogSink(d_sink.get());
            d_sink.reset();
        }
}
