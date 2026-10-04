/*!
 * \file gnss_sdr_log_sink.h
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


#ifndef GNSS_SDR_LOG_SINK_H
#define GNSS_SDR_LOG_SINK_H

#include <absl/log/log_sink.h>
#include <atomic>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * Utilities for the core GNSS receiver.
 * \{ */


/** A sink must be unregistered, and direct callers stopped, before destruction.
 * Directory/file creation errors throw; symlink and runtime I/O errors are
 * reported to stderr. An empty directory selects the system temporary directory.
 */
class GnssSdrLogSink : public absl::LogSink
{
public:
    explicit GnssSdrLogSink(const std::string& directory, const std::string& basename = "gnss-sdr");
    ~GnssSdrLogSink() override;
    GnssSdrLogSink(const GnssSdrLogSink&) = delete;
    GnssSdrLogSink& operator=(const GnssSdrLogSink&) = delete;
    GnssSdrLogSink(GnssSdrLogSink&&) = delete;
    GnssSdrLogSink& operator=(GnssSdrLogSink&&) = delete;

    void Send(const absl::LogEntry& entry) noexcept override;
    void Flush() noexcept override;
    const std::string& filename() const noexcept { return d_filename; }

private:
    struct FileCloser
    {
        void operator()(std::FILE* file) const noexcept { std::fclose(file); }
    };

    void UpdateLatestLink(const std::string& basename) const;
    void ReportWriteError() noexcept;

    std::string d_filename;
    std::unique_ptr<std::FILE, FileCloser> d_logfile;
    std::mutex d_mutex;
    std::atomic_flag d_error_reported = ATOMIC_FLAG_INIT;
};

/** Owns a registered sink. Initialize Abseil once before calling Register().
 * Register/Shutdown belong to the owning thread; Send/Flush may run concurrently.
 */
class GnssSdrLogSinkGuard
{
public:
    GnssSdrLogSinkGuard() = default;
    ~GnssSdrLogSinkGuard();
    GnssSdrLogSinkGuard(const GnssSdrLogSinkGuard&) = delete;
    GnssSdrLogSinkGuard& operator=(const GnssSdrLogSinkGuard&) = delete;
    GnssSdrLogSinkGuard(GnssSdrLogSinkGuard&&) = delete;
    GnssSdrLogSinkGuard& operator=(GnssSdrLogSinkGuard&&) = delete;

    void Register(const std::string& directory, const std::string& basename = "gnss-sdr");
    void Shutdown() noexcept;

private:
    std::unique_ptr<GnssSdrLogSink> d_sink;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_LOG_SINK_H
