/*!
 * \file ntrip_rtcm_client.cc
 * \brief NTRIP client for RTCM3 fixed-base corrections
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * SPDX-FileCopyrightText: 2026 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "ntrip_rtcm_client.h"
#include "ntrip_rtk_signals.h"
#include "rtklib_rtcm.h"
#include "rtklib_rtkcmn.h"
#include "rtklib_stream.h"
#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <limits>
#include <mutex>
#include <netdb.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <utility>
#include <vector>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


namespace
{
constexpr std::size_t READ_BUFFER_SIZE = 8192;
constexpr int WORKER_POLL_INTERVAL_MS = 10;
constexpr int MAX_INTERVAL_MS = 24 * 60 * 60 * 1000;
// the interval floor rejects busy-loop configurations; zero keeps its
// dedicated meaning (reconnects disabled)
constexpr int MIN_INTERVAL_MS = 1000;


// identity of a broadcast ephemeris: issue numbers, reference times, data
// source, and health — a rebroadcast of the same data set matches all of them
bool same_broadcast_ephemeris(const eph_t& a, const eph_t& b)
{
    return a.sat == b.sat && a.iode == b.iode && a.iodc == b.iodc &&
           a.week == b.week && a.code == b.code && a.svh == b.svh &&
           timediff(a.toe, b.toe) == 0.0 && timediff(a.toc, b.toc) == 0.0;
}


bool valid_gtime(const gtime_t& time)
{
    return time.time != 0 && std::isfinite(time.sec);
}


bool valid_ecef_position(const std::array<double, 3>& position)
{
    return std::isfinite(position[0]) && std::isfinite(position[1]) &&
           std::isfinite(position[2]) &&
           std::sqrt(position[0] * position[0] + position[1] * position[1] +
                     position[2] * position[2]) > 1.0;
}


/* Band membership comes from RTKLIB's own code table (obsfreqs[] via
   code2obs() in rtklib_rtkcmn.cc), so a code added there - as this branch
   itself did for B1C's CODE_L1D - is accepted here without a parallel edit.
   The caller has already filtered by satellite system, which restricts the
   codes the RTCM decoder can produce to that system's signal set. */
bool code_in_band(unsigned char code, int band)
{
    int freq = 0;
    code2obs(code, &freq);
    return freq == band;
}


// Deliberately an explicit list, not a band lookup: this is the GNSS-SDR
// prefer-B1C rule for RTKLIB's shared first slot (B1I satellites are
// dropped), not band membership
bool supported_bds_b1c_code(unsigned char code)
{
    switch (code)
        {
        case CODE_L1D:
        case CODE_L1P:
        case CODE_L1X:
            return true;
        default:
            return false;
        }
}


void clear_observation_slot(obsd_t* observation, int slot)
{
    observation->SNR[slot] = 0;
    observation->LLI[slot] = 0;
    observation->code[slot] = CODE_NONE;
    observation->L[slot] = 0.0;
    observation->P[slot] = 0.0;
    observation->D[slot] = 0.0F;
}


int current_rtcm3_message_type(const rtcm_t& rtcm)
{
    if (rtcm.len < 2)
        {
            return 0;
        }
    return static_cast<int>(getbitu(rtcm.buff, 24, 12));
}


// One builder for the RTKLIB stream path, measured by validate() and used
// by the transport, so the two cannot drift on the path format
std::string ntrip_transport_path_string(const std::string& host, std::uint16_t port,
    const std::string& clean_mountpoint, int version, bool tls)
{
    std::ostringstream path;
    path << host << ':' << port << '/' << clean_mountpoint << "::NTRIP=" << version;
    if (tls)
        {
            path << "::TLS";
        }
    return path.str();
}


// non-printable-ASCII, or any of the explicitly rejected characters
bool has_non_graphic_or(const std::string& value, const char* rejected)
{
    return std::any_of(value.cbegin(), value.cend(), [rejected](char character) {
        const auto byte = static_cast<unsigned char>(character);
        return byte <= 0x20U || byte >= 0x7FU ||
               std::strchr(rejected, character) != nullptr;
    });
}


std::string normalized_mountpoint(const std::string& mountpoint)
{
    std::size_t first = 0;
    while (first < mountpoint.size() && mountpoint[first] == '/')
        {
            ++first;
        }
    return mountpoint.substr(first);
}


class Stream_Owner
{
public:
    Stream_Owner()
    {
        std::memset(&d_stream, 0, sizeof(d_stream));
        strinit(&d_stream);
    }

    ~Stream_Owner()
    {
        close();
        pthread_mutex_destroy(&d_stream.lock);
    }

    Stream_Owner(const Stream_Owner&) = delete;
    Stream_Owner& operator=(const Stream_Owner&) = delete;
    Stream_Owner(Stream_Owner&&) = delete;
    Stream_Owner& operator=(Stream_Owner&&) = delete;

    bool open(const std::string& path, const std::string& hostname,
        const std::string& username, const std::string& password)
    {
        if (stropen(&d_stream, STR_NTRIPCLI, STR_MODE_RW, path.c_str()) == 0)
            {
                return false;
            }
        auto* ntrip = static_cast<ntrip_t*>(d_stream.port);
        if (ntrip == nullptr)
            {
                close();
                return false;
            }
        // Credentials are installed directly instead of being embedded in the
        // stream path, which RTKLIB stores verbatim in the stream object.
        std::snprintf(ntrip->user, sizeof(ntrip->user), "%s", username.c_str());
        std::snprintf(ntrip->passwd, sizeof(ntrip->passwd), "%s", password.c_str());
        // The stream path carries a pre-resolved numeric address; restore the
        // caster host name for the NTRIP v2 Host header and TLS verification.
        if (ntripsethost(ntrip, hostname.c_str(), d_stream.msg) == 0)
            {
                close();
                return false;
            }
        return true;
    }

    void close()
    {
        if (d_stream.port != nullptr)
            {
                auto* ntrip = static_cast<ntrip_t*>(d_stream.port);
                secure_wipe(ntrip->user, sizeof(ntrip->user));
                secure_wipe(ntrip->passwd, sizeof(ntrip->passwd));
                // Keep unopened and disconnected transports explicitly invalid
                // before handing ownership back to RTKLIB's stream cleanup.
                if (ntrip->tcp != nullptr && ntrip->tcp->svr.state < 1)
                    {
                        ntrip->tcp->svr.sock = -1;
                    }
                strclose(&d_stream);
                d_stream.port = nullptr;
                d_stream.state = 0;
            }
    }

    stream_t* get()
    {
        return &d_stream;
    }

private:
    stream_t d_stream;
};


class Rtcm_Owner
{
public:
    // The analyzer loses the unique_ptr member's ownership while following
    // init_rtcm(), whose failure paths also release all partial allocations.
    // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
    Rtcm_Owner()
        : d_rtcm(new rtcm_t{})
    {
        // init_rtcm() populates pointers owned by the value-initialized
        // control structure, so it must run after d_rtcm is constructed.
        // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
        d_initialized = init_rtcm(d_rtcm.get()) != 0;
    }

    ~Rtcm_Owner()
    {
        if (d_initialized)
            {
                free_rtcm(d_rtcm.get());
            }
    }

    Rtcm_Owner(const Rtcm_Owner&) = delete;
    Rtcm_Owner& operator=(const Rtcm_Owner&) = delete;
    Rtcm_Owner(Rtcm_Owner&&) = delete;
    Rtcm_Owner& operator=(Rtcm_Owner&&) = delete;

    bool initialized() const
    {
        return d_initialized;
    }

    rtcm_t* get()
    {
        return d_rtcm.get();
    }

private:
    std::unique_ptr<rtcm_t> d_rtcm;
    bool d_initialized = false;
};


enum class Handshake_Result : std::uint8_t
{
    SUCCESS,
    STOPPED,
    CONNECT_FAILED,
    CONNECT_TIMEOUT,
    SEND_FAILED,
    RESPONSE_TIMEOUT,
    RESPONSE_REJECTED,
    RETRY_V1
};


enum class Resolve_Result : std::uint8_t
{
    SUCCESS,
    STOPPED,
    FAILED
};


struct Host_Resolution_State
{
    std::mutex mutex;
    std::condition_variable condition;
    std::string numeric_ipv4;
    bool complete = false;
    bool success = false;
};


using Hostname_Resolver =
    std::function<bool(const std::string&, std::string*)>;


bool resolve_ipv4_hostname(const std::string& hostname,
    std::string* numeric_ipv4) noexcept
{
    bool success = false;
    struct addrinfo* addresses = nullptr;
    try
        {
            struct addrinfo hints = {};
            hints.ai_family = AF_INET;
            hints.ai_socktype = SOCK_STREAM;
            if (getaddrinfo(hostname.c_str(), nullptr, &hints, &addresses) == 0)
                {
                    for (const struct addrinfo* address = addresses;
                        address != nullptr; address = address->ai_next)
                        {
                            if (address->ai_family != AF_INET || address->ai_addr == nullptr)
                                {
                                    continue;
                                }
                            const auto* ipv4_address = reinterpret_cast<const struct sockaddr_in*>(address->ai_addr);
                            char buffer[INET_ADDRSTRLEN] = {};
                            if (inet_ntop(AF_INET, &ipv4_address->sin_addr,
                                    buffer, sizeof(buffer)) != nullptr)
                                {
                                    *numeric_ipv4 = buffer;
                                    success = true;
                                    break;
                                }
                        }
                }
        }
    catch (...)
        {
            success = false;
            numeric_ipv4->clear();
        }
    if (addresses != nullptr)
        {
            freeaddrinfo(addresses);
        }
    return success;
}


void run_hostname_resolution(const std::shared_ptr<Host_Resolution_State>& state,
    const std::string& hostname, const Hostname_Resolver& resolver) noexcept
{
    std::string numeric_ipv4;
    bool success = false;
    try
        {
            success = resolver && resolver(hostname, &numeric_ipv4);
        }
    catch (...)
        {
            success = false;
            numeric_ipv4.clear();
        }

    try
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->numeric_ipv4.swap(numeric_ipv4);
            state->success = success;
            state->complete = true;
        }
    catch (...)
        {
            // The resolver owns no client state. If reporting fails under
            // memory pressure, the worker's timeout path remains safe.
        }
    state->condition.notify_all();
}

}  // namespace


void secure_clear_ntrip_credentials(Pvt_Conf* conf) noexcept
{
    if (conf == nullptr)
        {
            return;
        }
    conf->ntrip_username.clear();
    conf->ntrip_password.clear();
    conf->ntrip_password_env.clear();
}


void Ntrip_Rtcm_Snapshot::refresh_age(const gtime_t& rover_time, double max_age_s)
{
    if (!has_observations)
        {
            return;
        }
    const gtime_t age_reference =
        valid_gtime(rover_time) ? rover_time : utc2gpst(timeget());
    age_s = timediff(age_reference, observation_time);
    fresh = std::isfinite(age_s) && std::fabs(age_s) <= max_age_s;
}


bool ntrip_text_has_space_or_control(const std::string& text)
{
    return std::any_of(text.cbegin(), text.cend(), [](char character) {
        const auto byte = static_cast<unsigned char>(character);
        return std::isspace(byte) != 0 || std::iscntrl(byte) != 0;
    });
}


std::string Ntrip_Rtcm_Client_Config::validate() const
{
    const std::string clean_mountpoint = normalized_mountpoint(mountpoint);
    if (host.empty())
        {
            return "the caster address must be a non-empty hostname or IPv4 address";
        }
    if (has_non_graphic_or(host, ":/@[]"))
        {
            return "the caster address must be a hostname or IPv4 address without a scheme, credentials, port, or path";
        }
    if (port == 0)
        {
            return "the caster port must not be zero";
        }
    if (clean_mountpoint.empty() || clean_mountpoint.size() >= 256)
        {
            return "the mountpoint must name one caster mountpoint";
        }
    if (has_non_graphic_or(clean_mountpoint, "@:/"))
        {
            return "the mountpoint must name one caster mountpoint without a path separator";
        }
    const std::string& username_value = username.value();
    const std::string& password_value = password.value();
    if (username_value.size() >= 256 || password_value.size() >= 256 ||
        username_value.find(':') != std::string::npos ||
        ntrip_text_has_space_or_control(username_value) ||
        ntrip_text_has_space_or_control(password_value))
        {
            return "the username or the password contains characters unsupported by the NTRIP client";
        }
    if (!password_value.empty() && username_value.empty())
        {
            return "a username is required when a password is configured";
        }
    if (timeout_ms < MIN_INTERVAL_MS || timeout_ms > MAX_INTERVAL_MS ||
        (reconnect_interval_ms != 0 && reconnect_interval_ms < MIN_INTERVAL_MS) ||
        reconnect_interval_ms > MAX_INTERVAL_MS)
        {
            return "timeout and reconnect intervals must be within 1000 ms and 24 hours; reconnect may also be zero";
        }
    if (send_gga && (gga_period_ms < MIN_INTERVAL_MS || gga_period_ms > MAX_INTERVAL_MS))
        {
            return "the GGA period must be within 1000 ms and 24 hours";
        }
    if (!std::isfinite(max_age_s) || max_age_s <= 0.0)
        {
            return "the maximum correction age must be a finite positive value";
        }
    if (station_id < 0 || station_id > 4095)
        {
            return "the station ID must be in the range 0..4095";
        }
    if (version != 1 && version != 2)
        {
            return "the NTRIP version must be 1 or 2";
        }
    // measure the real worst-case transport path instead of modeling its
    // format by hand: the widest host this configuration can produce is
    // either the configured name or a resolved IPv4 literal (15 characters)
    const std::string worst_case_host =
        host.size() >= 15 ? host : std::string(15, 'n');
    if (ntrip_transport_path_string(worst_case_host, 65535, clean_mountpoint,
            version, true)
            .size() >= MAXSTRPATH)
        {
            return "the NTRIP endpoint is too long";
        }
    return "";
}


Ntrip_Rtcm_Client_Config make_ntrip_rtcm_client_config(const Pvt_Conf& conf)
{
    Ntrip_Rtcm_Client_Config config;
    config.enabled = conf.ntrip_client_enabled;
    config.host = conf.ntrip_caster_address;
    config.port = conf.ntrip_port;
    config.mountpoint = normalized_mountpoint(conf.ntrip_mountpoint);
    config.username = conf.ntrip_username;
    config.password = conf.ntrip_password;
    config.reconnect_interval_ms = conf.ntrip_reconnect_interval_ms;
    config.timeout_ms = conf.ntrip_timeout_ms;
    config.max_age_s = conf.ntrip_max_correction_age_s;
    config.send_gga = conf.ntrip_send_gga;
    config.gga_period_ms = conf.ntrip_gga_period_ms;
    config.version = conf.ntrip_version;
    config.tls_enabled = conf.ntrip_tls_enabled;
    config.station_id = conf.ntrip_station_id;
    return config;
}


// The layout follows mutex-protected state groups. Saving a few padding bytes
// in this single-instance class is not worth separating each lock from its data.
// NOLINTNEXTLINE(clang-analyzer-optin.performance.Padding)
class Ntrip_Rtcm_Client::Impl
{
public:
    explicit Impl(const Ntrip_Rtcm_Client_Config& config)
        : d_config(config),
          d_hostname_resolver(resolve_ipv4_hostname)
    {
        d_status.state = d_config.enabled ? Ntrip_Rtcm_Client_State::STOPPED : Ntrip_Rtcm_Client_State::DISABLED;
        d_status.message = d_config.enabled ? "stopped" : "disabled";
    }

    ~Impl() noexcept
    {
        stop();
    }

    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(Impl&&) = delete;

    bool start()
    {
        std::lock_guard<std::mutex> lifecycle_lock(d_lifecycle_mutex);
        if (d_running.load())
            {
                return true;
            }
        if (d_worker.joinable())
            {
                d_worker.join();
            }
        if (!d_config.enabled)
            {
                set_state(Ntrip_Rtcm_Client_State::DISABLED, "disabled", false);
                return false;
            }

        std::string validation_error;
        if (!validate_config(&validation_error))
            {
                set_state(Ntrip_Rtcm_Client_State::ERROR,
                    std::string("invalid configuration: ") + validation_error, false);
                return false;
            }

        LOG(INFO) << "NTRIP client configured for v" << d_config.version
                  << (d_config.tls_enabled ? " (TLS 1.2+) at " : " at ")
                  << d_config.host << ':' << d_config.port
                  << '/' << normalized_mountpoint(d_config.mountpoint);
        reset_output_data();
        reset_status();
        d_stop_requested.store(false);
        d_running.store(true);
        set_state(Ntrip_Rtcm_Client_State::STARTING, "starting", false);
        try
            {
                d_worker = std::thread(&Impl::worker_entry, this);
            }
        catch (...)
            {
                d_running.store(false);
                set_state(Ntrip_Rtcm_Client_State::ERROR,
                    "could not create NTRIP worker", false);
                return false;
            }
        return true;
    }


    void stop() noexcept
    {
        // Signal first so shutdown does not depend on diagnostic allocation or
        // on acquiring the lifecycle mutex while start() is finishing.
        d_stop_requested.store(true);
        d_wait_condition.notify_all();
        try
            {
                if (d_running.load())
                    {
                        set_state(Ntrip_Rtcm_Client_State::STOPPING, "stopping", false);
                    }
            }
        catch (...)
            {
                // Status is best-effort during noexcept teardown.
            }

        try
            {
                std::lock_guard<std::mutex> lifecycle_lock(d_lifecycle_mutex);
                // start() may have reset this flag while stop() waited for the
                // lifecycle lock, so assert it again immediately before join.
                d_stop_requested.store(true);
                d_wait_condition.notify_all();
                if (d_worker.joinable())
                    {
                        d_worker.join();
                    }
                d_running.store(false);
                try
                    {
                        set_state(d_config.enabled ? Ntrip_Rtcm_Client_State::STOPPED : Ntrip_Rtcm_Client_State::DISABLED,
                            d_config.enabled ? "stopped" : "disabled", false);
                    }
                catch (...)
                    {
                        // Status is best-effort during noexcept teardown.
                    }
            }
        catch (...)
            {
                // The worker target owns a pointer to this Impl, so detaching
                // after an unexpected join failure would permit a use-after-
                // free. This ownership model must fail closed instead.
                std::terminate();
            }
    }

    bool running() const noexcept
    {
        return d_running.load();
    }

    bool set_hostname_resolver_for_test(Hostname_Resolver resolver)
    {
        std::lock_guard<std::mutex> lifecycle_lock(d_lifecycle_mutex);
        if (d_running.load() || !resolver)
            {
                return false;
            }
        d_hostname_resolver = std::move(resolver);
        return true;
    }

    Ntrip_Rtcm_Client_Status status() const
    {
        std::lock_guard<std::mutex> lock(d_status_mutex);
        Ntrip_Rtcm_Client_Status result = d_status;
        result.running = d_running.load();
        return result;
    }

    std::uint64_t state_generation() const noexcept
    {
        return d_state_generation.load(std::memory_order_relaxed);
    }

    std::uint64_t snapshot_generation() const noexcept
    {
        return d_output_generation.load(std::memory_order_acquire);
    }

    Ntrip_Rtcm_Snapshot latest_snapshot() const
    {
        return latest_snapshot(utc2gpst(timeget()));
    }

    Ntrip_Rtcm_Snapshot latest_snapshot(const gtime_t& reference_time) const
    {
        Ntrip_Rtcm_Snapshot result;
        {
            std::lock_guard<std::mutex> lock(d_output_mutex);
            result.has_observations = d_has_observations;
            result.observations = d_observations;
            result.observation_time = d_observation_time;
            result.generation = d_output_generation.load(std::memory_order_relaxed);
            result.has_base_position = d_has_base_position;
            result.base_position_ecef_m = d_base_position_ecef_m;
            result.base_position_message_type = d_base_position_message_type;
            result.station_id = d_has_observations ? d_observation_station_id : d_base_position_station_id;
            result.itrf = d_base_position_itrf;
            result.antenna_height_m = d_antenna_height_m;
            result.ephemerides = d_ephemerides;
        }

        result.refresh_age(reference_time, d_config.max_age_s);
        return result;
    }

    void update_rover_position(const std::array<double, 3>& position_ecef_m)
    {
        {
            std::lock_guard<std::mutex> lock(d_input_mutex);
            d_rover_position_ecef_m = position_ecef_m;
            d_has_rover_position = valid_ecef_position(position_ecef_m);
        }
        d_wait_condition.notify_all();
    }

    void update_rover_position(const std::array<double, 3>& position_ecef_m,
        const gtime_t& rover_time)
    {
        update_rover_position(position_ecef_m);
        update_rover_time(rover_time);
    }

    void update_rover_time(const gtime_t& rover_time)
    {
        if (!valid_gtime(rover_time))
            {
                return;
            }
        {
            std::lock_guard<std::mutex> lock(d_input_mutex);
            d_rover_time = rover_time;
            d_has_rover_time = true;
            ++d_rover_time_generation;
        }
        d_wait_condition.notify_all();
    }

private:
    bool validate_config(std::string* reason) const
    {
        const std::string error = d_config.validate();
        if (!error.empty())
            {
                *reason = error;
                return false;
            }
        return true;
    }

    std::string transport_path() const
    {
        return transport_path(d_config.host);
    }

    std::string transport_path(const std::string& host) const
    {
        return transport_path(host, d_config.version);
    }

    std::string transport_path(const std::string& host, int version) const
    {
        return ntrip_transport_path_string(host, d_config.port,
            normalized_mountpoint(d_config.mountpoint), version,
            d_config.tls_enabled);
    }

    Resolve_Result resolve_transport_host(
        std::shared_ptr<Host_Resolution_State>* pending_resolution,
        std::string* resolved_host,
        std::string* failure_message)
    {
        struct in_addr numeric_address = {};
        if (inet_pton(AF_INET, d_config.host.c_str(), &numeric_address) == 1)
            {
                *resolved_host = d_config.host;
                return Resolve_Result::SUCCESS;
            }

        set_state(Ntrip_Rtcm_Client_State::CONNECTING,
            "resolving NTRIP caster hostname", false);
        if (!*pending_resolution)
            {
                pending_resolution->reset(new Host_Resolution_State);
                try
                    {
                        std::unique_ptr<std::thread> resolver(new std::thread(
                            run_hostname_resolution, *pending_resolution,
                            d_config.host, d_hostname_resolver));
                        try
                            {
                                resolver->detach();
                            }
                        catch (...)
                            {
                                // A join here could reintroduce an unbounded
                                // shutdown wait. Keep the exceptional joinable
                                // handle alive; its task owns no client state
                                // and still reports through the shared result.
                                (void)resolver.release();  // NOLINT(bugprone-unused-return-value)
                            }
                    }
                catch (...)
                    {
                        pending_resolution->reset();
                        *failure_message = "could not start hostname resolver";
                        return Resolve_Result::FAILED;
                    }
            }

        const std::chrono::steady_clock::time_point deadline =
            std::chrono::steady_clock::now() +
            std::chrono::milliseconds(d_config.timeout_ms);
        while (!d_stop_requested.load() &&
               std::chrono::steady_clock::now() < deadline)
            {
                // Stop requests arrive through d_wait_condition, never through
                // this condition variable, so the wait stays bounded to one
                // poll interval instead of blocking on the resolver
                Host_Resolution_State& resolution_state = **pending_resolution;
                std::unique_lock<std::mutex> lock(resolution_state.mutex);
                if (resolution_state.condition.wait_for(lock,
                        std::chrono::milliseconds(WORKER_POLL_INTERVAL_MS),
                        [&resolution_state] { return resolution_state.complete; }))
                    {
                        break;
                    }
            }
        if (d_stop_requested.load())
            {
                return Resolve_Result::STOPPED;
            }

        const std::shared_ptr<Host_Resolution_State> resolution = *pending_resolution;
        bool complete = false;
        bool success = false;
        std::string numeric_ipv4;
        {
            std::lock_guard<std::mutex> lock(resolution->mutex);
            complete = resolution->complete;
            success = resolution->success;
            numeric_ipv4 = resolution->numeric_ipv4;
        }
        if (!complete)
            {
                // Keep one lookup in flight across reconnect cycles. Portable
                // getaddrinfo() has no safe cancellation mechanism, so replacing
                // a stalled detached resolver on every timeout would accumulate
                // threads without bound. If it eventually completes, a later
                // cycle consumes its result and clears the shared state below.
                *failure_message = "NTRIP caster hostname resolution timed out";
                return Resolve_Result::FAILED;
            }
        pending_resolution->reset();
        if (!success)
            {
                *failure_message = "NTRIP caster hostname could not be resolved";
                return Resolve_Result::FAILED;
            }
        if (numeric_ipv4 != d_logged_resolved_ipv4)
            {
                LOG(INFO) << "NTRIP client: caster hostname " << d_config.host
                          << " resolved to " << numeric_ipv4;
                d_logged_resolved_ipv4 = numeric_ipv4;
            }
        *resolved_host = std::move(numeric_ipv4);
        return Resolve_Result::SUCCESS;
    }

    std::string negotiating_message(int version) const
    {
        std::string message = "negotiating NTRIP v";
        message += (version == 1 ? '1' : '2');
        message += " stream";
        if (d_config.tls_enabled)
            {
                message += " over TLS";
            }
        return message;
    }

    static std::string streaming_message(int version, bool fallback_from_v2)
    {
        std::string message = "receiving RTCM3 corrections over NTRIP v";
        message += (version == NTRIP_VERSION_1 ? '1' : '2');
        if (fallback_from_v2)
            {
                message += " (fallback from v2)";
            }
        return message;
    }

    void worker_entry() noexcept
    {
        try
            {
                worker_loop();
            }
        catch (...)
            {
                try
                    {
                        set_state(Ntrip_Rtcm_Client_State::ERROR,
                            "NTRIP worker failed", false);
                    }
                catch (...)
                    {
                        // Preserve noexcept even if allocating the diagnostic
                        // status string fails under memory pressure.
                        d_running.store(false);
                    }
            }
        log_session_summary();
        d_running.store(false);
    }

    void log_session_summary() noexcept
    {
        try
            {
                Ntrip_Rtcm_Client_Status counters;
                {
                    std::lock_guard<std::mutex> lock(d_status_mutex);
                    counters = d_status;
                }
                LOG(INFO) << "NTRIP client: session summary: "
                          << counters.bytes_received << " bytes received, "
                          << counters.rtcm_messages_decoded << " RTCM3 messages ("
                          << counters.observation_epochs_decoded << " observation epochs, "
                          << counters.decoder_errors << " decoder errors), "
                          << counters.reconnect_count << " reconnects";
            }
        catch (...)
            {
                // the summary is best-effort diagnostics
            }
    }

    void worker_loop()
    {
        std::uint64_t applied_rover_time_generation = 0;
        int active_version = d_config.version;
        bool fallback_from_v2 = false;
        bool v1_fallback_confirmed = false;
        std::string immediate_retry_host;
        // one decoder for the whole worker lifetime: its lock-time table is
        // the baseline lossoflock() uses to flag base-receiver cycle slips,
        // and a per-connection decoder would zero that baseline on every
        // reconnect — a base slip during a brief outage would then arrive
        // with no loss-of-lock indication and stale carrier ambiguities
        // would survive it (rtksvr keeps one rtcm_t across stream reconnects
        // for the same reason)
        Rtcm_Owner decoder;
        if (!decoder.initialized())
            {
                set_state(Ntrip_Rtcm_Client_State::ERROR,
                    "could not initialize RTCM3 decoder", false);
                return;
            }
        if (d_config.station_id > 0)
            {
                std::snprintf(decoder.get()->opt,
                    sizeof(decoder.get()->opt), "-STA=%d",
                    d_config.station_id);
            }
        while (!d_stop_requested.load())
            {
                std::string failure_message = "correction stream disconnected";
                bool retry_with_v1 = false;
                std::string resolved_host;
                Resolve_Result resolve_result = Resolve_Result::SUCCESS;
                if (!immediate_retry_host.empty())
                    {
                        resolved_host.swap(immediate_retry_host);
                    }
                else
                    {
                        resolve_result = resolve_transport_host(
                            &d_pending_resolution, &resolved_host, &failure_message);
                    }
                if (resolve_result == Resolve_Result::STOPPED)
                    {
                        return;
                    }
                if (resolve_result == Resolve_Result::SUCCESS)
                    {
                        // a new connection starts at an arbitrary point of the
                        // caster's byte stream: drop any partially assembled
                        // frame left over from the previous connection, but
                        // keep the per-satellite lock-time baseline
                        decoder.get()->nbyte = 0;
                        decoder.get()->len = 0;
                        d_logged_first_observations = false;
                        d_logged_decoder_error = false;
                        Stream_Owner stream;
                        set_state(Ntrip_Rtcm_Client_State::CONNECTING,
                            "connecting to NTRIP caster", false);
                        if (!stream.open(transport_path(resolved_host, active_version), d_config.host,
                                d_config.username.value(), d_config.password.value()))
                            {
                                failure_message = "could not initialize NTRIP transport";
                            }
                        else
                            {
                                strsettimeout(stream.get(), d_config.timeout_ms,
                                    d_config.reconnect_interval_ms);
                                const Handshake_Result handshake =
                                    perform_handshake(stream.get(), active_version);
                                if (handshake == Handshake_Result::SUCCESS)
                                    {
                                        const auto* ntrip = static_cast<const ntrip_t*>(stream.get()->port);
                                        if (active_version == NTRIP_VERSION_2 && ntrip != nullptr &&
                                            ntrip->version == NTRIP_VERSION_1)
                                            {
                                                active_version = NTRIP_VERSION_1;
                                                fallback_from_v2 = true;
                                                LOG(INFO) << "NTRIP client: caster answered in v1; "
                                                             "continuing this connection with NTRIP v1";
                                            }
                                        if (active_version == NTRIP_VERSION_1 && fallback_from_v2)
                                            {
                                                v1_fallback_confirmed = true;
                                            }
                                        apply_decoder_time(decoder.get(), &applied_rover_time_generation, true);
                                        set_state(Ntrip_Rtcm_Client_State::STREAMING,
                                            streaming_message(active_version, fallback_from_v2), true);
                                        failure_message = stream_corrections(stream.get(), decoder.get(),
                                            &applied_rover_time_generation);
                                    }
                                else if (handshake == Handshake_Result::STOPPED)
                                    {
                                        return;
                                    }
                                else
                                    {
                                        failure_message = handshake_failure_message(handshake);
                                        if (stream.get()->msg[0] != '\0')
                                            {
                                                failure_message += " (";
                                                failure_message += stream.get()->msg;
                                                failure_message += ')';
                                            }
                                        retry_with_v1 =
                                            active_version == NTRIP_VERSION_2 &&
                                            handshake == Handshake_Result::RETRY_V1;
                                    }
                            }
                    }

                if (d_stop_requested.load())
                    {
                        return;
                    }
                if (!retry_with_v1 && fallback_from_v2 &&
                    active_version == NTRIP_VERSION_1)
                    {
                        failure_message.insert(0, "NTRIP v1 fallback: ");
                    }
                if (retry_with_v1)
                    {
                        active_version = NTRIP_VERSION_1;
                        fallback_from_v2 = true;
                        immediate_retry_host = std::move(resolved_host);
                        set_state(Ntrip_Rtcm_Client_State::CONNECTING,
                            "NTRIP v2 negotiation failed; retrying with v1", false);
                        continue;
                    }
                if (fallback_from_v2 && !v1_fallback_confirmed &&
                    active_version == NTRIP_VERSION_1)
                    {
                        // the v1 fallback was inferred from a failed v2 handshake
                        // (which a transient drop — e.g. a caster restart closing
                        // the connection before any response byte — also produces)
                        // and never carried a stream: start the next cycle from
                        // the configured version instead of locking the whole
                        // session to v1 on that one-shot evidence
                        active_version = d_config.version;
                        fallback_from_v2 = false;
                        LOG(INFO) << "NTRIP client: unconfirmed v1 fallback discarded; "
                                     "the next attempt uses the configured NTRIP v"
                                  << d_config.version;
                    }
                if (d_config.reconnect_interval_ms == 0)
                    {
                        set_state(Ntrip_Rtcm_Client_State::ERROR,
                            failure_message + "; automatic reconnect is disabled", false);
                        return;
                    }
                increment_reconnect_count();
                set_state(Ntrip_Rtcm_Client_State::RECONNECT_WAIT,
                    failure_message, false);
                if (wait_interruptible(d_config.reconnect_interval_ms))
                    {
                        return;
                    }
            }
    }

    /* Evidence that the caster cannot speak NTRIP v2 on this connection and
       a fresh-connection v1 retry is warranted: it explicitly asked for v1,
       answered in v1, or dropped the request without any response bytes.
       Single home of the heuristic, shared by the connection-closed and
       timeout paths of perform_handshake() - a refinement applied to only
       one of them would make the two downgrade under different rules. */
    static bool v1_downgrade_indicated(const ntrip_t* ntrip, int requested_version)
    {
        return requested_version == NTRIP_VERSION_2 &&
               (ntrip->v1_retry_requested != 0 ||
                   ntrip->version == NTRIP_VERSION_1 ||
                   ntrip->response_received == 0);
    }

    Handshake_Result perform_handshake(stream_t* stream, int requested_version)
    {
        auto* ntrip = static_cast<ntrip_t*>(stream->port);
        if (ntrip == nullptr || ntrip->tcp == nullptr)
            {
                return Handshake_Result::CONNECT_FAILED;
            }

        const std::chrono::steady_clock::time_point deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(d_config.timeout_ms);
        bool connected = false;
        while (!d_stop_requested.load() && std::chrono::steady_clock::now() < deadline)
            {
                // waitntrip() advances the whole negotiation: TCP connection,
                // optional TLS handshake, NTRIP request, and response parsing
                // (v2 with same-connection v1 response compatibility, or
                // forced v1). Fresh-connection fallback is owned by the worker.
                const int negotiated = waitntrip(ntrip, stream->msg);
                if (!connected && ntrip->tcp->svr.state == 2)
                    {
                        connected = true;
                        set_state(Ntrip_Rtcm_Client_State::AUTHENTICATING,
                            negotiating_message(ntrip->version), false);
                    }
                if (negotiated != 0 && ntrip->state == 2)
                    {
                        return Handshake_Result::SUCCESS;
                    }
                if (ntrip->tcp->svr.state == 0)
                    {
                        // A complete request is sufficient evidence even when
                        // a fast caster accepts and closes within this single
                        // waitntrip() call, before connected can be latched.
                        if (ntrip->request_sent != 0)
                            {
                                if (v1_downgrade_indicated(ntrip, requested_version))
                                    {
                                        return Handshake_Result::RETRY_V1;
                                    }
                                return Handshake_Result::RESPONSE_REJECTED;
                            }
                        // TCP, TLS, and partial-request failures are never
                        // interpreted as protocol downgrade evidence.
                        return Handshake_Result::CONNECT_FAILED;
                    }
                if (ntrip->tcp->svr.state < 0)
                    {
                        return Handshake_Result::CONNECT_FAILED;
                    }
                if (wait_interruptible(WORKER_POLL_INTERVAL_MS))
                    {
                        return Handshake_Result::STOPPED;
                    }
            }
        if (d_stop_requested.load())
            {
                return Handshake_Result::STOPPED;
            }
        if (!connected)
            {
                return Handshake_Result::CONNECT_TIMEOUT;
            }
        if (ntrip->request_sent == 0)
            {
                return Handshake_Result::SEND_FAILED;
            }
        if (v1_downgrade_indicated(ntrip, requested_version))
            {
                return Handshake_Result::RETRY_V1;
            }
        return Handshake_Result::RESPONSE_TIMEOUT;
    }

    std::string stream_corrections(stream_t* stream, rtcm_t* decoder,
        std::uint64_t* applied_rover_time_generation)
    {
        auto* ntrip = static_cast<ntrip_t*>(stream->port);
        std::array<unsigned char, READ_BUFFER_SIZE> buffer = {{0}};
        bool sent_gga = false;
        bool warned_gga_suppressed = false;
        std::chrono::steady_clock::time_point last_gga;
        std::chrono::steady_clock::time_point last_data = std::chrono::steady_clock::now();

        while (!d_stop_requested.load())
            {
                apply_decoder_time(decoder, applied_rover_time_generation, false);
                const int count = strread(stream, buffer.data(), static_cast<int>(buffer.size()));
                const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
                if (count > 0)
                    {
                        last_data = now;
                        add_received_bytes(static_cast<std::uint64_t>(count));
                        decode_bytes(decoder, buffer.data(), count);
                    }
                else if (d_config.timeout_ms > 0 &&
                         std::chrono::duration_cast<std::chrono::milliseconds>(now - last_data).count() >
                             d_config.timeout_ms)
                    {
                        // The TCP-level inactivity watchdog counts writes too, so
                        // the periodic GGA upload keeps feeding it while a hung
                        // caster ACKs and sends nothing: enforce the timeout on
                        // received-data age here so the stream reconnects
                        return "correction stream inactive";
                    }

                if (ntrip == nullptr || ntrip->tcp == nullptr ||
                    ntrip->tcp->svr.state != 2 || ntrip->state != 2)
                    {
                        return "correction stream disconnected";
                    }

                if (d_config.send_gga)
                    {
                        if (!sent_gga ||
                            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_gga).count() >=
                                d_config.gga_period_ms)
                            {
                                std::array<double, 3> rover_position = {{0.0, 0.0, 0.0}};
                                bool has_rover_position = false;
                                {
                                    std::lock_guard<std::mutex> lock(d_input_mutex);
                                    rover_position = d_rover_position_ecef_m;
                                    has_rover_position = d_has_rover_position;
                                }
                                if (has_rover_position)
                                    {
                                        strsendnmea(stream, rover_position.data());
                                        last_gga = now;
                                        if (!sent_gga)
                                            {
                                                LOG(INFO) << "NTRIP client: NMEA GGA rover position "
                                                             "uploaded to the caster (period "
                                                          << d_config.gga_period_ms << " ms)";
                                            }
                                        sent_gga = true;
                                    }
                                else if (!sent_gga && !warned_gga_suppressed)
                                    {
                                        LOG(INFO) << "NTRIP client: GGA upload is enabled but no "
                                                     "valid rover position is available yet; the "
                                                     "caster receives no position until the first fix";
                                        warned_gga_suppressed = true;
                                    }
                            }
                    }

                if (count == 0 && wait_interruptible(WORKER_POLL_INTERVAL_MS))
                    {
                        return "stopped";
                    }
            }
        return "stopped";
    }

    void decode_bytes(rtcm_t* decoder, const unsigned char* data, int count)
    {
        for (int i = 0; i < count; ++i)
            {
                const int result = input_rtcm3(decoder, data[i]);
                if (result < 0)
                    {
                        increment_decoder_errors();
                        if (!d_logged_decoder_error)
                            {
                                LOG(WARNING) << "NTRIP client: RTCM3 decoder error (corrupted "
                                                "frame or unsupported message); further errors "
                                                "on this connection are only counted";
                                d_logged_decoder_error = true;
                            }
                        continue;
                    }
                if (result == 0)
                    {
                        continue;
                    }

                increment_decoded_messages();
                const int message_type = current_rtcm3_message_type(*decoder);
                if (result == 1)
                    {
                        // The epoch-ready trigger may be a non-GPS message: casters
                        // interleaving constellations close the epoch on the last
                        // one sent (e.g. GLONASS 1012 with synchronous flag 0).
                        // store_observations() filters the epoch per satellite.
                        store_observations(*decoder);
                    }
                else if (result == 2)
                    {
                        store_ephemeris(*decoder);
                    }
                else if (result == 5 && (message_type == 1005 || message_type == 1006))
                    {
                        store_base_position(*decoder, message_type);
                    }
            }
    }

    void store_observations(const rtcm_t& decoder)
    {
        std::vector<obsd_t> filtered;
        filtered.reserve(static_cast<std::size_t>(decoder.obs.n));
        for (int i = 0; i < decoder.obs.n; ++i)
            {
                const int system = satsys(decoder.obs.data[i].sat, nullptr);
                if (system != SYS_GPS && system != SYS_GAL && system != SYS_BDS)
                    {
                        continue;
                    }
                obsd_t observation = decoder.obs.data[i];
                // Each satellite must provide at least its first band (GPS L1,
                // Galileo E1, or BeiDou B1C). Second-band observations - GPS
                // L2 in slot 1 or L5 in slot 2, Galileo E5a in slot 2 (E5a and
                // L5 share RTKLIB's third frequency index) - are kept when
                // present; the solver pairs whichever bands the receiver is
                // configured for, and single-frequency receivers need only the
                // first band. BeiDou is B1C-only for now: the decoder prefers
                // B1C over B1I in the shared first slot, and B1I-only
                // satellites are dropped.
                bool has_first_band = false;
                for (int slot = 0; slot < NFREQ + NEXOBS; ++slot)
                    {
                        const bool has_measurement =
                            (std::isfinite(observation.P[slot]) && observation.P[slot] != 0.0) ||
                            (std::isfinite(observation.L[slot]) && observation.L[slot] != 0.0);
                        // a slot survives when the signal table lists a signal
                        // of this system in it and the observation code belongs
                        // to the slot's band; the BeiDou first slot applies the
                        // explicit prefer-B1C rule instead of band membership,
                        // dropping B1I satellites
                        bool keep = false;
                        bool is_first_band = false;
                        if (has_measurement)
                            {
                                for (const auto& signal : ntrip_rtk_signals())
                                    {
                                        if (signal.system != system || signal.band_slot != slot)
                                            {
                                                continue;
                                            }
                                        const bool code_matches =
                                            (system == SYS_BDS && slot == 0)
                                                ? supported_bds_b1c_code(observation.code[slot])
                                                : code_in_band(observation.code[slot], slot + 1);
                                        keep = code_matches;
                                        is_first_band = ntrip_rtk_is_entry_band(signal);
                                        break;  // one table row per (system, slot)
                                    }
                            }
                        if (!keep)
                            {
                                clear_observation_slot(&observation, slot);
                            }
                        else
                            {
                                has_first_band = has_first_band || is_first_band;
                            }
                    }
                if (has_first_band)
                    {
                        observation.rcv = 2;
                        filtered.push_back(observation);
                    }
            }
        if (filtered.empty())
            {
                return;
            }

        const std::size_t satellite_count = filtered.size();
        bool invalidated_base_position = false;
        int invalidated_base_station_id = 0;
        {
            std::lock_guard<std::mutex> lock(d_output_mutex);
            if (d_has_base_position && d_base_position_station_id != decoder.staid)
                {
                    invalidated_base_position = true;
                    invalidated_base_station_id = d_base_position_station_id;
                    invalidate_base_position_locked();
                }
            d_observations.swap(filtered);
            d_observation_time = d_observations.front().time;
            d_observation_station_id = decoder.staid;
            d_has_observations = true;
            ++d_output_generation;
        }
        increment_observation_epochs();
        if (invalidated_base_position)
            {
                LOG(WARNING) << "NTRIP client: observations arrived from station "
                             << decoder.staid << " while the cached base position "
                                                 "belongs to station "
                             << invalidated_base_station_id
                             << "; base position dropped";
            }
        if (!d_logged_first_observations)
            {
                LOG(INFO) << "NTRIP client: receiving base observations from station "
                          << decoder.staid << " (" << satellite_count << " satellites)";
                d_logged_first_observations = true;
            }
    }

    void store_ephemeris(const rtcm_t& decoder)
    {
        const int system = decoder.ephsat > 0 ? satsys(decoder.ephsat, nullptr) : 0;
        // keep the stored (and snapshot-copied) ephemerides aligned with the
        // systems store_observations admits: an ephemeris for a system whose
        // base observations are dropped could never pair with anything
        if ((system != SYS_GPS && system != SYS_GAL && system != SYS_BDS) ||
            decoder.nav.eph == nullptr || decoder.ephsat > decoder.nav.n)
            {
                return;
            }
        const eph_t ephemeris = decoder.nav.eph[decoder.ephsat - 1];
        if (ephemeris.sat <= 0)
            {
                return;
            }

        std::lock_guard<std::mutex> lock(d_output_mutex);
        // kept sorted by satellite number so the solver's per-epoch lookup
        // can binary-search the snapshot copy
        const auto position = std::lower_bound(
            d_ephemerides.begin(), d_ephemerides.end(), ephemeris.sat,
            [](const eph_t& stored, int sat) { return stored.sat < sat; });
        if (position != d_ephemerides.end() && position->sat == ephemeris.sat)
            {
                // casters rebroadcast unchanged ephemerides every few
                // seconds: only an actually new data set may bump the
                // generation, or the consumers' generation-keyed snapshot
                // cache is invalidated for nothing
                if (!same_broadcast_ephemeris(*position, ephemeris))
                    {
                        *position = ephemeris;
                        ++d_output_generation;
                        DLOG(INFO) << "NTRIP client: updated broadcast ephemeris for satellite "
                                   << ephemeris.sat;
                    }
                return;
            }
        d_ephemerides.insert(position, ephemeris);
        ++d_output_generation;
        DLOG(INFO) << "NTRIP client: stored broadcast ephemeris for satellite "
                   << ephemeris.sat;
    }

    void store_base_position(const rtcm_t& decoder, int message_type)
    {
        std::array<double, 3> position = {{0.0, 0.0, 0.0}};
        sta2antpos(&decoder.sta, position.data());

        bool announce = false;
        bool invalidated_observations = false;
        int invalidated_station_id = 0;
        {
            std::lock_guard<std::mutex> lock(d_output_mutex);
            const bool base_position_changed =
                !d_has_base_position ||
                d_base_position_ecef_m != position ||
                d_base_position_message_type != message_type ||
                d_base_position_station_id != decoder.staid ||
                d_base_position_itrf != decoder.sta.itrf ||
                d_antenna_height_m != decoder.sta.hgt;
            const bool observation_station_mismatch =
                d_has_observations &&
                d_observation_station_id != decoder.staid;
            // a rebroadcast of the same position is not news; a first fill,
            // a station change, or a moved base (e.g. a VRS re-centering) is
            announce = !d_has_base_position ||
                       d_base_position_station_id != decoder.staid ||
                       d_base_position_ecef_m != position;
            if (base_position_changed)
                {
                    d_base_position_ecef_m = position;
                    d_base_position_message_type = message_type;
                    d_base_position_station_id = decoder.staid;
                    d_base_position_itrf = decoder.sta.itrf;
                    d_antenna_height_m = decoder.sta.hgt;
                    d_has_base_position = true;
                }
            if (observation_station_mismatch)
                {
                    invalidated_observations = true;
                    invalidated_station_id = d_observation_station_id;
                    // This publishes both the observation invalidation and
                    // any base-position update as one locked output change.
                    invalidate_observations_locked();
                }
            else if (base_position_changed)
                {
                    ++d_output_generation;
                }
        }
        if (invalidated_observations)
            {
                LOG(WARNING) << "NTRIP client: base position arrived from station "
                             << decoder.staid << " while the cached observations "
                                                 "belong to station "
                             << invalidated_station_id
                             << "; base observations dropped";
            }
        if (announce)
            {
                LOG(INFO) << "NTRIP client: base station " << decoder.staid
                          << " position received (RTCM " << message_type
                          << "): ECEF [" << position[0] << ", " << position[1]
                          << ", " << position[2] << "] m, antenna height "
                          << decoder.sta.hgt << " m";
            }
    }

    void apply_decoder_time(rtcm_t* decoder,
        std::uint64_t* applied_rover_time_generation, bool force)
    {
        std::lock_guard<std::mutex> lock(d_input_mutex);
        if (!force && *applied_rover_time_generation == d_rover_time_generation)
            {
                return;
            }
        decoder->time = d_has_rover_time ? d_rover_time : utc2gpst(timeget());
        *applied_rover_time_generation = d_rover_time_generation;
    }

    void reset_output_data()
    {
        std::lock_guard<std::mutex> lock(d_output_mutex);
        d_observations.clear();
        d_observation_time = gtime_t{0, 0.0};
        d_has_observations = false;
        /* a reset is an output change too: never rewind the generation, or a
           caller could match a stale cached snapshot after restart */
        ++d_output_generation;
        d_observation_station_id = 0;
        invalidate_base_position_locked();
        d_ephemerides.clear();
    }

    void invalidate_observations_locked()
    {
        d_observations.clear();
        d_observation_time = gtime_t{0, 0.0};
        d_has_observations = false;
        d_observation_station_id = 0;
        ++d_output_generation;
    }

    void invalidate_base_position_locked()
    {
        d_has_base_position = false;
        d_base_position_ecef_m = {{0.0, 0.0, 0.0}};
        d_base_position_message_type = 0;
        d_base_position_station_id = 0;
        d_base_position_itrf = 0;
        d_antenna_height_m = 0.0;
    }

    bool wait_interruptible(int milliseconds)
    {
        if (d_stop_requested.load())
            {
                return true;
            }
        std::unique_lock<std::mutex> lock(d_wait_mutex);
        d_wait_condition.wait_for(lock, std::chrono::milliseconds(milliseconds),
            [this]() { return d_stop_requested.load(); });
        return d_stop_requested.load();
    }

    static const char* handshake_failure_message(Handshake_Result result)
    {
        switch (result)
            {
            case Handshake_Result::CONNECT_TIMEOUT:
                return "NTRIP caster connection timed out";
            case Handshake_Result::CONNECT_FAILED:
                return "NTRIP caster connection failed";
            case Handshake_Result::SEND_FAILED:
                return "NTRIP request could not be sent";
            case Handshake_Result::RESPONSE_TIMEOUT:
                return "NTRIP caster response timed out";
            case Handshake_Result::RESPONSE_REJECTED:
                return "NTRIP request was rejected";
            case Handshake_Result::RETRY_V1:
                return "NTRIP v2 exchange requires a v1 retry";
            case Handshake_Result::STOPPED:
                return "stopped";
            case Handshake_Result::SUCCESS:
            default:
                return "NTRIP connection failed";
            }
    }

    void reset_status()
    {
        std::lock_guard<std::mutex> lock(d_status_mutex);
        d_status = Ntrip_Rtcm_Client_Status();
        d_state_generation.fetch_add(1, std::memory_order_relaxed);
    }

    void set_state(Ntrip_Rtcm_Client_State state, const std::string& message,
        bool connected)
    {
        {
            std::lock_guard<std::mutex> lock(d_status_mutex);
            if (d_status.state == state && d_status.connected == connected &&
                d_status.message == message)
                {
                    // reasserting the current status is not a transition:
                    // no generation bump, no log line
                    return;
                }
            d_status.state = state;
            d_status.connected = connected;
            d_status.message = message;
            d_state_generation.fetch_add(1, std::memory_order_relaxed);
        }
        // every lifecycle transition logs exactly once, outside the status
        // lock; failures and reconnect waits are warnings, the rest is
        // informational
        if (state == Ntrip_Rtcm_Client_State::ERROR ||
            state == Ntrip_Rtcm_Client_State::RECONNECT_WAIT)
            {
                LOG(WARNING) << "NTRIP client: " << message;
            }
        else
            {
                LOG(INFO) << "NTRIP client: " << message;
            }
    }

    void add_received_bytes(std::uint64_t count)
    {
        std::lock_guard<std::mutex> lock(d_status_mutex);
        d_status.bytes_received += count;
    }

    void increment_decoded_messages()
    {
        std::lock_guard<std::mutex> lock(d_status_mutex);
        ++d_status.rtcm_messages_decoded;
    }

    void increment_observation_epochs()
    {
        std::lock_guard<std::mutex> lock(d_status_mutex);
        ++d_status.observation_epochs_decoded;
    }

    void increment_decoder_errors()
    {
        std::lock_guard<std::mutex> lock(d_status_mutex);
        ++d_status.decoder_errors;
    }

    void increment_reconnect_count()
    {
        std::lock_guard<std::mutex> lock(d_status_mutex);
        ++d_status.reconnect_count;
    }

    Ntrip_Rtcm_Client_Config d_config;
    Hostname_Resolver d_hostname_resolver;

    mutable std::mutex d_lifecycle_mutex;
    std::thread d_worker;
    std::atomic<bool> d_stop_requested{false};
    std::atomic<bool> d_running{false};
    std::mutex d_wait_mutex;
    std::condition_variable d_wait_condition;
    std::shared_ptr<Host_Resolution_State> d_pending_resolution;

    std::string d_logged_resolved_ipv4;
    bool d_logged_first_observations = false;
    bool d_logged_decoder_error = false;

    mutable std::mutex d_status_mutex;
    Ntrip_Rtcm_Client_Status d_status;
    std::atomic<std::uint64_t> d_state_generation{0};

    mutable std::mutex d_input_mutex;
    std::array<double, 3> d_rover_position_ecef_m = {{0.0, 0.0, 0.0}};
    bool d_has_rover_position = false;
    gtime_t d_rover_time = {0, 0.0};
    bool d_has_rover_time = false;
    std::uint64_t d_rover_time_generation = 0;

    mutable std::mutex d_output_mutex;
    std::vector<obsd_t> d_observations;
    gtime_t d_observation_time = {0, 0.0};
    bool d_has_observations = false;
    // written only under d_output_mutex; atomic so the per-epoch change
    // detector reads it without contending with the worker's deep copies
    std::atomic<std::uint64_t> d_output_generation{0};
    int d_observation_station_id = 0;
    std::array<double, 3> d_base_position_ecef_m = {{0.0, 0.0, 0.0}};
    bool d_has_base_position = false;
    int d_base_position_message_type = 0;
    int d_base_position_station_id = 0;
    int d_base_position_itrf = 0;
    double d_antenna_height_m = 0.0;
    std::vector<eph_t> d_ephemerides;
};


Ntrip_Rtcm_Client::Ntrip_Rtcm_Client(const Ntrip_Rtcm_Client_Config& config)
    : d_impl(new Impl(config))
{
}


Ntrip_Rtcm_Client::~Ntrip_Rtcm_Client() noexcept = default;


bool Ntrip_Rtcm_Client::start()
{
    return d_impl->start();
}


void Ntrip_Rtcm_Client::stop() noexcept
{
    d_impl->stop();
}


bool Ntrip_Rtcm_Client::running() const noexcept
{
    return d_impl->running();
}


Ntrip_Rtcm_Client_Status Ntrip_Rtcm_Client::status() const
{
    return d_impl->status();
}


bool Ntrip_Rtcm_Client::set_hostname_resolver_for_test(
    Hostname_Resolver resolver)
{
    return d_impl->set_hostname_resolver_for_test(std::move(resolver));
}


Ntrip_Rtcm_Snapshot Ntrip_Rtcm_Client::latest_snapshot() const
{
    return d_impl->latest_snapshot();
}


Ntrip_Rtcm_Snapshot Ntrip_Rtcm_Client::latest_snapshot(const gtime_t& rover_time) const
{
    return d_impl->latest_snapshot(rover_time);
}


std::uint64_t Ntrip_Rtcm_Client::snapshot_generation() const noexcept
{
    return d_impl->snapshot_generation();
}


std::uint64_t Ntrip_Rtcm_Client::state_generation() const noexcept
{
    return d_impl->state_generation();
}


void Ntrip_Rtcm_Client::update_rover_position(
    const std::array<double, 3>& position_ecef_m)
{
    d_impl->update_rover_position(position_ecef_m);
}


void Ntrip_Rtcm_Client::update_rover_position(
    const std::array<double, 3>& position_ecef_m, const gtime_t& rover_time)
{
    d_impl->update_rover_position(position_ecef_m, rover_time);
}


void Ntrip_Rtcm_Client::update_rover_time(const gtime_t& rover_time)
{
    d_impl->update_rover_time(rover_time);
}
