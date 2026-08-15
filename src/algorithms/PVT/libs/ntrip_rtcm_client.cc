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
#include "rtklib_rtcm.h"
#include "rtklib_rtkcmn.h"
#include "rtklib_stream.h"
#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <atomic>
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

namespace
{
constexpr std::size_t READ_BUFFER_SIZE = 8192;
constexpr int WORKER_POLL_INTERVAL_MS = 10;
constexpr int MAX_INTERVAL_MS = 24 * 60 * 60 * 1000;


void secure_clear(std::string* value)
{
    if (value == nullptr || value->empty())
        {
            return;
        }
    volatile char* data = &(*value)[0];
    for (std::size_t i = 0; i < value->size(); ++i)
        {
            data[i] = 0;
        }
    value->clear();
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


bool supported_gps_l1_code(unsigned char code)
{
    switch (code)
        {
        case CODE_L1C:
        case CODE_L1P:
        case CODE_L1W:
        case CODE_L1Y:
        case CODE_L1M:
        case CODE_L1N:
        case CODE_L1S:
        case CODE_L1L:
        case CODE_L1X:
            return true;
        default:
            return false;
        }
}


bool supported_gps_l2_code(unsigned char code)
{
    switch (code)
        {
        case CODE_L2C:
        case CODE_L2D:
        case CODE_L2S:
        case CODE_L2L:
        case CODE_L2X:
        case CODE_L2P:
        case CODE_L2W:
        case CODE_L2Y:
        case CODE_L2M:
        case CODE_L2N:
            return true;
        default:
            return false;
        }
}


bool supported_gps_l1_l2_message(int message_type)
{
    return message_type == 1004 ||
           (message_type >= 1074 && message_type <= 1077);
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
                std::memset(ntrip->user, 0, sizeof(ntrip->user));
                std::memset(ntrip->passwd, 0, sizeof(ntrip->passwd));
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
    RESPONSE_REJECTED
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


void resolve_ipv4_hostname(const std::shared_ptr<Host_Resolution_State>& state,
    const std::string& hostname) noexcept
{
    std::string numeric_ipv4;
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
                                    numeric_ipv4 = buffer;
                                    success = true;
                                    break;
                                }
                        }
                }
        }
    catch (...)
        {
            success = false;
            numeric_ipv4.clear();
        }
    if (addresses != nullptr)
        {
            freeaddrinfo(addresses);
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


// The layout follows mutex-protected state groups. Saving a few padding bytes
// in this single-instance class is not worth separating each lock from its data.
// NOLINTNEXTLINE(clang-analyzer-optin.performance.Padding)
class Ntrip_Rtcm_Client::Impl
{
public:
    explicit Impl(const Ntrip_Rtcm_Client_Config& config)
        : d_config(config)
    {
        d_status.state = d_config.enabled ? Ntrip_Rtcm_Client_State::STOPPED : Ntrip_Rtcm_Client_State::DISABLED;
        d_status.message = d_config.enabled ? "stopped" : "disabled";
    }

    ~Impl() noexcept
    {
        stop();
        secure_clear(&d_config.username);
        secure_clear(&d_config.password);
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

    Ntrip_Rtcm_Client_Status status() const
    {
        std::lock_guard<std::mutex> lock(d_status_mutex);
        Ntrip_Rtcm_Client_Status result = d_status;
        result.running = d_running.load();
        return result;
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
            result.observation_generation = d_observation_generation;
            result.has_base_position = d_has_base_position;
            result.base_position_ecef_m = d_base_position_ecef_m;
            result.base_position_message_type = d_base_position_message_type;
            result.station_id = d_has_observations ? d_observation_station_id : d_base_position_station_id;
            result.itrf = d_base_position_itrf;
            result.antenna_height_m = d_antenna_height_m;
            result.gps_ephemerides = d_gps_ephemerides;
        }

        if (result.has_observations)
            {
                const gtime_t age_reference = valid_gtime(reference_time) ? reference_time : utc2gpst(timeget());
                result.age_s = timediff(age_reference, result.observation_time);
                result.fresh = std::isfinite(result.age_s) &&
                               std::fabs(result.age_s) <= d_config.max_age_s;
            }
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
        const std::string mountpoint = normalized_mountpoint(d_config.mountpoint);
        if (d_config.host.empty())
            {
                *reason = "host is empty";
                return false;
            }
        if (d_config.port == 0)
            {
                *reason = "port is zero";
                return false;
            }
        if (mountpoint.empty() || mountpoint.size() >= 256)
            {
                *reason = "mountpoint is empty or too long";
                return false;
            }
        for (char i : d_config.host)
            {
                const auto c = static_cast<unsigned char>(i);
                if (c <= 0x20U || c >= 0x7FU || c == ':' || c == '/' || c == '@' ||
                    c == '[' || c == ']')
                    {
                        *reason = "host contains an unsupported character";
                        return false;
                    }
            }
        for (char i : mountpoint)
            {
                const auto c = static_cast<unsigned char>(i);
                if (c <= 0x20U || c >= 0x7FU || c == '@' || c == ':')
                    {
                        *reason = "mountpoint contains an unsupported character";
                        return false;
                    }
            }
        if (d_config.username.size() >= 256 || d_config.password.size() >= 256)
            {
                *reason = "credentials are too long";
                return false;
            }
        if (d_config.username.empty() && !d_config.password.empty())
            {
                *reason = "password requires a username";
                return false;
            }
        if (d_config.username.find(':') != std::string::npos ||
            d_config.username.find('\r') != std::string::npos ||
            d_config.username.find('\n') != std::string::npos)
            {
                *reason = "username contains an unsupported character";
                return false;
            }
        if (d_config.timeout_ms <= 0 || d_config.timeout_ms > MAX_INTERVAL_MS)
            {
                *reason = "timeout is outside the supported range";
                return false;
            }
        if (d_config.reconnect_interval_ms < 0 ||
            d_config.reconnect_interval_ms > MAX_INTERVAL_MS)
            {
                *reason = "reconnect interval is outside the supported range";
                return false;
            }
        if (!std::isfinite(d_config.max_age_s) || d_config.max_age_s <= 0.0)
            {
                *reason = "maximum correction age must be positive";
                return false;
            }
        if (d_config.send_gga &&
            (d_config.gga_period_ms <= 0 || d_config.gga_period_ms > MAX_INTERVAL_MS))
            {
                *reason = "GGA period is outside the supported range";
                return false;
            }
        if (d_config.station_id < 0 || d_config.station_id > 4095)
            {
                *reason = "station ID is outside the 12-bit RTCM range";
                return false;
            }
        if (d_config.version != 1 && d_config.version != 2)
            {
                *reason = "NTRIP version must be 1 or 2";
                return false;
            }
        if (transport_path().size() >= MAXSTRPATH)
            {
                *reason = "NTRIP endpoint is too long";
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
        std::ostringstream path;
        path << host << ':' << d_config.port << '/'
             << normalized_mountpoint(d_config.mountpoint)
             << "::NTRIP=" << d_config.version;
        if (d_config.tls_enabled)
            {
                path << "::TLS";
            }
        return path.str();
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
                            resolve_ipv4_hostname, *pending_resolution,
                            d_config.host));
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
                std::unique_lock<std::mutex> lock((*pending_resolution)->mutex);
                if ((*pending_resolution)->complete)
                    {
                        break;
                    }
                (*pending_resolution)->condition.wait_for(lock, std::chrono::milliseconds(WORKER_POLL_INTERVAL_MS));
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
                *failure_message = "NTRIP caster hostname resolution timed out";
                return Resolve_Result::FAILED;
            }
        pending_resolution->reset();
        if (!success)
            {
                *failure_message = "NTRIP caster hostname could not be resolved";
                return Resolve_Result::FAILED;
            }
        *resolved_host = numeric_ipv4;
        return Resolve_Result::SUCCESS;
    }

    std::string negotiating_message() const
    {
        std::string message = "negotiating NTRIP v";
        message += (d_config.version == 1 ? '1' : '2');
        message += " stream";
        if (d_config.tls_enabled)
            {
                message += " over TLS";
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
        d_running.store(false);
    }

    void worker_loop()
    {
        std::uint64_t applied_rover_time_generation = 0;
        while (!d_stop_requested.load())
            {
                std::string failure_message = "correction stream disconnected";
                std::string resolved_host;
                const Resolve_Result resolve_result = resolve_transport_host(
                    &d_pending_resolution, &resolved_host, &failure_message);
                if (resolve_result == Resolve_Result::STOPPED)
                    {
                        return;
                    }
                if (resolve_result == Resolve_Result::SUCCESS)
                    {
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
                        Stream_Owner stream;
                        set_state(Ntrip_Rtcm_Client_State::CONNECTING,
                            "connecting to NTRIP caster", false);
                        if (!stream.open(transport_path(resolved_host), d_config.host,
                                d_config.username, d_config.password))
                            {
                                failure_message = "could not initialize NTRIP transport";
                            }
                        else
                            {
                                strsettimeout(stream.get(), d_config.timeout_ms,
                                    d_config.reconnect_interval_ms);
                                const Handshake_Result handshake = perform_handshake(stream.get());
                                if (handshake == Handshake_Result::SUCCESS)
                                    {
                                        apply_decoder_time(decoder.get(), &applied_rover_time_generation, true);
                                        set_state(Ntrip_Rtcm_Client_State::STREAMING,
                                            "receiving RTCM3 corrections", true);
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
                                    }
                            }
                    }

                if (d_stop_requested.load())
                    {
                        return;
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

    Handshake_Result perform_handshake(stream_t* stream)
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
                // (v2 with automatic v1 fallback, or forced v1).
                const int previous_state = ntrip->state;
                const int negotiated = waitntrip(ntrip, stream->msg);
                if (!connected && ntrip->tcp->svr.state == 2)
                    {
                        connected = true;
                        set_state(Ntrip_Rtcm_Client_State::AUTHENTICATING,
                            negotiating_message(), false);
                    }
                if (negotiated != 0 && ntrip->state == 2)
                    {
                        return Handshake_Result::SUCCESS;
                    }
                if (connected && ntrip->tcp->svr.state == 0)
                    {
                        // The transport dropped after being established: either
                        // the caster rejected the request that was already sent
                        // (a response was pending) or the TLS/send step failed.
                        return previous_state == 1
                                   ? Handshake_Result::RESPONSE_REJECTED
                                   : Handshake_Result::CONNECT_FAILED;
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
        return connected ? Handshake_Result::RESPONSE_TIMEOUT
                         : Handshake_Result::CONNECT_TIMEOUT;
    }

    std::string stream_corrections(stream_t* stream, rtcm_t* decoder,
        std::uint64_t* applied_rover_time_generation)
    {
        auto* ntrip = static_cast<ntrip_t*>(stream->port);
        std::array<unsigned char, READ_BUFFER_SIZE> buffer = {{0}};
        bool sent_gga = false;
        std::chrono::steady_clock::time_point last_gga;

        while (!d_stop_requested.load())
            {
                apply_decoder_time(decoder, applied_rover_time_generation, false);
                const int count = strread(stream, buffer.data(), static_cast<int>(buffer.size()));
                if (count > 0)
                    {
                        add_received_bytes(static_cast<std::uint64_t>(count));
                        decode_bytes(decoder, buffer.data(), count);
                    }

                if (ntrip == nullptr || ntrip->tcp == nullptr ||
                    ntrip->tcp->svr.state != 2 || ntrip->state != 2)
                    {
                        return "correction stream disconnected";
                    }

                if (d_config.send_gga)
                    {
                        const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
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
                                        sent_gga = true;
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
                        continue;
                    }
                if (result == 0)
                    {
                        continue;
                    }

                increment_decoded_messages();
                const int message_type = current_rtcm3_message_type(*decoder);
                if (result == 1 && supported_gps_l1_l2_message(message_type))
                    {
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
                if (satsys(decoder.obs.data[i].sat, nullptr) != SYS_GPS)
                    {
                        continue;
                    }
                obsd_t observation = decoder.obs.data[i];
                bool has_l1 = false;
                bool has_l2 = false;
                for (int slot = 0; slot < NFREQ + NEXOBS; ++slot)
                    {
                        const bool has_measurement =
                            (std::isfinite(observation.P[slot]) && observation.P[slot] != 0.0) ||
                            (std::isfinite(observation.L[slot]) && observation.L[slot] != 0.0);
                        const bool keep_l1 = slot == 0 && has_measurement &&
                                             supported_gps_l1_code(observation.code[slot]);
                        const bool keep_l2 = slot == 1 && has_measurement &&
                                             supported_gps_l2_code(observation.code[slot]);
                        const bool keep = keep_l1 || keep_l2;
                        if (!keep)
                            {
                                clear_observation_slot(&observation, slot);
                            }
                        else
                            {
                                has_l1 = has_l1 || keep_l1;
                                has_l2 = has_l2 || keep_l2;
                            }
                    }
                if (has_l1 && has_l2)
                    {
                        observation.rcv = 2;
                        filtered.push_back(observation);
                    }
            }
        if (filtered.empty())
            {
                return;
            }

        {
            std::lock_guard<std::mutex> lock(d_output_mutex);
            if (d_has_base_position && d_base_position_station_id != decoder.staid)
                {
                    invalidate_base_position_locked();
                }
            d_observations.swap(filtered);
            d_observation_time = d_observations.front().time;
            d_observation_station_id = decoder.staid;
            d_has_observations = true;
            ++d_observation_generation;
        }
        increment_observation_epochs();
    }

    void store_ephemeris(const rtcm_t& decoder)
    {
        if (decoder.ephsat <= 0 || satsys(decoder.ephsat, nullptr) != SYS_GPS ||
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
        for (auto& d_gps_ephemeride : d_gps_ephemerides)
            {
                if (d_gps_ephemeride.sat == ephemeris.sat)
                    {
                        d_gps_ephemeride = ephemeris;
                        return;
                    }
            }
        d_gps_ephemerides.push_back(ephemeris);
    }

    void store_base_position(const rtcm_t& decoder, int message_type)
    {
        std::array<double, 3> position = {{decoder.sta.pos[0], decoder.sta.pos[1], decoder.sta.pos[2]}};
        double geodetic_position[3] = {0.0, 0.0, 0.0};
        double delta_ecef[3] = {0.0, 0.0, 0.0};
        ecef2pos(position.data(), geodetic_position);
        if (decoder.sta.deltype != 0)
            {
                double antenna_height_enu[3] = {0.0, 0.0, decoder.sta.hgt};
                enu2ecef(geodetic_position, antenna_height_enu, delta_ecef);
                for (std::size_t i = 0; i < position.size(); ++i)
                    {
                        position[i] += decoder.sta.del[i] + delta_ecef[i];
                    }
            }
        else
            {
                enu2ecef(geodetic_position, decoder.sta.del, delta_ecef);
                for (std::size_t i = 0; i < position.size(); ++i)
                    {
                        position[i] += delta_ecef[i];
                    }
            }

        std::lock_guard<std::mutex> lock(d_output_mutex);
        if (d_has_observations && d_observation_station_id != decoder.staid)
            {
                invalidate_observations_locked();
            }
        d_base_position_ecef_m = position;
        d_base_position_message_type = message_type;
        d_base_position_station_id = decoder.staid;
        d_base_position_itrf = decoder.sta.itrf;
        d_antenna_height_m = decoder.sta.hgt;
        d_has_base_position = true;
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
        d_observation_generation = 0;
        d_observation_station_id = 0;
        invalidate_base_position_locked();
        d_gps_ephemerides.clear();
    }

    void invalidate_observations_locked()
    {
        d_observations.clear();
        d_observation_time = gtime_t{0, 0.0};
        d_has_observations = false;
        d_observation_station_id = 0;
        ++d_observation_generation;
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
    }

    void set_state(Ntrip_Rtcm_Client_State state, const std::string& message,
        bool connected)
    {
        std::lock_guard<std::mutex> lock(d_status_mutex);
        d_status.state = state;
        d_status.connected = connected;
        d_status.message = message;
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

    mutable std::mutex d_lifecycle_mutex;
    std::thread d_worker;
    std::atomic<bool> d_stop_requested{false};
    std::atomic<bool> d_running{false};
    std::mutex d_wait_mutex;
    std::condition_variable d_wait_condition;
    std::shared_ptr<Host_Resolution_State> d_pending_resolution;

    mutable std::mutex d_status_mutex;
    Ntrip_Rtcm_Client_Status d_status;

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
    std::uint64_t d_observation_generation = 0;
    int d_observation_station_id = 0;
    std::array<double, 3> d_base_position_ecef_m = {{0.0, 0.0, 0.0}};
    bool d_has_base_position = false;
    int d_base_position_message_type = 0;
    int d_base_position_station_id = 0;
    int d_base_position_itrf = 0;
    double d_antenna_height_m = 0.0;
    std::vector<eph_t> d_gps_ephemerides;
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


Ntrip_Rtcm_Snapshot Ntrip_Rtcm_Client::latest_snapshot() const
{
    return d_impl->latest_snapshot();
}


Ntrip_Rtcm_Snapshot Ntrip_Rtcm_Client::latest_snapshot(const gtime_t& rover_time) const
{
    return d_impl->latest_snapshot(rover_time);
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
