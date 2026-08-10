/*!
 * \file ntrip_rtcm_client.h
 * \brief NTRIP v1 client for RTCM3 fixed-base corrections
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

#ifndef GNSS_SDR_NTRIP_RTCM_CLIENT_H
#define GNSS_SDR_NTRIP_RTCM_CLIENT_H

#include "rtklib.h"
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */

struct Ntrip_Rtcm_Client_Config
{
    bool enabled = false;
    std::string host;
    std::uint16_t port = 2101;
    std::string mountpoint;
    std::string username;
    std::string password;
    // Zero disables automatic reconnect; positive values select the delay.
    int reconnect_interval_ms = 10000;
    int timeout_ms = 10000;
    double max_age_s = 5.0;
    bool send_gga = false;
    int gga_period_ms = 10000;

    // Zero accepts any stream station while keeping observations and position
    // bound to one ID. A positive value filters to that exact 12-bit ID.
    int station_id = 0;
};


enum class Ntrip_Rtcm_Client_State
{
    DISABLED,
    STOPPED,
    STARTING,
    CONNECTING,
    AUTHENTICATING,
    STREAMING,
    RECONNECT_WAIT,
    ERROR,
    STOPPING
};


struct Ntrip_Rtcm_Client_Status
{
    Ntrip_Rtcm_Client_State state = Ntrip_Rtcm_Client_State::STOPPED;
    bool running = false;
    bool connected = false;
    std::string message;
    std::uint64_t bytes_received = 0;
    std::uint64_t rtcm_messages_decoded = 0;
    std::uint64_t observation_epochs_decoded = 0;
    std::uint64_t decoder_errors = 0;
    std::uint64_t reconnect_count = 0;
};


struct Ntrip_Rtcm_Snapshot
{
    bool has_observations = false;
    std::vector<obsd_t> observations;
    gtime_t observation_time = {0, 0.0};

    // Signed reference-minus-base time difference. Freshness uses its absolute
    // value, so a small future-dated base epoch is accepted as well.
    double age_s = 0.0;
    bool fresh = false;
    std::uint64_t observation_generation = 0;

    bool has_base_position = false;
    std::array<double, 3> base_position_ecef_m = {{0.0, 0.0, 0.0}};
    int base_position_message_type = 0;
    int station_id = 0;
    int itrf = 0;
    double antenna_height_m = 0.0;

    // Latest decoded GPS broadcast ephemeris for each satellite seen on the
    // correction stream. The records own no dynamic memory.
    std::vector<eph_t> gps_ephemerides;
};


/*!
 * \brief Receives and decodes a fixed-base RTCM3 stream from an NTRIP v1
 * caster.
 *
 * The object owns one worker thread. The public snapshot and status methods
 * copy data under a mutex and can be called concurrently with the worker.
 */
class Ntrip_Rtcm_Client
{
public:
    explicit Ntrip_Rtcm_Client(const Ntrip_Rtcm_Client_Config& config);
    ~Ntrip_Rtcm_Client() noexcept;

    Ntrip_Rtcm_Client(const Ntrip_Rtcm_Client&) = delete;
    Ntrip_Rtcm_Client& operator=(const Ntrip_Rtcm_Client&) = delete;
    Ntrip_Rtcm_Client(Ntrip_Rtcm_Client&&) = delete;
    Ntrip_Rtcm_Client& operator=(Ntrip_Rtcm_Client&&) = delete;

    // Returns false when disabled, invalidly configured, or if the worker
    // thread could not be created. Calling start() on a running client is safe.
    bool start();
    void stop() noexcept;
    bool running() const noexcept;
    Ntrip_Rtcm_Client_Status status() const;

    // The no-argument overload evaluates age against current GPST. The rover
    // overload should be used by an RTK solver so recorded data does not depend
    // on wall-clock time.
    Ntrip_Rtcm_Snapshot latest_snapshot() const;
    Ntrip_Rtcm_Snapshot latest_snapshot(const gtime_t& rover_time) const;

    // Position is ECEF in metres. Invalid/non-finite positions disable GGA
    // until another valid position is supplied.
    void update_rover_position(const std::array<double, 3>& position_ecef_m);
    void update_rover_position(const std::array<double, 3>& position_ecef_m,
        const gtime_t& rover_time);

    // Supplies the approximate GPST used to resolve RTCM epoch rollovers. It
    // can be called before start() and updated once per rover observation epoch.
    void update_rover_time(const gtime_t& rover_time);

private:
    class Impl;
    std::unique_ptr<Impl> d_impl;
};

/** \} */
/** \} */

#endif  // GNSS_SDR_NTRIP_RTCM_CLIENT_H
