/*!
 * \file ntrip_rtcm_client_test.cc
 * \brief Unit tests for the NTRIP v1/v2 RTCM3 fixed-base client
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
#include "rtklib_tls.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <sys/select.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

class Ntrip_Rtcm_Client_Test_Access
{
public:
    static bool set_hostname_resolver(Ntrip_Rtcm_Client& client,
        const std::function<bool(const std::string&, std::string*)>& resolver)
    {
        return client.set_hostname_resolver_for_test(resolver);
    }
};

namespace ntrip_rtcm_client_test
{
constexpr int TEST_GPS_WEEK = 2300;
constexpr double TEST_TOW_S = 345600.0;
constexpr int TEST_STATION_ID = 2003;
constexpr int SOCKET_WAIT_MS = 50;
constexpr int SERVER_TIMEOUT_MS = 2000;
constexpr char TRACE_USERNAME[] = "trace-user";
constexpr char TRACE_PASSWORD[] = "NTRIP-PASSWORD-SENTINEL-7f3c";
constexpr char TRACE_BASIC_TOKEN[] =
    "dHJhY2UtdXNlcjpOVFJJUC1QQVNTV09SRC1TRU5USU5FTC03ZjNj";


class File_Descriptor_Zero_Guard
{
public:
    File_Descriptor_Zero_Guard()
        : d_saved_descriptor(dup(STDIN_FILENO))
    {
        if (d_saved_descriptor < 0)
            {
                const int descriptor = open("/dev/null", O_RDONLY);
                if (descriptor >= 0 && descriptor != STDIN_FILENO)
                    {
                        if (dup2(descriptor, STDIN_FILENO) < 0)
                            {
                                d_setup_failed = true;
                            }
                        close(descriptor);
                    }
                else if (descriptor < 0)
                    {
                        d_setup_failed = true;
                    }
            }
        d_ready = !d_setup_failed && fcntl(STDIN_FILENO, F_GETFD) >= 0;
    }

    ~File_Descriptor_Zero_Guard()
    {
        if (d_saved_descriptor >= 0)
            {
                dup2(d_saved_descriptor, STDIN_FILENO);
                close(d_saved_descriptor);
            }
        else if (d_ready)
            {
                close(STDIN_FILENO);
            }
    }

    File_Descriptor_Zero_Guard(const File_Descriptor_Zero_Guard&) = delete;
    File_Descriptor_Zero_Guard& operator=(const File_Descriptor_Zero_Guard&) = delete;

    bool ready() const
    {
        return d_ready;
    }

private:
    int d_saved_descriptor = -1;
    bool d_ready = false;
    bool d_setup_failed = false;
};


class Socket_Pair_Guard
{
public:
    ~Socket_Pair_Guard()
    {
        if (d_sender >= 0)
            {
                close(d_sender);
            }
        if (d_receiver >= 0)
            {
                close(d_receiver);
            }
    }

    Socket_Pair_Guard(const Socket_Pair_Guard&) = delete;
    Socket_Pair_Guard& operator=(const Socket_Pair_Guard&) = delete;

    Socket_Pair_Guard() = default;

    bool open_pair()
    {
        int sockets[2] = {-1, -1};
        if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) != 0)
            {
                return false;
            }
        d_sender = sockets[0];
        d_receiver = sockets[1];
        return true;
    }

    int sender() const
    {
        return d_sender;
    }

    int receiver() const
    {
        return d_receiver;
    }

private:
    int d_sender = -1;
    int d_receiver = -1;
};


class Reserved_Loopback_Port
{
public:
    ~Reserved_Loopback_Port()
    {
        release();
    }

    Reserved_Loopback_Port(const Reserved_Loopback_Port&) = delete;
    Reserved_Loopback_Port& operator=(const Reserved_Loopback_Port&) = delete;

    Reserved_Loopback_Port() = default;

    bool reserve()
    {
        d_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (d_socket < 0)
            {
                return false;
            }

        const int reuse_address = 1;
        setsockopt(d_socket, SOL_SOCKET, SO_REUSEADDR,
            &reuse_address, sizeof(reuse_address));
        struct sockaddr_in address = {};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = 0;
        if (bind(d_socket, reinterpret_cast<struct sockaddr*>(&address),
                sizeof(address)) != 0)
            {
                release();
                return false;
            }

        socklen_t address_length = sizeof(address);
        if (getsockname(d_socket,
                reinterpret_cast<struct sockaddr*>(&address),
                &address_length) != 0)
            {
                release();
                return false;
            }
        d_port = ntohs(address.sin_port);
        return d_port != 0;
    }

    void release()
    {
        if (d_socket >= 0)
            {
                close(d_socket);
                d_socket = -1;
            }
    }

    std::uint16_t port() const
    {
        return d_port;
    }

private:
    int d_socket = -1;
    std::uint16_t d_port = 0;
};


class Blocking_Hostname_Resolver
{
public:
    Blocking_Hostname_Resolver()
        : d_state(new State)
    {
    }

    ~Blocking_Hostname_Resolver()
    {
        release();
    }

    Blocking_Hostname_Resolver(const Blocking_Hostname_Resolver&) = delete;
    Blocking_Hostname_Resolver& operator=(const Blocking_Hostname_Resolver&) = delete;

    std::function<bool(const std::string&, std::string*)> callback() const
    {
        const std::shared_ptr<State> state = d_state;
        return [state](const std::string&, std::string* numeric_ipv4) {
            {
                std::unique_lock<std::mutex> lock(state->mutex);
                ++state->calls;
                state->condition.notify_all();
                state->condition.wait(lock,
                    [state] { return state->released; });
            }
            *numeric_ipv4 = "127.0.0.1";
            {
                std::lock_guard<std::mutex> lock(state->mutex);
                ++state->completions;
            }
            state->condition.notify_all();
            return true;
        };
    }

    bool wait_for_calls(std::size_t count,
        std::chrono::milliseconds timeout) const
    {
        std::unique_lock<std::mutex> lock(d_state->mutex);
        return d_state->condition.wait_for(lock, timeout,
            [this, count] { return d_state->calls >= count; });
    }

    std::size_t call_count() const
    {
        std::lock_guard<std::mutex> lock(d_state->mutex);
        return d_state->calls;
    }

    void release()
    {
        {
            std::lock_guard<std::mutex> lock(d_state->mutex);
            d_state->released = true;
        }
        d_state->condition.notify_all();
    }

    bool wait_for_completions(std::size_t count,
        std::chrono::milliseconds timeout) const
    {
        std::unique_lock<std::mutex> lock(d_state->mutex);
        return d_state->condition.wait_for(lock, timeout,
            [this, count] { return d_state->completions >= count; });
    }

private:
    struct State
    {
        std::mutex mutex;
        std::condition_variable condition;
        std::size_t calls = 0;
        std::size_t completions = 0;
        bool released = false;
    };

    std::shared_ptr<State> d_state;
};


int read_maximum_size_chunk()
{
    tcpcli_t tcp_client = {};
    tcp_client.svr.state = 2;
    tcp_client.svr.sock = -1;

    ntrip_t ntrip = {};
    ntrip.state = 2;
    ntrip.type = 1;
    ntrip.version = NTRIP_VERSION_2;
    ntrip.chunked = 1;
    ntrip.tcp = &tcp_client;

    char encoded_chunk[sizeof(unsigned int) * 2U + 5U] = "";
    const int encoded_size = std::snprintf(encoded_chunk,
        sizeof(encoded_chunk), "%X\r\nX", UINT_MAX);
    if (encoded_size <= 0 ||
        encoded_size >= static_cast<int>(sizeof(encoded_chunk)))
        {
            return 1;
        }
    std::memcpy(ntrip.buff, encoded_chunk,
        static_cast<std::size_t>(encoded_size));
    ntrip.nb = encoded_size;

    unsigned char output = 0;
    char message[MAXSTRMSG] = "";
    const int count = readntrip(&ntrip, &output, 1, message);
    if (count != 1 || output != 'X' ||
        ntrip.chunk_remaining != UINT_MAX - 1U || ntrip.nb != 0)
        {
            return 1;
        }
    return 0;
}


#if GTEST_HAS_DEATH_TEST
int handshake_with_closed_tls_peer()
{
    struct sigaction action = {};
    action.sa_handler = SIG_DFL;
    if (sigemptyset(&action.sa_mask) != 0 ||
        sigaction(SIGPIPE, &action, nullptr) != 0)
        {
            return 1;
        }

    sigset_t signal_set;
    if (sigemptyset(&signal_set) != 0 ||
        sigaddset(&signal_set, SIGPIPE) != 0 ||
        sigprocmask(SIG_UNBLOCK, &signal_set, nullptr) != 0)
        {
            return 2;
        }

    int sockets[2] = {-1, -1};
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) != 0)
        {
            return 3;
        }

    Rtklib_Tls_Client tls_client("localhost");
    char message[MAXSTRMSG] = "";
    if (!tls_client.initialize(message))
        {
            close(sockets[0]);
            close(sockets[1]);
            return 4;
        }

    close(sockets[1]);
    const int handshake_result = tls_client.handshake(sockets[0], message);
    close(sockets[0]);
    return handshake_result == -1 ? 0 : 5;
}
#endif


struct V2_Response_Result
{
    int parsed = 0;
    int state = 0;
    int transport_state = 0;
    int chunked = 0;
    int chunk_state = 0;
    unsigned int chunk_remaining = 0;
    int v1_retry_requested = 0;
    std::string payload;
    std::string message;
};


V2_Response_Result parse_v2_response(const std::string& response,
    const char* mountpoint = "BASE")
{
    tcpcli_t tcp_client = {};
    tcp_client.svr.state = 2;
    tcp_client.svr.sock = -1;

    ntrip_t ntrip = {};
    ntrip.state = 1;
    ntrip.type = 1;
    ntrip.version = NTRIP_VERSION_2;
    ntrip.chunked = 1;
    ntrip.chunk_state = 1;
    ntrip.chunk_remaining = 42;
    ntrip.tcp = &tcp_client;
    std::snprintf(ntrip.host, sizeof(ntrip.host), "caster.example");
    std::snprintf(ntrip.mntpnt, sizeof(ntrip.mntpnt), "%s", mountpoint);
    if (response.size() >= sizeof(ntrip.buff))
        {
            return V2_Response_Result();
        }
    std::memcpy(ntrip.buff, response.data(), response.size());
    ntrip.nb = static_cast<int>(response.size());
    ntrip.buff[ntrip.nb] = '\0';

    char message[MAXSTRMSG] = "";
    V2_Response_Result result;
    result.parsed = rspntrip_c_v2(&ntrip, message);
    result.state = ntrip.state;
    result.transport_state = tcp_client.svr.state;
    result.chunked = ntrip.chunked;
    result.chunk_state = ntrip.chunk_state;
    result.chunk_remaining = ntrip.chunk_remaining;
    result.v1_retry_requested = ntrip.v1_retry_requested;
    result.payload.assign(reinterpret_cast<char*>(ntrip.buff),
        static_cast<std::size_t>(ntrip.nb));
    result.message = message;
    return result;
}


bool wait_readable(int socket, int timeout_ms)
{
    for (;;)
        {
            fd_set read_set;
            FD_ZERO(&read_set);
            FD_SET(socket, &read_set);
            struct timeval timeout = {
                timeout_ms / 1000, (timeout_ms % 1000) * 1000};
            const int result = select(socket + 1, &read_set, nullptr, nullptr, &timeout);
            if (result < 0 && errno == EINTR)
                {
                    continue;
                }
            return result > 0 && FD_ISSET(socket, &read_set);
        }
}


bool send_all(int socket, const unsigned char* data, std::size_t size)
{
#if defined(SO_NOSIGPIPE)
    const int enabled = 1;
    setsockopt(socket, SOL_SOCKET, SO_NOSIGPIPE, &enabled, sizeof(enabled));
#endif

    std::size_t sent = 0;
    while (sent < size)
        {
#if defined(MSG_NOSIGNAL)
            constexpr int send_flags = MSG_NOSIGNAL;
#else
            constexpr int send_flags = 0;
#endif
            const ssize_t count = send(socket, data + sent, size - sent, send_flags);
            if (count < 0 && errno == EINTR)
                {
                    continue;
                }
            if (count <= 0)
                {
                    return false;
                }
            sent += static_cast<std::size_t>(count);
        }
    return true;
}


bool set_nonblocking(int socket)
{
    const int flags = fcntl(socket, F_GETFL, 0);
    return flags >= 0 && fcntl(socket, F_SETFL, flags | O_NONBLOCK) == 0;
}


std::size_t fill_socket_send_buffer(int socket)
{
    const unsigned char fill[256] = {'#'};
    std::size_t total = 0;
    for (;;)
        {
#if defined(MSG_NOSIGNAL)
            constexpr int send_flags = MSG_NOSIGNAL;
#else
            constexpr int send_flags = 0;
#endif
            const ssize_t count = send(socket, fill, sizeof(fill), send_flags);
            if (count > 0)
                {
                    total += static_cast<std::size_t>(count);
                    continue;
                }
            if (count < 0 && errno == EINTR)
                {
                    continue;
                }
            if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                {
                    return total;
                }
            return 0;
        }
}


std::size_t receive_available(int socket, std::size_t maximum,
    std::string* received)
{
    std::size_t total = 0;
    char buffer[256];
    while (total < maximum)
        {
            const std::size_t wanted = std::min(sizeof(buffer), maximum - total);
            const ssize_t count = recv(socket, buffer, wanted, 0);
            if (count > 0)
                {
                    if (received != nullptr)
                        {
                            received->append(buffer, static_cast<std::size_t>(count));
                        }
                    total += static_cast<std::size_t>(count);
                    continue;
                }
            if (count < 0 && errno == EINTR)
                {
                    continue;
                }
            break;
        }
    return total;
}


void append_chunk(std::vector<unsigned char>* out, const unsigned char* data,
    std::size_t size)
{
    char header[16];
    const int header_size = std::snprintf(header, sizeof(header), "%zx\r\n", size);
    out->insert(out->end(), header, header + header_size);
    out->insert(out->end(), data, data + size);
    out->push_back('\r');
    out->push_back('\n');
}


std::vector<unsigned char> make_mt1005()
{
    // Public RTCM 1005 fixture also used by RtcmTest.MT1005.
    const unsigned char frame[] = {
        0xD3, 0x00, 0x13, 0x3E, 0xD7, 0xD3, 0x02, 0x02, 0x98,
        0x0E, 0xDE, 0xEF, 0x34, 0xB4, 0xBD, 0x62, 0xAC, 0x09,
        0x41, 0x98, 0x6F, 0x33, 0x36, 0x0B, 0x98};
    return std::vector<unsigned char>(frame, frame + sizeof(frame));
}


std::vector<unsigned char> make_mt1006()
{
    const std::vector<unsigned char> mt1005 = make_mt1005();
    constexpr int payload_size = 21;
    std::vector<unsigned char> frame(static_cast<std::size_t>(3 + payload_size + 3), 0);
    frame[0] = 0xD3;
    setbitu(frame.data(), 14, 10, payload_size);
    std::copy(mt1005.begin() + 3, mt1005.begin() + 22, frame.begin() + 3);
    setbitu(frame.data(), 24, 12, 1006);
    setbitu(frame.data(), 176, 16, 0);  // Zero antenna height keeps the ARP unchanged.

    const unsigned int crc = rtk_crc24q(frame.data(), 3 + payload_size);
    frame[3 + payload_size] = static_cast<unsigned char>((crc >> 16U) & 0xFFU);
    frame[4 + payload_size] = static_cast<unsigned char>((crc >> 8U) & 0xFFU);
    frame[5 + payload_size] = static_cast<unsigned char>(crc & 0xFFU);
    return frame;
}


void append_mt1004_satellite(std::vector<unsigned char>* frame, int* bit,
    unsigned int encoded_prn)
{
    setbitu(frame->data(), *bit, 6, encoded_prn);
    *bit += 6;
    setbitu(frame->data(), *bit, 1, 0);  // GPS L1 C/A
    *bit += 1;
    setbitu(frame->data(), *bit, 24, 100000);
    *bit += 24;
    setbits(frame->data(), *bit, 20, 0);
    *bit += 20;
    setbitu(frame->data(), *bit, 7, 30);
    *bit += 7;
    setbitu(frame->data(), *bit, 8, 70);
    *bit += 8;
    setbitu(frame->data(), *bit, 8, 180);
    *bit += 8;
    setbitu(frame->data(), *bit, 2, 0);  // GPS L2C(M+L)
    *bit += 2;
    setbits(frame->data(), *bit, 14, 0);
    *bit += 14;
    setbits(frame->data(), *bit, 20, 0);
    *bit += 20;
    setbitu(frame->data(), *bit, 7, 30);
    *bit += 7;
    setbitu(frame->data(), *bit, 8, 180);
    *bit += 8;
}


std::vector<unsigned char> make_mt1004()
{
    // 64 header bits plus two 125-bit observation records require 40 payload
    // bytes. The second record uses encoded PRN 40, which RTCM maps to SBAS 120;
    // it verifies that the client publishes GPS observations only.
    constexpr int payload_size = 40;
    std::vector<unsigned char> frame(static_cast<std::size_t>(3 + payload_size + 3), 0);
    frame[0] = 0xD3;
    setbitu(frame.data(), 14, 10, payload_size);

    int bit = 24;
    setbitu(frame.data(), bit, 12, 1004);
    bit += 12;
    setbitu(frame.data(), bit, 12, TEST_STATION_ID);
    bit += 12;
    setbitu(frame.data(), bit, 30,
        static_cast<unsigned int>(TEST_TOW_S * 1000.0));
    bit += 30;
    setbitu(frame.data(), bit, 1, 0);  // Last message in the epoch
    bit += 1;
    setbitu(frame.data(), bit, 5, 2);  // GPS plus an SBAS record
    bit += 5;
    setbitu(frame.data(), bit, 4, 0);  // No smoothing
    bit += 4;

    append_mt1004_satellite(&frame, &bit, 3);   // GPS PRN 3
    append_mt1004_satellite(&frame, &bit, 40);  // SBAS PRN 120

    const unsigned int crc = rtk_crc24q(frame.data(), 3 + payload_size);
    frame[3 + payload_size] = static_cast<unsigned char>((crc >> 16U) & 0xFFU);
    frame[4 + payload_size] = static_cast<unsigned char>((crc >> 8U) & 0xFFU);
    frame[5 + payload_size] = static_cast<unsigned char>(crc & 0xFFU);
    return frame;
}


std::vector<unsigned char> make_mt1002()
{
    // Extended L1-only GPS observation: a valid single-band correction
    // epoch since single-frequency RTK support.
    constexpr int payload_size = 18;
    std::vector<unsigned char> frame(static_cast<std::size_t>(3 + payload_size + 3), 0);
    frame[0] = 0xD3;
    setbitu(frame.data(), 14, 10, payload_size);

    int bit = 24;
    setbitu(frame.data(), bit, 12, 1002);
    bit += 12;
    setbitu(frame.data(), bit, 12, TEST_STATION_ID);
    bit += 12;
    setbitu(frame.data(), bit, 30,
        static_cast<unsigned int>(TEST_TOW_S * 1000.0));
    bit += 30;
    setbitu(frame.data(), bit, 1, 0);
    bit += 1;
    setbitu(frame.data(), bit, 5, 1);
    bit += 5;
    setbitu(frame.data(), bit, 4, 0);
    bit += 4;
    setbitu(frame.data(), bit, 6, 8);
    bit += 6;
    setbitu(frame.data(), bit, 1, 0);
    bit += 1;
    setbitu(frame.data(), bit, 24, 100000);
    bit += 24;
    setbits(frame.data(), bit, 20, 0);
    bit += 20;
    setbitu(frame.data(), bit, 7, 30);
    bit += 7;
    setbitu(frame.data(), bit, 8, 70);
    bit += 8;
    setbitu(frame.data(), bit, 8, 180);

    const unsigned int crc = rtk_crc24q(frame.data(), 3 + payload_size);
    frame[3 + payload_size] = static_cast<unsigned char>((crc >> 16U) & 0xFFU);
    frame[4 + payload_size] = static_cast<unsigned char>((crc >> 8U) & 0xFFU);
    frame[5 + payload_size] = static_cast<unsigned char>(crc & 0xFFU);
    return frame;
}


std::vector<unsigned char> make_mt1074(bool half_cycle_ambiguity = false,
    unsigned int lock_indicator = 5,
    double tow_s = TEST_TOW_S)
{
    // One GPS satellite with L1 C/A (signal ID 2) and L2C(M) (signal ID 15).
    // Nonzero fine code and phase fields make both decoded bands usable.
    constexpr int payload_size = 36;
    std::vector<unsigned char> frame(static_cast<std::size_t>(3 + payload_size + 3), 0);
    frame[0] = 0xD3;
    setbitu(frame.data(), 14, 10, payload_size);

    int bit = 24;
    setbitu(frame.data(), bit, 12, 1074);
    bit += 12;
    setbitu(frame.data(), bit, 12, TEST_STATION_ID);
    bit += 12;
    setbitu(frame.data(), bit, 30,
        static_cast<unsigned int>(tow_s * 1000.0));
    bit += 30;
    setbitu(frame.data(), bit, 1, 0);  // Last message in the epoch
    bit += 1;
    setbitu(frame.data(), bit, 3, 0);  // Issue of data station
    bit += 3;
    bit += 7;  // Reserved
    setbitu(frame.data(), bit, 2, 0);
    bit += 2;
    setbitu(frame.data(), bit, 2, 0);
    bit += 2;
    setbitu(frame.data(), bit, 1, 0);
    bit += 1;
    setbitu(frame.data(), bit, 3, 0);
    bit += 3;

    for (int satellite_id = 1; satellite_id <= 64; ++satellite_id)
        {
            setbitu(frame.data(), bit++, 1, satellite_id == 3 ? 1U : 0U);
        }
    for (int signal_id = 1; signal_id <= 32; ++signal_id)
        {
            const bool selected = signal_id == 2 || signal_id == 15;
            setbitu(frame.data(), bit++, 1, selected ? 1U : 0U);
        }
    setbitu(frame.data(), bit++, 1, 1);
    setbitu(frame.data(), bit++, 1, 1);

    setbitu(frame.data(), bit, 8, 70);  // Rough range in milliseconds
    bit += 8;
    setbitu(frame.data(), bit, 10, 512);
    bit += 10;
    setbits(frame.data(), bit, 15, 1000);
    bit += 15;
    setbits(frame.data(), bit, 15, 2000);
    bit += 15;
    setbits(frame.data(), bit, 22, 1500);
    bit += 22;
    setbits(frame.data(), bit, 22, 2500);
    bit += 22;
    setbitu(frame.data(), bit, 4, lock_indicator);  // Lock time indicator, signal 1
    bit += 4;
    setbitu(frame.data(), bit, 4, lock_indicator);  // Lock time indicator, signal 2
    bit += 4;
    setbitu(frame.data(), bit++, 1, half_cycle_ambiguity ? 1U : 0U);
    setbitu(frame.data(), bit++, 1, half_cycle_ambiguity ? 1U : 0U);
    setbitu(frame.data(), bit, 6, 45);
    bit += 6;
    setbitu(frame.data(), bit, 6, 45);

    const unsigned int crc = rtk_crc24q(frame.data(), 3 + payload_size);
    frame[3 + payload_size] = static_cast<unsigned char>((crc >> 16U) & 0xFFU);
    frame[4 + payload_size] = static_cast<unsigned char>((crc >> 8U) & 0xFFU);
    frame[5 + payload_size] = static_cast<unsigned char>(crc & 0xFFU);
    return frame;
}


std::vector<unsigned char> make_mt1124()
{
    // One BeiDou satellite (C19) broadcasting B1I (signal ID 2) and B1C pilot
    // (signal ID 31), which contend for the same first frequency slot
    constexpr int payload_size = 36;
    std::vector<unsigned char> frame(static_cast<std::size_t>(3 + payload_size + 3), 0);
    frame[0] = 0xD3;
    setbitu(frame.data(), 14, 10, payload_size);

    int bit = 24;
    setbitu(frame.data(), bit, 12, 1124);
    bit += 12;
    setbitu(frame.data(), bit, 12, TEST_STATION_ID);
    bit += 12;
    setbitu(frame.data(), bit, 30,
        static_cast<unsigned int>(TEST_TOW_S * 1000.0));
    bit += 30;
    setbitu(frame.data(), bit, 1, 0);  // Last message in the epoch
    bit += 1;
    setbitu(frame.data(), bit, 3, 0);  // Issue of data station
    bit += 3;
    bit += 7;  // Reserved
    setbitu(frame.data(), bit, 2, 0);
    bit += 2;
    setbitu(frame.data(), bit, 2, 0);
    bit += 2;
    setbitu(frame.data(), bit, 1, 0);
    bit += 1;
    setbitu(frame.data(), bit, 3, 0);
    bit += 3;

    for (int satellite_id = 1; satellite_id <= 64; ++satellite_id)
        {
            setbitu(frame.data(), bit++, 1, satellite_id == 19 ? 1U : 0U);
        }
    for (int signal_id = 1; signal_id <= 32; ++signal_id)
        {
            const bool selected = signal_id == 2 || signal_id == 31;
            setbitu(frame.data(), bit++, 1, selected ? 1U : 0U);
        }
    setbitu(frame.data(), bit++, 1, 1);
    setbitu(frame.data(), bit++, 1, 1);

    setbitu(frame.data(), bit, 8, 75);  // Rough range in milliseconds
    bit += 8;
    setbitu(frame.data(), bit, 10, 512);
    bit += 10;
    setbits(frame.data(), bit, 15, 1000);
    bit += 15;
    setbits(frame.data(), bit, 15, 2000);
    bit += 15;
    setbits(frame.data(), bit, 22, 1500);
    bit += 22;
    setbits(frame.data(), bit, 22, 2500);
    bit += 22;
    setbitu(frame.data(), bit, 4, 5);  // Lock time indicator, signal 1
    bit += 4;
    setbitu(frame.data(), bit, 4, 5);  // Lock time indicator, signal 2
    bit += 4;
    setbitu(frame.data(), bit++, 1, 0);
    setbitu(frame.data(), bit++, 1, 0);
    setbitu(frame.data(), bit, 6, 45);
    bit += 6;
    setbitu(frame.data(), bit, 6, 45);

    const unsigned int crc = rtk_crc24q(frame.data(), 3 + payload_size);
    frame[3 + payload_size] = static_cast<unsigned char>((crc >> 16U) & 0xFFU);
    frame[4 + payload_size] = static_cast<unsigned char>((crc >> 8U) & 0xFFU);
    frame[5 + payload_size] = static_cast<unsigned char>(crc & 0xFFU);
    return frame;
}


enum class Loopback_Caster_Close_Mode
{
    KEEP_OPEN,
    AFTER_PAYLOAD,
    ON_ACCEPT
};


enum class Loopback_Caster_Response_Mode
{
    V1_ICY,
    V2_HTTP_CHUNKED,
    V2_HTTP_UNAUTHORIZED,
    STRICT_V1_AFTER_V2_REJECTION,
    STRICT_V1_AFTER_V2_CLOSE,
    V2_RECOVERY_AFTER_TRANSIENT_CLOSES,
    LOCK_RESET_AFTER_RECONNECT,
    REPEATED_BASE_POSITIONS
};


class Loopback_Ntrip_Caster
{
public:
    explicit Loopback_Ntrip_Caster(
        Loopback_Caster_Close_Mode close_mode = Loopback_Caster_Close_Mode::KEEP_OPEN,
        Loopback_Caster_Response_Mode response_mode = Loopback_Caster_Response_Mode::V1_ICY)
        : d_close_mode(close_mode),
          d_response_mode(response_mode)
    {
    }

    ~Loopback_Ntrip_Caster()
    {
        d_stop_requested.store(true);
        if (d_worker.joinable())
            {
                d_worker.join();
            }
        if (d_listen_socket >= 0)
            {
                close(d_listen_socket);
            }
    }

    Loopback_Ntrip_Caster(const Loopback_Ntrip_Caster&) = delete;
    Loopback_Ntrip_Caster& operator=(const Loopback_Ntrip_Caster&) = delete;

    bool start(std::uint16_t requested_port = 0)
    {
        d_listen_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (d_listen_socket < 0)
            {
                record_start_failure("socket");
                return false;
            }

        const int reuse_address = 1;
        setsockopt(d_listen_socket, SOL_SOCKET, SO_REUSEADDR,
            &reuse_address, sizeof(reuse_address));
        struct sockaddr_in address = {};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(requested_port);
        if (bind(d_listen_socket, reinterpret_cast<struct sockaddr*>(&address),
                sizeof(address)) != 0)
            {
                record_start_failure("bind");
                close(d_listen_socket);
                d_listen_socket = -1;
                return false;
            }
        if (listen(d_listen_socket, 1) != 0)
            {
                record_start_failure("listen");
                close(d_listen_socket);
                d_listen_socket = -1;
                return false;
            }

        socklen_t address_length = sizeof(address);
        if (getsockname(d_listen_socket,
                reinterpret_cast<struct sockaddr*>(&address),
                &address_length) != 0)
            {
                record_start_failure("getsockname");
                close(d_listen_socket);
                d_listen_socket = -1;
                return false;
            }
        d_port = ntohs(address.sin_port);
        try
            {
                d_worker = std::thread(&Loopback_Ntrip_Caster::serve, this);
            }
        catch (...)
            {
                d_start_failure = "thread";
                close(d_listen_socket);
                d_listen_socket = -1;
                return false;
            }
        return true;
    }

    void join()
    {
        if (d_worker.joinable())
            {
                d_worker.join();
            }
    }

    std::uint16_t port() const
    {
        return d_port;
    }

    const std::string& start_failure() const
    {
        return d_start_failure;
    }

    int start_errno() const
    {
        return d_start_errno;
    }

    const std::string& request() const
    {
        return d_request;
    }

    const std::vector<std::string>& requests() const
    {
        return d_requests;
    }

    std::size_t accepted_connection_count() const
    {
        return d_accepted_connection_count;
    }

    bool accepted_client() const
    {
        return d_accepted_client;
    }

    bool sent_fragmented_response() const
    {
        return d_sent_fragmented_response;
    }

    bool sent_coalesced_payload() const
    {
        return d_sent_coalesced_payload;
    }

    bool peer_closed_cleanly() const
    {
        return d_peer_closed_cleanly;
    }

    bool server_closed_after_payload() const
    {
        return d_server_closed_after_payload;
    }

    bool server_closed_on_accept() const
    {
        return d_server_closed_on_accept;
    }

    bool received_first_v2_request() const
    {
        return d_received_first_v2_request;
    }

    // safe to poll while the caster is serving
    bool received_gga() const
    {
        return d_received_gga.load();
    }

private:
    void record_start_failure(const char* operation)
    {
        d_start_failure = operation;
        d_start_errno = errno;
    }

    int accept_client()
    {
        const std::chrono::steady_clock::time_point accept_deadline =
            std::chrono::steady_clock::now() +
            std::chrono::milliseconds(SERVER_TIMEOUT_MS);
        while (!d_stop_requested.load() &&
               std::chrono::steady_clock::now() < accept_deadline)
            {
                if (!wait_readable(d_listen_socket, SOCKET_WAIT_MS))
                    {
                        continue;
                    }
                const int client_socket = accept(d_listen_socket, nullptr, nullptr);
                if (client_socket >= 0)
                    {
                        d_accepted_client = true;
                        ++d_accepted_connection_count;
                        return client_socket;
                    }
                if (errno != EINTR)
                    {
                        return -1;
                    }
            }
        return -1;
    }

    void serve()
    {
        int client_socket = accept_client();
        if (client_socket < 0)
            {
                return;
            }

        if (d_close_mode == Loopback_Caster_Close_Mode::ON_ACCEPT)
            {
                // Discarding unread request bytes and setting zero linger makes
                // the peer observe a reset while it is writing/authenticating.
                struct linger reset_linger = {};
                reset_linger.l_onoff = 1;
                reset_linger.l_linger = 0;
                setsockopt(client_socket, SOL_SOCKET, SO_LINGER,
                    &reset_linger, sizeof(reset_linger));
                close(client_socket);
                d_server_closed_on_accept = true;
                return;
            }

        std::string received_request;
        receive_request(client_socket, &received_request);
        d_requests.push_back(received_request);

        if (d_response_mode == Loopback_Caster_Response_Mode::V2_HTTP_UNAUTHORIZED)
            {
                const char unauthorized[] =
                    "HTTP/1.1 401 Unauthorized\r\n"
                    "Connection: close\r\n"
                    "\r\n";
                d_request = received_request;
                send_all(client_socket,
                    reinterpret_cast<const unsigned char*>(unauthorized),
                    sizeof(unauthorized) - 1);
                shutdown(client_socket, SHUT_RDWR);
                close(client_socket);
                return;
            }

        if (d_response_mode ==
                Loopback_Caster_Response_Mode::STRICT_V1_AFTER_V2_REJECTION ||
            d_response_mode ==
                Loopback_Caster_Response_Mode::STRICT_V1_AFTER_V2_CLOSE)
            {
                const char rejection[] =
                    "HTTP/1.1 400 Bad Request\r\n"
                    "Connection: close\r\n"
                    "\r\n";
                d_received_first_v2_request =
                    received_request.find("GET /BASE HTTP/1.1\r\n") == 0;
                if (d_response_mode ==
                    Loopback_Caster_Response_Mode::STRICT_V1_AFTER_V2_REJECTION)
                    {
                        d_received_first_v2_request =
                            d_received_first_v2_request &&
                            send_all(client_socket,
                                reinterpret_cast<const unsigned char*>(rejection),
                                sizeof(rejection) - 1);
                    }
                shutdown(client_socket, SHUT_RDWR);
                close(client_socket);
                client_socket = accept_client();
                if (client_socket < 0)
                    {
                        return;
                    }
                received_request.clear();
                receive_request(client_socket, &received_request);
                d_requests.push_back(received_request);
            }

        if (d_response_mode ==
            Loopback_Caster_Response_Mode::V2_RECOVERY_AFTER_TRANSIENT_CLOSES)
            {
                // two consecutive zero-byte closes: the v2 attempt and its v1
                // retry both look like a transient caster drop; the third
                // connection is then served normally
                for (int closed_connection = 0; closed_connection < 2; ++closed_connection)
                    {
                        shutdown(client_socket, SHUT_RDWR);
                        close(client_socket);
                        client_socket = accept_client();
                        if (client_socket < 0)
                            {
                                return;
                            }
                        received_request.clear();
                        receive_request(client_socket, &received_request);
                        d_requests.push_back(received_request);
                    }
            }

        if (d_response_mode ==
            Loopback_Caster_Response_Mode::LOCK_RESET_AFTER_RECONNECT)
            {
                // first connection: healthy stream with a long lock time, then
                // a drop; second connection: the base receiver re-locked during
                // the outage, so the lock-time indicator decreased
                const char icy_response[] = "ICY 200 OK\r\n\r\n";
                const std::vector<unsigned char> base_position = make_mt1005();

                std::vector<unsigned char> first_payload = base_position;
                const std::vector<unsigned char> locked_observations =
                    make_mt1074(false, 9, TEST_TOW_S);
                first_payload.insert(first_payload.end(),
                    locked_observations.begin(), locked_observations.end());
                send_all(client_socket,
                    reinterpret_cast<const unsigned char*>(icy_response),
                    sizeof(icy_response) - 1);
                send_all(client_socket, first_payload.data(), first_payload.size());
                // give the client time to ingest the epoch before the drop
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                shutdown(client_socket, SHUT_RDWR);
                close(client_socket);

                client_socket = accept_client();
                if (client_socket < 0)
                    {
                        return;
                    }
                received_request.clear();
                receive_request(client_socket, &received_request);
                d_requests.push_back(received_request);
                d_request = received_request;

                std::vector<unsigned char> second_payload = base_position;
                const std::vector<unsigned char> relocked_observations =
                    make_mt1074(false, 1, TEST_TOW_S + 1.0);
                second_payload.insert(second_payload.end(),
                    relocked_observations.begin(), relocked_observations.end());
                send_all(client_socket,
                    reinterpret_cast<const unsigned char*>(icy_response),
                    sizeof(icy_response) - 1);
                send_all(client_socket, second_payload.data(), second_payload.size());

                // hold the connection open until the peer closes it (the
                // client's stop()), a stop request, or the server timeout
                const std::chrono::steady_clock::time_point close_deadline =
                    std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(SERVER_TIMEOUT_MS);
                unsigned char discard[64];
                while (!d_stop_requested.load() &&
                       std::chrono::steady_clock::now() < close_deadline)
                    {
                        if (!wait_readable(client_socket, SOCKET_WAIT_MS))
                            {
                                continue;
                            }
                        const ssize_t count = recv(client_socket, discard,
                            sizeof(discard), 0);
                        if (count == 0)
                            {
                                d_peer_closed_cleanly = true;
                                break;
                            }
                        if (count < 0 && errno != EINTR)
                            {
                                break;
                            }
                    }
                shutdown(client_socket, SHUT_RDWR);
                close(client_socket);
                return;
            }
        d_request = received_request;

        if (d_request.find("\r\n\r\n") != std::string::npos)
            {
                std::vector<unsigned char> rtcm_payload;
                const std::vector<unsigned char> base_position = make_mt1005();
                const std::vector<unsigned char> base_position_with_height = make_mt1006();
                if (d_response_mode ==
                    Loopback_Caster_Response_Mode::REPEATED_BASE_POSITIONS)
                    {
                        for (int repetition = 0; repetition < 2; ++repetition)
                            {
                                rtcm_payload.insert(rtcm_payload.end(),
                                    base_position.begin(), base_position.end());
                            }
                        for (int repetition = 0; repetition < 2; ++repetition)
                            {
                                rtcm_payload.insert(rtcm_payload.end(),
                                    base_position_with_height.begin(),
                                    base_position_with_height.end());
                            }
                    }
                else
                    {
                        std::vector<unsigned char> corrupted_frame = base_position;
                        corrupted_frame.back() ^= 0x01U;
                        const std::vector<unsigned char> observations = make_mt1004();
                        const std::vector<unsigned char> l1_only_observations = make_mt1002();
                        const std::vector<unsigned char> msm4_observations = make_mt1074();
                        rtcm_payload.insert(rtcm_payload.end(),
                            corrupted_frame.begin(), corrupted_frame.end());
                        rtcm_payload.insert(rtcm_payload.end(),
                            base_position.begin(), base_position.end());
                        rtcm_payload.insert(rtcm_payload.end(),
                            base_position_with_height.begin(),
                            base_position_with_height.end());
                        rtcm_payload.insert(rtcm_payload.end(),
                            observations.begin(), observations.end());
                        rtcm_payload.insert(rtcm_payload.end(),
                            l1_only_observations.begin(), l1_only_observations.end());
                        rtcm_payload.insert(rtcm_payload.end(),
                            msm4_observations.begin(), msm4_observations.end());
                    }

                if (d_response_mode == Loopback_Caster_Response_Mode::V2_HTTP_CHUNKED)
                    {
                        // Fragment the status line so the client has to wait
                        // for more data before recognizing the HTTP response.
                        const char first_fragment[] = "HTTP/1.1 20";
                        d_sent_fragmented_response = send_all(client_socket,
                            reinterpret_cast<const unsigned char*>(first_fragment),
                            sizeof(first_fragment) - 1);
                        std::this_thread::sleep_for(std::chrono::milliseconds(20));

                        const char remaining_headers[] =
                            "0 OK\r\n"
                            "Ntrip-Version: Ntrip/2.0\r\n"
                            "Transfer-Encoding: chunked\r\n"
                            "\r\n";
                        std::vector<unsigned char> coalesced_payload(
                            remaining_headers,
                            remaining_headers + sizeof(remaining_headers) - 1);
                        // Split the RTCM frames across two HTTP chunks so the
                        // client reassembles a frame crossing a chunk boundary.
                        const std::size_t first_chunk_size = rtcm_payload.size() / 2;
                        append_chunk(&coalesced_payload, rtcm_payload.data(),
                            first_chunk_size);
                        append_chunk(&coalesced_payload,
                            rtcm_payload.data() + first_chunk_size,
                            rtcm_payload.size() - first_chunk_size);
                        d_sent_coalesced_payload = d_sent_fragmented_response &&
                                                   send_all(client_socket,
                                                       coalesced_payload.data(),
                                                       coalesced_payload.size());
                    }
                else
                    {
                        const unsigned char first_fragment[] = {'I', 'C', 'Y', ' ', '2', '0'};
                        d_sent_fragmented_response = send_all(client_socket,
                            first_fragment, sizeof(first_fragment));
                        std::this_thread::sleep_for(std::chrono::milliseconds(20));

                        std::vector<unsigned char> coalesced_payload = {
                            '0', ' ', 'O', 'K', '\r', '\n'};
                        coalesced_payload.insert(coalesced_payload.end(),
                            rtcm_payload.begin(), rtcm_payload.end());
                        d_sent_coalesced_payload = d_sent_fragmented_response &&
                                                   send_all(client_socket,
                                                       coalesced_payload.data(),
                                                       coalesced_payload.size());
                    }
            }

        if (d_close_mode == Loopback_Caster_Close_Mode::AFTER_PAYLOAD &&
            d_sent_coalesced_payload)
            {
                shutdown(client_socket, SHUT_RDWR);
                close(client_socket);
                d_server_closed_after_payload = true;
                return;
            }

        const std::chrono::steady_clock::time_point close_deadline =
            std::chrono::steady_clock::now() +
            std::chrono::milliseconds(SERVER_TIMEOUT_MS);
        unsigned char discard[256];
        std::string upstream;
        while (!d_stop_requested.load() &&
               std::chrono::steady_clock::now() < close_deadline)
            {
                if (!wait_readable(client_socket, SOCKET_WAIT_MS))
                    {
                        continue;
                    }
                const ssize_t count = recv(client_socket, discard, sizeof(discard), 0);
                if (count == 0)
                    {
                        d_peer_closed_cleanly = true;
                        break;
                    }
                if (count < 0 && errno != EINTR)
                    {
                        break;
                    }
                if (count > 0 && !d_received_gga.load())
                    {
                        upstream.append(reinterpret_cast<const char*>(discard),
                            static_cast<std::size_t>(count));
                        if (upstream.find("$GPGGA") != std::string::npos)
                            {
                                d_received_gga.store(true);
                            }
                        // a GGA sentence is under 128 bytes: keep only enough
                        // to bridge a split across recv() boundaries
                        if (upstream.size() > 512)
                            {
                                upstream.erase(0, upstream.size() - 128);
                            }
                    }
            }
        shutdown(client_socket, SHUT_RDWR);
        close(client_socket);
    }

    void receive_request(int client_socket, std::string* request)
    {
        const std::chrono::steady_clock::time_point deadline =
            std::chrono::steady_clock::now() +
            std::chrono::milliseconds(SERVER_TIMEOUT_MS);
        char buffer[1024];
        while (!d_stop_requested.load() && request->size() < 8192 &&
               request->find("\r\n\r\n") == std::string::npos &&
               std::chrono::steady_clock::now() < deadline)
            {
                if (!wait_readable(client_socket, SOCKET_WAIT_MS))
                    {
                        continue;
                    }
                const ssize_t count = recv(client_socket, buffer, sizeof(buffer), 0);
                if (count <= 0)
                    {
                        return;
                    }
                request->append(buffer, static_cast<std::size_t>(count));
            }
    }

    int d_listen_socket = -1;
    std::uint16_t d_port = 0;
    std::thread d_worker;
    std::atomic<bool> d_stop_requested{false};
    std::string d_request;
    std::vector<std::string> d_requests;
    std::size_t d_accepted_connection_count = 0;
    bool d_accepted_client = false;
    bool d_sent_fragmented_response = false;
    bool d_sent_coalesced_payload = false;
    bool d_peer_closed_cleanly = false;
    Loopback_Caster_Close_Mode d_close_mode = Loopback_Caster_Close_Mode::KEEP_OPEN;
    Loopback_Caster_Response_Mode d_response_mode = Loopback_Caster_Response_Mode::V1_ICY;
    bool d_server_closed_after_payload = false;
    bool d_server_closed_on_accept = false;
    bool d_received_first_v2_request = false;
    std::atomic<bool> d_received_gga{false};
    std::string d_start_failure;
    int d_start_errno = 0;
};


class Rtklib_Trace_Capture
{
public:
    ~Rtklib_Trace_Capture()
    {
        if (d_active)
            {
                tracelevel(0);
                traceclose();
            }
        if (!d_path.empty())
            {
                unlink(d_path.c_str());
            }
    }

    Rtklib_Trace_Capture(const Rtklib_Trace_Capture&) = delete;
    Rtklib_Trace_Capture& operator=(const Rtklib_Trace_Capture&) = delete;

    Rtklib_Trace_Capture() = default;

    bool start()
    {
        char path_template[] = "/tmp/gnss_sdr_ntrip_trace_XXXXXX";
        const int descriptor = mkstemp(path_template);
        if (descriptor < 0)
            {
                return false;
            }
        close(descriptor);
        d_path = path_template;
        traceopen(d_path.c_str());
        tracelevel(5);
        d_active = true;
        return true;
    }

    std::string finish()
    {
        if (d_active)
            {
                tracelevel(0);
                traceclose();
                d_active = false;
            }
        std::ifstream input(d_path.c_str(), std::ios::binary);
        std::ostringstream contents;
        contents << input.rdbuf();
        return contents.str();
    }

private:
    std::string d_path;
    bool d_active = false;
};

}  // namespace ntrip_rtcm_client_test


TEST(NtripRtcmClientTest, V2ResponseAcceptsGnssDataContentTypeAndPayload)
{
    using namespace ntrip_rtcm_client_test;

    const V2_Response_Result response = parse_v2_response(
        "HTTP/1.1 200 OK\r\n"
        "Ntrip-Version: Ntrip/2.0\r\n"
        "Content-Type: GnSs/DaTa; source=BASE\r\n"
        "\r\n"
        "RTCM");

    EXPECT_EQ(1, response.parsed) << response.message;
    EXPECT_EQ(2, response.state);
    EXPECT_EQ(2, response.transport_state);
    EXPECT_EQ(0, response.chunked);
    EXPECT_EQ("RTCM", response.payload);
}


TEST(NtripRtcmClientTest, V2ResponseWithoutContentTypeRemainsCompatible)
{
    using namespace ntrip_rtcm_client_test;

    const V2_Response_Result response = parse_v2_response(
        "HTTP/1.1 200 OK\r\n"
        "Ntrip-Version: Ntrip/2.0\r\n"
        "\r\n");

    EXPECT_EQ(1, response.parsed) << response.message;
    EXPECT_EQ(2, response.state);
    EXPECT_EQ(2, response.transport_state);
}


TEST(NtripRtcmClientTest, V2ResponseRejectsExplicitNonCorrectionContentTypes)
{
    using namespace ntrip_rtcm_client_test;

    // a sourcetable answer to a mountpoint request means the mountpoint does
    // not exist, and a textual document is an error page served with status
    // 200 — neither is a correction stream
    const char* const rejected_content_types[] = {
        "gnss/sourcetable",
        "text/html; charset=utf-8",
        "text/plain",
        "application/json"};
    for (const char* const content_type : rejected_content_types)
        {
            SCOPED_TRACE(content_type);
            const V2_Response_Result response = parse_v2_response(
                std::string("HTTP/1.1 200 OK\r\nContent-Type: ") +
                content_type + "\r\n\r\nnot corrections");
            EXPECT_EQ(0, response.parsed);
            EXPECT_EQ(0, response.state);
            EXPECT_EQ(0, response.transport_state);
            EXPECT_EQ(0, response.chunked);
            EXPECT_EQ(0, response.chunk_state);
            EXPECT_EQ(0U, response.chunk_remaining);
            EXPECT_TRUE(response.payload.empty());
        }
}


TEST(NtripRtcmClientTest, V2ResponseToleratesNonStandardBinaryContentTypes)
{
    using namespace ntrip_rtcm_client_test;

    // NTRIP 2.0 mandates gnss/data, but deployed casters also label their
    // correction streams with generic binary types: rejecting them would loop
    // the client through reject-reconnect forever with no recovery path,
    // while the RTCM decoder validates every frame anyway
    const char* const tolerated_content_types[] = {
        "application/octet-stream",
        "application/rtcm3"};
    for (const char* const content_type : tolerated_content_types)
        {
            SCOPED_TRACE(content_type);
            const V2_Response_Result response = parse_v2_response(
                std::string("HTTP/1.1 200 OK\r\nContent-Type: ") +
                content_type + "\r\n\r\nRTCM");
            EXPECT_EQ(1, response.parsed) << response.message;
            EXPECT_EQ(2, response.state);
            EXPECT_EQ(2, response.transport_state);
            EXPECT_EQ("RTCM", response.payload);
        }
}


TEST(NtripRtcmClientTest, V2ResponseAcceptsSourcetableForEmptyMountpoint)
{
    using namespace ntrip_rtcm_client_test;

    const V2_Response_Result response = parse_v2_response(
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: gnss/sourcetable\r\n"
        "\r\n"
        "ENDSOURCETABLE\r\n",
        "");

    EXPECT_EQ(1, response.parsed) << response.message;
    EXPECT_EQ(2, response.state);
    EXPECT_EQ(2, response.transport_state);
    EXPECT_EQ(0, response.chunked);
    EXPECT_EQ("ENDSOURCETABLE\r\n", response.payload);
}


TEST(NtripRtcmClientTest, V2ResponseRejectsDuplicateSemanticHeaders)
{
    using namespace ntrip_rtcm_client_test;

    const char* const duplicate_headers[] = {
        "Content-Type: gnss/data\r\n"
        "Content-Type: gnss/data\r\n",
        "Content-Type: gnss/data\r\n"
        "Transfer-Encoding: chunked\r\n"
        "Transfer-Encoding: chunked\r\n"};
    for (const char* const headers : duplicate_headers)
        {
            SCOPED_TRACE(headers);
            const V2_Response_Result response = parse_v2_response(
                std::string("HTTP/1.1 200 OK\r\n") + headers + "\r\n");
            EXPECT_EQ(0, response.parsed);
            EXPECT_EQ(0, response.state);
            EXPECT_EQ(0, response.transport_state);
            EXPECT_EQ(0, response.chunked);
            EXPECT_EQ(0, response.chunk_state);
            EXPECT_EQ(0U, response.chunk_remaining);
            EXPECT_TRUE(response.payload.empty());
        }
}


TEST(NtripRtcmClientTest, V2ResponseDoesNotMatchChunkedAsASubstring)
{
    using namespace ntrip_rtcm_client_test;

    const V2_Response_Result response = parse_v2_response(
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: gnss/data\r\n"
        "Transfer-Encoding: notchunked\r\n"
        "\r\n");

    EXPECT_EQ(0, response.parsed);
    EXPECT_EQ(0, response.state);
    EXPECT_EQ(0, response.transport_state);
    EXPECT_EQ(0, response.chunked);
    EXPECT_EQ(0, response.chunk_state);
    EXPECT_EQ(0U, response.chunk_remaining);
    EXPECT_TRUE(response.payload.empty());
}


TEST(NtripRtcmClientTest, V2ResponseRejectsUnsupportedTransferCodingChain)
{
    using namespace ntrip_rtcm_client_test;

    const char* const rejected_encodings[] = {
        "gzip, chunked",
        "chunked, chunked"};
    for (const char* const transfer_encoding : rejected_encodings)
        {
            SCOPED_TRACE(transfer_encoding);
            const V2_Response_Result response = parse_v2_response(
                std::string("HTTP/1.1 200 OK\r\n") +
                "Content-Type: gnss/data\r\n" +
                "Transfer-Encoding: " + transfer_encoding +
                "\r\n\r\n");
            EXPECT_EQ(0, response.parsed);
            EXPECT_EQ(0, response.state);
            EXPECT_EQ(0, response.transport_state);
            EXPECT_EQ(0, response.chunked);
            EXPECT_EQ(0, response.chunk_state);
            EXPECT_EQ(0U, response.chunk_remaining);
            EXPECT_TRUE(response.payload.empty());
        }
}


TEST(NtripRtcmClientTest, V2ResponseMatchesChunkedTokenCaseInsensitively)
{
    using namespace ntrip_rtcm_client_test;

    const V2_Response_Result response = parse_v2_response(
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: gnss/data\r\n"
        "Transfer-Encoding: \tChUnKeD \t\r\n"
        "\r\n");

    EXPECT_EQ(1, response.parsed) << response.message;
    EXPECT_EQ(2, response.state);
    EXPECT_EQ(1, response.chunked);
}


TEST(NtripRtcmClientTest, V2ResponseRequestsV1RetryOnlyForCompatibilityStatuses)
{
    using namespace ntrip_rtcm_client_test;

    const int compatibility_statuses[] = {400, 501, 505};
    for (const int status : compatibility_statuses)
        {
            SCOPED_TRACE(status);
            const V2_Response_Result response = parse_v2_response(
                std::string("HTTP/1.1 ") + std::to_string(status) +
                " Compatibility Failure\r\n\r\n");
            EXPECT_EQ(0, response.parsed);
            EXPECT_EQ(1, response.v1_retry_requested);
        }

    const int non_compatibility_statuses[] = {301, 401, 403, 404, 429, 500, 503};
    for (const int status : non_compatibility_statuses)
        {
            SCOPED_TRACE(status);
            const V2_Response_Result response = parse_v2_response(
                std::string("HTTP/1.1 ") + std::to_string(status) +
                " Request Failure\r\n\r\n");
            EXPECT_EQ(0, response.parsed);
            EXPECT_EQ(0, response.v1_retry_requested);
        }
}


TEST(NtripRtcmClientTest, PartialRequestWritesResumeWithoutDuplicatingBytes)
{
    using namespace ntrip_rtcm_client_test;

    Socket_Pair_Guard sockets;
    ASSERT_TRUE(sockets.open_pair());
    const int send_buffer_size = 1024;
    ASSERT_EQ(0, setsockopt(sockets.sender(), SOL_SOCKET, SO_SNDBUF,
                     &send_buffer_size, sizeof(send_buffer_size)));
#if defined(SO_SNDLOWAT)
    const int send_low_water_mark = 1;
    // Linux fixes this value at one and returns ENOPROTOOPT when setting it;
    // systems that permit changing it should expose every newly freed byte.
    setsockopt(sockets.sender(), SOL_SOCKET, SO_SNDLOWAT,
        &send_low_water_mark, sizeof(send_low_water_mark));
#endif
    ASSERT_TRUE(set_nonblocking(sockets.sender()));
    ASSERT_TRUE(set_nonblocking(sockets.receiver()));

    const std::size_t filler_size = fill_socket_send_buffer(sockets.sender());
    ASSERT_GT(filler_size, 0U);

    tcpcli_t tcp_client = {};
    tcp_client.svr.state = 2;
    tcp_client.svr.sock = sockets.sender();

    ntrip_t ntrip = {};
    ntrip.state = 0;
    ntrip.type = 1;
    ntrip.version = NTRIP_VERSION_2;
    ntrip.tcp = &tcp_client;
    ntrip.request_length = NTRIP_MAXRSP / 2;
    for (int i = 0; i < ntrip.request_length; ++i)
        {
            ntrip.request_buff[i] = static_cast<unsigned char>('A' + i % 26);
        }
    const std::string expected_request(
        reinterpret_cast<char*>(ntrip.request_buff),
        static_cast<std::size_t>(ntrip.request_length));

    char message[MAXSTRMSG] = "";
    EXPECT_EQ(0, reqntrip_c(&ntrip, message));
    EXPECT_EQ(0, ntrip.state);
    EXPECT_EQ(0, ntrip.request_offset);

    EXPECT_EQ(filler_size,
        receive_available(sockets.receiver(), filler_size, nullptr));

    EXPECT_EQ(0, reqntrip_c(&ntrip, message));
    ASSERT_GT(ntrip.request_offset, 0);
    ASSERT_LT(ntrip.request_offset, ntrip.request_length);

    std::string received_request;
    const std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
    while (ntrip.state == 0 && std::chrono::steady_clock::now() < deadline)
        {
            receive_available(sockets.receiver(), NTRIP_MAXRSP,
                &received_request);
            reqntrip_c(&ntrip, message);
        }
    receive_available(sockets.receiver(), NTRIP_MAXRSP, &received_request);

    EXPECT_EQ(1, ntrip.state) << message;
    EXPECT_EQ(0, ntrip.request_length);
    EXPECT_EQ(0, ntrip.request_offset);
    EXPECT_EQ(expected_request, received_request);
}


TEST(NtripRtcmClientTest, MaximumSizeChunkMakesProgressWithoutNarrowing)
{
    using namespace ntrip_rtcm_client_test;

#if GTEST_HAS_DEATH_TEST
    ASSERT_EXIT(
        {
            alarm(2);
            _exit(read_maximum_size_chunk());
        },
        ::testing::ExitedWithCode(0), "");
#else
    EXPECT_EQ(0, read_maximum_size_chunk());
#endif
}


#if GTEST_HAS_DEATH_TEST
TEST(NtripRtcmClientTest, TlsHandshakeWithClosedPeerDoesNotRaiseSigpipe)
{
    using namespace ntrip_rtcm_client_test;

    ASSERT_EXIT(
        {
            alarm(2);
            _exit(handshake_with_closed_tls_peer());
        },
        ::testing::ExitedWithCode(0), "");
}
#endif


TEST(NtripRtcmClientTest, ClosingUnopenedTransportPreservesDescriptorZero)
{
    using namespace ntrip_rtcm_client_test;

    File_Descriptor_Zero_Guard descriptor_guard;
    ASSERT_TRUE(descriptor_guard.ready());

    char message[MAXSTRMSG] = "";
    tcpcli_t* tcp_client = opentcpcli("127.0.0.1:2101", message);
    ASSERT_NE(nullptr, tcp_client);
    EXPECT_EQ(-1, tcp_client->svr.sock);

    closetcpcli(tcp_client);
    EXPECT_GE(fcntl(STDIN_FILENO, F_GETFD), 0);
}


TEST(NtripRtcmClientTest, GeneratedObservationFixtureIsValidRtcm3)
{
    using namespace ntrip_rtcm_client_test;
    rtcm_t decoder = {};
    ASSERT_EQ(1, init_rtcm(&decoder));
    decoder.time = gpst2time(TEST_GPS_WEEK, TEST_TOW_S);

    int decode_result = 0;
    const std::vector<unsigned char> frame = make_mt1004();
    for (const unsigned char byte : frame)
        {
            decode_result = input_rtcm3(&decoder, byte);
        }

    EXPECT_EQ(1, decode_result);
    EXPECT_EQ(2, decoder.obs.n);
    if (decoder.obs.n >= 2)
        {
            EXPECT_EQ(SYS_GPS, satsys(decoder.obs.data[0].sat, nullptr));
            EXPECT_EQ(SYS_SBS, satsys(decoder.obs.data[1].sat, nullptr));
        }
    free_rtcm(&decoder);
}


TEST(NtripRtcmClientTest, GeneratedMsm4FixtureIsValidDualBandRtcm3)
{
    using namespace ntrip_rtcm_client_test;
    rtcm_t decoder = {};
    ASSERT_EQ(1, init_rtcm(&decoder));
    decoder.time = gpst2time(TEST_GPS_WEEK, TEST_TOW_S);

    int decode_result = 0;
    const std::vector<unsigned char> frame = make_mt1074();
    for (const unsigned char byte : frame)
        {
            decode_result = input_rtcm3(&decoder, byte);
        }

    ASSERT_EQ(1, decode_result);
    ASSERT_EQ(1, decoder.obs.n);
    EXPECT_EQ(SYS_GPS, satsys(decoder.obs.data[0].sat, nullptr));
    EXPECT_EQ(CODE_L1C, decoder.obs.data[0].code[0]);
    EXPECT_EQ(CODE_L2S, decoder.obs.data[0].code[1]);
    EXPECT_NE(0.0, decoder.obs.data[0].P[0]);
    EXPECT_NE(0.0, decoder.obs.data[0].P[1]);
    EXPECT_NE(0.0, decoder.obs.data[0].L[0]);
    EXPECT_NE(0.0, decoder.obs.data[0].L[1]);
    free_rtcm(&decoder);
}


TEST(NtripRtcmClientTest, GeneratedBeidouMsm4PrefersB1cOverB1iInTheSharedSlot)
{
    using namespace ntrip_rtcm_client_test;
    rtcm_t decoder = {};
    ASSERT_EQ(1, init_rtcm(&decoder));
    decoder.time = gpst2time(TEST_GPS_WEEK, TEST_TOW_S);

    int decode_result = 0;
    const std::vector<unsigned char> frame = make_mt1124();
    for (const unsigned char byte : frame)
        {
            decode_result = input_rtcm3(&decoder, byte);
        }

    ASSERT_EQ(1, decode_result);
    ASSERT_EQ(1, decoder.obs.n);
    int prn = 0;
    EXPECT_EQ(SYS_BDS, satsys(decoder.obs.data[0].sat, &prn));
    EXPECT_EQ(19, prn);
    // Both signals target the first slot; the code priority table must
    // resolve the contention in favor of the B1C pilot
    EXPECT_EQ(CODE_L1P, decoder.obs.data[0].code[0]);
    EXPECT_NE(0.0, decoder.obs.data[0].P[0]);
    EXPECT_NE(0.0, decoder.obs.data[0].L[0]);
    free_rtcm(&decoder);
}


TEST(NtripRtcmClientTest, Msm4HalfCycleAmbiguityDoesNotFlagACycleSlip)
{
    using namespace ntrip_rtcm_client_test;
    rtcm_t decoder = {};
    ASSERT_EQ(1, init_rtcm(&decoder));
    decoder.time = gpst2time(TEST_GPS_WEEK, TEST_TOW_S);

    int decode_result = 0;
    const std::vector<unsigned char> frame = make_mt1074(true);
    for (const unsigned char byte : frame)
        {
            decode_result = input_rtcm3(&decoder, byte);
        }

    ASSERT_EQ(1, decode_result);
    ASSERT_EQ(1, decoder.obs.n);

    // An unresolved half-cycle ambiguity sets bit 1 only. Reporting bit 0 as
    // well would make detslp_ll() declare a cycle slip on every epoch the base
    // keeps the flag raised, resetting the ambiguity and preventing a fix.
    EXPECT_EQ(2, decoder.obs.data[0].LLI[0]);
    EXPECT_EQ(2, decoder.obs.data[0].LLI[1]);
    free_rtcm(&decoder);
}


TEST(NtripRtcmClientTest, Msm4LossOfLockAndHalfCycleAmbiguityKeepSeparateBits)
{
    using namespace ntrip_rtcm_client_test;
    rtcm_t decoder = {};
    ASSERT_EQ(1, init_rtcm(&decoder));
    decoder.time = gpst2time(TEST_GPS_WEEK, TEST_TOW_S);

    int decode_result = 0;
    const std::vector<unsigned char> locked = make_mt1074(false, 9, TEST_TOW_S);
    for (const unsigned char byte : locked)
        {
            decode_result = input_rtcm3(&decoder, byte);
        }
    ASSERT_EQ(1, decode_result);
    ASSERT_EQ(1, decoder.obs.n);
    EXPECT_EQ(0, decoder.obs.data[0].LLI[0]);

    // Lock indicator decreases (loss of lock) while the half-cycle ambiguity is
    // unresolved: both flags must survive, one per bit. Adding them as 1 + 3
    // would yield 4, silently dropping the slip.
    const std::vector<unsigned char> reacquired = make_mt1074(true, 1, TEST_TOW_S + 1.0);
    for (const unsigned char byte : reacquired)
        {
            decode_result = input_rtcm3(&decoder, byte);
        }
    ASSERT_EQ(1, decode_result);
    ASSERT_EQ(1, decoder.obs.n);
    EXPECT_EQ(3, decoder.obs.data[0].LLI[0]);
    EXPECT_EQ(3, decoder.obs.data[0].LLI[1]);
    free_rtcm(&decoder);
}


TEST(NtripRtcmClientTest, DisabledAndInvalidConfigurationsDoNotStartAWorker)
{
    Ntrip_Rtcm_Client_Config disabled_config;
    Ntrip_Rtcm_Client disabled_client(disabled_config);

    EXPECT_FALSE(disabled_client.running());
    EXPECT_FALSE(disabled_client.start());
    EXPECT_FALSE(disabled_client.running());
    EXPECT_EQ(Ntrip_Rtcm_Client_State::DISABLED,
        disabled_client.status().state);
    EXPECT_FALSE(disabled_client.latest_snapshot().has_observations);
    disabled_client.stop();
    EXPECT_EQ(Ntrip_Rtcm_Client_State::DISABLED,
        disabled_client.status().state);

    Ntrip_Rtcm_Client_Config invalid_config;
    invalid_config.enabled = true;
    invalid_config.mountpoint = "BASE";
    invalid_config.username = "test-user";
    invalid_config.password = "test-secret";
    Ntrip_Rtcm_Client invalid_client(invalid_config);

    EXPECT_FALSE(invalid_client.start());
    const Ntrip_Rtcm_Client_Status invalid_status = invalid_client.status();
    EXPECT_EQ(Ntrip_Rtcm_Client_State::ERROR, invalid_status.state);
    EXPECT_FALSE(invalid_status.running);
    EXPECT_FALSE(invalid_status.connected);
    EXPECT_EQ(std::string::npos, invalid_status.message.find("test-user"));
    EXPECT_EQ(std::string::npos, invalid_status.message.find("test-secret"));
    invalid_client.stop();
    EXPECT_FALSE(invalid_client.running());
}


TEST(NtripRtcmClientTest, TimedOutDnsLookupRemainsSingleFlightAcrossReconnects)
{
    using namespace ntrip_rtcm_client_test;

    Ntrip_Rtcm_Client_Config config;
    config.enabled = true;
    config.host = "stalled-resolver.invalid";
    config.mountpoint = "BASE";
    config.reconnect_interval_ms = 1000;
    config.timeout_ms = 1000;

    Blocking_Hostname_Resolver resolver;
    Ntrip_Rtcm_Client client(config);
    ASSERT_TRUE(Ntrip_Rtcm_Client_Test_Access::set_hostname_resolver(
        client, resolver.callback()));
    ASSERT_TRUE(client.start());
    ASSERT_TRUE(resolver.wait_for_calls(1, std::chrono::milliseconds(2000)));

    Ntrip_Rtcm_Client_Status retry_status;
    const std::chrono::steady_clock::time_point retry_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(4500);
    do
        {
            retry_status = client.status();
            if (retry_status.reconnect_count >= 2 &&
                retry_status.state == Ntrip_Rtcm_Client_State::RECONNECT_WAIT)
                {
                    break;
                }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    while (std::chrono::steady_clock::now() < retry_deadline);

    const std::chrono::steady_clock::time_point stop_start =
        std::chrono::steady_clock::now();
    client.stop();
    const std::chrono::milliseconds stop_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - stop_start);
    // No resolver can be launched after stop(). Give any thread already
    // created by the faulty path a final chance to enter its callback before
    // taking the invocation count used by the assertion and cleanup wait.
    resolver.wait_for_calls(2, std::chrono::milliseconds(250));
    const std::size_t resolver_calls = resolver.call_count();
    resolver.release();
    const bool all_callbacks_completed = resolver.wait_for_completions(
        resolver_calls, std::chrono::milliseconds(1000));

    ASSERT_GE(retry_status.reconnect_count, 2U);
    ASSERT_EQ(Ntrip_Rtcm_Client_State::RECONNECT_WAIT, retry_status.state);
    EXPECT_EQ(1U, resolver_calls);
    EXPECT_LT(stop_duration.count(), 1000);
    EXPECT_TRUE(all_callbacks_completed);
}


TEST(NtripRtcmClientTest, AuthenticatedFragmentedStreamPublishesFixedBaseSnapshot)
{
    using namespace ntrip_rtcm_client_test;

    Loopback_Ntrip_Caster caster;
    ASSERT_TRUE(caster.start()) << caster.start_failure() << ": "
                                << std::strerror(caster.start_errno());
    ASSERT_NE(0, caster.port());

    Rtklib_Trace_Capture trace_capture;
    ASSERT_TRUE(trace_capture.start());

    Ntrip_Rtcm_Client_Config config;
    config.enabled = true;
    config.host = "localhost";
    config.port = caster.port();
    config.mountpoint = "/BASE";
    config.username = TRACE_USERNAME;
    config.password = TRACE_PASSWORD;
    config.reconnect_interval_ms = 1000;
    config.timeout_ms = 1000;
    config.max_age_s = 1.0;
    config.station_id = TEST_STATION_ID;

    const gtime_t rover_time = gpst2time(TEST_GPS_WEEK, TEST_TOW_S);
    Ntrip_Rtcm_Client client(config);
    client.update_rover_time(rover_time);
    ASSERT_TRUE(client.start());

    Ntrip_Rtcm_Snapshot snapshot;
    const std::chrono::steady_clock::time_point snapshot_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
    do
        {
            snapshot = client.latest_snapshot(rover_time);
            if (snapshot.has_base_position && snapshot.has_observations)
                {
                    break;
                }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    while (std::chrono::steady_clock::now() < snapshot_deadline);

    const Ntrip_Rtcm_Client_Status streaming_status = client.status();
    const std::chrono::steady_clock::time_point stop_start =
        std::chrono::steady_clock::now();
    client.stop();
    const std::chrono::milliseconds stop_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - stop_start);
    caster.join();
    const std::string rtklib_trace = trace_capture.finish();

    ASSERT_TRUE(snapshot.has_base_position);
    ASSERT_TRUE(snapshot.has_observations);
    // The same-TOW epoch merges the dual-band satellite (PRN 3, from 1004 and
    // MSM4) with the single-band satellite of the extended L1-only message
    // (PRN 8), kept since single-frequency RTK support
    ASSERT_EQ(2U, snapshot.observations.size());
    EXPECT_EQ(TEST_STATION_ID, snapshot.station_id);
    EXPECT_EQ(1006, snapshot.base_position_message_type);
    EXPECT_NEAR(1114104.5999, snapshot.base_position_ecef_m[0], 1e-4);
    EXPECT_NEAR(-4850729.7108, snapshot.base_position_ecef_m[1], 1e-4);
    EXPECT_NEAR(3975521.4643, snapshot.base_position_ecef_m[2], 1e-4);
    EXPECT_DOUBLE_EQ(0.0, snapshot.antenna_height_m);

    const obsd_t* dual_band_observation = nullptr;
    const obsd_t* single_band_observation = nullptr;
    for (const obsd_t& observation : snapshot.observations)
        {
            int prn = 0;
            EXPECT_EQ(SYS_GPS, satsys(observation.sat, &prn));
            EXPECT_EQ(2, observation.rcv);
            if (prn == 3)
                {
                    dual_band_observation = &observation;
                }
            else
                {
                    EXPECT_EQ(8, prn);
                    single_band_observation = &observation;
                }
        }
    ASSERT_NE(nullptr, dual_band_observation);
    ASSERT_NE(nullptr, single_band_observation);
    EXPECT_EQ(CODE_L1C, dual_band_observation->code[0]);
    EXPECT_EQ(CODE_L2S, dual_band_observation->code[1]);
    EXPECT_EQ(CODE_NONE, dual_band_observation->code[2]);
    EXPECT_GT(dual_band_observation->P[0], 0.0);
    EXPECT_GT(dual_band_observation->P[1], 0.0);
    EXPECT_DOUBLE_EQ(0.0, dual_band_observation->P[2]);
    EXPECT_DOUBLE_EQ(0.0, dual_band_observation->L[2]);
    EXPECT_NEAR(0.0, timediff(dual_band_observation->time, rover_time), 1e-9);
    EXPECT_EQ(CODE_L1C, single_band_observation->code[0]);
    EXPECT_EQ(CODE_NONE, single_band_observation->code[1]);
    EXPECT_EQ(CODE_NONE, single_band_observation->code[2]);
    EXPECT_GT(single_band_observation->P[0], 0.0);
    EXPECT_TRUE(snapshot.fresh);
    EXPECT_NEAR(0.0, snapshot.age_s, 1e-9);
    EXPECT_GT(snapshot.generation, 0U);
    EXPECT_EQ(snapshot.generation, client.snapshot_generation());

    const Ntrip_Rtcm_Snapshot slightly_future_snapshot =
        client.latest_snapshot(timeadd(rover_time, -0.5));
    EXPECT_TRUE(slightly_future_snapshot.fresh);
    EXPECT_NEAR(-0.5, slightly_future_snapshot.age_s, 1e-9);
    const Ntrip_Rtcm_Snapshot stale_snapshot =
        client.latest_snapshot(timeadd(rover_time, 1.5));
    EXPECT_FALSE(stale_snapshot.fresh);
    EXPECT_NEAR(1.5, stale_snapshot.age_s, 1e-9);

    EXPECT_EQ(Ntrip_Rtcm_Client_State::STREAMING, streaming_status.state);
    EXPECT_TRUE(streaming_status.running);
    EXPECT_TRUE(streaming_status.connected);
    EXPECT_GE(streaming_status.rtcm_messages_decoded, 5U);
    // 1004, the single-band 1002, and the MSM4 epoch all count
    EXPECT_EQ(3U, streaming_status.observation_epochs_decoded);
    EXPECT_GE(streaming_status.decoder_errors, 1U);
    EXPECT_FALSE(client.running());
    EXPECT_EQ(Ntrip_Rtcm_Client_State::STOPPED, client.status().state);
    EXPECT_LT(stop_duration.count(), 1000);

    EXPECT_TRUE(caster.accepted_client());
    EXPECT_TRUE(caster.sent_fragmented_response());
    EXPECT_TRUE(caster.sent_coalesced_payload());
    EXPECT_TRUE(caster.peer_closed_cleanly());
    EXPECT_EQ(0U, caster.request().find("GET /BASE HTTP/1.1\r\n"));
    EXPECT_NE(std::string::npos, caster.request().find("Host: localhost:"));
    EXPECT_NE(std::string::npos,
        caster.request().find("Ntrip-Version: Ntrip/2.0\r\n"));
    EXPECT_NE(std::string::npos,
        caster.request().find(std::string("Authorization: Basic ") +
                              TRACE_BASIC_TOKEN + "\r\n"));
    EXPECT_EQ(std::string::npos,
        caster.request().find(std::string(TRACE_USERNAME) + ':' +
                              TRACE_PASSWORD + '@'));

    ASSERT_FALSE(rtklib_trace.empty());
    EXPECT_NE(std::string::npos, rtklib_trace.find("stropen:"));
    EXPECT_EQ(std::string::npos, rtklib_trace.find(TRACE_PASSWORD));
    EXPECT_EQ(std::string::npos, rtklib_trace.find(TRACE_BASIC_TOKEN));
}


TEST(NtripRtcmClientTest, RepeatedBasePositionsDoNotAdvanceSnapshotGeneration)
{
    using namespace ntrip_rtcm_client_test;

    Loopback_Ntrip_Caster caster(Loopback_Caster_Close_Mode::AFTER_PAYLOAD,
        Loopback_Caster_Response_Mode::REPEATED_BASE_POSITIONS);
    ASSERT_TRUE(caster.start()) << caster.start_failure() << ": "
                                << std::strerror(caster.start_errno());

    Ntrip_Rtcm_Client_Config config;
    config.enabled = true;
    config.host = "127.0.0.1";
    config.port = caster.port();
    config.mountpoint = "/BASE";
    config.version = 1;
    config.reconnect_interval_ms = 0;
    config.timeout_ms = 1000;
    config.station_id = TEST_STATION_ID;

    Ntrip_Rtcm_Client client(config);
    const std::uint64_t generation_before_start = client.snapshot_generation();
    ASSERT_TRUE(client.start());

    const std::chrono::steady_clock::time_point stopped_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
    while (client.running() &&
           std::chrono::steady_clock::now() < stopped_deadline)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

    const Ntrip_Rtcm_Client_Status final_status = client.status();
    const Ntrip_Rtcm_Snapshot snapshot = client.latest_snapshot();
    client.stop();
    caster.join();

    ASSERT_FALSE(final_status.running);
    ASSERT_EQ(4U, final_status.rtcm_messages_decoded);
    ASSERT_TRUE(snapshot.has_base_position);
    EXPECT_EQ(1006, snapshot.base_position_message_type);
    // One bump resets the output at start, one stores the first 1005, and one
    // publishes the visible 1005-to-1006 message-type change. The repeated
    // 1005 and 1006 frames describe identical snapshots and add no bumps.
    EXPECT_EQ(generation_before_start + 3U, snapshot.generation);
    EXPECT_EQ(snapshot.generation, client.snapshot_generation());
    EXPECT_TRUE(caster.sent_coalesced_payload());
    EXPECT_TRUE(caster.server_closed_after_payload());
}


TEST(NtripRtcmClientTest, RoverGgaReachesTheCasterWhenEnabled)
{
    using namespace ntrip_rtcm_client_test;

    Loopback_Ntrip_Caster caster;
    ASSERT_TRUE(caster.start()) << caster.start_failure() << ": "
                                << std::strerror(caster.start_errno());
    ASSERT_NE(0, caster.port());

    Ntrip_Rtcm_Client_Config config;
    config.enabled = true;
    config.host = "localhost";
    config.port = caster.port();
    config.mountpoint = "/BASE";
    config.username = TRACE_USERNAME;
    config.password = TRACE_PASSWORD;
    config.reconnect_interval_ms = 1000;
    config.timeout_ms = 1000;
    config.max_age_s = 1.0;
    config.station_id = TEST_STATION_ID;
    config.send_gga = true;
    config.gga_period_ms = 1000;

    Ntrip_Rtcm_Client client(config);
    // A VRS caster serves no corrections until it receives the rover GGA,
    // so the position must be known before the stream connects
    client.update_rover_position({{4797642.0, 166322.0, 4185504.0}});
    ASSERT_TRUE(client.start());

    const std::chrono::steady_clock::time_point gga_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
    while (!caster.received_gga() &&
           std::chrono::steady_clock::now() < gga_deadline)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    client.stop();
    caster.join();

    EXPECT_TRUE(caster.accepted_client());
    EXPECT_TRUE(caster.received_gga());
}


TEST(NtripRtcmClientTest, NtripV2ChunkedStreamPublishesFixedBaseSnapshot)
{
    using namespace ntrip_rtcm_client_test;

    Loopback_Ntrip_Caster caster(Loopback_Caster_Close_Mode::KEEP_OPEN,
        Loopback_Caster_Response_Mode::V2_HTTP_CHUNKED);
    ASSERT_TRUE(caster.start()) << caster.start_failure() << ": "
                                << std::strerror(caster.start_errno());
    ASSERT_NE(0, caster.port());

    Ntrip_Rtcm_Client_Config config;
    config.enabled = true;
    config.host = "localhost";
    config.port = caster.port();
    config.mountpoint = "/BASE";
    config.username = TRACE_USERNAME;
    config.password = TRACE_PASSWORD;
    config.reconnect_interval_ms = 1000;
    config.timeout_ms = 1000;
    config.max_age_s = 1.0;
    config.station_id = TEST_STATION_ID;

    const gtime_t rover_time = gpst2time(TEST_GPS_WEEK, TEST_TOW_S);
    Ntrip_Rtcm_Client client(config);
    client.update_rover_time(rover_time);
    ASSERT_TRUE(client.start());

    Ntrip_Rtcm_Snapshot snapshot;
    const std::chrono::steady_clock::time_point snapshot_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
    do
        {
            snapshot = client.latest_snapshot(rover_time);
            if (snapshot.has_base_position && snapshot.has_observations)
                {
                    break;
                }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    while (std::chrono::steady_clock::now() < snapshot_deadline);

    const Ntrip_Rtcm_Client_Status streaming_status = client.status();
    client.stop();
    caster.join();

    EXPECT_TRUE(caster.sent_fragmented_response());
    EXPECT_TRUE(caster.sent_coalesced_payload());
    EXPECT_EQ(0U, caster.request().find("GET /BASE HTTP/1.1\r\n"));
    EXPECT_NE(std::string::npos,
        caster.request().find("Ntrip-Version: Ntrip/2.0\r\n"));

    ASSERT_TRUE(snapshot.has_base_position);
    ASSERT_TRUE(snapshot.has_observations);
    // dual-band PRN 3 plus the single-band PRN 8 from the L1-only message
    ASSERT_EQ(2U, snapshot.observations.size());
    EXPECT_EQ(TEST_STATION_ID, snapshot.station_id);
    EXPECT_EQ(1006, snapshot.base_position_message_type);
    EXPECT_NEAR(1114104.5999, snapshot.base_position_ecef_m[0], 1e-4);
    EXPECT_NEAR(-4850729.7108, snapshot.base_position_ecef_m[1], 1e-4);
    EXPECT_NEAR(3975521.4643, snapshot.base_position_ecef_m[2], 1e-4);
    EXPECT_EQ(Ntrip_Rtcm_Client_State::STREAMING, streaming_status.state);
    // 1004, the single-band 1002, and the MSM4 epoch all count
    EXPECT_EQ(3U, streaming_status.observation_epochs_decoded);
}


void expect_v2_failure_to_retry_fresh_connection_as_v1(
    ntrip_rtcm_client_test::Loopback_Caster_Response_Mode response_mode)
{
    using namespace ntrip_rtcm_client_test;

    Loopback_Ntrip_Caster caster(Loopback_Caster_Close_Mode::KEEP_OPEN,
        response_mode);
    ASSERT_TRUE(caster.start()) << caster.start_failure() << ": "
                                << std::strerror(caster.start_errno());

    Ntrip_Rtcm_Client_Config config;
    config.enabled = true;
    config.host = "127.0.0.1";
    config.port = caster.port();
    config.mountpoint = "/BASE";
    config.username = TRACE_USERNAME;
    config.password = TRACE_PASSWORD;
    config.reconnect_interval_ms = 0;
    config.timeout_ms = 1000;
    config.max_age_s = 1.0;
    config.station_id = TEST_STATION_ID;

    const gtime_t rover_time = gpst2time(TEST_GPS_WEEK, TEST_TOW_S);
    Ntrip_Rtcm_Client client(config);
    client.update_rover_time(rover_time);
    ASSERT_TRUE(client.start());

    Ntrip_Rtcm_Snapshot snapshot;
    const std::chrono::steady_clock::time_point snapshot_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(2500);
    do
        {
            snapshot = client.latest_snapshot(rover_time);
            if (snapshot.has_base_position && snapshot.has_observations)
                {
                    break;
                }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    while (std::chrono::steady_clock::now() < snapshot_deadline);

    const Ntrip_Rtcm_Client_Status streaming_status = client.status();
    client.stop();
    caster.join();

    ASSERT_EQ(2U, caster.accepted_connection_count());
    ASSERT_EQ(2U, caster.requests().size());
    EXPECT_TRUE(caster.received_first_v2_request());
    EXPECT_EQ(0U, caster.requests()[0].find("GET /BASE HTTP/1.1\r\n"));
    EXPECT_NE(std::string::npos,
        caster.requests()[0].find("Ntrip-Version: Ntrip/2.0\r\n"));
    EXPECT_EQ(0U, caster.requests()[1].find("GET /BASE HTTP/1.0\r\n"));
    EXPECT_EQ(std::string::npos,
        caster.requests()[1].find("Ntrip-Version:"));
    EXPECT_EQ(0U, streaming_status.reconnect_count);
    EXPECT_EQ(Ntrip_Rtcm_Client_State::STREAMING, streaming_status.state);
    EXPECT_NE(std::string::npos,
        streaming_status.message.find("NTRIP v1 (fallback from v2)"));
    EXPECT_TRUE(snapshot.has_base_position);
    EXPECT_TRUE(snapshot.has_observations);
}


TEST(NtripRtcmClientTest, V2RejectionRetriesFreshConnectionAsV1)
{
    using namespace ntrip_rtcm_client_test;

    expect_v2_failure_to_retry_fresh_connection_as_v1(
        Loopback_Caster_Response_Mode::STRICT_V1_AFTER_V2_REJECTION);
}


TEST(NtripRtcmClientTest, V2CloseRetriesFreshConnectionAsV1)
{
    using namespace ntrip_rtcm_client_test;

    expect_v2_failure_to_retry_fresh_connection_as_v1(
        Loopback_Caster_Response_Mode::STRICT_V1_AFTER_V2_CLOSE);
}


TEST(NtripRtcmClientTest, UnconfirmedV1FallbackRestoresConfiguredV2OnReconnect)
{
    using namespace ntrip_rtcm_client_test;

    Loopback_Ntrip_Caster caster(Loopback_Caster_Close_Mode::KEEP_OPEN,
        Loopback_Caster_Response_Mode::V2_RECOVERY_AFTER_TRANSIENT_CLOSES);
    ASSERT_TRUE(caster.start()) << caster.start_failure() << ": "
                                << std::strerror(caster.start_errno());

    Ntrip_Rtcm_Client_Config config;
    config.enabled = true;
    config.host = "127.0.0.1";
    config.port = caster.port();
    config.mountpoint = "/BASE";
    config.username = TRACE_USERNAME;
    config.password = TRACE_PASSWORD;
    config.reconnect_interval_ms = 1000;
    config.timeout_ms = 1000;
    config.max_age_s = 1.0;
    config.station_id = TEST_STATION_ID;

    const gtime_t rover_time = gpst2time(TEST_GPS_WEEK, TEST_TOW_S);
    Ntrip_Rtcm_Client client(config);
    client.update_rover_time(rover_time);
    ASSERT_TRUE(client.start());

    Ntrip_Rtcm_Snapshot snapshot;
    const std::chrono::steady_clock::time_point snapshot_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(3500);
    do
        {
            snapshot = client.latest_snapshot(rover_time);
            if (snapshot.has_base_position && snapshot.has_observations)
                {
                    break;
                }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    while (std::chrono::steady_clock::now() < snapshot_deadline);

    client.stop();
    caster.join();

    ASSERT_EQ(3U, caster.accepted_connection_count());
    ASSERT_EQ(3U, caster.requests().size());
    // the v2 attempt and its immediate v1 retry both die on a zero-byte close
    EXPECT_EQ(0U, caster.requests()[0].find("GET /BASE HTTP/1.1\r\n"));
    EXPECT_NE(std::string::npos,
        caster.requests()[0].find("Ntrip-Version: Ntrip/2.0\r\n"));
    EXPECT_EQ(0U, caster.requests()[1].find("GET /BASE HTTP/1.0\r\n"));
    // the fallback never carried a stream, so the next cycle starts from the
    // configured version again instead of staying locked to v1 for the session
    EXPECT_EQ(0U, caster.requests()[2].find("GET /BASE HTTP/1.1\r\n"));
    EXPECT_NE(std::string::npos,
        caster.requests()[2].find("Ntrip-Version: Ntrip/2.0\r\n"));
    EXPECT_TRUE(snapshot.has_base_position);
    EXPECT_TRUE(snapshot.has_observations);
}


TEST(NtripRtcmClientTest, BaseLockTimeBaselineSurvivesReconnect)
{
    using namespace ntrip_rtcm_client_test;

    Loopback_Ntrip_Caster caster(Loopback_Caster_Close_Mode::KEEP_OPEN,
        Loopback_Caster_Response_Mode::LOCK_RESET_AFTER_RECONNECT);
    ASSERT_TRUE(caster.start()) << caster.start_failure() << ": "
                                << std::strerror(caster.start_errno());

    Ntrip_Rtcm_Client_Config config;
    config.enabled = true;
    config.host = "127.0.0.1";
    config.port = caster.port();
    config.mountpoint = "/BASE";
    config.username = TRACE_USERNAME;
    config.password = TRACE_PASSWORD;
    config.version = 1;
    config.reconnect_interval_ms = 1000;
    config.timeout_ms = 1000;
    config.max_age_s = 5.0;
    config.station_id = TEST_STATION_ID;

    const gtime_t rover_time = gpst2time(TEST_GPS_WEEK, TEST_TOW_S + 1.0);
    Ntrip_Rtcm_Client client(config);
    client.update_rover_time(rover_time);
    ASSERT_TRUE(client.start());

    // the first stream must publish observations before the drop
    Ntrip_Rtcm_Snapshot baseline;
    const std::chrono::steady_clock::time_point baseline_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
    do
        {
            baseline = client.latest_snapshot(rover_time);
            if (baseline.has_observations)
                {
                    break;
                }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    while (std::chrono::steady_clock::now() < baseline_deadline);
    ASSERT_TRUE(baseline.has_observations);

    // the reconnected stream carries a decreased lock-time indicator: with the
    // lock-time baseline preserved across the reconnect, the observations must
    // arrive flagged with a loss-of-lock indication
    Ntrip_Rtcm_Snapshot slipped;
    const std::chrono::steady_clock::time_point slip_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(3500);
    do
        {
            slipped = client.latest_snapshot(rover_time);
            if (slipped.has_observations &&
                (slipped.observations[0].LLI[0] & 1U) != 0)
                {
                    break;
                }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    while (std::chrono::steady_clock::now() < slip_deadline);

    client.stop();
    caster.join();

    ASSERT_EQ(2U, caster.accepted_connection_count());
    ASSERT_EQ(2U, caster.requests().size());
    ASSERT_TRUE(slipped.has_observations);
    // a per-connection decoder would restart the lock table at zero and
    // report the re-locked base satellite as continuous (LLI 0)
    EXPECT_EQ(1, slipped.observations[0].LLI[0]);
    EXPECT_EQ(1, slipped.observations[0].LLI[1]);
}


TEST(NtripRtcmClientTest, V2AuthenticationFailureDoesNotDowngrade)
{
    using namespace ntrip_rtcm_client_test;

    Loopback_Ntrip_Caster caster(Loopback_Caster_Close_Mode::KEEP_OPEN,
        Loopback_Caster_Response_Mode::V2_HTTP_UNAUTHORIZED);
    ASSERT_TRUE(caster.start()) << caster.start_failure() << ": "
                                << std::strerror(caster.start_errno());

    Ntrip_Rtcm_Client_Config config;
    config.enabled = true;
    config.host = "127.0.0.1";
    config.port = caster.port();
    config.mountpoint = "/BASE";
    config.username = TRACE_USERNAME;
    config.password = TRACE_PASSWORD;
    config.reconnect_interval_ms = 0;
    config.timeout_ms = 1000;

    Ntrip_Rtcm_Client client(config);
    ASSERT_TRUE(client.start());

    Ntrip_Rtcm_Client_Status status;
    const std::chrono::steady_clock::time_point error_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
    do
        {
            status = client.status();
            if (status.state == Ntrip_Rtcm_Client_State::ERROR)
                {
                    break;
                }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    while (std::chrono::steady_clock::now() < error_deadline);

    client.stop();
    caster.join();

    EXPECT_EQ(Ntrip_Rtcm_Client_State::ERROR, status.state);
    EXPECT_EQ(1U, caster.accepted_connection_count());
    ASSERT_EQ(1U, caster.requests().size());
    EXPECT_EQ(0U, caster.requests()[0].find("GET /BASE HTTP/1.1\r\n"));
    EXPECT_EQ(0U, status.reconnect_count);
    EXPECT_EQ(std::string::npos, status.message.find("fallback"));
}


TEST(NtripRtcmClientTest, ConnectFailureBeforeRequestDoesNotDowngradeV2)
{
    using namespace ntrip_rtcm_client_test;

    Reserved_Loopback_Port reserved_port;
    ASSERT_TRUE(reserved_port.reserve());

    Ntrip_Rtcm_Client_Config config;
    config.enabled = true;
    config.host = "127.0.0.1";
    config.port = reserved_port.port();
    config.mountpoint = "/BASE";
    config.reconnect_interval_ms = 1000;
    config.timeout_ms = 1000;
    config.max_age_s = 1.0;
    config.station_id = TEST_STATION_ID;

    const gtime_t rover_time = gpst2time(TEST_GPS_WEEK, TEST_TOW_S);
    Ntrip_Rtcm_Client client(config);
    client.update_rover_time(rover_time);
    ASSERT_TRUE(client.start());

    Ntrip_Rtcm_Client_Status failed_status;
    const std::chrono::steady_clock::time_point failure_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
    do
        {
            failed_status = client.status();
            if (failed_status.reconnect_count > 0)
                {
                    break;
                }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    while (std::chrono::steady_clock::now() < failure_deadline);
    ASSERT_GT(failed_status.reconnect_count, 0U);

    const std::uint16_t caster_port = reserved_port.port();
    reserved_port.release();
    Loopback_Ntrip_Caster caster(Loopback_Caster_Close_Mode::KEEP_OPEN,
        Loopback_Caster_Response_Mode::V2_HTTP_CHUNKED);
    ASSERT_TRUE(caster.start(caster_port))
        << caster.start_failure() << ": " << std::strerror(caster.start_errno());

    Ntrip_Rtcm_Snapshot snapshot;
    const std::chrono::steady_clock::time_point snapshot_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(2500);
    do
        {
            snapshot = client.latest_snapshot(rover_time);
            if (snapshot.has_base_position && snapshot.has_observations)
                {
                    break;
                }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    while (std::chrono::steady_clock::now() < snapshot_deadline);

    client.stop();
    caster.join();

    ASSERT_EQ(1U, caster.accepted_connection_count());
    ASSERT_EQ(1U, caster.requests().size());
    EXPECT_EQ(0U, caster.requests()[0].find("GET /BASE HTTP/1.1\r\n"));
    EXPECT_NE(std::string::npos,
        caster.requests()[0].find("Ntrip-Version: Ntrip/2.0\r\n"));
    EXPECT_TRUE(snapshot.has_base_position);
    EXPECT_TRUE(snapshot.has_observations);
}


TEST(NtripRtcmClientTest, ForcedV1ClientSendsLegacyRequest)
{
    using namespace ntrip_rtcm_client_test;

    Loopback_Ntrip_Caster caster;
    ASSERT_TRUE(caster.start()) << caster.start_failure() << ": "
                                << std::strerror(caster.start_errno());
    ASSERT_NE(0, caster.port());

    Ntrip_Rtcm_Client_Config config;
    config.enabled = true;
    config.host = "localhost";
    config.port = caster.port();
    config.mountpoint = "/BASE";
    config.username = TRACE_USERNAME;
    config.password = TRACE_PASSWORD;
    config.reconnect_interval_ms = 1000;
    config.timeout_ms = 1000;
    config.max_age_s = 1.0;
    config.station_id = TEST_STATION_ID;
    config.version = 1;

    const gtime_t rover_time = gpst2time(TEST_GPS_WEEK, TEST_TOW_S);
    Ntrip_Rtcm_Client client(config);
    client.update_rover_time(rover_time);
    ASSERT_TRUE(client.start());

    Ntrip_Rtcm_Snapshot snapshot;
    const std::chrono::steady_clock::time_point snapshot_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
    do
        {
            snapshot = client.latest_snapshot(rover_time);
            if (snapshot.has_base_position && snapshot.has_observations)
                {
                    break;
                }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    while (std::chrono::steady_clock::now() < snapshot_deadline);

    client.stop();
    caster.join();

    EXPECT_EQ(1U, caster.accepted_connection_count());
    ASSERT_EQ(1U, caster.requests().size());
    EXPECT_EQ(0U, caster.request().find("GET /BASE HTTP/1.0\r\n"));
    EXPECT_EQ(std::string::npos, caster.request().find("HTTP/1.1"));
    EXPECT_EQ(std::string::npos, caster.request().find("Ntrip-Version:"));
    ASSERT_TRUE(snapshot.has_base_position);
    ASSERT_TRUE(snapshot.has_observations);
}


TEST(NtripRtcmClientTest, ZeroReconnectIntervalStopsAfterStreamDisconnect)
{
    using namespace ntrip_rtcm_client_test;

    Loopback_Ntrip_Caster caster(Loopback_Caster_Close_Mode::AFTER_PAYLOAD);
    ASSERT_TRUE(caster.start()) << caster.start_failure() << ": "
                                << std::strerror(caster.start_errno());

    Ntrip_Rtcm_Client_Config config;
    config.enabled = true;
    config.host = "127.0.0.1";
    config.port = caster.port();
    config.mountpoint = "BASE";
    config.reconnect_interval_ms = 0;
    config.timeout_ms = 1000;
    config.max_age_s = 1.0;
    config.station_id = TEST_STATION_ID;

    const gtime_t rover_time = gpst2time(TEST_GPS_WEEK, TEST_TOW_S);
    Ntrip_Rtcm_Client client(config);
    client.update_rover_time(rover_time);
    ASSERT_TRUE(client.start());

    Ntrip_Rtcm_Snapshot snapshot;
    const std::chrono::steady_clock::time_point snapshot_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
    do
        {
            snapshot = client.latest_snapshot(rover_time);
            if (snapshot.has_base_position && snapshot.has_observations)
                {
                    break;
                }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    while (std::chrono::steady_clock::now() < snapshot_deadline);

    const std::chrono::steady_clock::time_point stopped_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
    while (client.running() &&
           std::chrono::steady_clock::now() < stopped_deadline)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    const Ntrip_Rtcm_Client_Status disconnected_status = client.status();
    client.stop();
    caster.join();

    EXPECT_TRUE(snapshot.has_base_position);
    EXPECT_TRUE(snapshot.has_observations);
    EXPECT_FALSE(disconnected_status.running);
    EXPECT_FALSE(disconnected_status.connected);
    EXPECT_EQ(Ntrip_Rtcm_Client_State::ERROR, disconnected_status.state);
    EXPECT_NE(std::string::npos,
        disconnected_status.message.find("automatic reconnect is disabled"));
    EXPECT_EQ(0U, disconnected_status.reconnect_count);
    EXPECT_TRUE(caster.accepted_client());
    EXPECT_TRUE(caster.server_closed_after_payload());
}


TEST(NtripRtcmClientTest, PeerResetDuringAuthenticationDoesNotTerminateClient)
{
    using namespace ntrip_rtcm_client_test;

    Loopback_Ntrip_Caster caster(Loopback_Caster_Close_Mode::ON_ACCEPT);
    ASSERT_TRUE(caster.start()) << caster.start_failure() << ": "
                                << std::strerror(caster.start_errno());

    Ntrip_Rtcm_Client_Config config;
    config.enabled = true;
    config.host = "127.0.0.1";
    config.port = caster.port();
    config.mountpoint = "BASE";
    config.username = "reset-test-user";
    config.password = "reset-test-password";
    config.reconnect_interval_ms = 0;
    config.timeout_ms = 1000;

    Ntrip_Rtcm_Client client(config);
    ASSERT_TRUE(client.start());

    const std::chrono::steady_clock::time_point stopped_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
    while (client.running() &&
           std::chrono::steady_clock::now() < stopped_deadline)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    const Ntrip_Rtcm_Client_Status reset_status = client.status();
    client.stop();
    caster.join();

    EXPECT_TRUE(caster.accepted_client());
    EXPECT_TRUE(caster.server_closed_on_accept());
    EXPECT_TRUE(caster.request().empty());
    EXPECT_FALSE(reset_status.running);
    EXPECT_FALSE(reset_status.connected);
    EXPECT_EQ(Ntrip_Rtcm_Client_State::ERROR, reset_status.state);
    EXPECT_EQ(0U, reset_status.reconnect_count);
    EXPECT_NE(std::string::npos,
        reset_status.message.find("automatic reconnect is disabled"));
}
