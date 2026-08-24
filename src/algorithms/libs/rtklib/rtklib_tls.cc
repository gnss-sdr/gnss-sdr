/*!
 * \file rtklib_tls.cc
 * \brief TLS 1.2-or-newer client transport for RTKLIB streams
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2026 Carles Fernandez-Prades
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * -----------------------------------------------------------------------------
 */

#include "rtklib_tls.h"
#include <arpa/inet.h>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <utility>
#include <vector>

#ifdef USE_GNUTLS_FALLBACK
#include <gnutls/gnutls.h>
#include <gnutls/x509.h>
#else
#include <openssl/err.h>
#include <openssl/pem.h>
#include <openssl/ssl.h>
#include <openssl/x509_vfy.h>
#endif

namespace
{
// OpenSSL's socket BIO and older GnuTLS releases can call send() without
// MSG_NOSIGNAL. Block SIGPIPE only in the calling thread while TLS performs
// I/O, consume only a signal raised by that operation, and leave the process-
// wide signal disposition untouched.
class Tls_Sigpipe_Guard
{
public:
    Tls_Sigpipe_Guard() noexcept
    {
        if (sigemptyset(&d_sigpipe_set) != 0 ||
            sigaddset(&d_sigpipe_set, SIGPIPE) != 0)
            {
                return;
            }

        if (pthread_sigmask(SIG_BLOCK, &d_sigpipe_set, &d_previous_mask) != 0)
            {
                return;
            }
        d_restore_mask = true;

        sigset_t pending_signals;
        if (sigpending(&pending_signals) != 0)
            {
                pthread_sigmask(SIG_SETMASK, &d_previous_mask, nullptr);
                d_restore_mask = false;
                return;
            }

        d_sigpipe_was_pending = sigismember(&pending_signals, SIGPIPE) == 1;
        d_ready = true;
    }

    ~Tls_Sigpipe_Guard()
    {
        if (!d_restore_mask)
            {
                return;
            }

        const int saved_errno = errno;
        if (!d_sigpipe_was_pending)
            {
                sigset_t pending_signals;
                if (sigpending(&pending_signals) == 0 &&
                    sigismember(&pending_signals, SIGPIPE) == 1)
                    {
                        int received_signal = 0;
                        sigwait(&d_sigpipe_set, &received_signal);
                    }
            }
        pthread_sigmask(SIG_SETMASK, &d_previous_mask, nullptr);
        errno = saved_errno;
    }

    Tls_Sigpipe_Guard(const Tls_Sigpipe_Guard &) = delete;
    Tls_Sigpipe_Guard &operator=(const Tls_Sigpipe_Guard &) = delete;
    Tls_Sigpipe_Guard(Tls_Sigpipe_Guard &&) = delete;
    Tls_Sigpipe_Guard &operator=(Tls_Sigpipe_Guard &&) = delete;

    bool ready() const noexcept { return d_ready; }

private:
    sigset_t d_sigpipe_set{};
    sigset_t d_previous_mask{};
    bool d_sigpipe_was_pending = false;
    bool d_restore_mask = false;
    bool d_ready = false;
};


bool is_numeric_address(const std::string &hostname)
{
    struct in_addr addr4{};
    struct in6_addr addr6{};
    return inet_pton(AF_INET, hostname.c_str(), &addr4) == 1 ||
           inet_pton(AF_INET6, hostname.c_str(), &addr6) == 1;
}

void set_tls_msg(char *msg, const char *text)
{
    if (msg)
        {
            std::snprintf(msg, MAXSTRMSG, "%s", text);
        }
}

#ifdef USE_GNUTLS_FALLBACK
std::once_flag gnutls_init_flag;
int gnutls_init_result = GNUTLS_E_SUCCESS;

#if GNUTLS_VERSION_NUMBER >= 0x030603
constexpr char GNUTLS_TLS_1_2_OR_NEWER_PRIORITY[] =
    "-VERS-SSL3.0:-VERS-TLS1.0:-VERS-TLS1.1";
#endif

void initialize_gnutls_once()
{
    gnutls_init_result = gnutls_global_init();
}

void gnutls_set_socket_transport(gnutls_session_t session, socket_t socket)
{
#if GNUTLS_VERSION_NUMBER >= 0x030200
    gnutls_transport_set_int(session, socket);
#else
    gnutls_transport_set_ptr(session,
        reinterpret_cast<gnutls_transport_ptr_t>(
            static_cast<std::intptr_t>(socket)));
#endif
}

#if GNUTLS_VERSION_NUMBER >= 0x030104
bool gnutls_leaf_allows_server_auth(gnutls_session_t session)
{
    if (gnutls_certificate_type_get(session) != GNUTLS_CRT_X509)
        {
            return false;
        }

    unsigned int peer_count = 0;
    const gnutls_datum_t *peers =
        gnutls_certificate_get_peers(session, &peer_count);
    if (peers == nullptr || peer_count == 0 ||
        peers[0].data == nullptr || peers[0].size == 0)
        {
            return false;
        }

    gnutls_x509_crt_t leaf = nullptr;
    if (gnutls_x509_crt_init(&leaf) < 0)
        {
            return false;
        }

    bool allowed = false;
    if (gnutls_x509_crt_import(leaf, &peers[0], GNUTLS_X509_FMT_DER) >= 0)
        {
            bool saw_key_purpose = false;
            for (unsigned int index = 0;; ++index)
                {
                    char oid[64] = {};
                    size_t oid_size = sizeof(oid);
                    unsigned int critical = 0;
                    const int ret = gnutls_x509_crt_get_key_purpose_oid(
                        leaf, index, oid, &oid_size, &critical);
                    if (ret == GNUTLS_E_REQUESTED_DATA_NOT_AVAILABLE)
                        {
                            // No EKU extension means unrestricted use. Once
                            // an EKU was seen, reaching the end without a
                            // permitted purpose is a mismatch.
                            allowed = !saw_key_purpose;
                            break;
                        }
                    if (ret == GNUTLS_E_SHORT_MEMORY_BUFFER)
                        {
                            // Both accepted OIDs fit in this buffer, so an
                            // overlong OID cannot be one of them.
                            saw_key_purpose = true;
                            continue;
                        }
                    if (ret < 0)
                        {
                            break;
                        }

                    saw_key_purpose = true;
                    if (std::strcmp(oid, GNUTLS_KP_TLS_WWW_SERVER) == 0 ||
                        std::strcmp(oid, GNUTLS_KP_ANY) == 0)
                        {
                            allowed = true;
                            break;
                        }
                }
        }

    gnutls_x509_crt_deinit(leaf);
    return allowed;
}
#endif
#else
#if OPENSSL_VERSION_NUMBER < 0x10100000L
std::once_flag openssl_init_flag;

void initialize_openssl_once()
{
    SSL_library_init();
    SSL_load_error_strings();
}
#endif

bool openssl_load_trust_anchor(SSL_CTX *context, const std::string &pem)
{
    BIO *bio = BIO_new_mem_buf(const_cast<char *>(pem.data()),
        static_cast<int>(pem.size()));
    if (bio == nullptr)
        {
            return false;
        }

    X509 *certificate = PEM_read_bio_X509(bio, nullptr, nullptr, nullptr);
    BIO_free(bio);
    if (certificate == nullptr)
        {
            return false;
        }

    X509_STORE *store = SSL_CTX_get_cert_store(context);
    const bool loaded = store != nullptr &&
                        X509_STORE_add_cert(store, certificate) == 1;
    X509_free(certificate);
    return loaded;
}
#endif
}  // namespace


class Rtklib_Tls_Client::Impl
{
public:
    explicit Impl(std::string hostname) : d_hostname(std::move(hostname)) {}

    Impl(const Impl &) = delete;
    Impl &operator=(const Impl &) = delete;
    Impl(Impl &&) = delete;
    Impl &operator=(Impl &&) = delete;

    bool initialize(char *msg, const std::string *trust_anchor_pem = nullptr)
    {
#ifdef USE_GNUTLS_FALLBACK
        std::call_once(gnutls_init_flag, initialize_gnutls_once);
        if (gnutls_init_result < 0)
            {
                set_tls_msg(msg, "GnuTLS initialization failed");
                return false;
            }

#if GNUTLS_VERSION_NUMBER < 0x030104
        set_tls_msg(msg, "GnuTLS >= 3.1.4 required for NTRIP TLS");
        return false;
#else
        int ret = gnutls_certificate_allocate_credentials(&d_credentials);
        if (ret < 0)
            {
                set_tls_msg(msg, "GnuTLS credentials failed");
                return false;
            }

        if (trust_anchor_pem != nullptr)
            {
                gnutls_datum_t trust_anchor;
                trust_anchor.data = reinterpret_cast<unsigned char *>(
                    const_cast<char *>(trust_anchor_pem->data()));
                trust_anchor.size = static_cast<unsigned int>(trust_anchor_pem->size());
                ret = gnutls_certificate_set_x509_trust_mem(d_credentials,
                    &trust_anchor, GNUTLS_X509_FMT_PEM);
            }
        else
            {
                ret = gnutls_certificate_set_x509_system_trust(d_credentials);
            }
        if (ret < 0 || (trust_anchor_pem != nullptr && ret == 0))
            {
                gnutls_certificate_free_credentials(d_credentials);
                d_credentials = nullptr;
                if (trust_anchor_pem == nullptr)
                    {
                        set_tls_msg(msg, "GnuTLS system trust failed");
                    }
                else
                    {
                        set_tls_msg(msg, "GnuTLS test trust failed");
                    }
                return false;
            }
#endif
#else
#if OPENSSL_VERSION_NUMBER < 0x10100000L
        std::call_once(openssl_init_flag, initialize_openssl_once);
#endif
#if OPENSSL_VERSION_NUMBER >= 0x10100000L
        d_context = SSL_CTX_new(TLS_client_method());
#else
        d_context = SSL_CTX_new(SSLv23_client_method());
#endif
        if (!d_context)
            {
                set_tls_msg(msg, "OpenSSL context failed");
                return false;
            }

#if OPENSSL_VERSION_NUMBER < 0x10002000L
        SSL_CTX_free(d_context);
        d_context = nullptr;
        set_tls_msg(msg, "OpenSSL >= 1.0.2 required for NTRIP TLS");
        return false;
#elif OPENSSL_VERSION_NUMBER >= 0x10100000L && !defined(LIBRESSL_VERSION_NUMBER)
        if (SSL_CTX_set_min_proto_version(d_context, TLS1_2_VERSION) != 1)
            {
                SSL_CTX_free(d_context);
                d_context = nullptr;
                set_tls_msg(msg, "OpenSSL TLS 1.2 policy failed");
                return false;
            }
#else
        const long disabled_protocols = SSL_OP_NO_SSLv2 | SSL_OP_NO_SSLv3 |
                                        SSL_OP_NO_TLSv1 | SSL_OP_NO_TLSv1_1;
        if ((SSL_CTX_set_options(d_context, disabled_protocols) &
                disabled_protocols) != disabled_protocols)
            {
                SSL_CTX_free(d_context);
                d_context = nullptr;
                set_tls_msg(msg, "OpenSSL TLS 1.2 policy failed");
                return false;
            }
#endif

        // SSL_write() demands a retry after WANT_READ/WANT_WRITE to present
        // the very same buffer and length. These modes make a moved buffer
        // legal and let partial writes report the bytes actually consumed,
        // but ssl3_write_pending() still rejects a retry SHORTER than the
        // pending record with SSL_R_BAD_WRITE_RETRY, and an equal-or-longer
        // retry silently transmits the cached record instead of the new data:
        // write() below therefore keeps its own copy of a backpressured
        // buffer and flushes exactly that copy before accepting new data
        SSL_CTX_set_mode(d_context,
            SSL_MODE_ENABLE_PARTIAL_WRITE | SSL_MODE_ACCEPT_MOVING_WRITE_BUFFER);
        SSL_CTX_set_verify(d_context, SSL_VERIFY_PEER, nullptr);
        bool trust_loaded = false;
        if (trust_anchor_pem != nullptr)
            {
                trust_loaded = openssl_load_trust_anchor(d_context,
                    *trust_anchor_pem);
            }
        else
            {
                trust_loaded = SSL_CTX_set_default_verify_paths(d_context) == 1;
            }
        if (!trust_loaded)
            {
                SSL_CTX_free(d_context);
                d_context = nullptr;
                if (trust_anchor_pem == nullptr)
                    {
                        set_tls_msg(msg, "OpenSSL system trust failed");
                    }
                else
                    {
                        set_tls_msg(msg, "OpenSSL test trust failed");
                    }
                return false;
            }
#endif
        return true;
    }

    ~Impl()
    {
        reset();
#ifdef USE_GNUTLS_FALLBACK
        if (d_credentials)
            {
                gnutls_certificate_free_credentials(d_credentials);
            }
#else
        if (d_context)
            {
                SSL_CTX_free(d_context);
            }
#endif
    }

    bool set_hostname(const std::string &hostname)
    {
        if (hostname.empty() || d_session != nullptr)
            {
                return false;
            }
        d_hostname = hostname;
        return true;
    }

    void reset()
    {
#ifdef USE_GNUTLS_FALLBACK
        if (d_session)
            {
                gnutls_deinit(d_session);
                d_session = nullptr;
            }
        d_send_pending = false;
#else
        if (d_session)
            {
                SSL_free(d_session);
                d_session = nullptr;
            }
        d_pending_record.clear();
#endif
        d_established = false;
    }

    int handshake(socket_t sock, char *msg)
    {
        if (d_established)
            {
                return 1;
            }

#ifdef USE_GNUTLS_FALLBACK
        if (!d_session)
            {
                unsigned int init_flags = GNUTLS_CLIENT | GNUTLS_NONBLOCK;
#if GNUTLS_VERSION_NUMBER >= 0x030402
                init_flags |= GNUTLS_NO_SIGNAL;
#endif
                int ret = gnutls_init(&d_session, init_flags);
                if (ret < 0)
                    {
                        set_tls_msg(msg, "GnuTLS session failed");
                        return -1;
                    }
#if GNUTLS_VERSION_NUMBER >= 0x030603
                ret = gnutls_set_default_priority_append(d_session,
                    GNUTLS_TLS_1_2_OR_NEWER_PRIORITY, nullptr, 0);
#else
                ret = gnutls_priority_set_direct(d_session,
                    "NORMAL:-VERS-SSL3.0:-VERS-TLS1.0:-VERS-TLS1.1", nullptr);
#endif
                if (ret < 0)
                    {
                        reset();
                        set_tls_msg(msg, "GnuTLS TLS 1.2 policy failed");
                        return -1;
                    }
                ret = gnutls_credentials_set(d_session, GNUTLS_CRD_CERTIFICATE, d_credentials);
                if (ret < 0)
                    {
                        reset();
                        set_tls_msg(msg, "GnuTLS setup failed");
                        return -1;
                    }
                if (!is_numeric_address(d_hostname))
                    {
                        ret = gnutls_server_name_set(d_session, GNUTLS_NAME_DNS,
                            d_hostname.c_str(), d_hostname.size());
                        if (ret < 0)
                            {
                                reset();
                                set_tls_msg(msg, "GnuTLS SNI failed");
                                return -1;
                            }
                    }
                gnutls_set_socket_transport(d_session, sock);
            }

        Tls_Sigpipe_Guard sigpipe_guard;
        if (!sigpipe_guard.ready())
            {
                reset();
                set_tls_msg(msg, "TLS signal protection failed");
                return -1;
            }
        int ret = gnutls_handshake(d_session);
        if (ret == GNUTLS_E_AGAIN || ret == GNUTLS_E_INTERRUPTED)
            {
                return 0;
            }
        if (ret < 0)
            {
                reset();
                set_tls_msg(msg, "GnuTLS handshake failed");
                return -1;
            }

#if GNUTLS_VERSION_NUMBER >= 0x030300
        unsigned int status = 0;
        gnutls_typed_vdata_st verification_data[2] = {};
        verification_data[0].type = GNUTLS_DT_DNS_HOSTNAME;
        verification_data[0].data = const_cast<unsigned char *>(
            reinterpret_cast<const unsigned char *>(d_hostname.c_str()));
        verification_data[1].type = GNUTLS_DT_KEY_PURPOSE_OID;
        verification_data[1].data = const_cast<unsigned char *>(
            reinterpret_cast<const unsigned char *>(GNUTLS_KP_TLS_WWW_SERVER));
        ret = gnutls_certificate_verify_peers(d_session,
            verification_data, 2, &status);
#elif GNUTLS_VERSION_NUMBER >= 0x030104
        unsigned int status = 0;
        ret = gnutls_certificate_verify_peers3(
            d_session, d_hostname.c_str(), &status);
#else
        unsigned int status = GNUTLS_CERT_INVALID;
        ret = GNUTLS_E_UNIMPLEMENTED_FEATURE;
#endif
#if GNUTLS_VERSION_NUMBER >= 0x030104
        // GnuTLS 3.1 and 3.2 cannot receive a purpose in their peer
        // verification API. Keep the explicit leaf check on newer releases
        // as well so the compatibility path is covered by normal CI.
        if (ret >= 0 && status == 0 &&
            !gnutls_leaf_allows_server_auth(d_session))
            {
                status |= GNUTLS_CERT_INVALID |
                          GNUTLS_CERT_SIGNER_CONSTRAINTS_FAILURE;
            }
#endif
        if (ret < 0 || status != 0)
            {
                reset();
                set_tls_msg(msg, "GnuTLS certificate failed");
                return -1;
            }
#else
        if (!d_session)
            {
                d_session = SSL_new(d_context);
                if (!d_session || SSL_set_fd(d_session, sock) != 1)
                    {
                        reset();
                        set_tls_msg(msg, "OpenSSL session failed");
                        return -1;
                    }

#if OPENSSL_VERSION_NUMBER < 0x10002000L
                reset();
                set_tls_msg(msg, "OpenSSL >= 1.0.2 required for NTRIP TLS");
                return -1;
#else
                X509_VERIFY_PARAM *param = SSL_get0_param(d_session);
                if (!param)
                    {
                        reset();
                        set_tls_msg(msg, "OpenSSL verification setup failed");
                        return -1;
                    }
                if (!is_numeric_address(d_hostname))
                    {
                        if (SSL_set_tlsext_host_name(d_session, d_hostname.c_str()) != 1 ||
                            X509_VERIFY_PARAM_set1_host(param, d_hostname.c_str(), 0) != 1)
                            {
                                reset();
                                set_tls_msg(msg, "OpenSSL hostname setup failed");
                                return -1;
                            }
                    }
                else if (X509_VERIFY_PARAM_set1_ip_asc(param, d_hostname.c_str()) != 1)
                    {
                        reset();
                        set_tls_msg(msg, "OpenSSL IP verification failed");
                        return -1;
                    }
#endif
                SSL_set_connect_state(d_session);
            }

        Tls_Sigpipe_Guard sigpipe_guard;
        if (!sigpipe_guard.ready())
            {
                reset();
                set_tls_msg(msg, "TLS signal protection failed");
                return -1;
            }
        int ret = SSL_connect(d_session);
        if (ret != 1)
            {
                const int ssl_error = SSL_get_error(d_session, ret);
                if (ssl_error == SSL_ERROR_WANT_READ || ssl_error == SSL_ERROR_WANT_WRITE)
                    {
                        return 0;
                    }
                reset();
                set_tls_msg(msg, "OpenSSL handshake failed");
                return -1;
            }

        if (SSL_get_verify_result(d_session) != X509_V_OK)
            {
                reset();
                set_tls_msg(msg, "OpenSSL certificate failed");
                return -1;
            }
#endif

        d_established = true;
        return 1;
    }

    int read(unsigned char *buff, int n, char *msg)
    {
        if (!d_established || n <= 0)
            {
                return 0;
            }
#ifdef USE_GNUTLS_FALLBACK
        Tls_Sigpipe_Guard sigpipe_guard;
        if (!sigpipe_guard.ready())
            {
                set_tls_msg(msg, "TLS signal protection failed");
                return -1;
            }
        const ssize_t ret = gnutls_record_recv(d_session, buff, static_cast<size_t>(n));
        if (ret > 0)
            {
                return static_cast<int>(ret);
            }
        if (ret == GNUTLS_E_AGAIN || ret == GNUTLS_E_INTERRUPTED)
            {
                return 0;
            }
        if (ret == 0)
            {
                set_tls_msg(msg, "TLS connection closed");
            }
        else
            {
                set_tls_msg(msg, "GnuTLS receive failed");
            }
        return -1;
#else
        Tls_Sigpipe_Guard sigpipe_guard;
        if (!sigpipe_guard.ready())
            {
                set_tls_msg(msg, "TLS signal protection failed");
                return -1;
            }
        const int ret = SSL_read(d_session, buff, n);
        if (ret > 0)
            {
                return ret;
            }
        const int ssl_error = SSL_get_error(d_session, ret);
        if (ssl_error == SSL_ERROR_WANT_READ || ssl_error == SSL_ERROR_WANT_WRITE)
            {
                return 0;
            }
        set_tls_msg(msg, ssl_error == SSL_ERROR_ZERO_RETURN ? "TLS connection closed" : "OpenSSL receive failed");
        return -1;
#endif
    }

    int write(const unsigned char *buff, int n, char *msg)
    {
        if (!d_established || n <= 0)
            {
                return 0;
            }
#ifdef USE_GNUTLS_FALLBACK
        Tls_Sigpipe_Guard sigpipe_guard;
        if (!sigpipe_guard.ready())
            {
                set_tls_msg(msg, "TLS signal protection failed");
                return -1;
            }
        // after GNUTLS_E_AGAIN the record stays cached inside the session and
        // the next gnutls_record_send() transmits that cached record, ignoring
        // the new arguments and returning the cached size: track the pending
        // state so a caller retrying with different data (a fresh GGA
        // sentence) is never told more than its own length was written, and a
        // stable-buffer retry loop keeps its accounting exact
        const ssize_t ret = gnutls_record_send(d_session, buff, static_cast<size_t>(n));
        if (ret >= 0)
            {
                const bool flushed_cached_record = d_send_pending;
                d_send_pending = false;
                if (flushed_cached_record && ret > static_cast<ssize_t>(n))
                    {
                        return n;
                    }
                return static_cast<int>(ret);
            }
        if (ret == GNUTLS_E_AGAIN || ret == GNUTLS_E_INTERRUPTED)
            {
                d_send_pending = true;
                return 0;
            }
        set_tls_msg(msg, "GnuTLS send failed");
        return -1;
#else
        Tls_Sigpipe_Guard sigpipe_guard;
        if (!sigpipe_guard.ready())
            {
                set_tls_msg(msg, "TLS signal protection failed");
                return -1;
            }
        // OpenSSL keeps the record pending after WANT_READ/WANT_WRITE and
        // rejects any retry shorter than it (SSL_R_BAD_WRITE_RETRY), even
        // with SSL_MODE_ACCEPT_MOVING_WRITE_BUFFER; an equal-or-longer retry
        // silently transmits the cached record in place of the new data.
        // Flush the stored copy of the backpressured buffer first: a stable-
        // buffer retry (the handshake path) resends its own data exactly
        // once, and a fresh GGA retry costs at most one stale-by-a-period
        // sentence, with the report clamped to the caller's length
        if (!d_pending_record.empty())
            {
                const int pending = static_cast<int>(d_pending_record.size());
                const int flushed = SSL_write(d_session, d_pending_record.data(), pending);
                if (flushed <= 0)
                    {
                        const int flush_error = SSL_get_error(d_session, flushed);
                        if (flush_error == SSL_ERROR_WANT_READ || flush_error == SSL_ERROR_WANT_WRITE)
                            {
                                return 0;
                            }
                        set_tls_msg(msg, "OpenSSL send failed");
                        return -1;
                    }
                d_pending_record.clear();
                return flushed > n ? n : flushed;
            }
        const int ret = SSL_write(d_session, buff, n);
        if (ret > 0)
            {
                return ret;
            }
        const int ssl_error = SSL_get_error(d_session, ret);
        if (ssl_error == SSL_ERROR_WANT_READ || ssl_error == SSL_ERROR_WANT_WRITE)
            {
                d_pending_record.assign(buff, buff + n);
                return 0;
            }
        set_tls_msg(msg, "OpenSSL send failed");
        return -1;
#endif
    }

private:
    std::string d_hostname;
    bool d_established = false;
#ifdef USE_GNUTLS_FALLBACK
    gnutls_certificate_credentials_t d_credentials = nullptr;
    gnutls_session_t d_session = nullptr;
    bool d_send_pending = false;
#else
    SSL_CTX *d_context = nullptr;
    SSL *d_session = nullptr;
    std::vector<unsigned char> d_pending_record;
#endif
};


Rtklib_Tls_Client::Rtklib_Tls_Client(std::string hostname)
    : d_impl(new Impl(std::move(hostname)))
{
}


Rtklib_Tls_Client::~Rtklib_Tls_Client() = default;


bool Rtklib_Tls_Client::initialize(char *msg)
{
    return d_impl->initialize(msg);
}


bool Rtklib_Tls_Client::initialize_with_trust_anchor(
    const std::string &trust_anchor_pem, char *msg)
{
    return d_impl->initialize(msg, &trust_anchor_pem);
}


void Rtklib_Tls_Client::reset()
{
    d_impl->reset();
}


bool Rtklib_Tls_Client::set_hostname(const std::string &hostname)
{
    return d_impl->set_hostname(hostname);
}


int Rtklib_Tls_Client::handshake(socket_t sock, char *msg)
{
    return d_impl->handshake(sock, msg);
}


int Rtklib_Tls_Client::read(unsigned char *buff, int n, char *msg)
{
    return d_impl->read(buff, n, msg);
}


int Rtklib_Tls_Client::write(const unsigned char *buff, int n, char *msg)
{
    return d_impl->write(buff, n, msg);
}
