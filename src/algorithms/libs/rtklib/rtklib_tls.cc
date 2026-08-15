/*!
 * \file rtklib_tls.cc
 * \brief TLS client transport for RTKLIB streams
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
#include <cstdio>
#include <mutex>
#include <utility>

#ifdef USE_GNUTLS_FALLBACK
#include <gnutls/gnutls.h>
#else
#include <openssl/err.h>
#include <openssl/ssl.h>
#include <openssl/x509_vfy.h>
#endif

namespace
{
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

void initialize_gnutls_once()
{
    gnutls_init_result = gnutls_global_init();
}
#else
#if OPENSSL_VERSION_NUMBER < 0x10100000L
std::once_flag openssl_init_flag;

void initialize_openssl_once()
{
    SSL_library_init();
    SSL_load_error_strings();
}
#endif
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

    bool initialize(char *msg)
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

        ret = gnutls_certificate_set_x509_system_trust(d_credentials);
        if (ret < 0)
            {
                gnutls_certificate_free_credentials(d_credentials);
                d_credentials = nullptr;
                set_tls_msg(msg, "GnuTLS system trust failed");
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

        SSL_CTX_set_verify(d_context, SSL_VERIFY_PEER, nullptr);
        if (SSL_CTX_set_default_verify_paths(d_context) != 1)
            {
                SSL_CTX_free(d_context);
                d_context = nullptr;
                set_tls_msg(msg, "OpenSSL system trust failed");
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

    void reset()
    {
#ifdef USE_GNUTLS_FALLBACK
        if (d_session)
            {
                gnutls_deinit(d_session);
                d_session = nullptr;
            }
#else
        if (d_session)
            {
                SSL_free(d_session);
                d_session = nullptr;
            }
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
                int ret = gnutls_init(&d_session, GNUTLS_CLIENT | GNUTLS_NONBLOCK);
                if (ret < 0)
                    {
                        set_tls_msg(msg, "GnuTLS session failed");
                        return -1;
                    }
                if ((ret = gnutls_set_default_priority(d_session)) < 0 ||
                    (ret = gnutls_credentials_set(d_session, GNUTLS_CRD_CERTIFICATE, d_credentials)) < 0)
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
                gnutls_transport_set_int(d_session, sock);
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

#if GNUTLS_VERSION_NUMBER >= 0x030104
        unsigned int status = 0;
        ret = gnutls_certificate_verify_peers3(d_session, d_hostname.c_str(), &status);
        if (ret < 0 || status != 0)
            {
                reset();
                set_tls_msg(msg, "GnuTLS certificate failed");
                return -1;
            }
#else
        reset();
        set_tls_msg(msg, "GnuTLS >= 3.1.4 required for NTRIP TLS");
        return -1;
#endif
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
        const ssize_t ret = gnutls_record_send(d_session, buff, static_cast<size_t>(n));
        if (ret >= 0)
            {
                return static_cast<int>(ret);
            }
        if (ret == GNUTLS_E_AGAIN || ret == GNUTLS_E_INTERRUPTED)
            {
                return 0;
            }
        set_tls_msg(msg, "GnuTLS send failed");
        return -1;
#else
        const int ret = SSL_write(d_session, buff, n);
        if (ret > 0)
            {
                return ret;
            }
        const int ssl_error = SSL_get_error(d_session, ret);
        if (ssl_error == SSL_ERROR_WANT_READ || ssl_error == SSL_ERROR_WANT_WRITE)
            {
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
#else
    SSL_CTX *d_context = nullptr;
    SSL *d_session = nullptr;
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


void Rtklib_Tls_Client::reset()
{
    d_impl->reset();
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
