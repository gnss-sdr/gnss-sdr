/*!
 * \file rtklib_tls_test.cc
 * \brief Unit tests for the RTKLIB TLS client transport
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

#include "rtklib_tls.h"
#include <gtest/gtest.h>
#include <cstring>
#include <mutex>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#ifdef USE_GNUTLS_FALLBACK
#include <gnutls/gnutls.h>
#else
#include <openssl/err.h>
#include <openssl/pem.h>
#include <openssl/ssl.h>
#include <openssl/ssl3.h>
#endif

namespace rtklib_tls_test
{
constexpr char TLS_TEST_CERTIFICATE[] = R"pem(-----BEGIN CERTIFICATE-----
MIIDRjCCAi6gAwIBAgIUHbjh0Z1ZWxD/houVkXl9m55byyYwDQYJKoZIhvcNAQEL
BQAwFDESMBAGA1UEAwwJbG9jYWxob3N0MB4XDTI2MDgxNTA3NTkzMFoXDTM2MDgx
MjA3NTkzMFowFDESMBAGA1UEAwwJbG9jYWxob3N0MIIBIjANBgkqhkiG9w0BAQEF
AAOCAQ8AMIIBCgKCAQEAp44yAzbhzs41KRaIoPar4k6BdOXUs//MVU201x7aBsMT
gYlEcuUhVoM7Ev7HSiCrAsgGPEWAp6b/EOm/7AqD2yZNFe0e6nuF+OA9Vx3E7oke
I7BpbpQ+7+Q/gD9jp+mI5T7i233JlBJln1J0x1bxwSdb/pz16zLbSQd5CfHPoc7m
nGdj2dYQzI1Rw5baSTO+nRJUVPpFf16DNBO13RIIEOEDVxXELpUNMj1THLGgTU8E
hu/4XTl4469L6CD02Y0FRNMRQFxxGwFtwJkyZ5aBHSQ8ZFSziOBAC6z7Ponp3vE2
+Fl0SlOt9mMK8ANzqRJODwqCiyHaHDuKe19MbfXBHwIDAQABo4GPMIGMMB0GA1Ud
DgQWBBRMruFCt3TWMJChUYYzxGuT9mhI4DAfBgNVHSMEGDAWgBRMruFCt3TWMJCh
UYYzxGuT9mhI4DAUBgNVHREEDTALgglsb2NhbGhvc3QwDwYDVR0TAQH/BAUwAwEB
/zAOBgNVHQ8BAf8EBAMCAqQwEwYDVR0lBAwwCgYIKwYBBQUHAwEwDQYJKoZIhvcN
AQELBQADggEBAGKpikxI4na/BQsb5LDo6RMGNmJw/Soqik9520hq72WX0OQJ32x+
cRecPBr7mm+ASm88eAkarWvTVOhOiQCxAXQIRN6uLE1gmUFX/ZHxD02AB1KtWukn
2Hm674bCfyPWoKn9/sE7mSKcsjIboBotQPv95M6yRSa2CvqR39ICTl4tQ1UXHDoy
C0tsMNVa/6G2crQwtq6sCeLMiQIjwvjHoIL9Jwp1gA7M7pZTC1nAx76arA/1m89L
lK4oiMLfuHQaqYYHGICNu68zZpD9YfwY6ErxnslBrCh9NJRQTj17G+yDzgFdCE/m
rDyVsDDOeFu0LCy7VXg9K1qmkHuESeCy3xE=
-----END CERTIFICATE-----
)pem";

constexpr char TLS_TEST_PRIVATE_KEY[] = R"pem(-----BEGIN PRIVATE KEY-----
MIIEvQIBADANBgkqhkiG9w0BAQEFAASCBKcwggSjAgEAAoIBAQCnjjIDNuHOzjUp
Foig9qviToF05dSz/8xVTbTXHtoGwxOBiURy5SFWgzsS/sdKIKsCyAY8RYCnpv8Q
6b/sCoPbJk0V7R7qe4X44D1XHcTuiR4jsGlulD7v5D+AP2On6YjlPuLbfcmUEmWf
UnTHVvHBJ1v+nPXrMttJB3kJ8c+hzuacZ2PZ1hDMjVHDltpJM76dElRU+kV/XoM0
E7XdEggQ4QNXFcQulQ0yPVMcsaBNTwSG7/hdOXjjr0voIPTZjQVE0xFAXHEbAW3A
mTJnloEdJDxkVLOI4EALrPs+iene8Tb4WXRKU632YwrwA3OpEk4PCoKLIdocO4p7
X0xt9cEfAgMBAAECggEAB/MtuUxR03OiPxRUIDsD9cVezBKZkH7LDaMVuP08QATi
eItwRXlMpyOr2l9fSkuZgZTHAvYTQCEiygUlN4haMDw1fRxWkq6QsTnGf0sozF6S
IQReREJhaBVu0+FuE4n3llTxA5QIy+9Bhhkt9EVSTCVXbpG481Ni5yilHA0umlE4
opclaOtZC/B9OMKHcjpHg894E99DnWJY3yN9ceAv8cAqTDenm7rsA/vhhY5CR481
uZrTvEdu0SY01rFnNrTffqSz0638euMTS/Zs2MUFfIob4MEM2SBTrMON0ZnIig1U
irlnqf4LW8vwk6tTOwBf7QlIZWgMAmJE4CNDkhTMSQKBgQDWZE0t1kLWUcsZ1l+s
Lk4K5OEjTS1QkGoQAV+STD0MZa8dViXCQ8ZlTd3SuCJlVYN965rwsAZdXxizKaG3
LDcbelzwozVx4RN8cAx5Hr7St/nOxoxneByWRrdAZ3Q+31ze5BWHB0/FNzbQKmrz
VjiAUxQNkXGX0LuLvq8Bt5IpyQKBgQDIEuhxiC9TZr/7eiQap4lkWPsR4dw60Wby
mjOonpHmeWsc2BC/UrQAm88jaHz3jlVR7o/TWSDnr3q6wQeAhMgglDtjeWVjgz0A
/rz730RqeCJy+p0Q+vlbjMsJKbFW31FHpnGWSlNJNR/ukWhY8ZzDXkGBOghl/W4G
sH4YdrgHpwKBgEE0/phra6a99/Ui07SN3OTJWoSfK6IuWmQF1uirUYIcx1c35YIh
lQ0o0sgNg0Co8rZw245LK5RooR2VEv+gbh5oaC40pcO+PmwBtHl+VXHHzS4MC5ZI
3QED/yaSkLOywg909es82RFVytEjpaXfWo4FNrstuov/S5ukYpw2FjmpAoGAIb1u
jZbVxE7TmWkjpYsTVGTdEkrwhMrDfa4bgBgdqqxXL+oQCPO9f0zHRcVdLbJHGSYO
AypIrkmdfNkIltfNmBDnYwMZ4gpDw2MyI5enRf00cgdpbW+llZbMtqLdUTUf0+Fx
zlKCcRoQZH8JBdbZBOlkSpKqz+hJIb+pxB5hMokCgYEAiu3+lrv66I2sFTwI6ZtZ
7JgavFOm82GM5pVObiOrtsKZvmwRF/T5zormjnm4B7VVr7mtxpLtlEQFcvd2fh/6
kHqVAAP1Rlu8ODL/VQKTIialUIMfD4txpwG9yjLst+7BuY8trAWOnDCZ03B/Xu78
xdMrgAdtCksfXBKvBdxn6Xw=
-----END PRIVATE KEY-----
)pem";

constexpr int TLS_HANDSHAKE_TIMEOUT_S = 2;


enum class Tls_Test_Version
{
    TLS_1_1,
    TLS_1_2
};


class Socket_Pair
{
public:
    Socket_Pair()
    {
        int sockets[2] = {-1, -1};
        if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) != 0)
            {
                return;
            }
        d_client = sockets[0];
        d_server = sockets[1];

        struct timeval timeout = {TLS_HANDSHAKE_TIMEOUT_S, 0};
        if (setsockopt(d_client, SOL_SOCKET, SO_RCVTIMEO,
                &timeout, sizeof(timeout)) != 0 ||
            setsockopt(d_client, SOL_SOCKET, SO_SNDTIMEO,
                &timeout, sizeof(timeout)) != 0 ||
            setsockopt(d_server, SOL_SOCKET, SO_RCVTIMEO,
                &timeout, sizeof(timeout)) != 0 ||
            setsockopt(d_server, SOL_SOCKET, SO_SNDTIMEO,
                &timeout, sizeof(timeout)) != 0)
            {
                close(d_client);
                close(d_server);
                d_client = -1;
                d_server = -1;
            }
    }

    ~Socket_Pair()
    {
        if (d_client >= 0)
            {
                close(d_client);
            }
        if (d_server >= 0)
            {
                close(d_server);
            }
    }

    Socket_Pair(const Socket_Pair&) = delete;
    Socket_Pair& operator=(const Socket_Pair&) = delete;

    bool valid() const { return d_client >= 0 && d_server >= 0; }
    int client() const { return d_client; }
    int server() const { return d_server; }

private:
    int d_client = -1;
    int d_server = -1;
};


#ifdef USE_GNUTLS_FALLBACK
std::once_flag tls_test_init_flag;
int tls_test_init_result = GNUTLS_E_SUCCESS;


void initialize_tls_test_backend()
{
    tls_test_init_result = gnutls_global_init();
}


const char* priority_for(Tls_Test_Version version)
{
    return version == Tls_Test_Version::TLS_1_1 ? "NORMAL:-VERS-ALL:+VERS-TLS1.1" : "NORMAL:-VERS-ALL:+VERS-TLS1.2";
}


gnutls_datum_t make_datum(const char* pem)
{
    gnutls_datum_t datum;
    datum.data = reinterpret_cast<unsigned char*>(const_cast<char*>(pem));
    datum.size = static_cast<unsigned int>(std::strlen(pem));
    return datum;
}


class Tls_Test_Server
{
public:
    ~Tls_Test_Server()
    {
        if (d_session != nullptr)
            {
                gnutls_deinit(d_session);
            }
        if (d_credentials != nullptr)
            {
                gnutls_certificate_free_credentials(d_credentials);
            }
    }

    Tls_Test_Server(const Tls_Test_Server&) = delete;
    Tls_Test_Server& operator=(const Tls_Test_Server&) = delete;
    Tls_Test_Server() = default;

    bool initialize(int socket, Tls_Test_Version version)
    {
        std::call_once(tls_test_init_flag, initialize_tls_test_backend);
        if (tls_test_init_result < 0 ||
            gnutls_certificate_allocate_credentials(&d_credentials) < 0)
            {
                return false;
            }

        gnutls_datum_t certificate = make_datum(TLS_TEST_CERTIFICATE);
        gnutls_datum_t private_key = make_datum(TLS_TEST_PRIVATE_KEY);
        if (gnutls_certificate_set_x509_key_mem(d_credentials,
                &certificate, &private_key, GNUTLS_X509_FMT_PEM) < 0 ||
            gnutls_init(&d_session, GNUTLS_SERVER) < 0)
            {
                return false;
            }

        const char* priority_error = nullptr;
        if (gnutls_priority_set_direct(d_session, priority_for(version),
                &priority_error) < 0 ||
            gnutls_credentials_set(d_session, GNUTLS_CRD_CERTIFICATE,
                d_credentials) < 0)
            {
                return false;
            }
#if GNUTLS_VERSION_NUMBER >= 0x03030A
        gnutls_session_set_ptr(d_session, this);
        gnutls_handshake_set_hook_function(d_session,
            GNUTLS_HANDSHAKE_SERVER_HELLO, GNUTLS_HOOK_POST,
            &Tls_Test_Server::server_hello_hook);
#endif
        gnutls_transport_set_int(d_session, socket);
        return true;
    }

    void handshake()
    {
        d_handshake_result = gnutls_handshake(d_session);
        if (d_handshake_result < 0)
            {
                d_protocol_version_rejected =
                    gnutls_alert_get(d_session) == GNUTLS_A_PROTOCOL_VERSION;
                d_error = gnutls_strerror(d_handshake_result);
            }
    }

    bool server_hello_sent() const { return d_server_hello_sent; }
    bool handshake_completed() const { return d_handshake_result == 0; }
    bool protocol_version_rejected() const { return d_protocol_version_rejected; }
    const std::string& error() const { return d_error; }

private:
#if GNUTLS_VERSION_NUMBER >= 0x03030A
    static int server_hello_hook(gnutls_session_t session,
        unsigned int handshake_type, unsigned int when,
        unsigned int incoming, const gnutls_datum_t* message)
    {
        static_cast<void>(handshake_type);
        static_cast<void>(when);
        static_cast<void>(message);
        Tls_Test_Server* server = static_cast<Tls_Test_Server*>(
            gnutls_session_get_ptr(session));
        if (server != nullptr && incoming == 0)
            {
                server->d_server_hello_sent = true;
            }
        return 0;
    }
#endif

    gnutls_certificate_credentials_t d_credentials = nullptr;
    gnutls_session_t d_session = nullptr;
    int d_handshake_result = GNUTLS_E_INTERNAL_ERROR;
    bool d_server_hello_sent = false;
    bool d_protocol_version_rejected = false;
    std::string d_error;
};


bool native_client_handshake(int socket, Tls_Test_Version version)
{
    gnutls_certificate_credentials_t credentials = nullptr;
    gnutls_session_t session = nullptr;
    if (gnutls_certificate_allocate_credentials(&credentials) < 0 ||
        gnutls_init(&session, GNUTLS_CLIENT) < 0)
        {
            if (credentials != nullptr)
                {
                    gnutls_certificate_free_credentials(credentials);
                }
            return false;
        }

    const char* priority_error = nullptr;
    const bool configured =
        gnutls_priority_set_direct(session, priority_for(version),
            &priority_error) >= 0 &&
        gnutls_credentials_set(session, GNUTLS_CRD_CERTIFICATE,
            credentials) >= 0;
    if (configured)
        {
            gnutls_transport_set_int(session, socket);
        }
    const bool connected = configured && gnutls_handshake(session) == 0;
    gnutls_deinit(session);
    gnutls_certificate_free_credentials(credentials);
    return connected;
}

#else
std::once_flag tls_test_init_flag;


void initialize_tls_test_backend()
{
#if OPENSSL_VERSION_NUMBER < 0x10100000L
    SSL_library_init();
    SSL_load_error_strings();
#endif
}


int protocol_for(Tls_Test_Version version)
{
    return version == Tls_Test_Version::TLS_1_1 ? TLS1_1_VERSION : TLS1_2_VERSION;
}


bool set_exact_protocol(SSL_CTX* context, Tls_Test_Version version)
{
    const int protocol = protocol_for(version);
#if OPENSSL_VERSION_NUMBER >= 0x10100000L
    SSL_CTX_set_security_level(context, 0);
    return SSL_CTX_set_min_proto_version(context, protocol) == 1 &&
           SSL_CTX_set_max_proto_version(context, protocol) == 1;
#else
    long options = SSL_OP_NO_SSLv2 | SSL_OP_NO_SSLv3;
    if (version == Tls_Test_Version::TLS_1_1)
        {
            options |= SSL_OP_NO_TLSv1 | SSL_OP_NO_TLSv1_2;
        }
    else
        {
            options |= SSL_OP_NO_TLSv1 | SSL_OP_NO_TLSv1_1;
        }
    SSL_CTX_set_options(context, options);
    return true;
#endif
}


bool install_test_identity(SSL_CTX* context)
{
    BIO* certificate_bio = BIO_new_mem_buf(
        const_cast<char*>(TLS_TEST_CERTIFICATE), -1);
    BIO* private_key_bio = BIO_new_mem_buf(
        const_cast<char*>(TLS_TEST_PRIVATE_KEY), -1);
    if (certificate_bio == nullptr || private_key_bio == nullptr)
        {
            BIO_free(certificate_bio);
            BIO_free(private_key_bio);
            return false;
        }

    X509* certificate = PEM_read_bio_X509(
        certificate_bio, nullptr, nullptr, nullptr);
    EVP_PKEY* private_key = PEM_read_bio_PrivateKey(
        private_key_bio, nullptr, nullptr, nullptr);
    BIO_free(certificate_bio);
    BIO_free(private_key_bio);
    if (certificate == nullptr || private_key == nullptr)
        {
            X509_free(certificate);
            EVP_PKEY_free(private_key);
            return false;
        }

    const bool installed = SSL_CTX_use_certificate(context, certificate) == 1 &&
                           SSL_CTX_use_PrivateKey(context, private_key) == 1 &&
                           SSL_CTX_check_private_key(context) == 1;
    X509_free(certificate);
    EVP_PKEY_free(private_key);
    return installed;
}


class Tls_Test_Server
{
public:
    ~Tls_Test_Server()
    {
        if (d_session != nullptr)
            {
                SSL_free(d_session);
            }
        if (d_context != nullptr)
            {
                SSL_CTX_free(d_context);
            }
    }

    Tls_Test_Server(const Tls_Test_Server&) = delete;
    Tls_Test_Server& operator=(const Tls_Test_Server&) = delete;
    Tls_Test_Server() = default;

    bool initialize(int socket, Tls_Test_Version version)
    {
        std::call_once(tls_test_init_flag, initialize_tls_test_backend);
#if OPENSSL_VERSION_NUMBER >= 0x10100000L
        d_context = SSL_CTX_new(TLS_server_method());
#else
        d_context = SSL_CTX_new(SSLv23_server_method());
#endif
        if (d_context == nullptr ||
            !set_exact_protocol(d_context, version) ||
            SSL_CTX_set_cipher_list(d_context, "ALL") != 1 ||
            !install_test_identity(d_context))
            {
                return false;
            }

        d_session = SSL_new(d_context);
        if (d_session == nullptr || SSL_set_fd(d_session, socket) != 1)
            {
                return false;
            }
        SSL_set_msg_callback(d_session, &Tls_Test_Server::message_callback);
        SSL_set_msg_callback_arg(d_session, this);
        SSL_set_accept_state(d_session);
        return true;
    }

    void handshake()
    {
        d_handshake_result = SSL_accept(d_session);
        if (d_handshake_result != 1)
            {
                const unsigned long error = ERR_peek_last_error();
                if (error != 0)
                    {
                        char message[256] = "";
                        ERR_error_string_n(error, message, sizeof(message));
                        d_error = message;
                    }
            }
    }

    bool server_hello_sent() const { return d_server_hello_sent; }
    bool handshake_completed() const { return d_handshake_result == 1; }
    bool protocol_version_rejected() const { return d_protocol_version_rejected; }
    const std::string& error() const { return d_error; }

private:
    static void message_callback(int write_direction, int,
        int content_type, const void* buffer, std::size_t length,
        SSL* session, void* argument)
    {
        static_cast<void>(session);
        Tls_Test_Server* server = static_cast<Tls_Test_Server*>(argument);
        const unsigned char* message = static_cast<const unsigned char*>(buffer);
        if (server != nullptr && write_direction != 0 &&
            content_type == SSL3_RT_HANDSHAKE && length > 0 &&
            message[0] == SSL3_MT_SERVER_HELLO)
            {
                server->d_server_hello_sent = true;
            }
        if (server != nullptr && write_direction == 0 &&
            content_type == SSL3_RT_ALERT && length >= 2 &&
            message[1] == TLS1_AD_PROTOCOL_VERSION)
            {
                server->d_protocol_version_rejected = true;
            }
    }

    SSL_CTX* d_context = nullptr;
    SSL* d_session = nullptr;
    int d_handshake_result = 0;
    bool d_server_hello_sent = false;
    bool d_protocol_version_rejected = false;
    std::string d_error;
};


bool native_client_handshake(int socket, Tls_Test_Version version)
{
    std::call_once(tls_test_init_flag, initialize_tls_test_backend);
#if OPENSSL_VERSION_NUMBER >= 0x10100000L
    SSL_CTX* context = SSL_CTX_new(TLS_client_method());
#else
    SSL_CTX* context = SSL_CTX_new(SSLv23_client_method());
#endif
    if (context == nullptr || !set_exact_protocol(context, version) ||
        SSL_CTX_set_cipher_list(context, "ALL") != 1)
        {
            SSL_CTX_free(context);
            return false;
        }
    SSL_CTX_set_verify(context, SSL_VERIFY_NONE, nullptr);

    SSL* session = SSL_new(context);
    const bool configured = session != nullptr &&
                            SSL_set_fd(session, socket) == 1;
    if (configured)
        {
            SSL_set_connect_state(session);
        }
    const bool connected = configured && SSL_connect(session) == 1;
    SSL_free(session);
    SSL_CTX_free(context);
    return connected;
}
#endif


struct Tls_Handshake_Result
{
    bool initialized = false;
    bool server_hello_sent = false;
    bool server_handshake_completed = false;
    bool protocol_version_rejected = false;
    int client_result = -2;
    std::string client_message;
    std::string server_error;
};


bool backend_supports(Tls_Test_Version version)
{
    Socket_Pair sockets;
    if (!sockets.valid())
        {
            return false;
        }

    Tls_Test_Server server;
    if (!server.initialize(sockets.server(), version))
        {
            return false;
        }
    std::thread server_thread(&Tls_Test_Server::handshake, &server);
    const bool client_connected = native_client_handshake(
        sockets.client(), version);
    server_thread.join();
    return client_connected && server.handshake_completed() &&
           server.server_hello_sent();
}


Tls_Handshake_Result run_rtklib_handshake(Tls_Test_Version version)
{
    Tls_Handshake_Result result;
    Socket_Pair sockets;
    if (!sockets.valid())
        {
            result.server_error = "socketpair setup failed";
            return result;
        }

    Tls_Test_Server server;
    if (!server.initialize(sockets.server(), version))
        {
            result.server_error = "TLS server setup failed";
            return result;
        }

    Rtklib_Tls_Client client("localhost");
    char message[MAXSTRMSG] = "";
    if (!client.initialize(message))
        {
            result.server_error = message;
            return result;
        }
    result.initialized = true;

    std::thread server_thread(&Tls_Test_Server::handshake, &server);
    result.client_result = client.handshake(sockets.client(), message);
    shutdown(sockets.client(), SHUT_RDWR);
    server_thread.join();

    result.server_hello_sent = server.server_hello_sent();
    result.server_handshake_completed = server.handshake_completed();
    result.protocol_version_rejected = server.protocol_version_rejected();
    result.client_message = message;
    result.server_error = server.error();
    return result;
}
}  // namespace rtklib_tls_test


TEST(RtklibTlsTest, RejectsTls11OnlyServer)
{
    using namespace rtklib_tls_test;

    if (!backend_supports(Tls_Test_Version::TLS_1_1))
        {
            GTEST_SKIP() << "The active TLS backend cannot enable TLS 1.1";
        }

    const Tls_Handshake_Result result =
        run_rtklib_handshake(Tls_Test_Version::TLS_1_1);
    ASSERT_TRUE(result.initialized) << result.server_error;
    EXPECT_EQ(-1, result.client_result) << result.client_message;
#ifdef USE_GNUTLS_FALLBACK
#if GNUTLS_VERSION_NUMBER >= 0x03030A
    EXPECT_TRUE(result.server_hello_sent)
        << "The TLS 1.1 test server did not send its ServerHello: "
        << result.server_error;
#endif
    EXPECT_EQ("GnuTLS handshake failed", result.client_message)
        << "The client progressed past TLS 1.1 negotiation";
#else
    EXPECT_TRUE(!result.server_hello_sent ||
                result.protocol_version_rejected)
        << "The client progressed past the server's TLS 1.1 selection";
#endif
    EXPECT_FALSE(result.server_handshake_completed);
}


TEST(RtklibTlsTest, AllowsTls12Negotiation)
{
    using namespace rtklib_tls_test;

    ASSERT_TRUE(backend_supports(Tls_Test_Version::TLS_1_2))
        << "The active TLS backend failed its TLS 1.2 control handshake";

    const Tls_Handshake_Result result =
        run_rtklib_handshake(Tls_Test_Version::TLS_1_2);
    ASSERT_TRUE(result.initialized) << result.server_error;
#if !defined(USE_GNUTLS_FALLBACK) || GNUTLS_VERSION_NUMBER >= 0x03030A
    EXPECT_TRUE(result.server_hello_sent)
        << "The client did not negotiate TLS 1.2: " << result.server_error;
#endif
    EXPECT_FALSE(result.protocol_version_rejected)
        << "The client rejected the server's TLS 1.2 selection";
    EXPECT_EQ(-1, result.client_result)
        << "The self-signed test certificate should remain untrusted";
#ifdef USE_GNUTLS_FALLBACK
    EXPECT_TRUE(result.server_handshake_completed)
        << "The TLS 1.2 handshake did not reach certificate validation: "
        << result.server_error;
#endif
}
