/*!
 * \file supl.h
 * \brief SUPL library with some RRLP
 * \author Carles Fernandez, 2017 cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (c) 2007 Tatu Mannisto <tatu a-t tajuma d-o-t com>
 * SPDX-License-Identifier: BSD-1-Clause
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_SUPL_H
#define GNSS_SDR_SUPL_H

#ifdef USE_EXPORT
#define EXPORT __attribute__((visibility("default")))
#else
#define EXPORT
#endif
// clang-format off
#if USE_OPENSSL_FALLBACK
#include <openssl/crypto.h>
#include <openssl/x509.h>
#include <openssl/pem.h>
#include <openssl/ssl.h>
#include <openssl/err.h>
#else
#include <gnutls/gnutls.h>
#include <gnutls/compat.h>
#include <gnutls/crypto.h>
#include <gnutls/openssl.h>
#include <gnutls/x509.h>
#endif
// clang-format on
#include <PDU.h>
#include <ULP-PDU.h>


/** \addtogroup Core
 * \{ */
/** \addtogroup SUPL_Library core_libs_supl
 * \{ */


#define SUPL_PORT "7275"

/* error messages */

#define E_SUPL_CONNECT (-1)
#define E_SUPL_ENCODE_START (-2)
#define E_SUPL_RECV_RESPONSE (-3)
#define E_SUPL_SUPLRESPONSE (-4)
#define E_SUPL_ENCODE_POSINIT (-5)
#define E_SUPL_RECV_SUPLPOS (-6)
#define E_SUPL_SUPLPOS (-7)
#define E_SUPL_DECODE_RRLP (-8)
#define E_SUPL_RRLP_ACK (-9)
#define E_SUPL_ENCODE (-10)
#define E_SUPL_WRITE (-11)
#define E_SUPL_READ (-12)
#define E_SUPL_INTERNAL (-13)
#define E_SUPL_DECODE (-14)
#define E_SUPL_ENCODE_RRLP (-15)

/* diagnostic & debug values */
#define SUPL_DEBUG_RRLP 1
#define SUPL_DEBUG_SUPL 2
#define SUPL_DEBUG_DEBUG 4

/* flags for additional assistance requests */
#define SUPL_REQUEST_ALMANAC 1

/* flags for collected assist data elements */
#define SUPL_RRLP_ASSIST_REFTIME (1)
#define SUPL_RRLP_ASSIST_REFLOC (2)
#define SUPL_RRLP_ASSIST_IONO (4)
#define SUPL_RRLP_ASSIST_EPHEMERIS (8)
#define SUPL_RRLP_ASSIST_UTC (16)

#define SUPL_ACQUIS_DOPPLER (1)
#define SUPL_ACQUIS_ANGLE (2)

#define MAX_EPHEMERIS 32

struct supl_acquis_s
{
    u_int8_t prn;
    u_int8_t parts;
    int16_t doppler0;
    int8_t doppler1;
    u_int8_t d_win;
    u_int16_t code_ph;
    u_int8_t code_ph_int;
    u_int8_t bit_num;
    u_int16_t code_ph_win;
    u_int8_t az;
    u_int8_t el;
    u_int8_t fill[2];
};

struct supl_almanac_s
{
    u_int8_t prn;
    u_int16_t e;
    u_int8_t toa;
    int16_t Ksii;
    int16_t OMEGA_dot;
    u_int32_t A_sqrt;
    int32_t OMEGA_0;
    int32_t w;
    int32_t M0;
    int16_t AF0;
    int16_t AF1;
    // int32_t health;
};

struct supl_ephemeris_s
{
    u_int8_t prn;
    u_int8_t fill1;
    u_int16_t delta_n;
    int32_t M0;
    u_int32_t e;
    u_int32_t A_sqrt;
    int32_t OMEGA_0;
    int32_t i0;
    int32_t w;
    int32_t OMEGA_dot;
    int16_t i_dot;
    int16_t Cuc;
    int16_t Cus;
    int16_t Crc;
    int16_t Crs;
    int16_t Cic;
    int16_t Cis;
    u_int16_t toe;
    u_int16_t IODC;
    u_int16_t toc;
    int32_t AF0;
    int16_t AF1;
    int8_t AF2;
    u_int8_t nav_model;

    /* nav model */
    u_int8_t bits;
    u_int8_t ura;
    u_int8_t health;
    char reserved[11];
    int8_t tgd;
    u_int8_t AODA;
};

struct supl_ionospheric_s
{
    int8_t a0, a1, a2, a3, b0, b1, b2, b3;
};

struct supl_utc_s
{
    int32_t a0;
    int32_t a1;
    int8_t delta_tls;
    u_int8_t tot;
    u_int8_t wnt;
    u_int8_t wnlsf;
    u_int8_t dn;
    u_int8_t delta_tlsf;
    u_int8_t fill[8];
};

typedef struct supl_rrlp_ctx_s
{
    int set;

    struct
    {
        long gps_tow, gps_week;
        struct timeval stamp;
    } time;

    struct
    {
        int uncertainty;
        double lat, lon; /* of the base station */
    } pos;

    struct supl_ionospheric_s iono;

    struct supl_utc_s utc;

    int cnt_eph;
    struct supl_ephemeris_s eph[MAX_EPHEMERIS];

    int cnt_alm;
    int alm_week;
    struct supl_almanac_s alm[MAX_EPHEMERIS];

    int cnt_acq;
    int acq_time;
    struct supl_acquis_s acq[MAX_EPHEMERIS];
} supl_assist_t;

typedef struct supl_param_s
{
    int set;
    int request;

    struct
    {
        int mcc, mnc, lac, ci;
    } gsm;

    struct
    {
        int mcc, mnc, uc;
    } wcdma;

    struct
    {
        int mcc, mnc, lac, ci;
        double lat, lon;
        int uncert;
    } known;

    char msisdn[8];
} supl_param_t;

typedef struct supl_ctx_s
{
    supl_param_t p;

    int fd;
    SSL *ssl;
    SSL_CTX *ssl_ctx;

    struct
    {
        void *buf;
        size_t size;
    } slp_session_id;
} supl_ctx_t;

int supl_ctx_new(supl_ctx_t *ctx);
int supl_ctx_free(supl_ctx_t *ctx);

void supl_set_gsm_cell(supl_ctx_t *ctx, int mcc, int mns, int lac, int ci);
void supl_set_wcdma_cell(supl_ctx_t *ctx, int mcc, int mns, int uc);
void supl_set_gsm_cell_known(supl_ctx_t *ctx, int mcc, int mns, int lac, int ci, double lat, double lon, int uncert);
void supl_set_server(supl_ctx_t *ctx, char *server);
void supl_set_fd(supl_ctx_t *ctx, int fd);
void supl_request(supl_ctx_t *ctx, int request);

int supl_get_assist(supl_ctx_t *ctx, char *server, supl_assist_t *assist);
void supl_set_debug(FILE *log, int flags);

/*
** stuff above should be enough for supl client implementation
*/

typedef void (*supl_debug_cb)(char format, ...);

typedef struct supl_ulp_s
{
    ULP_PDU_t *pdu;
    size_t size;
    unsigned char buffer[8192];
} supl_ulp_t;

typedef struct supl_rrlp_s
{
    PDU_t *pdu;
    size_t size;
    unsigned char buffer[8192];
} supl_rrlp_t;

void supl_ulp_free(supl_ulp_t *pdu);
int supl_ulp_encode(supl_ulp_t *pdu);
int supl_ulp_decode(supl_ulp_t *pdu);
int supl_decode_rrlp(supl_ulp_t *pdu, PDU_t **rrlp);
int supl_collect_rrlp(supl_assist_t *assist, PDU_t *rrlp, struct timeval *t);

int supl_server_connect(supl_ctx_t *ctx, char *server);
void supl_close(supl_ctx_t *ctx);
int supl_ulp_send(supl_ctx_t *ctx, supl_ulp_t *pdu);
int supl_ulp_recv(supl_ctx_t *ctx, supl_ulp_t *pdu);


/** \} */
/** \} */
#endif  // GNSS_SDR_SUPL_H
