/*!
 * \file supl.c
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

#include "supl.h"
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>


#define PARAM_GSM_CELL_CURRENT 1
#define PARAM_GSM_CELL_KNOWN 2
#define PARAM_WCDMA_CELL_CURRENT 4

#define OPTIONAL_MISSING ((void *)0)

#ifdef SUPL_DEBUG
static struct supl_debug_s
{
    FILE *log;
    int verbose_rrlp, verbose_supl, debug;
    int sent, recv, out_msg, in_msg;
} debug;
#endif

static int server_connect(char *server);
static int pdu_make_ulp_start(supl_ctx_t *ctx, supl_ulp_t *pdu);
static int pdu_make_ulp_pos_init(supl_ctx_t *ctx, supl_ulp_t *pdu);
static int pdu_make_ulp_rrlp_ack(supl_ctx_t *ctx, supl_ulp_t *pdu, PDU_t *rrlp);
static int supl_more_rrlp(PDU_t *rrlp);
static int supl_response_harvest(supl_ctx_t *ctx, supl_ulp_t *pdu);

int EXPORT supl_ulp_decode(supl_ulp_t *pdu)
{
    ULP_PDU_t *ulp;
    asn_codec_ctx_t ctx;
    asn_dec_rval_t rval;

    ulp = calloc(1, sizeof(ULP_PDU_t));
    pdu->pdu = ulp;

    ctx.max_stack_size = 0;
    rval = uper_decode_complete(&ctx, &asn_DEF_ULP_PDU, (void **)&ulp, pdu->buffer, pdu->size);
    if (rval.code == RC_OK)
        {
            return 0;
        }

    free(ulp);
    pdu->pdu = 0;

    return E_SUPL_DECODE;
}


int EXPORT supl_ulp_encode(supl_ulp_t *pdu)
{
    asn_enc_rval_t ret;
    int pdu_len;

    ret = uper_encode_to_buffer(&asn_DEF_ULP_PDU, pdu->pdu, pdu->buffer, sizeof(pdu->buffer));
    if (ret.encoded != -1)
        {
            memset(pdu->buffer, 0, sizeof(pdu->buffer));

            pdu_len = (ret.encoded + 7) >> 3;
            (pdu->pdu)->length = pdu_len;

            ret = uper_encode_to_buffer(&asn_DEF_ULP_PDU, pdu->pdu, pdu->buffer, sizeof(pdu->buffer));
            if (ret.encoded > 0)
                {
                    int len = (ret.encoded + 7) >> 3;

                    if (len == pdu_len)
                        {
                            pdu->size = pdu_len;
                            return 0;
                        }
                }
        }

    return E_SUPL_ENCODE;
}


void EXPORT supl_ulp_free(supl_ulp_t *pdu)
{
    asn_DEF_ULP_PDU.free_struct(&asn_DEF_ULP_PDU, pdu->pdu, 0);
}


int EXPORT supl_ulp_send(supl_ctx_t *ctx, supl_ulp_t *pdu)
{
    int err;

#if SUPL_DEBUG
    if (debug.verbose_supl)
        {
            fprintf(debug.log, "Send %lu bytes\n", pdu->size);
            xer_fprint(debug.log, &asn_DEF_ULP_PDU, pdu->pdu);
        }
#endif

    err = SSL_write(ctx->ssl, pdu->buffer, pdu->size);
    if (err <= 0)
        {
#if SUPL_DEBUG
            if (debug.debug) fprintf(debug.log, "Error: SSL_write error: %s\n", strerror(errno));
#endif
            return E_SUPL_WRITE;
        }

#ifdef SUPL_DEBUG
    debug.sent += pdu->size;
    debug.out_msg++;
#endif

    return 0;
}


int EXPORT supl_ulp_recv(supl_ctx_t *ctx, supl_ulp_t *pdu)
{
    int64_t err;
    int64_t n;
    asn_dec_rval_t rval;
    ULP_PDU_t *length;

    err = SSL_read(ctx->ssl, pdu->buffer, sizeof(pdu->buffer));
    if (err <= 0)
        {
#ifdef SUPL_DEBUG
            if (debug.debug) fprintf(debug.log, "Error: SSL_read error: %s\n", strerror(errno));
#endif
            return E_SUPL_READ;
        }
    n = err;

    length = calloc(1, sizeof(ULP_PDU_t));
    // decode the very first bytes of the ULP_PDU message, just enough to the get message length
    rval = uper_decode_complete(0, &asn_DEF_ULP_PDU, (void **)&length, pdu->buffer, n < 6 ? n : 6);
    if (rval.code == RC_WMORE)
        {
            // read the missing data
            for (n = err; n < length->length; n += err)
                {
#ifdef SUPL_DEBUG
                    if (debug.debug) fprintf(debug.log, "SSL_read got %u bytes (total %lu)\n", n, length->length);
#endif
                    err = SSL_read(ctx->ssl, &pdu->buffer[n], sizeof(pdu->buffer) - n);
                    if (err <= 0)
                        {
#ifdef SUPL_DEBUG
                            if (debug.debug) fprintf(debug.log, "Error: SSL_read (again) error: %s\n", strerror(errno));
#endif
                            return E_SUPL_READ;
                        }
                }
        }

    asn_DEF_ULP_PDU.free_struct(&asn_DEF_ULP_PDU, length, 0);

    pdu->size = n;

    if (supl_ulp_decode(pdu))
        {
            return E_SUPL_DECODE;
        }

#ifdef SUPL_DEBUG
    if (debug.verbose_supl)
        {
            fprintf(debug.log, "Recv %lu bytes\n", pdu->size);
            xer_fprint(debug.log, &asn_DEF_ULP_PDU, pdu->pdu);
        }
#endif

#ifdef SUPL_DEBUG
    debug.recv += err;
    debug.in_msg++;
#endif

    return 0;
}


int EXPORT supl_decode_rrlp(supl_ulp_t *ulp_pdu, PDU_t **ret_rrlp)
{
    asn_dec_rval_t rval;
    OCTET_STRING_t *rrlp_pdu;
    PDU_t *rrlp;
    ULP_PDU_t *ulp;

    ulp = ulp_pdu->pdu;

    if (!(ulp->message.present == UlpMessage_PR_msSUPLPOS &&
            ulp->message.choice.msSUPLPOS.posPayLoad.present == PosPayLoad_PR_rrlpPayload))
        {
            return 0;
        }
    rrlp_pdu = &ulp->message.choice.msSUPLPOS.posPayLoad.choice.rrlpPayload;

    rrlp = calloc(1, sizeof(PDU_t));
    rval = uper_decode_complete(0, &asn_DEF_PDU, (void **)&rrlp, rrlp_pdu->buf, rrlp_pdu->size);
    switch (rval.code)
        {
        case RC_OK:
#ifdef SUPL_DEBUG
            if (rval.consumed != rrlp_pdu->size)
                {
                    if (debug.debug) fprintf(debug.log, "Warning: %lu bytes left over in RRLP decoding\n", rval.consumed);
                }
#endif

            *ret_rrlp = rrlp;
            return 0;

        case RC_WMORE:
            asn_DEF_PDU.free_struct(&asn_DEF_PDU, rrlp, 0);
            ret_rrlp = 0;
            return E_SUPL_DECODE_RRLP;

        default:
            asn_DEF_PDU.free_struct(&asn_DEF_PDU, rrlp, 0);
            ret_rrlp = 0;
            return E_SUPL_DECODE_RRLP;
        }

    return E_SUPL_INTERNAL;
}


int EXPORT supl_server_connect(supl_ctx_t *ctx, char *server)
{
    int err;
    SSL_METHOD *meth;

    SSLeay_add_ssl_algorithms();
    // meth = TLSv1_client_method();
    meth = (SSL_METHOD *)SSLv23_client_method();
    SSL_load_error_strings();
    ctx->ssl_ctx = SSL_CTX_new(meth);
    if (!ctx->ssl_ctx)
        {
            return E_SUPL_CONNECT;
        }

    ctx->ssl = SSL_new(ctx->ssl_ctx);
    if (!ctx->ssl)
        {
            return E_SUPL_CONNECT;
        }

    if (server)
        {
            ctx->fd = server_connect(server);
            if (ctx->fd == -1)
                {
                    return E_SUPL_CONNECT;
                }
        }

    SSL_set_fd(ctx->ssl, ctx->fd);
    err = SSL_connect(ctx->ssl);
    if (err == -1)
        {
            return E_SUPL_CONNECT;
        }

#if 0
  {
    X509 *s_cert = SSL_get_peer_certificate(ctx->ssl);
    FILE *fp = fopen("/tmp/s_cert.pem", "w");
    if (fp)
      PEM_write_X509(fp, s_cert);
    fclose(fp);
    X509_free(s_cert);
  }
#endif

    return 0;
}


void EXPORT supl_close(supl_ctx_t *ctx)
{
    SSL_shutdown(ctx->ssl);
    SSL_free(ctx->ssl);
    SSL_CTX_free(ctx->ssl_ctx);
    close(ctx->fd);
}


static int server_connect(char *server)
{
    int fd = -1;
    struct addrinfo *ailist;
    struct addrinfo *aip;
    struct addrinfo hint;
    int err;

    memset(&hint, 0, sizeof(struct addrinfo));
    hint.ai_socktype = SOCK_STREAM;
    err = getaddrinfo(server, SUPL_PORT, &hint, &ailist);
    if (err != 0)
        {
            return -1;
        }

    for (aip = ailist; aip; aip = aip->ai_next)
        {
            if ((fd = socket(aip->ai_family, SOCK_STREAM, 0)) < 0)
                {
                    err = errno;
                }
            if (connect(fd, aip->ai_addr, aip->ai_addrlen) != 0)
                {
                    freeaddrinfo(aip);
                    if (close(fd) != 0)
                        {
                            // avoid warning
                        }
                    return -1;
                }
            break;
        }
    freeaddrinfo(aip);
    return fd;
}


static int pdu_make_ulp_start(supl_ctx_t *ctx, supl_ulp_t *pdu)
{
    ULP_PDU_t *ulp;
    SetSessionID_t *session_id;
    int err;

    ulp = calloc(1, sizeof(ULP_PDU_t));
    session_id = calloc(1, sizeof(SetSessionID_t));

    ulp->length = 0;
    ulp->version.maj = 1;
    ulp->version.min = 0;
    ulp->version.servind = 0;

    session_id->sessionId = 1;
    // session_id->setId.present = SETId_PR_msisdn;
    // (void)OCTET_STRING_fromBuf(&session_id->setId.choice.msisdn, ctx->p.msisdn, 8);
    session_id->setId.present = SETId_PR_imsi;
    (void)OCTET_STRING_fromBuf(&session_id->setId.choice.imsi, ctx->p.msisdn, 8);

    ulp->sessionID.setSessionID = session_id;
    ulp->sessionID.slpSessionID = OPTIONAL_MISSING;

    ulp->message.present = UlpMessage_PR_msSUPLSTART;
    ulp->message.choice.msSUPLSTART.sETCapabilities.posTechnology.agpsSETBased = 1;
    // (void)asn_long2INTEGER(&ulp->message.choice.msSUPLSTART.sETCapabilities.prefMethod, PrefMethod_noPreference);
    (void)asn_long2INTEGER(&ulp->message.choice.msSUPLSTART.sETCapabilities.prefMethod, PrefMethod_agpsSETBasedPreferred);
    ulp->message.choice.msSUPLSTART.sETCapabilities.posProtocol.rrlp = 1;

    if (ctx->p.set & PARAM_GSM_CELL_CURRENT)
        {
            ulp->message.choice.msSUPLSTART.locationId.cellInfo.present = CellInfo_PR_gsmCell;
            ulp->message.choice.msSUPLSTART.locationId.cellInfo.choice.gsmCell.refMCC = ctx->p.gsm.mcc;
            ulp->message.choice.msSUPLSTART.locationId.cellInfo.choice.gsmCell.refMNC = ctx->p.gsm.mnc;
            ulp->message.choice.msSUPLSTART.locationId.cellInfo.choice.gsmCell.refLAC = ctx->p.gsm.lac;
            ulp->message.choice.msSUPLSTART.locationId.cellInfo.choice.gsmCell.refCI = ctx->p.gsm.ci;
            ulp->message.choice.msSUPLSTART.locationId.cellInfo.choice.gsmCell.nMR = OPTIONAL_MISSING;
            ulp->message.choice.msSUPLSTART.locationId.cellInfo.choice.gsmCell.tA = OPTIONAL_MISSING;
        }
    else if (ctx->p.set & PARAM_WCDMA_CELL_CURRENT)
        {
            ulp->message.choice.msSUPLSTART.locationId.cellInfo.present = CellInfo_PR_wcdmaCell;
            ulp->message.choice.msSUPLSTART.locationId.cellInfo.choice.wcdmaCell.refMCC = ctx->p.wcdma.mcc;
            ulp->message.choice.msSUPLSTART.locationId.cellInfo.choice.wcdmaCell.refMNC = ctx->p.wcdma.mnc;
            ulp->message.choice.msSUPLSTART.locationId.cellInfo.choice.wcdmaCell.refUC = ctx->p.wcdma.uc;
        }

    (void)asn_long2INTEGER(&ulp->message.choice.msSUPLSTART.locationId.status, Status_current);

    ulp->message.choice.msSUPLSTART.qoP = OPTIONAL_MISSING;

    pdu->pdu = ulp;

    err = supl_ulp_encode(pdu);
    if (err < 0)
        {
            asn_DEF_ULP_PDU.free_struct(&asn_DEF_ULP_PDU, ulp, 0);
            return err;
        }

    return 0;
}


// get slpSessionID from SUPLRESPONSE pdu if preset
static int supl_response_harvest(supl_ctx_t *ctx, supl_ulp_t *pdu)
{
    int ret;
    ULP_PDU_t *ulp = pdu->pdu;
    void *buf;

    ctx->slp_session_id.buf = 0;

    ret = uper_encode_to_new_buffer(&asn_DEF_SlpSessionID, 0, (void *)ulp->sessionID.slpSessionID, &buf);
    if (ret == -1)
        {
            return -1;
        }

    ctx->slp_session_id.buf = buf;
    ctx->slp_session_id.size = ret;

    return 0;
}


static int pdu_make_ulp_pos_init(supl_ctx_t *ctx, supl_ulp_t *pdu)
{
    int err;
    ULP_PDU_t *ulp;
    SetSessionID_t *session_id;
    RequestedAssistData_t *req_adata;

    ulp = calloc(1, sizeof(ULP_PDU_t));
    session_id = calloc(1, sizeof(SetSessionID_t));
    req_adata = calloc(1, sizeof(RequestedAssistData_t));

    ulp->length = 0;
    ulp->version.maj = 1;
    ulp->version.min = 0;
    ulp->version.servind = 0;

    session_id->sessionId = 1;
    // session_id->setId.present = SETId_PR_msisdn;
    // (void)OCTET_STRING_fromBuf(&session_id->setId.choice.msisdn, ctx->p.msisdn, 8);
    session_id->setId.present = SETId_PR_imsi;
    (void)OCTET_STRING_fromBuf(&session_id->setId.choice.imsi, ctx->p.msisdn, 8);

    ulp->sessionID.setSessionID = session_id;
    // ulp->sessionID.slpSessionID = OPTIONAL_MISSING;
    if (ctx->slp_session_id.buf)
        {
            (void)uper_decode_complete(0, &asn_DEF_SlpSessionID, (void **)&ulp->sessionID.slpSessionID, ctx->slp_session_id.buf, ctx->slp_session_id.size);
        }
    else
        {
            ulp->sessionID.slpSessionID = OPTIONAL_MISSING;
        }

    ulp->message.present = UlpMessage_PR_msSUPLPOSINIT;
    ulp->message.choice.msSUPLPOSINIT.sETCapabilities.posTechnology.agpsSETBased = 1;
    // (void)asn_long2INTEGER(&ulp->message.choice.msSUPLPOSINIT.sETCapabilities.prefMethod, PrefMethod_noPreference);
    (void)asn_long2INTEGER(&ulp->message.choice.msSUPLPOSINIT.sETCapabilities.prefMethod, PrefMethod_agpsSETBasedPreferred);
    ulp->message.choice.msSUPLPOSINIT.sETCapabilities.posProtocol.rrlp = 1;

    // GNSS-SDR mod
    //  Use ctx->p.request to switch between a pre-defined set of assistance data request
    //  reason: Some SUPL servers do not respond to Acquisition assistance depending on the status of other assistance flags

    switch (ctx->p.request)
        {
        case 0:                                             // request almanac, time, and cell positions
            req_adata->acquisitionAssistanceRequested = 0;  // 1
            req_adata->navigationModelRequested = 0;        // 1
            req_adata->referenceTimeRequested = 1;
            req_adata->utcModelRequested = 1;          // 1
            req_adata->ionosphericModelRequested = 1;  // 1
            req_adata->referenceLocationRequested = 1;
            req_adata->almanacRequested = 1;
            req_adata->realTimeIntegrityRequested = 1;  // 1
            break;
        case 1:                                             // request Navigation Model (Ephemeris)
            req_adata->acquisitionAssistanceRequested = 0;  // 1
            req_adata->navigationModelRequested = 1;        // 1
            req_adata->referenceTimeRequested = 1;
            req_adata->utcModelRequested = 0;          // 1
            req_adata->ionosphericModelRequested = 0;  // 1
            req_adata->referenceLocationRequested = 0;
            req_adata->almanacRequested = 0;
            req_adata->realTimeIntegrityRequested = 0;  // 1
            break;
        case 2:                                             // request Acquisition assistance (Doppler assistance)
            req_adata->acquisitionAssistanceRequested = 1;  // 1
            req_adata->navigationModelRequested = 0;        // 1
            req_adata->referenceTimeRequested = 1;
            req_adata->utcModelRequested = 1;          // 1
            req_adata->ionosphericModelRequested = 1;  // 1
            req_adata->referenceLocationRequested = 1;
            req_adata->almanacRequested = 1;
            req_adata->realTimeIntegrityRequested = 1;  // 1
            break;
        default:
            req_adata->acquisitionAssistanceRequested = 0;  // 1
            req_adata->navigationModelRequested = 0;        // 1
            req_adata->referenceTimeRequested = 1;
            req_adata->utcModelRequested = 1;          // 1
            req_adata->ionosphericModelRequested = 1;  // 1
            req_adata->referenceLocationRequested = 1;
            req_adata->almanacRequested = 1;
            req_adata->realTimeIntegrityRequested = 1;  // 1
            break;
        }

    ulp->message.choice.msSUPLPOSINIT.requestedAssistData = req_adata;

    if (ctx->p.set & PARAM_GSM_CELL_CURRENT)
        {
            ulp->message.choice.msSUPLPOSINIT.locationId.cellInfo.present = CellInfo_PR_gsmCell;
            ulp->message.choice.msSUPLPOSINIT.locationId.cellInfo.choice.gsmCell.refMCC = ctx->p.gsm.mcc;
            ulp->message.choice.msSUPLPOSINIT.locationId.cellInfo.choice.gsmCell.refMNC = ctx->p.gsm.mnc;
            ulp->message.choice.msSUPLPOSINIT.locationId.cellInfo.choice.gsmCell.refLAC = ctx->p.gsm.lac;
            ulp->message.choice.msSUPLPOSINIT.locationId.cellInfo.choice.gsmCell.refCI = ctx->p.gsm.ci;
            ulp->message.choice.msSUPLPOSINIT.locationId.cellInfo.choice.gsmCell.nMR = OPTIONAL_MISSING;
            ulp->message.choice.msSUPLPOSINIT.locationId.cellInfo.choice.gsmCell.tA = OPTIONAL_MISSING;
        }
    else if (ctx->p.set & PARAM_WCDMA_CELL_CURRENT)
        {
            ulp->message.choice.msSUPLPOSINIT.locationId.cellInfo.present = CellInfo_PR_wcdmaCell;
            ulp->message.choice.msSUPLPOSINIT.locationId.cellInfo.choice.wcdmaCell.refMCC = ctx->p.wcdma.mcc;
            ulp->message.choice.msSUPLPOSINIT.locationId.cellInfo.choice.wcdmaCell.refMNC = ctx->p.wcdma.mnc;
            ulp->message.choice.msSUPLPOSINIT.locationId.cellInfo.choice.wcdmaCell.refUC = ctx->p.wcdma.uc;
        }

    if (ctx->p.set & PARAM_GSM_CELL_KNOWN)
        {
            Position_t *pos = calloc(1, sizeof(Position_t));
            struct tm *tm;
            time_t t;

            (void)asn_long2INTEGER(&ulp->message.choice.msSUPLPOSINIT.locationId.status, Status_stale);

            t = time(0);
            tm = gmtime(&t);
            asn_UT2time(&pos->timestamp, tm, 1);
            (void)asn_long2INTEGER(&pos->positionEstimate.latitudeSign, latitudeSign_north);
            pos->positionEstimate.latitude = (1 << 23) / 90.0 * ctx->p.known.lat;
            pos->positionEstimate.longitude = (1 << 24) / 360.0 * ctx->p.known.lon;
            // TODO: set position estimate

            ulp->message.choice.msSUPLPOSINIT.position = pos;
        }
    else
        {
            (void)asn_long2INTEGER(&ulp->message.choice.msSUPLPOSINIT.locationId.status, Status_current);
            ulp->message.choice.msSUPLPOSINIT.position = OPTIONAL_MISSING;
        }

    ulp->message.choice.msSUPLPOSINIT.sUPLPOS = OPTIONAL_MISSING;

    ulp->message.choice.msSUPLPOSINIT.ver = OPTIONAL_MISSING;

    pdu->pdu = ulp;

    err = supl_ulp_encode(pdu);
    if (err < 0)
        {
            asn_DEF_ULP_PDU.free_struct(&asn_DEF_ULP_PDU, ulp, 0);
            return err;
        }

    return 0;
}


static int pdu_make_ulp_rrlp_ack(supl_ctx_t *ctx, supl_ulp_t *pdu, PDU_t *rrlp)
{
    int err;
    PDU_t *rrlp_ack;
    ULP_PDU_t *ulp;
    SetSessionID_t *session_id;
    asn_enc_rval_t ret;
    int pdu_len;
    char buffer[1024];

    rrlp_ack = calloc(1, sizeof(PDU_t));

    /* create RRLP assistanceDataAck */

    rrlp_ack->referenceNumber = rrlp->referenceNumber;
    rrlp_ack->component.present = RRLP_Component_PR_assistanceDataAck;

    ret = uper_encode_to_buffer(&asn_DEF_PDU, rrlp_ack, buffer, sizeof(buffer));
    asn_DEF_ULP_PDU.free_struct(&asn_DEF_PDU, rrlp_ack, 0);
    if (ret.encoded == -1)
        {
            return E_SUPL_ENCODE_RRLP;
        }
    pdu_len = (ret.encoded + 7) >> 3;

    /* embedded it in SUPLPOS */

    ulp = calloc(1, sizeof(ULP_PDU_t));
    session_id = calloc(1, sizeof(SetSessionID_t));

    ulp->length = 0;
    ulp->version.maj = 1;
    ulp->version.min = 0;
    ulp->version.servind = 0;

    session_id->sessionId = 1;
    //  session_id->setId.present = SETId_PR_msisdn;
    // (void)OCTET_STRING_fromBuf(&session_id->setId.choice.msisdn, ctx->p.msisdn, 8);
    session_id->setId.present = SETId_PR_imsi;
    (void)OCTET_STRING_fromBuf(&session_id->setId.choice.imsi, ctx->p.msisdn, 8);

    ulp->sessionID.setSessionID = session_id;
    // ulp->sessionID.slpSessionID = OPTIONAL_MISSING;
    if (ctx->slp_session_id.buf)
        {
            (void)uper_decode_complete(0, &asn_DEF_SlpSessionID, (void **)&ulp->sessionID.slpSessionID, ctx->slp_session_id.buf, ctx->slp_session_id.size);
        }
    else
        {
            ulp->sessionID.slpSessionID = OPTIONAL_MISSING;
        }

    ulp->message.present = UlpMessage_PR_msSUPLPOS;
    ulp->message.choice.msSUPLPOS.posPayLoad.present = PosPayLoad_PR_rrlpPayload;
    (void)OCTET_STRING_fromBuf(&ulp->message.choice.msSUPLPOS.posPayLoad.choice.rrlpPayload, buffer, pdu_len);

    pdu->pdu = ulp;

    err = supl_ulp_encode(pdu);
    if (err < 0)
        {
            supl_ulp_free(pdu);
            return err;
        }

    return 0;
}

/*
**
**
*/

int EXPORT supl_collect_rrlp(supl_assist_t *assist, PDU_t *rrlp, struct timeval *t)
{
    ControlHeader_t *hdr;

    if (rrlp->component.present != RRLP_Component_PR_assistanceData)
        {
            return 0;
        }
    if (!rrlp->component.choice.assistanceData.gps_AssistData)
        {
            return 0;
        }

    hdr = &rrlp->component.choice.assistanceData.gps_AssistData->controlHeader;

    if (hdr->referenceTime)
        {
            assist->set |= SUPL_RRLP_ASSIST_REFTIME;
            assist->time.gps_tow = hdr->referenceTime->gpsTime.gpsTOW23b;
            assist->time.gps_week = hdr->referenceTime->gpsTime.gpsWeek;
            memcpy(&assist->time.stamp, t, sizeof(struct timeval));
        }

    if (hdr->refLocation)
        {
            OCTET_STRING_t *loc;

            loc = &hdr->refLocation->threeDLocation;
            if (loc->size == 14 && loc->buf[0] == 0x90)
                {
                    double lat;
                    double lon;
                    long l;

                    /* from 3GPP TS 23.032 V4.0.0 (2001-04) */

                    l = (loc->buf[1] & 0x7f) << 16 |
                        (loc->buf[2] << 8) |
                        loc->buf[3];
                    if (loc->buf[1] & 0x80)
                        {
                            l *= -1;
                        }
                    lat = 90.0 / (1 << 23) * l;

                    l = (loc->buf[4] << 16) |
                        (loc->buf[5] << 8) |
                        loc->buf[6];
                    lon = 360.0 / (1 << 24) * l;

                    /* max of uncertainty ellipsoid axis */
                    /* uncert in meters = 10 * (1.1 ^ l - 1) */
                    /* l == 96 => 100 km uncertainty => not usable */
                    l = loc->buf[9];
                    if (loc->buf[10] > l)
                        {
                            l = loc->buf[10];
                        }

                    assist->set |= SUPL_RRLP_ASSIST_REFLOC;
                    assist->pos.lat = lat;
                    assist->pos.lon = lon;
                    assist->pos.uncertainty = l;
                }
        }

    if (hdr->acquisAssist)
        {
            int n;

            assist->acq_time = hdr->acquisAssist->timeRelation.gpsTOW;

            for (n = 0; n < hdr->acquisAssist->acquisList.list.count; n++)
                {
                    struct AcquisElement *e = hdr->acquisAssist->acquisList.list.array[n];
                    int i = assist->cnt_acq++;

                    assist->acq[i].prn = e->svid + 1;
                    assist->acq[i].parts = 0;
                    assist->acq[i].doppler0 = e->doppler0;

                    if (e->addionalDoppler)
                        {
                            assist->acq[i].parts |= SUPL_ACQUIS_DOPPLER;
                            assist->acq[i].doppler1 = e->addionalDoppler->doppler1;
                            assist->acq[i].d_win = e->addionalDoppler->dopplerUncertainty;
                        }

                    assist->acq[i].code_ph = e->codePhase;
                    assist->acq[i].code_ph_int = e->intCodePhase;
                    assist->acq[i].bit_num = e->gpsBitNumber;
                    assist->acq[i].code_ph_win = e->codePhaseSearchWindow;

                    if (e->addionalAngle)
                        {
                            assist->acq[i].parts |= SUPL_ACQUIS_ANGLE;
                            assist->acq[i].az = e->addionalAngle->azimuth;
                            assist->acq[i].el = e->addionalAngle->elevation;
                        }
                }
        }

    if (hdr->almanac)
        {
            int n;

            for (n = 0; n < hdr->almanac->almanacList.list.count; n++)
                {
                    struct AlmanacElement *e = hdr->almanac->almanacList.list.array[n];
                    int i = assist->cnt_alm++;

                    assist->alm[i].prn = e->satelliteID + 1;
                    assist->alm[i].e = e->almanacE;
                    assist->alm[i].toa = e->alamanacToa;  // nice touch 3gpp
                    assist->alm[i].Ksii = e->almanacKsii;
                    assist->alm[i].OMEGA_dot = e->almanacOmegaDot;
                    assist->alm[i].A_sqrt = e->almanacAPowerHalf;
                    assist->alm[i].OMEGA_0 = e->almanacOmega0;
                    assist->alm[i].w = e->almanacW;
                    assist->alm[i].M0 = e->almanacM0;
                    assist->alm[i].AF0 = e->almanacAF0;
                    assist->alm[i].AF1 = e->almanacAF1;
                }
        }

    if (hdr->navigationModel)
        {
            UncompressedEphemeris_t *ue;
            int n;

            for (n = 0; n < hdr->navigationModel->navModelList.list.count; n++)
                {
                    struct NavModelElement *e = hdr->navigationModel->navModelList.list.array[n];
                    int i = assist->cnt_eph++;

                    assist->eph[i].prn = e->satelliteID + 1;

                    /* what is the difference between these two */
                    ue = 0;
                    if (e->satStatus.present == SatStatus_PR_newNaviModelUC)
                        {
                            ue = &e->satStatus.choice.newNaviModelUC;
                        }
                    if (e->satStatus.present == SatStatus_PR_newSatelliteAndModelUC)
                        {
                            ue = &e->satStatus.choice.newSatelliteAndModelUC;
                        }

                    if (ue)
                        {
#if 0
    assist->eph_x[i].L2P = ue->ephemL2Pflag;
    assist->eph_x[i].fit = ue->ephemFitFlag;
#endif
                            assist->eph[i].delta_n = ue->ephemDeltaN;
                            assist->eph[i].M0 = ue->ephemM0;
#if 0
    // this is needed for asn1c version 0.9.22
    {
      long v;
      asn_INTEGER2long((INTEGER_t *)&ue->ephemE, &v);
      assist->eph[i].e = v;
      asn_INTEGER2long((INTEGER_t *)&ue->ephemAPowerHalf, &v);
      assist->eph[i].e = v;
    }
#else
                            assist->eph[i].e = ue->ephemE;
                            assist->eph[i].A_sqrt = ue->ephemAPowerHalf;
#endif
                            assist->eph[i].OMEGA_0 = ue->ephemOmegaA0;
                            assist->eph[i].i0 = ue->ephemI0;
                            assist->eph[i].w = ue->ephemW;
                            assist->eph[i].OMEGA_dot = ue->ephemOmegaADot;
                            assist->eph[i].i_dot = ue->ephemIDot;
                            assist->eph[i].Cuc = ue->ephemCuc;
                            assist->eph[i].Cus = ue->ephemCus;
                            assist->eph[i].Crc = ue->ephemCrc;
                            assist->eph[i].Crs = ue->ephemCrs;
                            assist->eph[i].Cic = ue->ephemCic;
                            assist->eph[i].Cis = ue->ephemCis;
                            assist->eph[i].toe = ue->ephemToe;
                            assist->eph[i].IODC = ue->ephemIODC;
                            assist->eph[i].toc = ue->ephemToc;
                            assist->eph[i].AF0 = ue->ephemAF0;
                            assist->eph[i].AF1 = ue->ephemAF1;
                            assist->eph[i].AF2 = ue->ephemAF2;

                            assist->eph[i].nav_model = 1;
                            assist->eph[i].bits = ue->ephemCodeOnL2;
                            assist->eph[i].ura = ue->ephemURA;
                            assist->eph[i].health = ue->ephemSVhealth;
                            assist->eph[i].AODA = ue->ephemAODA;
                            assist->eph[i].tgd = ue->ephemTgd;
                        }
                }
        }

    if (hdr->ionosphericModel)
        {
            assist->set |= SUPL_RRLP_ASSIST_IONO;
            assist->iono.a0 = hdr->ionosphericModel->alfa0;
            assist->iono.a1 = hdr->ionosphericModel->alfa1;
            assist->iono.a2 = hdr->ionosphericModel->alfa2;
            assist->iono.a3 = hdr->ionosphericModel->alfa3;  // missed in original supl client
            assist->iono.b0 = hdr->ionosphericModel->beta0;
            assist->iono.b1 = hdr->ionosphericModel->beta1;
            assist->iono.b2 = hdr->ionosphericModel->beta2;
            assist->iono.b3 = hdr->ionosphericModel->beta3;
        }

    if (hdr->utcModel)
        {
            assist->set |= SUPL_RRLP_ASSIST_UTC;
            assist->utc.a0 = hdr->utcModel->utcA0;
            assist->utc.a1 = hdr->utcModel->utcA1;
            assist->utc.tot = hdr->utcModel->utcTot;
            assist->utc.wnt = hdr->utcModel->utcWNt;
            assist->utc.delta_tls = hdr->utcModel->utcDeltaTls;
            assist->utc.wnlsf = hdr->utcModel->utcWNlsf;
            assist->utc.dn = hdr->utcModel->utcDN;
            assist->utc.delta_tlsf = hdr->utcModel->utcDeltaTlsf;
        }

    return 1;
}


int EXPORT supl_ctx_new(supl_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(supl_ctx_t));
#ifdef SUPL_DEBUG
    memset(&debug, 0, sizeof(struct supl_debug_s));
#endif

    return 0;
}


int EXPORT supl_ctx_free(supl_ctx_t *ctx)
{
    if (ctx->slp_session_id.buf)
        {
            free(ctx->slp_session_id.buf);
            ctx->slp_session_id.buf = 0;
        }

    return 0;
}


static int supl_more_rrlp(PDU_t *rrlp)
{
    int64_t value;

    return (rrlp->component.present == RRLP_Component_PR_assistanceData &&
            rrlp->component.choice.assistanceData.moreAssDataToBeSent &&
            asn_INTEGER2long((INTEGER_t *)rrlp->component.choice.assistanceData.moreAssDataToBeSent, &value) == 0 &&
            value == MoreAssDataToBeSent_moreMessagesOnTheWay);
}


int EXPORT supl_get_assist(supl_ctx_t *ctx, char *server, supl_assist_t *assist)
{
    supl_ulp_t ulp;

    //  memcpy(ctx->p.msisdn, "\xde\xad\xbe\xef\xf0\x0b\xaa\x42", 8);
    memcpy(ctx->p.msisdn, "\xFF\xFF\x91\x94\x48\x45\x83\x98", 8);

    /*
     ** connect to server
     */

    if (supl_server_connect(ctx, server) < 0)
        {
            return E_SUPL_CONNECT;
        }

    /*
     ** send SUPL_START
     */

    if (pdu_make_ulp_start(ctx, &ulp) < 0)
        {
            return E_SUPL_ENCODE_START;
        }

    (void)supl_ulp_send(ctx, &ulp);
    supl_ulp_free(&ulp);

    /*
     ** should receive SUPL_RESPONSE back
     */

    if (supl_ulp_recv(ctx, &ulp) < 0)
        {
            return E_SUPL_RECV_RESPONSE;
        }

    if (ulp.pdu->message.present != UlpMessage_PR_msSUPLRESPONSE)
        {
            supl_ulp_free(&ulp);
            return E_SUPL_SUPLRESPONSE;
        }

    // get and copy slpSessionID if present
    supl_response_harvest(ctx, &ulp);

    supl_ulp_free(&ulp);

    /*
     ** send SUPL_POS_INIT
     */

    if (pdu_make_ulp_pos_init(ctx, &ulp) < 0)
        {
            return E_SUPL_ENCODE_POSINIT;
        }

    (void)supl_ulp_send(ctx, &ulp);

    /*
     ** should get SUPLPOS back - bounce back and forth until all messages have arrived
     */

    memset(assist, 0, sizeof(supl_assist_t));

    while (1)
        {
            struct timeval t;
            PDU_t *rrlp;

            supl_ulp_free(&ulp);

            /* record packet recv time */
            gettimeofday(&t, 0);

            if (supl_ulp_recv(ctx, &ulp) < 0)
                {
                    return E_SUPL_RECV_SUPLPOS;
                }

            if (ulp.pdu->message.present == UlpMessage_PR_msSUPLEND)
                {
                    break;
                }

            if (ulp.pdu->message.present != UlpMessage_PR_msSUPLPOS)
                {
                    supl_ulp_free(&ulp);
                    return E_SUPL_SUPLPOS;
                }

            /* get the beef, the RRLP payload */

            if (supl_decode_rrlp(&ulp, &rrlp) < 0)
                {
                    supl_ulp_free(&ulp);
                    return E_SUPL_DECODE_RRLP;
                }

#ifdef SUPL_DEBUG
            if (debug.verbose_rrlp)
                {
                    fprintf(debug.log, "Embedded RRLP message\n");
                    xer_fprint(debug.log, &asn_DEF_PDU, rrlp);
                }
#endif

            /* remember important stuff from it */

            supl_collect_rrlp(assist, rrlp, &t);

            if (!supl_more_rrlp(rrlp))
                {
                    asn_DEF_ULP_PDU.free_struct(&asn_DEF_PDU, rrlp, 0);
                    break;
                }

            /* More data coming in, send SUPLPOS + RRLP ACK */

            if (pdu_make_ulp_rrlp_ack(ctx, &ulp, rrlp) < 0)
                {
                    return E_SUPL_RRLP_ACK;
                }

            supl_ulp_send(ctx, &ulp);
            asn_DEF_ULP_PDU.free_struct(&asn_DEF_PDU, rrlp, 0);
        }

    supl_ulp_free(&ulp);

    /*
     ** send SUPL_END (but who really cares)
     */

    supl_close(ctx);

    return 0;
}


void EXPORT supl_set_gsm_cell(supl_ctx_t *ctx, int mcc, int mns, int lac, int ci)
{
    ctx->p.set |= PARAM_GSM_CELL_CURRENT;

    ctx->p.gsm.mcc = mcc;
    ctx->p.gsm.mnc = mns;
    ctx->p.gsm.lac = lac;
    ctx->p.gsm.ci = ci;
}


void EXPORT supl_set_gsm_cell_known(supl_ctx_t *ctx, int mcc, int mns, int lac, int ci, double lat, double lon, int uncert)
{
    ctx->p.set |= PARAM_GSM_CELL_KNOWN;

    ctx->p.known.mcc = mcc;
    ctx->p.known.mnc = mns;
    ctx->p.known.lac = lac;
    ctx->p.known.ci = ci;
    ctx->p.known.lat = lat;
    ctx->p.known.lon = lon;
    ctx->p.known.uncert = uncert;
}


void EXPORT supl_set_wcdma_cell(supl_ctx_t *ctx, int mcc, int mns, int uc)
{
    ctx->p.set |= PARAM_WCDMA_CELL_CURRENT;

    ctx->p.wcdma.mcc = mcc;
    ctx->p.wcdma.mnc = mns;
    ctx->p.wcdma.uc = uc;
}


void EXPORT supl_request(supl_ctx_t *ctx, int request)
{
    ctx->p.request = request;
}


void EXPORT supl_set_debug(FILE *log, int flags)
{
#ifdef SUPL_DEBUG
    debug.log = log;
    if (flags & SUPL_DEBUG_RRLP) debug.verbose_rrlp = 1;
    if (flags & SUPL_DEBUG_SUPL) debug.verbose_supl = 1;
    if (flags & SUPL_DEBUG_DEBUG) debug.debug = 1;
#endif
}
