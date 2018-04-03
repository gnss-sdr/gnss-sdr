/*!
 * \file rtklib_rtcm.cc
 * \brief RTCM functions
 * \authors <ul>
 *          <li> 2007-2013, T. Takasu
 *          <li> 2017, Javier Arribas
 *          <li> 2017, Carles Fernandez
 *          </ul>
 *
 * This is a derived work from RTKLIB http://www.rtklib.com/
 * The original source code at https://github.com/tomojitakasu/RTKLIB is
 * released under the BSD 2-clause license with an additional exclusive clause
 * that does not apply here. This additional clause is reproduced below:
 *
 * " The software package includes some companion executive binaries or shared
 * libraries necessary to execute APs on Windows. These licenses succeed to the
 * original ones of these software. "
 *
 * Neither the executive binaries nor the shared libraries are required by, used
 * or included in GNSS-SDR.
 *
 * -------------------------------------------------------------------------
 * Copyright (C) 2007-2013, T. Takasu
 * Copyright (C) 2017, Javier Arribas
 * Copyright (C) 2017, Carles Fernandez
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *----------------------------------------------------------------------------*/

#include "rtklib_rtcm.h"
#include "rtklib_rtkcmn.h"

//extern int encode_rtcm3(rtcm_t *rtcm, int type, int sync);


/* initialize rtcm control -----------------------------------------------------
 * initialize rtcm control struct and reallocate memory for observation and
 * ephemeris buffer in rtcm control struct
 * args   : rtcm_t *raw   IO     rtcm control struct
 * return : status (1:ok,0:memory allocation error)
 *-----------------------------------------------------------------------------*/
int init_rtcm(rtcm_t *rtcm)
{
    gtime_t time0 = {0, 0.0};
    obsd_t data0 = {{0, 0.0}, 0, 0, {0}, {0}, {0}, {0.0}, {0.0}, {0.0}};
    eph_t eph0 = {0, -1, -1, 0, 0, 0, 0, 0, {0, 0.0}, {0, 0.0}, {0, 0.0},
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {0.0}, 0.0, 0.0};
    geph_t geph0 = {0, -1, 0, 0, 0, 0, {0, 0.0}, {0, 0.0}, {0.0}, {0.0}, {0.0},
        0.0, 0.0, 0.0};
    ssr_t ssr0 = {{{0, 0.0}}, {0.0}, {0}, 0, 0, 0, 0, {0.0}, {0.0}, {0.0}, 0.0, {0.0}, {0.0}, {0.0}, 0.0, 0.0, '0'};
    int i, j;

    trace(3, "init_rtcm:\n");

    rtcm->staid = rtcm->stah = rtcm->seqno = rtcm->outtype = 0;
    rtcm->time = rtcm->time_s = time0;
    rtcm->sta.name[0] = rtcm->sta.marker[0] = '\0';
    rtcm->sta.antdes[0] = rtcm->sta.antsno[0] = '\0';
    rtcm->sta.rectype[0] = rtcm->sta.recver[0] = rtcm->sta.recsno[0] = '\0';
    rtcm->sta.antsetup = rtcm->sta.itrf = rtcm->sta.deltype = 0;
    for (i = 0; i < 3; i++)
        {
            rtcm->sta.pos[i] = rtcm->sta.del[i] = 0.0;
        }
    rtcm->sta.hgt = 0.0;
    rtcm->dgps = NULL;
    for (i = 0; i < MAXSAT; i++)
        {
            rtcm->ssr[i] = ssr0;
        }
    rtcm->msg[0] = rtcm->msgtype[0] = rtcm->opt[0] = '\0';
    for (i = 0; i < 6; i++) rtcm->msmtype[i][0] = '\0';
    rtcm->obsflag = rtcm->ephsat = 0;
    for (i = 0; i < MAXSAT; i++)
        for (j = 0; j < NFREQ + NEXOBS; j++)
            {
                rtcm->cp[i][j] = 0.0;
                rtcm->lock[i][j] = rtcm->loss[i][j] = 0;
                rtcm->lltime[i][j] = time0;
            }
    rtcm->nbyte = rtcm->nbit = rtcm->len = 0;
    rtcm->word = 0;
    for (i = 0; i < 100; i++) rtcm->nmsg2[i] = 0;
    for (i = 0; i < 300; i++) rtcm->nmsg3[i] = 0;

    rtcm->obs.data = NULL;
    rtcm->nav.eph = NULL;
    rtcm->nav.geph = NULL;

    /* reallocate memory for observation and ephemris buffer */
    if (!(rtcm->obs.data = (obsd_t *)malloc(sizeof(obsd_t) * MAXOBS)) ||
        !(rtcm->nav.eph = (eph_t *)malloc(sizeof(eph_t) * MAXSAT)) ||
        !(rtcm->nav.geph = (geph_t *)malloc(sizeof(geph_t) * MAXPRNGLO)))
        {
            free_rtcm(rtcm);
            return 0;
        }
    rtcm->obs.n = 0;
    rtcm->nav.n = MAXSAT;
    rtcm->nav.ng = MAXPRNGLO;
    for (i = 0; i < MAXOBS; i++) rtcm->obs.data[i] = data0;
    for (i = 0; i < MAXSAT; i++) rtcm->nav.eph[i] = eph0;
    for (i = 0; i < MAXPRNGLO; i++) rtcm->nav.geph[i] = geph0;
    return 1;
}


/* free rtcm control ----------------------------------------------------------
 * free observation and ephemris buffer in rtcm control struct
 * args   : rtcm_t *raw   IO     rtcm control struct
 * return : none
 *-----------------------------------------------------------------------------*/
void free_rtcm(rtcm_t *rtcm)
{
    trace(3, "free_rtcm:\n");

    /* free memory for observation and ephemeris buffer */
    free(rtcm->obs.data);
    rtcm->obs.data = NULL;
    rtcm->obs.n = 0;
    free(rtcm->nav.eph);
    rtcm->nav.eph = NULL;
    rtcm->nav.n = 0;
    free(rtcm->nav.geph);
    rtcm->nav.geph = NULL;
    rtcm->nav.ng = 0;
}


/* input rtcm 2 message from stream --------------------------------------------
 * fetch next rtcm 2 message and input a message from byte stream
 * args   : rtcm_t *rtcm IO   rtcm control struct
 *          unsigned char data I stream data (1 byte)
 * return : status (-1: error message, 0: no message, 1: input observation data,
 *                  2: input ephemeris, 5: input station pos/ant parameters,
 *                  6: input time parameter, 7: input dgps corrections,
 *                  9: input special message)
 * notes  : before firstly calling the function, time in rtcm control struct has
 *          to be set to the approximate time within 1/2 hour in order to resolve
 *          ambiguity of time in rtcm messages.
 *          supported msgs RTCM ver.2: 1,3,9,14,16,17,18,19,22
 *          refer [1] for RTCM ver.2
 *-----------------------------------------------------------------------------*/
int input_rtcm2(rtcm_t *rtcm, unsigned char data)
{
    unsigned char preamb;
    int i;

    trace(5, "input_rtcm2: data=%02x\n", data);

    if ((data & 0xC0) != 0x40) return 0; /* ignore if upper 2bit != 01 */

    for (i = 0; i < 6; i++, data >>= 1)
        { /* decode 6-of-8 form */
            rtcm->word = (rtcm->word << 1) + (data & 1);

            /* synchronize frame */
            if (rtcm->nbyte == 0)
                {
                    preamb = (unsigned char)(rtcm->word >> 22);
                    if (rtcm->word & 0x40000000) preamb ^= 0xFF; /* decode preamble */
                    if (preamb != RTCM2PREAMB) continue;

                    /* check parity */
                    if (!decode_word(rtcm->word, rtcm->buff)) continue;
                    rtcm->nbyte = 3;
                    rtcm->nbit = 0;
                    continue;
                }
            if (++rtcm->nbit < 30)
                continue;
            else
                rtcm->nbit = 0;

            /* check parity */
            if (!decode_word(rtcm->word, rtcm->buff + rtcm->nbyte))
                {
                    trace(2, "rtcm2 partity error: i=%d word=%08x\n", i, rtcm->word);
                    rtcm->nbyte = 0;
                    rtcm->word &= 0x3;
                    continue;
                }
            rtcm->nbyte += 3;
            if (rtcm->nbyte == 6) rtcm->len = (rtcm->buff[5] >> 3) * 3 + 6;
            if (rtcm->nbyte < rtcm->len) continue;
            rtcm->nbyte = 0;
            rtcm->word &= 0x3;

            /* decode rtcm2 message */
            return decode_rtcm2(rtcm);
        }
    return 0;
}


/* input rtcm 3 message from stream --------------------------------------------
 * fetch next rtcm 3 message and input a message from byte stream
 * args   : rtcm_t *rtcm IO   rtcm control struct
 *          unsigned char data I stream data (1 byte)
 * return : status (-1: error message, 0: no message, 1: input observation data,
 *                  2: input ephemeris, 5: input station pos/ant parameters,
 *                  10: input ssr messages)
 * notes  : before firstly calling the function, time in rtcm control struct has
 *          to be set to the approximate time within 1/2 week in order to resolve
 *          ambiguity of time in rtcm messages.
 *
 *          to specify input options, set rtcm->opt to the following option
 *          strings separated by spaces.
 *
 *          -EPHALL  : input all ephemerides
 *          -STA=nnn : input only message with STAID=nnn
 *          -GLss    : select signal ss for GPS MSM (ss=1C,1P,...)
 *          -RLss    : select signal ss for GLO MSM (ss=1C,1P,...)
 *          -ELss    : select signal ss for GAL MSM (ss=1C,1B,...)
 *          -JLss    : select signal ss for QZS MSM (ss=1C,2C,...)
 *          -CLss    : select signal ss for BDS MSM (ss=2I,7I,...)
 *
 *          supported RTCM 3 messages
 *                  (ref [2][3][4][5][6][7][8][9][10][11][12][13][14][15])
 *
 *            TYPE       GPS     GLOASS    GALILEO    QZSS     BEIDOU     SBAS
 *         ----------------------------------------------------------------------
 *          OBS C-L1  : 1001~     1009~       -         -         -         -
 *              F-L1  : 1002      1010        -         -         -         -
 *              C-L12 : 1003~     1011~       -         -         -         -
 *              F-L12 : 1004      1012        -         -         -         -
 *
 *          NAV       : 1019      1020      1045*     1044*     1047*       -
 *                        -         -       1046*       -         -         -
 *
 *          MSM 1     : 1071~     1081~     1091~     1111*~    1121*~    1101*~
 *              2     : 1072~     1082~     1092~     1112*~    1122*~    1102*~
 *              3     : 1073~     1083~     1093~     1113*~    1123*~    1103*~
 *              4     : 1074      1084      1094      1114*     1124*     1104*
 *              5     : 1075      1085      1095      1115*     1125*     1105*
 *              6     : 1076      1086      1096      1116*     1126*     1106*
 *              7     : 1077      1087      1097      1117*     1127*     1107*
 *
 *          SSR OBT   : 1057      1063      1240*     1246*     1258*       -
 *              CLK   : 1058      1064      1241*     1247*     1259*       -
 *              BIAS  : 1059      1065      1242*     1248*     1260*       -
 *              OBTCLK: 1060      1066      1243*     1249*     1261*       -
 *              URA   : 1061      1067      1244*     1250*     1262*       -
 *              HRCLK : 1062      1068      1245*     1251*     1263*       -
 *
 *          ANT INFO  : 1005 1006 1007 1008 1033
 *         ----------------------------------------------------------------------
 *                                                    (* draft, ~ only encode)
 *
 *          for MSM observation data with multiple signals for a frequency,
 *          a signal is selected according to internal priority. to select
 *          a specified signal, use the input options.
 *
 *          rtcm3 message format:
 *            +----------+--------+-----------+--------------------+----------+
 *            | preamble | 000000 |  length   |    data message    |  parity  |
 *            +----------+--------+-----------+--------------------+----------+
 *            |<-- 8 --->|<- 6 -->|<-- 10 --->|<--- length x 8 --->|<-- 24 -->|
 *
 *-----------------------------------------------------------------------------*/
int input_rtcm3(rtcm_t *rtcm, unsigned char data)
{
    trace(5, "input_rtcm3: data=%02x\n", data);

    /* synchronize frame */
    if (rtcm->nbyte == 0)
        {
            if (data != RTCM3PREAMB) return 0;
            rtcm->buff[rtcm->nbyte++] = data;
            return 0;
        }
    rtcm->buff[rtcm->nbyte++] = data;

    if (rtcm->nbyte == 3)
        {
            rtcm->len = getbitu(rtcm->buff, 14, 10) + 3; /* length without parity */
        }
    if (rtcm->nbyte < 3 || rtcm->nbyte < rtcm->len + 3) return 0;
    rtcm->nbyte = 0;

    /* check parity */
    if (rtk_crc24q(rtcm->buff, rtcm->len) != getbitu(rtcm->buff, rtcm->len * 8, 24))
        {
            trace(2, "rtcm3 parity error: len=%d\n", rtcm->len);
            return 0;
        }
    /* decode rtcm3 message */
    return decode_rtcm3(rtcm);
}


/* input rtcm 2 message from file ----------------------------------------------
 * fetch next rtcm 2 message and input a message from file
 * args   : rtcm_t *rtcm IO   rtcm control struct
 *          FILE  *fp    I    file pointer
 * return : status (-2: end of file, -1...10: same as above)
 * notes  : same as above
 *-----------------------------------------------------------------------------*/
int input_rtcm2f(rtcm_t *rtcm, FILE *fp)
{
    int i, data = 0, ret;

    trace(4, "input_rtcm2f: data=%02x\n", data);

    for (i = 0; i < 4096; i++)
        {
            if ((data = fgetc(fp)) == EOF) return -2;
            if ((ret = input_rtcm2(rtcm, (unsigned char)data))) return ret;
        }
    return 0; /* return at every 4k bytes */
}


/* input rtcm 3 message from file ----------------------------------------------
 * fetch next rtcm 3 message and input a message from file
 * args   : rtcm_t *rtcm IO   rtcm control struct
 *          FILE  *fp    I    file pointer
 * return : status (-2: end of file, -1...10: same as above)
 * notes  : same as above
 *-----------------------------------------------------------------------------*/
int input_rtcm3f(rtcm_t *rtcm, FILE *fp)
{
    int i, data = 0, ret;

    trace(4, "input_rtcm3f: data=%02x\n", data);

    for (i = 0; i < 4096; i++)
        {
            if ((data = fgetc(fp)) == EOF) return -2;
            if ((ret = input_rtcm3(rtcm, (unsigned char)data))) return ret;
        }
    return 0; /* return at every 4k bytes */
}


/* generate rtcm 2 message -----------------------------------------------------
 * generate rtcm 2 message
 * args   : rtcm_t *rtcm   IO rtcm control struct
 *          int    type    I  message type
 *          int    sync    I  sync flag (1:another message follows)
 * return : status (1:ok,0:error)
 *-----------------------------------------------------------------------------*/
int gen_rtcm2(rtcm_t *rtcm, int type, int sync)
{
    trace(4, "gen_rtcm2: type=%d sync=%d\n", type, sync);

    rtcm->nbit = rtcm->len = rtcm->nbyte = 0;

    /* not yet implemented */

    return 0;
}


///* generate rtcm 3 message -----------------------------------------------------
// * generate rtcm 3 message
// * args   : rtcm_t *rtcm   IO rtcm control struct
// *          int    type    I  message type
// *          int    sync    I  sync flag (1:another message follows)
// * return : status (1:ok,0:error)
// *-----------------------------------------------------------------------------*/
//int gen_rtcm3(rtcm_t *rtcm, int type, int sync)
//{
//    unsigned int crc;
//    int i = 0;
//
//    trace(4, "gen_rtcm3: type=%d sync=%d\n", type, sync);
//
//    rtcm->nbit = rtcm->len = rtcm->nbyte = 0;
//
//    /* set preamble and reserved */
//    setbitu(rtcm->buff, i, 8, RTCM3PREAMB); i +=  8;
//    setbitu(rtcm->buff, i, 6, 0          ); i +=  6;
//    setbitu(rtcm->buff, i, 10, 0          ); i += 10;
//
//    /* encode rtcm 3 message body */
//    if (!encode_rtcm3(rtcm, type, sync)) return 0;
//
//    /* padding to align 8 bit boundary */
//    for (i = rtcm->nbit;i%8;i++)
//        {
//            setbitu(rtcm->buff, i, 1, 0);
//        }
//    /* message length (header+data) (bytes) */
//    if ((rtcm->len = i/8) >= 3+1024)
//        {
//            trace(2, "generate rtcm 3 message length error len=%d\n", rtcm->len-3);
//            rtcm->nbit = rtcm->len = 0;
//            return 0;
//        }
//    /* message length without header and parity */
//    setbitu(rtcm->buff, 14, 10, rtcm->len-3);
//
//    /* crc-24q */
//    crc = rtk_crc24q(rtcm->buff, rtcm->len);
//    setbitu(rtcm->buff, i, 24, crc);
//
//    /* length total (bytes) */
//    rtcm->nbyte = rtcm->len+3;
//
//    return 1;
//}
