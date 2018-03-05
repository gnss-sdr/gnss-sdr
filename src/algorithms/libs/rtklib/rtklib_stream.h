/*!
 * \file rtklib_sbas.h
 * \brief sbas functions
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
 *-----------------------------------------------------------------------------*/

#ifndef GNSS_SDR_RTKLIB_STREAM_H_
#define GNSS_SDR_RTKLIB_STREAM_H_

#include "rtklib.h"

/* constants -----------------------------------------------------------------*/

#define TINTACT 200       /* period for stream active (ms) */
#define SERIBUFFSIZE 4096 /* serial buffer size (bytes) */
#define TIMETAGH_LEN 64   /* time tag file header length */
#define MAXCLI 32         /* max client connection for tcp svr */
#define MAXSTATMSG 32     /* max length of status message */

#define VER_RTKLIB "2.4.2"
#define NTRIP_AGENT "RTKLIB/" VER_RTKLIB
#define NTRIP_CLI_PORT 2101                       /* default ntrip-client connection port */
#define NTRIP_SVR_PORT 80                         /* default ntrip-server connection port */
#define NTRIP_MAXRSP 32768                        /* max size of ntrip response */
#define NTRIP_MAXSTR 256                          /* max length of mountpoint string */
#define NTRIP_RSP_OK_CLI "ICY 200 OK\r\n"         /* ntrip response: client */
#define NTRIP_RSP_OK_SVR "OK\r\n"                 /* ntrip response: server */
#define NTRIP_RSP_SRCTBL "SOURCETABLE 200 OK\r\n" /* ntrip response: source table */
#define NTRIP_RSP_TBLEND "ENDSOURCETABLE"
#define NTRIP_RSP_HTTP "HTTP/"  /* ntrip response: http */
#define NTRIP_RSP_ERROR "ERROR" /* ntrip response: error */

#define FTP_CMD "wget" /* ftp/http command */
#define FTP_TIMEOUT 30 /* ftp/http timeout (s) */


serial_t *openserial(const char *path, int mode, char *msg);

void closeserial(serial_t *serial);

int readserial(serial_t *serial, unsigned char *buff, int n, char *msg);

int writeserial(serial_t *serial, unsigned char *buff, int n, char *msg);

int stateserial(serial_t *serial);

int openfile_(file_t *file, gtime_t time, char *msg);

void closefile_(file_t *file);

file_t *openfile(const char *path, int mode, char *msg);

void closefile(file_t *file);

void swapfile(file_t *file, gtime_t time, char *msg);

void swapclose(file_t *file);

int statefile(file_t *file);

int readfile(file_t *file, unsigned char *buff, int nmax, char *msg);

int writefile(file_t *file, unsigned char *buff, int n, char *msg);

void syncfile(file_t *file1, file_t *file2);

void decodetcppath(const char *path, char *addr, char *port, char *user,
    char *passwd, char *mntpnt, char *str);

int errsock(void);

int setsock(socket_t sock, char *msg);

socket_t accept_nb(socket_t sock, struct sockaddr *addr, socklen_t *len);

int connect_nb(socket_t sock, struct sockaddr *addr, socklen_t len);

int recv_nb(socket_t sock, unsigned char *buff, int n);

int send_nb(socket_t sock, unsigned char *buff, int n);

int gentcp(tcp_t *tcp, int type, char *msg);

void discontcp(tcp_t *tcp, int tcon);

tcpsvr_t *opentcpsvr(const char *path, char *msg);

void closetcpsvr(tcpsvr_t *tcpsvr);

void updatetcpsvr(tcpsvr_t *tcpsvr, char *msg);

int accsock(tcpsvr_t *tcpsvr, char *msg);

int waittcpsvr(tcpsvr_t *tcpsvr, char *msg);

int readtcpsvr(tcpsvr_t *tcpsvr, unsigned char *buff, int n, char *msg);

int writetcpsvr(tcpsvr_t *tcpsvr, unsigned char *buff, int n, char *msg);

int statetcpsvr(tcpsvr_t *tcpsvr);

int consock(tcpcli_t *tcpcli, char *msg);

tcpcli_t *opentcpcli(const char *path, char *msg);

void closetcpcli(tcpcli_t *tcpcli);

int waittcpcli(tcpcli_t *tcpcli, char *msg);

int readtcpcli(tcpcli_t *tcpcli, unsigned char *buff, int n, char *msg);

int writetcpcli(tcpcli_t *tcpcli, unsigned char *buff, int n, char *msg);

int statetcpcli(tcpcli_t *tcpcli);

int encbase64(char *str, const unsigned char *byte, int n);

int reqntrip_s(ntrip_t *ntrip, char *msg);

int reqntrip_c(ntrip_t *ntrip, char *msg);

int rspntrip_s(ntrip_t *ntrip, char *msg);

int rspntrip_c(ntrip_t *ntrip, char *msg);

int waitntrip(ntrip_t *ntrip, char *msg);

ntrip_t *openntrip(const char *path, int type, char *msg);

void closentrip(ntrip_t *ntrip);

int readntrip(ntrip_t *ntrip, unsigned char *buff, int n, char *msg);

int writentrip(ntrip_t *ntrip, unsigned char *buff, int n, char *msg);

int statentrip(ntrip_t *ntrip);

void decodeftppath(const char *path, char *addr, char *file, char *user,
    char *passwd, int *topts);

gtime_t nextdltime(const int *topts, int stat);

void *ftpthread(void *arg);

ftp_t *openftp(const char *path, int type, char *msg);

void closeftp(ftp_t *ftp);

int readftp(ftp_t *ftp, unsigned char *buff, int n, char *msg);

int stateftp(ftp_t *ftp);

void strinitcom(void);

void strinit(stream_t *stream);

int stropen(stream_t *stream, int type, int mode, const char *path);

void strclose(stream_t *stream);

void strsync(stream_t *stream1, stream_t *stream2);

void strlock(stream_t *stream);

void strunlock(stream_t *stream);

int strread(stream_t *stream, unsigned char *buff, int n);

int strwrite(stream_t *stream, unsigned char *buff, int n);

int strstat(stream_t *stream, char *msg);

void strsum(stream_t *stream, int *inb, int *inr, int *outb, int *outr);

void strsetopt(const int *opt);

void strsettimeout(stream_t *stream, int toinact, int tirecon);

void strsetdir(const char *dir);

void strsetproxy(const char *addr);

gtime_t strgettime(stream_t *stream);

void strsendnmea(stream_t *stream, const double *pos);

int gen_hex(const char *msg, unsigned char *buff);

void strsendcmd(stream_t *str, const char *cmd);


#endif
