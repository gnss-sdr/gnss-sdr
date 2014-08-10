/*----------------------------------------------------------------------------------------------*/
/*! \file gn3s.cpp
//
// FILENAME: gn3s.cpp
//
// DESCRIPTION: Impelements the GN3S class.
//
// DEVELOPERS: Gregory W. Heckler (2003-2009)
//
// LICENSE TERMS: Copyright (c) Gregory W. Heckler 2009
//
// This file is part of the GPS Software Defined Radio (GPS-SDR)
//
// The GPS-SDR is free software; you can redistribute it and/or modify it under the terms of the
// GNU General Public License as published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version. The GPS-SDR is distributed in the hope that
// it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// Note:  Comments within this file follow a syntax that is compatible with
//        DOXYGEN and are utilized for automated document extraction
//
// Reference:
*/
/*----------------------------------------------------------------------------------------------*/

#include "sdr.h"
#include "fx2.h"

/* global variables ----------------------------------------------------------*/
using namespace std;
Fx2_dev fx2_d;

/* GN3S initialization ---------------------------------------------------------
* search front end and initialization
* args   : none
* return : int                  status 0:okay -1:failure
*-----------------------------------------------------------------------------*/
extern int gn3s_init(void)
{
    char version; /* version of modules */
    unsigned char uc_flags[5];

    /* Look for a device */
    fx2_d = Fx2_dev(1); /* PID 0x0b39 */
    if (fx2_d.usb_fx2_find() != 0) {
        version = 2;
        if (sdrini.fend!=FEND_GN3SV2) {
            SDRPRINTF("error: wrong frontend type, GN3SV2 is found\n");
            return -1;
        }
    } else {
        fx2_d = Fx2_dev(2); /* PID 0x0b3a */
        if (fx2_d.usb_fx2_find() != 0) {
            version = 3;
            if (sdrini.fend!=FEND_GN3SV3) {
                SDRPRINTF("error: wrong frontend type, GN3SV3 is found\n");
                return -1;
            }
        } else {
            fx2_d = Fx2_dev(3); /* PID 0x0b3f */
            if(fx2_d.usb_fx2_find()!=0) {
                version = 3;
                if (sdrini.fend!=FEND_GN3SV3) {
                    SDRPRINTF("error: wrong frontend type, GN3SV3 is found\n");
                    return -1;
                }
            } else {
                SDRPRINTF("error: no GN3S frontend found\n");
                return -1;
            }
        }
    }
    fx2_d.usb_fx2_init();
    /* version 2 */
    if (version==2) {
        fx2_d.usrp_xfer(VRQ_XFER, 1);
    }
    /* version 3 */
    if (version==3) {
        fx2_d.usrp_xfer (VRQ_AGC,0); /* AGC OFF */
        fx2_d.usrp_xfer (VRQ_CMODE,132 );
        fx2_d.usrp_xfer (VRQ_XFER, 0 );
        fx2_d.usrp_xfer (VRQ_XFER, 1);
        fx2_d.usrp_xfer2(VRQ_FLAGS, 0, uc_flags, 5);
        fx2_d.usrp_xfer (VRQ_XFER, 0 );
        fx2_d.usrp_xfer(VRQ_CMODE,GN3S_MODE);
        fx2_d.usrp_xfer(VRQ_XFER, 1);
    }
    return 0;
}
/* stop front-end --------------------------------------------------------------
* stop grabber of front end
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
extern void gn3s_quit(void)
{
    fx2_d.close();
    fx2_d.~Fx2_dev();
}
/* data expansion to binary (GN3S v2) ------------------------------------------
* get current data buffer from memory buffer
* args   : char   *buf      I   memory buffer
*          int    n         I   number of grab data
*          char   *expbuff  O   extracted data buffer
* return : none
*-----------------------------------------------------------------------------*/
extern void gn3s_exp_v2(unsigned char *buf, int n, char *expbuf)
{
    int i;
    char LUT_1bit[2]={1,-1};
    char shift=0, endshift=0;

    /* data shift (refer gn3s-full.cc) */
    /* http://ccar.colorado.edu/gnss/files/GN3Sv2.7z */
    if ((buf[  0]&0x02)!=2) shift=1;
    if ((buf[n-1]&0x02)!=0) endshift=1;

    for (i=0;i<n;i++) {
        if (shift) {
            if ((i==(n-1))&&!endshift)     expbuf[i]=0;
            else if ((i==(n-1))&&endshift) expbuf[i-1]=0;
            else expbuf[i]=LUT_1bit[buf[i+1]&0x01];
        } else {
            if ((i==(n-1))&&endshift) expbuf[i]=0;
            else expbuf[i]=LUT_1bit[buf[i]&0x01];
        }
    }
}
/* get current data buffer (GN3S v2) -------------------------------------------
* get current data buffer from memory buffer
* args   : uint64_t buffloc I   buffer location
*          int    n         I   number of grab data
*          int    dtype     I   data type (DTYPEI or DTYPEIQ)
*          char   *expbuff  O   extracted data buffer
* return : none
*-----------------------------------------------------------------------------*/
extern void gn3s_getbuff_v2(uint64_t buffloc, int n, int dtype, char *expbuf)
{
    uint64_t membuffloc=2*buffloc%(MEMBUFFLEN*GN3S_BUFFSIZE);
    int nout;
    n=2*n;
    nout=(int)((membuffloc+n)-(MEMBUFFLEN*GN3S_BUFFSIZE));

    mlock(hbuffmtx);
    if (nout>0) {
        gn3s_exp_v2(&sdrstat.buff[membuffloc],n-nout,expbuf);
        gn3s_exp_v2(&sdrstat.buff[0],nout,&expbuf[n-nout]);
    } else {
        gn3s_exp_v2(&sdrstat.buff[membuffloc],n,expbuf);
    }
    unmlock(hbuffmtx);
}
/* data expansion to binary (GN3S v3) ------------------------------------------
* get current data buffer from memory buffer
* args   : char   *buf      I   memory buffer
*          int    n         I   number of grab data
*          int    i_mode    I   GN3S data grabber mode
*          char   *expbuff  O   extracted data buffer
* return : none
*-----------------------------------------------------------------------------*/
extern void gn3s_exp_v3(unsigned char *buf, int n, int i_mode, char *expbuf)
{
    int i=0;
    int i_idx=0;
    int i_dtype=0;
    char LUT_2bit[4]={1,-1,3,-3};
    char LUT_I_4bit[16]={1,-1,0,0,3,-3,0,0,0,0,0,0,0,0,0,0};
    char LUT_Q_4bit[16]={1,0,-1,0,0,0,0,0,3,0,-3,0,0,0,0,0};
    char MASK_2bit=0x03;
    char MASK_4bit_I=0x05;
    char MASK_4bit_Q=0x0A;

    /* determine  mode */
    if (((i_mode% 100)==32)|| ((i_mode% 100)==38)) {
        i_dtype =2;
    }
    if (((i_mode% 100)==36)|| ((i_mode% 100)==42)) {
        i_dtype =4;
    }

    for (i=0;i<n;i++) {
        switch (i_dtype) {
        case 2:
            expbuf[i_idx]=LUT_2bit[buf[i] & MASK_2bit];
            i_idx++;
            break;
        case 4:
            expbuf[i_idx]=LUT_I_4bit[buf[i] & MASK_4bit_I];
            i_idx++;
            expbuf[i_idx]=LUT_Q_4bit[buf[i] & MASK_4bit_Q];
            i_idx++;
            break;
        }
    }
}
/* get current data buffer (GN3S v3) -------------------------------------------
* get current data buffer from memory buffer
* args   : uint64_t buffloc I   buffer location
*          int    n         I   number of grab data
*          int    dtype     I   data type (DTYPEI or DTYPEIQ)
*          char   *expbuff  O   extracted data buffer
* return : none
*-----------------------------------------------------------------------------*/
extern void gn3s_getbuff_v3(uint64_t buffloc, int n, int dtype, char *expbuf)
{
    uint64_t membuffloc=buffloc%(MEMBUFFLEN*GN3S_BUFFSIZE);
    int nout=(int)((membuffloc+n)-(MEMBUFFLEN*GN3S_BUFFSIZE));

    mlock(hbuffmtx);
    if (nout>0) {
        gn3s_exp_v3(&sdrstat.buff[membuffloc],n-nout,GN3S_MODE,expbuf);
        gn3s_exp_v3(&sdrstat.buff[0],nout,GN3S_MODE,&expbuf[dtype*(n-nout)]);
    } else {
        gn3s_exp_v3(&sdrstat.buff[membuffloc],n,GN3S_MODE,expbuf);
    }
    unmlock(hbuffmtx);
}
/* push data to memory buffer --------------------------------------------------
* push data to memory buffer from GN3S front end
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
extern int gn3s_pushtomembuf(void)
{
    bool b_overrun=false;
    int nbuff;

    /* check buffer overrun */
    fx2_d.check_rx_overrun(&b_overrun);
    if (b_overrun) return -1;

    mlock(hbuffmtx);
    nbuff=fx2_d.read_IF(
        &sdrstat.buff[(sdrstat.buffcnt%MEMBUFFLEN)*GN3S_BUFFSIZE]);
    unmlock(hbuffmtx);

    if (nbuff!=GN3S_BUFFSIZE) {
        SDRPRINTF("GN3S read IF error...\n");
    }

    mlock(hreadmtx);
    sdrstat.buffcnt++;
    unmlock(hreadmtx);


    return 0;
}
/* push data to memory buffer --------------------------------------------------
* post-processing function: push data to memory buffer from GN3S binary IF file
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
extern void fgn3s_pushtomembuf(void)
{
    size_t nread;

    mlock(hbuffmtx);

    nread=fread(&sdrstat.buff[(sdrstat.buffcnt%MEMBUFFLEN)*GN3S_BUFFSIZE],
        1,GN3S_BUFFSIZE,sdrini.fp1);

    unmlock(hbuffmtx);

    if (nread<GN3S_BUFFSIZE) {
        sdrstat.stopflag=ON;
        SDRPRINTF("end of file!\n");
    }

    mlock(hreadmtx);
    sdrstat.buffcnt++;
    unmlock(hreadmtx);
}
/* get current data buffer from IF file ----------------------------------------
* post-processing function: get current data buffer from memory buffer
* args   : uint64_t buffloc I   buffer location
*          int    n         I   number of grab data
*          int    dtype     I   data type (DTYPEI or DTYPEIQ)
*          char   *expbuff  O   extracted data buffer
* return : none
*-----------------------------------------------------------------------------*/
extern void fgn3s_getbuff(uint64_t buffloc, int n, int dtype, char *expbuf)
{
    uint64_t membuffloc=dtype*buffloc%(MEMBUFFLEN*dtype*GN3S_BUFFSIZE);
    int nout;

    n=dtype*n;
    nout=(int)((membuffloc+n)-(MEMBUFFLEN*dtype*GN3S_BUFFSIZE));

    mlock(hbuffmtx);
    if (nout>0) {
        memcpy(expbuf,&sdrstat.buff[membuffloc],n-nout);
        memcpy(&expbuf[(n-nout)],&sdrstat.buff[0],nout);
    } else {
        memcpy(expbuf,&sdrstat.buff[membuffloc],n);
    }
    unmlock(hbuffmtx);
}
