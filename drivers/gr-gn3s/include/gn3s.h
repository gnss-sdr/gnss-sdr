/*----------------------------------------------------------------------------------------------*/
/*! \file gn3s.h
//
// FILENAME: gn3s.h
//
// DESCRIPTION: Defines the GN3S class.
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


#ifndef GN3S_H_
#define GN3S_H_


#define MODE_NARROW_16_I   32  /* SF:16.368MHz, IF:4.092MHz, 2bit, I */
#define MODE_NARROW_8_IQ   36  /* SF: 8.184MHz, IF:    0MHz, 4bit, IQ */
#define MODE_NARROW_5_I    38  /* SF: 5.456MHz, IF:4.092MHz, 2bit, I */
#define MODE_NARROW_4_IQ   42  /* SF: 4.092MHz, IF:    0MHz, 4bit, IQ */
#define MODE_WIDE_16_I     132 /* SF:16.368MHz, IF:4.092MHz, 2bit, I */
#define MODE_WIDE_8_IQ     136 /* SF: 8.184MHz, IF:    0MHz, 4bit, IQ */
#define MODE_WIDE_5_I      138 /* SF: 5.456MHz, IF:4.092MHz, 2bit, I */
#define MODE_WIDE_4_IQ     142 /* SF: 4.092MHz, IF:    0MHz, 4bit, IQ */

#define GN3S_MODE          MODE_NARROW_16_I /* GN3S operating mode */
#define GN3S_DECIMATE      1      /* decimate mode */
#define GN3S_PACK          1      /* pack mode */
#define GN3S_BUFFSIZE      32*512 /* buffer size: 16 kB */

/* global functions */
extern int  gn3s_init(void);
extern void gn3s_quit(void);
extern void gn3s_exp_v2(unsigned char *buf, int n, char *expbuf);
extern void gn3s_getbuff_v2(uint64_t buffloc, int n, int dtype, char *expbuf);
extern void gn3s_exp_v3(unsigned char *buf, int n, int i_mode, char *expbuf);
extern void gn3s_getbuff_v3(uint64_t buffloc, int n, int dtype, char *expbuf);
extern int  gn3s_pushtomembuf(void);
extern void fgn3s_pushtomembuf(void);
extern void fgn3s_getbuff(uint64_t buffloc, int n, int dtype, char *expbuf);


#endif
