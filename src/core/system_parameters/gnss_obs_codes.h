/*!
 * \file gnss_obs_codes.h
 * \brief  GNSS Observable codes
 * \author Carles Fernandez, 2017. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GNSS_OBS_CODES_H
#define GNSS_SDR_GNSS_OBS_CODES_H

#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


constexpr uint32_t CODE_NONE = 0;  //!<   obs code: none or unknown
constexpr uint32_t CODE_L1C = 1;   //!<   obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS)
constexpr uint32_t CODE_L1P = 2;   //!<   obs code: L1P,G1P    (GPS,GLO)
constexpr uint32_t CODE_L1W = 3;   //!<   obs code: L1 Z-track (GPS)
constexpr uint32_t CODE_L1Y = 4;   //!<   obs code: L1Y        (GPS)
constexpr uint32_t CODE_L1M = 5;   //!<   obs code: L1M        (GPS)
constexpr uint32_t CODE_L1N = 6;   //!<   obs code: L1codeless (GPS)
constexpr uint32_t CODE_L1S = 7;   //!<   obs code: L1C(D)     (GPS,QZS)
constexpr uint32_t CODE_L1L = 8;   //!<   obs code: L1C(P)     (GPS,QZS)
constexpr uint32_t CODE_L1E = 9;   //!<   (not used)
constexpr uint32_t CODE_L1A = 10;  //!<   obs code: E1A        (GAL)
constexpr uint32_t CODE_L1B = 11;  //!<   obs code: E1B        (GAL)
constexpr uint32_t CODE_L1X = 12;  //!<   obs code: E1B+C,L1C(D+P) (GAL,QZS)
constexpr uint32_t CODE_L1Z = 13;  //!<   obs code: E1A+B+C,L1SAIF (GAL,QZS)
constexpr uint32_t CODE_L2C = 14;  //!<   obs code: L2C/A,G1C/A (GPS,GLO)
constexpr uint32_t CODE_L2D = 15;  //!<   obs code: L2 L1C/A-(P2-P1) (GPS)
constexpr uint32_t CODE_L2S = 16;  //!<   obs code: L2C(M)     (GPS,QZS)
constexpr uint32_t CODE_L2L = 17;  //!<   obs code: L2C(L)     (GPS,QZS)
constexpr uint32_t CODE_L2X = 18;  //!<   obs code: L2C(M+L),B1I+Q (GPS,QZS,BDS)
constexpr uint32_t CODE_L2P = 19;  //!<   obs code: L2P,G2P    (GPS,GLO)
constexpr uint32_t CODE_L2W = 20;  //!<   obs code: L2 Z-track (GPS)
constexpr uint32_t CODE_L2Y = 21;  //!<   obs code: L2Y        (GPS)
constexpr uint32_t CODE_L2M = 22;  //!<   obs code: L2M        (GPS)
constexpr uint32_t CODE_L2N = 23;  //!<   obs code: L2codeless (GPS)
constexpr uint32_t CODE_L5I = 24;  //!<   obs code: L5/E5aI    (GPS,GAL,QZS,SBS)
constexpr uint32_t CODE_L5Q = 25;  //!<   obs code: L5/E5aQ    (GPS,GAL,QZS,SBS)
constexpr uint32_t CODE_L5X = 26;  //!<   obs code: L5/E5aI+Q/L5B+C (GPS,GAL,QZS,IRN,SBS)
constexpr uint32_t CODE_L7I = 27;  //!<   obs code: E5bI,B2I   (GAL,BDS)
constexpr uint32_t CODE_L7Q = 28;  //!<   obs code: E5bQ,B2Q   (GAL,BDS)
constexpr uint32_t CODE_L7X = 29;  //!<   obs code: E5bI+Q,B2I+Q (GAL,BDS)
constexpr uint32_t CODE_L6A = 30;  //!<   obs code: E6A        (GAL)
constexpr uint32_t CODE_L6B = 31;  //!<   obs code: E6B        (GAL)
constexpr uint32_t CODE_L6C = 32;  //!<   obs code: E6C        (GAL)
constexpr uint32_t CODE_L6X = 33;  //!<   obs code: E6B+C,LEXS+L,B3I+Q (GAL,QZS,BDS)
constexpr uint32_t CODE_L6Z = 34;  //!<   obs code: E6A+B+C    (GAL)
constexpr uint32_t CODE_L6S = 35;  //!<   obs code: LEXS       (QZS)
constexpr uint32_t CODE_L6L = 36;  //!<   obs code: LEXL       (QZS)
constexpr uint32_t CODE_L8I = 37;  //!<   obs code: E5(a+b)I   (GAL)
constexpr uint32_t CODE_L8Q = 38;  //!<   obs code: E5(a+b)Q   (GAL)
constexpr uint32_t CODE_L8X = 39;  //!<   obs code: E5(a+b)I+Q (GAL)
constexpr uint32_t CODE_L2I = 40;  //!<   obs code: B1I        (BDS)
constexpr uint32_t CODE_L2Q = 41;  //!<   obs code: B1Q        (BDS)
constexpr uint32_t CODE_L6I = 42;  //!<   obs code: B3I        (BDS)
constexpr uint32_t CODE_L6Q = 43;  //!<   obs code: B3Q        (BDS)
constexpr uint32_t CODE_L3I = 44;  //!<   obs code: G3I        (GLO)
constexpr uint32_t CODE_L3Q = 45;  //!<   obs code: G3Q        (GLO)
constexpr uint32_t CODE_L3X = 46;  //!<   obs code: G3I+Q      (GLO)
constexpr uint32_t CODE_L1I = 47;  //!<   obs code: B1I        (BDS)
constexpr uint32_t CODE_L1Q = 48;  //!<   obs code: B1Q        (BDS)
constexpr uint32_t CODE_L5A = 49;  //!<   obs code: L5A SPS    (IRN)
constexpr uint32_t CODE_L5B = 50;  //!<   obs code: L5B RS(D)  (IRN)
constexpr uint32_t CODE_L5C = 51;  //!<   obs code: L5C RS(P)  (IRN)
constexpr uint32_t CODE_L9A = 52;  //!<   obs code: SA SPS     (IRN)
constexpr uint32_t CODE_L9B = 53;  //!<   obs code: SB RS(D)   (IRN)
constexpr uint32_t CODE_L9C = 54;  //!<   obs code: SC RS(P)   (IRN)
constexpr uint32_t CODE_L9X = 55;  //!<   obs code: SB+C       (IRN)
constexpr int32_t MAXCODE = 55;    //!<   max number of obs code


/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_OBS_CODES_H
