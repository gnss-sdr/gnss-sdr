/*!
 * \file MATH_CONSTANTS.h
 * \brief  Defines useful mathematical constants and their scaled versions
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_MATH_CONSTANTS_H
#define GNSS_SDR_MATH_CONSTANTS_H

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


constexpr double GNSS_OMEGA_EARTH_DOT = 7.2921151467e-5;  //!< Default Earth rotation rate, [rad/s]
constexpr double SPEED_OF_LIGHT_M_S = 299792458.0;        //!< Speed of light in vacuum [m/s]
constexpr double SPEED_OF_LIGHT_M_MS = 299792.4580;       //!< Speed of light in vacuum [m/ms]

// Physical constants for GPS
constexpr double GPS_GM = 3.986005e14;      //!< Universal gravitational constant times the mass of the Earth, [m^3/s^2] IS-GPS-200M, 20.3.3.3.3.1
constexpr double GPS_F = -4.442807633e-10;  //!< Constant, [s/(m)^(1/2)], IS-GPS-200M, 20.3.3.3.3.1

// Physical constants for Galileo
constexpr double GALILEO_GM = 3.986004418e14;   //!< Geocentric gravitational constant[m^3/s^2], OS SIS ICD v2.0, pag. 44
constexpr double GALILEO_F = -4.442807309e-10;  //!< Constant, [s/(m)^(1/2)]. OS SIS ICD v2.0, pag. 47

// Physical constants for GLONASS
constexpr double GLONASS_OMEGA_EARTH_DOT = 7.292115e-5;  //!< Earth rotation rate, [rad/s] ICD L1, L2 GLONASS Edition 5.1 2008 pag. 55
constexpr double GLONASS_GM = 398600.44e9;               //!< Universal gravitational constant times the mass of the Earth, [m^3/s^2]

// Physical constants for Beidou
constexpr double BEIDOU_OMEGA_EARTH_DOT = 7.2921150e-5;  //!< Earth rotation rate, [rad/s] as defined in BDS-SIS-ICD-B1I-3.0 2019-02, pag. 3
constexpr double BEIDOU_GM = 3.986004418e14;             //!< Universal gravitational constant times the mass of the Earth, [m^3/s^2] as defined in CGCS2000
constexpr double BEIDOU_F = -4.442807309e-10;            //!< Constant, [s/(m)^(1/2)] F=-2(GM)^.5/C^2

constexpr double GNSS_PI = 3.1415926535898;  //!< pi constant as defined for GNSS
constexpr double HALF_PI = GNSS_PI / 2.0;    //!< pi/2
constexpr double TWO_PI = 2.0 * GNSS_PI;     //!< 2 * pi


// Constants for scaling the ephemeris found in the data message
// the format is the following: TWO_N5 -> 2^-5, TWO_P4 -> 2^4, PI_TWO_N43 -> Pi*2^-43, etc etc
// Additionally some of the PI*2^N terms are used in the tracking stuff
//   TWO_PX ==> 2^X
//   TWO_NX ==> 2^-X
//   PI_TWO_NX ==> Pi*2^-X

constexpr double TWO_P3 = 8.0;                      //!< 2^3
constexpr double TWO_P4 = 16.0;                     //!< 2^4
constexpr double TWO_P8 = 256.0;                    //!< 2^8
constexpr double TWO_P11 = 2048.0;                  //!< 2^11
constexpr double TWO_P12 = 4096.0;                  //!< 2^12
constexpr double TWO_P14 = 16384.0;                 //!< 2^14
constexpr double TWO_P16 = 65536.0;                 //!< 2^16
constexpr double TWO_P19 = 524288.0;                //!< 2^19
constexpr double TWO_P31 = 2147483648.0;            //!< 2^31
constexpr double TWO_P32 = 4294967296.0;            //!< 2^32
constexpr double TWO_P56 = 7.205759403792794e+016;  //!< 2^56
constexpr double TWO_P57 = 1.441151880758559e+017;  //!< 2^57

constexpr double TWO_N2 = 0.25;                     //!< 2^-2
constexpr double TWO_N5 = 0.03125;                  //!< 2^-5
constexpr double TWO_N6 = 0.015625;                 //!< 2^-6
constexpr double TWO_N8 = 0.00390625;               //!< 2^-8
constexpr double TWO_N9 = 0.001953125;              //!< 2^-9
constexpr double TWO_N10 = 0.0009765625;            //!< 2^-10
constexpr double TWO_N11 = 4.882812500000000e-004;  //!< 2^-11
constexpr double TWO_N14 = 0.00006103515625;        //!< 2^-14
constexpr double TWO_N15 = 3.051757812500000e-005;  //!< 2^-15
constexpr double TWO_N16 = 1.525878906250000e-005;  //!< 2^-16
constexpr double TWO_N17 = 7.629394531250000e-006;  //!< 2^-17
constexpr double TWO_N18 = 3.814697265625000e-006;  //!< 2^-18
constexpr double TWO_N19 = 1.907348632812500e-006;  //!< 2^-19
constexpr double TWO_N20 = 9.536743164062500e-007;  //!< 2^-20
constexpr double TWO_N21 = 4.768371582031250e-007;  //!< 2^-21
constexpr double TWO_N22 = 2.384185791015625e-007;  //!< 2^-22
constexpr double TWO_N23 = 1.192092895507810e-007;  //!< 2^-23
constexpr double TWO_N24 = 5.960464477539063e-008;  //!< 2^-24
constexpr double TWO_N25 = 2.980232238769531e-008;  //!< 2^-25
constexpr double TWO_N26 = 1.490116119384765e-009;  //!< 2^-26
constexpr double TWO_N27 = 7.450580596923828e-009;  //!< 2^-27
constexpr double TWO_N29 = 1.862645149230957e-009;  //!< 2^-29
constexpr double TWO_N30 = 9.313225746154785e-010;  //!< 2^-30
constexpr double TWO_N31 = 4.656612873077393e-010;  //!< 2^-31
constexpr double TWO_N32 = 2.328306436538696e-010;  //!< 2^-32
constexpr double TWO_N33 = 1.164153218269348e-010;  //!< 2^-33
constexpr double TWO_N34 = 5.82076609134674e-011;   //!< 2^-34
constexpr double TWO_N35 = 2.91038304567337e-011;   //!< 2^-35
constexpr double TWO_N38 = 3.637978807091713e-012;  //!< 2^-38
constexpr double TWO_N39 = 1.818989403545856e-012;  //!< 2^-39
constexpr double TWO_N40 = 9.094947017729280e-013;  //!< 2^-40
constexpr double TWO_N43 = 1.136868377216160e-013;  //!< 2^-43
constexpr double TWO_N44 = 5.684341886080802e-14;   //!< 2^-44
constexpr double TWO_N46 = 1.4210854715202e-014;    //!< 2^-46
constexpr double TWO_N48 = 3.552713678800501e-15;   //!< 2^-46

constexpr double TWO_N50 = 8.881784197001252e-016;                                //!< 2^-50
constexpr double TWO_N51 = 4.44089209850063e-016;                                 //!< 2^-51
constexpr double TWO_N55 = 2.775557561562891e-017;                                //!< 2^-55
constexpr double TWO_N57 = 6.938893903907228e-18;                                 //!< 2^-57
constexpr double TWO_N59 = 1.73472347597681e-018;                                 //!< 2^-59
constexpr double TWO_N60 = 8.673617379884036e-19;                                 //!< 2^-60
constexpr double TWO_N66 = 1.3552527156068805425093160010874271392822265625e-20;  //!< 2^-66
constexpr double TWO_N68 = 3.388131789017201e-21;                                 //!< 2^-68

constexpr double PI_TWO_N19 = 5.992112452678286e-006;  //!< Pi*2^-19
constexpr double PI_TWO_N43 = 3.571577341960839e-013;  //!< Pi*2^-43
constexpr double PI_TWO_N31 = 1.462918079267160e-009;  //!< Pi*2^-31
constexpr double PI_TWO_N38 = 1.142904749427469e-011;  //!< Pi*2^-38
constexpr double PI_TWO_N23 = 3.745070282923929e-007;  //!< Pi*2^-23

constexpr double D2R = GNSS_PI / 180.0;  //!< deg to rad
constexpr double R2D = 180.0 / GNSS_PI;  //!< rad to deg
constexpr double SC2RAD = GNSS_PI;       //!< semi-circle to radian (IS-GPS)
constexpr double AS2R = D2R / 3600.0;    //!< arc sec to radian

constexpr double AU = 149597870691.0;  //!< 1 Astronomical Unit AU (m) distance from Earth to the Sun.


/** \} */
/** \} */
#endif  // GNSS_SDR_MATH_CONSTANTS_H
