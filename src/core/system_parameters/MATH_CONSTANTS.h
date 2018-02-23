/*!
 * \file MATH_CONSTANTS.h
 * \brief  Defines useful mathematical constants and their scaled versions
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_MATH_CONSTANTS_H_
#define GNSS_SDR_MATH_CONSTANTS_H_

#include<string>

/* Constants for scaling the ephemeris found in the data message
        the format is the following: TWO_N5 -> 2^-5, TWO_P4 -> 2^4, PI_TWO_N43 -> Pi*2^-43, etc etc
        Additionally some of the PI*2^N terms are used in the tracking stuff
        TWO_PX ==> 2^X
        TWO_NX ==> 2^-X
        PI_TWO_NX ==> Pi*2^-X
        PI_TWO_PX ==> Pi*2^X
        ONE_PI_TWO_PX = (1/Pi)*2^X
*/

const double PI = 3.1415926535897932;             //!<  pi
const double PI_2 = 2.0 * PI;                     //!<  2 * pi

const double TWO_P4 = (16);                       //!< 2^4
const double TWO_P11 = (2048);                    //!< 2^11
const double TWO_P12 = (4096);                    //!< 2^12
const double TWO_P14 = (16384);                   //!< 2^14
const double TWO_P16 = (65536);                   //!< 2^16
const double TWO_P19 = (524288);                  //!< 2^19
const double TWO_P31 = (2147483648.0);            //!< 2^31
const double TWO_P32 = (4294967296.0);            //!< 2^32 this is too big for an int so add the x.0
const double TWO_P56 = (7.205759403792794e+016);  //!< 2^56
const double TWO_P57 = (1.441151880758559e+017);  //!< 2^57

const double TWO_N2 = (0.25);                     //!< 2^-2
const double TWO_N5 = (0.03125);                  //!< 2^-5
const double TWO_N6 = (0.015625);                 //!< 2^-6
const double TWO_N8 = (0.00390625);               //!< 2^-8
const double TWO_N9 = (0.001953125);              //!< 2^-9
const double TWO_N10 = (0.0009765625);            //!< 2^-10
const double TWO_N11 = (4.882812500000000e-004);  //!< 2^-11
const double TWO_N14 = (0.00006103515625);        //!< 2^-14
const double TWO_N15 = (0.00003051757813);        //!< 2^-15
const double TWO_N16 = (0.0000152587890625);      //!< 2^-16
const double TWO_N17 = (7.629394531250000e-006);  //!< 2^-17
const double TWO_N18 = (3.814697265625000e-006);  //!< 2^-18
const double TWO_N19 = (1.907348632812500e-006);  //!< 2^-19
const double TWO_N20 = (9.536743164062500e-007);  //!< 2^-20
const double TWO_N21 = (4.768371582031250e-007);  //!< 2^-21
const double TWO_N23 = (1.192092895507810e-007);  //!< 2^-23
const double TWO_N24 = (5.960464477539063e-008);  //!< 2^-24
const double TWO_N25 = (2.980232238769531e-008);  //!< 2^-25
const double TWO_N27 = (7.450580596923828e-009);  //!< 2^-27
const double TWO_N29 = (1.862645149230957e-009);  //!< 2^-29
const double TWO_N30 = (9.313225746154785e-010);  //!< 2^-30
const double TWO_N31 = (4.656612873077393e-010);  //!< 2^-31
const double TWO_N32 = (2.328306436538696e-010);  //!< 2^-32
const double TWO_N33 = (1.164153218269348e-010);  //!< 2^-33
const double TWO_N34 = (5.82076609134674e-011);   //!< 2^-34
const double TWO_N35 = (2.91038304567337e-011);   //!< 2^-35
const double TWO_N38 = (3.637978807091713e-012);  //!< 2^-38
const double TWO_N39 = (1.818989403545856e-012);  //!< 2^-39
const double TWO_N40 = (9.094947017729280e-013);  //!< 2^-40
const double TWO_N43 = (1.136868377216160e-013);  //!< 2^-43
const double TWO_N44 = (5.684341886080802e-14);   //!< 2^-44
const double TWO_N46 = (1.4210854715202e-014);    //!< 2^-46
const double TWO_N48 = (3.552713678800501e-15);   //!< 2^-46

const double TWO_N50 = (8.881784197001252e-016);  //!< 2^-50
const double TWO_N51 = (4.44089209850063e-016);   //!< 2^-51
const double TWO_N55 = (2.775557561562891e-017);  //!< 2^-55
const double TWO_N57 = (6.938893903907228e-18);   //!< 2^-57
const double TWO_N59 = (1.73472347597681e-018);   //!< 2^-59
const double TWO_N60 = (8.673617379884036e-19);   //!< 2^-60
const double TWO_N68 = (3.388131789017201e-21);   //!< 2^-68


const double PI_TWO_N19 = (5.992112452678286e-006);  //!< Pi*2^-19
const double PI_TWO_N43 = (3.571577341960839e-013);  //!< Pi*2^-43
const double PI_TWO_N31 = (1.462918079267160e-009);  //!< Pi*2^-31
const double PI_TWO_N38 = (1.142904749427469e-011);  //!< Pi*2^-38
const double PI_TWO_N23 = (3.745070282923929e-007);  //!< Pi*2^-23

const double D2R = (PI/180.0);          //!< deg to rad
const double R2D = (180.0/PI);          //!< rad to deg
const double SC2RAD = 3.1415926535898;  //!<  semi-circle to radian (IS-GPS)
const double AS2R = (D2R / 3600.0);     //!<  arc sec to radian

const double DEFAULT_OMEGA_EARTH_DOT = 7.2921151467e-5; //!<  Default Earth rotation rate, [rad/s]
const double SPEED_OF_LIGHT = 299792458.0;              //!<  [m/s]
const double AU = 149597870691.0;                       //!<  1 Astronomical Unit AU (m) distance from Earth to the Sun.

#endif /* GNSS_SDR_MATH_CONSTANTS_H_ */
