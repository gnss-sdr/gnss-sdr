/*!
 * \file GLONASS_L1_L2_CA.h
 * \brief  Defines system parameters for GLONASS L1 C/A signal and NAV data
 * \note File renamed from GLONASS_L1_CA.h to GLONASS_L1_L2_CA.h to accommodate GLO L2 addition
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GLONASS_L1_L2_CA_H_
#define GNSS_SDR_GLONASS_L1_L2_CA_H_

#include "MATH_CONSTANTS.h"
#include "gnss_frequencies.h"
#include <cstdint>
#include <map>
#include <utility>  // std::pair
#include <vector>


// Physical constants
const double GLONASS_C_M_S = SPEED_OF_LIGHT;                   //!< The speed of light, [m/s]
const double GLONASS_C_M_MS = 299792.4580;                     //!< The speed of light, [m/ms]
const double GLONASS_PI = 3.1415926535898;                     //!< Pi as defined in IS-GPS-200E
const double GLONASS_TWO_PI = 6.283185307179586;               //!< 2Pi as defined in IS-GPS-200E
const double GLONASS_OMEGA_EARTH_DOT = 7.292115e-5;            //!< Earth rotation rate, [rad/s]
const double GLONASS_GM = 398600.4418e9;                       //!< Universal gravitational constant times the mass of the Earth, [m^3/s^2]
const double GLONASS_F_M_A = 0.35e9;                           //!< Gravitational constant of atmosphere [m^3/s^2]
const double GLONASS_SEMI_MAJOR_AXIS = 6378136;                //!< Semi-major axis of Earth [m]
const double GLONASS_FLATTENING = 1 / 29825784;                //!< Flattening parameter
const double GLONASS_GRAVITY = 97803284;                       //!< Equatorial acceleration of gravity [mGal]
const double GLONASS_GRAVITY_CORRECTION = 0.87;                //!< Correction to acceleration of gravity at sea-level due to Atmosphere[uGal]
const double GLONASS_J2 = 1082625.75e-9;                       //!< Second zonal harmonic of the geopotential
const double GLONASS_J4 = -2370.89e-9;                         //!< Fourth zonal harmonic of the geopotential
const double GLONASS_J6 = 6.08e-9;                             //!< Sixth zonal harmonic of the geopotential
const double GLONASS_J8 = 1.40e-11;                            //!< Eighth zonal harmonic of the geopotential
const double GLONASS_U0 = 62636861.4;                          //!< Normal potential at surface of common terrestrial ellipsoid [m^2/s^2]
const double GLONASS_C20 = -1082.63e-6;                        //!< Second zonal coefficient of spherical harmonic expansion
const double GLONASS_EARTH_RADIUS = 6378.136;                  //!< Equatorial radius of Earth [km]
const double GLONASS_EARTH_INCLINATION = 0.000409148809899e3;  //!< Mean inclination of ecliptic to equator (23 deg 26 min 33 sec) [rad]

const double GLONASS_TAU_0 = -0.005835151531174e3;  //!< (-334 deg 19 min 46.40 sec) [rad];
const double GLONASS_TAU_1 = 0.071018041257371e3;   //!< (4069 deg 02 min 02.52 sec) [rad];

const double GLONASS_MOON_Q0 = -0.001115184961435e3;          //!< (-63 deg 53 min 43.41 sec) [rad]
const double GLONASS_MOON_Q1 = 8.328691103668023e3;           //!< (477198 deg 50 min 56.79 sec) [rad]
const double GLONASS_MOON_OMEGA_0 = 0.004523601514852e3;      //!< (259 deg 10 min 59.79 sec) [rad]
const double GLONASS_MOON_OMEGA_1 = -0.033757146246552e3;     //!< (-1934 deg 08 min 31.23 sec) [rad]
const double GLONASS_MOON_GM = 4902.835;                      //!< Lunar gravitational constant [km^3/s^2]
const double GLONASS_MOON_SEMI_MAJOR_AXIS = 3.84385243e5;     //!< Semi-major axis of lunar orbit [km];
const double GLONASS_MOON_ECCENTRICITY = 0.054900489;         //!< Eccentricity of lunar orbit
const double GLONASS_MOON_INCLINATION = 0.000089803977407e3;  //!< Inclination of lunar orbit to ecliptic plane (5 deg 08 min 43.4 sec) [rad]

const double GLONASS_SUN_OMEGA = 0.004908229466869e3;  //!< TODO What is this operation in the seconds with T?(281 deg 13 min 15.0 + 6189.03 x T sec) [rad]
const double GLONASS_SUN_Q0 = 0.006256583774423e3;     //!< (358 deg 28 min 33.04 sec) [rad]
const double GLONASS_SUN_Q1 = 0e3;                     //!< TODO Why is the value greater than 60?(129596579.10 sec) [rad]
const double GLONASS_SUN_GM = 0.1325263e12;            //!< Solar gravitational constant [km^3/s^2]
const double GLONASS_SUN_SEMI_MAJOR_AXIS = 1.49598e8;  //!< Semi-major axis of solar orbit [km];
const double GLONASS_SUN_ECCENTRICITY = 0.016719;      //!< Eccentricity of solar orbit

const double GLONASS_L2_CA_FREQ_HZ = FREQ2_GLO;        //!< L2 [Hz]
const double GLONASS_L2_CA_DFREQ_HZ = DFRQ2_GLO;       //!< Freq Bias for GLONASS L1 [Hz]
const double GLONASS_L2_CA_CODE_RATE_HZ = 0.511e6;     //!< GLONASS L1 C/A code rate [chips/s]
const double GLONASS_L2_CA_CODE_LENGTH_CHIPS = 511.0;  //!< GLONASS L1 C/A code length [chips]
const double GLONASS_L2_CA_CODE_PERIOD = 0.001;        //!< GLONASS L1 C/A code period [seconds]
const double GLONASS_L2_CA_CHIP_PERIOD = 1.9569e-06;   //!< GLONASS L1 C/A chip period [seconds]
const double GLONASS_L2_CA_SYMBOL_RATE_BPS = 1000;

const double GLONASS_L1_CA_FREQ_HZ = FREQ1_GLO;        //!< L1 [Hz]
const double GLONASS_L1_CA_DFREQ_HZ = DFRQ1_GLO;       //!< Freq Bias for GLONASS L1 [Hz]
const double GLONASS_L1_CA_CODE_RATE_HZ = 0.511e6;     //!< GLONASS L1 C/A code rate [chips/s]
const double GLONASS_L1_CA_CODE_LENGTH_CHIPS = 511.0;  //!< GLONASS L1 C/A code length [chips]
const double GLONASS_L1_CA_CODE_PERIOD = 0.001;        //!< GLONASS L1 C/A code period [seconds]
const double GLONASS_L1_CA_CHIP_PERIOD = 1.9569e-06;   //!< GLONASS L1 C/A chip period [seconds]
const double GLONASS_L1_CA_SYMBOL_RATE_BPS = 1000;

const int32_t GLONASS_CA_NBR_SATS = 24;  // STRING DATA WITHOUT PREAMBLE

/*!
 * \brief Record of leap seconds definition for GLOT to GPST conversion and vice versa
 * \details Each entry is defined by an array of 7 elements consisting of yr,month,day,hr,min,sec,utc-gpst
 * \note Ideally should use leap seconds definitions of rtklib
 */
const double GLONASS_LEAP_SECONDS[19][7] = {
    {2017, 1, 1, 0, 0, 0, -18},
    {2015, 7, 1, 0, 0, 0, -17},
    {2012, 7, 1, 0, 0, 0, -16},
    {2009, 1, 1, 0, 0, 0, -15},
    {2006, 1, 1, 0, 0, 0, -14},
    {1999, 1, 1, 0, 0, 0, -13},
    {1997, 7, 1, 0, 0, 0, -12},
    {1996, 1, 1, 0, 0, 0, -11},
    {1994, 7, 1, 0, 0, 0, -10},
    {1993, 7, 1, 0, 0, 0, -9},
    {1992, 7, 1, 0, 0, 0, -8},
    {1991, 1, 1, 0, 0, 0, -7},
    {1990, 1, 1, 0, 0, 0, -6},
    {1988, 1, 1, 0, 0, 0, -5},
    {1985, 7, 1, 0, 0, 0, -4},
    {1983, 7, 1, 0, 0, 0, -3},
    {1982, 7, 1, 0, 0, 0, -2},
    {1981, 7, 1, 0, 0, 0, -1},
    {}};

//!< GLONASS SV's orbital slots PRN = (orbital_slot - 1)
const std::map<uint32_t, int32_t> GLONASS_PRN = {
    {
        0,
        8,
    },  //For test
    {
        1,
        1,
    },  //Plane 1
    {
        2,
        -4,
    },  //Plane 1
    {
        3,
        5,
    },  //Plane 1
    {
        4,
        6,
    },  //Plane 1
    {
        5,
        1,
    },  //Plane 1
    {
        6,
        -4,
    },  //Plane 1
    {
        7,
        5,
    },  //Plane 1
    {
        8,
        6,
    },  //Plane 1
    {
        9,
        -2,
    },  //Plane 2
    {
        10,
        -7,
    },  //Plane 2
    {
        11,
        0,
    },  //Plane 2
    {
        12,
        -1,
    },  //Plane 2
    {
        13,
        -2,
    },  //Plane 2
    {
        14,
        -7,
    },  //Plane 2
    {
        15,
        0,
    },  //Plane 2
    {
        16,
        -1,
    },  //Plane 2
    {
        17,
        4,
    },  //Plane 3
    {
        18,
        -3,
    },  //Plane 3
    {
        19,
        3,
    },  //Plane 3
    {
        20,
        -5,
    },  //Plane 3
    {
        21,
        4,
    },  //Plane 3
    {
        22,
        -3,
    },  //Plane 3
    {
        23,
        3,
    },         //Plane 3
    {24, 2}};  //Plane 3

const double GLONASS_STARTOFFSET_MS = 68.802;  //[ms] Initial sign. travel time (this cannot go here)

// OBSERVABLE HISTORY DEEP FOR INTERPOLATION
const int32_t GLONASS_L1_CA_HISTORY_DEEP = 100;

// NAVIGATION MESSAGE DEMODULATION AND DECODING
#define GLONASS_GNAV_PREAMBLE                                                                    \
    {                                                                                            \
        1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0 \
    }
const double GLONASS_GNAV_PREAMBLE_DURATION_S = 0.300;
const int32_t GLONASS_GNAV_PREAMBLE_LENGTH_BITS = 30;
const int32_t GLONASS_GNAV_PREAMBLE_LENGTH_SYMBOLS = 300;
const int32_t GLONASS_GNAV_PREAMBLE_PERIOD_SYMBOLS = 2000;
const int32_t GLONASS_GNAV_TELEMETRY_RATE_BITS_SECOND = 50;  //!< NAV message bit rate [bits/s]
const int32_t GLONASS_GNAV_TELEMETRY_SYMBOLS_PER_BIT = 10;
const int32_t GLONASS_GNAV_TELEMETRY_SYMBOLS_PER_PREAMBLE_BIT = 10;
const int32_t GLONASS_GNAV_TELEMETRY_RATE_SYMBOLS_SECOND = GLONASS_GNAV_TELEMETRY_RATE_BITS_SECOND * GLONASS_GNAV_TELEMETRY_SYMBOLS_PER_BIT;  //!< NAV message bit rate [symbols/s]
const int32_t GLONASS_GNAV_STRING_SYMBOLS = 2000;                                                                                             //!< Number of bits per string in the GNAV message (85 data bits + 30 time mark bits) [bits]
const int32_t GLONASS_GNAV_STRING_BITS = 85;                                                                                                  //!< Number of bits per string in the GNAV message (85 data bits + 30 time mark bits) [bits]
const int32_t GLONASS_GNAV_HAMMING_CODE_BITS = 8;                                                                                             //!< Number of bits in hamming code sequence of GNAV message
const int32_t GLONASS_GNAV_DATA_SYMBOLS = 1700;                                                                                               // STRING DATA WITHOUT PREAMBLE

const std::vector<int32_t> GLONASS_GNAV_CRC_I_INDEX{9, 10, 12, 13, 15, 17, 19, 20, 22, 24, 26, 28, 30, 32, 34, 35, 37, 39, 41, 43, 45, 47, 49, 51, 53, 55, 57, 59, 61, 63, 65, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84};
const std::vector<int32_t> GLONASS_GNAV_CRC_J_INDEX{9, 11, 12, 14, 15, 18, 19, 21, 22, 25, 26, 29, 30, 33, 34, 36, 37, 40, 41, 44, 45, 48, 49, 52, 53, 56, 57, 60, 61, 64, 65, 67, 68, 71, 72, 75, 76, 79, 80, 83, 84};
const std::vector<int32_t> GLONASS_GNAV_CRC_K_INDEX{10, 11, 12, 16, 17, 18, 19, 23, 24, 25, 26, 31, 32, 33, 34, 38, 39, 40, 41, 46, 47, 48, 49, 54, 55, 56, 57, 62, 63, 64, 65, 69, 70, 71, 72, 77, 78, 79, 80, 85};
const std::vector<int32_t> GLONASS_GNAV_CRC_L_INDEX{13, 14, 15, 16, 17, 18, 19, 27, 28, 29, 30, 31, 32, 33, 34, 42, 43, 44, 45, 46, 47, 48, 49, 58, 59, 60, 61, 62, 63, 64, 65, 73, 74, 75, 76, 77, 78, 79, 80};
const std::vector<int32_t> GLONASS_GNAV_CRC_M_INDEX{20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 81, 82, 83, 84, 85};
const std::vector<int32_t> GLONASS_GNAV_CRC_N_INDEX{35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65};
const std::vector<int32_t> GLONASS_GNAV_CRC_P_INDEX{66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85};
const std::vector<int32_t> GLONASS_GNAV_CRC_Q_INDEX{9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85};

// GLONASS GNAV NAVIGATION MESSAGE STRUCTURE
// NAVIGATION MESSAGE FIELDS POSITIONS (from IS-GPS-200E Appendix II)

// FRAME 1-4
// COMMON FIELDS
const std::vector<std::pair<int32_t, int32_t>> STRING_ID({{2, 4}});
const std::vector<std::pair<int32_t, int32_t>> KX({{78, 8}});
//STRING 1
const std::vector<std::pair<int32_t, int32_t>> P1({{8, 2}});
const std::vector<std::pair<int32_t, int32_t>> T_K_HR({{10, 5}});
const std::vector<std::pair<int32_t, int32_t>> T_K_MIN({{15, 6}});
const std::vector<std::pair<int32_t, int32_t>> T_K_SEC({{21, 1}});
const std::vector<std::pair<int32_t, int32_t>> X_N_DOT({{22, 24}});
const std::vector<std::pair<int32_t, int32_t>> X_N_DOT_DOT({{46, 5}});
const std::vector<std::pair<int32_t, int32_t>> X_N({{51, 27}});

//STRING 2
const std::vector<std::pair<int32_t, int32_t>> B_N({{6, 3}});
const std::vector<std::pair<int32_t, int32_t>> P2({{9, 1}});
const std::vector<std::pair<int32_t, int32_t>> T_B({{10, 7}});
const std::vector<std::pair<int32_t, int32_t>> Y_N_DOT({{22, 24}});
const std::vector<std::pair<int32_t, int32_t>> Y_N_DOT_DOT({{46, 5}});
const std::vector<std::pair<int32_t, int32_t>> Y_N({{51, 27}});

//STRING 3
const std::vector<std::pair<int32_t, int32_t>> P3({{6, 1}});
const std::vector<std::pair<int32_t, int32_t>> GAMMA_N({{7, 11}});
const std::vector<std::pair<int32_t, int32_t>> P({{19, 2}});
const std::vector<std::pair<int32_t, int32_t>> EPH_L_N({{21, 1}});
const std::vector<std::pair<int32_t, int32_t>> Z_N_DOT({{22, 24}});
const std::vector<std::pair<int32_t, int32_t>> Z_N_DOT_DOT({{46, 5}});
const std::vector<std::pair<int32_t, int32_t>> Z_N({{51, 27}});

// STRING 4
const std::vector<std::pair<int32_t, int32_t>> TAU_N({{6, 22}});
const std::vector<std::pair<int32_t, int32_t>> DELTA_TAU_N({{28, 5}});
const std::vector<std::pair<int32_t, int32_t>> E_N({{33, 5}});
const std::vector<std::pair<int32_t, int32_t>> P4({{52, 1}});
const std::vector<std::pair<int32_t, int32_t>> F_T({{53, 4}});
const std::vector<std::pair<int32_t, int32_t>> N_T({{60, 11}});
const std::vector<std::pair<int32_t, int32_t>> N({{71, 5}});
const std::vector<std::pair<int32_t, int32_t>> M({{76, 2}});

// STRING 5
const std::vector<std::pair<int32_t, int32_t>> DAY_NUMBER_A({{6, 11}});
const std::vector<std::pair<int32_t, int32_t>> TAU_C({{17, 32}});
const std::vector<std::pair<int32_t, int32_t>> N_4({{50, 5}});
const std::vector<std::pair<int32_t, int32_t>> TAU_GPS({{55, 22}});
const std::vector<std::pair<int32_t, int32_t>> ALM_L_N({{77, 1}});

// STRING 6, 8, 10, 12, 14
const std::vector<std::pair<int32_t, int32_t>> C_N({{6, 1}});
const std::vector<std::pair<int32_t, int32_t>> M_N_A({{7, 2}});
const std::vector<std::pair<int32_t, int32_t>> N_A({{9, 5}});
const std::vector<std::pair<int32_t, int32_t>> TAU_N_A({{14, 10}});
const std::vector<std::pair<int32_t, int32_t>> LAMBDA_N_A({{24, 21}});
const std::vector<std::pair<int32_t, int32_t>> DELTA_I_N_A({{45, 18}});
const std::vector<std::pair<int32_t, int32_t>> EPSILON_N_A({{63, 15}});

//STRING 7, 9, 11, 13, 15
const std::vector<std::pair<int32_t, int32_t>> OMEGA_N_A({{6, 16}});
const std::vector<std::pair<int32_t, int32_t>> T_LAMBDA_N_A({{22, 21}});
const std::vector<std::pair<int32_t, int32_t>> DELTA_T_N_A({{43, 22}});
const std::vector<std::pair<int32_t, int32_t>> DELTA_T_DOT_N_A({{65, 7}});
const std::vector<std::pair<int32_t, int32_t>> H_N_A({{72, 5}});

// STRING 14 FRAME 5
const std::vector<std::pair<int32_t, int32_t>> B1({{6, 11}});
const std::vector<std::pair<int32_t, int32_t>> B2({{17, 10}});


#endif /* GNSS_SDR_GLONASS_L1_L2_CA_H_ */
