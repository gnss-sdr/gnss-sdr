/*!
 * \file GPS_L1_CA.h
 * \brief  Defines system parameters for GPS L1 C/A signal and NAV data
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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


#ifndef GNSS_SDR_GPS_L1_CA_H_
#define GNSS_SDR_GPS_L1_CA_H_

#include <complex>
#include <gnss_satellite.h>

// Physical constants
const double GPS_C_m_s        = 299792458.0;      //!< The speed of light, [m/s]
const double GPS_C_m_ms       = 299792.4580;      //!< The speed of light, [m/ms]
const double GPS_PI           = 3.1415926535898;  //!< Pi as defined in IS-GPS-200E
const double GPS_TWO_PI       = 6.283185307179586;//!< 2Pi as defined in IS-GPS-200E
const double OMEGA_EARTH_DOT  = 7.2921151467e-5;  //!< Earth rotation rate, [rad/s]
const double GM               = 3.986005e14;      //!< Universal gravitational constant times the mass of the Earth, [m^3/s^2]
const double F                = -4.442807633e-10; //!< Constant, [s/(m)^(1/2)]


// carrier and code frequencies
const double  GPS_L1_FREQ_HZ              = 1.57542e9; //!< L1 [Hz]
const double  GPS_L1_CA_CODE_RATE_HZ      = 1.023e6;   //!< GPS L1 C/A code rate [chips/s]
const double  GPS_L1_CA_CODE_LENGTH_CHIPS = 1023.0;    //!< GPS L1 C/A code length [chips]

/*!
 * \brief Maximum Time-Of-Arrival (TOA) difference between satellites for a receiver operated on Earth surface is 20 ms
 *
 * According to the GPS orbit model described in [1] Pag. 32.
 * It should be taken into account to set the buffer size for the PRN start timestamp in the pseudoranges block.
 * [1] J. Bao-Yen Tsui, Fundamentals of Global Positioning System Receivers. A Software Approach, John Wiley & Sons,
 * Inc., Hoboken, NJ, 2nd edition, 2005.
 */
const double MAX_TOA_DELAY_MS = 20;



//#define NAVIGATION_SOLUTION_RATE_MS 1000 // this cannot go here
const double GPS_STARTOFFSET_ms = 68.802; //[ms] Initial sign. travel time (this cannot go here)


// NAVIGATION MESSAGE DEMODULATION AND DECODING

#define GPS_PREAMBLE {1, 0, 0, 0, 1, 0, 1, 1}
const int GPS_CA_PREAMBLE_LENGTH_BITS = 8;
const int GPS_CA_TELEMETRY_RATE_BITS_SECOND = 50;   //!< NAV message bit rate [bits/s]
const int GPS_CA_TELEMETRY_RATE_SYMBOLS_SECOND = GPS_CA_TELEMETRY_RATE_BITS_SECOND*20;   //!< NAV message bit rate [symbols/s]
const int GPS_WORD_LENGTH = 4;                      // CRC + GPS WORD (-2 -1 0 ... 29) Bits = 4 bytes
const int GPS_SUBFRAME_LENGTH=40;                // GPS_WORD_LENGTH x 10 = 40 bytes
const int GPS_SUBFRAME_BITS=300;               //!< Number of bits per subframe in the NAV message [bits]
const int GPS_SUBFRAME_SECONDS=6;				//!< Subframe duration [seconds]
const int GPS_WORD_BITS=30;                    //!< Number of bits per word in the NAV message [bits]


/*!
 *  \brief Navigation message bits slice structure: A portion of bits is indicated by
 *  the start position inside the subframe and the length in number of bits  */
typedef struct bits_slice
{
    int position;
    int length;
    bits_slice(int p,int l)
    {
        position=p;
        length=l;
    }
} bits_slice;


/* Constants for scaling the ephemeris found in the data message
        the format is the following: TWO_N5 -> 2^-5, TWO_P4 -> 2^4, PI_TWO_N43 -> Pi*2^-43, etc etc
        Additionally some of the PI*2^N terms are used in the tracking stuff
        TWO_PX ==> 2^X
        TWO_NX ==> 2^-X
        PI_TWO_NX ==> Pi*2^-X
        PI_TWO_PX ==> Pi*2^X
        ONE_PI_TWO_PX = (1/Pi)*2^X
*/
const double TWO_P4               =(16);                           //!< 2^4
const double TWO_P11              =(2048);                        //!< 2^11
const double TWO_P12              =(4096);                          //!< 2^12
const double TWO_P14              =(16384);                         //!< 2^14
const double TWO_P16              =(65536);                         //!< 2^16
const double TWO_P19              =(524288);                        //!< 2^19
const double TWO_P31              =(2147483648.0);                  //!< 2^31
const double TWO_P32              =(4294967296.0);                  //!< 2^32 this is too big for an int so add the x.0
const double TWO_P56              =(7.205759403792794e+016);        //!< 2^56
const double TWO_P57              =(1.441151880758559e+017);        //!< 2^57

const double TWO_N5               =(0.03125);                       //!< 2^-5
const double TWO_N11              =(4.882812500000000e-004);        //!< 2^-11
const double TWO_N19              =(1.907348632812500e-006);        //!< 2^-19
const double TWO_N20              =(9.536743164062500e-007);        //!< 2^-20
const double TWO_N21              =(4.768371582031250e-007);        //!< 2^-21
const double TWO_N24              =(5.960464477539063e-008);        //!< 2^-24
const double TWO_N25              =(2.980232238769531e-008);        //!< 2^-25
const double TWO_N27              =(7.450580596923828e-009);        //!< 2^-27
const double TWO_N29              =(1.862645149230957e-009);        //!< 2^-29
const double TWO_N30              =(9.313225746154785e-010);        //!< 2^-30
const double TWO_N31              =(4.656612873077393e-010);        //!< 2^-31
const double TWO_N32              =(2.328306436538696e-010);        //!< 2^-32
const double TWO_N33              =(1.164153218269348e-010);        //!< 2^-33
const double TWO_N38              =(3.637978807091713e-012);        //!< 2^-38
const double TWO_N43              =(1.136868377216160e-013);        //!< 2^-43
const double TWO_N50              =(8.881784197001252e-016);        //!< 2^-50
const double TWO_N55              =(2.775557561562891e-017);        //!< 2^-55


const double PI_TWO_N19           =(5.992112452678286e-006);        //!< Pi*2^-19
const double PI_TWO_N43           =(3.571577341960839e-013);        //!< Pi*2^-43
const double PI_TWO_N31           =(1.462918079267160e-009);        //!< Pi*2^-31
const double PI_TWO_N38           =(1.142904749427469e-011);        //!< Pi*2^-38
const double PI_TWO_N23           =(3.745070282923929e-007);        //!< Pi*2^-23



// GPS NAVIGATION MESSAGE STRUCTURE
// NAVIGATION MESSAGE FIELDS POSITIONS (from IS-GPS-200E Appendix II)

// SUBFRAME 1-5 (TLM and HOW)

const bits_slice TOW[]= {{31,17}};
const bits_slice INTEGRITY_STATUS_FLAG[] = {{23,1}};
const bits_slice ALERT_FLAG[] = {{48,1}};
const bits_slice ANTI_SPOOFING_FLAG[] = {{49,1}};
const bits_slice SUBFRAME_ID[]= {{50,3}};


// SUBFRAME 1
const bits_slice GPS_WEEK[]= {{61,10}};
const bits_slice CA_OR_P_ON_L2[]= {{71,2}}; //*
const bits_slice SV_ACCURACY[]= {{73,4}};
const bits_slice SV_HEALTH[]= {{77,6}};
const bits_slice L2_P_DATA_FLAG[] = {{91,1}};
const bits_slice T_GD[]= {{197,8}};
const double T_GD_LSB=TWO_N31;

const bits_slice IODC[]= {{83,2},{211,8}};
const bits_slice T_OC[]= {{219,16}};
const double T_OC_LSB=TWO_P4;

const bits_slice A_F2[]= {{241,8}};
const double A_F2_LSB=TWO_N55;
const bits_slice A_F1[]= {{249,16}};
const double A_F1_LSB=TWO_N43;
const bits_slice A_F0[]= {{271,22}};
const double A_F0_LSB=TWO_N31;

// SUBFRAME 2

const bits_slice IODE_SF2[]= {{61,8}};
const bits_slice C_RS[]= {{69,16}};
const double C_RS_LSB=TWO_N5;
const bits_slice DELTA_N[]= {{91,16}};
const double DELTA_N_LSB=PI_TWO_N43;
const bits_slice M_0[]= {{107,8},{121,24}};
const double M_0_LSB=PI_TWO_N31;
const bits_slice C_UC[]= {{151,16}};
const double C_UC_LSB=TWO_N29;
const bits_slice E[]= {{167,8},{181,24}};
const double E_LSB=TWO_N33;
const bits_slice C_US[]= {{211,16}};
const double C_US_LSB=TWO_N29;
const bits_slice SQRT_A[]= {{227,8},{241,24}};
const double SQRT_A_LSB=TWO_N19;
const bits_slice T_OE[]= {{271,16}};
const double T_OE_LSB=TWO_P4;
const bits_slice FIT_INTERVAL_FLAG[]= {{271,1}};
const bits_slice AODO[] = {{272,5}};
const int AODO_LSB = 900;

// SUBFRAME 3

const bits_slice C_IC[]= {{61,16}};
const double C_IC_LSB=TWO_N29;
const bits_slice OMEGA_0[]= {{77,8},{91,24}};
const double OMEGA_0_LSB=PI_TWO_N31;
const bits_slice C_IS[]= {{121,16}};
const double C_IS_LSB=TWO_N29;
const bits_slice I_0[]= {{137,8},{151,24}};
const double I_0_LSB=PI_TWO_N31;
const bits_slice C_RC[]= {{181,16}};
const double C_RC_LSB=TWO_N5;
const bits_slice OMEGA[]= {{197,8},{211,24}};
const double OMEGA_LSB=PI_TWO_N31;
const bits_slice OMEGA_DOT[]= {{241,24}};
const double OMEGA_DOT_LSB=PI_TWO_N43;
const bits_slice IODE_SF3[]= {{271,8}};

const bits_slice I_DOT[]= {{279,14}};
const double I_DOT_LSB=PI_TWO_N43;


// SUBFRAME 4-5

const bits_slice SV_DATA_ID[]= {{61,2}};
const bits_slice SV_PAGE[]= {{63,6}};

// SUBFRAME 4

//! \todo read all pages of subframe 4

// Page 18 - Ionospheric and UTC data
const bits_slice ALPHA_0[]= {{69,8}};
const double ALPHA_0_LSB=TWO_N30;
const bits_slice ALPHA_1[]= {{77,8}};
const double ALPHA_1_LSB=TWO_N27;
const bits_slice ALPHA_2[]= {{91,8}};
const double ALPHA_2_LSB=TWO_N24;
const bits_slice ALPHA_3[]= {{99,8}};
const double ALPHA_3_LSB=TWO_N24;
const bits_slice BETA_0[]= {{107,8}};
const double BETA_0_LSB=TWO_P11;
const bits_slice BETA_1[]= {{121,8}};
const double BETA_1_LSB=TWO_P14;
const bits_slice BETA_2[]= {{129,8}};
const double BETA_2_LSB=TWO_P16;
const bits_slice BETA_3[]= {{137,8}};
const double BETA_3_LSB=TWO_P16;
const bits_slice A_1[]= {{151,24}};
const double A_1_LSB=TWO_N50;
const bits_slice A_0[]= {{181,24},{211,8}};
const double A_0_LSB=TWO_N30;
const bits_slice T_OT[]= {{219,8}};
const double T_OT_LSB=TWO_P12;
const bits_slice WN_T[]= {{227,8}};
const double WN_T_LSB = 1;
const bits_slice DELTAT_LS[]= {{241,8}};
const double DELTAT_LS_LSB = 1;
const bits_slice WN_LSF[]= {{249,8}};
const double WN_LSF_LSB = 1;
const bits_slice DN[]= {{257,8}};
const double DN_LSB = 1;
const bits_slice DELTAT_LSF[]= {{271,8}};
const double DELTAT_LSF_LSB = 1;

// Page 25 - Antispoofing, SV config and SV health (PRN 25 -32)
const bits_slice HEALTH_SV25[]={{229,6}};
const bits_slice HEALTH_SV26[]={{241,6}};
const bits_slice HEALTH_SV27[]={{247,6}};
const bits_slice HEALTH_SV28[]={{253,6}};
const bits_slice HEALTH_SV29[]={{259,6}};
const bits_slice HEALTH_SV30[]={{271,6}};
const bits_slice HEALTH_SV31[]={{277,6}};
const bits_slice HEALTH_SV32[]={{283,6}};





// SUBFRAME 5
//! \todo read all pages of subframe 5


// page 25 - Health (PRN 1 - 24)
const bits_slice T_OA[]={{69,8}};
const double T_OA_LSB = TWO_P12;
const bits_slice WN_A[]={{77,8}};
const bits_slice HEALTH_SV1[]={{91,6}};
const bits_slice HEALTH_SV2[]={{97,6}};
const bits_slice HEALTH_SV3[]={{103,6}};
const bits_slice HEALTH_SV4[]={{109,6}};
const bits_slice HEALTH_SV5[]={{121,6}};
const bits_slice HEALTH_SV6[]={{127,6}};
const bits_slice HEALTH_SV7[]={{133,6}};
const bits_slice HEALTH_SV8[]={{139,6}};
const bits_slice HEALTH_SV9[]={{151,6}};
const bits_slice HEALTH_SV10[]={{157,6}};
const bits_slice HEALTH_SV11[]={{163,6}};
const bits_slice HEALTH_SV12[]={{169,6}};
const bits_slice HEALTH_SV13[]={{181,6}};
const bits_slice HEALTH_SV14[]={{187,6}};
const bits_slice HEALTH_SV15[]={{193,6}};
const bits_slice HEALTH_SV16[]={{199,6}};
const bits_slice HEALTH_SV17[]={{211,6}};
const bits_slice HEALTH_SV18[]={{217,6}};
const bits_slice HEALTH_SV19[]={{223,6}};
const bits_slice HEALTH_SV20[]={{229,6}};
const bits_slice HEALTH_SV21[]={{241,6}};
const bits_slice HEALTH_SV22[]={{247,6}};
const bits_slice HEALTH_SV23[]={{253,6}};
const bits_slice HEALTH_SV24[]={{259,6}};

/*

inline void ca_code_generator_complex(std::complex<float>* _dest, signed int _prn, unsigned int _chip_shift)
{

        unsigned int G1[1023];
        unsigned int G2[1023];
        unsigned int G1_register[10], G2_register[10];
        unsigned int feedback1, feedback2;
        unsigned int lcv, lcv2;
        unsigned int delay;
        signed int prn = _prn-1; //Move the PRN code to fit an array indices

        // G2 Delays as defined in IS-GPS-200E
        signed int delays[32] = {5, 6, 7, 8, 17, 18, 139, 140, 141, 251,
                252, 254, 255, 256, 257, 258, 469, 470, 471, 472,
                473, 474, 509, 512, 513, 514, 515, 516, 859, 860,
                861, 862};
        // PRN sequences 33 through 37 are reserved for other uses (e.g. ground transmitters)

        // A simple error check
        if((prn < 0) || (prn > 32))
                return;

        for(lcv = 0; lcv < 10; lcv++)
        {
                G1_register[lcv] = 1;
                G2_register[lcv] = 1;
        }

        // Generate G1 & G2 Register
        for(lcv = 0; lcv < 1023; lcv++)
        {
                G1[lcv] = G1_register[0];
                G2[lcv] = G2_register[0];

                feedback1 = G1_register[7]^G1_register[0];
                feedback2 = (G2_register[8] + G2_register[7] + G2_register[4] + G2_register[2] + G2_register[1] + G2_register[0]) & 0x1;

                for(lcv2 = 0; lcv2 < 9; lcv2++)
                {
                        G1_register[lcv2] = G1_register[lcv2 + 1];
                        G2_register[lcv2] = G2_register[lcv2 + 1];
                }

                G1_register[9] = feedback1;
                G2_register[9] = feedback2;
        }

        // Set the delay
        delay = 1023 - delays[prn];
        delay += _chip_shift;
        delay %= 1023;
        // Generate PRN from G1 and G2 Registers
        for(lcv = 0; lcv < 1023; lcv++)
        {
                _dest[lcv] = std::complex<float>(G1[(lcv +  _chip_shift)%1023]^G2[delay], 0);
                if(_dest[lcv].real() == 0.0) //javi
                {
                        _dest[lcv].real(-1.0);
                }
                delay++;
                delay %= 1023;
                //std::cout<<_dest[lcv].real(); //OK
        }
}

*/


#endif /* GNSS_SDR_GPS_L1_CA_H_ */
