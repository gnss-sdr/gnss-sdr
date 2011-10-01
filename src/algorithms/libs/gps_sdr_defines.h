/*----------------------------------------------------------------------------------------------*/
/*! \file defines.h
//
// FILENAME: defines.h
//
// DESCRIPTION: Important, non board-specific defines, many associated with IS-GPS-200D.
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

#ifndef DEFINES_H_
#define DEFINES_H_

/* For unnamed pipes */
/*----------------------------------------------------------------------------------------------*/
#define READ	(0)									//!< Read handle
#define WRITE	(1)									//!< Write handle
#define TRUE	(1)									//!< True?
#define FALSE	(0)									//!< False!
/*----------------------------------------------------------------------------------------------*/

/* First make some type prototypes */
/*----------------------------------------------------------------------------------------------*/
typedef unsigned char		uint8;					//!< Unsigned byte
typedef unsigned short		uint16;					//!< Unsigned word
//typedef unsigned long		uint32;					//!< Unsigned double word
typedef unsigned long long	uint64;					//!< Unsigned quadruple word
typedef signed char			int8;					//!< Signed byte
typedef signed short		int16;					//!< Signed word
//typedef signed long		int32;					//!< Signed double word
typedef signed long long	int64;					//!< Signed quadruple word
typedef signed int			int32;
typedef unsigned int		uint32;
/*----------------------------------------------------------------------------------------------*/

/* Constants for scaling the ephemeris found in the data message
	the format is the following: TWO_N5 -> 2^-5, TWO_P4 -> 2^4, PI_TWO_N43 -> Pi*2^-43, etc etc
	Additionally some of the PI*2^N terms are used in the tracking stuff
	TWO_PX ==> 2^X
	TWO_NX ==> 2^-X
	PI_TWO_NX ==> Pi*2^-X
	PI_TWO_PX ==> Pi*2^X
	ONE_PI_TWO_PX = (1/Pi)*2^X
*/
/*----------------------------------------------------------------------------------------------*/
#define TWO_P4				(16)						//!< 2^4
#define TWO_P11				(2048)						//!< 2^11
#define TWO_P12				(4096)						//!< 2^12
#define TWO_P14				(16384)						//!< 2^14
#define TWO_P16				(65536)						//!< 2^16
#define TWO_P19				(524288)					//!< 2^19
#define TWO_P31				(2147483648.0)				//!< 2^31
#define TWO_P32 			(4294967296.0)				//!< 2^32 this is too big for an int so add the x.0
#define TWO_P57 			(1.441151880758559e+017)	//!< 2^57

#define TWO_N5				(0.03125)					//!< 2^-5
#define TWO_N11				(4.882812500000000e-004)	//!< 2^-11
#define TWO_N19				(1.907348632812500e-006)	//!< 2^-19
#define TWO_N20				(9.536743164062500e-007)	//!< 2^-20
#define TWO_N21				(4.768371582031250e-007)	//!< 2^-21
#define TWO_N24				(5.960464477539063e-008)	//!< 2^-24
#define TWO_N25				(2.980232238769531e-008)	//!< 2^-25
#define TWO_N27				(7.450580596923828e-009)	//!< 2^-27
#define TWO_N29				(1.862645149230957e-009)	//!< 2^-29
#define TWO_N30				(9.313225746154785e-010)	//!< 2^-30
#define TWO_N31				(4.656612873077393e-010)	//!< 2^-31
#define TWO_N32				(2.328306436538696e-010)	//!< 2^-32
#define TWO_N33				(1.164153218269348e-010)	//!< 2^-33
#define TWO_N38				(3.637978807091713e-012)	//!< 2^-38
#define TWO_N43				(1.136868377216160e-013)	//!< 2^-43
#define TWO_N50				(8.881784197001252e-016)	//!< 2^-50
#define TWO_N55				(2.775557561562891e-017)	//!< 2^-55
#define TWO_P56				(7.205759403792794e+016)	//!< 2^56
#define TWO_P57 			(1.441151880758559e+017)	//!< 2^57

#define PI_TWO_N19			(5.992112452678286e-006)	//!< Pi*2^-19
#define PI_TWO_N43			(3.571577341960839e-013)	//!< Pi*2^-43
#define PI_TWO_N31			(1.462918079267160e-009)	//!< Pi*2^-31
#define PI_TWO_N38			(1.142904749427469e-011)	//!< Pi*2^-38
#define PI_TWO_N23			(3.745070282923929e-007)	//!< Pi*2^-23

/* from ICD-GPS-2000 !!! Do not extend the number of significant digits !!! */
#ifndef PI
#define PI 					(3.141592653589793)			//!< Pi
#endif
#ifndef TWO_PI
#define TWO_PI				(6.283185307179586)			//!< 2*Pi
#endif

#define THREE_PI_OVER_2     (4.7123889803)				//!< 3*Pi/2
#define PI_OVER_2			(1.5707963267)				//!< Pi/2
#define RAD_2_DEG			(57.29577951308232)			//!< 180/Pi
#define DEG_2_RAD			(0.01745329251994)			//!< Pi/180
/*----------------------------------------------------------------------------------------------*/


/* Channel Defines */
/*----------------------------------------------------------------------------------------------*/
#define FRAME_SIZE_PLUS_2 (12)					//!< Number of 30 bit words in subframe plus 2
#define PREAMBLE        (unsigned long)(0x008B)	//!< GPS data message preamble.
#define INVERSEPREAMBLE (unsigned long)(0x0074)	//!< Inverted preamble.
/*----------------------------------------------------------------------------------------------*/


/* PVT Stuff */
/*----------------------------------------------------------------------------------------------*/
#define GRAVITY_CONSTANT		(3.986005e14)				//!< Earth's WGS-84 gravitational constant (m^3/s^2) as specified in IS-GPS-200D
#define WGS84OE					(7.2921151467e-5)			//!< Earth's WGS-84 rotation rate (rads/s) as specified in IS-GPS-200D
#define WGS84_MAJOR_AXIS		(6378137.0)					//!< Earth's WGS-84 ellipsoid's major axis
#define WGS84_MINOR_AXIS		(6356752.314245)			//!< Earth's WGS-84 ellipsoid's minor axis
#define SECONDS_IN_1024_WEEKS	(619315200.0)				//!< Number of seconds in 1024 weeks
#define SECONDS_IN_WEEK			(604800.0)					//!< Number of seconds in a week
#define HALF_OF_SECONDS_IN_WEEK (302400.0)					//!< Number of seconds in one half of a week
#define SECONDS_IN_DAY			(86400.0)					//!< Number of seconds in a day
#define SECONDS_IN_HOUR			(3600.0)					//!< Number of seconds in an hour
#define SECONDS_IN_MINUTE		(60.0)						//!< Number of seconds in a minute
#define SECONDS_IN_HALF_HOUR	(1800.0)					//!< Number of seconds in a half hour
#define FOUR_HOURS				(14400.0)					//!< Number of seconds in 4 hrs
#define SECONDS_PER_MZCOUNT		(0.6)						//!< Seconds per RTCM-104 modified Z-count.
#define L1						(1.57542e9)
#define L2						(1.2276e9)
#define L1_WAVELENGTH			(1.902936727983649e-001)	//!< Meters
#define L1_OVER_C				(5.255035468570728)			//!< L1 over Speed O Light
#define C_OVER_L1				(0.190293672798365)			//!< Speed O Light over L1
#ifndef SPEED_OF_LIGHT
	#define SPEED_OF_LIGHT		(2.99792458e8)				//!< Speed of light (m/s) as specified in IS-GPS-200D
#endif
#define INVERSE_L1				(6.347513678891978e-10)		//!< 1/L1
#define INVERSE_SPEED_OF_LIGHT	(3.33564095198152049576e-9)	//!< Inverse speed of light
#define CODE_RATE 				(1.023e6)					//!< L1 C/A code chipping rate
#define INVERSE_CODE_RATE		(9.775171065493646e-07)		//!< 1/L1 C/A code chipping rate
#define CODE_CHIPS				(1023)
#define SAMPS_MS				(2048)						//!< All incoming signals are resampled to this sampling frequency
#define SAMPLE_FREQUENCY		(2048000) 					//!< All incoming signals are resampled to this sampling frequency
#define INVERSE_SAMPLE_FREQUENCY (4.882812500000000e-7)
/*----------------------------------------------------------------------------------------------*/


/* Global Serial Port */
/*----------------------------------------------------------------------------------------------*/
#define CHECK_STATUS(x)			if((x) != NU_SUCCESS) {ERC_System_Error((x));}
#define READ 					(0)
#define WRITE 					(1)
/*----------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------*/
#define NON_EXISTENT_SV			(666)
#define NON_EXISTENT_IODE		(9999)
#define STALE_SPS_VALUE			(99999)
/*----------------------------------------------------------------------------------------------*/

#endif //DEFINES_H_
