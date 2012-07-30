/*----------------------------------------------------------------------------------------------*/
//
// FILENAME: gps_source.h
//
// DESCRIPTION: Defines the GPS_Source class.
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
/*----------------------------------------------------------------------------------------------*/

#ifndef GN3S_SOURCE_H_
#define GN3S_SOURCE_H_

#include "gn3s_defines.h"
#include "gn3s.h"

/*! \ingroup CLASSES
 *
 */
class gn3s_Source
{

	private:

		/* Generic variables */
		int source_type;		//!< Source type
		int sample_mode;		//!< Sample mode
		int leftover;			//!< Leftover bytes for USRP double buffering
		int bwrite;			//!< Bytes somthing something?
		int ms_count;			//!< Count the numbers of ms processed

		bool flag_first_read;
		/* Tag overflows */
		time_t rawtime;
		struct tm * timeinfo;

		/* AGC Values */
		int agc_scale;		//!< To do the AGC
		int overflw;			//!< Overflow counter
		int soverflw;			//!< Overflow counter

		/* Data buffers */
		signed char gbuff[GN3S_SAMPS_5MS*2]; 	//!< Byte buffer for GN3S
		GN3S_CPX buff[GN3S_SAMPS_5MS]; 		//!< Base buffer for GN3S

		/* SOURCE_SIGE_GN3S Handles */
		gn3s *gn3s_a;

	private:

		void Open_GN3S();			//!< Open the SparkFun GN3S Sampler
		void Close_GN3S();			//!< Close the SparkFun GN3S Sampler
		void Read_GN3S(gn3s_ms_packet *_p,int n_samples);	//!< Read from the SparkFun GN3S Sampler

	public:

		gn3s_Source();	//!< Create the GPS source with the proper hardware type
		~gn3s_Source();					//!< Kill the object
		void Read(gn3s_ms_packet *_p,int n_samples);		//!< Read in a single ms of data
		int getScale(){return(agc_scale);}
		int getOvrflw(){return(overflw);}

};

#endif /* GN3S_SOURCE_H_ */
