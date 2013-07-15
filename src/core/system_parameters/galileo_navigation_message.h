/*!
 * \file galileo_navigation_message.h
 * \brief  Implementation of a Galileo NAV Data message decoder as described in Galileo ICD
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2013  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_GALILEO_NAVIGATION_MESSAGE_H_
#define GNSS_SDR_GALILEO_NAVIGATION_MESSAGE_H_

#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <algorithm>
#include <bitset>
#include "boost/assign.hpp"
#include <cmath>
#include <utility>

#include "Galileo_E1.h"



class Galileo_Navigation_Message {

private:


	//bool read_navigation_bool(std::bitset<GALILEO_SUBFRAME_BITS> bits, const std::vector<std::pair<int,int> > parameter);
	//void print_galileo_word_bytes(unsigned int GPS_word);
	unsigned long int read_navigation_unsigned(std::bitset<GALILEO_DATA_JK_BITS> bits, const std::vector< std::pair<int,int> > parameter);
	unsigned long int read_page_type_unsigned(std::bitset<GALILEO_PAGE_TYPE_BITS> bits, const std::vector< std::pair<int,int> > parameter);
	//signed long int read_navigation_signed(std::bitset<GALILEO_SUBFRAME_BITS> bits, const std::vector<std::pair<int,int> > parameter);

	int x,y;

public:

	/*Word type 1: Ephemeris (1/4)*/
	int IOD_nav_1;  // IOD_nav page 1
	double t0e; 	// Ephemeris reference time [s]
	double M0;		// Mean anomaly at reference time [semi-circles]
	double e;		// Eccentricity
	double A;   	// Square root of the semi-major axis [metres^1/2]

	/*Word type 2: Ephemeris (2/4)*/

	/*Word type 3: Ephemeris (3/4) and SISA*/

	/*Word type 4: Ephemeris (4/4) and Clock correction parameters*/

	/*Word type 5: Ionospheric correction, BGD, signal health and data validity status and GST*/

	/*Word type 6: GST-UTC conversion parameters*/

	/*Word type 7: Almanac for SVID1 (1/2), almanac reference time and almanac reference week number*/

	/*Word type 8: Almanac for SVID1 (2/2) and SVID2 (1/2)*/

	/*Word type 9: Almanac for SVID2 (2/2) and SVID3 (1/2)*/

	/*Word type 10: Almanac for SVID3 (2/2) and GST-GPS conversion parameters*/

	/*Word type 0: I/NAV Spare Word*/


	void split_page(char *page); 			/* Takes in input a page (Odd or Even) of 120 bit, split it according ICD 4.3.2.3 and join Data_k with Data_j*/

	int page_jk_decoder(char *data_jk);		/* Takes in input Data_jk (128 bit) and split it in ephemeris parameters according ICD 4.3.5*/

	void reset();


	/* Default constructor */

	Galileo_Navigation_Message();
};

#endif /* GALILEO_NAVIGATION_MESSAGE_H_ */
