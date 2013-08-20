/*!
 * \file Galileo_Navigation_Message.cc
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

#include "galileo_navigation_message.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/crc.hpp>      // for boost::crc_basic, boost::crc_optimal
#include <boost/dynamic_bitset.hpp>


#include <iostream>
#include <cstring>
#include <string>

typedef boost::crc_optimal<24, 0x1864CFBu, 0x0, 0x0, false, false> CRC_Galileo_INAV_type;


void Galileo_Navigation_Message::reset()
{
	 flag_even_word = 0;

	 flag_all_ephemeris = false; // flag indicating that all words containing ephemeris have been received
	 flag_ephemeris_1 = false;    // flag indicating that ephemeris 1/4 (word 1) have been received
	 flag_ephemeris_2 = false;    // flag indicating that ephemeris 2/4 (word 2) have been received
	 flag_ephemeris_3 = false;    // flag indicating that ephemeris 3/4 (word 3) have been received
	 flag_ephemeris_4 = false;    // flag indicating that ephemeris 4/4 (word 4) have been received

	 flag_iono_and_GST = false;   // flag indicating that ionospheric parameters (word 5) have been received
	 flag_utc_model = false;      // flag indicating that utc model parameters (word 6) have been received

	 flag_all_almanac = false;	  // flag indicating that all almanac have been received
	 flag_almanac_1 = false;      // flag indicating that almanac 1/4 (word 7) have been received
	 flag_almanac_2 = false;      // flag indicating that almanac 2/4 (word 8) have been received
	 flag_almanac_3 = false;      // flag indicating that almanac 3/4 (word 9) have been received
	 flag_almanac_4 = false;      // flag indicating that almanac 4/4 (word 10) have been received



	 /*Word type 1: Ephemeris (1/4)*/
	 IOD_nav_1 = 0;
	 t0e_1 = 0;
	 M0_1 = 0;
	 e_1 = 0;
	 A_1 = 0;

	 /*Word type 2: Ephemeris (2/4)*/
	 IOD_nav_2 = 0;  // IOD_nav page 2
	 OMEGA_0_2 = 0; // Longitude of ascending node of orbital plane at weekly epoch [semi-circles]
	 i_0_2 = 0;     // Inclination angle at reference time  [semi-circles]
	 omega_2 = 0;   // Argument of perigee [semi-circles]
	 iDot_2 = 0;    // Rate of inclination angle [semi-circles/sec]


	 /*Word type 3: Ephemeris (3/4) and SISA*/
	 IOD_nav_3 = 0;  		//
	 OMEGA_dot_3 = 0;		// Rate of right ascension [semi-circles/sec]
	 delta_n_3 = 0;		    // Mean motion difference from computed value  [semi-circles/sec]
	 C_uc_3 = 0;			// Amplitude of the cosine harmonic correction term to the argument of latitude [radians]
	 C_us_3 = 0;			// Amplitude of the sine harmonic correction term to the argument of latitude [radians]
	 C_rc_3 = 0;			// Amplitude of the cosine harmonic correction term to the orbit radius [meters]
	 C_rs_3 = 0;			// Amplitude of the sine harmonic correction term to the orbit radius [meters]
	 SISA_3 = 0;			//


	 /*Word type 4: Ephemeris (4/4) and Clock correction parameters*/
	 IOD_nav_4 = 0;		//
	 SV_ID_PRN_4 = 0;	//
	 C_ic_4 = 0;		// Amplitude of the cosine harmonic correction 	term to the angle of inclination [radians]
	 C_is_4 = 0;		// Amplitude of the sine harmonic correction term to the angle of inclination [radians]
	 /*Clock correction parameters*/
	 t0c_4 = 0;			//
	 af0_4 = 0;			//
	 af1_4 = 0;			//
	 af2_4 = 0;			//
	 spare_4 = 0;

	 /*Word type 5: Ionospheric correction, BGD, signal health and data validity status and GST*/
	 /*Ionospheric correction*/
	 /*Az*/
	 ai0_5 = 0;		//
	 ai1_5 = 0;		//
	 ai2_5 = 0;		//
	 /*Ionospheric disturbance flag*/
	 Region1_flag_5 = 0;	//Region1_flag_5;
	 Region2_flag_5 = 0;	//
	 Region3_flag_5 = 0;	//
	 Region4_flag_5 = 0;	//
	 Region5_flag_5 = 0;	//
	 BGD_E1E5a_5 = 0;	//
	 BGD_E1E5b_5 = 0;	//
	 E5b_HS_5 = 0;		//
	 E1B_HS_5 = 0;		//
	 E5b_DVS_5 = 0;	//
	 E1B_DVS_5 = 0;	//
	/*GST*/
	 WN_5 = 0;
	 TOW_5 = 0;
	 spare_5 = 0;

	 /*Word type 6: GST-UTC conversion parameters*/
	 A0_6 = 0;
	 A1_6 = 0;
	 Delta_tLS_6 = 0;
	 t0t_6 = 0;
	 WNot_6 = 0;
	 WN_LSF_6 = 0;
	 DN_6 = 0;
	 Delta_tLSF_6 = 0;
	 TOW_6 = 0;

	 /*Word type 7: Almanac for SVID1 (1/2), almanac reference time and almanac reference week number*/
	 IOD_a_7 = 0;
	 WN_a_7 = 0;
	 t0a_7 = 0;
	 SVID1_7 = 0;
	 DELTA_A_7 = 0;
	 e_7 = 0;
	 omega_7 = 0;
	 delta_i_7 = 0;
	 Omega0_7 = 0;
	 Omega_dot_7 = 0;
	 M0_7 = 0;

	/*Word type 8: Almanac for SVID1 (2/2) and SVID2 (1/2)*/
	IOD_a_8 = 0;
	af0_8 = 0;
	af1_8 = 0;
	E5b_HS_8 = 0;
	E1B_HS_8 = 0;
	SVID2_8 = 0;
	DELTA_A_8 = 0;
	e_8 = 0;
	omega_8 = 0;
	delta_i_8 = 0;
	Omega0_8 = 0;
	Omega_dot_8 = 0;

	/*Word type 9: Almanac for SVID2 (2/2) and SVID3 (1/2)*/

	IOD_a_9 = 0;
	WN_a_9 = 0;
	t0a_9 = 0;
	M0_9 = 0;
	af0_9 = 0;
	af1_9 = 0;
	E5b_HS_9 = 0;
	E1B_HS_9 = 0;
	SVID3_9 = 0;
	DELTA_A_9 = 0;
	e_9 = 0;
	omega_9 = 0;
	delta_i_9 = 0;

	/*Word type 10: Almanac for SVID3 (2/2) and GST-GPS conversion parameters*/

	IOD_a_10 = 0;
	Omega0_10 = 0;
	Omega_dot_10 = 0;
	M0_10 = 0;
	af0_10 = 0;
	af1_10 = 0;
	E5b_HS_10 = 0;
	E1B_HS_10 = 0;
	//GST-GPS
	A_0G_10 = 0;
	A_1G_10 = 0;
	t_0G_10 = 0;
	WN_0G_10 = 0;

	/*Word type 0: I/NAV Spare Word*/

	Time_0 = 0;
	WN_0 = 0;
	TOW_0 = 0;

}


Galileo_Navigation_Message::Galileo_Navigation_Message()
{
    reset();
}


bool Galileo_Navigation_Message::CRC_test(std::bitset<GALILEO_DATA_FRAME_BITS> bits,boost::uint32_t checksum)
{

	CRC_Galileo_INAV_type CRC_Galileo;

    boost::uint32_t crc_computed;
    // Galileo INAV frame for CRC is not an integer multiple of bytes
    // it needs to be filled with zeroes at the start of the frame.
    // This operation is done in the transformation from bits to bytes
    // using boost::dynamic_bitset.
    // ToDo: Use boost::dynamic_bitset for all the bitset operations in this class

    boost::dynamic_bitset<unsigned char> frame_bits(std::string(bits.to_string()));

    std::vector<unsigned char> bytes;
    boost::to_block_range(frame_bits, std::back_inserter(bytes));
    std::reverse(bytes.begin(),bytes.end());


    CRC_Galileo.process_bytes( bytes.data(), GALILEO_DATA_FRAME_BYTES );

    crc_computed=CRC_Galileo.checksum();
    if (checksum==crc_computed){
    	return true;
    }else{
    	return false;
    }


}
unsigned long int Galileo_Navigation_Message::read_navigation_unsigned(std::bitset<GALILEO_DATA_JK_BITS> bits, const std::vector<std::pair<int,int> > parameter)
{
    unsigned long int value = 0;
    int num_of_slices = parameter.size();
    for (int i=0; i<num_of_slices; i++)
        {
            for (int j=0; j<parameter[i].second; j++)
                {
                    value <<= 1; //shift left
                    if (bits[GALILEO_DATA_JK_BITS - parameter[i].first - j] == 1)
                        {
                            value += 1; // insert the bit
                        }
                }
        }
    return value;
}



unsigned long int Galileo_Navigation_Message::read_page_type_unsigned(std::bitset<GALILEO_PAGE_TYPE_BITS> bits, const std::vector<std::pair<int,int> > parameter)
{
    unsigned long int value = 0;
    int num_of_slices = parameter.size();
    for (int i=0; i<num_of_slices; i++)
        {
            for (int j=0; j<parameter[i].second; j++)
                {
                    value <<= 1; //shift left
                    if (bits[GALILEO_PAGE_TYPE_BITS - parameter[i].first - j] == 1)
                        {
                            value += 1; // insert the bit
                        }
                }
        }
    return value;
}



signed long int Galileo_Navigation_Message::read_navigation_signed(std::bitset<GALILEO_DATA_JK_BITS> bits, const std::vector<std::pair<int,int> > parameter)
{
    signed long int value = 0;
    int num_of_slices = parameter.size();
    // Discriminate between 64 bits and 32 bits compiler
    int long_int_size_bytes = sizeof(signed long int);
    if (long_int_size_bytes == 8) // if a long int takes 8 bytes, we are in a 64 bits system
        {
            // read the MSB and perform the sign extension
            if (bits[GALILEO_DATA_JK_BITS - parameter[0].first] == 1)
                {
                    value ^= 0xFFFFFFFFFFFFFFFF; //64 bits variable
                }
            else
                {
                    value &= 0;
                }

            for (int i=0; i<num_of_slices; i++)
                {
                    for (int j=0; j<parameter[i].second; j++)
                        {
                            value <<= 1; //shift left
                            value &= 0xFFFFFFFFFFFFFFFE; //reset the corresponding bit (for the 64 bits variable)
                            if (bits[GALILEO_DATA_JK_BITS - parameter[i].first - j] == 1)
                                {
                                    value += 1; // insert the bit
                                }
                        }
                }
        }
    else  // we assume we are in a 32 bits system
        {
            // read the MSB and perform the sign extension
            if (bits[GALILEO_DATA_JK_BITS - parameter[0].first] == 1)
                {
                    value ^= 0xFFFFFFFF;
                }
            else
                {
                    value &= 0;
                }

            for (int i=0; i<num_of_slices; i++)
                {
                    for (int j=0; j<parameter[i].second; j++)
                        {
                            value <<= 1; //shift left
                            value &= 0xFFFFFFFE; //reset the corresponding bit
                            if (bits[GALILEO_DATA_JK_BITS - parameter[i].first - j] == 1)
                                {
                                    value += 1; // insert the bit
                                }
                        }
                }
        }
    return value;
}

bool Galileo_Navigation_Message::read_navigation_bool(std::bitset<GALILEO_DATA_JK_BITS> bits, const std::vector<std::pair<int,int> > parameter)
{
    bool value;

    if (bits[GALILEO_DATA_JK_BITS - parameter[0].first] == 1)
        {
            value = true;
        }
    else
        {
            value = false;
        }
    return value;
}

/*void Galileo_Navigation_Message::print_galileo_word_bytes(unsigned int GPS_word)
{
    std::cout << " Word =";
    std::cout << std::bitset<32>(GPS_word);
    std::cout << std::endl;
}*/


void Galileo_Navigation_Message::split_page(const char *page, int flag_even_word){

	// ToDo: Clean all the tests and create an independent google test code for the telemetry decoder.


	std::cout << "Entered in Galileo_Navigation_Message::split_page(const char *page, int flag_even_word" << std::endl << std::endl;;
	std::string page_string = page;
	//char correct_tail[7]="011110"; //the viterbi decoder output change the tail to this value (why?)

	char correct_tail[7]="000000";

	int Page_type=0;
	static std::string page_Even; //declare in this way it can "remember the previous even page while reading the odd page..ok!

	std::cout << "Start decoding Galileo I/NAV " << std::endl;

	if(page_string.at(0)=='1')// if page is odd
	{
		//std::cout<< "page_string.at(0) split page="<<page_string.at(0) << std::endl;
		std::string page_Odd = page_string; //chiamo la stringa sembre page_Odd
		//std::cout<<"Page odd string in split page"<< std::endl << page_Odd << std::endl;

		if (flag_even_word==1)/*Under this condition An odd page has been received but the previous even page is kept in memory and it is considered to join pages*/
				{
					//std::cout<<"previous page even "<< std::endl << page_Even << std::endl;
					std::string page_INAV_even = page_Even;
					//std::cout << "page inav solo even" << page_INAV_even << std::endl;
					std::string page_INAV = page_INAV_even + page_Odd; //Join pages: Even+Odd=INAV page
					//std::cout << "page inav eve +odd " << page_INAV<< std::endl;
					std::string Even_bit = page_INAV.substr (0,1);
					//std::cout << "Even bit = " << Even_bit << endl;
					std::string Page_type_even = page_INAV.substr (1,1);
					//std::cout << "Page type even = " << Page_type_even << endl;
					std::string nominal = "0";

					if (Page_type_even.compare(nominal) != 0)
							std::cout << "Alert frame "<< std::endl;
					else std::cout << "Nominal Page" << std::endl;

					std::string Data_k = page_INAV.substr (2,112);
					//std::cout << "Data_k " << endl << Data_k << endl;
					std::string Odd_bit = page_INAV.substr (114,1);
					std::string Page_type_Odd = page_INAV.substr (115,1);
					//std::cout << "Page_type_Odd: " << Page_type_Odd << endl;
					std::string Data_j = page_INAV.substr (116,16);
					//std::cout << "Data_j: " << Data_j << endl;

					std::string Reserved_1 = page_INAV.substr (132,40);
					std::string SAR = page_INAV.substr (172,22);
					std::string Spare = page_INAV.substr (194,2);
					std::string CRC_data = page_INAV.substr (196,24);
					std::string Reserved_2 = page_INAV.substr (220,8);
					std::string Tail_odd = page_INAV.substr (228,6);

					//************ CRC checksum control *******/
					std::stringstream TLM_word_for_CRC_stream;

					TLM_word_for_CRC_stream<<page_INAV;
					std::string TLM_word_for_CRC;
					TLM_word_for_CRC=TLM_word_for_CRC_stream.str().substr(0,GALILEO_DATA_FRAME_BITS);

					std::bitset<GALILEO_DATA_FRAME_BITS> TLM_word_for_CRC_bits(TLM_word_for_CRC);
					std::bitset<24> checksum(CRC_data);

					//if (Tail_odd.compare(correct_tail) != 0)
					//		std::cout << "Tail odd is not correct!" << std::endl;
					//else std::cout<<"Tail odd is correct!"<<std::endl;

					if (CRC_test(TLM_word_for_CRC_bits,checksum.to_ulong())==true)
					{
						// CRC correct: Decode word
						std::cout<<"CRC correct!"<<std::endl;

						std::string page_number_bits = Data_k.substr (0,6);
						//std::cout << "Page number bits from Data k" << std::endl << page_number_bits << std::endl;

						std::bitset<GALILEO_PAGE_TYPE_BITS> page_type_bits (page_number_bits); // from string to bitset
						Page_type = (int)read_page_type_unsigned(page_type_bits, type);
						std::cout << "Page number (first 6 bits of Data k converted to decimal) = " << Page_type << std::endl;
						std::string Data_jk_ephemeris = Data_k + Data_j;
						//std::cout<<"Data j k ephemeris" << endl << Data_jk_ephemeris << endl;

						page_jk_decoder(Data_jk_ephemeris.c_str()); // Corresponding to ephemeris_decode.m in matlab code

						/*if (have_new_ephemeris()==true){
							std::cout<<"All ephemeris have been received" << std::endl;
						}*/

						double t_GST;
						if ((have_new_iono_and_GST() == true) and (flag_all_ephemeris==true))
						{
							std::cout <<"GST and ephemeris parameters have been received, now it is possible to compute satellite position"<< std::endl;
							t_GST = Galileo_System_Time(WN_5, TOW_5);
							std::cout << "Galileo System Time [sec]: " << t_GST << std::endl;
							satellitePosition(t_GST);
							flag_all_ephemeris=false;
						}

						double t_UTC;
						if ((have_new_iono_and_GST() == true) and (have_new_utc_model() == true))
						{
							t_UTC = GST_to_UTC_time(t_GST, WN_5);
							std::cout << "UTC [sec]: " << t_UTC << std::endl;
						}

					}else{
						// CRC wrong.. discard frame
						std::cout<<"CRC error!"<<std::endl;
					}
					//********** end of CRC checksum control ***/
				}

		} /*end if (page_string.at(0)=='1') */

	else{
	page_Even = page_string.substr (0,114);
	//std::cout << "Page even in split page" << std::endl << page_Even << std::endl;
	std::string tail_Even =  page_string.substr (114,6);
	//std::cout << "tail_even_string: " << tail_Even <<std::endl;
	//if (tail_Even.compare(correct_tail) != 0)
	//	 std::cout << "Tail even is not correct!" << std::endl;
	//else std::cout<<"Tail even is correct!"<< std::endl;

	}

}


bool Galileo_Navigation_Message::have_new_ephemeris() //Check if we have a new ephemeris stored in the galileo navigation class
{
	/*std::cout << "flag ephememeris 1: " << flag_ephemeris_1 <<std::endl;
	std::cout << "flag ephememeris 2: " << flag_ephemeris_2 <<std::endl;
	std::cout << "flag ephememeris 3: " << flag_ephemeris_3 <<std::endl;
	std::cout << "flag ephememeris 4: " << flag_ephemeris_4 <<std::endl;*/
	if ((flag_ephemeris_1 == true) and (flag_ephemeris_2 == true) and (flag_ephemeris_3 == true) and (flag_ephemeris_4 == true))
	{
		std::cout<< "All ephemeris have been received"<< std::endl;
		flag_ephemeris_1 = false;// clear the flag
		flag_ephemeris_2 = false;// clear the flag
		flag_ephemeris_3 = false;// clear the flag
		flag_ephemeris_4 = false;// clear the flag
		flag_all_ephemeris = true;
		return true;
	 }
	else
		return false;
}


bool Galileo_Navigation_Message::have_new_iono_and_GST() //Check if we have a new iono data set stored in the galileo navigation class
{
	if (flag_iono_and_GST == true)
	{
		flag_iono_and_GST=false; // clear the flag
		return true;
	}else
		return false;
}


bool Galileo_Navigation_Message::have_new_utc_model() // Check if we have a new utc data set stored in the galileo navigation class
{
	if (flag_utc_model == true)
	{
		flag_utc_model=false; // clear the flag
		return true;
	}
	else
		return false;
}


bool Galileo_Navigation_Message::have_new_almanac() //Check if we have a new almanac data set stored in the galileo navigation class
{
	if ((flag_almanac_1 == true) and (flag_almanac_2 == true) and (flag_almanac_3 == true) and (flag_almanac_4 == true))
	{
		//std::cout<< "All almanac have been received"<< std::endl;
		flag_almanac_1 = false;
		flag_almanac_2 = false;
		flag_almanac_3 = false;
		flag_almanac_4 = false;
		flag_all_almanac = true;
		return true;
	 }
	else
		return false;
}

Galileo_Ephemeris Galileo_Navigation_Message::get_ephemeris()
{

	Galileo_Ephemeris ephemeris;
	ephemeris.M0_1 = M0_1;		// Mean anomaly at reference time [semi-circles]
	ephemeris.delta_n_3 = delta_n_3;		// Mean motion difference from computed value  [semi-circles/sec]
	ephemeris.e_1 =	e_1;	// Eccentricity
	ephemeris.A_1 =  A_1; 	// Square root of the semi-major axis [metres^1/2]
	ephemeris.OMEGA_0_2 = OMEGA_0_2;// Longitude of ascending node of orbital plane at weekly epoch [semi-circles]
	ephemeris.i_0_2 = i_0_2;    // Inclination angle at reference time  [semi-circles]
	ephemeris.omega_2 = omega_2;  // Argument of perigee [semi-circles]
	ephemeris.OMEGA_dot_3 =	OMEGA_dot_3;	// Rate of right ascension [semi-circles/sec]
	ephemeris.iDot_2 = iDot_2;   // Rate of inclination angle [semi-circles/sec]
	ephemeris.C_uc_3 = C_uc_3;		// Amplitude of the cosine harmonic correction term to the argument of latitude [radians]
	ephemeris.C_us_3 = C_us_3;			// Amplitude of the sine harmonic correction term to the argument of latitude [radians]
	ephemeris.C_rc_3 = C_rc_3;			// Amplitude of the cosine harmonic correction term to the orbit radius [meters]
	ephemeris.C_rs_3 = C_rs_3;			// Amplitude of the sine harmonic correction term to the orbit radius [meters]
	ephemeris.C_ic_4 = C_ic_4;		// Amplitude of the cosine harmonic correction 	term to the angle of inclination [radians]
	ephemeris.C_is_4 = C_is_4;		// Amplitude of the sine harmonic correction term to the angle of inclination [radians]
	ephemeris.t0e_1 = t0e_1;	// Ephemeris reference time [s]

	/*Clock correction parameters*/
	ephemeris.t0c_4 = t0c_4;			//Clock correction data reference Time of Week [sec]
	ephemeris.af0_4 = af0_4;			//SV clock bias correction coefficient [s]
	ephemeris.af1_4 = af1_4;		//SV clock drift correction coefficient [s/s]
	ephemeris.af2_4 = af2_4;		//SV clock drift rate correction coefficient [s/s^2]

	/*GST*/
	ephemeris.WN_5 = WN_5;//Week number
	ephemeris.TOW_5 = TOW_5;//Time of Week
	return ephemeris;
}


Galileo_Iono Galileo_Navigation_Message::get_iono()
{
	Galileo_Iono iono;
	 /*Ionospheric correction*/
	 /*Az*/
	 iono.ai0_5 = ai0_5;		//Effective Ionisation Level 1st order parameter [sfu]
	 iono.ai1_5 = ai1_5;		//Effective Ionisation Level 2st order parameter [sfu/degree]
	 iono.ai2_5 = ai2_5;		//Effective Ionisation Level 3st order parameter [sfu/degree]

	 /*Ionospheric disturbance flag*/
	 iono.Region1_flag_5 = Region1_flag_5;	// Ionospheric Disturbance Flag for region 1
	 iono.Region2_flag_5 = Region2_flag_5;	// Ionospheric Disturbance Flag for region 2
	 iono.Region3_flag_5 = Region3_flag_5;	// Ionospheric Disturbance Flag for region 3
	 iono.Region4_flag_5 = Region4_flag_5;	// Ionospheric Disturbance Flag for region 4
	 iono.Region5_flag_5 = Region5_flag_5;	// Ionospheric Disturbance Flag for region 5

	return iono;
}


Galileo_Utc_Model Galileo_Navigation_Message::get_utc_model()
{
	Galileo_Utc_Model utc_model;
	//Gal_utc_model.valid = flag_utc_model_valid;
	/*Word type 6: GST-UTC conversion parameters*/
	utc_model.A0_6 = A0_6;
	utc_model.A1_6 = A1_6;
	utc_model.Delta_tLS_6 = Delta_tLS_6;
	utc_model.t0t_6 = t0t_6;
	utc_model.WNot_6 = WNot_6;
	utc_model.WN_LSF_6 = WN_LSF_6;
	utc_model.DN_6 = DN_6;
	utc_model.Delta_tLSF_6 = Delta_tLSF_6;

	/*GST*/
	//utc_model.WN_5 = WN_5; //Week number
	//utc_model.TOW_5 = WN_5; //Time of Week
	return utc_model;
}


Galileo_Almanac Galileo_Navigation_Message::get_almanac()
{
	Galileo_Almanac almanac;
	/*Word type 7: Almanac for SVID1 (1/2), almanac reference time and almanac reference week number*/
	    almanac.IOD_a_7 = IOD_a_7;
		almanac.WN_a_7 = WN_a_7;
		almanac.t0a_7 = t0a_7;
		almanac.SVID1_7 = SVID1_7;
		almanac.DELTA_A_7 = DELTA_A_7;
		almanac.e_7 = e_7;
		almanac.omega_7 = omega_7;
		almanac.delta_i_7 = delta_i_7;
		almanac.Omega0_7 = Omega0_7;
		almanac.Omega_dot_7 = Omega_dot_7;
		almanac.M0_7 = M0_7;

		/*Word type 8: Almanac for SVID1 (2/2) and SVID2 (1/2)*/
		almanac.IOD_a_8 = IOD_a_8;
		almanac.af0_8 = af0_8;
		almanac.af1_8 = af1_8;
		almanac.E5b_HS_8 = E5b_HS_8;
		almanac.E1B_HS_8 = E1B_HS_8;
		almanac.SVID2_8 = SVID2_8;
		almanac.DELTA_A_8 = DELTA_A_8;
		almanac.e_8 = e_8;
		almanac.omega_8 = omega_8;
		almanac.delta_i_8 = delta_i_8;
		almanac.Omega0_8 = Omega0_8;
		almanac.Omega_dot_8 = Omega_dot_8;

		/*Word type 9: Almanac for SVID2 (2/2) and SVID3 (1/2)*/
		almanac.IOD_a_9 = IOD_a_9;
		almanac.WN_a_9 = WN_a_9;
		almanac.t0a_9 = t0a_9;
		almanac.M0_9 = M0_9;
		almanac.af0_9 = af0_9;
		almanac.af1_9 = af1_9;
		almanac.E5b_HS_9 = E5b_HS_9;
		almanac.E1B_HS_9 = E1B_HS_9;
		almanac.SVID3_9 = SVID3_9;
		almanac.DELTA_A_9 = DELTA_A_9;
		almanac.e_9 = e_9;
		almanac.omega_9 = omega_9;
		almanac.delta_i_9 = delta_i_9;


		/*Word type 10: Almanac for SVID3 (2/2)*/
		almanac.IOD_a_10 = IOD_a_10;
		almanac.Omega0_10 = Omega0_10;
		almanac.Omega_dot_10 = Omega_dot_10;
		almanac.M0_10 = M0_10;
		almanac.af0_10 = af0_10;
		almanac.af1_10 = af1_10;
		almanac.E5b_HS_10 = E5b_HS_10;
		almanac.E1B_HS_10 = E1B_HS_10;
	return almanac;
}


int Galileo_Navigation_Message::page_jk_decoder(const char *data_jk)
{
	std::cout << "--------------------------------------------------------------------------" << std::endl;
	std::cout<< "Entered in function Galileo_Navigation_Message::page_jk_decoder(const char *data_jk)" << std::endl;

    int page_number = 0;

    std::string data_jk_string = data_jk;
    std::bitset<GALILEO_DATA_JK_BITS> data_jk_bits (data_jk_string);

    //std::cout << "Data_jk_bits (bitset)  "<< endl << data_jk_bits << endl;

    page_number = (int)read_navigation_unsigned(data_jk_bits, PAGE_TYPE_bit);
    std::cout << "Page number = " << page_number << std::endl;

    switch (page_number)
        {
    case 1: /*Word type 1: Ephemeris (1/4)*/
        		IOD_nav_1=(int)read_navigation_unsigned(data_jk_bits, IOD_nav_1_bit);
        		std::cout<<"IOD_nav_1= "<< IOD_nav_1 <<std::endl;

        		t0e_1=(double)read_navigation_unsigned(data_jk_bits, T0E_1_bit);
        		t0e_1 = t0e_1 * t0e_1_LSB;
        		std::cout << "t0e_1= " << t0e_1 <<std::endl;

        		M0_1 = (double)read_navigation_unsigned(data_jk_bits, M0_1_bit);
        		M0_1 =  M0_1 * M0_1_LSB;
        		std::cout << "M0_1= " << M0_1<<std::endl;

        		e_1 = (double)read_navigation_unsigned(data_jk_bits, e_1_bit);
        		e_1 = e_1 * e_1_LSB;
        		std::cout << "e_1= " << e_1 <<std::endl;

        		A_1 = (double)read_navigation_unsigned(data_jk_bits, A_1_bit);
        		A_1 = A_1 * A_1_LSB_gal;
        		std::cout << "A_1= " << A_1 <<std::endl;
        		flag_ephemeris_1 = true;
        		break;

        	case 2:  /*Word type 2: Ephemeris (2/4)*/
        		IOD_nav_2 = (int)read_navigation_unsigned(data_jk_bits, IOD_nav_2_bit);
        		std::cout<<"IOD_nav_2= "<< IOD_nav_2 <<std::endl;

        		OMEGA_0_2 = (double)read_navigation_unsigned(data_jk_bits, OMEGA_0_2_bit);
        		OMEGA_0_2 = OMEGA_0_2 * OMEGA_0_2_LSB;
        		std::cout<<"OMEGA_0_2= "<< OMEGA_0_2 <<std::endl;
        		i_0_2 = (double)read_navigation_unsigned(data_jk_bits, i_0_2_bit);
        		i_0_2 = i_0_2 * i_0_2_LSB;
        		std::cout<<"i_0_2= "<< i_0_2 <<std::endl;
        		omega_2 = (double)read_navigation_unsigned(data_jk_bits, omega_2_bit);
        		omega_2 = omega_2 * omega_2_LSB;
        		std::cout<<"omega_2= "<< omega_2 <<std::endl;
        		iDot_2 = (double)read_navigation_unsigned(data_jk_bits, iDot_2_bit);
        		iDot_2 = iDot_2 * iDot_2_LSB;
        		std::cout<<"iDot_2= "<< iDot_2 <<std::endl;
        		flag_ephemeris_2 = true;
        		break;

        	case 3:  /*Word type 3: Ephemeris (3/4) and SISA*/
        		IOD_nav_3 = (int)read_navigation_unsigned(data_jk_bits, IOD_nav_3_bit);
        		std::cout<<"IOD_nav_3= "<< IOD_nav_3 <<std::endl;
        		OMEGA_dot_3 = (double)read_navigation_unsigned(data_jk_bits, OMEGA_dot_3_bit);
        		OMEGA_dot_3 = OMEGA_dot_3 * OMEGA_dot_3_LSB;
        		std::cout<<"OMEGA_dot_3= "<< OMEGA_dot_3 <<std::endl;
        		delta_n_3 = (double)read_navigation_unsigned(data_jk_bits, delta_n_3_bit);
        		delta_n_3 = delta_n_3 * delta_n_3_LSB;
        		std::cout<<"delta_n_3= "<< delta_n_3 <<std::endl;
        		C_uc_3 = (double)read_navigation_unsigned(data_jk_bits, C_uc_3_bit);
        		C_uc_3 = C_uc_3 * C_uc_3_LSB;
        		std::cout<<"C_uc_3= "<< C_uc_3 <<std::endl;
        		C_us_3 = (double)read_navigation_unsigned(data_jk_bits, C_us_3_bit);
        		C_us_3 = C_us_3 * C_us_3_LSB;
        		std::cout<<"C_us_3= "<< C_us_3 <<std::endl;
        		C_rc_3 = (double)read_navigation_unsigned(data_jk_bits, C_rc_3_bit);
        		C_rc_3 = C_rc_3 * C_rc_3_LSB;
        		std::cout<<"C_rc_3= "<< C_rc_3 <<std::endl;
        		C_rs_3 = (double)read_navigation_unsigned(data_jk_bits, C_rs_3_bit);
        		C_rs_3 = C_rs_3 * C_rs_3_LSB;
        		std::cout<<"C_rs_3= "<< C_rs_3 <<std::endl;
        		SISA_3 = (double)read_navigation_unsigned(data_jk_bits, SISA_3_bit);
        		std::cout<<"SISA_3= "<< SISA_3 <<std::endl;
        		flag_ephemeris_3 = true;
        		break;

        	case 4: /* Word type 4: Ephemeris (4/4) and Clock correction parameters*/
        		IOD_nav_4 = (int)read_navigation_unsigned(data_jk_bits, IOD_nav_4_bit);
        		std::cout<<"IOD_nav_4= "<< IOD_nav_4 <<std::endl;
        		SV_ID_PRN_4 = (int)read_navigation_unsigned(data_jk_bits, SV_ID_PRN_4_bit);
        		std::cout<<"SV_ID_PRN_4= "<< SV_ID_PRN_4 <<std::endl;
        		C_ic_4 = (double)read_navigation_unsigned(data_jk_bits, C_ic_4_bit);
        		C_ic_4 = C_ic_4 * C_ic_4_LSB;
        		std::cout<<"C_ic_4= "<< C_ic_4 <<std::endl;
        		C_is_4 = (double)read_navigation_unsigned(data_jk_bits, C_is_4_bit);
        		C_is_4 = C_is_4 * C_is_4_LSB;
        		std::cout<<"C_is_4= "<< C_is_4 <<std::endl;
        		/*Clock correction parameters*/
        		t0c_4 = (double)read_navigation_unsigned(data_jk_bits, t0c_4_bit);
        		t0c_4 = t0c_4 * t0c_4_LSB;
        		std::cout<<"t0c_4= "<< t0c_4 <<std::endl;
        		af0_4 = (double)read_navigation_unsigned(data_jk_bits, af0_4_bit);
        		af0_4 = af0_4 * af0_4_LSB;
        		std::cout<<"af0_4 = "<< af0_4  <<std::endl;
        		af1_4 = (double)read_navigation_unsigned(data_jk_bits, af1_4_bit);
        		af1_4 = af1_4 * af1_4_LSB;
        		std::cout<<"af1_4 = "<< af1_4  <<std::endl;
        		af2_4 = (double)read_navigation_unsigned(data_jk_bits, af2_4_bit);
        		af2_4 = af2_4 * af2_4_LSB;
        		std::cout<<"af2_4 = "<< af2_4  <<std::endl;
        		spare_4 = (double)read_navigation_unsigned(data_jk_bits, spare_4_bit);
        		std::cout<<"spare_4 = "<< spare_4  <<std::endl;
        		flag_ephemeris_4 = true;
        		break;

        	case 5: /*Word type 5: Ionospheric correction, BGD, signal health and data validity status and GST*/
        		 /*Ionospheric correction*/
        		 /*Az*/
        		ai0_5 = (double)read_navigation_unsigned(data_jk_bits, ai0_5_bit);
        		ai0_5 = ai0_5 * ai0_5_LSB;
        		std::cout<<"ai0_5= "<< ai0_5 <<std::endl;
        		ai1_5 = (double)read_navigation_unsigned(data_jk_bits, ai1_5_bit);
        		ai1_5 = ai1_5 * ai1_5_LSB;
        		std::cout<<"ai1_5= "<< ai1_5 <<std::endl;
        		ai2_5 = (double)read_navigation_unsigned(data_jk_bits, ai2_5_bit);
        		ai2_5 = ai2_5 * ai2_5_LSB;
        		std::cout<<"ai2_5= "<< ai2_5 <<std::endl;
        		/*Ionospheric disturbance flag*/
        		Region1_flag_5 = (bool)read_navigation_bool(data_jk_bits, Region1_5_bit);
        		std::cout<<"Region1_flag_5= "<< Region1_flag_5 <<std::endl;
        		Region2_flag_5 = (bool)read_navigation_bool(data_jk_bits, Region2_5_bit);
        		std::cout<<"Region2_flag_5= "<< Region2_flag_5 <<std::endl;
        		Region3_flag_5 = (bool)read_navigation_bool(data_jk_bits, Region3_5_bit);
        		std::cout<<"Region3_flag_5= "<< Region3_flag_5 <<std::endl;
        		Region4_flag_5 = (bool)read_navigation_bool(data_jk_bits, Region4_5_bit);
        		std::cout<<"Region4_flag_5= "<< Region4_flag_5 <<std::endl;
        		Region5_flag_5 = (bool)read_navigation_bool(data_jk_bits, Region5_5_bit);
        		std::cout<<"Region5_flag_5= "<< Region5_flag_5 <<std::endl;
        		BGD_E1E5a_5 = (double)read_navigation_unsigned(data_jk_bits, BGD_E1E5a_5_bit);
        		BGD_E1E5a_5 = BGD_E1E5a_5 * BGD_E1E5a_5_LSB;
        		std::cout<<"BGD_E1E5a_5= "<< BGD_E1E5a_5 <<std::endl;
        		BGD_E1E5b_5 = (double)read_navigation_unsigned(data_jk_bits, BGD_E1E5b_5_bit);
        		BGD_E1E5b_5 = BGD_E1E5b_5 * BGD_E1E5b_5_LSB;
        		std::cout<<"BGD_E1E5b_5= "<< BGD_E1E5b_5 <<std::endl;
        		E5b_HS_5 = (double)read_navigation_unsigned(data_jk_bits, E5b_HS_5_bit);
        		std::cout<<"E5b_HS_5= "<< E5b_HS_5 <<std::endl;
        		E1B_HS_5 = (double)read_navigation_unsigned(data_jk_bits, E1B_HS_5_bit);
        		std::cout<<"E1B_HS_5= "<< E1B_HS_5 <<std::endl;
        		E5b_DVS_5 = (double)read_navigation_unsigned(data_jk_bits, E5b_DVS_5_bit);
        		std::cout<<"E5b_DVS_5= "<< E5b_DVS_5 <<std::endl;
        		E1B_DVS_5 = (double)read_navigation_unsigned(data_jk_bits, E1B_DVS_5_bit);
        		std::cout<<"E1B_DVS_5= "<< E1B_DVS_5 <<std::endl;
        		/*GST*/
        		WN_5 = (double)read_navigation_unsigned(data_jk_bits, WN_5_bit);
        		std::cout<<"WN_5= "<< WN_5 <<std::endl;
        		TOW_5 = (double)read_navigation_unsigned(data_jk_bits, TOW_5_bit);
        		std::cout<<"TOW_5= "<< TOW_5 <<std::endl;
        		spare_5 = (double)read_navigation_unsigned(data_jk_bits, spare_5_bit);
        		std::cout<<"spare_5= "<< spare_5 <<std::endl;
        		flag_iono_and_GST = true;
        		break;

        	case 6: /*Word type 6: GST-UTC conversion parameters*/
        	    A0_6= (double)read_navigation_unsigned(data_jk_bits, A0_6_bit);
        	    A0_6= A0_6 * A0_6_LSB;
        	    std::cout << "A0_6= " << A0_6 << std::endl;

        	    A1_6= (double)read_navigation_unsigned(data_jk_bits, A1_6_bit);
        	    A1_6= A1_6 * A1_6_LSB;
        	    std::cout << "A1_6= " << A1_6 << std::endl;

        	    Delta_tLS_6= (double)read_navigation_unsigned(data_jk_bits, Delta_tLS_6_bit);
        	    std::cout << "Delta_tLS_6= " << Delta_tLS_6 << std::endl;

        	    t0t_6= (double)read_navigation_unsigned(data_jk_bits, t0t_6_bit);
        	    t0t_6= t0t_6 * t0t_6_LSB;
        	    std::cout << "t0t_6= " << t0t_6 << std::endl;

        	    WNot_6= (double)read_navigation_unsigned(data_jk_bits, WNot_6_bit);
        	    std::cout << "WNot_6= " << WNot_6 << std::endl;

        	    WN_LSF_6= (double)read_navigation_unsigned(data_jk_bits, WN_LSF_6_bit);
        	    std::cout << "WN_LSF_6= " << WN_LSF_6 << std::endl;

        	    DN_6= (double)read_navigation_unsigned(data_jk_bits, DN_6_bit);
        	    std::cout << "DN_6= " << DN_6 << std::endl;

        	    Delta_tLSF_6= (double)read_navigation_unsigned(data_jk_bits, Delta_tLSF_6_bit);
        	    std::cout << "Delta_tLSF_6= " << Delta_tLSF_6 << std::endl;

        	    TOW_6= (double)read_navigation_unsigned(data_jk_bits, TOW_6_bit);
        	    std::cout << "TOW_6= " << TOW_6 << std::endl;
        	    flag_utc_model = true;
        	    break;

         case 7: /*Word type 7: Almanac for SVID1 (1/2), almanac reference time and almanac reference week number*/

        	    IOD_a_7= (double)read_navigation_unsigned(data_jk_bits, IOD_a_7_bit);
        	    std::cout << "IOD_a_7= " << IOD_a_7 << std::endl;

        	    WN_a_7= (double)read_navigation_unsigned(data_jk_bits, WN_a_7_bit);
        	    std::cout << "WN_a_7= " << WN_a_7 << std::endl;

        	    t0a_7= (double)read_navigation_unsigned(data_jk_bits, t0a_7_bit);
        	    t0a_7= t0a_7 * t0a_7_LSB;
        	    std::cout << "t0a_7= " << t0a_7 << std::endl;

        	    SVID1_7= (double)read_navigation_unsigned(data_jk_bits, SVID1_7_bit);
        	    std::cout << "SVID1_7= " << SVID1_7 << std::endl;

        	    DELTA_A_7= (double)read_navigation_unsigned(data_jk_bits, DELTA_A_7_bit);
        	    DELTA_A_7= DELTA_A_7 * DELTA_A_7_LSB;
        	    std::cout << "DELTA_A_7= " << DELTA_A_7 << std::endl;

        	    e_7= (double)read_navigation_unsigned(data_jk_bits, e_7_bit);
        	    e_7= e_7 * e_7_LSB;
        	    std::cout << "e_7= " << e_7 << std::endl;

        	    omega_7= (double)read_navigation_unsigned(data_jk_bits, omega_7_bit);
        	    omega_7= omega_7 * omega_7_LSB;
        	    std::cout << "omega_7= " << omega_7 << std::endl;

        	    delta_i_7= (double)read_navigation_unsigned(data_jk_bits, delta_i_7_bit);
        	    delta_i_7= delta_i_7 * delta_i_7_LSB;
        	    std::cout << "delta_i_7= " << delta_i_7 << std::endl;

        	    Omega0_7= (double)read_navigation_unsigned(data_jk_bits, Omega0_7_bit);
        	    Omega0_7= Omega0_7 * Omega0_7_LSB;
        	    std::cout << "Omega0_7= " << Omega0_7 << std::endl;

        	    Omega_dot_7= (double)read_navigation_unsigned(data_jk_bits, Omega_dot_7_bit);
        	    Omega_dot_7= Omega_dot_7 * Omega_dot_7_LSB;
        	    std::cout << "Omega_dot_7= " << Omega_dot_7 << std::endl;

        	    M0_7= (double)read_navigation_unsigned(data_jk_bits, M0_7_bit);
        	    M0_7= M0_7 * M0_7_LSB;
        	    std::cout << "M0_7= " << M0_7 << std::endl;
        	    flag_almanac_1 = true;
        	    break;

        case 8: /*Word type 8: Almanac for SVID1 (2/2) and SVID2 (1/2)*/

        		IOD_a_8= (double)read_navigation_unsigned(data_jk_bits, IOD_a_8_bit);
        		std::cout << "IOD_a_8= " << IOD_a_8 << std::endl;

        		af0_8= (double)read_navigation_unsigned(data_jk_bits, af0_8_bit);
        		af0_8= af0_8 * af0_8_LSB;
        		std::cout << "af0_8= " << af0_8 << std::endl;

        	    af1_8= (double)read_navigation_unsigned(data_jk_bits, af1_8_bit);
        	    af1_8= af1_8 * af1_8_LSB;
        	    std::cout << "af1_8= " << af1_8 << std::endl;

        	    E5b_HS_8= (double)read_navigation_unsigned(data_jk_bits, E5b_HS_8_bit);
        	    std::cout << "E5b_HS_8= " << E5b_HS_8 << std::endl;

        	    E1B_HS_8= (double)read_navigation_unsigned(data_jk_bits, E1B_HS_8_bit);
        	    std::cout << "E1B_HS_8= " << E1B_HS_8 << std::endl;

        	    SVID2_8= (double)read_navigation_unsigned(data_jk_bits, SVID2_8_bit);
        	    std::cout << "SVID2_8= " << SVID2_8 << std::endl;

        	    DELTA_A_8= (double)read_navigation_unsigned(data_jk_bits, DELTA_A_8_bit);
        	    DELTA_A_8= DELTA_A_8 * DELTA_A_8_LSB;
        	    std::cout << "DELTA_A_8= " << DELTA_A_8 << std::endl;

        	    e_8= (double)read_navigation_unsigned(data_jk_bits, e_8_bit);
        	    e_8= e_8 * e_8_LSB;
        	    std::cout << "e_8= " << e_8 << std::endl;

        	    omega_8= (double)read_navigation_unsigned(data_jk_bits, omega_8_bit);
        	    omega_8= omega_8 * omega_8_LSB;
        	    std::cout << "omega_8= " << omega_8 << std::endl;

        	    delta_i_8= (double)read_navigation_unsigned(data_jk_bits, delta_i_8_bit);
        	    delta_i_8= delta_i_8 * delta_i_8_LSB;
        	    std::cout << "delta_i_8= " << delta_i_8 << std::endl;

        	    Omega0_8= (double)read_navigation_unsigned(data_jk_bits, Omega0_8_bit);
        	    Omega0_8= Omega0_8 * Omega0_8_LSB;
        	    std::cout << "Omega0_8= " << Omega0_8 << std::endl;

        	    Omega_dot_8= (double)read_navigation_unsigned(data_jk_bits, Omega_dot_8_bit);
        	    Omega_dot_8= Omega_dot_8 * Omega_dot_8_LSB;
        	    std::cout << "Omega_dot_8= " << Omega_dot_8 << std::endl;
        	    flag_almanac_2 = true;
        	    break;

        	case 9: /*Word type 9: Almanac for SVID2 (2/2) and SVID3 (1/2)*/

        	    IOD_a_9= (double)read_navigation_unsigned(data_jk_bits, IOD_a_9_bit);
        	    std::cout << "IOD_a_9= " << IOD_a_9 << std::endl;

        	    WN_a_9= (double)read_navigation_unsigned(data_jk_bits, WN_a_9_bit);
        	    std::cout << "WN_a_9= " << WN_a_9 << std::endl;

        	    t0a_9= (double)read_navigation_unsigned(data_jk_bits, t0a_9_bit);
        	    t0a_9= t0a_9 * t0a_9_LSB;
        	    std::cout << "t0a_9= " << t0a_9 << std::endl;

        	    M0_9= (double)read_navigation_unsigned(data_jk_bits, M0_9_bit);
        	    M0_9= M0_9 * M0_9_LSB;
        	    std::cout << "M0_9= " << M0_9 << std::endl;

        	    af0_9= (double)read_navigation_unsigned(data_jk_bits, af0_9_bit);
        	    af0_9= af0_9 * af0_9_LSB;
        	    std::cout << "af0_9= " << af0_9 << std::endl;

        	    af1_9= (double)read_navigation_unsigned(data_jk_bits, af1_9_bit);
        	    af1_9= af1_9 * af1_9_LSB;
        	    std::cout << "af1_9= " << af1_9 << std::endl;

        	    E1B_HS_9= (double)read_navigation_unsigned(data_jk_bits, E1B_HS_9_bit);
        	    std::cout << "E1B_HS_9= " << E1B_HS_9 << std::endl;

        	    E1B_HS_9= (double)read_navigation_unsigned(data_jk_bits, E1B_HS_9_bit);
        	    std::cout << "E1B_HS_9= " << E1B_HS_9 << std::endl;

        	    SVID3_9= (double)read_navigation_unsigned(data_jk_bits,SVID3_9_bit);
        	    std::cout << "SVID3_9= " << SVID3_9 << std::endl;


        	    DELTA_A_9= (double)read_navigation_unsigned(data_jk_bits, DELTA_A_9_bit);
        	    DELTA_A_9= DELTA_A_9 * DELTA_A_9_LSB;
        	    std::cout << "DELTA_A_9= " << DELTA_A_9 << std::endl;

        	    e_9= (double)read_navigation_unsigned(data_jk_bits, e_9_bit);
        	    e_9= e_9 * e_9_LSB;
        	    std::cout << "e_9= " << e_9 << std::endl;

        	    omega_9= (double)read_navigation_unsigned(data_jk_bits, omega_9_bit);
        	    omega_9= omega_9 * omega_9_LSB;
        	    std::cout << "omega_9= " << omega_9 << std::endl;

        	    delta_i_9= (double)read_navigation_unsigned(data_jk_bits, delta_i_9_bit);
        	    delta_i_9= delta_i_9 * delta_i_9_LSB;
        	    std::cout << "delta_i_9= " << delta_i_9 << std::endl;
        	    flag_almanac_3 = true;
        	    break;

        case 10: /*Word type 10: Almanac for SVID3 (2/2) and GST-GPS conversion parameters*/

        	    IOD_a_10= (double)read_navigation_unsigned(data_jk_bits, IOD_a_10_bit);
        	    std::cout << "IOD_a_10= " << IOD_a_10 << std::endl;

        	    Omega0_10= (double)read_navigation_unsigned(data_jk_bits, Omega0_10_bit);
        	    Omega0_10= Omega0_10 * Omega0_10_LSB;
        	    std::cout << "Omega0_10= " << Omega0_10 << std::endl;

        	    Omega_dot_10= (double)read_navigation_unsigned(data_jk_bits, Omega_dot_10_bit);
        	    Omega_dot_10= Omega_dot_10 * Omega_dot_10_LSB;
        	    std::cout << "Omega_dot_10= " << Omega_dot_10 << std::endl;

        	    M0_10= (double)read_navigation_unsigned(data_jk_bits, M0_10_bit);
        	    M0_10= M0_10 * M0_10_LSB;
        	    std::cout << "M0_10= " << M0_10 << std::endl;

        	    af0_10= (double)read_navigation_unsigned(data_jk_bits, af0_10_bit);
        	    af0_10= af0_10 * af0_10_LSB;
        	    std::cout << "af0_10= " << af0_10 << std::endl;

        	    af1_10= (double)read_navigation_unsigned(data_jk_bits, af1_10_bit);
        	    af1_10= af1_10 * af1_10_LSB;
        	    std::cout << "af1_10= " << af1_10 << std::endl;

        	    E5b_HS_10= (double)read_navigation_unsigned(data_jk_bits, E5b_HS_10_bit);
        	    std::cout << "E5b_HS_10= " << E5b_HS_10 << std::endl;

        	    E1B_HS_10= (double)read_navigation_unsigned(data_jk_bits, E1B_HS_10_bit);
        	    std::cout << "E1B_HS_10= " << E1B_HS_10 << std::endl;

        	    A_0G_10= (double)read_navigation_unsigned(data_jk_bits, A_0G_10_bit);
        	    A_0G_10= A_0G_10 * A_0G_10_LSB;
        	    std::cout << "A_0G_10= " << A_0G_10 << std::endl;

        	    A_1G_10= (double)read_navigation_unsigned(data_jk_bits, A_1G_10_bit);
        	    A_1G_10= A_1G_10 * A_1G_10_LSB;
        	    std::cout << "A_1G_10= " << A_1G_10 << std::endl;

        	    t_0G_10= (double)read_navigation_unsigned(data_jk_bits, A_1G_10_bit);
        	    t_0G_10= t_0G_10 * t_0G_10_LSB;
        	    std::cout << "t_0G_10= " << t_0G_10 << std::endl;

        	    WN_0G_10= (double)read_navigation_unsigned(data_jk_bits, WN_0G_10_bit);
        	    std::cout << "WN_0G_10= " << WN_0G_10 << std::endl;
        	    flag_almanac_4 = true;
        	    break;

       case 0: /*Word type 0: I/NAV Spare Word*/
        	    Time_0= (double)read_navigation_unsigned(data_jk_bits, Time_0_bit);
        	    std::cout << "Time_0= " << Time_0 << std::endl;

         	    WN_0= (double)read_navigation_unsigned(data_jk_bits, WN_0_bit);
         	    std::cout << "WN_0= " << WN_0 << std::endl;

         	    TOW_0= (double)read_navigation_unsigned(data_jk_bits, TOW_0_bit);
          	    std::cout << "TOW_0= " << TOW_0 << std::endl;

          	 break;
        }

   return page_number;
}



void Galileo_Navigation_Message::satellitePosition(double transmitTime) //when this function in used, the input must be the transmitted time (t) in second computed by Galileo_System_Time (above function)
{

    double tk;  // Time from ephemeris reference epoch
    //double t;   // Galileo System Time (ICD, paragraph 5.1.2)
    double a;   // Semi-major axis
    double n;   // Corrected mean motion
    double n0;  // Computed mean motion
    double M;   // Mean anomaly
    double E;   //Eccentric Anomaly (to be solved by iteration)
    double E_old;
    double dE;
    double nu; //True anomaly
    double phi; //argument of Latitude
    double u;   // Correct argument of latitude
    double r;  // Correct radius
    double i;
    double Omega;

    // Find Galileo satellite's position ----------------------------------------------

    // Restore semi-major axis
    a = A_1*A_1;

    // Computed mean motion
    n0 = sqrt(GALILEO_GM / (a*a*a));

    // Time from ephemeris reference epoch
    //tk = check_t(transmitTime - d_Toe); this is tk for GPS; for Galileo it is different
    //t = WN_5*86400*7 + TOW_5; //WN_5*86400*7 are the second from the origin of the Galileo time
    tk = transmitTime - t0e_1;

    // Corrected mean motion
    n = n0 + delta_n_3;

    // Mean anomaly
    M = M0_1 + n * tk;

    // Reduce mean anomaly to between 0 and 2pi
    M = fmod((M + 2* GALILEO_PI), (2* GALILEO_PI));

    // Initial guess of eccentric anomaly
    E = M;

    // --- Iteratively compute eccentric anomaly ----------------------------
    for (int ii = 1; ii<20; ii++)
        {
            E_old   = E;
            E       = M + e_1 * sin(E);
            dE      = fmod(E - E_old, 2*GALILEO_PI);
            if (fabs(dE) < 1e-12)
                {
                    //Necessary precision is reached, exit from the loop
                    break;
                }
        }

    // Compute the true anomaly

    double tmp_Y = sqrt(1.0 - e_1 * e_1) * sin(E);
    double tmp_X = cos(E) - e_1;
    nu = atan2(tmp_Y, tmp_X);

    // Compute angle phi (argument of Latitude)
    phi = nu + omega_2;

    // Reduce phi to between 0 and 2*pi rad
    phi = fmod((phi), (2*GALILEO_PI));

    // Correct argument of latitude
    u = phi + C_uc_3 * cos(2*phi) +  C_us_3 * sin(2*phi);

    // Correct radius
    r = a * (1 - e_1*cos(E)) +  C_rc_3 * cos(2*phi) +  C_rs_3 * sin(2*phi);

    // Correct inclination
    i = i_0_2 + iDot_2 * tk + C_ic_4 * cos(2*phi) + C_is_4 * sin(2*phi);

    // Compute the angle between the ascending node and the Greenwich meridian
    Omega = OMEGA_0_2 + (OMEGA_dot_3 - GALILEO_OMEGA_EARTH_DOT)*tk - GALILEO_OMEGA_EARTH_DOT * t0e_1;

    // Reduce to between 0 and 2*pi rad
    Omega = fmod((Omega + 2*GALILEO_PI), (2*GALILEO_PI));

    // --- Compute satellite coordinates in Earth-fixed coordinates
    galileo_satpos_X = cos(u) * r * cos(Omega) - sin(u) * r * cos(i) * sin(Omega);
    galileo_satpos_Y = cos(u) * r * sin(Omega) + sin(u) * r * cos(i) * cos(Omega); //***********************NOTE: in GALILEO ICD this expression is not correct because it has minus (- sin(u) * r * cos(i) * cos(Omega)) instead of plus
    galileo_satpos_Z = sin(u) * r * sin(i);

    std::cout << "Galileo satellite position X [m]: " << galileo_satpos_X << std::endl;
    std::cout << "Galileo satellite position Y [m]: " << galileo_satpos_Y << std::endl;
    std::cout << "Galileo satellite position Z [m]: " << galileo_satpos_Z << std::endl;
    double vector_position = sqrt(galileo_satpos_X*galileo_satpos_X + galileo_satpos_Y*galileo_satpos_Y + galileo_satpos_Z*galileo_satpos_Z);
    std::cout << "Vector Earth Center-Satellite [Km]: " << vector_position/1000 << std::endl;

    // Satellite's velocity. Can be useful for Vector Tracking loops
    double Omega_dot = OMEGA_dot_3 - GALILEO_OMEGA_EARTH_DOT;
    galileo_satvel_X = - Omega_dot * (cos(u) * r + sin(u) * r * cos(i)) + galileo_satpos_X * cos(Omega) - galileo_satpos_Y * cos(i) * sin(Omega);
    galileo_satvel_Y = Omega_dot * (cos(u) * r * cos(Omega) - sin(u) * r * cos(i) * sin(Omega)) + galileo_satpos_X * sin(Omega) + galileo_satpos_Y * cos(i) * cos(Omega);
    galileo_satvel_Z = galileo_satpos_Y * sin(i);

}


double Galileo_Navigation_Message::Galileo_System_Time(double WN, double TOW){
	/* GALIELO SYSTEM TIME, ICD 5.1.2
	 * input parameter:
	 * WN: The Week Number is an integer counter that gives the sequential week number
	   from the origin of the Galileo time. It covers 4096 weeks (about 78 years).
	   Then the counter is reset to zero to cover additional period modulo 4096

	   TOW: The Time of Week is defined as the number of seconds that have occurred since
	   the transition from the previous week. The TOW covers an entire week from 0 to
	   604799 seconds and is reset to zero at the end of each week

	   WN and TOW are received in page 5

	   output:
	   t: it is the transmitted time in Galileo System Time (expressed in seconds)

	   The GST start epoch shall be 00:00 UT on Sunday 22nd August 1999 (midnight between 21st and 22nd August).
	   At the start epoch, GST shall be ahead of UTC by thirteen (13)
	   leap seconds. Since the next leap second was inserted at 01.01.2006, this implies that
       as of 01.01.2006 GST is ahead of UTC by fourteen (14) leap seconds.

       The epoch denoted in the navigation messages by TOW and WN
	   will be measured relative to the leading edge of the first chip of the
	   first code sequence of the first page symbol. The transmission timing of the navigation
	   message provided through the TOW is synchronised to each satellite’s version of Galileo System Time (GST).
	 *
	 */
	double t=0;
	double sec_in_day = 86400;
	double day_in_week = 7;
	t = WN * sec_in_day * day_in_week + TOW; // second from the origin of the Galileo time

	return t;

}



double Galileo_Navigation_Message::sv_clock_drift(double transmitTime){
	/* Satellite Time Correction Algorithm, ICD 5.1.4
	 *
	 */
    double dt;
    dt = transmitTime - t0c_4;
    Galileo_satClkDrift = af0_4 + af1_4*dt + (af2_4 * dt)*(af2_4 * dt) + Galileo_dtr;
    return Galileo_satClkDrift;
}

// compute the relativistic correction term
double Galileo_Navigation_Message::sv_clock_relativistic_term(double transmitTime) //Satellite Time Correction Algorithm, ICD 5.1.4
{
    double tk;
    double a;
    double n;
    double n0;
    double E;
    double E_old;
    double dE;
    double M;

      // Restore semi-major axis
      a = A_1*A_1;

      n0 = sqrt(GALILEO_GM / (a*a*a));

      // Time from ephemeris reference epoch
      //tk = check_t(transmitTime - d_Toe); this is tk for GPS; for Galileo it is different
      //t = WN_5*86400*7 + TOW_5; //WN_5*86400*7 are the second from the origin of the Galileo time
      tk = transmitTime - t0e_1;

      // Corrected mean motion
      n = n0 + delta_n_3;

      // Mean anomaly
      M = M0_1 + n * tk;

      // Reduce mean anomaly to between 0 and 2pi
      M = fmod((M + 2* GALILEO_PI), (2* GALILEO_PI));

      // Initial guess of eccentric anomaly
      E = M;

      // --- Iteratively compute eccentric anomaly ----------------------------
      for (int ii = 1; ii<20; ii++)
          {
              E_old   = E;
              E       = M + e_1 * sin(E);
              dE      = fmod(E - E_old, 2*GALILEO_PI);
              if (fabs(dE) < 1e-12)
                  {
                      //Necessary precision is reached, exit from the loop
                       break;
                   }
           }


    // Compute relativistic correction term
    Galileo_dtr = GALILEO_F * e_1* A_1 * sin(E);
    return Galileo_dtr;
}

double Galileo_Navigation_Message::GST_to_UTC_time(double t_e, int WN) //t_e is GST (WN+TOW) in second
{
	double t_Utc;
		double t_Utc_daytime;
		double Delta_t_Utc =  Delta_tLS_6 + A0_6 + A1_6 * (t_e - t0t_6 + 604800 * (double)(WN - WNot_6));

		// Determine if the effectivity time of the leap second event is in the past
		int  weeksToLeapSecondEvent = WN_LSF_6 - WN;

		if ((weeksToLeapSecondEvent) >= 0) // is not in the past
		{
			//Detect if the effectivity time and user's time is within six hours  = 6 * 60 *60 = 21600 s
			int secondOfLeapSecondEvent = DN_6 * 24 * 60 * 60;
			if (weeksToLeapSecondEvent > 0)
			{
				t_Utc_daytime = fmod(t_e - Delta_t_Utc, 86400);
			}
			else //we are in the same week than the leap second event
			{
				if  (abs(t_e - secondOfLeapSecondEvent) > 21600)
				{
					/* 5.1.7a
					 * Whenever the leap second adjusted time indicated by the WN_LSF and the DN values
					 * is not in the past (relative to the user's present time), and the user's
					 * present time does not fall in the time span which starts at six hours prior
					 * to the effective time and ends at six hours after the effective time,
					 * the GST/Utc relationship is given by
					 */
					t_Utc_daytime = fmod(t_e - Delta_t_Utc, 86400);
				}
				else
				{
					/* 5.1.7b
					 * Whenever the user's current time falls within the time span of six hours
					 * prior to the leap second adjustment to six hours after the adjustment time, ,
					 * the effective time is computed according to the following equations:
					 */

					int W = fmod(t_e - Delta_t_Utc - 43200, 86400) + 43200;
					t_Utc_daytime = fmod(W, 86400 + Delta_tLSF_6 - Delta_tLS_6);
					//implement something to handle a leap second event!
				}
				if ( (t_e - secondOfLeapSecondEvent) > 21600)
				{
					Delta_t_Utc = Delta_tLSF_6 + A0_6 + A1_6 * (t_e - t0t_6 + 604800*(double)(WN - WNot_6));
					t_Utc_daytime = fmod(t_e - Delta_t_Utc, 86400);
				}
			}
		}
		else // the effectivity time is in the past
		{
			/* 5.1.7c
			 * Whenever the leap second adjustment time, as indicated by the WN_LSF and DN values,
			 * is in the past (relative to the user’s current time) and the user’s present time does not
			 * fall in the time span which starts six hours prior to the leap second adjustment time and
			 * ends six hours after the adjustment time, the effective time is computed according to
			 * the following equation:
			 */
			Delta_t_Utc = Delta_tLSF_6 + A0_6 + A1_6 * (t_e - t0t_6 + 604800 * (double)(WN - WNot_6));
			t_Utc_daytime = fmod(t_e - Delta_t_Utc, 86400);
		}

		double secondsOfWeekBeforeToday = 43200 * floor(t_e / 43200);
		t_Utc = secondsOfWeekBeforeToday + t_Utc_daytime;
		return t_Utc;

}



