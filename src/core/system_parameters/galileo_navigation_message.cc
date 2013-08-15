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

using namespace std;

typedef boost::crc_optimal<24, 0x1864CFBu, 0x0, 0x0, false, false> CRC_Galileo_INAV_type;


void Galileo_Navigation_Message::reset()
{
	flag_even_word = 0;
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
	   		Delta_alpha_7 = 0;
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

	// ToDo: Replace all the C structures and string operations with std::string and std::stringstream C++ classes.
	// ToDo: Clean all the tests and create an independent google test code for the telemetry decoder.

	cout << "--------------------------------------------------------------------------" << endl;
	cout << "Entered in Galileo_Navigation_Message::split_page(char *page)" << endl << endl;;

	char Even_bit[2]={'\0'}, Odd_bit[2]={'\0'}, Page_type_Odd[2]={'\0'}, Page_type_even[2]={'\0'}, tail_Even[7]={'\0'}; //HO DATO A TUTTI UNO SPAZIO IN PIÙ PER L'ULTIMO CARATTERE
	char page_Odd[121]={'\0'};
	char page_INAV[235]={'\0'};
	char Data_j[16]={'\0'}, Data_k[112]={'\0'}, page_number_bits[6]={'\0'};
	char Data_jk_ephemeris[129]={'\0'};
	char Reserved_1[40]={'\0'};
	char SAR[22]={'\0'};
	char Spare[2]={'\0'};

	char CRC_data[24]={'\0'};
	char Reserved_2[8]={'\0'};
	char Tail_odd[6]={'\0'};
	//char correct_tail[7]="000000";
	char correct_tail[7]="011110"; //the viterbi decoder output change the tail to this value (why?)

	int Page_type=0;
	static char page_Even[114];
	/* Test to decode page 1 even joined to its corresponding
		 The Even pages given here are without their tails bits*/
		//test to detect page 1------> char page_Even[115]="000000010001011111010111101101100111110110101110101111100011001000000000001101110101110010000100101010100000010011";
		//test to detect page 2------> char page_Even[115]="000000100001011111110010111010111100000010111100000010011100000000100001011110100111000111100111000000100000000010";
		//test to detect page 3------>char page_Even[115]="000000110001011111111111111100001001100110001000110001011111110011111100110001011000111110000011011111000011110000";
		//test to detect page 4------>char page_Even[115]="000001000001011111001100000000000000101100000000000111010101111011011000000001000101001111101010011100000000010110://
		/*test to detect page 5------>char page_Even[115]="000001010010000010000000100010000010111010100000011111110001111111000111111001011000110010110011111111110110000000";*/
		//test to detect page 10-----> char page_Even[115]="000010101111101010101010101010101010101010101010101010101010101010101010101010101010111100000000000000000000000000";
		//test to detect page 6------>char page_Even[115]="000001100000000000000000000000011100001100000000000000000000000000100011010111111010001101000011100001000110101101
		/* test to detect page 7------> char page_Even[115]="000001111111101001011111010011000000000110100000001110010111111110010111110111101001000001110000111111110000101101"; */
		/* test to detect page 8------> char page_Even[115]="000010001111000000100001111100000011000001111010100000000000110100000001111010011110111000011110111101001000001110"; */
		/* test to detect page 9------> char page_Even[115]="000010011111101001011111111000101110001100000000011010100000000010111111100000001010101010101010101010101010101010"; */
		/* test to detect page 10------> char page_Even[115]="000010101111101010101010101010101010101010101010101010101010101010101010101010101010111100000000000000000000000000"; */

		/* test to detect page 0------>  char page_Even[115]="000000001001010101010101010101010101010101010101010101010101010101010101010101010101010101010101010010110001100101";*/

	cout << "Start decoding Galileo I/NAV " << endl;
//	cout<<"page input"<<page<<endl;
	cout<<page_Even<<endl;

;
	if(page[0]=='1') // if page is odd

	{
		std::cout<< "page[0] split page="<<page[0]<<endl;
//		std::cout << "Page Odd mara split page" << endl;
		strcpy(page_Odd, page);
		std::cout<<"Page odd in split page"<< endl << page_Odd << endl;

		if (flag_even_word==1)
				{
					/*Under this condition An odd page has been received but the previous even page in keep in memory and it is considered to join pages*/

					strncpy(page_INAV, page_Even, 114); //Join pages: Even+Odd=INAV page
					strncat(page_INAV, page_Odd,120);
					//std::cout << "Page INAV  Even+Odd " << endl << page_INAV << endl;

				    strncpy(Even_bit, &page_INAV[0], 1);
					//std::cout << "Even bit = " << Even_bit << endl;
					strncpy(Page_type_even, &page_INAV[1], 1);
					//std::cout << "Page type even = " << Page_type_even << endl;

					if(atoi(Page_type_even)==1) std::cout << "Alert frame "<< endl;
					else std::cout << "Nominal Page" << endl;

					strncpy(Data_k, &page_INAV[2], 112);
					//std::cout << "Data_k " << endl << Data_k << endl;

					strncpy(Odd_bit, &page_INAV[114], 1);
					//std::cout << "Odd bit: " << Odd_bit << endl;
					strncpy(Page_type_Odd, &page_INAV[115], 1);
					//std::cout << "Page_type_Odd: " << Page_type_Odd << endl;
					strncpy(Data_j, &page_INAV[116], 16);
					//std::cout << "Data_j: " << endl << Data_j << endl;

					strncpy(Reserved_1, &page_INAV[132], 40);
					strncpy(SAR, &page_INAV[172], 22);
					strncpy(Spare, &page_INAV[194], 2);
					strncpy(CRC_data, &page_INAV[196], 24);
					strncpy(Reserved_2, &page_INAV[220], 8);
					strncpy(Tail_odd, &page_INAV[228], 6);
					std::cout << "tail odd: " << endl << Tail_odd << endl;
					if(strcmp (Tail_odd,correct_tail) != 0)
								std::cout << "Tail odd is not correct!" << endl;
					else std::cout<<"Tail odd is correct!"<<endl;

					//************ CRC checksum control *******/
					std::stringstream TLM_word_for_CRC_stream;

					TLM_word_for_CRC_stream<<page_INAV;
					std::string TLM_word_for_CRC;
					TLM_word_for_CRC=TLM_word_for_CRC_stream.str().substr(0,GALILEO_DATA_FRAME_BITS);

					std::cout <<"frame for CRC="<<TLM_word_for_CRC<<std::endl;
					std::cout <<"frame length="<<TLM_word_for_CRC.length()<<std::endl;
					std::bitset<GALILEO_DATA_FRAME_BITS> TLM_word_for_CRC_bits(TLM_word_for_CRC);
					std::bitset<24> checksum(CRC_data);

					if (CRC_test(TLM_word_for_CRC_bits,checksum.to_ulong())==true)
					{
						// CRC correct: Decode word
						std::cout<<"CRC correct!"<<std::endl;
					}else{
						// CRC wrong.. discard frame
						std::cout<<"CRC error!"<<std::endl;
					}
					//********** end of CRC checksum control ***/

					strncpy(page_number_bits, &Data_k[0], 6);
					std::cout << "Page number bits from Data k" << endl << page_number_bits << endl;

					stringstream ss_page_number;
					string s_page_number;
					ss_page_number << page_number_bits; // from char to stringstream
					ss_page_number >> s_page_number; 	// from stringstream to string

					std::bitset<GALILEO_PAGE_TYPE_BITS> page_type_bits (s_page_number); // from string to bitset

					Page_type = (int)read_page_type_unsigned(page_type_bits, type);

					std::cout << "Page number (first 6 bits of Data k converted to decimal) = " << Page_type << endl;

					strncpy(Data_jk_ephemeris, &Data_k[0], 112); 	// 	Join data_j + data_k = Data_jk_ephemeris;
					strncat(Data_jk_ephemeris, Data_j, 16); 	//  Data_jk_ephemeris is the input for the function page decoder

					//std::cout<<"Data j k ephemeris" << endl << Data_jk_ephemeris << endl;

					page_jk_decoder(Data_jk_ephemeris); // Corresponding to ephemeris_decode.m in matlab code

					//internal_flag_even_word_arrived=0;

				}

		} /*if(page[0]=='1') */

	else{

	strncpy(page_Even, &page[0], 114); //ora che ha memorizzato page even dovrebbe mantenerla per la prossima volta che entro nella funzione
	std::cout << "Page even in split page" << std::endl << page_Even << std::endl;
	strncpy(tail_Even, &page[114], 6);

	if(strcmp (tail_Even,correct_tail) != 0)
			std::cout << "Tail even is not correct!" << endl;
	else std::cout<<"Tail even is correct!"<<endl;

	//flag_even_word=1;

	}

}


bool Galileo_Navigation_Message::have_new_ephemeris()
{
	//ToDo: Check if whe have a new ephemeris data set stored in the galileo navigation class
	return false;
}

bool Galileo_Navigation_Message::have_new_iono()
{
	//ToDo: Check if whe have a new iono data set stored in the galileo navigation class
	return false;
}
bool Galileo_Navigation_Message::have_new_utc_model()
{
	//ToDo: Check if whe have a new utc data set stored in the galileo navigation class
	return false;
}
bool Galileo_Navigation_Message::have_new_almanac()
{
	//ToDo: Check if whe have a new almanac data set stored in the galileo navigation class
	return false;
}

Galileo_Ephemeris Galileo_Navigation_Message::get_ephemeris()
{

	Galileo_Ephemeris ephemeris;
	//ToDo:: fill the object
	return ephemeris;
}


Galileo_Iono Galileo_Navigation_Message::get_iono()
{
	Galileo_Iono iono;
	//ToDo:: fill the object
	return iono;
}


Galileo_Utc_Model Galileo_Navigation_Message::get_utc_model()
{
	Galileo_Utc_Model utc_model;
	//ToDo:: fill the object
	return utc_model;
}


Galileo_Almanac Galileo_Navigation_Message::get_almanac()
{
	Galileo_Almanac almanac;
	//ToDo:: fill the object
	return almanac;
}


int Galileo_Navigation_Message::page_jk_decoder(char *data_jk)
{
	std::cout << "--------------------------------------------------------------------------" << endl;
	std::cout<< "Entered in function Galileo_Navigation_Message::page_jk_decoder(char *data_jk)" << endl << endl;

    int page_number = 0;

    stringstream ss;
    string str;
    ss << data_jk;
    ss >> str;

    std::bitset<GALILEO_DATA_JK_BITS> data_jk_bits (str);

    //std::cout << "Data_jk_bits (bitset)  "<< endl << data_jk_bits << endl;

    page_number = (int)read_navigation_unsigned(data_jk_bits, PAGE_TYPE_bit);
    std::cout << "Page number = " << page_number << endl;

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

        	    Delta_alpha_7= (double)read_navigation_unsigned(data_jk_bits, Delta_alpha_7_bit);
        	    Delta_alpha_7= Delta_alpha_7 * Delta_alpha_7_LSB;
        	    std::cout << "Delta_alpha_7= " << Delta_alpha_7 << std::endl;

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


   cout<<"--------------------------------------------------------------------------"<<endl;
    return page_number;
}
