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
#include "boost/date_time/posix_time/posix_time.hpp"

using namespace std;

void Galileo_Navigation_Message::reset()
{

	/*Word type 1: Ephemeris (1/4)*/
    IOD_nav_1 = 0;
    t0e = 0;
    M0 = 0;
    e = 0;
    A = 0;

    /*d_TOW_SF1 = 0;
    d_TOW_SF2 = 0;
    d_TOW_SF3 = 0;
    d_TOW_SF4 = 0;
    d_TOW_SF5 = 0;

    d_IODE_SF2 = 0;
    d_IODE_SF3 = 0;
    d_Crs = 0;
    d_Delta_n = 0;
    d_M_0 = 0;
    d_Cuc = 0;
    d_e_eccentricity = 0;
    d_Cus = 0;
    d_sqrt_A = 0;
    d_Toe = 0;
    d_Toc = 0;
    d_Cic = 0;
    d_OMEGA0 = 0;
    d_Cis = 0;
    d_i_0 = 0;
    d_Crc = 0;
    d_OMEGA = 0;
    d_OMEGA_DOT = 0;
    d_IDOT = 0;
    i_code_on_L2 = 0;
    i_GPS_week = 0;
    b_L2_P_data_flag = false;
    i_SV_accuracy = 0;
    i_SV_health = 0;
    d_TGD = 0;
    d_IODC = -1;
    i_AODO = 0;

    b_fit_interval_flag = false;
    d_spare1 = 0;
    d_spare2 = 0;

    d_A_f0 = 0;
    d_A_f1 = 0;
    d_A_f2 = 0;

    //clock terms
    //d_master_clock=0;
    d_dtr = 0;
    d_satClkCorr = 0;

    // satellite positions
    d_satpos_X = 0;
    d_satpos_Y = 0;
    d_satpos_Z = 0;

    // info
    i_channel_ID = 0;
    i_satellite_PRN = 0;

    // time synchro
    d_subframe_timestamp_ms = 0;

    // flags
    b_alert_flag = false;
    b_integrity_status_flag = false;
    b_antispoofing_flag = false;

    // Ionosphere and UTC
    flag_iono_valid = false;
    flag_utc_model_valid = true;
    d_alpha0 = 0;
    d_alpha1 = 0;
    d_alpha2 = 0;
    d_alpha3 = 0;
    d_beta0 = 0;
    d_beta1 = 0;
    d_beta2 = 0;
    d_beta3 = 0;
    d_A1 = 0;
    d_A0 = 0;
    d_t_OT = 0;
    i_WN_T = 0;
    d_DeltaT_LS = 0;
    i_WN_LSF = 0;
    i_DN = 0;
    d_DeltaT_LSF= 0;

    //Almanac
    d_Toa = 0;
    i_WN_A = 0;
    for (int i=1; i < 32; i++ )
        {
            almanacHealth[i] = 0;
        }

    // Satellite velocity
    d_satvel_X = 0;
    d_satvel_Y = 0;
    d_satvel_Z = 0;

    //Plane A (info from http://www.navcen.uscg.gov/?Do=constellationStatus)
    satelliteBlock[9] = "IIA";
    satelliteBlock[31] = "IIR-M";
    satelliteBlock[8] = "IIA";
    satelliteBlock[7] = "IIR-M";
    satelliteBlock[27] = "IIA";
    //Plane B
    satelliteBlock[16] = "IIR";
    satelliteBlock[25] = "IIF";
    satelliteBlock[28] = "IIR";
    satelliteBlock[12] = "IIR-M";
    satelliteBlock[30] = "IIA";
    //Plane C
    satelliteBlock[29] = "IIR-M";
    satelliteBlock[3] = "IIA";
    satelliteBlock[19] = "IIR";
    satelliteBlock[17] = "IIR-M";
    satelliteBlock[6] = "IIA";
    //Plane D
    satelliteBlock[2] = "IIR";
    satelliteBlock[1] = "IIF";
    satelliteBlock[21] = "IIR";
    satelliteBlock[4] = "IIA";
    satelliteBlock[11] = "IIR";
    satelliteBlock[24] = "IIA"; // Decommissioned from active service on 04 Nov 2011
    //Plane E
    satelliteBlock[20] = "IIR";
    satelliteBlock[22] = "IIR";
    satelliteBlock[5] = "IIR-M";
    satelliteBlock[18] = "IIR";
    satelliteBlock[32] = "IIA";
    satelliteBlock[10] = "IIA";
    //Plane F
    satelliteBlock[14] = "IIR";
    satelliteBlock[15] = "IIR-M";
    satelliteBlock[13] = "IIR";
    satelliteBlock[23] = "IIR";
    satelliteBlock[26] = "IIA";*/
}


Galileo_Navigation_Message::Galileo_Navigation_Message()
{
    reset();
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



/*signed long int Galileo_Navigation_Message::read_navigation_signed(std::bitset<GALILEO_SUBFRAME_BITS> bits, const std::vector<std::pair<int,int> > parameter)
{
    signed long int value = 0;
    int num_of_slices = parameter.size();
    // Discriminate between 64 bits and 32 bits compiler
    int long_int_size_bytes = sizeof(signed long int);
    if (long_int_size_bytes == 8) // if a long int takes 8 bytes, we are in a 64 bits system
        {
            // read the MSB and perform the sign extension
            if (bits[GALILEO_SUBFRAME_BITS - parameter[0].first] == 1)
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
                            if (bits[GALILEO_SUBFRAME_BITS - parameter[i].first - j] == 1)
                                {
                                    value += 1; // insert the bit
                                }
                        }
                }
        }
    else  // we assume we are in a 32 bits system
        {
            // read the MSB and perform the sign extension
            if (bits[GALILEO_SUBFRAME_BITS - parameter[0].first] == 1)
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
                            if (bits[GALILEO_SUBFRAME_BITS - parameter[i].first - j] == 1)
                                {
                                    value += 1; // insert the bit
                                }
                        }
                }
        }
    return value;
}*/

/*bool Galileo_Navigation_Message::read_navigation_bool(std::bitset<GALILEO_SUBFRAME_BITS> bits, const std::vector<std::pair<int,int> > parameter)
{
    bool value;

    if (bits[GALILEO_SUBFRAME_BITS - parameter[0].first] == 1)
        {
            value = true;
        }
    else
        {
            value = false;
        }
    return value;
}*/


/*void Galileo_Navigation_Message::print_galileo_word_bytes(unsigned int GPS_word)
{
    std::cout << " Word =";
    std::cout << std::bitset<32>(GPS_word);
    std::cout << std::endl;
}*/



void Galileo_Navigation_Message::split_page(char *page){
	cout << "--------------------------------------------------------------------------" << endl;
	cout << "Entered in Galileo_Navigation_Message::split_page(char *page)" << endl << endl;;

	char Even_bit[2]={'\0'}, Odd_bit[2]={'\0'}, Page_type_Odd[2]={'\0'}, Page_type_even[2]={'\0'}, tail_Even[7]={'\0'}; //HO DATO A TUTTI UNO SPAZIO IN PIÙ PER L'ULTIMO CARATTERE
	char page_Odd[121]={'\0'},  page_INAV[235]={'\0'};
	char Data_j[16]={'\0'}, Data_k[112]={'\0'}, page_number_bits[6]={'\0'};
	char Data_jk_ephemeris[128]={'\0'};
	char Reserved_1[40]={'\0'};
	char SAR[22]={'\0'};
	char Spare[2]={'\0'};
	char CRC_data[24]={'\0'};
	char Reserved_2[8]={'\0'};
	char Tail_odd[6]={'\0'};
	char correct_tail[7]="000000";
	int flag_even_word_arrived=1; /*******************REMEMBER TO CHANGE flag_even_word_arrived=0 *****+**********/
	int Page_type=0;

	/* Test to decode page 1 even joined to its corresponding
	 The Even pages given here are without their tails bits*/
	char page_Even[115]="000000010001011111010111101101100111110110101110101111100011001000000000001101110101110010000100101010100000010011";
	//test to detect page 3------> char page_Even[115]="000000110001011111111111111100001001100110001000110001011111110011111100110001011000111110000011011111000011110000";
	//test to detect page 10-----> char page_Even[115]="000010101111101010101010101010101010101010101010101010101010101010101010101010101010111100000000000000000000000000";
	//test to detect page 5------> char page_Even[115]="000001010010000010000000100010000010111010100000011111110001111111000111111001011000110010110011111111110110000000";
	//test to detect page 2------> char page_Even[115]="000000100001011111110010111010111100000010111100000010011100000000100001011110100111000111100111000000100000000010";


	cout << "Start decoding Galileo I/NAV " << endl;

	if(page[0]=='1') // if page is odd
		{
			strcpy(page_Odd, page);
			//cout << "Page Odd " << endl << page_Odd<< endl;

			if (flag_even_word_arrived==1) //to test the function and satisfy this condition the flag is INITIALIZED AT 1
				{

				    strncpy(page_INAV, page_Even, 114); //Join pages: Even+Odd=INAV page
				    strncat(page_INAV, page_Odd,120);

					//cout << "Page INAV  Even+Odd " << endl << page_INAV << endl;

					strncpy(Even_bit, &page_INAV[0], 1);
					//cout << "Even bit = " << Even_bit << endl;
					strncpy(Page_type_even, &page_INAV[1], 1);
					//cout << "Page type even = " << Page_type_even << endl;

					if(atoi(Page_type_even)==1) cout << "Alert frame "<< endl;
					else cout << "Nominal Page" << endl;

					strncpy(Data_k, &page_INAV[2], 112);
					//cout << "Data_k " << endl << Data_k << endl;

					strncpy(Odd_bit, &page_INAV[114], 1);
					//cout << "Odd bit: " << Odd_bit << endl;
					strncpy(Page_type_Odd, &page_INAV[115], 1);
					//cout << "Page_type_Odd: " << Page_type_Odd << endl;
					strncpy(Data_j, &page_INAV[116], 16);
					//cout << "Data_j: " << endl << Data_j << endl;

					strncpy(Reserved_1, &page_INAV[132], 40);
					strncpy(SAR, &page_INAV[172], 22);
					strncpy(Spare, &page_INAV[194], 2);
					strncpy(CRC_data, &page_INAV[196], 24);
					strncpy(Reserved_2, &page_INAV[220], 8);
					strncpy(Tail_odd, &page_INAV[228], 6);
					//cout << "tail odd: " << endl << Tail_odd << endl;
					if(strcmp (Tail_odd,correct_tail) != 0)
								cout << "Tail odd is not correct!" << endl;
					else cout<<"Tail odd is correct!"<<endl;

					/************* CRC cycle control ***********/

					strncpy(page_number_bits, &Data_k[0], 6);
					//cout << "Page number bits from Data k" << endl << page_number_bits << endl;

					stringstream ss_page_number;
					string s_page_number;
					ss_page_number << page_number_bits; // from char to stringstream
					ss_page_number >> s_page_number; 	// from stringstream to string

					std::bitset<GALILEO_PAGE_TYPE_BITS> page_type_bits (s_page_number); // from string to bitset

					Page_type = (int)read_page_type_unsigned(page_type_bits, type);

					cout << "Page number (first 6 bits of Data k converted to decimal) = " << Page_type << endl;

					strncpy(Data_jk_ephemeris, Data_k, 112); 	// 	Join data_j + data_k = Data_jk_ephemeris;
					strncat(Data_jk_ephemeris, Data_j, 16); 	//  Data_jk_ephemeris is the input for the function page decoder*/

					//cout<<"Data j k ephemeris" << endl << Data_jk_ephemeris << endl;

					page_jk_decoder(Data_jk_ephemeris); // Corresponding to ephemeris_decode.m in matlab code

					flag_even_word_arrived=0;

				}

		} /*if(page[0]=='1') */

	else{
	cout << "Page Even" << endl;
	strncpy(page_Even, &page[0], 114);
	//cout << page_Even << endl;
	strncpy(tail_Even, &page[114], 6);

	if(strcmp (tail_Even,correct_tail) != 0)
			cout << "Tail even is not correct!" << endl;
	else cout<<"Tail even is correct!"<<endl;

	flag_even_word_arrived=1;

	}
	cout << "flag EVEN word arrived = " << flag_even_word_arrived << endl;
	cout << "--------------------------------------------------------------------------" << endl;
}


int Galileo_Navigation_Message::page_jk_decoder(char *data_jk)
{
	cout << "--------------------------------------------------------------------------" << endl;
	cout<< "Entered in function Galileo_Navigation_Message::page_jk_decoder(char *data_jk)" << endl << endl;

    int page_number = 0;

    stringstream ss;
    string str;
    ss << data_jk;
    ss >> str;

    std::bitset<GALILEO_DATA_JK_BITS> data_jk_bits (str);

    //cout << "Data_jk_bits (bitset)  "<< endl << data_jk_bits << endl;

    page_number = (int)read_navigation_unsigned(data_jk_bits, PAGE_TYPE_bit);
    cout << "Page number (for the test it must return page 1) = " << page_number << endl;

    switch (page_number)
        {
    	case 1: /*Word type 1: Ephemeris (1/4)*/

    		IOD_nav_1=(int)read_navigation_unsigned(data_jk_bits, IOD_nav_page1);
    		cout<<"IOD_nav_1= "<< IOD_nav_1 <<endl;

    		t0e=(double)read_navigation_unsigned(data_jk_bits, T0E_bit);
    		t0e = t0e * t0e_LSB;
    		cout << "t0e= " << t0e <<endl;

    		M0 = (double)read_navigation_unsigned(data_jk_bits, M0_bit);
    		M0 =  M0 * M0_LSB;
    		cout << "M0= " << M0<<endl;

    		e = (double)read_navigation_unsigned(data_jk_bits, e_bit);
    		e = e * e_LSB;
    		cout << "e= " << e <<endl;

    		A = (double)read_navigation_unsigned(data_jk_bits, A_bit);
    		A = A * A_LSB;
    		cout << "A= " << A <<endl;
        }


    /*unsigned int gps_word;

    // UNPACK BYTES TO BITS AND REMOVE THE CRC REDUNDANCE
    std::bitset<GALILEO_SUBFRAME_BITS> subframe_bits;
    std::bitset<GPS_WORD_BITS+2> word_bits;
    for (int i=0; i<10; i++)
        {
            memcpy(&gps_word, &subframe[i*4], sizeof(char)*4);
            word_bits = std::bitset<(GPS_WORD_BITS+2)>(gps_word);
            for (int j=0; j<GPS_WORD_BITS; j++)
                {
                    subframe_bits[GPS_WORD_BITS*(9-i) + j] = word_bits[j];
                }
        }

    subframe_ID = (int)read_navigation_unsigned(subframe_bits, SUBFRAME_ID);

    // Decode all 5 sub-frames
    switch (subframe_ID)
    {
    //--- Decode the sub-frame id ------------------------------------------
    // ICD (IS-GPS-200E Appendix II). http://www.losangeles.af.mil/shared/media/document/AFD-100813-045.pdf
    case 1:
        //--- It is subframe 1 -------------------------------------
        // Compute the time of week (TOW) of the first sub-frames in the array ====
        // Also correct the TOW. The transmitted TOW is actual TOW of the next
        // subframe and we need the TOW of the first subframe in this data block
        // (the variable subframe at this point contains bits of the last subframe).
        //TOW = bin2dec(subframe(31:47)) * 6 - 30;
        d_TOW_SF1 = (double)read_navigation_unsigned(subframe_bits, TOW);
        //we are in the first subframe (the transmitted TOW is the start time of the next subframe) !
        d_TOW_SF1 = d_TOW_SF1*6;
        d_TOW = d_TOW_SF1 - 6; // Set transmission time
        b_integrity_status_flag = read_navigation_bool(subframe_bits, INTEGRITY_STATUS_FLAG);
        b_alert_flag = read_navigation_bool(subframe_bits, ALERT_FLAG);
        b_antispoofing_flag = read_navigation_bool(subframe_bits, ANTI_SPOOFING_FLAG);
        i_GPS_week = (int)read_navigation_unsigned(subframe_bits, GPS_WEEK);
        i_SV_accuracy = (int)read_navigation_unsigned(subframe_bits, SV_ACCURACY);  // (20.3.3.3.1.3)
        i_SV_health = (int)read_navigation_unsigned(subframe_bits, SV_HEALTH);
        b_L2_P_data_flag = read_navigation_bool(subframe_bits, L2_P_DATA_FLAG); //
        i_code_on_L2 = (int)read_navigation_unsigned(subframe_bits, CA_OR_P_ON_L2);
        d_TGD = (double)read_navigation_signed(subframe_bits, T_GD);
        d_TGD = d_TGD*T_GD_LSB;
        d_IODC = (double)read_navigation_unsigned(subframe_bits, IODC);
        d_Toc = (double)read_navigation_unsigned(subframe_bits, T_OC);
        d_Toc = d_Toc*T_OC_LSB;
        d_A_f0 = (double)read_navigation_signed(subframe_bits, A_F0);
        d_A_f0 = d_A_f0*A_F0_LSB;
        d_A_f1 = (double)read_navigation_signed(subframe_bits, A_F1);
        d_A_f1 = d_A_f1*A_F1_LSB;
        d_A_f2 = (double)read_navigation_signed(subframe_bits, A_F2);
        d_A_f2 = d_A_f2*A_F2_LSB;

        break;

    case 2:  //--- It is subframe 2 -------------------
        d_TOW_SF2 = (double)read_navigation_unsigned(subframe_bits, TOW);
        d_TOW_SF2 = d_TOW_SF2*6;
        d_TOW = d_TOW_SF2 - 6; // Set transmission time
        b_integrity_status_flag = read_navigation_bool(subframe_bits, INTEGRITY_STATUS_FLAG);
        b_alert_flag = read_navigation_bool(subframe_bits, ALERT_FLAG);
        b_antispoofing_flag = read_navigation_bool(subframe_bits, ANTI_SPOOFING_FLAG);
        d_IODE_SF2 = (double)read_navigation_unsigned(subframe_bits, IODE_SF2);
        d_Crs = (double)read_navigation_signed(subframe_bits, C_RS);
        d_Crs = d_Crs * C_RS_LSB;
        d_Delta_n = (double)read_navigation_signed(subframe_bits, DELTA_N);
        d_Delta_n = d_Delta_n * DELTA_N_LSB;
        d_M_0 = (double)read_navigation_signed(subframe_bits, M_0);
        d_M_0 = d_M_0 * M_0_LSB;
        d_Cuc = (double)read_navigation_signed(subframe_bits, C_UC);
        d_Cuc = d_Cuc * C_UC_LSB;
        d_e_eccentricity = (double)read_navigation_unsigned(subframe_bits, E);
        d_e_eccentricity = d_e_eccentricity * E_LSB;
        d_Cus = (double)read_navigation_signed(subframe_bits, C_US);
        d_Cus = d_Cus * C_US_LSB;
        d_sqrt_A = (double)read_navigation_unsigned(subframe_bits, SQRT_A);
        d_sqrt_A = d_sqrt_A * SQRT_A_LSB;
        d_Toe = (double)read_navigation_unsigned(subframe_bits, T_OE);
        d_Toe = d_Toe * T_OE_LSB;
        b_fit_interval_flag = read_navigation_bool(subframe_bits, FIT_INTERVAL_FLAG);
        i_AODO = (int)read_navigation_unsigned(subframe_bits, AODO);
        i_AODO = i_AODO * AODO_LSB;

        break;

    case 3: // --- It is subframe 3 -------------------------------------
        d_TOW_SF3 = (double)read_navigation_unsigned(subframe_bits, TOW);
        d_TOW_SF3 = d_TOW_SF3*6;
        d_TOW = d_TOW_SF3 - 6; // Set transmission time
        b_integrity_status_flag = read_navigation_bool(subframe_bits, INTEGRITY_STATUS_FLAG);
        b_alert_flag = read_navigation_bool(subframe_bits, ALERT_FLAG);
        b_antispoofing_flag = read_navigation_bool(subframe_bits, ANTI_SPOOFING_FLAG);
        d_Cic = (double)read_navigation_signed(subframe_bits, C_IC);
        d_Cic = d_Cic * C_IC_LSB;
        d_OMEGA0 = (double)read_navigation_signed(subframe_bits, OMEGA_0);
        d_OMEGA0 = d_OMEGA0 * OMEGA_0_LSB;
        d_Cis = (double)read_navigation_signed(subframe_bits, C_IS);
        d_Cis = d_Cis * C_IS_LSB;
        d_i_0 = (double)read_navigation_signed(subframe_bits, I_0);
        d_i_0 = d_i_0 * I_0_LSB;
        d_Crc = (double)read_navigation_signed(subframe_bits, C_RC);
        d_Crc = d_Crc * C_RC_LSB;
        d_OMEGA = (double)read_navigation_signed(subframe_bits, OMEGA);
        d_OMEGA = d_OMEGA * OMEGA_LSB;
        d_OMEGA_DOT = (double)read_navigation_signed(subframe_bits, OMEGA_DOT);
        d_OMEGA_DOT = d_OMEGA_DOT * OMEGA_DOT_LSB;
        d_IODE_SF3 = (double)read_navigation_unsigned(subframe_bits, IODE_SF3);
        d_IDOT = (double)read_navigation_signed(subframe_bits, I_DOT);
        d_IDOT = d_IDOT*I_DOT_LSB;

        break;

    case 4: // --- It is subframe 4 ---------- Almanac, ionospheric model, UTC parameters, SV health (PRN: 25-32)
        d_TOW_SF4 = (double)read_navigation_unsigned(subframe_bits, TOW);
        d_TOW_SF4 = d_TOW_SF4*6;
        d_TOW = d_TOW_SF4 - 6; // Set transmission time
        b_integrity_status_flag = read_navigation_bool(subframe_bits, INTEGRITY_STATUS_FLAG);
        b_alert_flag = read_navigation_bool(subframe_bits, ALERT_FLAG);
        b_antispoofing_flag = read_navigation_bool(subframe_bits, ANTI_SPOOFING_FLAG);
        SV_data_ID = (int)read_navigation_unsigned(subframe_bits, SV_DATA_ID);
        SV_page = (int)read_navigation_unsigned(subframe_bits, SV_PAGE);

        if (SV_page == 13)
            {
                //! \TODO read Estimated Range Deviation (ERD) values
            }

        if (SV_page == 18)
            {
                // Page 18 - Ionospheric and UTC data
                d_alpha0 = (double)read_navigation_signed(subframe_bits, ALPHA_0);
                d_alpha0 = d_alpha0 * ALPHA_0_LSB;
                d_alpha1 = (double)read_navigation_signed(subframe_bits, ALPHA_1);
                d_alpha1 = d_alpha1 * ALPHA_1_LSB;
                d_alpha2 = (double)read_navigation_signed(subframe_bits, ALPHA_2);
                d_alpha2 = d_alpha2 * ALPHA_2_LSB;
                d_alpha3 = (double)read_navigation_signed(subframe_bits, ALPHA_3);
                d_alpha3 = d_alpha3 * ALPHA_3_LSB;
                d_beta0 = (double)read_navigation_signed(subframe_bits, BETA_0);
                d_beta0 = d_beta0 * BETA_0_LSB;
                d_beta1 = (double)read_navigation_signed(subframe_bits, BETA_1);
                d_beta1 = d_beta1 * BETA_1_LSB;
                d_beta2 = (double)read_navigation_signed(subframe_bits, BETA_2);
                d_beta2 = d_beta2 * BETA_2_LSB;
                d_beta3 = (double)read_navigation_signed(subframe_bits, BETA_3);
                d_beta3 = d_beta3 * BETA_3_LSB;
                d_A1 = (double)read_navigation_signed(subframe_bits, A_1);
                d_A1 = d_A1 * A_1_LSB;
                d_A0 = (double)read_navigation_signed(subframe_bits, A_0);
                d_A0 = d_A0 * A_0_LSB;
                d_t_OT = (double)read_navigation_unsigned(subframe_bits, T_OT);
                d_t_OT = d_t_OT * T_OT_LSB;
                i_WN_T = (int)read_navigation_unsigned(subframe_bits, WN_T);
                d_DeltaT_LS = (double)read_navigation_signed(subframe_bits, DELTAT_LS);
                i_WN_LSF = (int)read_navigation_unsigned(subframe_bits, WN_LSF);
                i_DN = (int)read_navigation_unsigned(subframe_bits, DN);  // Right-justified ?
                d_DeltaT_LSF = (double)read_navigation_signed(subframe_bits, DELTAT_LSF);
                flag_iono_valid = true;
                flag_utc_model_valid = true;
            }

        if (SV_page == 25)
            {
                // Page 25 Anti-Spoofing, SV config and almanac health (PRN: 25-32)
                //! \TODO Read Anti-Spoofing, SV config
                almanacHealth[25] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV25);
                almanacHealth[26] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV26);
                almanacHealth[27] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV27);
                almanacHealth[28] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV28);
                almanacHealth[29] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV29);
                almanacHealth[30] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV30);
                almanacHealth[31] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV31);
                almanacHealth[32] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV32);
            }

        break;

    case 5://--- It is subframe 5 -----------------almanac health (PRN: 1-24) and Almanac reference week number and time.
        d_TOW_SF5 = (double)read_navigation_unsigned(subframe_bits, TOW);
        d_TOW_SF5 = d_TOW_SF5*6;
        d_TOW = d_TOW_SF5 - 6; // Set transmission time
        b_integrity_status_flag = read_navigation_bool(subframe_bits, INTEGRITY_STATUS_FLAG);
        b_alert_flag = read_navigation_bool(subframe_bits, ALERT_FLAG);
        b_antispoofing_flag = read_navigation_bool(subframe_bits, ANTI_SPOOFING_FLAG);
        SV_data_ID = (int)read_navigation_unsigned(subframe_bits, SV_DATA_ID);
        SV_page = (int)read_navigation_unsigned(subframe_bits, SV_PAGE);
        if (SV_page < 25)
            {
                //! \TODO read almanac
            }
        if (SV_page == 25)
            {
                d_Toa = (double)read_navigation_unsigned(subframe_bits, T_OA);
                d_Toa = d_Toa * T_OA_LSB;
                i_WN_A = (int)read_navigation_unsigned(subframe_bits, WN_A);
                almanacHealth[1] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV1);
                almanacHealth[2] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV2);
                almanacHealth[3] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV3);
                almanacHealth[4] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV4);
                almanacHealth[5] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV5);
                almanacHealth[6] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV6);
                almanacHealth[7] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV7);
                almanacHealth[8] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV8);
                almanacHealth[9] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV9);
                almanacHealth[10] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV10);
                almanacHealth[11] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV11);
                almanacHealth[12] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV12);
                almanacHealth[13] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV13);
                almanacHealth[14] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV14);
                almanacHealth[15] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV15);
                almanacHealth[16] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV16);
                almanacHealth[17] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV17);
                almanacHealth[18] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV18);
                almanacHealth[19] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV19);
                almanacHealth[20] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV20);
                almanacHealth[21] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV21);
                almanacHealth[22] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV22);
                almanacHealth[23] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV23);
                almanacHealth[24] = (int)read_navigation_unsigned(subframe_bits, HEALTH_SV24);
            }
        break;

    default:
        break;
    } // switch subframeID ...
*/cout<<"--------------------------------------------------------------------------"<<endl;
    return page_number;
}
