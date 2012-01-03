/*!
 * \file gps_navigation_message.cc
 * \brief  Implementation of a GPS NAV Data message decoder
 *
 * See http://www.gps.gov/technical/icwg/IS-GPS-200E.pdf Appendix II
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

#include "gps_navigation_message.h"


#define num_of_slices(x) sizeof(x)/sizeof(bits_slice)


void gps_navigation_message::reset()
{
    d_TOW=0;
    //broadcast orbit 1
    d_IODE_SF2=0;
    d_IODE_SF3=0;
    d_Crs=0;
    d_Delta_n=0;
    d_M_0=0;
    //broadcast orbit 2
    d_Cuc=0;
    d_e_eccentricity=0;
    d_Cus=0;
    d_sqrt_A=0;
    //broadcast orbit 3
    d_Toe=0;
    d_Toc=0;
    d_Cic=0;
    d_OMEGA0=0;
    d_Cis=0;
    //broadcast orbit 4
    d_i_0=0;
    d_Crc=0;
    d_OMEGA=0;
    d_OMEGA_DOT=0;
    //broadcast orbit 5
    d_IDOT=0;
    d_codes_on_L2=0;
    d_GPS_week=0;
    b_L2_P_data_flag=false;
    //broadcast orbit 6
    d_SV_accuracy=0;
    d_SV_health=0;
    d_TGD=0;
    d_IODC=-1;
    //broadcast orbit 7

    d_fit_interval=0;
    d_spare1=0;
    d_spare2=0;

    d_A_f0=0;
    d_A_f1=0;
    d_A_f2=0;

    //clock terms
    d_master_clock=0;
    d_dtr=0;
    d_satClkCorr=0;

    // satellite positions
    d_satpos_X=0;
    d_satpos_Y=0;
    d_satpos_Z=0;

    // info
    d_channel_ID=0;
    d_satellite_PRN=0;

    // time synchro
    d_subframe1_timestamp_ms=0;

    // flags
    b_alert_flag = false;
    b_integrity_status_flag = false;
    b_antispoofing_flag = false;


    // Ionosphere and UTC
    d_alpha0=0;
    d_alpha1=0;
    d_alpha2=0;
    d_alpha3=0;
    d_beta0=0;
    d_beta1=0;
    d_beta2=0;
    d_beta3=0;
    d_A1=0;
    d_A0=0;
    d_t_OT=0;
    d_WN_T=0;
    d_DeltaT_LS=0;
    d_WN_LSF=0;
    d_DN=0;
    d_DeltaT_LSF=0;

}



gps_navigation_message::gps_navigation_message()
{
    reset();
}



void gps_navigation_message::print_gps_word_bytes(unsigned int GPS_word)
{
  std::cout << " Word =";
  std::cout<<std::bitset<32>(GPS_word);
  std::cout<<std::endl;
}



bool gps_navigation_message::read_navigation_bool(std::bitset<GPS_SUBFRAME_BITS> bits, const bits_slice *slices)
{
    bool value;

    if (bits[GPS_SUBFRAME_BITS-slices[0].position]==1)
        {
            value=true;
        }
    else
        {
            value=false;
        }

    return value;
}





unsigned long int gps_navigation_message::read_navigation_unsigned(std::bitset<GPS_SUBFRAME_BITS> bits, const bits_slice *slices, int num_of_slices)
{
    unsigned long int value;

    value=0;
    for (int i=0;i<num_of_slices;i++)
        {
            for (int j=0;j<slices[i].length;j++)
                {
                    value<<=1; //shift left
                    if (bits[GPS_SUBFRAME_BITS-slices[i].position-j]==1)
                        {
                            value+=1; // insert the bit
                        }
                }
        }
    return value;
}






signed long int gps_navigation_message::read_navigation_signed(std::bitset<GPS_SUBFRAME_BITS> bits, const bits_slice *slices, int num_of_slices)
{
    signed long int value=0;

    // read the MSB and perform the sign extension
    if (bits[GPS_SUBFRAME_BITS-slices[0].position]==1)
        {
            value^=0xFFFFFFFF;
        }else{
                value&=0;
        }
    for (int i=0;i<num_of_slices;i++)
        {
            for (int j=0;j<slices[i].length;j++)
                {
                    value<<=1; //shift left
                    value&=0xFFFFFFFE; //reset the corresponding bit
                    if (bits[GPS_SUBFRAME_BITS-slices[i].position-j]==1)
                        {
                            value+=1; // insert the bit
                        }
                }
        }
    return value;
}





double gps_navigation_message::check_t(double time)
{
    double corrTime;
    double half_week = 302400;     // seconds
    corrTime = time;
    if (time > half_week)
        {
            corrTime = time - 2*half_week;
        }else if (time < -half_week)
            {
                corrTime = time + 2*half_week;
            }
    return corrTime;
}



void gps_navigation_message::master_clock(double transmitTime)
{
    double dt;
    double satClkCorr;
    // Find initial satellite clock correction --------------------------------

    // --- Find time difference ---------------------------------------------
    dt = check_t(transmitTime - d_Toc);

    //--- Calculate clock correction ---------------------------------------
    satClkCorr = (d_A_f2 * dt + d_A_f1) * dt +  d_A_f0 - d_TGD;

    d_master_clock = transmitTime - satClkCorr;
}



void gps_navigation_message::satpos()
{
    double tk;
    double a;
    double n;
    double n0;
    double M;
    double E;
    double E_old;
    double dE;
    double nu;
    double phi;
    double u;
    double r;
    double i;
    double Omega;

    // Find satellite's position ----------------------------------------------

    // Restore semi-major axis
    a   = d_sqrt_A*d_sqrt_A;

    // Time correction
    tk  = check_t(d_master_clock - d_Toe);

    // Initial mean motion
    n0  = sqrt(GM / (a*a*a));
    // Mean motion
    n   = n0 + d_Delta_n;

    // Mean anomaly
    M   = d_M_0 + n * tk;
    // Reduce mean anomaly to between 0 and 360 deg

    M   = fmod((M + 2*GPS_PI),(2*GPS_PI));

    // Initial guess of eccentric anomaly
    E   = M;

    // --- Iteratively compute eccentric anomaly ----------------------------
    for (int ii = 1;ii<20;ii++)
        {
            E_old   = E;
            E       = M + d_e_eccentricity * sin(E);
            dE      = fmod(E - E_old,2*GPS_PI);
            if (fabs(dE) < 1e-12)
                {
                    //Necessary precision is reached, exit from the loop
                    break;
                }
        }

    // Compute relativistic correction term
    d_dtr = F * d_e_eccentricity * d_sqrt_A * sin(E);

    // Calculate the true anomaly
    double tmp_Y=sqrt(1.0 - d_e_eccentricity*d_e_eccentricity) * sin(E);
    double tmp_X=cos(E)-d_e_eccentricity;
    nu   = atan2(tmp_Y, tmp_X);

    // Compute angle phi
    phi = nu + d_OMEGA;

    // Reduce phi to between 0 and 2*pi rad
    phi = fmod((phi),(2*GPS_PI));

    // Correct argument of latitude
    u = phi +  d_Cuc * cos(2*phi) +  d_Cus * sin(2*phi);

    // Correct radius
    r = a * (1 - d_e_eccentricity*cos(E)) +  d_Crc * cos(2*phi) +  d_Crs * sin(2*phi);


    // Correct inclination
    i = d_i_0 + d_IDOT * tk + d_Cic * cos(2*phi) +d_Cis * sin(2*phi);

    // Compute the angle between the ascending node and the Greenwich meridian
    Omega = d_OMEGA0 + (d_OMEGA_DOT - OMEGA_EARTH_DOT)*tk - OMEGA_EARTH_DOT * d_Toe;
    // Reduce to between 0 and 2*pi rad
    Omega = fmod((Omega + 2*GPS_PI),(2*GPS_PI));

    // debug
    /*
    if (this->d_channel_ID==0){
	  std::cout<<"tk"<<tk<<std::endl;
	  std::cout<<"E="<<E<<std::endl;
	  std::cout<<"d_dtr="<<d_dtr<<std::endl;
	  std::cout<<"nu="<<nu<<std::endl;
	  std::cout<<"phi="<<phi<<std::endl;
	  std::cout<<"u="<<u<<" r="<<r<<" Omega="<<Omega<<std::endl;
	  std::cout<<"i="<<i<<"\r\n";
	  std::cout<<"tmp_Y="<<tmp_Y<<"\r\n";
	  std::cout<<"tmp_X="<<tmp_X<<"\r\n";
     }
     */

    // --- Compute satellite coordinates ------------------------------------
    d_satpos_X = cos(u)*r * cos(Omega) - sin(u)*r * cos(i)*sin(Omega);
    d_satpos_Y = cos(u)*r * sin(Omega) + sin(u)*r * cos(i)*cos(Omega);
    d_satpos_Z = sin(u)*r * sin(i);
}







void gps_navigation_message::relativistic_clock_correction(double transmitTime)
{
    double dt;
    // Find final satellite clock correction --------------------------------

    // --- Find time difference ---------------------------------------------
    dt = check_t(transmitTime - d_Toc);

    //Include relativistic correction in clock correction --------------------
    d_satClkCorr = (d_A_f2 * dt + d_A_f1) * dt + d_A_f0 -d_TGD + d_dtr;
}







int gps_navigation_message::subframe_decoder(char *subframe)
{
    int subframe_ID=0;
    int SV_data_ID=0;
    int SV_page=0;
    //double tmp_TOW;

    unsigned int gps_word;

    // UNPACK BYTES TO BITS AND REMOVE THE CRC REDUNDANCE
    std::bitset<GPS_SUBFRAME_BITS> subframe_bits;
    std::bitset<GPS_WORD_BITS+2> word_bits;
    for (int i=0;i<10;i++)
        {
            memcpy(&gps_word,&subframe[i*4],sizeof(char)*4);
            word_bits=std::bitset<(GPS_WORD_BITS+2)>(gps_word);
            for (int j=0;j<GPS_WORD_BITS;j++)
                {
                    subframe_bits[GPS_WORD_BITS*(9-i)+j]=word_bits[j];
                }
        }

    // *** DEBUG
    //std::cout<<"bitset subframe="<<subframe_bits<<std::endl;
    /*
    for (int i=0; i<10;i++)
    {
    memcpy(&gps_word,&d_subframe[i*4],sizeof(char)*4);
    print_gps_word_bytes(gps_word);
    }
     */
    subframe_ID=(int)read_navigation_unsigned(subframe_bits,SUBFRAME_ID,num_of_slices(SUBFRAME_ID));
    //std::cout<<"subframe ID="<<subframe_ID<<std::endl;

    // Decode all 5 sub-frames
    switch (subframe_ID){
    //--- Decode the sub-frame id ------------------------------------------
    // ICD (IS-GPS-200E Appendix II). http://www.losangeles.af.mil/shared/media/document/AFD-100813-045.pdf
    case 1:
        //--- It is subframe 1 -------------------------------------

        // Compute the time of week (TOW) of the first sub-frames in the array ====
        // Also correct the TOW. The transmitted TOW is actual TOW of the next
        // subframe and we need the TOW of the first subframe in this data block
        // (the variable subframe at this point contains bits of the last subframe).
        //TOW = bin2dec(subframe(31:47)) * 6 - 30;
        d_TOW = (double)read_navigation_unsigned(subframe_bits,TOW,num_of_slices(TOW));
        d_TOW = d_TOW*6-6; //we are in the first subframe (the transmitted TOW is the start time of the next subframe, thus we need to substract one subframe (6 seconds)) !

        b_integrity_status_flag = read_navigation_bool(subframe_bits, INTEGRITY_STATUS_FLAG);
        b_alert_flag = read_navigation_bool(subframe_bits, ALERT_FLAG);
        b_antispoofing_flag = read_navigation_bool(subframe_bits, ANTI_SPOOFING_FLAG);

        // It contains WN, SV clock corrections, health and accuracy
        d_GPS_week = (double)read_navigation_unsigned(subframe_bits,GPS_WEEK,num_of_slices(GPS_WEEK));
        d_SV_accuracy = (double)read_navigation_unsigned(subframe_bits,SV_ACCURACY,num_of_slices(SV_ACCURACY));  //should be an int (20.3.3.3.1.3)
        d_SV_health = (double)read_navigation_unsigned(subframe_bits,SV_HEALTH,num_of_slices(SV_HEALTH));

        b_L2_P_data_flag = read_navigation_bool(subframe_bits,L2_P_DATA_FLAG); //
        d_codes_on_L2 = (double)read_navigation_unsigned(subframe_bits,CA_OR_P_ON_L2,num_of_slices(CA_OR_P_ON_L2));
        d_TGD = (double)read_navigation_signed(subframe_bits,T_GD,num_of_slices(T_GD));
        d_TGD = d_TGD*T_GD_LSB;
        d_IODC = (double)read_navigation_unsigned(subframe_bits,IODC,num_of_slices(IODC));
        d_Toc = (double)read_navigation_unsigned(subframe_bits,T_OC,num_of_slices(T_OC));
        d_Toc = d_Toc*T_OC_LSB;
        d_A_f0 = (double)read_navigation_signed(subframe_bits,A_F0,num_of_slices(A_F0));
        d_A_f0 = d_A_f0*A_F0_LSB;
        d_A_f1 = (double)read_navigation_signed(subframe_bits,A_F1,num_of_slices(A_F1));
        d_A_f1 = d_A_f1*A_F1_LSB;
        d_A_f2 = (double)read_navigation_signed(subframe_bits,A_F2,num_of_slices(A_F2));
        d_A_f2 = d_A_f2*A_F2_LSB;


        /*
        eph.weekNumber  = bin2dec(subframe(61:70)) + 1024;
        eph.accuracy    = bin2dec(subframe(73:76));
        eph.health      = bin2dec(subframe(77:82));
        eph.T_GD        = twosComp2dec(subframe(197:204)) * 2^(-31);
        eph.IODC        = bin2dec([subframe(83:84) subframe(197:204)]);
        eph.t_oc        = bin2dec(subframe(219:234)) * 2^4;
        eph.a_f2        = twosComp2dec(subframe(241:248)) * 2^(-55);
        eph.a_f1        = twosComp2dec(subframe(249:264)) * 2^(-43);
        eph.a_f0        = twosComp2dec(subframe(271:292)) * 2^(-31);
         */
        break;

    case 2:
        //tmp_TOW=(double)read_navigation_unsigned(subframe_bits,TOW,num_of_slices(TOW));
        //std::cout<<"tmp_TOW="<<tmp_TOW<<std::endl;

        b_integrity_status_flag = read_navigation_bool(subframe_bits, INTEGRITY_STATUS_FLAG);
        b_alert_flag = read_navigation_bool(subframe_bits, ALERT_FLAG);
        b_antispoofing_flag = read_navigation_bool(subframe_bits, ANTI_SPOOFING_FLAG);
        // --- It is subframe 2 -------------------------------------
        // It contains first part of ephemeris parameters
        d_IODE_SF2 = (double)read_navigation_unsigned(subframe_bits,IODE_SF2,num_of_slices(IODE_SF2));
        d_Crs = (double)read_navigation_signed(subframe_bits,C_RS,num_of_slices(C_RS));
        d_Crs =d_Crs * C_RS_LSB;
        d_Delta_n = (double)read_navigation_signed(subframe_bits,DELTA_N,num_of_slices(DELTA_N));
        d_Delta_n = d_Delta_n * DELTA_N_LSB;
        d_M_0 = (double)read_navigation_signed(subframe_bits,M_0,num_of_slices(M_0));
        d_M_0 = d_M_0 * M_0_LSB;
        d_Cuc = (double)read_navigation_signed(subframe_bits,C_UC,num_of_slices(C_UC));
        d_Cuc = d_Cuc * C_UC_LSB;
        d_e_eccentricity = (double)read_navigation_unsigned(subframe_bits,E,num_of_slices(E));
        d_e_eccentricity = d_e_eccentricity * E_LSB;
        d_Cus = (double)read_navigation_signed(subframe_bits,C_US,num_of_slices(C_US));
        d_Cus = d_Cus * C_US_LSB;
        d_sqrt_A = (double)read_navigation_unsigned(subframe_bits,SQRT_A,num_of_slices(SQRT_A));
        d_sqrt_A = d_sqrt_A * SQRT_A_LSB;
        d_Toe = (double)read_navigation_unsigned(subframe_bits,T_OE,num_of_slices(T_OE));
        d_Toe = d_Toe * T_OE_LSB;

        break;
        /*
        eph.IODE_sf2    = bin2dec(subframe(61:68));
        eph.C_rs        = twosComp2dec(subframe(69: 84)) * 2^(-5);
        eph.deltan      = twosComp2dec(subframe(91:106)) * 2^(-43) * gpsPi;
        eph.M_0         = twosComp2dec([subframe(107:114) subframe(121:144)])* 2^(-31) * gpsPi;
        eph.C_uc        = twosComp2dec(subframe(151:166)) * 2^(-29);
        eph.e           = bin2dec([subframe(167:174) subframe(181:204)])* 2^(-33);
        eph.C_us        = twosComp2dec(subframe(211:226)) * 2^(-29);
        eph.sqrtA       = bin2dec([subframe(227:234) subframe(241:264)])* 2^(-19);
        eph.t_oe        = bin2dec(subframe(271:286)) * 2^4;
         */
    case 3:
        //tmp_TOW=(double)read_navigation_unsigned(subframe_bits,TOW,num_of_slices(TOW));
        //std::cout<<"tmp_TOW="<<tmp_TOW<<std::endl;
        b_integrity_status_flag = read_navigation_bool(subframe_bits, INTEGRITY_STATUS_FLAG);
        b_alert_flag = read_navigation_bool(subframe_bits, ALERT_FLAG);
        b_antispoofing_flag = read_navigation_bool(subframe_bits, ANTI_SPOOFING_FLAG);
        // --- It is subframe 3 -------------------------------------
        // It contains second part of ephemeris parameters
        d_Cic = (double)read_navigation_signed(subframe_bits,C_IC,num_of_slices(C_IC));
        d_Cic = d_Cic * C_IC_LSB;
        d_OMEGA0 = (double)read_navigation_signed(subframe_bits,OMEGA_0,num_of_slices(OMEGA_0));
        d_OMEGA0 = d_OMEGA0 * OMEGA_0_LSB;
        d_Cis = (double)read_navigation_signed(subframe_bits,C_IS,num_of_slices(C_IS));
        d_Cis = d_Cis * C_IS_LSB;
        d_i_0 = (double)read_navigation_signed(subframe_bits,I_0,num_of_slices(I_0));
        d_i_0 = d_i_0 * I_0_LSB;
        d_Crc = (double)read_navigation_signed(subframe_bits,C_RC,num_of_slices(C_RC));
        d_Crc = d_Crc * C_RC_LSB;
        d_OMEGA = (double)read_navigation_signed(subframe_bits,OMEGA,num_of_slices(OMEGA));
        d_OMEGA = d_OMEGA * OMEGA_LSB;
        d_OMEGA_DOT = (double)read_navigation_signed(subframe_bits,OMEGA_DOT,num_of_slices(OMEGA_DOT));
        d_OMEGA_DOT = d_OMEGA_DOT * OMEGA_DOT_LSB;
        d_IODE_SF3 = (double)read_navigation_unsigned(subframe_bits,IODE_SF3,num_of_slices(IODE_SF3));
        d_IDOT = (double)read_navigation_signed(subframe_bits,I_DOT,num_of_slices(I_DOT));
        d_IDOT = d_IDOT*I_DOT_LSB;

        break;
        /*
        eph.C_ic        = twosComp2dec(subframe(61:76)) * 2^(-29);
        eph.omega_0     = twosComp2dec([subframe(77:84) subframe(91:114)])* 2^(-31) * gpsPi;
        eph.C_is        = twosComp2dec(subframe(121:136)) * 2^(-29);
        eph.i_0         = twosComp2dec([subframe(137:144) subframe(151:174)])* 2^(-31) * gpsPi;
        eph.C_rc        = twosComp2dec(subframe(181:196)) * 2^(-5);
        eph.omega       = twosComp2dec([subframe(197:204) subframe(211:234)])* 2^(-31) * gpsPi;
        eph.omegaDot    = twosComp2dec(subframe(241:264)) * 2^(-43) * gpsPi;
        eph.IODE_sf3    = bin2dec(subframe(271:278));
        eph.iDot        = twosComp2dec(subframe(279:292)) * 2^(-43) * gpsPi;
         */
    case 4:
        //tmp_TOW=(double)read_navigation_unsigned(subframe_bits,TOW,num_of_slices(TOW));
        //std::cout<<"tmp_TOW="<<tmp_TOW<<std::endl;
        b_integrity_status_flag = read_navigation_bool(subframe_bits, INTEGRITY_STATUS_FLAG);
        b_alert_flag = read_navigation_bool(subframe_bits, ALERT_FLAG);
        b_antispoofing_flag = read_navigation_bool(subframe_bits, ANTI_SPOOFING_FLAG);
        // --- It is subframe 4 -------------------------------------
        // Almanac, ionospheric model, UTC parameters.
        // SV health (PRN: 25-32)
        SV_data_ID = (int)read_navigation_unsigned(subframe_bits,SV_DATA_ID,num_of_slices(SV_DATA_ID));
        SV_page = (int)read_navigation_unsigned(subframe_bits,SV_PAGE,num_of_slices(SV_PAGE));

        if (SV_page == 4)
            {
                // Page 18 - Ionospheric and UTC data

                d_alpha0 = (double)read_navigation_signed(subframe_bits, ALPHA_0,num_of_slices(ALPHA_0));
                d_alpha0 = d_alpha0 * ALPHA_0_LSB;
                d_alpha1 = (double)read_navigation_signed(subframe_bits, ALPHA_1,num_of_slices(ALPHA_1));
                d_alpha1 = d_alpha1 * ALPHA_1_LSB;
                d_alpha2 = (double)read_navigation_signed(subframe_bits, ALPHA_2,num_of_slices(ALPHA_2));
                d_alpha2 = d_alpha2 * ALPHA_2_LSB;
                d_alpha3 = (double)read_navigation_signed(subframe_bits, ALPHA_3,num_of_slices(ALPHA_3));
                d_alpha3 = d_alpha3 * ALPHA_3_LSB;
                d_beta0 = (double)read_navigation_signed(subframe_bits, BETA_0,num_of_slices(BETA_0));
                d_beta0 = d_beta0 * BETA_0_LSB;
                d_beta1 = (double)read_navigation_signed(subframe_bits, BETA_1,num_of_slices(BETA_1));
                d_beta1 = d_beta1 * BETA_1_LSB;
                d_beta2 = (double)read_navigation_signed(subframe_bits, BETA_2,num_of_slices(BETA_2));
                d_beta2 = d_beta2 * BETA_2_LSB;
                d_beta3 = (double)read_navigation_signed(subframe_bits, BETA_3,num_of_slices(BETA_3));
                d_beta3 = d_beta3 * BETA_3_LSB;
                d_A1 = (double)read_navigation_signed(subframe_bits, A_1,num_of_slices(A_1));
                d_A1 = d_A1 * A_1_LSB;
                d_A0 = (double)read_navigation_signed(subframe_bits, A_0,num_of_slices(A_0));
                d_A0 = d_A0 * A_0_LSB;
                d_t_OT = (double)read_navigation_unsigned(subframe_bits, T_OT,num_of_slices(T_OT));
                d_t_OT = d_t_OT * T_OT_LSB;
                d_WN_T = (double)read_navigation_unsigned(subframe_bits, WN_T,num_of_slices(WN_T));
                d_DeltaT_LS = (double)read_navigation_signed(subframe_bits, DELTAT_LS,num_of_slices(DELTAT_LS));
                d_WN_LSF = (double)read_navigation_unsigned(subframe_bits, WN_LSF,num_of_slices(WN_LSF));
                d_DN = (double)read_navigation_unsigned(subframe_bits, DN,num_of_slices(DN));;  // Right-justified ?
                d_DeltaT_LSF = (double)read_navigation_signed(subframe_bits, DELTAT_LSF,num_of_slices(DELTAT_LSF));

            }

        break;

    case 5:
        //tmp_TOW=(double)read_navigation_unsigned(subframe_bits,TOW,num_of_slices(TOW));
        //std::cout<<"tmp_TOW="<<tmp_TOW<<std::endl;
        //--- It is subframe 5 -------------------------------------
        // SV almanac and health (PRN: 1-24).
        // Almanac reference week number and time.
        SV_data_ID = (int)read_navigation_unsigned(subframe_bits,SV_DATA_ID,num_of_slices(SV_DATA_ID));
        SV_page = (int)read_navigation_unsigned(subframe_bits,SV_PAGE,num_of_slices(SV_PAGE));

        break;
    default:
        break;
    } // switch subframeID ...

    return subframe_ID;
}





bool gps_navigation_message::satellite_validation()
{

    bool flag_data_valid = false;

    // check Issue Of Ephemeris Data (IODE IODC..) to find a possible interrupted reception
    // and check if the data have been filled (!=0)
    if (d_IODE_SF2 == d_IODE_SF3 and d_IODC == d_IODE_SF2 and d_IODC!=-1)
        {
            flag_data_valid=true;
        }
    return flag_data_valid;
}
