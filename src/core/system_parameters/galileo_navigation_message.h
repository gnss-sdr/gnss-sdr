/*!
 * \file galileo_navigation_message.h
 * \brief  Implementation of a Galileo I/NAV Data message
 *         as described in Galileo OS SIS ICD Issue 1.2 (Nov. 2015)
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_GALILEO_NAVIGATION_MESSAGE_H_
#define GNSS_SDR_GALILEO_NAVIGATION_MESSAGE_H_

#include "galileo_ephemeris.h"
#include "galileo_iono.h"
#include "galileo_almanac.h"
#include "galileo_utc_model.h"
#include "Galileo_E1.h"
#include <boost/cstdint.hpp>  // for boost::uint32_t
#include <bitset>
#include <map>
#include <string>
#include <utility>
#include <vector>


/*!
 * \brief This class handles the Galileo I/NAV Data message, as described in the
 * Galileo Open Service Signal in Space Interface Control Document (OS SIS ICD), Issue 1.2 (Nov 2015).
 * See https://www.gsc-europa.eu/system/files/galileo_documents/Galileo_OS_SIS_ICD.pdf
 */
class Galileo_Navigation_Message
{
private:
    bool CRC_test(std::bitset<GALILEO_DATA_FRAME_BITS> bits, boost::uint32_t checksum);
    bool read_navigation_bool(std::bitset<GALILEO_DATA_JK_BITS> bits, const std::vector<std::pair<int, int> > parameter);
    //void print_galileo_word_bytes(unsigned int GPS_word);
    unsigned long int read_navigation_unsigned(std::bitset<GALILEO_DATA_JK_BITS> bits, const std::vector<std::pair<int, int> > parameter);
    unsigned long int read_page_type_unsigned(std::bitset<GALILEO_PAGE_TYPE_BITS> bits, const std::vector<std::pair<int, int> > parameter);
    signed long int read_navigation_signed(std::bitset<GALILEO_DATA_JK_BITS> bits, const std::vector<std::pair<int, int> > parameter);

public:
    int Page_type_time_stamp;
    int flag_even_word;
    std::string page_Even;
    bool flag_CRC_test;
    bool flag_all_ephemeris;  //!< Flag indicating that all words containing ephemeris have been received
    bool flag_ephemeris_1;    //!< Flag indicating that ephemeris 1/4 (word 1) have been received
    bool flag_ephemeris_2;    //!< Flag indicating that ephemeris 2/4 (word 2) have been received
    bool flag_ephemeris_3;    //!< Flag indicating that ephemeris 3/4 (word 3) have been received
    bool flag_ephemeris_4;    //!< Flag indicating that ephemeris 4/4 (word 4) have been received

    bool flag_iono_and_GST;  //!< Flag indicating that ionospheric and GST parameters (word 5) have been received
    bool flag_TOW_5;
    bool flag_TOW_6;
    bool flag_TOW_set;    //!< it is true when page 5 or page 6 arrives
    bool flag_utc_model;  //!< Flag indicating that utc model parameters (word 6) have been received

    bool flag_all_almanac;  //!< Flag indicating that all almanac have been received
    bool flag_almanac_1;    //!< Flag indicating that almanac 1/4 (word 7) have been received
    bool flag_almanac_2;    //!< Flag indicating that almanac 2/4 (word 8) have been received
    bool flag_almanac_3;    //!< Flag indicating that almanac 3/4 (word 9) have been received
    bool flag_almanac_4;    //!< Flag indicating that almanac 4/4 (word 10) have been received

    int IOD_ephemeris;

    bool flag_GGTO;
    bool flag_GGTO_1;
    bool flag_GGTO_2;
    bool flag_GGTO_3;
    bool flag_GGTO_4;

    /*Word type 1: Ephemeris (1/4)*/
    int IOD_nav_1;  //!< IOD_nav page 1
    double t0e_1;   //!< Ephemeris reference time [s]
    double M0_1;    //!< Mean anomaly at reference time [semi-circles]
    double e_1;     //!< Eccentricity
    double A_1;     //!< Square root of the semi-major axis [meters^1/2]

    /*Word type 2: Ephemeris (2/4)*/
    int IOD_nav_2;     //!< IOD_nav page 2
    double OMEGA_0_2;  //!< Longitude of ascending node of orbital plane at weekly epoch [semi-circles]
    double i_0_2;      //!< Inclination angle at reference time  [semi-circles]
    double omega_2;    //!< Argument of perigee [semi-circles]
    double iDot_2;     //!< Rate of inclination angle [semi-circles/sec]

    /*Word type 3: Ephemeris (3/4) and SISA*/
    int IOD_nav_3;       //
    double OMEGA_dot_3;  //!< Rate of right ascension [semi-circles/sec]
    double delta_n_3;    //!< Mean motion difference from computed value  [semi-circles/sec]
    double C_uc_3;       //!< Amplitude of the cosine harmonic correction term to the argument of latitude [radians]
    double C_us_3;       //!< Amplitude of the sine harmonic correction term to the argument of latitude [radians]
    double C_rc_3;       //!< Amplitude of the cosine harmonic correction term to the orbit radius [meters]
    double C_rs_3;       //!< Amplitude of the sine harmonic correction term to the orbit radius [meters]
    double SISA_3;

    /*Word type 4: Ephemeris (4/4) and Clock correction parameters*/
    int IOD_nav_4;    //
    int SV_ID_PRN_4;  //
    double C_ic_4;    //!<Amplitude of the cosine harmonic correction term to the angle of inclination [radians]
    double C_is_4;    //!< Amplitude of the sine harmonic correction term to the angle of inclination [radians]
    /*Clock correction parameters*/
    double t0c_4;  //!< Clock correction data reference Time of Week [sec]
    double af0_4;  //!< SV clock bias correction coefficient [s]
    double af1_4;  //!< SV clock drift correction coefficient [s/s]
    double af2_4;  //!< clock drift rate correction coefficient [s/s^2]
    double spare_4;

    /* Word type 5: Ionospheric correction, BGD, signal health and data validity status and GST*/
    /* Ionospheric correction*/
    double ai0_5;  //!< Effective Ionisation Level 1st order parameter [sfu]
    double ai1_5;  //!< Effective Ionisation Level 2st order parameter [sfu/degree]
    double ai2_5;  //!< Effective Ionisation Level 3st order parameter [sfu/degree]

    /*Ionospheric disturbance flag*/
    bool Region1_flag_5;  //!< Ionospheric Disturbance Flag for region 1
    bool Region2_flag_5;  //!< Ionospheric Disturbance Flag for region 2
    bool Region3_flag_5;  //!< Ionospheric Disturbance Flag for region 3
    bool Region4_flag_5;  //!< Ionospheric Disturbance Flag for region 4
    bool Region5_flag_5;  //!< Ionospheric Disturbance Flag for region 5
    double BGD_E1E5a_5;   //!< E1-E5a Broadcast Group Delay [s]
    double BGD_E1E5b_5;   //!< E1-E5b Broadcast Group Delay [s]

    double E5b_HS_5;   //!< E5b Signal Health Status
    double E1B_HS_5;   //!< E1B Signal Health Status
    double E5b_DVS_5;  //!< E5b Data Validity Status
    double E1B_DVS_5;  //!< E1B Data Validity Status
    /*GST*/
    double WN_5;
    double TOW_5;
    double spare_5;

    /* Word type 6: GST-UTC conversion parameters */
    double A0_6;
    double A1_6;
    double Delta_tLS_6;
    double t0t_6;
    double WNot_6;
    double WN_LSF_6;
    double DN_6;
    double Delta_tLSF_6;
    double TOW_6;

    /* Word type 7: Almanac for SVID1 (1/2), almanac reference time and almanac reference week number */
    int IOD_a_7;
    double WN_a_7;
    double t0a_7;
    int SVID1_7;
    double DELTA_A_7;
    double e_7;
    double omega_7;
    double delta_i_7;
    double Omega0_7;
    double Omega_dot_7;
    double M0_7;

    /* Word type 8: Almanac for SVID1 (2/2) and SVID2 (1/2) */
    int IOD_a_8;
    double af0_8;
    double af1_8;
    double E5b_HS_8;
    double E1B_HS_8;
    int SVID2_8;
    double DELTA_A_8;
    double e_8;
    double omega_8;
    double delta_i_8;
    double Omega0_8;
    double Omega_dot_8;

    /* Word type 9: Almanac for SVID2 (2/2) and SVID3 (1/2) */
    int IOD_a_9;
    double WN_a_9;
    double t0a_9;
    double M0_9;
    double af0_9;
    double af1_9;
    double E5b_HS_9;
    double E1B_HS_9;
    int SVID3_9;
    double DELTA_A_9;
    double e_9;
    double omega_9;
    double delta_i_9;

    /* Word type 10: Almanac for SVID3 (2/2) and GST-GPS conversion parameters */
    int IOD_a_10;
    double Omega0_10;
    double Omega_dot_10;
    double M0_10;
    double af0_10;
    double af1_10;
    double E5b_HS_10;
    double E1B_HS_10;
    // GST-GPS conversion
    double A_0G_10;   //!< Constant term of the offset Delta t systems
    double A_1G_10;   //!< Rate of change of the offset Delta t systems
    double t_0G_10;   //!< Reference time for Galileo/GPS Time Offset (GGTO) data
    double WN_0G_10;  //!< Week Number of Galileo/GPS Time Offset (GGTO) reference
    /*Word type 0: I/NAV Spare Word*/
    double Time_0;
    double WN_0;
    double TOW_0;

    double Galileo_satClkDrift;
    double Galileo_dtr;  //!< Relativistic clock correction term

    // satellite positions
    double galileo_satpos_X;  //!< Earth-fixed coordinate x of the satellite [m]. Intersection of the IERS Reference Meridian (IRM) and the plane passing through the origin and normal to the Z-axis.
    double galileo_satpos_Y;  //!< Earth-fixed coordinate y of the satellite [m]. Completes a right-handed, Earth-Centered, Earth-Fixed orthogonal coordinate system.
    double galileo_satpos_Z;  //!< Earth-fixed coordinate z of the satellite [m]. The direction of the IERS (International Earth Rotation and Reference Systems Service) Reference Pole (IRP).
    // Satellite velocity
    double galileo_satvel_X;  //!< Earth-fixed velocity coordinate x of the satellite [m]
    double galileo_satvel_Y;  //!< Earth-fixed velocity coordinate y of the satellite [m]
    double galileo_satvel_Z;  //!< Earth-fixed velocity coordinate z of the satellite [m]

    /*
     * \brief Takes in input a page (Odd or Even) of 120 bit, split it according ICD 4.3.2.3 and join Data_k with Data_j
     */
    void split_page(std::string page_string, int flag_even_word);

    /*
     * \brief Takes in input Data_jk (128 bit) and split it in ephemeris parameters according ICD 4.3.5
     *
     * Takes in input Data_jk (128 bit) and split it in ephemeris parameters according ICD 4.3.5
     */
    int page_jk_decoder(const char *data_jk);

    void reset();

    /*
     * \brief Returns true if new Ephemeris has arrived. The flag is set to false when the function is executed
     */
    bool have_new_ephemeris();

    /*
     * \brief Returns true if new Iono model has arrived. The flag is set to false when the function is executed
     */
    bool have_new_iono_and_GST();

    /*
     * \brief Returns true if new UTC model has arrived. The flag is set to false when the function is executed
     */
    bool have_new_utc_model();

    /*
     * \brief Returns true if new UTC model has arrived. The flag is set to false when the function is executed
     */
    bool have_new_almanac();

    /*
     * \brief Returns a Galileo_Ephemeris object filled with the latest navigation data received
     */
    Galileo_Ephemeris get_ephemeris();

    /*
     * \brief Returns a Galileo_Iono object filled with the latest navigation data received
     */
    Galileo_Iono get_iono();

    /*
     * \brief Returns a Galileo_Utc_Model object filled with the latest navigation data received
     */
    Galileo_Utc_Model get_utc_model();

    /*
     * \brief Returns a Galileo_Almanac object filled with the latest navigation data received
     */
    Galileo_Almanac get_almanac();

    Galileo_Navigation_Message();
};

#endif /* GALILEO_NAVIGATION_MESSAGE_H_ */
