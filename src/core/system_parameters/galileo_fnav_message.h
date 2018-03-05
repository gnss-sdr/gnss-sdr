/*!
 * \file galileo_fnav_message.h
 * \brief  Implementation of a Galileo F/NAV Data message
 *         as described in Galileo OS SIS ICD Issue 1.2 (Nov. 2015)
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \based on work from:
 * <ul>
 * <li> Javier Arribas, 2011. jarribas(at)cttc.es
 * </ul>
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GALILEO_FNAV_MESSAGE_H_
#define GNSS_SDR_GALILEO_FNAV_MESSAGE_H_


#include "galileo_ephemeris.h"
#include "galileo_iono.h"
#include "galileo_almanac.h"
#include "galileo_utc_model.h"
#include "Galileo_E5a.h"
#include <boost/cstdint.hpp>  // for boost::uint16_t
#include <bitset>
#include <string>
#include <vector>
#include <utility>

/*!
 * \brief This class handles the Galileo F/NAV Data message, as described in the
 * Galileo Open Service Signal in Space Interface Control Document (OS SIS ICD), Issue 1.2 (Nov 2015).
 * See https://www.gsc-europa.eu/system/files/galileo_documents/Galileo_OS_SIS_ICD.pdf
 */
class Galileo_Fnav_Message
{
public:
    //    void Galileo_Fnav_Message::split_page(std::string page_string);
    //    void Galileo_Fnav_Message::reset();
    //    bool Galileo_Fnav_Message::have_new_ephemeris();
    //    bool Galileo_Fnav_Message::have_new_iono_and_GST();
    //    bool Galileo_Fnav_Message::have_new_utc_model();
    //    bool Galileo_Fnav_Message::have_new_almanac();
    //    Galileo_Ephemeris Galileo_Fnav_Message::get_ephemeris();
    //    Galileo_Iono Galileo_Fnav_Message::get_iono();
    //    Galileo_Utc_Model Galileo_Fnav_Message::get_utc_model();
    //    Galileo_Almanac Galileo_Fnav_Message::get_almanac();
    //
    void split_page(std::string page_string);
    void reset();
    bool have_new_ephemeris();
    bool have_new_iono_and_GST();
    bool have_new_utc_model();
    bool have_new_almanac();
    Galileo_Ephemeris get_ephemeris();
    Galileo_Iono get_iono();
    Galileo_Utc_Model get_utc_model();
    Galileo_Almanac get_almanac();

    Galileo_Fnav_Message();

    bool flag_CRC_test;
    bool flag_all_ephemeris;  //!< Flag indicating that all words containing ephemeris have been received
    bool flag_ephemeris_1;    //!< Flag indicating that ephemeris 1/3 (word 2) have been received
    bool flag_ephemeris_2;    //!< Flag indicating that ephemeris 2/3 (word 3) have been received
    bool flag_ephemeris_3;    //!< Flag indicating that ephemeris 3/3 (word 4) have been received

    bool flag_iono_and_GST;  //!< Flag indicating that ionospheric and GST parameters (word 1) have been received
    bool flag_TOW_1;
    bool flag_TOW_2;
    bool flag_TOW_3;
    bool flag_TOW_4;
    bool flag_TOW_set;    //!< it is true when page 1,2,3 or 4 arrives
    bool flag_utc_model;  //!< Flag indicating that utc model parameters (word 4) have been received

    bool flag_all_almanac;  //!< Flag indicating that all almanac have been received
    bool flag_almanac_1;    //!< Flag indicating that almanac 1/2 (word 5) have been received
    bool flag_almanac_2;    //!< Flag indicating that almanac 2/2 (word 6) have been received

    int IOD_ephemeris;

    int page_type;
    /* WORD 1 SVID, Clock correction, SISA, Ionospheric correction, BGD, GST, Signal
     * health and Data validity status*/
    int FNAV_SV_ID_PRN_1;
    int FNAV_IODnav_1;
    double FNAV_t0c_1;
    double FNAV_af0_1;
    double FNAV_af1_1;
    double FNAV_af2_1;
    double FNAV_SISA_1;
    double FNAV_ai0_1;
    double FNAV_ai1_1;
    double FNAV_ai2_1;
    bool FNAV_region1_1;
    bool FNAV_region2_1;
    bool FNAV_region3_1;
    bool FNAV_region4_1;
    bool FNAV_region5_1;
    double FNAV_BGD_1;
    double FNAV_E5ahs_1;
    double FNAV_WN_1;
    double FNAV_TOW_1;
    bool FNAV_E5advs_1;

    // WORD 2 Ephemeris (1/3) and GST
    int FNAV_IODnav_2;
    double FNAV_M0_2;
    double FNAV_omegadot_2;
    double FNAV_e_2;
    double FNAV_a12_2;
    double FNAV_omega0_2;
    double FNAV_idot_2;
    double FNAV_WN_2;
    double FNAV_TOW_2;

    // WORD 3 Ephemeris (2/3) and GST
    int FNAV_IODnav_3;
    double FNAV_i0_3;
    double FNAV_w_3;
    double FNAV_deltan_3;
    double FNAV_Cuc_3;
    double FNAV_Cus_3;
    double FNAV_Crc_3;
    double FNAV_Crs_3;
    double FNAV_t0e_3;
    double FNAV_WN_3;
    double FNAV_TOW_3;

    /* WORD 4 Ephemeris (3/3), GST-UTC conversion, GST-GPS conversion and TOW.
    Note that the clock is repeated in this page type*/
    int FNAV_IODnav_4;
    double FNAV_Cic_4;
    double FNAV_Cis_4;
    double FNAV_A0_4;
    double FNAV_A1_4;
    double FNAV_deltatls_4;
    double FNAV_t0t_4;
    double FNAV_WNot_4;
    double FNAV_WNlsf_4;
    double FNAV_DN_4;
    double FNAV_deltatlsf_4;
    double FNAV_t0g_4;
    double FNAV_A0g_4;
    double FNAV_A1g_4;
    double FNAV_WN0g_4;
    double FNAV_TOW_4;

    // WORD 5 Almanac (SVID1 and SVID2(1/2)), Week Number and almanac reference time
    int FNAV_IODa_5;
    double FNAV_WNa_5;
    double FNAV_t0a_5;
    int FNAV_SVID1_5;
    double FNAV_Deltaa12_1_5;
    double FNAV_e_1_5;
    double FNAV_w_1_5;
    double FNAV_deltai_1_5;
    double FNAV_Omega0_1_5;
    double FNAV_Omegadot_1_5;
    double FNAV_M0_1_5;
    double FNAV_af0_1_5;
    double FNAV_af1_1_5;
    unsigned int FNAV_E5ahs_1_5;
    int FNAV_SVID2_5;
    double FNAV_Deltaa12_2_5;
    double FNAV_e_2_5;
    double FNAV_w_2_5;
    double FNAV_deltai_2_5;

    // WORD 6 Almanac (SVID2(2/2) and SVID3)
    int FNAV_IODa_6;
    double FNAV_Omega0_2_6;
    double FNAV_Omegadot_2_6;
    double FNAV_M0_2_6;
    double FNAV_af0_2_6;
    double FNAV_af1_2_6;
    double FNAV_E5ahs_2_6;
    int FNAV_SVID3_6;
    double FNAV_Deltaa12_3_6;
    double FNAV_e_3_6;
    double FNAV_w_3_6;
    double FNAV_deltai_3_6;
    double FNAV_Omega0_3_6;
    double FNAV_Omegadot_3_6;
    double FNAV_M0_3_6;
    double FNAV_af0_3_6;
    double FNAV_af1_3_6;
    double FNAV_E5ahs_3_6;


private:
    bool _CRC_test(std::bitset<GALILEO_FNAV_DATA_FRAME_BITS> bits, boost::uint32_t checksum);
    void decode_page(std::string data);
    unsigned long int read_navigation_unsigned(std::bitset<GALILEO_FNAV_DATA_FRAME_BITS> bits, const std::vector<std::pair<int, int>> parameter);
    signed long int read_navigation_signed(std::bitset<GALILEO_FNAV_DATA_FRAME_BITS> bits, const std::vector<std::pair<int, int>> parameter);

    std::string omega0_1;
    //std::string omega0_2;
    //bool omega_flag;
};

#endif /* GNSS_SDR_GALILEO_FNAV_MESSAGE_H_ */
