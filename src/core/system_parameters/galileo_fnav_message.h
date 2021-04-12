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
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GALILEO_FNAV_MESSAGE_H
#define GNSS_SDR_GALILEO_FNAV_MESSAGE_H


#include "Galileo_E5a.h"
#include "Galileo_FNAV.h"
#include "galileo_almanac_helper.h"
#include "galileo_ephemeris.h"
#include "galileo_iono.h"
#include "galileo_utc_model.h"
#include <bitset>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class handles the Galileo F/NAV Data message, as described in the
 * Galileo Open Service Signal in Space Interface Control Document (OS SIS ICD), Issue 2.0 (Jan. 2021).
 * See https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_OS_SIS_ICD_v2.0.pdf
 */
class Galileo_Fnav_Message
{
public:
    Galileo_Fnav_Message() = default;

    void split_page(const std::string& page_string);
    bool have_new_ephemeris();
    bool have_new_iono_and_GST();
    bool have_new_utc_model();
    bool have_new_almanac();
    Galileo_Ephemeris get_ephemeris() const;
    Galileo_Iono get_iono() const;
    Galileo_Utc_Model get_utc_model() const;
    Galileo_Almanac_Helper get_almanac() const;

    inline int32_t get_TOW1() const
    {
        return FNAV_TOW_1;
    }

    inline int32_t get_TOW2() const
    {
        return FNAV_TOW_2;
    }

    inline int32_t get_TOW3() const
    {
        return FNAV_TOW_3;
    }

    inline int32_t get_TOW4() const
    {
        return FNAV_TOW_4;
    }

    inline bool get_flag_CRC_test() const
    {
        return flag_CRC_test;
    }

    inline bool get_flag_TOW_set() const
    {
        return flag_TOW_set;
    }

    inline void set_flag_TOW_set(bool flag_tow)
    {
        flag_TOW_set = flag_tow;
    }

    inline bool is_TOW1_set() const
    {
        return flag_TOW_1;
    }

    inline void set_TOW1_flag(bool flag_tow1)
    {
        flag_TOW_1 = flag_tow1;
    }

    inline bool is_TOW2_set() const
    {
        return flag_TOW_2;
    }

    inline void set_TOW2_flag(bool flag_tow2)
    {
        flag_TOW_2 = flag_tow2;
    }

    inline bool is_TOW3_set() const
    {
        return flag_TOW_3;
    }

    inline void set_TOW3_flag(bool flag_tow3)
    {
        flag_TOW_3 = flag_tow3;
    }

    inline bool is_TOW4_set() const
    {
        return flag_TOW_4;
    }

    inline void set_TOW4_flag(bool flag_tow4)
    {
        flag_TOW_4 = flag_tow4;
    }

private:
    bool _CRC_test(const std::bitset<GALILEO_FNAV_DATA_FRAME_BITS>& bits, uint32_t checksum) const;
    void decode_page(const std::string& data);
    uint64_t read_navigation_unsigned(const std::bitset<GALILEO_FNAV_DATA_FRAME_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const;
    int64_t read_navigation_signed(const std::bitset<GALILEO_FNAV_DATA_FRAME_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const;

    std::string omega0_1{};
    // std::string omega0_2{};
    // bool omega_flag{};

    int32_t IOD_ephemeris{};

    int32_t page_type{};
    // WORD 1 SVID, Clock correction, SISA, Ionospheric correction, BGD, GST, Signal
    // health and Data validity status
    int32_t FNAV_SV_ID_PRN_1{};
    int32_t FNAV_IODnav_1{};
    int32_t FNAV_t0c_1{};
    double FNAV_af0_1{};
    double FNAV_af1_1{};
    double FNAV_af2_1{};
    int32_t FNAV_SISA_1{};
    double FNAV_ai0_1{};
    double FNAV_ai1_1{};
    double FNAV_ai2_1{};
    bool FNAV_region1_1{};
    bool FNAV_region2_1{};
    bool FNAV_region3_1{};
    bool FNAV_region4_1{};
    bool FNAV_region5_1{};
    double FNAV_BGD_1{};
    int32_t FNAV_E5ahs_1{};
    int32_t FNAV_WN_1{};
    int32_t FNAV_TOW_1{};
    bool FNAV_E5advs_1{};

    // WORD 2 Ephemeris (1/3) and GST
    int32_t FNAV_IODnav_2{};
    double FNAV_M0_2{};
    double FNAV_omegadot_2{};
    double FNAV_e_2{};
    double FNAV_a12_2{};
    double FNAV_omega0_2{};
    double FNAV_idot_2{};
    int32_t FNAV_WN_2{};
    int32_t FNAV_TOW_2{};

    // WORD 3 Ephemeris (2/3) and GST
    int32_t FNAV_IODnav_3{};
    double FNAV_i0_3{};
    double FNAV_w_3{};
    double FNAV_deltan_3{};
    double FNAV_Cuc_3{};
    double FNAV_Cus_3{};
    double FNAV_Crc_3{};
    double FNAV_Crs_3{};
    int32_t FNAV_t0e_3{};
    int32_t FNAV_WN_3{};
    int32_t FNAV_TOW_3{};

    // WORD 4 Ephemeris (3/3), GST-UTC conversion, GST-GPS conversion and TOW.
    // Note that the clock is repeated in this page type
    int32_t FNAV_IODnav_4{};
    double FNAV_Cic_4{};
    double FNAV_Cis_4{};
    double FNAV_A0_4{};
    double FNAV_A1_4{};
    int32_t FNAV_deltatls_4{};
    int32_t FNAV_t0t_4{};
    int32_t FNAV_WNot_4{};
    int32_t FNAV_WNlsf_4{};
    int32_t FNAV_DN_4{};
    int32_t FNAV_deltatlsf_4{};
    int32_t FNAV_t0g_4{};
    double FNAV_A0g_4{};
    double FNAV_A1g_4{};
    int32_t FNAV_WN0g_4{};
    int32_t FNAV_TOW_4{};

    // WORD 5 Almanac (SVID1 and SVID2(1/2)), Week Number and almanac reference time
    int32_t FNAV_IODa_5{};
    int32_t FNAV_WNa_5{};
    int32_t FNAV_t0a_5{};
    int32_t FNAV_SVID1_5{};
    double FNAV_Deltaa12_1_5{};
    double FNAV_e_1_5{};
    double FNAV_w_1_5{};
    double FNAV_deltai_1_5{};
    double FNAV_Omega0_1_5{};
    double FNAV_Omegadot_1_5{};
    double FNAV_M0_1_5{};
    double FNAV_af0_1_5{};
    double FNAV_af1_1_5{};
    uint32_t FNAV_E5ahs_1_5{};
    int32_t FNAV_SVID2_5{};
    double FNAV_Deltaa12_2_5{};
    double FNAV_e_2_5{};
    double FNAV_w_2_5{};
    double FNAV_deltai_2_5{};

    // WORD 6 Almanac (SVID2(2/2) and SVID3)
    int32_t FNAV_IODa_6{};
    double FNAV_Omega0_2_6{};
    double FNAV_Omegadot_2_6{};
    double FNAV_M0_2_6{};
    double FNAV_af0_2_6{};
    double FNAV_af1_2_6{};
    int32_t FNAV_E5ahs_2_6{};
    int32_t FNAV_SVID3_6{};
    double FNAV_Deltaa12_3_6{};
    double FNAV_e_3_6{};
    double FNAV_w_3_6{};
    double FNAV_deltai_3_6{};
    double FNAV_Omega0_3_6{};
    double FNAV_Omegadot_3_6{};
    double FNAV_M0_3_6{};
    double FNAV_af0_3_6{};
    double FNAV_af1_3_6{};
    int32_t FNAV_E5ahs_3_6{};

    bool flag_CRC_test{};
    bool flag_all_ephemeris{};  // Flag indicating that all words containing ephemeris have been received
    bool flag_ephemeris_1{};    // Flag indicating that ephemeris 1/3 (word 2) have been received
    bool flag_ephemeris_2{};    // Flag indicating that ephemeris 2/3 (word 3) have been received
    bool flag_ephemeris_3{};    // Flag indicating that ephemeris 3/3 (word 4) have been received

    bool flag_iono_and_GST{};  // Flag indicating that ionospheric and GST parameters (word 1) have been received
    bool flag_TOW_1{};
    bool flag_TOW_2{};
    bool flag_TOW_3{};
    bool flag_TOW_4{};
    bool flag_TOW_set{};    // it is true when page 1,2,3 or 4 arrives
    bool flag_utc_model{};  // Flag indicating that utc model parameters (word 4) have been received

    bool flag_all_almanac{};  // Flag indicating that all Almanac data have been received
    bool flag_almanac_1{};    // Flag indicating that almanac 1/2 (word 5) have been received
    bool flag_almanac_2{};    // Flag indicating that almanac 2/2 (word 6) have been received
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_FNAV_MESSAGE_H
