/*!
 * \file galileo_inav_message.h
 * \brief  Implementation of a Galileo I/NAV Data message
 *         as described in Galileo OS SIS ICD Issue 2.0 (Jan. 2021)
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GALILEO_INAV_MESSAGE_H
#define GNSS_SDR_GALILEO_INAV_MESSAGE_H

#include "Galileo_INAV.h"
#include "galileo_almanac_helper.h"
#include "galileo_ephemeris.h"
#include "galileo_iono.h"
#include "galileo_utc_model.h"
#include "gnss_sdr_make_unique.h"  // for std::unique_ptr in C++11
#include <bitset>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

class ReedSolomon;  // Forward declaration of the ReedSolomon class

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class handles the Galileo I/NAV Data message, as described in the
 * Galileo Open Service Signal in Space Interface Control Document (OS SIS ICD), Issue 2.0 (Jan. 2021).
 * See https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_OS_SIS_ICD_v2.0.pdf
 */
class Galileo_Inav_Message
{
public:
    Galileo_Inav_Message();

    ~Galileo_Inav_Message();

    /*
     * \brief Takes in input a page (Odd or Even) of 120 bit, split it according ICD 4.3.2.3 and join Data_k with Data_j
     */
    void split_page(std::string page_string, int32_t flag_even_word);

    /*
     * \brief Takes in input Data_jk (128 bit) and split it in ephemeris parameters according ICD 4.3.5
     *
     * Takes in input Data_jk (128 bit) and split it in ephemeris parameters according ICD 4.3.5
     */
    int32_t page_jk_decoder(const char* data_jk);

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
     * \brief Returns true if new Reduced CED parameters have arrived. The flag is set to false when the function is executed
     */
    bool have_new_reduced_ced();

    /*
     * \brief Returns a Galileo_Ephemeris object filled with the latest navigation data received
     */
    Galileo_Ephemeris get_ephemeris() const;

    /*
     * \brief Returns a Galileo_Iono object filled with the latest navigation data received
     */
    Galileo_Iono get_iono() const;

    /*
     * \brief Returns a Galileo_Utc_Model object filled with the latest navigation data received
     */
    Galileo_Utc_Model get_utc_model() const;

    /*
     * \brief Returns a Galileo_Almanac_Helper object filled with the latest navigation data received
     */
    Galileo_Almanac_Helper get_almanac() const;

    /*
     * \brief Returns a Galileo_Ephemeris object filled with the latest reduced CED received
     */
    Galileo_Ephemeris get_reduced_ced() const;

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

    inline int32_t get_Galileo_week() const
    {
        return WN_0;
    }

    inline int32_t get_TOW5() const
    {
        return TOW_5;
    }

    inline int32_t get_TOW6() const
    {
        return TOW_6;
    }

    inline bool is_TOW5_set() const
    {
        return flag_TOW_5;
    }

    inline void set_TOW5_flag(bool flag_tow5)
    {
        flag_TOW_5 = flag_tow5;
    }

    inline bool is_TOW6_set() const
    {
        return flag_TOW_6;
    }

    inline void set_TOW6_flag(bool flag_tow6)
    {
        flag_TOW_6 = flag_tow6;
    }

    inline int32_t get_TOW0() const
    {
        return TOW_0;
    }

    inline bool is_TOW0_set() const
    {
        return flag_TOW_0;
    }

    inline void set_TOW0_flag(bool flag_tow0)
    {
        flag_TOW_0 = flag_tow0;
    }

    inline bool get_flag_GGTO() const
    {
        return (flag_GGTO_1 == true and flag_GGTO_2 == true and flag_GGTO_3 == true and flag_GGTO_4 == true);
    }

    inline double get_A0G() const
    {
        return A_0G_10;
    }

    inline double get_A1G() const
    {
        return A_1G_10;
    }

    inline double get_t0G() const
    {
        return t_0G_10;
    }

    inline double get_WN0G() const
    {
        return WN_0G_10;
    }

    /*
     * \brief Initialize PRN field so we do not need to wait for page 4.
     */
    inline void init_PRN(uint32_t prn)
    {
        SV_ID_PRN_4 = prn;
    }

    /*
     * \brief Enable Reed-Solomon in Galileo E1B
     */
    inline void enable_reed_solomon()
    {
        enable_rs = true;
    }

private:
    bool CRC_test(const std::bitset<GALILEO_DATA_FRAME_BITS>& bits, uint32_t checksum) const;
    bool read_navigation_bool(const std::bitset<GALILEO_DATA_JK_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const;
    uint64_t read_navigation_unsigned(const std::bitset<GALILEO_DATA_JK_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const;
    uint64_t read_page_type_unsigned(const std::bitset<GALILEO_PAGE_TYPE_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const;
    int64_t read_navigation_signed(const std::bitset<GALILEO_DATA_JK_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const;
    uint8_t read_octet_unsigned(const std::bitset<GALILEO_DATA_JK_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const;
    void read_page_1(const std::bitset<GALILEO_DATA_JK_BITS>& data_bits);
    void read_page_2(const std::bitset<GALILEO_DATA_JK_BITS>& data_bits);
    void read_page_3(const std::bitset<GALILEO_DATA_JK_BITS>& data_bits);
    void read_page_4(const std::bitset<GALILEO_DATA_JK_BITS>& data_bits);
    std::bitset<GALILEO_DATA_JK_BITS> regenerate_page_1(const std::vector<uint8_t>& decoded) const;
    std::bitset<GALILEO_DATA_JK_BITS> regenerate_page_2(const std::vector<uint8_t>& decoded) const;
    std::bitset<GALILEO_DATA_JK_BITS> regenerate_page_3(const std::vector<uint8_t>& decoded) const;
    std::bitset<GALILEO_DATA_JK_BITS> regenerate_page_4(const std::vector<uint8_t>& decoded) const;

    std::string page_Even{};

    int32_t Page_type_time_stamp{};
    int32_t IOD_ephemeris{};

    // Word type 1: Ephemeris (1/4)
    int32_t IOD_nav_1{};  // IOD_nav page 1
    int32_t t0e_1{};      // Ephemeris reference time [s]
    double M0_1{};        // Mean anomaly at reference time [semi-circles]
    double e_1{};         // Eccentricity
    double A_1{};         // Square root of the semi-major axis [meters^1/2]

    // Word type 2: Ephemeris (2/4)
    int32_t IOD_nav_2{};  // IOD_nav page 2
    double OMEGA_0_2{};   // Longitude of ascending node of orbital plane at weekly epoch [semi-circles]
    double i_0_2{};       // Inclination angle at reference time  [semi-circles]
    double omega_2{};     // Argument of perigee [semi-circles]
    double iDot_2{};      // Rate of inclination angle [semi-circles/sec]

    // Word type 3: Ephemeris (3/4) and SISA
    int32_t IOD_nav_3{};   //
    double OMEGA_dot_3{};  // Rate of right ascension [semi-circles/sec]
    double delta_n_3{};    // Mean motion difference from computed value  [semi-circles/sec]
    double C_uc_3{};       // Amplitude of the cosine harmonic correction term to the argument of latitude [radians]
    double C_us_3{};       // Amplitude of the sine harmonic correction term to the argument of latitude [radians]
    double C_rc_3{};       // Amplitude of the cosine harmonic correction term to the orbit radius [meters]
    double C_rs_3{};       // Amplitude of the sine harmonic correction term to the orbit radius [meters]
    int32_t SISA_3{};

    // Word type 4: Ephemeris (4/4) and Clock correction parameters*/
    int32_t IOD_nav_4{};    //
    int32_t SV_ID_PRN_4{};  //
    double C_ic_4{};        // Amplitude of the cosine harmonic correction term to the angle of inclination [radians]
    double C_is_4{};        // Amplitude of the sine harmonic correction term to the angle of inclination [radians]

    // Clock correction parameters
    int32_t t0c_4{};  // Clock correction data reference Time of Week [sec]
    double af0_4{};   // SV clock bias correction coefficient [s]
    double af1_4{};   // SV clock drift correction coefficient [s/s]
    double af2_4{};   // clock drift rate correction coefficient [s/s^2]
    double spare_4{};

    //  Word type 5: Ionospheric correction, BGD, signal health and data validity status and GST*/
    // Ionospheric correction
    double ai0_5{};  // Effective Ionisation Level 1st order parameter [sfu]
    double ai1_5{};  // Effective Ionisation Level 2st order parameter [sfu/degree]
    double ai2_5{};  // Effective Ionisation Level 3st order parameter [sfu/degree]

    // Ionospheric disturbance flag
    bool Region1_flag_5{};  // Ionospheric Disturbance Flag for region 1
    bool Region2_flag_5{};  // Ionospheric Disturbance Flag for region 2
    bool Region3_flag_5{};  // Ionospheric Disturbance Flag for region 3
    bool Region4_flag_5{};  // Ionospheric Disturbance Flag for region 4
    bool Region5_flag_5{};  // Ionospheric Disturbance Flag for region 5
    double BGD_E1E5a_5{};   // E1-E5a Broadcast Group Delay [s]
    double BGD_E1E5b_5{};   // E1-E5b Broadcast Group Delay [s]

    int32_t E5b_HS_5{};  // E5b Signal Health Status
    int32_t E1B_HS_5{};  // E1B Signal Health Status
    bool E5b_DVS_5{};    // E5b Data Validity Status
    bool E1B_DVS_5{};    // E1B Data Validity Status

    // GST
    int32_t WN_5{};
    int32_t TOW_5{};
    double spare_5{};

    // Word type 6: GST-UTC conversion parameters
    double A0_6{};
    double A1_6{};
    int32_t Delta_tLS_6{};
    int32_t t0t_6{};
    int32_t WNot_6{};
    int32_t WN_LSF_6{};
    int32_t DN_6{};
    int32_t Delta_tLSF_6{};
    int32_t TOW_6{};

    // Word type 7: Almanac for SVID1 (1/2), almanac reference time and almanac reference week number
    int32_t IOD_a_7{};
    int32_t WN_a_7{};
    int32_t t0a_7{};
    int32_t SVID1_7{};
    double DELTA_A_7{};
    double e_7{};
    double omega_7{};
    double delta_i_7{};
    double Omega0_7{};
    double Omega_dot_7{};
    double M0_7{};

    // Word type 8: Almanac for SVID1 (2/2) and SVID2 (1/2)
    int32_t IOD_a_8{};
    double af0_8{};
    double af1_8{};
    int32_t E5b_HS_8{};
    int32_t E1B_HS_8{};
    int32_t SVID2_8{};
    double DELTA_A_8{};
    double e_8{};
    double omega_8{};
    double delta_i_8{};
    double Omega0_8{};
    double Omega_dot_8{};

    // Word type 9: Almanac for SVID2 (2/2) and SVID3 (1/2)
    int32_t IOD_a_9{};
    int32_t WN_a_9{};
    int32_t t0a_9{};
    double M0_9{};
    double af0_9{};
    double af1_9{};
    int32_t E5b_HS_9{};
    int32_t E1B_HS_9{};
    int32_t SVID3_9{};
    double DELTA_A_9{};
    double e_9{};
    double omega_9{};
    double delta_i_9{};

    //  Word type 10: Almanac for SVID3 (2/2) and GST-GPS conversion parameters
    int32_t IOD_a_10{};
    double Omega0_10{};
    double Omega_dot_10{};
    double M0_10{};
    double af0_10{};
    double af1_10{};
    int32_t E5b_HS_10{};
    int32_t E1B_HS_10{};

    // GST-GPS conversion
    double A_0G_10{};    // Constant term of the offset Delta t systems
    double A_1G_10{};    // Rate of change of the offset Delta t systems
    int32_t t_0G_10{};   // Reference time for Galileo/GPS Time Offset (GGTO) data
    int32_t WN_0G_10{};  // Week Number of Galileo/GPS Time Offset (GGTO) reference

    // Word type 0: I/NAV Spare Word
    int32_t Time_0{};
    int32_t WN_0{};
    int32_t TOW_0{};

    // Word type 16: Reduced Clock and Ephemeris Data (CED) parameters
    double ced_DeltaAred{};
    double ced_exred{};
    double ced_eyred{};
    double ced_Deltai0red{};
    double ced_Omega0red{};
    double ced_lambda0red{};
    double ced_af0red{};
    double ced_af1red{};

    double Galileo_satClkDrift{};

    int32_t current_IODnav{};

    std::vector<uint8_t> rs_buffer;   // Reed-Solomon buffer
    std::unique_ptr<ReedSolomon> rs;  // The Reed-Solomon decoder
    std::vector<int> inav_rs_pages;   // Pages 1,2,3,4,17,18,19,20. Holds 1 if the page has arrived, 0 otherwise.

    uint8_t IODnav_LSB17{};
    uint8_t IODnav_LSB18{};
    uint8_t IODnav_LSB19{};
    uint8_t IODnav_LSB20{};

    bool flag_CRC_test{};
    bool flag_all_ephemeris{};  // Flag indicating that all words containing ephemeris have been received
    bool flag_ephemeris_1{};    // Flag indicating that ephemeris 1/4 (word 1) have been received
    bool flag_ephemeris_2{};    // Flag indicating that ephemeris 2/4 (word 2) have been received
    bool flag_ephemeris_3{};    // Flag indicating that ephemeris 3/4 (word 3) have been received
    bool flag_ephemeris_4{};    // Flag indicating that ephemeris 4/4 (word 4) have been received

    bool flag_iono_and_GST{};  // Flag indicating that ionospheric and GST parameters (word 5) have been received
    bool flag_TOW_5{};
    bool flag_TOW_6{};
    bool flag_TOW_0{};
    bool flag_TOW_set{};    // it is true when page 5 or page 6 arrives
    bool flag_utc_model{};  // Flag indicating that utc model parameters (word 6) have been received

    bool flag_all_almanac{};  // Flag indicating that all Almanac data have been received
    bool flag_almanac_1{};    // Flag indicating that almanac 1/4 (word 7) have been received
    bool flag_almanac_2{};    // Flag indicating that almanac 2/4 (word 8) have been received
    bool flag_almanac_3{};    // Flag indicating that almanac 3/4 (word 9) have been received
    bool flag_almanac_4{};    // Flag indicating that almanac 4/4 (word 10) have been received

    bool flag_GGTO_1{};
    bool flag_GGTO_2{};
    bool flag_GGTO_3{};
    bool flag_GGTO_4{};

    bool flag_CED{};
    bool enable_rs{};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_INAV_MESSAGE_H
