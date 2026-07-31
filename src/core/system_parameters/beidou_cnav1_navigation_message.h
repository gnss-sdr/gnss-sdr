/*!
 * \file beidou_cnav1_navigation_message.h
 * \brief B-CNAV1 navigation message parser (§6.2)
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef GNSS_SDR_BEIDOU_CNAV1_NAVIGATION_MESSAGE_H
#define GNSS_SDR_BEIDOU_CNAV1_NAVIGATION_MESSAGE_H

#include "beidou_cnav1_ephemeris.h"
#include "beidou_cnav1_iono.h"
#include "beidou_cnav1_utc_model.h"
#include <array>

struct Bds3_B1c_PageCommon
{
    int32_t page_id{};
    int32_t sismai{};
    int32_t hs{};
    bool dif{};
    bool sif{};
    bool aif{};
};

struct Bds3_B1c_AlmanacReduced
{
    int32_t prn{};
    int32_t sat_type{};
    double delta_a_m{};
    double omega0_rad{};
    double phi0_rad{};
    int32_t health{};
};

struct Bds3_B1c_AlmanacMedium
{
    int32_t prn{};
    int32_t sat_type{};
    int32_t wna{};
    int32_t toa_s{};
    double eccentricity{};
    double delta_i_rad{};
    double sqrt_a_m_sqrt{};
    double omega0_rad{};
    double omega_dot_rad_s{};
    double omega_rad{};
    double m0_rad{};
    double af0_s{};
    double af1_s_s{};
    int32_t health{};
};

struct Bds3_B1c_Eop
{
    int32_t t_eop_s{};
    double pm_x_arcsec{};
    double pm_x_dot_arcsec_day{};
    double pm_y_arcsec{};
    double pm_y_dot_arcsec_day{};
    double delta_ut1_s{};
    double delta_ut1_dot_s_day{};
};

struct Bds3_B1c_Bgto
{
    int32_t gnss_id{};
    int32_t wn0_bgto{};
    int32_t t0_bgto_s{};
    double a0_bgto_s{};
    double a1_bgto_s_s{};
    double a2_bgto_s_s2{};
};

struct Bds3_B1c_PageData
{
    Bds3_B1c_PageCommon common{};
    int32_t sisaioc{};
    int32_t sisaioe{};
    int32_t wna{};
    int32_t toa_s{};
    std::array<Bds3_B1c_AlmanacReduced, 4> reduced_almanac{};
    Bds3_B1c_Eop eop{};
    Bds3_B1c_Bgto bgto{};
    Bds3_B1c_AlmanacMedium medium_almanac{};
};

struct Beidou_Cnav1_PageData_Message
{
    int32_t PRN{};
    Bds3_B1c_PageData page_data{};
};

class Beidou_Cnav1_Navigation_Message
{
public:
    bool decode_frame(const float* symbols, int32_t num_symbols, int32_t expected_prn = 0, int32_t* fail_stage = nullptr);
    bool decode_frame_symbols(const float* symbols, int32_t num_symbols, int32_t expected_prn = 0, int32_t* fail_stage = nullptr);
    bool probe_subframe1_prn(const float* symbols, int32_t num_symbols, int32_t expected_prn) const;
    bool have_new_ephemeris() const;
    bool have_new_iono() const;
    bool have_new_utc_model() const;
    bool have_new_page_data() const;
    void clear_flags();
    const Beidou_Cnav1_Ephemeris& get_ephemeris() const;
    const Beidou_Cnav1_Iono& get_iono() const;
    const Beidou_Cnav1_Utc_Model& get_utc_model() const;
    const Bds3_B1c_PageData& get_page_data() const;
    double get_tow_s() const;

private:
    Beidou_Cnav1_Ephemeris ephemeris_{};
    Beidou_Cnav1_Iono iono_{};
    Beidou_Cnav1_Utc_Model utc_model_{};
    bool flag_new_ephemeris_{false};
    bool flag_new_iono_{false};
    bool flag_new_utc_{false};
    bool flag_new_page_data_{false};
    double tow_s_{0.0};
    Bds3_B1c_PageData page_data_{};
};

#endif
