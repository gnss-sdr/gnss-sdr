/*!
 * \file gps_cnav2_navigation_message.cc
 * \brief Implementation of a GPS CNAV-2 (L1C) navigation message decoder
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 * \author José Antonio Mayo, 2026. contact(at)tatjam.eu
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "gps_cnav2_navigation_message.h"
#include "GPS_CNAV.h"
#include "GPS_CNAV2.h"
#include "GPS_L1C.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace
{
using Field = std::vector<std::pair<int32_t, int32_t>>;

uint64_t read_unsigned(const std::vector<uint8_t>& bits, const Field& field)
{
    uint64_t value = 0ULL;
    for (const auto& f : field)
        {
            for (int32_t j = 0; j < f.second; j++)
                {
                    value = (value << 1U) | static_cast<uint64_t>(bits[f.first - 1 + j] & 1U);
                }
        }
    return value;
}


int64_t read_signed(const std::vector<uint8_t>& bits, const Field& field)
{
    int64_t value = (bits[field[0].first - 1] & 1U) ? -1LL : 0LL;
    for (const auto& f : field)
        {
            for (int32_t j = 0; j < f.second; j++)
                {
                    value = static_cast<int64_t>(static_cast<uint64_t>(value) << 1U) | static_cast<int64_t>(bits[f.first - 1 + j] & 1U);
                }
        }
    return value;
}


double read_group_delay(const std::vector<uint8_t>& bits, const Field& field, double lsb)
{
    const int64_t value = read_signed(bits, field);
    if (value == CNAV2_GROUP_DELAY_NOT_AVAILABLE)
        {
            return 0.0;
        }
    return static_cast<double>(value) * lsb;
}


// Codewords of the 256 TOI hypotheses with MSB = 0, as +1 / -1 for "0" / "1"
const std::vector<std::array<float, GPS_L1C_TOI_SYMBOLS>>& toi_codebook()
{
    static const std::vector<std::array<float, GPS_L1C_TOI_SYMBOLS>> codebook = []() {
        std::vector<std::array<float, GPS_L1C_TOI_SYMBOLS>> words(256);
        for (int32_t lsbs = 0; lsbs < 256; lsbs++)
            {
                const auto bits = Gps_CNAV2_Navigation_Message::encode_toi(lsbs);
                for (int32_t i = 0; i < GPS_L1C_TOI_SYMBOLS; i++)
                    {
                        words[lsbs][i] = bits[i] ? -1.0F : 1.0F;
                    }
            }
        return words;
    }();
    return codebook;
}


std::string bits_to_string(const std::vector<uint8_t>& bits, size_t nbits)
{
    std::string s(nbits, '0');
    for (size_t i = 0; i < nbits; i++)
        {
            s[i] = bits[i] ? '1' : '0';
        }
    return s;
}


// A subframe passes the CRC check without carrying navigation data when it is
// the default message, which replaces it in the event of message generation
// failure (alternating ones and zeros beginning with one, IS-GPS-800J 3.5.1),
// or when it is all zeros, a codeword with a null CRC
bool carries_no_data(const std::vector<uint8_t>& bits, int32_t data_bits)
{
    bool alternating = true;
    bool zeros = true;
    for (int32_t i = 0; i < data_bits && (alternating || zeros); i++)
        {
            alternating = alternating && (bits[i] == ((i % 2 == 0) ? 1 : 0));
            zeros = zeros && (bits[i] == 0);
        }
    return alternating || zeros;
}


/*
 * Log-likelihood ratios of BPSK symbols with unknown amplitude and noise
 * variance, both estimated with the second and fourth moments of the frame.
 */
std::vector<float> symbols_to_llr(const float* symbols, size_t n)
{
    double m2 = 0.0;
    double m4 = 0.0;
    double m1 = 0.0;
    for (size_t i = 0; i < n; i++)
        {
            const double y2 = static_cast<double>(symbols[i]) * static_cast<double>(symbols[i]);
            m1 += std::abs(static_cast<double>(symbols[i]));
            m2 += y2;
            m4 += y2 * y2;
        }
    m1 /= static_cast<double>(n);
    m2 /= static_cast<double>(n);
    m4 /= static_cast<double>(n);
    const double radicand = 2.0 * m2 * m2 - m4;
    double amplitude2 = radicand > 0.0 ? std::sqrt(radicand) : m1 * m1;
    double noise_var = m2 - amplitude2;
    if (noise_var <= 1e-3 * m2)
        {
            // very high SNR or degenerate estimate: bound the LLR magnitude
            noise_var = 1e-3 * m2;
            amplitude2 = m2 - noise_var;
        }
    const double scale = 2.0 * std::sqrt(std::max(amplitude2, 0.0)) / noise_var;
    std::vector<float> llr(n);
    for (size_t i = 0; i < n; i++)
        {
            llr[i] = static_cast<float>(scale * static_cast<double>(symbols[i]));
        }
    return llr;
}
}  // namespace


Gps_CNAV2_Navigation_Message::Gps_CNAV2_Navigation_Message()
    : d_sf2_code(2),
      d_sf3_code(3)
{
}


void Gps_CNAV2_Navigation_Message::set_prn(uint32_t prn)
{
    d_prn = prn;
    d_ephemeris.PRN = prn;
    d_eop.PRN = prn;
}


void Gps_CNAV2_Navigation_Message::reset()
{
    const uint32_t prn = d_prn;
    d_ephemeris = Gps_CNAV2_Ephemeris();
    d_iono = Gps_CNAV_Iono();
    d_utc_model = Gps_CNAV_Utc_Model();
    d_eop = Gps_CNAV_Eop();
    d_last_published_sf2.clear();
    d_sf2_bits.clear();
    d_sf3_bits.clear();
    d_toi = -1;
    d_itow = -1;
    d_sf3_page = -1;
    d_toi_quality = 0.0F;
    d_sf2_ok = false;
    d_sf3_ok = false;
    d_sf2_no_data = false;
    d_sf3_no_data = false;
    d_new_ephemeris = false;
    d_new_iono = false;
    d_new_utc = false;
    d_new_eop = false;
    set_prn(prn);
}


std::vector<uint8_t> Gps_CNAV2_Navigation_Message::encode_toi(int32_t toi)
{
    // BCH(51,8) generator of IS-GPS-800J Figure 3.2-4: stages 1 to 8, output
    // from stage 8, feedback 1 + X + X^4 + X^5 + X^6 + X^7 + X^8. The eight LSBs
    // of TOI are the initial conditions, MSB shifted in first (into stage 8).
    const auto msb = static_cast<uint8_t>((toi >> 8) & 1);
    std::array<uint8_t, 8> stage{};  // stage[i] is stage i + 1
    for (int32_t i = 0; i < 8; i++)
        {
            stage[i] = static_cast<uint8_t>((toi >> i) & 1);
        }
    std::vector<uint8_t> symbols(GPS_L1C_TOI_SYMBOLS);
    symbols[0] = msb;
    for (int32_t i = 1; i < GPS_L1C_TOI_SYMBOLS; i++)
        {
            symbols[i] = stage[7] ^ msb;
            const auto feedback = static_cast<uint8_t>(stage[0] ^ stage[3] ^ stage[4] ^ stage[5] ^ stage[6] ^ stage[7]);
            for (int32_t j = 7; j > 0; j--)
                {
                    stage[j] = stage[j - 1];
                }
            stage[0] = feedback;
        }
    return symbols;
}


int32_t Gps_CNAV2_Navigation_Message::decode_toi(const float* symbols, float& quality)
{
    const auto& codebook = toi_codebook();
    float magnitude = 0.0F;
    for (int32_t i = 0; i < GPS_L1C_TOI_SYMBOLS; i++)
        {
            magnitude += std::abs(symbols[i]);
        }
    float best = 0.0F;
    int32_t best_lsbs = 0;
    for (int32_t h = 0; h < 256; h++)
        {
            float corr = 0.0F;
            for (int32_t i = 0; i < GPS_L1C_TOI_SYMBOLS; i++)
                {
                    corr += symbols[i] * codebook[h][i];
                }
            if (std::abs(corr) > std::abs(best))
                {
                    best = corr;
                    best_lsbs = h;
                }
        }
    quality = magnitude > 0.0F ? std::abs(best) / magnitude : 0.0F;
    // A transmitted MSB = 1 inverts the whole word
    return best_lsbs | (best < 0.0F ? 256 : 0);
}


void Gps_CNAV2_Navigation_Message::interleave(const uint8_t* in, uint8_t* out)
{
    // Written row by row into 38 x 46 cells, read out column by column
    int32_t k = 0;
    for (int32_t col = 0; col < GPS_L1C_INTERLEAVER_COLS; col++)
        {
            for (int32_t row = 0; row < GPS_L1C_INTERLEAVER_ROWS; row++)
                {
                    out[k++] = in[row * GPS_L1C_INTERLEAVER_COLS + col];
                }
        }
}


void Gps_CNAV2_Navigation_Message::deinterleave(const float* in, float* out)
{
    int32_t k = 0;
    for (int32_t col = 0; col < GPS_L1C_INTERLEAVER_COLS; col++)
        {
            for (int32_t row = 0; row < GPS_L1C_INTERLEAVER_ROWS; row++)
                {
                    out[row * GPS_L1C_INTERLEAVER_COLS + col] = in[k++];
                }
        }
}


uint32_t Gps_CNAV2_Navigation_Message::crc24q(const std::vector<uint8_t>& bits, size_t nbits)
{
    // g(X) = X^24 + X^23 + X^18 + X^17 + X^14 + X^11 + X^10 + X^7 + X^6 + X^5 + X^4 + X^3 + X + 1
    constexpr uint32_t poly = 0x864CFBU;
    uint32_t crc = 0U;
    for (size_t i = 0; i < nbits; i++)
        {
            const uint32_t msb = (crc >> 23U) & 1U;
            crc = (crc << 1U) & 0xFFFFFFU;
            if ((msb ^ (bits[i] & 1U)) != 0U)
                {
                    crc ^= poly;
                }
        }
    return crc;
}


bool Gps_CNAV2_Navigation_Message::decode_frame(const float* symbols)
{
    d_sf2_ok = false;
    d_sf3_ok = false;
    d_sf2_no_data = false;
    d_sf3_no_data = false;
    d_sf3_page = -1;
    d_toi = decode_toi(symbols, d_toi_quality);

    constexpr int32_t coded = GPS_L1C_SF2_SYMBOLS + GPS_L1C_SF3_SYMBOLS;
    std::vector<float> deinterleaved(coded);
    deinterleave(symbols + GPS_L1C_TOI_SYMBOLS, deinterleaved.data());
    const std::vector<float> llr = symbols_to_llr(deinterleaved.data(), deinterleaved.size());

    std::vector<uint8_t> codeword;
    const std::vector<float> sf2_llr(llr.cbegin(), llr.cbegin() + GPS_L1C_SF2_SYMBOLS);
    if (d_sf2_code.decode(sf2_llr, codeword) && crc24q(codeword, GPS_L1C_SF2_BITS) == 0U)
        {
            codeword.resize(GPS_L1C_SF2_BITS);
            d_sf2_ok = true;
            d_sf2_bits = bits_to_string(codeword, GPS_L1C_SF2_BITS);
            d_sf2_no_data = carries_no_data(codeword, GPS_L1C_SF2_BITS - GPS_L1C_CRC_BITS);
            if (!d_sf2_no_data)
                {
                    decode_sf2(codeword);
                }
        }
    const std::vector<float> sf3_llr(llr.cbegin() + GPS_L1C_SF2_SYMBOLS, llr.cend());
    if (d_sf3_code.decode(sf3_llr, codeword) && crc24q(codeword, GPS_L1C_SF3_BITS) == 0U)
        {
            codeword.resize(GPS_L1C_SF3_BITS);
            d_sf3_ok = true;
            d_sf3_bits = bits_to_string(codeword, GPS_L1C_SF3_BITS);
            d_sf3_no_data = carries_no_data(codeword, GPS_L1C_SF3_BITS - GPS_L1C_CRC_BITS);
            if (!d_sf3_no_data)
                {
                    decode_sf3(codeword);
                }
        }
    return d_sf2_ok || d_sf3_ok;
}


bool Gps_CNAV2_Navigation_Message::is_frame_candidate(const float* symbols) const
{
    float quality = 0.0F;
    decode_toi(symbols, quality);
    if (quality < GPS_L1C_TOI_MIN_QUALITY)
        {
            return false;
        }
    constexpr int32_t coded = GPS_L1C_SF2_SYMBOLS + GPS_L1C_SF3_SYMBOLS;
    std::vector<float> deinterleaved(coded);
    deinterleave(symbols + GPS_L1C_TOI_SYMBOLS, deinterleaved.data());
    std::vector<uint8_t> hard(GPS_L1C_SF2_SYMBOLS);
    for (int32_t i = 0; i < GPS_L1C_SF2_SYMBOLS; i++)
        {
            hard[i] = deinterleaved[i] < 0.0F ? 1 : 0;
        }
    const auto unsatisfied = static_cast<float>(d_sf2_code.unsatisfied_checks(hard));
    return unsatisfied < GPS_L1C_SF2_MAX_UNSATISFIED_RATIO * static_cast<float>(d_sf2_code.message_bits());
}


int32_t Gps_CNAV2_Navigation_Message::get_frame_end_tow() const
{
    if (!d_sf2_ok || d_sf2_no_data || d_toi <= 0 || d_toi > GPS_L1C_TOI_MAX || d_itow < 0 || d_itow > 83)
        {
            return -1;
        }
    return d_itow * GPS_L1C_ITOW_PERIOD_S + d_toi * GPS_L1C_FRAME_PERIOD_S;
}


void Gps_CNAV2_Navigation_Message::decode_sf2(const std::vector<uint8_t>& bits)
{
    Gps_CNAV2_Ephemeris eph = d_ephemeris;
    eph.PRN = d_prn;
    eph.WN = static_cast<int32_t>(read_unsigned(bits, CNAV2_WN));
    d_itow = static_cast<int32_t>(read_unsigned(bits, CNAV2_ITOW));
    eph.ITOW = d_itow;
    eph.top = static_cast<int32_t>(read_unsigned(bits, CNAV2_TOP)) * CNAV2_TOP_LSB;
    eph.l1c_health = static_cast<int32_t>(read_unsigned(bits, CNAV2_L1C_HEALTH));
    eph.URAED = static_cast<int32_t>(read_signed(bits, CNAV2_URA_ED));
    eph.URA = eph.URAED;
    eph.toe = static_cast<int32_t>(read_unsigned(bits, CNAV2_TOE)) * CNAV2_TOE_LSB;
    // CNAV-2 has a single reference time for ephemeris and clock (3.5.5.3)
    eph.toe1 = eph.toe;
    eph.toe2 = eph.toe;
    eph.toc = eph.toe;
    eph.delta_A = static_cast<double>(read_signed(bits, CNAV2_DELTA_A)) * CNAV2_DELTA_A_LSB;
    eph.sqrtA = std::sqrt(CNAV_A_REF + eph.delta_A);
    eph.Adot = static_cast<double>(read_signed(bits, CNAV2_A_DOT)) * CNAV2_A_DOT_LSB;
    eph.delta_n = static_cast<double>(read_signed(bits, CNAV2_DELTA_N0)) * CNAV2_DELTA_N0_LSB;
    eph.delta_ndot = static_cast<double>(read_signed(bits, CNAV2_DELTA_N0_DOT)) * CNAV2_DELTA_N0_DOT_LSB;
    eph.M_0 = static_cast<double>(read_signed(bits, CNAV2_M0)) * CNAV2_M0_LSB;
    eph.ecc = static_cast<double>(read_unsigned(bits, CNAV2_E_ECCENTRICITY)) * CNAV2_E_ECCENTRICITY_LSB;
    eph.omega = static_cast<double>(read_signed(bits, CNAV2_OMEGA)) * CNAV2_OMEGA_LSB;
    eph.OMEGA_0 = static_cast<double>(read_signed(bits, CNAV2_OMEGA0)) * CNAV2_OMEGA0_LSB;
    eph.i_0 = static_cast<double>(read_signed(bits, CNAV2_I0)) * CNAV2_I0_LSB;
    eph.delta_OMEGAdot = static_cast<double>(read_signed(bits, CNAV2_DELTA_OMEGA_DOT)) * CNAV2_DELTA_OMEGA_DOT_LSB;
    eph.OMEGAdot = CNAV_OMEGA_DOT_REF * GNSS_PI + eph.delta_OMEGAdot;
    eph.idot = static_cast<double>(read_signed(bits, CNAV2_I0_DOT)) * CNAV2_I0_DOT_LSB;
    eph.Cis = static_cast<double>(read_signed(bits, CNAV2_CIS)) * CNAV2_CIS_LSB;
    eph.Cic = static_cast<double>(read_signed(bits, CNAV2_CIC)) * CNAV2_CIC_LSB;
    eph.Crs = static_cast<double>(read_signed(bits, CNAV2_CRS)) * CNAV2_CRS_LSB;
    eph.Crc = static_cast<double>(read_signed(bits, CNAV2_CRC)) * CNAV2_CRC_LSB;
    eph.Cus = static_cast<double>(read_signed(bits, CNAV2_CUS)) * CNAV2_CUS_LSB;
    eph.Cuc = static_cast<double>(read_signed(bits, CNAV2_CUC)) * CNAV2_CUC_LSB;
    eph.URANED0 = static_cast<int32_t>(read_signed(bits, CNAV2_URA_NED0));
    eph.URANED1 = static_cast<int32_t>(read_unsigned(bits, CNAV2_URA_NED1));
    eph.URANED2 = static_cast<int32_t>(read_unsigned(bits, CNAV2_URA_NED2));
    eph.URA0 = static_cast<double>(eph.URANED0);
    eph.URA1 = static_cast<double>(eph.URANED1);
    eph.URA2 = static_cast<double>(eph.URANED2);
    eph.af0 = static_cast<double>(read_signed(bits, CNAV2_AF0)) * CNAV2_AF0_LSB;
    eph.af1 = static_cast<double>(read_signed(bits, CNAV2_AF1)) * CNAV2_AF1_LSB;
    eph.af2 = static_cast<double>(read_signed(bits, CNAV2_AF2)) * CNAV2_AF2_LSB;
    eph.TGD = read_group_delay(bits, CNAV2_TGD, CNAV2_TGD_LSB);
    eph.ISCL1CP = read_group_delay(bits, CNAV2_ISCL1CP, CNAV2_ISCL1CP_LSB);
    eph.ISCL1CD = read_group_delay(bits, CNAV2_ISCL1CD, CNAV2_ISCL1CD_LSB);
    eph.integrity_status_flag = (read_unsigned(bits, CNAV2_ISF) != 0);
    eph.WNop = static_cast<int32_t>(read_unsigned(bits, CNAV2_WNOP));
    // Time of week at the start of this frame, when unambiguous
    const int32_t frame_end_tow = (d_toi > 0 && d_toi <= GPS_L1C_TOI_MAX) ? d_itow * GPS_L1C_ITOW_PERIOD_S + d_toi * GPS_L1C_FRAME_PERIOD_S : -1;
    eph.tow = frame_end_tow > 0 ? frame_end_tow - GPS_L1C_FRAME_PERIOD_S : d_itow * GPS_L1C_ITOW_PERIOD_S;
    d_ephemeris = eph;

    // The ITOW count changes every two hours with the same CEI data set
    std::vector<uint8_t> contents(bits.cbegin(), bits.cbegin() + GPS_L1C_SF2_BITS - GPS_L1C_CRC_BITS);
    std::fill(contents.begin() + CNAV2_ITOW[0].first - 1, contents.begin() + CNAV2_ITOW[0].first - 1 + CNAV2_ITOW[0].second, 0);
    if (contents != d_last_published_sf2)
        {
            d_last_published_sf2 = std::move(contents);
            d_new_ephemeris = true;
        }
}


void Gps_CNAV2_Navigation_Message::decode_sf3(const std::vector<uint8_t>& bits)
{
    const auto prn = static_cast<uint32_t>(read_unsigned(bits, CNAV2_SF3_PRN));
    if (d_prn != 0 && prn != d_prn)
        {
            return;
        }
    d_sf3_page = static_cast<int32_t>(read_unsigned(bits, CNAV2_SF3_PAGE));
    switch (d_sf3_page)
        {
        case 1:  // UTC & IONO
            {
                d_utc_model.A0 = static_cast<double>(read_signed(bits, CNAV2_A0)) * CNAV2_A0_LSB;
                d_utc_model.A1 = static_cast<double>(read_signed(bits, CNAV2_A1)) * CNAV2_A1_LSB;
                d_utc_model.A2 = static_cast<double>(read_signed(bits, CNAV2_A2)) * CNAV2_A2_LSB;
                d_utc_model.DeltaT_LS = static_cast<int32_t>(read_signed(bits, CNAV2_DELTA_TLS));
                d_utc_model.tot = static_cast<int32_t>(read_unsigned(bits, CNAV2_TOT)) * CNAV2_TOT_LSB;
                d_utc_model.WN_T = static_cast<int32_t>(read_unsigned(bits, CNAV2_WN_OT));
                d_utc_model.WN_LSF = static_cast<int32_t>(read_unsigned(bits, CNAV2_WN_LSF));
                d_utc_model.DN = static_cast<int32_t>(read_unsigned(bits, CNAV2_DN));
                d_utc_model.DeltaT_LSF = static_cast<int32_t>(read_signed(bits, CNAV2_DELTA_TLSF));
                d_utc_model.valid = true;
                d_new_utc = true;

                d_iono.alpha0 = static_cast<double>(read_signed(bits, CNAV2_ALPHA0)) * CNAV2_ALPHA0_LSB;
                d_iono.alpha1 = static_cast<double>(read_signed(bits, CNAV2_ALPHA1)) * CNAV2_ALPHA1_LSB;
                d_iono.alpha2 = static_cast<double>(read_signed(bits, CNAV2_ALPHA2)) * CNAV2_ALPHA2_LSB;
                d_iono.alpha3 = static_cast<double>(read_signed(bits, CNAV2_ALPHA3)) * CNAV2_ALPHA3_LSB;
                d_iono.beta0 = static_cast<double>(read_signed(bits, CNAV2_BETA0)) * CNAV2_BETA0_LSB;
                d_iono.beta1 = static_cast<double>(read_signed(bits, CNAV2_BETA1)) * CNAV2_BETA1_LSB;
                d_iono.beta2 = static_cast<double>(read_signed(bits, CNAV2_BETA2)) * CNAV2_BETA2_LSB;
                d_iono.beta3 = static_cast<double>(read_signed(bits, CNAV2_BETA3)) * CNAV2_BETA3_LSB;
                d_iono.valid = true;
                d_new_iono = true;

                // Inter-signal corrections for the legacy and modernized signals,
                // carried by the ephemeris
                const double isc_l1ca = read_group_delay(bits, CNAV2_ISCL1CA, CNAV2_ISCL1CA_LSB);
                const double isc_l2c = read_group_delay(bits, CNAV2_ISCL2C, CNAV2_ISCL2C_LSB);
                const double isc_l5i5 = read_group_delay(bits, CNAV2_ISCL5I5, CNAV2_ISCL5I5_LSB);
                const double isc_l5q5 = read_group_delay(bits, CNAV2_ISCL5Q5, CNAV2_ISCL5Q5_LSB);
                const bool iscs_changed = !d_ephemeris.sf3_page1_iscs ||
                                          d_ephemeris.ISCL1 != isc_l1ca || d_ephemeris.ISCL2 != isc_l2c ||
                                          d_ephemeris.ISCL5I != isc_l5i5 || d_ephemeris.ISCL5Q != isc_l5q5;
                d_ephemeris.ISCL1 = isc_l1ca;
                d_ephemeris.ISCL2 = isc_l2c;
                d_ephemeris.ISCL5I = isc_l5i5;
                d_ephemeris.ISCL5Q = isc_l5q5;
                d_ephemeris.sf3_page1_iscs = true;
                if (iscs_changed && !d_last_published_sf2.empty())
                    {
                        d_new_ephemeris = true;
                    }
                break;
            }
        case 2:  // GGTO & EOP
            {
                d_eop.PRN = d_prn;
                d_eop.tow = static_cast<double>(d_ephemeris.tow);
                d_eop.t_eop = static_cast<double>(read_unsigned(bits, CNAV2_T_EOP) * CNAV2_T_EOP_LSB);
                d_eop.pm_x = static_cast<double>(read_signed(bits, CNAV2_PM_X)) * CNAV2_PM_X_LSB;
                d_eop.pm_x_dot = static_cast<double>(read_signed(bits, CNAV2_PM_X_DOT)) * CNAV2_PM_X_DOT_LSB;
                d_eop.pm_y = static_cast<double>(read_signed(bits, CNAV2_PM_Y)) * CNAV2_PM_Y_LSB;
                d_eop.pm_y_dot = static_cast<double>(read_signed(bits, CNAV2_PM_Y_DOT)) * CNAV2_PM_Y_DOT_LSB;
                d_eop.delta_ut1_gps = static_cast<double>(read_signed(bits, CNAV2_DELTA_UT1)) * CNAV2_DELTA_UT1_LSB;
                d_eop.delta_ut1_gps_dot = static_cast<double>(read_signed(bits, CNAV2_DELTA_UT1_DOT)) * CNAV2_DELTA_UT1_DOT_LSB;
                d_eop.valid = true;
                d_new_eop = true;
                break;
            }
        default:
            // Pages 3 and 4 (almanac), 5 (differential corrections), 6 (text),
            // 7 (SV configuration) and 8 (ISM) are not used
            break;
        }
}


bool Gps_CNAV2_Navigation_Message::have_new_ephemeris()
{
    const bool new_data = d_new_ephemeris;
    d_new_ephemeris = false;
    return new_data;
}


bool Gps_CNAV2_Navigation_Message::have_new_iono()
{
    const bool new_data = d_new_iono;
    d_new_iono = false;
    return new_data;
}


bool Gps_CNAV2_Navigation_Message::have_new_utc_model()
{
    const bool new_data = d_new_utc;
    d_new_utc = false;
    return new_data;
}


bool Gps_CNAV2_Navigation_Message::have_new_eop()
{
    const bool new_data = d_new_eop;
    d_new_eop = false;
    return new_data;
}
