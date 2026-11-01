/*!
 * \file gps_cnav2_navigation_message.h
 * \brief Interface of a GPS CNAV-2 (L1C) navigation message decoder
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

#ifndef GNSS_SDR_GPS_CNAV2_NAVIGATION_MESSAGE_H
#define GNSS_SDR_GPS_CNAV2_NAVIGATION_MESSAGE_H

#include "gps_cnav2_ephemeris.h"
#include "gps_cnav2_ldpc.h"
#include "gps_cnav_eop.h"
#include "gps_cnav_iono.h"
#include "gps_cnav_utc_model.h"
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class decodes the GPS CNAV-2 navigation message of the L1CD
 * signal, as described in IS-GPS-800J.
 *
 * A frame is made of 1800 symbols: the BCH(51,8)-encoded TOI (subframe 1)
 * followed by the interleaved, LDPC-encoded subframes 2 and 3.
 */
class Gps_CNAV2_Navigation_Message
{
public:
    Gps_CNAV2_Navigation_Message();

    /*!
     * \brief Sets the PRN of the transmitting satellite. It is copied into
     * the ephemeris and checked against the PRN field of subframe 3.
     */
    void set_prn(uint32_t prn);

    /*!
     * \brief Forgets the decoded data, keeping the PRN
     */
    void reset();

    /*!
     * \brief Decodes a frame of 1800 soft symbols, in order of reception,
     * where a positive value stands for a "0" symbol.
     * \return true if subframe 2 or subframe 3 passed the CRC check
     */
    bool decode_frame(const float* symbols);

    /*!
     * \brief Fast test of whether 1800 soft symbols, in order of reception,
     * may be a frame: the TOI word must correlate well, and the hard decisions
     * of subframe 2 must satisfy clearly more parity checks than random data
     * (half of them). Much cheaper than decode_frame().
     */
    bool is_frame_candidate(const float* symbols) const;

    // Results of the last call to decode_frame()
    int32_t get_toi() const { return d_toi; }                       //!< TOI count (0 to 511, valid up to 399)
    float get_toi_quality() const { return d_toi_quality; }         //!< Normalized TOI correlation (1.0 for an error-free word)
    bool sf2_crc_ok() const { return d_sf2_ok; }                    //!< Subframe 2 passed the CRC check
    bool sf3_crc_ok() const { return d_sf3_ok; }                    //!< Subframe 3 passed the CRC check
    bool sf2_has_no_data() const { return d_sf2_no_data; }          //!< Subframe 2 passed the CRC check, but is the default message (IS-GPS-800J 3.5.1) or all zeros
    bool sf3_has_no_data() const { return d_sf3_no_data; }          //!< Subframe 3 passed the CRC check, but is the default message (IS-GPS-800J 3.5.1) or all zeros
    int32_t get_sf3_page() const { return d_sf3_page; }             //!< Page number of subframe 3, -1 if not decoded
    const std::string& get_sf2_bits() const { return d_sf2_bits; }  //!< Decoded subframe 2 as a string of '0' and '1'
    const std::string& get_sf3_bits() const { return d_sf3_bits; }  //!< Decoded subframe 3 as a string of '0' and '1'

    /*!
     * \brief Time of week at the end of the last decoded frame, that is, at
     * the start of the next one [s]. Returns -1 when subframe 2 failed the
     * CRC check or carries no data, or when TOI = 0, since then the
     * ITOW of the next frame may already belong to the next two-hour epoch
     * (IS-GPS-800J, 3.5.2).
     */
    int32_t get_frame_end_tow() const;

    /*!
     * \brief Week number of the last subframe 2 that passed the CRC check
     */
    int32_t get_wn() const { return d_ephemeris.WN; }

    bool have_new_ephemeris();  //!< Returns true once per new subframe 2 contents
    bool have_new_iono();       //!< Returns true once per subframe 3, page 1
    bool have_new_utc_model();  //!< Returns true once per subframe 3, page 1
    bool have_new_eop();        //!< Returns true once per subframe 3, page 2
    Gps_CNAV2_Ephemeris get_ephemeris() const { return d_ephemeris; }
    Gps_CNAV_Iono get_iono() const { return d_iono; }
    Gps_CNAV_Utc_Model get_utc_model() const { return d_utc_model; }
    Gps_CNAV_Eop get_eop() const { return d_eop; }

    /*!
     * \brief Encodes a TOI count into its 52 symbols (IS-GPS-800J, 3.2.3.2)
     */
    static std::vector<uint8_t> encode_toi(int32_t toi);

    /*!
     * \brief Maximum-likelihood TOI decoder over 52 soft symbols, as proposed
     * in IS-GPS-800J, 3.2.3.2. Returns the 9-bit TOI and writes the
     * correlation normalized by the sum of the symbol magnitudes.
     */
    static int32_t decode_toi(const float* symbols, float& quality);

    /*!
     * \brief Block interleaver of IS-GPS-800J, 3.2.3.5 (1748 symbols)
     */
    static void interleave(const uint8_t* in, uint8_t* out);

    /*!
     * \brief Inverse of interleave() (1748 symbols)
     */
    static void deinterleave(const float* in, float* out);

    /*!
     * \brief CRC-24Q of IS-GPS-800J, 3.2.3.3, computed over the first nbits
     */
    static uint32_t crc24q(const std::vector<uint8_t>& bits, size_t nbits);

private:
    void decode_sf2(const std::vector<uint8_t>& bits);
    void decode_sf3(const std::vector<uint8_t>& bits);

    Gps_Cnav2_Ldpc d_sf2_code;
    Gps_Cnav2_Ldpc d_sf3_code;

    Gps_CNAV2_Ephemeris d_ephemeris{};
    Gps_CNAV_Iono d_iono{};
    Gps_CNAV_Utc_Model d_utc_model{};
    Gps_CNAV_Eop d_eop{};

    std::vector<uint8_t> d_last_published_sf2;
    std::string d_sf2_bits;
    std::string d_sf3_bits;

    uint32_t d_prn{};
    int32_t d_toi{-1};
    int32_t d_itow{-1};
    int32_t d_sf3_page{-1};
    float d_toi_quality{};
    bool d_sf2_ok{};
    bool d_sf3_ok{};
    bool d_sf2_no_data{};
    bool d_sf3_no_data{};
    bool d_new_ephemeris{};
    bool d_new_iono{};
    bool d_new_utc{};
    bool d_new_eop{};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_CNAV2_NAVIGATION_MESSAGE_H
