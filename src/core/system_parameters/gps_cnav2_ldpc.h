/*!
 * \file gps_cnav2_ldpc.h
 * \brief Rate 1/2 LDPC codes of the GPS L1C CNAV-2 subframes 2 and 3
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * Parity-check matrices from IS-GPS-800J, Section 3.2.3.4 and Tables 6.2-2
 * to 6.2-13.
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

#ifndef GNSS_SDR_GPS_CNAV2_LDPC_H
#define GNSS_SDR_GPS_CNAV2_LDPC_H

#include <cstdint>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief Encoder and soft-decision decoder of the GPS L1C LDPC codes.
 *
 * The parity-check matrix H = [A B T; C D E] has m = k rows and n = 2k
 * columns, with k = 600 for subframe 2 and k = 274 for subframe 3. The
 * codewords are (s; p1; p2), with the k message bits s first.
 */
class Gps_Cnav2_Ldpc
{
public:
    /*!
     * \brief Builds the code of subframe 2 or 3 (any other value builds the
     * subframe 2 code)
     */
    explicit Gps_Cnav2_Ldpc(int32_t subframe);

    int32_t message_bits() const { return d_k; }   //!< k
    int32_t codeword_bits() const { return d_n; }  //!< n = 2k

    /*!
     * \brief Systematic encoder of IS-GPS-800J, 3.2.3.4. The message holds
     * k bits (0 or 1), and the returned codeword n bits.
     */
    std::vector<uint8_t> encode(const std::vector<uint8_t>& message) const;

    /*!
     * \brief Returns true if all parity checks of H are satisfied
     */
    bool check(const std::vector<uint8_t>& codeword) const;

    /*!
     * \brief Number of parity checks of H that the codeword does not satisfy
     */
    int32_t unsatisfied_checks(const std::vector<uint8_t>& codeword) const;

    /*!
     * \brief Layered belief-propagation decoder.
     *
     * \param llr n log-likelihood ratios, log(P(bit = 0) / P(bit = 1))
     * \param codeword decoded n bits (the hard decision when it fails)
     * \param max_iterations maximum number of iterations
     * \return true if the decoded codeword satisfies all parity checks
     */
    bool decode(const std::vector<float>& llr, std::vector<uint8_t>& codeword, int32_t max_iterations = 50) const;

private:
    std::vector<uint8_t> solve_t(std::vector<uint8_t> rhs) const;

    std::vector<std::vector<int32_t>> d_rows;  // column indices of each row of H
    std::vector<int32_t> d_edge_offset;        // first edge of each row
    int32_t d_k;
    int32_t d_n;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_CNAV2_LDPC_H
