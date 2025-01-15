/*!
 * \file reed_solomon.h
 * \brief Class implementing a Reed-Solomon encoder/decoder for blocks of 255
 * symbols and symbols of 8 bits.
 * \author Carles Fernandez, 2021. cfernandez(at)cttc.es
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


#ifndef GNSS_SDR_REED_SOLOMON_H
#define GNSS_SDR_REED_SOLOMON_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>


/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

/*!
 * \brief
 * Class implementing a Reed-Solomon encoder and decoder RS(255,K,d) where
 * k=255-nroots is the information vector length and d=nroots+1 is the minimum
 * Hamming distance, with symbols of 8 bits. It allows shortened RS codes.
 */
class ReedSolomon
{
public:
    /*!
     * \brief Default constructor.
     * Constructs a Reed Solomon object. The encode_with_generator_poly
     * and encode_with_generator_matrix methods are available for testing
     * purposes.
     *
     * gnss_signal: empty or "E6B" sets the Galileo E6B RS parameters.
     *  "E1B" sets the Galileo E1B (INAV) RS parameters.
     */
    explicit ReedSolomon(const std::string& gnss_signal = std::string());

    /*!
     * \brief Custom constructor for RS(255, 255-nroots, nroots+1). Parameters:
     *
     * nroots - the number of roots in the RS code generator polynomial,
     *  which is the same as the number of parity symbols in a block.
     *
     * minpoly - primitive polynomial.
     *
     * prim - the primitive root of the generator polynomial.
     *
     * fcr - first consecutive root of the Reed-Solomon generator polynomial.
     *
     * pad - the number of pad symbols in a block. If not defined, it defaults
     *  to 0.
     *
     * shortening - value of the shortening parameter. Defaults to 0.
     *  If different to 0, it defines a shortened RS encoder/decoder.
     *
     * genpoly_coeff - a vector of (nroots+1) elements containing the generator
     *  polynomial coefficients. Only used for encoding. Defaults to empty.
     *  If defined, the encode_with_generator_poly method can be used.
     *
     * gen_matrix - a (255-shortening)x(255-nroots-shortening) matrix containing
     *  the elements of the generator matrix. Only used for encoding. Defaults
     *  to empty. If defined, the encode_with_generator_matrix method can be
     *  used.
     */
    ReedSolomon(int nroots,
        int minpoly,
        int prim,
        int fcr,
        int pad = 0,
        int shortening = 0,
        const std::vector<uint8_t>& genpoly_coeff = std::vector<uint8_t>{},
        const std::vector<std::vector<uint8_t>>& gen_matrix = std::vector<std::vector<uint8_t>>{});

    /*!
     * \brief Decode an encoded block.
     *
     * The decoded symbols are at the first 255-nroots-shortening elements
     * of the data_to_decode vector.
     *
     * The second parameter is optional, and contains a vector of erasure
     * positions to be passed to the decoding algorithm. Defaults to empty.
     *
     * Returns the number of corrected errors, or -1 if decoding failed.
     */
    int decode(std::vector<uint8_t>& data_to_decode,
        const std::vector<int>& erasure_positions = std::vector<int>{}) const;

    /*!
     * \brief Encode data with the generator matrix (for testing purposes)
     *
     * Returns the encoded vector. It is set to all zeros if the generator
     * matrix is not defined.
     */
    std::vector<uint8_t> encode_with_generator_matrix(const std::vector<uint8_t>& data_to_encode) const;

    /*!
     * \brief Encode data with the generator polynomial (for testing purposes)
     *
     * Returns the encoded vector. It is set to all zeros if the generator
     * polynomial is not defined.
     */
    std::vector<uint8_t> encode_with_generator_poly(const std::vector<uint8_t>& data_to_encode) const;

private:
    static const int d_symbols_per_block = 255;  // the total number of symbols in a RS block.
    static const int d_symsize = 8;              // symbol size, in bits.

    int mod255(int x) const;
    int rs_min(int a, int b) const;
    int decode_rs_8(uint8_t* data, const int* eras_pos, int no_eras) const;

    uint8_t galois_mul(uint8_t a, uint8_t b) const;
    uint8_t galois_add(uint8_t a, uint8_t b) const;
    uint8_t galois_mul_table(uint8_t a, uint8_t b) const;

    void encode_rs_8(const uint8_t* data, uint8_t* parity) const;
    void init_log_tables();    // initialize d_log_table and d_antilog
    void init_alpha_tables();  // initialize d_alpha_to, d_index_of

    std::array<uint8_t, 256> d_alpha_to{};   // used for decoding
    std::array<uint8_t, 256> d_index_of{};   // used for decoding
    std::array<uint8_t, 256> d_log_table{};  // used for encoding
    std::array<uint8_t, 255> d_antilog{};    // used for encoding

    std::vector<std::vector<uint8_t>> d_genmatrix;  // used for encoding
    std::vector<uint8_t> d_genpoly_coeff;           // used for encoding
    std::vector<uint8_t> d_genpoly_index;           // used for encoding

    size_t d_data_in_block{};           // number of information symbols in a block
    size_t d_rows_G{};                  // number of rows of the generator matrix
    size_t d_columns_G{};               // number of rows of the generator matrix
    size_t d_info_symbols_shortened{};  // number of info symbols in the shortened code
    size_t d_data_symbols_shortened{};  // number of data symbols in the shortened code

    int d_nroots{};      // number of parity symbols in a block
    int d_prim{};        // The primitive root of the generator poly
    int d_pad{};         // the number of pad symbols in a block
    int d_iprim{};       // prim-th root of 1, index form
    int d_fcr{};         // first consecutive root
    int d_shortening{};  // shortening parameter

    uint8_t d_min_poly{};  // primitive polynomial
    uint8_t d_a0{};        // auxiliar variable
};

/** \} */
/** \} */
#endif  // GNSS_SDR_REED_SOLOMON_H
