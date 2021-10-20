/*!
 * \file viterbi_decoder.h
 * \brief Class that implements a Viterbi decoder
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

#ifndef GNSS_SDR_VITERBI_DECODER_H
#define GNSS_SDR_VITERBI_DECODER_H

#include <array>
#include <cstdint>
#include <vector>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_libs
 * Utilities for the decoding of GNSS navigation messages.
 * \{ */


/*!
 * \brief Class that implements a Viterbi decoder
 */
class Viterbi_Decoder
{
public:
    /*!
     * \brief Constructor of a Viterbi decoder
     * \param[in] KK  Constraint length
     * \param[in] nn  Coding rate 1/n
     * \param[in] LL  Data length
     * \param[in] g   Polynomial G1 and G2
     */
    Viterbi_Decoder(int32_t KK, int32_t nn, int32_t LL, const std::array<int32_t, 2>& g);

    /*!
     * \brief Uses the Viterbi algorithm to perform hard-decision decoding of a convolutional code.
     * \param[out] output_u_int    Hard decisions on the data bits
     * \param[in] input_c The received signal in LLR-form. For BPSK, must be in form r = 2*a*y/(sigma^2).
     *
     */
    void decode(std::vector<int32_t>& output_u_int, const std::vector<float>& input_c);

    /*!
     * \brief Reset internal status
     */
    void reset();

private:
    /*
     * Function that creates the transit and output vectors
     */
    void nsc_transit(std::vector<int32_t>& output_p,
        std::vector<int32_t>& trans_p,
        int32_t input) const;

    /*
     *  Computes the branch metric used for decoding.
     *  \return (returned float) The metric between the hypothetical symbol and the received vector
     *  \param[in] symbol        The hypothetical symbol
     *
     */
    float Gamma(int32_t symbol) const;

    /*
     * Determines if a symbol has odd (1) or even (0) parity
     *    Output parameters:
     * \return (returned int): The symbol's parity = 1 for odd and 0 for even
     *
     * \param[in] symbol  The integer-valued symbol
     * \param[in] length  The highest bit position in the symbol
     *
     * This function is used by nsc_enc_bit()
     */
    int32_t parity_counter(int32_t symbol, int32_t length) const;

    /*
     * Convolutionally encodes a single bit using a rate 1/n encoder.
     * Takes in one input bit at a time, and produces a n-bit output.
     *
     * \return (returned int): Computed output
     *
     * \param[in]  input     The input data bit (i.e. a 0 or 1).
     * \param[in]  state_in  The starting state of the encoder (an int from 0 to 2^m-1).
     * \param[out] state_out_p[]  An integer containing the final state of the encoder
     *                               (i.e. the state after encoding this bit)
     *
     * This function is used by nsc_transit()
     */
    int32_t nsc_enc_bit(int32_t* state_out_p,
        int32_t input,
        int32_t state_in) const;

    std::vector<float> d_prev_section{};
    std::vector<float> d_next_section{};

    std::vector<float> d_rec_array{};
    std::vector<float> d_metric_c{};
    std::vector<int32_t> d_prev_bit{};
    std::vector<int32_t> d_prev_state{};
    std::array<int32_t, 2> d_g{};

    std::vector<int32_t> d_out0;
    std::vector<int32_t> d_out1;
    std::vector<int32_t> d_state0;
    std::vector<int32_t> d_state1;

    float d_MAXLOG = 1e7;  // Define infinity
    int32_t d_KK{};
    int32_t d_nn{};
    int32_t d_LL{};

    int32_t d_mm{};
    int32_t d_states{};
    int32_t d_number_symbols{};
};

/** \} */
/** \} */
#endif  // GNSS_SDR_VITERBI_DECODER_H
