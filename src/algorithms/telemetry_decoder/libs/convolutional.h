/*!
 * \file convolutional.h
 * \brief General functions used to implement convolutional encoding.
 * \author Matthew C. Valenti, 2006-2008.
 * \author C. Fernandez-Prades, 2019.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2006-2008  Matthew C. Valenti
 * Copyright (C) 2019 C. Fernandez-Prades
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * This file is a derived work of the original file, which had this note:
 *
 * The functions in this file are part of the Iterative Solutions
 * Coded Modulation Library. The Iterative Solutions Coded Modulation
 * Library is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; either version 2.1 of the License,
 * or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef GNSS_SDR_CONVOLUTIONAL_H
#define GNSS_SDR_CONVOLUTIONAL_H

#include <volk_gnsssdr/volk_gnsssdr.h>
#include <vector>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_libs
 * Utilities for the decoding of GNSS navigation messages.
 * \{ */


/* define constants used throughout the library */
const float MAXLOG = 1e7; /* Define infinity */


/*!
 * \brief Determines if a symbol has odd (1) or even (0) parity
 *    Output parameters:
 * \return (returned int): The symbol's parity = 1 for odd and 0 for even
 *
 * \param[in] symbol  The integer-valued symbol
 * \param[in] length  The highest bit position in the symbol
 *
 * This function is used by nsc_enc_bit(), rsc_enc_bit(), and rsc_tail()
 */
inline int parity_counter(int symbol, int length)
{
    int counter;
    int temp_parity = 0;

    for (counter = 0; counter < length; counter++)
        {
            temp_parity = temp_parity ^ (symbol & 1);
            symbol = symbol >> 1;
        }
    return (temp_parity);
}


/*!
 * \brief Convolutionally encodes a single bit using a rate 1/n encoder.
 * Takes in one input bit at a time, and produces a n-bit output.
 *
 * \param[in]  input     The input data bit (i.e. a 0 or 1).
 * \param[in]  state_in  The starting state of the encoder (an int from 0 to 2^m-1).
 * \param[in]  g[]       An n-element vector containing the code generators in binary form.
 * \param[in]  KK        The constraint length of the convolutional code.
 * \param[out] output_p[]     An n-element vector containing the encoded bits.
 * \param[out] state_out_p[]  An integer containing the final state of the encoder
 *                               (i.e. the state after encoding this bit)
 *
 * This function is used by nsc_transit()
 */
inline int nsc_enc_bit(int state_out_p[],
    int input,
    int state_in,
    const int g[],
    int KK,
    int nn)
{
    /* declare variables */
    int state, i;
    int out_ = 0;

    /* create a word made up of state and new input */
    state = (input << (KK - 1)) ^ state_in;

    /* AND the word with the generators */
    for (i = 0; i < nn; i++)
        {
            /* update output symbol */
            out_ = (out_ << 1) + parity_counter(state & g[i], KK);
        }

    /* shift the state to make the new state */
    state_out_p[0] = state >> 1;
    return (out_);
}


/*!
 * \brief Function that creates the transit and output vectors
 */
inline void nsc_transit(int output_p[],
    int trans_p[],
    int input,
    int g[],
    int KK,
    int nn)
{
    int nextstate[1];
    int state, states;
    states = (1 << (KK - 1)); /* The number of states: 2^mm */

    /* Determine the output and next state for each possible starting state */
    for (state = 0; state < states; state++)
        {
            output_p[state] = nsc_enc_bit(nextstate, input, state, g, KK, nn);
            trans_p[state] = nextstate[0];
        }
    return;
}


/*!
 * \brief  Computes the branch metric used for decoding.
 *  \return (returned float) The metric between the hypothetical symbol and the received vector
 *  \param[in] rec_array     The received vector, of length nn
 *  \param[in] symbol        The hypothetical symbol
 *  \param[in] nn            The length of the received vector
 *
 */
inline float Gamma(const float rec_array[],
    int symbol,
    int nn)
{
    float rm = 0;
    int i;
    int mask = 1;

    for (i = 0; i < nn; i++)
        {
            if (symbol & mask)
                {
                    rm += rec_array[nn - i - 1];
                }
            mask = mask << 1;
        }
    return (rm);
}


/*!
 * \brief Uses the Viterbi algorithm to perform hard-decision decoding of a convolutional code.
 * \param[in] out0[]    The output bits for each state if input is a 0.
 * \param[in] state0[]  The next state if input is a 0.
 * \param[in] out1[]    The output bits for each state if input is a 1.
 * \param[in] state1[]  The next state if input is a 1.
 * \param[in] r[]       The received signal in LLR-form. For BPSK, must be in form r = 2*a*y/(sigma^2).
 * \param[in] KK        The constraint length of the convolutional code.
 * \param[in] LL        The number of data bits.
 * \param[out] output_u_int[]    Hard decisions on the data bits
 *
 */
inline void Viterbi(int output_u_int[],
    const int out0[],
    const int state0[],
    const int out1[],
    const int state1[],
    const float input_c[],
    int KK,
    int nn,
    int LL)
{
    int i, t, state, mm, states;
    int number_symbols;
    uint32_t max_index;
    float metric;
    float max_val;

    /* some derived constants */
    mm = KK - 1;
    states = 1 << mm;         /* 2^mm */
    number_symbols = 1 << nn; /* 2^nn */

    std::vector<float> prev_section(states, -MAXLOG);
    std::vector<float> next_section(states, -MAXLOG);
    std::vector<int> prev_bit(states * (LL + mm), 0);
    std::vector<int> prev_state(states * (LL + mm), 0);
    std::vector<float> rec_array(nn);
    std::vector<float> metric_c(number_symbols);

    prev_section[0] = 0.0; /* start in all-zeros state */

    /* go through trellis */
    for (t = 0; t < LL + mm; t++)
        {
            rec_array.assign(input_c + nn * t, input_c + nn * t + (nn - 1));

            /* precompute all possible branch metrics */
            for (i = 0; i < number_symbols; i++)
                {
                    metric_c[i] = Gamma(rec_array.data(), i, nn);
                }

            /* step through all states */
            for (state = 0; state < states; state++)
                {
                    /* hypothesis: info bit is a zero */
                    metric = prev_section[state] + metric_c[out0[state]];

                    /* store new metric if more than metric in storage */
                    if (metric > next_section[state0[state]])
                        {
                            next_section[state0[state]] = metric;
                            prev_state[t * states + state0[state]] = state;
                            prev_bit[t * states + state0[state]] = 0;
                        }

                    /* hypothesis: info bit is a one */
                    metric = prev_section[state] + metric_c[out1[state]];

                    /* store new metric if more than metric in storage */
                    if (metric > next_section[state1[state]])
                        {
                            next_section[state1[state]] = metric;
                            prev_state[t * states + state1[state]] = state;
                            prev_bit[t * states + state1[state]] = 1;
                        }
                }

            /* normalize */
            volk_gnsssdr_32f_index_max_32u(&max_index, next_section.data(), states);
            max_val = next_section[max_index];

            for (state = 0; state < states; state++)
                {
                    prev_section[state] = next_section[state] - max_val;
                    next_section[state] = -MAXLOG;
                }
        }

    /* trace-back operation */
    state = 0;

    /* tail, no need to output */
    for (t = LL + mm - 1; t >= LL; t--)
        {
            state = prev_state[t * states + state];
        }

    for (t = LL - 1; t >= 0; t--)
        {
            output_u_int[t] = prev_bit[t * states + state];
            state = prev_state[t * states + state];
        }
}


/** \} */
/** \} */
#endif  // GNSS_SDR_CONVOLUTIONAL_H
