/*!
 * \file viterbi_decoder.cc
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

#include "viterbi_decoder.h"
#include <volk_gnsssdr/volk_gnsssdr.h>  // for volk_gnsssdr_32f_index_max_32u
#include <algorithm>                    // for std::copy

Viterbi_Decoder::Viterbi_Decoder(int32_t KK,
    int32_t nn,
    int32_t LL,
    const std::array<int32_t, 2>& g) : d_g(g),
                                       d_KK(KK),
                                       d_nn(nn),
                                       d_LL(LL),
                                       d_mm(KK - 1),
                                       d_states(1 << d_mm),       //  2^d_mm
                                       d_number_symbols(1 << nn)  //  2^d_nn
{
    d_prev_section = std::vector<float>(d_states, -d_MAXLOG);
    d_next_section = std::vector<float>(d_states, -d_MAXLOG);
    d_rec_array = std::vector<float>(d_nn);
    d_metric_c = std::vector<float>(d_number_symbols);
    d_prev_bit = std::vector<int32_t>(d_states * (d_LL + d_mm));
    d_prev_state = std::vector<int32_t>(d_states * (d_LL + d_mm));
    d_out0 = std::vector<int32_t>(d_states);
    d_out1 = std::vector<int32_t>(d_states);
    d_state0 = std::vector<int32_t>(d_states);
    d_state1 = std::vector<int32_t>(d_states);
    nsc_transit(d_out0, d_state0, 0);
    nsc_transit(d_out1, d_state1, 1);
}


void Viterbi_Decoder::decode(std::vector<int32_t>& output_u_int, const std::vector<float>& input_c)
{
    int32_t i;
    int32_t t;
    int32_t state;
    uint32_t max_index;
    float metric;
    float max_val;

    d_prev_section[0] = 0.0;  //  start in all-zeros state

    // go through trellis
    for (t = 0; t < d_LL + d_mm; t++)
        {
            std::copy(input_c.begin() + d_nn * t, input_c.begin() + d_nn * t + (d_nn - 1), d_rec_array.begin());

            // precompute all possible branch metrics
            for (i = 0; i < d_number_symbols; i++)
                {
                    d_metric_c[i] = this->Gamma(i);
                }

            // step through all states
            for (state = 0; state < d_states; state++)
                {
                    // hypothesis: info bit is a zero
                    metric = d_prev_section[state] + d_metric_c[d_out0[state]];

                    // store new metric if more than metric in storage
                    if (metric > d_next_section[d_state0[state]])
                        {
                            d_next_section[d_state0[state]] = metric;
                            d_prev_state[t * d_states + d_state0[state]] = state;
                            d_prev_bit[t * d_states + d_state0[state]] = 0;
                        }

                    // hypothesis: info bit is a one
                    metric = d_prev_section[state] + d_metric_c[d_out1[state]];

                    // store new metric if more than metric in storage
                    if (metric > d_next_section[d_state1[state]])
                        {
                            d_next_section[d_state1[state]] = metric;
                            d_prev_state[t * d_states + d_state1[state]] = state;
                            d_prev_bit[t * d_states + d_state1[state]] = 1;
                        }
                }

            // normalize
            volk_gnsssdr_32f_index_max_32u(&max_index, d_next_section.data(), d_states);
            max_val = d_next_section[max_index];

            for (state = 0; state < d_states; state++)
                {
                    d_prev_section[state] = d_next_section[state] - max_val;
                    d_next_section[state] = -d_MAXLOG;
                }
        }

    // trace-back operation
    state = 0;

    // tail, no need to output
    for (t = d_LL + d_mm - 1; t >= d_LL; t--)
        {
            state = d_prev_state[t * d_states + state];
        }

    for (t = d_LL - 1; t >= 0; t--)
        {
            output_u_int[t] = d_prev_bit[t * d_states + state];
            state = d_prev_state[t * d_states + state];
        }
}


void Viterbi_Decoder::reset()
{
    d_out0 = std::vector<int32_t>(d_states);
    d_out1 = std::vector<int32_t>(d_states);
    d_state0 = std::vector<int32_t>(d_states);
    d_state1 = std::vector<int32_t>(d_states);
    nsc_transit(d_out0, d_state0, 0);
    nsc_transit(d_out1, d_state1, 1);
}


void Viterbi_Decoder::nsc_transit(std::vector<int32_t>& output_p,
    std::vector<int32_t>& trans_p,
    int32_t input) const
{
    int32_t nextstate;
    int32_t state;
    const int32_t states = (1 << (d_KK - 1));  // The number of states: 2^d_mm

    // Determine the output and next state for each possible starting state
    for (state = 0; state < states; state++)
        {
            output_p[state] = nsc_enc_bit(&nextstate, input, state);
            trans_p[state] = nextstate;
        }
}


float Viterbi_Decoder::Gamma(int32_t symbol) const
{
    float rm = 0;
    int32_t mask = 1;
    int32_t i;

    for (i = 0; i < d_nn; i++)
        {
            if (symbol & mask)
                {
                    rm += d_rec_array[d_nn - i - 1];
                }
            mask = mask << 1;
        }
    return rm;
}


int32_t Viterbi_Decoder::parity_counter(int32_t symbol, int32_t length) const
{
    int32_t counter;
    int32_t temp_parity = 0;

    for (counter = 0; counter < length; counter++)
        {
            temp_parity = temp_parity ^ (symbol & 1);
            symbol = symbol >> 1;
        }
    return temp_parity;
}


int32_t Viterbi_Decoder::nsc_enc_bit(int32_t* state_out_p,
    int32_t input,
    int32_t state_in) const
{
    // declare variables
    int32_t i;
    int32_t out_ = 0;

    // create a word made up of state and new input
    const int32_t state = (input << (d_KK - 1)) ^ state_in;

    // AND the word with the generators
    for (i = 0; i < d_nn; i++)
        {
            // update output symbol
            out_ = (out_ << 1) + parity_counter(state & d_g[i], d_KK);
        }

    // shift the state to make the new state
    *state_out_p = state >> 1;
    return out_;
}
