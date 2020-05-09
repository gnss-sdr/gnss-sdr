/*!
 * \file viterbi_decoder.h
 * \brief Interface of a Viterbi decoder class based on the Iterative Solutions
 * Coded Modulation Library by Matthew C. Valenti
 * \author Daniel Fehr 2013. daniel.co(at)bluewin.ch
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_VITERBI_DECODER_H
#define GNSS_SDR_VITERBI_DECODER_H

#include <cstddef>  // for size_t
#include <deque>
#include <vector>

/*!
 * \brief Class that implements a Viterbi decoder
 */
class Viterbi_Decoder
{
public:
    Viterbi_Decoder(const int g_encoder[], const int KK, const int nn);
    ~Viterbi_Decoder() = default;
    void reset();

    /*!
     * \brief Uses the Viterbi algorithm to perform hard-decision decoding of a convolutional code.
     *
     * \param[in]  input_c[]    The received signal in LLR-form. For BPSK, must be in form r = 2*a*y/(sigma^2).
     * \param[in]  LL           The number of data bits to be decoded (does not include the mm zero-tail-bits)
     *
     * \return  output_u_int[] Hard decisions on the data bits (without the mm zero-tail-bits)
     */
    float decode_block(const double input_c[], int* output_u_int, const int LL);

    float decode_continuous(const double sym[], const int traceback_depth, int bits[],
        const int nbits_requested, int& nbits_decoded);

private:
    class Prev
    {
    public:
        int num_states;
        Prev(int states, int t);
        Prev(const Prev& prev);
        Prev& operator=(const Prev& other);
        ~Prev();

        int get_anchestor_state_of_current_state(int current_state);
        int get_bit_of_current_state(int current_state);
        float get_metric_of_current_state(int current_state);
        int get_t();
        void set_current_state_as_ancestor_of_next_state(int next_state, int current_state);
        void set_decoded_bit_for_next_state(int next_state, int bit);
        void set_survivor_branch_metric_of_next_state(int next_state, float metric);

    private:
        int t;
        std::vector<int> state;
        std::vector<int> v_bit;
        std::vector<float> v_metric;
        int refcount;
    };

    // code properties
    int d_KK;
    int d_nn;

    // derived code properties
    int d_mm;
    int d_states;
    int d_number_symbols;

    // trellis definition
    std::vector<int> d_out0;
    std::vector<int> d_state0;
    std::vector<int> d_out1;
    std::vector<int> d_state1;

    // trellis state
    std::vector<float> d_pm_t;
    std::deque<Prev> d_trellis_paths;
    std::vector<float> d_metric_c;  /* Set of all possible branch metrics */
    std::vector<float> d_rec_array; /* Received values for one trellis section */
    bool d_trellis_state_is_initialised;

    // measures
    float d_indicator_metric;

    // operations on the trellis (change decoder state)
    void init_trellis_state();
    int do_acs(const double sym[], int nbits);
    int do_traceback(std::size_t traceback_length);
    int do_tb_and_decode(int traceback_length, int requested_decoding_length, int state, int output_u_int[], float& indicator_metric);

    // branch metric function
    float gamma(const float rec_array[], int symbol, int nn);

    // trellis generation
    void nsc_transit(int output_p[], int trans_p[], int input, const int g[], int KK, int nn);
    int nsc_enc_bit(int state_out_p[], int input, int state_in, const int g[], int KK, int nn);
    int parity_counter(int symbol, int length);
};

#endif  // GNSS_SDR_VITERBI_DECODER_H
