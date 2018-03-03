/*!
 * \file viterbi_decoder.h
 * \brief Interface of a Viterbi decoder class based on the Iterative Solutions
 * Coded Modulation Library by Matthew C. Valenti
 * \author Daniel Fehr 2013. daniel.co(at)bluewin.ch
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_VITERBI_DECODER_H_
#define GNSS_SDR_VITERBI_DECODER_H_

#include <cstddef>  // for size_t
#include <deque>

/*!
 * \brief Class that implements a Viterbi decoder
 */
class Viterbi_Decoder
{
public:
    Viterbi_Decoder(const int g_encoder[], const int KK, const int nn);
    ~Viterbi_Decoder();
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

    float decode_continuous(const double sym[], const int traceback_depth, int output_u_int[],
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
        int* state;
        int* bit;
        float* metric;
        int* refcount;
    };

    // code properties
    int d_KK;
    int d_nn;

    // derived code properties
    int d_mm;
    int d_states;
    int d_number_symbols;

    // trellis definition
    int* d_out0;
    int* d_state0;
    int* d_out1;
    int* d_state1;

    // trellis state
    float* d_pm_t;
    std::deque<Prev> d_trellis_paths;
    float* d_metric_c;  /* Set of all possible branch metrics */
    float* d_rec_array; /* Received values for one trellis section */
    bool d_trellis_state_is_initialised;

    // measures
    float d_indicator_metric;

    // operations on the trellis (change decoder state)
    void init_trellis_state();
    int do_acs(const double sym[], int nbits);
    int do_traceback(std::size_t traceback_length);
    int do_tb_and_decode(int traceback_length, int requested_decoding_length, int state, int bits[], float& indicator_metric);

    // branch metric function
    float gamma(float rec_array[], int symbol, int nn);

    // trellis generation
    void nsc_transit(int output_p[], int trans_p[], int input, const int g[], int KK, int nn);
    int nsc_enc_bit(int state_out_p[], int input, int state_in, const int g[], int KK, int nn);
    int parity_counter(int symbol, int length);
};

#endif /* GNSS_SDR_VITERBI_DECODER_H_ */
