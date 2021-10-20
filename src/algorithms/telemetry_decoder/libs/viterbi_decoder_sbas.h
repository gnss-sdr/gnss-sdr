/*!
 * \file viterbi_decoder_sbas.h
 * \brief Interface of a Viterbi decoder class based on the Iterative Solutions
 * Coded Modulation Library by Matthew C. Valenti
 * \author Daniel Fehr 2013. daniel.co(at)bluewin.ch
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_VITERBI_DECODER_SBAS_H
#define GNSS_SDR_VITERBI_DECODER_SBAS_H

#include <cstddef>  // for size_t
#include <deque>
#include <vector>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_libs telemetry_decoder_libs
 * \{ */


/*!
 * \brief Class that implements a Viterbi decoder
 */
class Viterbi_Decoder_Sbas
{
public:
    Viterbi_Decoder_Sbas(const int g_encoder[], int KK, int nn);

    void reset();

    /*!
     * \brief Uses the Viterbi algorithm to perform hard-decision decoding of a convolutional code.
     *
     * \param[in]  input_c[]    The received signal in LLR-form. For BPSK, must be in form r = 2*a*y/(sigma^2).
     * \param[in]  LL           The number of data bits to be decoded (does not include the mm zero-tail-bits)
     *
     * \return  output_u_int[] Hard decisions on the data bits (without the mm zero-tail-bits)
     */
    float decode_block(const double input_c[], int* output_u_int, int LL);

    float decode_continuous(const double sym[], int traceback_depth, int bits[],
        int nbits_requested, int& nbits_decoded);

private:
    class Prev
    {
    public:
        int num_states;
        Prev(int states, int t);
        Prev(const Prev& prev);
        Prev& operator=(const Prev& other);
        ~Prev();

        int get_anchestor_state_of_current_state(int current_state) const;
        int get_bit_of_current_state(int current_state) const;
        float get_metric_of_current_state(int current_state) const;
        int get_t() const;
        void set_current_state_as_ancestor_of_next_state(int next_state, int current_state);
        void set_decoded_bit_for_next_state(int next_state, int bit);
        void set_survivor_branch_metric_of_next_state(int next_state, float metric);

    private:
        std::vector<float> v_metric;
        std::vector<int> state;
        std::vector<int> v_bit;
        int t;
        int refcount;
    };

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

    // trellis state
    std::deque<Prev> d_trellis_paths;
    std::vector<float> d_pm_t;
    std::vector<float> d_metric_c;  /* Set of all possible branch metrics */
    std::vector<float> d_rec_array; /* Received values for one trellis section */

    // trellis definition
    std::vector<int> d_out0;
    std::vector<int> d_state0;
    std::vector<int> d_out1;
    std::vector<int> d_state1;

    // measures
    float d_indicator_metric;

    // code properties
    int d_KK;
    int d_nn;

    // derived code properties
    int d_mm;
    int d_states;
    int d_number_symbols;
    bool d_trellis_state_is_initialised;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_VITERBI_DECODER_SBAS_H
