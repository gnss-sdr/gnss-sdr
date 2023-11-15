/*!
 * \file viterbi_decoder_sbas.cc
 * \brief Implementation of a Viterbi decoder class based on the Iterative Solutions
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

#include "viterbi_decoder_sbas.h"
#include <glog/logging.h>
#include <algorithm>  // for fill_n
#include <ostream>    // for operator<<, basic_ostream, char_traits

// logging
#define EVENT 2   // logs important events which don't occur every block
#define FLOW 3    // logs the function calls of block processing functions
#define BLOCK 4   // once per block
#define SAMPLE 5  // about one log entry per sample
#define LMORE 6   // many entries per sample / very specific stuff


const float MAXLOG = 1e7; /* Define infinity */

Viterbi_Decoder_Sbas::Viterbi_Decoder_Sbas(const int g_encoder[],
    int KK,
    int nn) : d_KK(KK),  // Constraint Length
              d_nn(nn),  // Coding rate 1/n
              d_mm(KK - 1),
              d_states(static_cast<int>(1U << (KK - 1))),    // 2^mm
              d_number_symbols(static_cast<int>(1U << nn)),  // 2^nn
              d_trellis_state_is_initialised(false)
{
    /* create appropriate transition matrices (trellis) */
    d_out0 = std::vector<int>(d_states);
    d_out1 = std::vector<int>(d_states);
    d_state0 = std::vector<int>(d_states);
    d_state1 = std::vector<int>(d_states);

    nsc_transit(d_out0.data(), d_state0.data(), 0, g_encoder, d_KK, d_nn);
    nsc_transit(d_out1.data(), d_state1.data(), 1, g_encoder, d_KK, d_nn);

    // initialise trellis state
    Viterbi_Decoder_Sbas::init_trellis_state();
}


void Viterbi_Decoder_Sbas::reset()
{
    init_trellis_state();
}


/* Function decode_block()
 Description: Uses the Viterbi algorithm to perform hard-decision decoding of a convolutional code.
 Input parameters:
 r[]    The received signal in LLR-form. For BPSK, must be in form r = 2*a*y/(sigma^2).
 LL    The number of data bits to be decoded (doesn't include the mm zero-tail-bits)
 Output parameters:
 output_u_int[]    Hard decisions on the data bits (without the mm zero-tail-bits)
 */
float Viterbi_Decoder_Sbas::decode_block(const double input_c[], int output_u_int[], int LL)
{
    VLOG(FLOW) << "decode_block(): LL=" << LL;

    // init
    init_trellis_state();
    // do add compare select
    do_acs(input_c, LL + d_mm);
    // tail, no need to output -> traceback, but don't decode
    const int state = do_traceback(d_mm);
    // traceback and decode
    const int decoding_length_mismatch = do_tb_and_decode(d_mm, LL, state, output_u_int, d_indicator_metric);

    VLOG(FLOW) << "decoding length mismatch: " << decoding_length_mismatch;

    return d_indicator_metric;
}


float Viterbi_Decoder_Sbas::decode_continuous(const double sym[],
    int traceback_depth,
    int bits[],
    int nbits_requested,
    int& nbits_decoded)
{
    VLOG(FLOW) << "decode_continuous(): nbits_requested=" << nbits_requested;

    // do add compare select
    do_acs(sym, nbits_requested);
    // the ML sequence in the newest part of the trellis can not be decoded
    // since it depends on the future values -> traceback, but don't decode
    const int state = do_traceback(traceback_depth);
    // traceback and decode
    const int decoding_length_mismatch = do_tb_and_decode(traceback_depth, nbits_requested, state, bits, d_indicator_metric);
    nbits_decoded = nbits_requested + decoding_length_mismatch;

    VLOG(FLOW) << "decoding length mismatch (continuous decoding): " << decoding_length_mismatch;

    return d_indicator_metric;
}


void Viterbi_Decoder_Sbas::init_trellis_state()
{
    int state;
    // if trellis state has been initialised, free old state memory
    if (d_trellis_state_is_initialised)
        {
            // init trellis state
            d_pm_t.clear();
            d_rec_array.clear();
            d_metric_c.clear();
        }

    // reserve new trellis state memory
    d_pm_t = std::vector<float>(d_states);
    d_trellis_paths = std::deque<Prev>();
    d_rec_array = std::vector<float>(d_nn);
    d_metric_c = std::vector<float>(d_number_symbols);
    d_trellis_state_is_initialised = true;

    /* initialize trellis */
    for (state = 0; state < d_states; state++)
        {
            d_pm_t[state] = -MAXLOG;
            // d_pm_t_next[state] = -MAXLOG;
        }
    d_pm_t[0] = 0; /* start in all-zeros state */

    d_indicator_metric = 0;
}


int Viterbi_Decoder_Sbas::do_acs(const double sym[], int nbits)
{
    int t;
    int i;
    int state_at_t;
    float metric;
    float max_val;
    std::vector<float> pm_t_next(d_states);

    /* t:
     *    - state: state at t
     *    - d_prev_section[state_at_t]: path metric at t for state state_at_t
     *    - d_out0[state_at_t]: sent symbols for a data bit 0 if state is state_at_t at time t
     *
     */

    for (state_at_t = 0; state_at_t < d_states; state_at_t++)
        {
            pm_t_next[state_at_t] = -MAXLOG;
        }

    /* go through trellis */
    for (t = 0; t < nbits; t++)
        {
            /* Temporarily store the received symbols current decoding step */
            for (i = 0; i < d_nn; i++)
                {
                    d_rec_array[i] = static_cast<float>(sym[d_nn * t + i]);
                }

            /* precompute all possible branch metrics */
            for (i = 0; i < d_number_symbols; i++)
                {
                    d_metric_c[i] = gamma(d_rec_array.data(), i, d_nn);
                    VLOG(LMORE) << "metric for (tx_sym=" << i << "|ry_sym=(" << d_rec_array[0] << ", " << d_rec_array[1] << ") = " << d_metric_c[i];
                }

            // find the survivor branches leading the trellis states at t+1
            Prev next_trellis_states(d_states, t + 1);
            /* step through all states */
            for (state_at_t = 0; state_at_t < d_states; state_at_t++)
                {
                    const int next_state_if_0 = d_state0[state_at_t];
                    const int next_state_if_1 = d_state1[state_at_t];

                    /* hypothesis: info bit is a zero */
                    const float bm_0 = d_metric_c[d_out0[state_at_t]];
                    metric = d_pm_t[state_at_t] + bm_0;  // path metric + zerobranch metric

                    /* store new metric if more than metric in storage */
                    if (metric > pm_t_next[next_state_if_0])
                        {
                            pm_t_next[next_state_if_0] = metric;
                            next_trellis_states.set_current_state_as_ancestor_of_next_state(next_state_if_0, state_at_t);
                            next_trellis_states.set_decoded_bit_for_next_state(next_state_if_0, 0);
                            next_trellis_states.set_survivor_branch_metric_of_next_state(next_state_if_0, bm_0);
                        }

                    /* hypothesis: info bit is a one */
                    const float bm_1 = d_metric_c[d_out1[state_at_t]];
                    metric = d_pm_t[state_at_t] + bm_1;  // path metric + onebranch metric

                    /* store new metric if more than metric in storage */
                    if (metric > pm_t_next[next_state_if_1])
                        {
                            pm_t_next[next_state_if_1] = metric;
                            next_trellis_states.set_current_state_as_ancestor_of_next_state(next_state_if_1, state_at_t);
                            next_trellis_states.set_decoded_bit_for_next_state(next_state_if_1, 1);
                            next_trellis_states.set_survivor_branch_metric_of_next_state(next_state_if_1, bm_1);
                        }
                }

            d_trellis_paths.push_front(next_trellis_states);

            /* normalize -> afterwards, the largest metric value is always 0 */
            // max_val = 0;
            max_val = -MAXLOG;
            for (state_at_t = 0; state_at_t < d_states; state_at_t++)
                {
                    if (pm_t_next[state_at_t] > max_val)
                        {
                            max_val = pm_t_next[state_at_t];
                        }
                }
            VLOG(LMORE) << "max_val at t=" << t << ": " << max_val;
            for (state_at_t = 0; state_at_t < d_states; state_at_t++)
                {
                    d_pm_t[state_at_t] = pm_t_next[state_at_t] - max_val;
                    pm_t_next[state_at_t] = -MAXLOG;
                }
        }

    return t;
}


int Viterbi_Decoder_Sbas::do_traceback(size_t traceback_length)
{
    // traceback_length is in bits
    int state;
    std::deque<Prev>::iterator it;

    VLOG(FLOW) << "do_traceback(): traceback_length=" << traceback_length << '\n';

    if (d_trellis_paths.size() < traceback_length)
        {
            traceback_length = d_trellis_paths.size();
        }

    state = 0;  // maybe start not at state 0, but at state with best metric
    for (it = d_trellis_paths.begin(); it < d_trellis_paths.begin() + traceback_length; ++it)
        {
            state = it->get_anchestor_state_of_current_state(state);
        }
    return state;
}


int Viterbi_Decoder_Sbas::do_tb_and_decode(int traceback_length, int requested_decoding_length, int state, int output_u_int[], float& indicator_metric)
{
    int n_of_branches_for_indicator_metric = 500;
    std::deque<Prev>::iterator it;
    int n_im = 0;

    VLOG(FLOW) << "do_tb_and_decode(): requested_decoding_length=" << requested_decoding_length;
    // decode only decode_length bits -> overstep newer bits which are too much
    const int decoding_length_mismatch = static_cast<int>(d_trellis_paths.size()) - (traceback_length + requested_decoding_length);
    VLOG(BLOCK) << "decoding_length_mismatch=" << decoding_length_mismatch;
    const int overstep_length = decoding_length_mismatch >= 0 ? decoding_length_mismatch : 0;
    VLOG(BLOCK) << "overstep_length=" << overstep_length;

    for (it = d_trellis_paths.begin() + traceback_length;
         it < d_trellis_paths.begin() + traceback_length + overstep_length; ++it)
        {
            state = it->get_anchestor_state_of_current_state(state);
        }
    int t_out = static_cast<int>(d_trellis_paths.end() - (d_trellis_paths.begin() + traceback_length + overstep_length) - 1);  // requested_decoding_length-1;
    indicator_metric = 0;
    for (it = d_trellis_paths.begin() + traceback_length + overstep_length; it < d_trellis_paths.end(); ++it)
        {
            if (it - (d_trellis_paths.begin() + traceback_length + overstep_length) < n_of_branches_for_indicator_metric)
                {
                    n_im++;
                    indicator_metric += it->get_metric_of_current_state(state);
                    VLOG(SAMPLE) << "@t=" << it->get_t() << " b=" << it->get_bit_of_current_state(state) << " sm=" << indicator_metric << " d=" << it->get_metric_of_current_state(state);
                }
            output_u_int[t_out] = it->get_bit_of_current_state(state);
            state = it->get_anchestor_state_of_current_state(state);
            t_out--;
        }
    if (n_im > 0)
        {
            indicator_metric /= static_cast<float>(n_im);
        }

    VLOG(BLOCK) << "indicator metric: " << indicator_metric;
    // remove old states
    if (d_trellis_paths.begin() + traceback_length + overstep_length <= d_trellis_paths.end())
        {
            d_trellis_paths.erase(d_trellis_paths.begin() + traceback_length + overstep_length, d_trellis_paths.end());
        }
    return decoding_length_mismatch;
}


/* function Gamma()

 Description: Computes the branch metric used for decoding.

 Output parameters:
 (returned float)     The metric between the hypothetical symbol and the recevieved vector

 Input parameters:
 rec_array            The received vector, of length nn
 symbol                The hypothetical symbol
 nn                    The length of the received vector

 This function is used by siso()  */
float Viterbi_Decoder_Sbas::gamma(const float rec_array[], int symbol, int nn)
{
    float rm = 0;
    int i;
    unsigned int mask = 1U;
    float txsym;

    for (i = 0; i < nn; i++)
        {
            // if (symbol & mask) rm += rec_array[nn - i - 1];
            txsym = symbol & mask ? 1 : -1;
            rm += txsym * rec_array[nn - i - 1];
            mask = mask << 1U;
        }
    // rm = rm > 50 ? rm : -1000;

    return rm;
}


/* function that creates the transit and output vectors */
void Viterbi_Decoder_Sbas::nsc_transit(int output_p[], int trans_p[], int input, const int g[],
    int KK, int nn)
{
    int nextstate[1];
    int state;
    const int states = static_cast<int>(1U << (KK - 1)); /* The number of states: 2^mm */

    /* Determine the output and next state for each possible starting state */
    for (state = 0; state < states; state++)
        {
            output_p[state] = nsc_enc_bit(nextstate, input, state, g, KK, nn);
            trans_p[state] = nextstate[0];
        }
}


/* Function nsc_enc_bit()

 Description: Convolutionally encodes a single bit using a rate 1/n encoder.
 Takes in one input bit at a time, and produces a n-bit output.

 Input parameters:
 input        The input data bit (i.e. a 0 or 1).
 state_in    The starting state of the encoder (an int from 0 to 2^m-1).
 g[]            An n-element vector containing the code generators in binary form.
 KK            The constraint length of the convolutional code.
 nn            number of symbols bits per input bits (rate 1/nn)

 Output parameters:
 output_p[]        An n-element vector containing the encoded bits.
 state_out_p[]    An integer containing the final state of the encoder
 (i.e. the state after encoding this bit)

 This function is used by rsc_encode(), nsc_transit(), rsc_transit(), and nsc_transit() */
int Viterbi_Decoder_Sbas::nsc_enc_bit(int state_out_p[], int input, int state_in,
    const int g[], int KK, int nn)
{
    /* declare variables */
    int state;
    int i;
    int out = 0;

    /* create a word made up of state and new input */
    state = (input << (KK - 1)) ^ state_in;

    /* AND the word with the generators */
    for (i = 0; i < nn; i++)
        {
            /* update output symbol */
            out = (out << 1) + parity_counter(state & g[i], KK);
        }

    /* shift the state to make the new state */
    state_out_p[0] = state >> 1;
    return (out);
}


/* function parity_counter()

 Description: Determines if a symbol has odd (1) or even (0) parity

 Output parameters:
 (returned int): The symbol's parity = 1 for odd and 0 for even

 Input parameters:
 symbol:  The integer-valued symbol
 length:  The highest bit position in the symbol

 This function is used by nsc_enc_bit(), rsc_enc_bit(), and rsc_tail()  */
int Viterbi_Decoder_Sbas::parity_counter(int symbol, int length)
{
    int counter;
    unsigned int temp_parity = 0;
    for (counter = 0; counter < length; counter++)
        {
            temp_parity = temp_parity ^ (symbol & 1U);
            symbol = symbol >> 1U;
        }
    return static_cast<int>(temp_parity);
}


// prev helper class
Viterbi_Decoder_Sbas::Prev::Prev(int states,
    int tt) : num_states(states),
              t(tt),
              refcount(1)
{
    state = std::vector<int>(num_states);
    v_bit = std::vector<int>(num_states);
    v_metric = std::vector<float>(num_states);
}


// copy constructor
Viterbi_Decoder_Sbas::Prev::Prev(const Prev& prev) : num_states(prev.num_states),
                                                     v_metric(prev.v_metric),
                                                     state(prev.state),
                                                     v_bit(prev.v_bit),
                                                     t(prev.t),
                                                     refcount(prev.refcount)
{
    refcount++;
    VLOG(LMORE) << "Prev("
                << "?"
                << ", " << t << ")"
                << " copy, new refcount = " << refcount;
}


// assignment constructor
Viterbi_Decoder_Sbas::Prev& Viterbi_Decoder_Sbas::Prev::operator=(const Prev& other)
{
    // check for self-assignment
    if (&other == this)
        {
            return *this;
        }

    // handle old resources
    if (refcount != 1)
        {  // this object is not anymore using them
            refcount--;
        }

    // increase ref counter for this resource set
    refcount = other.refcount;
    refcount++;

    // take over resources
    t = other.t;
    state = other.state;
    v_bit = other.v_bit;
    v_metric = other.v_metric;

    VLOG(LMORE) << "Prev("
                << "?"
                << ", " << t << ")"
                << " assignment, new refcount = " << refcount;
    return *this;
}


Viterbi_Decoder_Sbas::Prev::~Prev()
{
    if (refcount != 1)
        {
            refcount--;
            VLOG(LMORE) << "~Prev("
                        << "?"
                        << ", " << t << ")"
                        << " destructor after copy, new refcount = " << refcount;
        }
}


int Viterbi_Decoder_Sbas::Prev::get_anchestor_state_of_current_state(int current_state) const
{
    // std::cout << "get prev state: for state " << current_state << " at time " << t << ", the prev state at time " << t - 1 << " is " << state[current_state] << '\n';
    if (num_states > current_state)
        {
            return state[current_state];
        }
    // std::cout << "alarm " << "num_states=" << num_states << " current_state=" << current_state << '\n';
    // return state[current_state];
    return 0;
}


int Viterbi_Decoder_Sbas::Prev::get_bit_of_current_state(int current_state) const
{
    // std::cout << "get prev bit  : for state " << current_state << " at time " << t << ", the send bit is " << bit[current_state] << '\n';
    if (num_states > current_state)
        {
            return v_bit[current_state];
        }
    return 0;
}


float Viterbi_Decoder_Sbas::Prev::get_metric_of_current_state(int current_state) const
{
    if (num_states > current_state)
        {
            return v_metric[current_state];
        }
    return 0;
}


int Viterbi_Decoder_Sbas::Prev::get_t() const
{
    return t;
}


void Viterbi_Decoder_Sbas::Prev::set_current_state_as_ancestor_of_next_state(int next_state, int current_state)
{
    if (num_states > next_state)
        {
            state[next_state] = current_state;
        }
}


void Viterbi_Decoder_Sbas::Prev::set_decoded_bit_for_next_state(int next_state, int bit)
{
    if (num_states > next_state)
        {
            this->v_bit[next_state] = bit;
        }
}


void Viterbi_Decoder_Sbas::Prev::set_survivor_branch_metric_of_next_state(int next_state, float metric)
{
    if (num_states > next_state)
        {
            this->v_metric[next_state] = metric;
        }
}
