/*!
 * \file convolutional.h
 * \brief General functions used to implement convolutional encoding.
 * \author Matthew C. Valenti
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2006-2008  Matthew C. Valenti
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
 *
 * This file is a derived work of the original file, which had this note:
 *
 * Last updated on May 22, 2008
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
//#ifndef GNSS_SDR_CONVOLUTIONAL_H_
//#define GNSS_SDR_CONVOLUTIONAL_H_
/* define constants used throughout the library */
#define MAXLOG 1e7  /* Define infinity */


/*!
 * \brief Converts an integer symbol into a vector of bits
 *
 * \param[out] binvec_p The binary vector
 * \param[in] symbol  The integer-valued symbol
 * \param[in] length  The length of the binary vector
 *
 * This function is used by conv_encode()
 */
static void itob(int binvec_p[], int symbol, int length)
{
    int counter;
    /* Go through each bit in the vector */
    for (counter = 0; counter < length; counter++)
        {
            binvec_p[length - counter - 1] = (symbol & 1);
            symbol = symbol >> 1;
        }
    return;
}



/*!
 * \brief Determines if a symbol has odd (1) or even (0) parity
 *	Output parameters:
 * \return (returned int): The symbol's parity = 1 for odd and 0 for even
 *
 * \param[in] symbol  The integer-valued symbol
 * \param[in] length  The highest bit position in the symbol
 *
 * This function is used by nsc_enc_bit(), rsc_enc_bit(), and rsc_tail()
 */
static int parity_counter(int symbol, int length)
{
    int counter;
    int temp_parity = 0;

    for (counter = 0; counter <  length; counter++)
        {
            temp_parity = temp_parity^(symbol & 1);
            symbol = symbol >> 1;
        }
    return(temp_parity);
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
 *					(i.e. the state after encoding this bit)
 *
 * This function is used by rsc_encode(), nsc_transit(), rsc_transit(), and nsc_transit()
 */
static int nsc_enc_bit(int state_out_p[],
                       int input,
                       int state_in,
                       int g[],
                       int KK,
                       int nn)
{
    /* declare variables */
    int state, i;
    int out_ = 0;

    /* create a word made up of state and new input */
    state = (input << (KK - 1))^state_in;

    /* AND the word with the generators */
    for (i = 0; i < nn; i++)
        {
            /* update output symbol */
            out_ = (out_ << 1) + parity_counter(state & g[i], KK);
        }

    /* shift the state to make the new state */
    state_out_p[0] = state >> 1;
    return(out_);
}



/*!
 * \brief  like nsc_enc_bit() but for a RSC code
 */
static int rsc_enc_bit(int state_out_p[],
                       int input,
                       int state_in,
                       int g[],
                       int KK,
                       int nn)
{
    /* declare variables */
    int state, i, out_, a_k;

    /* systematic output */
    out_ = input;

    /* determine feedback bit */
    a_k = input^parity_counter(g[0]&state_in, KK);

    /* create a word made up of state and feedback bit */
    state = (a_k << (KK - 1))^state_in;

    /* AND the word with the generators */
    for (i = 1; i < nn; i++)
        {
            /* update output symbol */
            out_ = (out_ << 1) + parity_counter(state & g[i], KK);
        }

    /* shift the state to make the new state */
    state_out_p[0] = state >> 1;
    return(out_);
}



/*!
 * \brief Function that creates the transit and output vectors
 */
static void nsc_transit(int output_p[],
                        int trans_p[],
                        int input,
                        int g[],
                        int KK,
                        int nn)
{
    int nextstate[1];
    int state, states;
    states = (1 << (KK - 1));  /* The number of states: 2^mm */

    /* Determine the output and next state for each possible starting state */
    for(state = 0; state < states; state++)
        {
            output_p[state]  = nsc_enc_bit(nextstate, input, state, g, KK, nn);
            trans_p[state]  = nextstate[0];
        }
    return;
}



/*!
 * \brief Calculates the "transition matrix" for the trellis.
 * This information tells the decoder what the next state and output bits
 * will be given the current state and input bit.
 *
 * \param[in] input    Either 0 or 1 --- the input data bit.
 * \param[in] g[]      A two element vector containing the code generators.
 * \param[in] KK       The constraint length of the convolutional code.
 * \param[out] output_p[]  A vector of length max_states = 2^(KK-1) containing
 * 		                the output symbols.
 * \param[out] trans_p[]   A vector of length max_states that tells the decoder
 *				what the next state will be given the input and current state.
 *
 *  This function is used by turbo_decode()
 */
static void rsc_transit(int output_p[],
                        int trans_p[],
                        int input,
                        int g[],
                        int KK,
                        int nn )
{
    int nextstate[1];
    int state, states;

    states = 1 << (KK - 1); // The number of states: 2^mm

    // Determine the output and next state for each possible starting state
    for(state = 0; state < states; state++)
        {
            output_p[state] = rsc_enc_bit( nextstate, input, state, g, KK, nn );
            trans_p[state]  = nextstate[0];
        }
    return;
}



/*!
 * \brief determines the tail for a RSC code
 */
static void rsc_tail(int tail_p[],
                     int g[],
                     int max_states,
                     int mm )
{
    int state;

    /* Determine the tail for each state */
    for(state = 0; state < max_states; state++)
        {
            /* determine feedback word */
            tail_p[state] = parity_counter(g[0]&state, mm);
        }
    return;
}



/*!
 * \brief Perform convolutional encoding
 */
static void conv_encode(int output_p[],
                        int input[],
                        int out0[],
                        int state0[],
                        int out1[],
                        int state1[],
                        int tail[],
                        int KK,
                        int LL,
                        int nn)
{
    int i, j, outsym;
    int *bin_vec;
    int state = 0;

    /* Negative value in "tail" is a flag that this is
       a tail-biting NSC code.  Determine initial state */

    if ( tail[0] < 0 )
        {
            for (i = LL - KK + 1; i < LL; i++)
                {
                    if (input[i])
                        {
                            /* Determine next state */
                            state = state1[state];
                        }
                    else
                        {
                            /* Determine next state */
                            state = state0[state];
                        }
                }
        }

    bin_vec = (int*)calloc( nn, sizeof(int) );

    /* encode data bits one bit at a time */
    for (i = 0; i < LL; i++)
        {
            if (input[i])
                {
                    /* Input is a one */
                    outsym = out1[state];  /* The output symbol */

                    /* Determine next state */
                    state = state1[state];
                }
            else
                {
                    /* Input is a zero */
                    outsym = out0[state];  /* The output symbol */

                    /* Determine next state */
                    state = state0[state];
                }

            /* Convert symbol to a binary vector */
            itob( bin_vec, outsym, nn );

            /* Assign to output */
            for (j = 0; j < nn; j++)
                output_p[nn*i + j] = bin_vec[j];
        }

    /* encode tail if needed */
    if (tail[0] >= 0)
        {
            for (i = LL; i < LL + KK - 1; i++)
                {
                    if (tail[state])
                        {
                            /* Input is a one */
                            outsym = out1[state];  /* The output symbol */

                            /* Determine next state */
                            state = state1[state];
                        }
                    else
                        {
                            /* Input is a zero */
                            outsym = out0[state];  /* The output symbol */

                            /* Determine next state */
                            state = state0[state];
                        }

                    /* Convert symbol to a binary vector	*/
                    itob( bin_vec, outsym, nn );

                    /* Assign to output */
                    for (j = 0; j < nn; j++)
                        output_p[nn*i + j] = bin_vec[j];
                }
        }

    free(bin_vec);
    return;
}


/*!
 * \brief  Computes the branch metric used for decoding.
 *  \return (returned float) The metric between the hypothetical symbol and the received vector
 *  \param[in] rec_array     The received vector, of length nn
 *  \param[in] symbol        The hypothetical symbol
 *  \param[in] nn            The length of the received vector
 *
 * This function is used by siso()
 */
static float Gamma(float  rec_array[],
                   int symbol,
                   int nn)
{
    float rm = 0;
    int i;
    int mask = 1;

    for (i = 0; i < nn; i++)
        {
            if (symbol & mask)
                rm += rec_array[nn - i - 1];
            mask = mask << 1;
        }
    return(rm);
}


/*!
 * \brief Uses the Viterbi algorithm to perform hard-decision decoding of a convolutional code.
 * \param[in] out0[]    The output bits for each state if input is a 0 (generated by rsc_transit).
 * \param[in] state0[]  The next state if input is a 0 (generated by rsc_transit).
 * \param[in] out1[]    The output bits for each state if input is a 1 (generated by rsc_transit).
 * \param[in] state1[]  The next state if input is a 1 (generated by rsc_transit).
 * \param[in] r[]       The received signal in LLR-form. For BPSK, must be in form r = 2*a*y/(sigma^2).
 * \param[in] KK        The constraint length of the convolutional code.
 * \param[in] LL        The number of data bits.
 * \param[out] output_u_int[]    Hard decisions on the data bits
 *
 */
static void Viterbi(int output_u_int[],
                    int out0[],
                    int state0[],
                    int out1[],
                    int state1[],
                    double input_c[],
                    int KK,
                    int nn,
                    int LL)
{
    int i, t, state, mm, states;
    int number_symbols;
    float metric;
    float *prev_section, *next_section;
    int *prev_bit;
    int *prev_state;
    float *metric_c;	/* Set of all possible branch metrics */
    float *rec_array;   /* Received values for one trellis section */
    float max_val;

    /* some derived constants */
    mm = KK - 1;
    states = 1 << mm;          /* 2^mm */
    number_symbols = 1 << nn;  /* 2^nn */

    /* dynamically allocate memory */
    prev_section = (float*)calloc( states, sizeof(float) );
    next_section = (float*)calloc( states, sizeof(float) );
    prev_bit = (int*)calloc( states*(LL + mm), sizeof(int) );
    prev_state = (int*)calloc( states*(LL + mm), sizeof(int) );
    rec_array = (float*)calloc( nn, sizeof(float) );
    metric_c = (float*)calloc( number_symbols, sizeof(float) );

    /* initialize trellis */
    for (state = 0; state < states; state++)
        {
            prev_section[state] = -MAXLOG;
            next_section[state] = -MAXLOG;
        }
    prev_section[0] = 0; /* start in all-zeros state */

    /* go through trellis */
    for (t = 0; t < LL + mm; t++)
        {
            for (i = 0; i < nn; i++)
        	    rec_array[i] = (float)input_c[nn*t + i];

            /* precompute all possible branch metrics */
            for (i = 0; i < number_symbols; i++)
                metric_c[i] = Gamma( rec_array, i, nn );

            /* step through all states */
            for (state = 0; state < states; state++)
                {
                    /* hypothesis: info bit is a zero */
                    metric = prev_section[state] + metric_c[ out0[ state ] ];

                    /* store new metric if more than metric in storage */
                    if ( metric > next_section[state0[state]] )
                        {
                            next_section[state0[state]] = metric;
                            prev_state[t*states + state0[state]] = state;
                            prev_bit[t*states + state0[state]] = 0;
                        }

                    /* hypothesis: info bit is a one */
                    metric = prev_section[state] + metric_c[ out1[ state ] ];

                    /* store new metric if more than metric in storage */
                    if ( metric > next_section[state1[state]] )
                        {
                            next_section[state1[state]] = metric;
                            prev_state[t*states + state1[state]] = state;
                            prev_bit[t*states + state1[state]] = 1;
                        }
                }

            /* normalize */
            max_val = 0;
            for (state = 0; state < states; state++)
                {
                    if (next_section[state] > max_val)
                        {
                            max_val = next_section[state];
                        }
                }
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
            state = prev_state[t*states + state];
        }

    for (t = LL - 1; t >= 0; t--)
        {
            output_u_int[t] = prev_bit[t*states + state];
            state = prev_state[t*states + state];
        }

    /* free the dynamically allocated memory */
    free(prev_section);
    free(next_section);
    free(prev_bit);
    free(prev_state);
    free(rec_array);
    free(metric_c);
}


/*!
 * \brief  Uses the Viterbi algorithm to perform hard-decision decoding of a tail-biting convolutional code.
 *	Input parameters:
 *		out0[]		The output bits for each state if input is a 0 (generated by rsc_transit).
 *		state0[]	The next state if input is a 0 (generated by rsc_transit).
 *		out1[]		The output bits for each state if input is a 1 (generated by rsc_transit).
 *		state1[]	The next state if input is a 1 (generated by rsc_transit).
 *		r[]		The received signal in LLR-form. For BPSK, must be in form r = 2*a*y/(sigma^2).
 *		KK		The constraint length of the convolutional code.
 *		LL		The number of data bits.
 *		depth		head and tail decoding length [Ref. W. Sung, Electronics Letters, vol. 36, no. 7]
 *	Output parameters:
 *		output_u_int[]		Hard decisions on the data bits
 */
static void ViterbiTb(int output_u_int[],
                      int out0[],
                      int state0[],
                      int out1[],
                      int state1[],
                      double input_c[],
                      int KK,
                      int nn,
                      int LL,
                      int depth)
{
    int i, t, state, mm, states, max_state;
    int number_symbols, starting_bit;
    float metric;
    float *prev_section, *next_section;
    int *prev_bit;
    int *prev_state;
    float *metric_c;	/* Set of all possible branch metrics */
    float *rec_array;   /* Received values for one trellis section */
    float max_val;

    /* some derived constants */
    mm = KK - 1;
    states = 1 << mm;			/* 2^mm */
    number_symbols = 1 << nn;	    /* 2^nn */

    /* dynamically allocate memory */
    prev_section = (float*)calloc( states, sizeof(float) );
    next_section = (float*)calloc( states, sizeof(float) );
    prev_bit = (int*)calloc( states*(LL + depth), sizeof(int) );
    prev_state = (int*)calloc( states*(LL + depth), sizeof(int) );
    rec_array = (float*)calloc( nn, sizeof(float) );
    metric_c = (float*)calloc( number_symbols, sizeof(float) );

    /* initialize trellis */
    for (state = 0; state < states; state++)
        {
            prev_section[state] = 0; /* equally likely starting state */
            next_section[state] = -MAXLOG;
        }

    /* go through trellis */
    for (t = -depth; t < LL + depth; t++)
        {
            /* determine the corresponding data bits */
            starting_bit = nn*(t % LL);
            if (starting_bit < 0 )
                starting_bit = nn*LL + starting_bit;

            for (i = 0; i < nn; i++)
                {
                    rec_array[i] = (float)input_c[starting_bit+i];
                }

            /* precompute all possible branch metrics */
            for (i = 0; i < number_symbols; i++)
                metric_c[i] = Gamma( rec_array, i, nn );

            /* step through all states */
            for (state = 0; state < states; state++)
                {
                    /* hypothesis: info bit is a zero */
                    metric = prev_section[state] + metric_c[ out0[ state ] ];

                    /* store new metric if more than metric in storage */
                    if ( metric > next_section[state0[state]] )
                        {
                            next_section[state0[state]] = metric;
                            if (t >= 0)
                                {
                                    prev_state[t*states+state0[state]] = state;
                                    prev_bit[t*states+state0[state]] = 0;
                                }
                        }

                    /* hypothesis: info bit is a one */
                    metric = prev_section[state] + metric_c[ out1[ state ] ];

                    /* store new metric if more than metric in storage */
                    if ( metric > next_section[state1[state]] )
                        {
                            next_section[state1[state]] = metric;
                            if (t >= 0)
                                {
                                    prev_state[t*states+state1[state]] = state;
                                    prev_bit[t*states+state1[state]] = 1;
                                }
                        }
                }

            /* normalize */
            max_val = 0;
            for (state = 0; state < states; state++)
                {
                    if (next_section[state] > max_val)
                        {
                            max_val = next_section[state];
                            max_state = state;
                        }
                }
            for (state = 0; state < states; state++)
                {
                    prev_section[state] = next_section[state] - max_val;
                    next_section[state] = -MAXLOG;
                }
        }

    /* trace-back operation */
    state = max_state;

    /* tail, no need to output */
    for (t = LL + depth - 1; t >= LL; t--)
        {
            state = prev_state[t*states + state];
        }

    for (t = LL - 1; t >= 0; t--)
        {
            output_u_int[t] = prev_bit[t*states + state];
            state = prev_state[t*states + state];
        }

    /* free the dynamically allocated memory */
    free(prev_section);
    free(next_section);
    free(prev_bit);
    free(prev_state);
    free(rec_array);
    free(metric_c);
}
//#endif
