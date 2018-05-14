/*!
 * \file viterbi27.c
 * \author Phil Karn, KA9Q
 * \brief K=7 r=1/2 Viterbi decoder in portable C
 *
 * -------------------------------------------------------------------------
 * This file was originally borrowed from libswiftnav
 * <https://github.com/swift-nav/libswiftnav>,
 * a portable C library implementing GNSS related functions and algorithms,
 * and then modified by J. Arribas and C. Fernandez
 *
 * Copyright (C) 2004, Phil Karn, KA9Q
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Lesser Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */


#include <stdlib.h>
#include "fec.h"

static inline int parity(int x)
{
    x ^= x >> 16;
    x ^= x >> 8;
    x ^= x >> 4;
    x &= 0xf;
    return (0x6996 >> x) & 1;
}


/** Initialize a v27_poly_t struct for use with a v27_t decoder.
 *
 * \param poly Structure to initialize.
 * \param polynomial Byte array representing the desired polynomials.
 */
void v27_poly_init(v27_poly_t *poly, const signed char polynomial[2])
{
    int state;

    for(state = 0; state < 32; state++)
        {
            poly->c0[state] = (polynomial[0] < 0) ^ parity((2*state) & abs(polynomial[0])) ? 255 : 0;
            poly->c1[state] = (polynomial[1] < 0) ^ parity((2*state) & abs(polynomial[1])) ? 255 : 0;
        }
}


/** Initialize a v27_t struct for Viterbi decoding.
 *
 * \param v Structure to initialize
 * \param decisions Array of v27_decision_t structs, capacity = decisions_count.
 *                  Must remain valid as long as v is used.
 * \param decisions_count Size of decisions array. Equal to the number of bit
 *                        decisions kept in history.
 * \param poly Struct describing the polynomials to use. Must remain valid as
 *             long as v is used. May be shared between multiple decoders.
 * \param initial_state Initial state of the decoder shift register. Usually zero.
 */
void v27_init(v27_t *v, v27_decision_t *decisions, unsigned int decisions_count,
              const v27_poly_t *poly, unsigned char initial_state)
{
    int i;

    v->old_metrics = v->metrics1;
    v->new_metrics = v->metrics2;
    v->poly = poly;
    v->decisions = decisions;
    v->decisions_index = 0;
    v->decisions_count = decisions_count;

    for(i = 0; i < 64; i++)
        v->old_metrics[i] = 63;

    v->old_metrics[initial_state & 63] = 0; /* Bias known start state */
}


/* C-language butterfly */
#define BFLY(i) {\
        unsigned int metric,m0,m1,decision;\
        metric = (v->poly->c0[i] ^ sym0) + (v->poly->c1[i] ^ sym1);\
        m0 = v->old_metrics[i] + metric;\
        m1 = v->old_metrics[i+32] + (510 - metric);\
        decision = (signed int)(m0-m1) > 0;\
        v->new_metrics[2*i] = decision ? m1 : m0;\
        d->w[i/16] |= decision << ((2*i)&31);\
        m0 -= (metric+metric-510);\
        m1 += (metric+metric-510);\
        decision = (signed int)(m0-m1) > 0;\
        v->new_metrics[2*i+1] = decision ? m1 : m0;\
        d->w[i/16] |= decision << ((2*i+1)&31);\
}

/** Update a v27_t decoder with a block of symbols.
 *
 * \param v Structure to update.
 * \param syms Array of symbols to use. Must contain two symbols per bit.
 *             0xff = strong 1, 0x00 = strong 0.
 * \param nbits Number of bits corresponding to the provided symbols.
 */
void v27_update(v27_t *v, const unsigned char *syms, int nbits)
{
    unsigned char sym0, sym1;
    unsigned int *tmp;
    int normalize = 0;

    while(nbits--)
        {
            v27_decision_t *d = &v->decisions[v->decisions_index];

            d->w[0] = d->w[1] = 0;
            sym0 = *syms++;
            sym1 = *syms++;

            BFLY(0);
            BFLY(1);
            BFLY(2);
            BFLY(3);
            BFLY(4);
            BFLY(5);
            BFLY(6);
            BFLY(7);
            BFLY(8);
            BFLY(9);
            BFLY(10);
            BFLY(11);
            BFLY(12);
            BFLY(13);
            BFLY(14);
            BFLY(15);
            BFLY(16);
            BFLY(17);
            BFLY(18);
            BFLY(19);
            BFLY(20);
            BFLY(21);
            BFLY(22);
            BFLY(23);
            BFLY(24);
            BFLY(25);
            BFLY(26);
            BFLY(27);
            BFLY(28);
            BFLY(29);
            BFLY(30);
            BFLY(31);

            /* Normalize metrics if they are nearing overflow */
            if(v->new_metrics[0] > (1 << 30))
                {
                    int i;
                    unsigned int minmetric = 1 << 31;

                    for(i = 0; i < 64; i++)
                        {
                            if(v->new_metrics[i] < minmetric)
                                minmetric = v->new_metrics[i];
                        }

                    for(i = 0; i < 64; i++)
                        v->new_metrics[i] -= minmetric;

                    normalize += minmetric;
                }

            /* Advance decision index */
            if(++v->decisions_index >= v->decisions_count)
                v->decisions_index = 0;

            /* Swap pointers to old and new metrics */
            tmp = v->old_metrics;
            v->old_metrics = v->new_metrics;
            v->new_metrics = tmp;
        }
}


/** Retrieve the most likely output bit sequence with known final state from
 *  a v27_t decoder.
 *
 * \param v Structure to use.
 * \param data Array used to store output bits, capacity = nbits.
 * \param nbits Number of bits to retrieve.
 * \param final_state Known final state of the decoder shift register.
 */
void v27_chainback_fixed(v27_t *v, unsigned char *data, unsigned int nbits,
                         unsigned char final_state)
{
    int k;
    unsigned int decisions_index = v->decisions_index;

    final_state %= 64;
    final_state <<= 2;

    while(nbits-- != 0)
        {

            /* Decrement decision index */
            decisions_index = (decisions_index == 0) ?
                    v->decisions_count-1 : decisions_index-1;

            v27_decision_t *d = &v->decisions[decisions_index];
            k = (d->w[(final_state >> 2) / 32] >> ((final_state >> 2) % 32)) & 1;
            /* The store into data[] only needs to be done every 8 bits.
             * But this avoids a conditional branch, and the writes will
             * combine in the cache anyway
             */
            data[nbits >> 3] = final_state = (final_state >> 1) | (k << 7);
        }
}


/** Retrieve the most likely output bit sequence with unknown final state from
 *  a v27_t decoder.
 *
 * \param v Structure to use.
 * \param data Array used to store output bits, capacity = nbits.
 * \param nbits Number of bits to retrieve.
 */
void v27_chainback_likely(v27_t *v, unsigned char *data, unsigned int nbits)
{
    /* Determine state with minimum metric */

    int i;
    unsigned int best_metric = 0xffffffff;
    unsigned char best_state = 0;
    for(i = 0; i < 64; i++)
        {
            if(v->new_metrics[i] < best_metric)
                {
                    best_metric = v->new_metrics[i];
                    best_state = i;
                }
        }

    v27_chainback_fixed(v, data, nbits, best_state);
}
