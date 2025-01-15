/*!
 * \file fec.h
 * \brief Utilities for the convolutional encoder of the libswiftnav library
 * \author Phil Karn, KA9Q
 *
 * -----------------------------------------------------------------------------
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * This file was originally borrowed from libswiftnav
 * <https://github.com/swift-nav/libswiftnav>,
 * a portable C library implementing GNSS related functions and algorithms,
 * and then modified by J. Arribas and C. Fernandez
 *
 * Copyright (C) 2004, Phil Karn, KA9Q
 *
 * SPDX-License-Identifier: LGPL-3.0-only
 *
 */


#ifndef GNSS_SDR_FEC_H
#define GNSS_SDR_FEC_H

/* r=1/2 k=7 convolutional encoder polynomials
 * The NASA-DSN convention is to use V27POLYA inverted, then V27POLYB
 * The CCSDS/NASA-GSFC convention is to use V27POLYB, then V27POLYA inverted
 */
#define V27POLYA 0x4f
#define V27POLYB 0x6d

typedef struct
{
    unsigned char c0[32];
    unsigned char c1[32];
} v27_poly_t;

typedef struct
{
    unsigned int w[2];
} v27_decision_t;

/* State info for instance of r=1/2 k=7 Viterbi decoder
 */
typedef struct
{
    unsigned int metrics1[64]; /* Path metric buffer 1 */
    unsigned int metrics2[64]; /* Path metric buffer 2 */
    /* Pointers to path metrics, swapped on every bit */
    unsigned int *old_metrics, *new_metrics;
    const v27_poly_t *poly;       /* Polynomial to use */
    v27_decision_t *decisions;    /* Beginning of decisions for block */
    unsigned int decisions_index; /* Index of current decision */
    unsigned int decisions_count; /* Number of decisions in history */
} v27_t;

void v27_poly_init(v27_poly_t *poly, const signed char polynomial[2]);

void v27_init(v27_t *v, v27_decision_t *decisions, unsigned int decisions_count,
    const v27_poly_t *poly, unsigned char initial_state);
void v27_update(v27_t *v, const unsigned char *syms, int nbits);
void v27_chainback_fixed(v27_t *v, unsigned char *data, unsigned int nbits,
    unsigned char final_state);
void v27_chainback_likely(v27_t *v, unsigned char *data, unsigned int nbits);

#endif
