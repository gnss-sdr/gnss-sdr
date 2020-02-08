/*-
 * SPDX-FileCopyrightText: (c) 2003, 2004 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * SPDX-License-Identifier: BSD-1-Clause
 */
#ifndef _CONSTR_SEQUENCE_OF_H
#define _CONSTR_SEQUENCE_OF_H

#include <asn_application.h>
#include <constr_SET_OF.h> /* Implemented using SET OF */

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * A set specialized functions dealing with the SEQUENCE OF type.
 * Generally implemented using SET OF.
 */
#define SEQUENCE_OF_free SET_OF_free
#define SEQUENCE_OF_print SET_OF_print
#define SEQUENCE_OF_constraint SET_OF_constraint
#define SEQUENCE_OF_decode_ber SET_OF_decode_ber
#define SEQUENCE_OF_decode_xer SET_OF_decode_xer
#define SEQUENCE_OF_decode_uper SET_OF_decode_uper
    der_type_encoder_f SEQUENCE_OF_encode_der;
    xer_type_encoder_f SEQUENCE_OF_encode_xer;
    per_type_encoder_f SEQUENCE_OF_encode_uper;

#ifdef __cplusplus
}
#endif

#endif /* _CONSTR_SET_OF_H_ */
