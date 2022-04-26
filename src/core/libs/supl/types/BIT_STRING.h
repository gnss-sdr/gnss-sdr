/*-
 * SPDX-FileCopyrightText: (c) 2003, 2004 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * SPDX-License-Identifier: BSD-1-Clause
 */
#ifndef _BIT_STRING_H
#define _BIT_STRING_H

#include <OCTET_STRING.h> /* Some help from OCTET STRING */

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct BIT_STRING_s
    {
        uint8_t *buf; /* BIT STRING body */
        int size;     /* Size of the above buffer */

        int bits_unused; /* Unused trailing bits in the last octet (0..7) */

        asn_struct_ctx_t _asn_ctx; /* Parsing across buffer boundaries */
    } BIT_STRING_t;

    extern asn_TYPE_descriptor_t asn_DEF_BIT_STRING;

    asn_struct_print_f BIT_STRING_print; /* Human-readable output */
    asn_constr_check_f BIT_STRING_constraint;
    xer_type_encoder_f BIT_STRING_encode_xer;

#ifdef __cplusplus
}
#endif

#endif /* _BIT_STRING_H_ */
