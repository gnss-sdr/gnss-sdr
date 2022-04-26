/*-
 * SPDX-FileCopyrightText: (c) 2003, 2004 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * SPDX-License-Identifier: BSD-1-Clause
 */
#ifndef _INTEGER_H
#define _INTEGER_H

#include <asn_application.h>
#include <asn_codecs_prim.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef ASN__PRIMITIVE_TYPE_t INTEGER_t;

    extern asn_TYPE_descriptor_t asn_DEF_INTEGER;

    /* Map with <tag> to integer value association */
    typedef struct asn_INTEGER_enum_map_s
    {
        int64_t nat_value;     /* associated native integer value */
        size_t enum_len;       /* strlen("tag") */
        const char *enum_name; /* "tag" */
    } asn_INTEGER_enum_map_t;

    /* This type describes an enumeration for INTEGER and ENUMERATED types */
    typedef struct asn_INTEGER_specifics_s
    {
        asn_INTEGER_enum_map_t *value2enum; /* N -> "tag"; sorted by N */
        unsigned int *enum2value;           /* "tag" => N; sorted by tag */
        int map_count;                      /* Elements in either map */
        int extension;                      /* This map is extensible */
        int strict_enumeration;             /* Enumeration set is fixed */
        int field_width;                    /* Size of native integer */
        int field_unsigned;                 /* Signed=0, unsigned=1 */
    } asn_INTEGER_specifics_t;

    asn_struct_print_f INTEGER_print;
    ber_type_decoder_f INTEGER_decode_ber;
    der_type_encoder_f INTEGER_encode_der;
    xer_type_decoder_f INTEGER_decode_xer;
    xer_type_encoder_f INTEGER_encode_xer;
    per_type_decoder_f INTEGER_decode_uper;
    per_type_encoder_f INTEGER_encode_uper;

    /***********************************
     * Some handy conversion routines. *
     ***********************************/

    /*
     * Returns 0 if it was possible to convert, -1 otherwise.
     * -1/EINVAL: Mandatory argument missing
     * -1/ERANGE: Value encoded is out of range for long representation
     * -1/ENOMEM: Memory allocation failed (in asn_long2INTEGER()).
     */
    int asn_INTEGER2long(const INTEGER_t *i, int64_t *l);
    int asn_INTEGER2ulong(const INTEGER_t *i, uint64_t *l);
    int asn_long2INTEGER(INTEGER_t *st, int64_t value);
    int asn_ulong2INTEGER(INTEGER_t *st, uint64_t value);

    /*
     * Convert the integer value into the corresponding enumeration map entry.
     */
    const asn_INTEGER_enum_map_t *INTEGER_map_value2enum(
        asn_INTEGER_specifics_t *specs, int64_t value);

#ifdef __cplusplus
}
#endif

#endif /* _INTEGER_H_ */
