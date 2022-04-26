/*-
 * SPDX-FileCopyrightText: (c) 2003, 2004 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * SPDX-License-Identifier: BSD-1-Clause
 */
/*
 * This type differs from the standard ENUMERATED in that it is modelled using
 * the fixed machine type (long, int, short), so it can hold only values of
 * limited length. There is no type (i.e., NativeEnumerated_t, any integer type
 * will do).
 * This type may be used when integer range is limited by subtype constraints.
 */
#ifndef _NativeEnumerated_H
#define _NativeEnumerated_H

#include <NativeInteger.h>

#ifdef __cplusplus
extern "C"
{
#endif

    extern asn_TYPE_descriptor_t asn_DEF_NativeEnumerated;

    xer_type_encoder_f NativeEnumerated_encode_xer;
    per_type_decoder_f NativeEnumerated_decode_uper;
    per_type_encoder_f NativeEnumerated_encode_uper;

#ifdef __cplusplus
}
#endif

#endif /* _NativeEnumerated_H_ */
