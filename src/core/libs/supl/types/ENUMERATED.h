/*-
 * SPDX-FileCopyrightText: (c) 2003, 2004 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * SPDX-License-Identifier: BSD-1-Clause
 */
#ifndef _ENUMERATED_H
#define _ENUMERATED_H

#include <INTEGER.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef INTEGER_t ENUMERATED_t; /* Implemented via INTEGER */

    extern asn_TYPE_descriptor_t asn_DEF_ENUMERATED;

    per_type_decoder_f ENUMERATED_decode_uper;
    per_type_encoder_f ENUMERATED_encode_uper;

#ifdef __cplusplus
}
#endif

#endif /* _ENUMERATED_H_ */
