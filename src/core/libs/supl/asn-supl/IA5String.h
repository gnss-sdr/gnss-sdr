/*-
 * SPDX-FileCopyrightText: (c) 2003, 2004 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * SPDX-License-Identifier: BSD-1-Clause
 */
#ifndef _IA5String_H
#define _IA5String_H

#include <OCTET_STRING.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef OCTET_STRING_t IA5String_t; /* Implemented via OCTET STRING */

    /*
     * IA5String ASN.1 type definition.
     */
    extern asn_TYPE_descriptor_t asn_DEF_IA5String;

    asn_constr_check_f IA5String_constraint;

#ifdef __cplusplus
}
#endif

#endif /* _IA5String_H_ */
