/*-
 * SPDX-FileCopyrightText: (c) 2003, 2004 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * SPDX-License-Identifier: BSD-1-Clause
 */
#ifndef _VisibleString_H
#define _VisibleString_H

#include <OCTET_STRING.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef OCTET_STRING_t VisibleString_t; /* Implemented via OCTET STRING */

    extern asn_TYPE_descriptor_t asn_DEF_VisibleString;

    asn_constr_check_f VisibleString_constraint;

#ifdef __cplusplus
}
#endif

#endif /* _VisibleString_H_ */
