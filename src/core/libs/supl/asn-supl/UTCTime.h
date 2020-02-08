/*-
 * SPDX-FileCopyrightText: (c) 2003, 2004 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * SPDX-License-Identifier: BSD-1-Clause
 */
#ifndef _UTCTime_H
#define _UTCTime_H

#include <OCTET_STRING.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef OCTET_STRING_t UTCTime_t; /* Implemented via OCTET STRING */

    extern asn_TYPE_descriptor_t asn_DEF_UTCTime;

    asn_struct_print_f UTCTime_print;
    asn_constr_check_f UTCTime_constraint;
    xer_type_encoder_f UTCTime_encode_xer;

    /***********************
     * Some handy helpers. *
     ***********************/

    struct tm; /* <time.h> */

    /* See asn_GT2time() in GeneralizedTime.h */
    time_t asn_UT2time(const UTCTime_t * /*st*/, struct tm *_tm, int as_gmt);

    /* See asn_time2GT() in GeneralizedTime.h */
    UTCTime_t *asn_time2UT(UTCTime_t *__opt_ut, const struct tm * /*tm*/,
        int force_gmt);

#ifdef __cplusplus
}
#endif

#endif /* _UTCTime_H_ */
