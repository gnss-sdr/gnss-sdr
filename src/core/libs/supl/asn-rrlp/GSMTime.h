/*
 * SPDX-FileCopyrightText: (c) 2003, 2004 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * SPDX-License-Identifier: BSD-1-Clause
 * Generated by asn1c-0.9.22 (http://lionet.info/asn1c)
 * From ASN.1 module "RRLP-Components"
 *     found in "../rrlp-components.asn"
 */

#ifndef _GSMTime_H
#define _GSMTime_H

#include <asn_application.h>

/* Including external dependencies */
#include "BCCHCarrier.h"
#include "BSIC.h"
#include "BitNumber.h"
#include "FrameNumber.h"
#include "TimeSlot.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* GSMTime */
    typedef struct GSMTime
    {
        BCCHCarrier_t bcchCarrier;
        BSIC_t bsic;
        FrameNumber_t frameNumber;
        TimeSlot_t timeSlot;
        BitNumber_t bitNumber;

        /* Context for parsing across buffer boundaries */
        asn_struct_ctx_t _asn_ctx;
    } GSMTime_t;

    /* Implementation */
    extern asn_TYPE_descriptor_t asn_DEF_GSMTime;

#ifdef __cplusplus
}
#endif

#endif /* _GSMTime_H_ */
#include <asn_internal.h>
