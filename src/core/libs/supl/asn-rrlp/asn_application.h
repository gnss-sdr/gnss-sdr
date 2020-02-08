/*-
 * SPDX-FileCopyrightText: (c) 2003, 2004 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * SPDX-License-Identifier: BSD-1-Clause
 */
/*
 * Application-level ASN.1 callbacks.
 */
#ifndef _ASN_APPLICATION_H
#define _ASN_APPLICATION_H

// clang-format off
#include "asn_system.h" /* for platform-dependent types */
#include "asn_codecs.h" /* for ASN.1 codecs specifics */
// clang-format on

#ifdef __cplusplus
extern "C"
{
#endif

    /*
     * Generic type of an application-defined callback to return various
     * types of data to the application.
     * EXPECTED RETURN VALUES:
     *  -1: Failed to consume bytes. Abort the mission.
     * Non-negative return values indicate success, and ignored.
     */
    typedef int(asn_app_consume_bytes_f)(const void *buffer, size_t size,
        void *application_specific_key);

    /*
     * A callback of this type is called whenever constraint validation fails
     * on some ASN.1 type. See "constraints.h" for more details on constraint
     * validation.
     * This callback specifies a descriptor of the ASN.1 type which failed
     * the constraint check, as well as human readable message on what
     * particular constraint has failed.
     */
    typedef void(asn_app_constraint_failed_f)(
        void *application_specific_key,
        struct asn_TYPE_descriptor_s *type_descriptor_which_failed,
        const void *structure_which_failed_ptr,
        const char *error_message_format, ...) GCC_PRINTFLIKE(4, 5);

#ifdef __cplusplus
}
#endif

#include "constr_TYPE.h" /* for asn_TYPE_descriptor_t */

#endif /* _ASN_APPLICATION_H_ */
