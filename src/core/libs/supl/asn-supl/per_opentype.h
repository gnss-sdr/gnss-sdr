/*
 * SPDX-FileCopyrightText: (c) 2003, 2004 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * SPDX-License-Identifier: BSD-1-Clause
 */
#ifndef _PER_OPENTYPE_H
#define _PER_OPENTYPE_H

#ifdef __cplusplus
extern "C"
{
#endif

    asn_dec_rval_t uper_open_type_get(asn_codec_ctx_t *opt_codec_ctx,
        asn_TYPE_descriptor_t *td,
        asn_per_constraints_t *constraints,
        void **sptr, asn_per_data_t *pd);

    int uper_open_type_skip(asn_codec_ctx_t *opt_codec_ctx, asn_per_data_t *pd);

    int uper_open_type_put(asn_TYPE_descriptor_t *td,
        asn_per_constraints_t *constraints, void *sptr,
        asn_per_outp_t *po);

#ifdef __cplusplus
}
#endif

#endif /* _PER_OPENTYPE_H_ */
