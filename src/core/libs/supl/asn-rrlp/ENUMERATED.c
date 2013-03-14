/*-
 * Copyright (c) 2003, 2005, 2006 Lev Walkin <vlm@lionet.info>.
 * All rights reserved.
 * Redistribution and modifications are permitted subject to BSD license.
 */
#include <asn_internal.h>
#include <ENUMERATED.h>
#include <NativeEnumerated.h>
#include <asn_codecs_prim.h>	/* Encoder and decoder of a primitive type */

/*
 * ENUMERATED basic type description.
 */
static ber_tlv_tag_t asn_DEF_ENUMERATED_tags[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_ENUMERATED = {
	"ENUMERATED",
	"ENUMERATED",
	ASN__PRIMITIVE_TYPE_free,
	INTEGER_print,			/* Implemented in terms of INTEGER */
	asn_generic_no_constraint,
	ber_decode_primitive,
	INTEGER_encode_der,		/* Implemented in terms of INTEGER */
	INTEGER_decode_xer,	/* This is temporary! */
	INTEGER_encode_xer,
	ENUMERATED_decode_uper,	/* Unaligned PER decoder */
	ENUMERATED_encode_uper,	/* Unaligned PER encoder */
	0, /* Use generic outmost tag fetcher */
	asn_DEF_ENUMERATED_tags,
	sizeof(asn_DEF_ENUMERATED_tags) / sizeof(asn_DEF_ENUMERATED_tags[0]),
	asn_DEF_ENUMERATED_tags,	/* Same as above */
	sizeof(asn_DEF_ENUMERATED_tags) / sizeof(asn_DEF_ENUMERATED_tags[0]),
	0,	/* No PER visible constraints */
	0, 0,	/* No members */
	0	/* No specifics */
};

asn_dec_rval_t
ENUMERATED_decode_uper(asn_codec_ctx_t *opt_codec_ctx, asn_TYPE_descriptor_t *td,
	asn_per_constraints_t *constraints, void **sptr, asn_per_data_t *pd) {
	asn_dec_rval_t rval;
	ENUMERATED_t *st = (ENUMERATED_t *)*sptr;
	long value;
	void *vptr = &value;

	if(!st) {
		st = (ENUMERATED_t *)(*sptr = CALLOC(1, sizeof(*st)));
		if(!st) _ASN_DECODE_FAILED;
	}

	rval = NativeEnumerated_decode_uper(opt_codec_ctx, td, constraints,
			(void **)&vptr, pd);
	if(rval.code == RC_OK)
		if(asn_long2INTEGER(st, value))
			rval.code = RC_FAIL;
	return rval;
}

asn_enc_rval_t
ENUMERATED_encode_uper(asn_TYPE_descriptor_t *td,
	asn_per_constraints_t *constraints, void *sptr, asn_per_outp_t *po) {
	ENUMERATED_t *st = (ENUMERATED_t *)sptr;
	long value;

	if(asn_INTEGER2long(st, &value))
		_ASN_ENCODE_FAILED;

	return NativeEnumerated_encode_uper(td, constraints, &value, po);
}

