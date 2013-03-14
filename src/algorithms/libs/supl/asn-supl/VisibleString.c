/*-
 * Copyright (c) 2003, 2006 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * Redistribution and modifications are permitted subject to BSD license.
 */
#include <asn_internal.h>
#include <VisibleString.h>

/*
 * VisibleString basic type description.
 */
static ber_tlv_tag_t asn_DEF_VisibleString_tags[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (26 << 2)),	/* [UNIVERSAL 26] IMPLICIT ...*/
	(ASN_TAG_CLASS_UNIVERSAL | (4 << 2))	/* ... OCTET STRING */
};
static asn_per_constraints_t asn_DEF_VisibleString_constraints = {
	{ APC_CONSTRAINED, 7, 7, 0x20, 0x7e },	/* Value */
	{ APC_SEMI_CONSTRAINED, -1, -1, 0, 0 },	/* Size */
	0, 0
};
asn_TYPE_descriptor_t asn_DEF_VisibleString = {
	"VisibleString",
	"VisibleString",
	OCTET_STRING_free,
	OCTET_STRING_print_utf8,   /* ASCII subset */
	VisibleString_constraint,
	OCTET_STRING_decode_ber,    /* Implemented in terms of OCTET STRING */
	OCTET_STRING_encode_der,
	OCTET_STRING_decode_xer_utf8,
	OCTET_STRING_encode_xer_utf8,
	OCTET_STRING_decode_uper,
	OCTET_STRING_encode_uper,
	0, /* Use generic outmost tag fetcher */
	asn_DEF_VisibleString_tags,
	sizeof(asn_DEF_VisibleString_tags)
	  / sizeof(asn_DEF_VisibleString_tags[0]) - 1,
	asn_DEF_VisibleString_tags,
	sizeof(asn_DEF_VisibleString_tags)
	  / sizeof(asn_DEF_VisibleString_tags[0]),
	&asn_DEF_VisibleString_constraints,
	0, 0,	/* No members */
	0	/* No specifics */
};

int
VisibleString_constraint(asn_TYPE_descriptor_t *td, const void *sptr,
		asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	const VisibleString_t *st = (const VisibleString_t *)sptr;

	if(st && st->buf) {
		uint8_t *buf = st->buf;
		uint8_t *end = buf + st->size;

		/*
		 * Check the alphabet of the VisibleString.
		 * ISO646, ISOReg#6
		 * The alphabet is a subset of ASCII between the space
		 * and "~" (tilde).
		 */
		for(; buf < end; buf++) {
			if(*buf < 0x20 || *buf > 0x7e) {
				_ASN_CTFAIL(app_key, td, sptr,
					"%s: value byte %ld (%d) "
					"not in VisibleString alphabet (%s:%d)",
					td->name,
					(long)((buf - st->buf) + 1),
					*buf,
					__FILE__, __LINE__);
				return -1;
			}
		}
	} else {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}

	return 0;
}
