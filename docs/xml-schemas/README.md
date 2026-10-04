# XML Schemas for Assisted GNSS-SDR

<!-- prettier-ignore-start -->
[comment]: # (
SPDX-License-Identifier: GPL-3.0-or-later
)

[comment]: # (
SPDX-FileCopyrightText: 2011-2026 Carles Fernandez-Prades <carles.fernandez@cttc.es>
)
<!-- prettier-ignore-end -->

GNSS-SDR can read assistance data from
[Extensible Markup Language (XML)](https://www.w3.org/XML/) files for faster
[Time-To-First-Fix](https://gnss-sdr.org/design-forces/availability/#time-to-first-fix-ttff),
and can store navigation data decoded from GNSS signals in the same format. This
folder provides XML Schemas that describe the XML files written by the PVT
block. The shared type definitions used by the individual schemas are in
[`common.xsd`](./common.xsd).

[XSD (XML Schema Definition)](https://www.w3.org/XML/Schema) is a World Wide Web
Consortium (W3C) recommendation that specifies how to formally describe the
elements in an XML document.

## GPS L1 C/A

- [ephemeris_map.xsd](./ephemeris_map.xsd) - GPS NAV message ephemeris
  parameters.
- [iono_model.xsd](./iono_model.xsd) - GPS NAV message ionospheric model
  parameters.
- [utc_model.xsd](./utc_model.xsd) - GPS NAV message UTC model parameters.
- [gps_almanac_map.xsd](./gps_almanac_map.xsd) - GPS NAV message almanac.

## GPS L2C and L5

- [cnav_ephemeris_map.xsd](./cnav_ephemeris_map.xsd) - GPS CNAV message
  ephemeris parameters.
- [cnav_utc_model.xsd](./cnav_utc_model.xsd) - GPS CNAV message UTC model
  parameters.
- [cnav_iono_model.xsd](./cnav_iono_model.xsd) - GPS CNAV message ionospheric
  model parameters.

## QZSS

- [ephemeris_map.xsd](./ephemeris_map.xsd) - QZSS LNAV message ephemeris
  parameters (the XML root is shared with GPS LNAV).
- [cnav_ephemeris_map.xsd](./cnav_ephemeris_map.xsd) - QZSS CNAV message
  ephemeris parameters (the XML root is shared with GPS CNAV).
- [qzss_iono_model.xsd](./qzss_iono_model.xsd) - QZSS LNAV message ionospheric
  model parameters.
- [qzss_utc_model.xsd](./qzss_utc_model.xsd) - QZSS LNAV message UTC model
  parameters.
- [qzss_cnav_iono_model.xsd](./qzss_cnav_iono_model.xsd) - QZSS CNAV message
  ionospheric model parameters.
- [qzss_cnav_utc_model.xsd](./qzss_cnav_utc_model.xsd) - QZSS CNAV message UTC
  model parameters.

## Galileo

- [gal_ephemeris_map.xsd](./gal_ephemeris_map.xsd) - Galileo ephemeris
  parameters.
- [gal_iono_model.xsd](./gal_iono_model.xsd) - Galileo ionospheric model
  parameters.
- [gal_utc_model.xsd](./gal_utc_model.xsd) - Galileo UTC model parameters.
- [gal_almanac_map.xsd](./gal_almanac_map.xsd) - Galileo almanac.

## GLONASS

- [gnav_ephemeris_map.xsd](./gnav_ephemeris_map.xsd) - GLONASS GNAV ephemeris
  parameters.
- [gnav_utc_model.xsd](./gnav_utc_model.xsd) - GLONASS GNAV UTC model
  parameters.

## BeiDou

- [bds_dnav_ephemeris_map.xsd](./bds_dnav_ephemeris_map.xsd) - BeiDou D1/D2 DNAV
  ephemeris parameters.
- [bds_dnav_iono_model.xsd](./bds_dnav_iono_model.xsd) - BeiDou D1/D2 DNAV
  ionospheric model parameters, including D2 grid corrections.
- [bds_dnav_utc_model.xsd](./bds_dnav_utc_model.xsd) - BeiDou D1/D2 DNAV UTC and
  inter-system time-offset parameters.
- [bds_dnav_almanac_map.xsd](./bds_dnav_almanac_map.xsd) - BeiDou D1/D2 DNAV
  almanac parameters.
- [bds_cnav1_ephemeris_map.xsd](./bds_cnav1_ephemeris_map.xsd) - BeiDou B-CNAV1
  ephemeris parameters.

## Validation

The schemas accept current Boost archives and the older archive versions for
which GNSS-SDR retains deserialization compatibility. The provided validation
utility selects schemas from the standard PVT output filenames. It accepts
individual files or directories:

```sh
python3 docs/xml-schemas/validate_xml.py ./pvt-output/
```

The utility requires `xmllint`, an XML Schema 1.0 validator distributed with
libxml2.

---

Please check https://gnss-sdr.org/docs/sp-blocks/global-parameters/ for more
information about the usage of XML files in GNSS-SDR.

You could find useful the utility program
[rinex2assist](https://github.com/gnss-sdr/gnss-sdr/tree/next/utils/rinex2assist)
for the generation of compatible XML files from recent, publicly available RINEX
navigation data files.
