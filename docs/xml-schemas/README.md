# XML Schemas for Assisted GNSS-SDR

<!-- prettier-ignore-start -->
[comment]: # (
SPDX-License-Identifier: GPL-3.0-or-later
)

[comment]: # (
SPDX-FileCopyrightText: 2011-2021 Carles Fernandez-Prades <carles.fernandez@cttc.es>
)
<!-- prettier-ignore-end -->

GNSS-SDR can read assistance data from
[Extensible Markup Language (XML)](https://www.w3.org/XML/) files for faster
[Time-To-First-Fix](https://gnss-sdr.org/design-forces/availability/#time-to-first-fix-ttff),
and can store navigation data decoded from GNSS signals in the same format. This
folder provides XML Schemas which describe those XML files structure.

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

## Galileo

- [gal_ephemeris_map.xsd](./gal_ephemeris_map.xsd) - Galileo ephemeris
  parameters.
- [gal_iono_model.xsd](./gal_iono_model.xsd) - Galileo ionospheric model
  parameters.
- [gal_utc_model.xsd](./gal_utc_model.xsd) - Galileo UTC model parameters.
- [gal_almanac_map.xsd](./gal_almanac_map.xsd) - Galileo almanac.

---

Please check https://gnss-sdr.org/docs/sp-blocks/global-parameters/ for more
information about the usage of XML files in GNSS-SDR.

You could find useful the utility program
[rinex2assist](https://github.com/gnss-sdr/gnss-sdr/tree/next/src/utils/rinex2assist)
for the generation of compatible XML files from recent, publicly available RINEX
navigation data files.
