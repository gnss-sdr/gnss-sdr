# XML Schemas for Assisted GNSS-SDR

GNSS-SDR can read assistance data from [Extensible Markup Language (XML)](https://www.w3.org/XML/) files, and can store navigation data decoded from GNSS signals in the same format. This page provides XML Schemas which describe the structure for those XML files.

[XSD (XML Schema Definition)](https://www.w3.org/XML/Schema) is a World Wide Web Consortium (W3C) recommendation that specifies how to formally describe the elements in an XML document.


GPS L1 C/A
----------

 - [ephemeris_map.xsd](./ephemeris_map.xsd)
 - [iono_model.xsd](./iono_model.xsd)
 - [utc_model.xsd](./utc_model.xsd)
 - [gps_almanac_map.xsd](./gps_almanac_map.xsd)
 
 
GPS L2C and L5
--------------
 
 - [cnav_ephemeris_map.xsd](./cnav_ephemeris_map.xsd)
 
 
Galileo OS
----------

 - [gal_ephemeris_map.xsd](./gal_ephemeris_map.xsd)
 - [gal_utc_model.xsd](./gal_utc_model.xsd)
 - [gal_almanac_map.xsd](./gal_almanac_map.xsd)
