Rinex2assist
------------

This program reads data from RINEX navigation files and generates XML files that can be read by GNSS-SDR as Assisted GNSS data. The usage is as follows:

```
$ rinex2assist /path/to/RINEX_nav_file 
```

The argument is mandatory (the name of the RINEX navigation file). The name `gps_ephemeris.xml` is given to the output if GPS NAV data is fould. If the RINEX file contains Galileo data, the corresponding `gal_ephemeris.xml` file will be generated. The program is also able to extract parameters of the UTC and the Ionospheric models from the RINEX header, if available. They will be called `gps_utc_model.xml`, `gps_iono.xml`, `gal_utc_model.xml` and `gal_iono.xml`.

There are some servers available for downloading RINEX navigation files. For instance:
  * NASA: [ftp://cddis.gsfc.nasa.gov/pub/gnss/data/hourly/](ftp://gssc.esa.int/gnss/data/hourly/)
  * ESA: [ftp://gssc.esa.int/gnss/data/hourly/](ftp://gssc.esa.int/gnss/data/hourly/)

Just make sure to pick up a [station near you](http://gpspp.sakura.ne.jp/gmap/igsnet.htm).

The program accepts either versions 2.xx or 3.xx for the RINEX navigation data file, as well as compressed files (ending in `.gz`).

Examples:

```
$ rinex2assist EBRE00ESP_R_20183290400_01H_GN.rnx.gz
Generated file: gps_ephemeris.xml
Generated file: gps_utc_model.xml
Generated file: gps_iono.xml
```

and

```
$ rinex2assist EBRE00ESP_R_20183290000_01H_EN.rnx.gz
Generated file: gal_ephemeris.xml
Generated file: gal_utc_model.xml
Generated file: gal_iono.xml
```


An example of GNSS-SDR configuration using ephemeris, UTC and ionospheric model parameters for GPS L1 and Galileo signals is shown below:

```
GNSS-SDR.AGNSS_XML_enabled=true
GNSS-SDR.AGNSS_ref_location=41.39,2.31
GNSS-SDR.AGNSS_gps_ephemeris_xml=gps_ephemeris.xml
GNSS-SDR.AGNSS_gps_iono_xml=gps_iono.xml
GNSS-SDR.AGNSS_gps_utc_model_xml=gps_utc_model.xml
GNSS-SDR.AGNSS_gal_ephemeris_xml=gal_ephemeris.xml
GNSS-SDR.AGNSS_gal_iono_xml=gal_iono.xml    
GNSS-SDR.AGNSS_gal_utc_model_xml=gal_utc_model.xml
```

More info about the usage of AGNSS data [here](https://gnss-sdr.org/docs/sp-blocks/global-parameters/#assisted-gnss-with-xml-files).
