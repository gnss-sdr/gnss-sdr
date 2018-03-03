/*!
 * \file front_end_cal.h
 * \brief Interface of the Front-end calibration program.
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_FRONT_END_CAL_H_
#define GNSS_SDR_FRONT_END_CAL_H_

#include <armadillo>
#include "file_configuration.h"
#include "concurrent_map.h"


class FrontEndCal
{
private:
    std::shared_ptr<ConfigurationInterface> configuration_;

    /*!
     * \brief LLA2ECEF Convert geodetic coordinates to Earth-centered Earth-fixed
     * (ECEF)  coordinates. P = LLA2ECEF( LLA ) converts an M-by-3 array of geodetic coordinates
     * (latitude, longitude and altitude), LLA, to an M-by-3 array of ECEF
     * coordinates, P.  LLA is in [degrees degrees meters].  P is in meters.
     * The default ellipsoid planet is WGS84. Original copyright (c) by Kai Borre.
     */
    arma::vec lla2ecef(const arma::vec &lla);
    /*!
     * GEODETIC2ECEF Convert geodetic to geocentric (ECEF) coordinates
     * [X, Y, Z] = GEODETIC2ECEF(PHI, LAMBDA, H, ELLIPSOID) converts geodetic
     * point locations specified by the coordinate arrays PHI (geodetic
     * latitude in radians), LAMBDA (longitude in radians), and H (ellipsoidal
     * height) to geocentric Cartesian coordinates X, Y, and Z.  The geodetic
     * coordinates refer to the reference ellipsoid specified by ELLIPSOID (a
     * row vector with the form [semimajor axis, eccentricity]).  H must use
     * the same units as the semimajor axis;  X, Y, and Z will be expressed in
     * these units also.
     *
     * The geocentric Cartesian coordinate system is fixed with respect to the
     * Earth, with its origin at the center of the ellipsoid and its X-, Y-,
     * and Z-axes intersecting the surface at the following points:
     * PHI  LAMBDA
     * X-axis:    0     0      (Equator at the Prime Meridian)
     * Y-axis:    0   pi/2     (Equator at 90-degrees East
     * Z-axis:  pi/2    0      (North Pole)
     *
     * A common synonym is Earth-Centered, Earth-Fixed coordinates, or ECEF.
     *
     * See also ECEF2GEODETIC, ECEF2LV, GEODETIC2GEOCENTRICLAT, LV2ECEF.
     *
     * Copyright 2004-2009 The MathWorks, Inc.
     * $Revision: 1.1.6.4 $  $Date: 2009/04/15 23:34:46 $
     * Reference
     * ---------
     * Paul R. Wolf and Bon A. Dewitt, "Elements of Photogrammetry with
     * Applications in GIS," 3rd Ed., McGraw-Hill, 2000 (Appendix F-3).
     */
    arma::vec geodetic2ecef(double phi, double lambda, double h, const arma::vec &ellipsoid);
    /*!
     * \brief Reads the ephemeris data from an external XML file
     *
     */
    bool read_assistance_from_XML();
    /*!
     * \brief Connects to Secure User Location Protocol (SUPL) server to obtain
     * the current GPS ephemeris and GPS assistance data.
     *
     */
    int Get_SUPL_Assist();

    const std::string eph_default_xml_filename = "./gps_ephemeris.xml";

public:
    /*!
     * \brief Sets the configuration data required by get_ephemeris function
     *
     */
    void set_configuration(std::shared_ptr<ConfigurationInterface> configuration);

    /*!
     * \brief This function connects to a Secure User Location Protocol (SUPL) server to obtain
     * the current GPS ephemeris and GPS assistance data. It requires the configuration parameters set by
     * set_configuration function.
     *
     */
    bool get_ephemeris();

    /*!
     * \brief This function estimates the GPS L1 satellite Doppler frequency [Hz] using the following data:
     * 1- Orbital model from the ephemeris
     * 2- Approximate GPS Time of Week (TOW)
     * 3- Approximate receiver Latitude and Longitude (WGS-84)
     *
     */
    double estimate_doppler_from_eph(unsigned int PRN, double TOW, double lat, double lon, double height);

    /*!
     * \brief This function models the Elonics E4000 + RTL2832 front-end
     * Inputs:
     *  f_bb_true_Hz    - Ideal output frequency in baseband [Hz]
     *  f_in_bb_meas_Hz - measured output frequency in baseband [Hz]
     * Outputs:
     *  estimated_fs_Hz  - Sampling frequency estimation based on the
     *  measurements and the front-end model
     *  estimated_f_if_bb_Hz - Equivalent bb if frequency estimation based on the
     *  measurements and the front-end model
     *  Front-end TUNER Elonics E4000 + RTL2832 sampler For GPS L1 1575.42 MHz
     *
     */
    void GPS_L1_front_end_model_E4000(double f_bb_true_Hz, double f_bb_meas_Hz, double fs_nominal_hz, double *estimated_fs_Hz, double *estimated_f_if_Hz, double *f_osc_err_ppm);

    FrontEndCal();
    ~FrontEndCal();
};

#endif
