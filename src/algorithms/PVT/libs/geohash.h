/*!
 * \file geohash.h
 * \brief Interface of a class that encodes / decodes geohashes
 * \author Carles Fernandez-Prades, 2023. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2023  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GEOHASH_H
#define GNSS_SDR_GEOHASH_H

#include <array>
#include <string>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */

/*!
 * \brief Class for geohash encoding / decoding
 * See https://en.wikipedia.org/wiki/Geohash
 */
class Geohash
{
public:
    Geohash() = default;

    /**
     * Encodes latitude/longitude to geohash, either to specified precision or
     * to automatically evaluated precision.
     *
     * @param   {double} lat - Latitude in degrees.
     * @param   {double} lon - Longitude in degrees.
     * @param   {int} [precision] - Number of characters in resulting geohash.
     * @returns {string} Geohash of supplied latitude/longitude.
     * @throws  Invalid geohash.
     *
     */
    std::string encode(double lat, double lon, int precision = -1) const;

    /**
     * Decode geohash to latitude/longitude (location is approximate centre of
     * geohash cell, to reasonable precision).
     *
     * @param   {string} geohash - Geohash string to be converted to
     * latitude/longitude.
     * @returns {lat, lon} (Center of) geohashed location.
     * @throws  Invalid geohash.
     *
     */
    std::array<double, 2> decode(std::string geohash) const;

private:
    /*
     * Returns SW/NE latitude/longitude bounds of specified geohash.
     */
    std::array<double, 4> bounds(std::string geohash) const;
    std::string base32{"0123456789bcdefghjkmnpqrstuvwxyz"};
};

/** \} */
/** \} */
#endif  // GNSS_SDR_GEOHASH_H
