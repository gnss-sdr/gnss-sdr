/*!
 * \file geohash.cc
 * \brief Implementation of a class for geohash encoding / decoding
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

#include "geohash.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>


std::string Geohash::encode(double lat, double lon, int precision) const
{
    // infer precision?
    if (precision == -1)
        {
            // refine geohash until it matches precision of supplied lat/lon
            for (int p = 1; p <= 12; ++p)
                {
                    const auto hash = Geohash::encode(lat, lon, p);
                    const auto posn = Geohash::decode(hash);

                    if ((std::fabs(posn[0] - lat) < std::numeric_limits<double>::epsilon()) &&
                        (std::fabs(posn[1] - lon) < std::numeric_limits<double>::epsilon()))
                        {
                            return hash;
                        }
                }
            precision = 12;  // set to maximum
        }

    if (std::isnan(lat) || std::isnan(lon) || precision < 1)
        {
            throw std::invalid_argument("Invalid geohash");
        }

    int idx = 0;  // index into base32 map
    int bit = 0;  // each char holds 5 bits
    bool evenBit = true;
    std::string geohash = "";

    double latMin = -90.0;
    double latMax = 90.0;
    double lonMin = -180.0;
    double lonMax = 180.0;

    while (geohash.length() < static_cast<size_t>(precision))
        {
            if (evenBit)
                {
                    // bisect E-W longitude
                    const double lonMid = (lonMin + lonMax) / 2.0;
                    if (lon >= lonMid)
                        {
                            idx = idx * 2 + 1;
                            lonMin = lonMid;
                        }
                    else
                        {
                            idx = idx * 2;
                            lonMax = lonMid;
                        }
                }
            else
                {
                    // bisect N-S latitude
                    const double latMid = (latMin + latMax) / 2.0;
                    if (lat >= latMid)
                        {
                            idx = idx * 2 + 1;
                            latMin = latMid;
                        }
                    else
                        {
                            idx = idx * 2;
                            latMax = latMid;
                        }
                }
            evenBit = !evenBit;

            if (++bit == 5)
                {
                    // 5 bits gives us a character: append it and start over
                    geohash += base32[idx];
                    bit = 0;
                    idx = 0;
                }
        }

    return geohash;
}


std::array<double, 2> Geohash::decode(std::string geohash) const
{
    const auto bounds = Geohash::bounds(std::move(geohash));

    const double latMin = bounds[0];
    const double lonMin = bounds[1];
    const double latMax = bounds[2];
    const double lonMax = bounds[3];

    // cell centre
    double lat = (latMin + latMax) / 2.0;
    double lon = (lonMin + lonMax) / 2.0;

    // round to close to centre without excessive precision: ⌊2-log10(Δ°)⌋ decimal places
    std::array<double, 2> latlon{};
    int decimalPlaces = std::floor(2.0 - std::log10(latMax - latMin));
    double factor = std::pow(10, decimalPlaces);
    latlon[0] = std::round(lat * factor) / factor;
    int decimalPlaces2 = std::floor(2.0 - std::log10(lonMax - lonMin));
    double factor2 = std::pow(10, decimalPlaces2);
    latlon[1] = std::round(lon * factor2) / factor2;

    return latlon;
}


std::array<double, 4> Geohash::bounds(std::string geohash) const
{
    if (geohash.length() == 0)
        {
            throw std::runtime_error("Invalid geohash");
        }

    std::transform(geohash.begin(), geohash.end(), geohash.begin(),
        [](unsigned char c) { return std::tolower(c); });

    bool evenBit = true;
    double latMin = -90.0;
    double latMax = 90.0;
    double lonMin = -180.0;
    double lonMax = 180.0;

    for (char chr : geohash)
        {
            int idx = base32.find(chr);
            if (idx == -1)
                {
                    throw std::runtime_error("Invalid geohash");
                }

            for (int n = 4; n >= 0; n--)
                {
                    int bitN = idx >> n & 1;
                    if (evenBit)
                        {
                            // longitude
                            double lonMid = (lonMin + lonMax) / 2.0;
                            if (bitN == 1)
                                {
                                    lonMin = lonMid;
                                }
                            else
                                {
                                    lonMax = lonMid;
                                }
                        }
                    else
                        {
                            // latitude
                            double latMid = (latMin + latMax) / 2.0;
                            if (bitN == 1)
                                {
                                    latMin = latMid;
                                }
                            else
                                {
                                    latMax = latMid;
                                }
                        }
                    evenBit = !evenBit;
                }
        }

    return {latMin, lonMin, latMax, lonMax};
}