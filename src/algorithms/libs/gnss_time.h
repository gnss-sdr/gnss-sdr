/* -------------------------------------------------------------------------
 *
 * Copyright (C) 2019 (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR-SIM is a software defined Global Navigation
 * Satellite Systems Simulator
 *
 * This file is part of GNSS-SDR-SIM.
 *
 */

#ifndef GNSS_SDR_SIM_GNSS_TIME_H
#define GNSS_SDR_SIM_GNSS_TIME_H

#include <boost/serialization/nvp.hpp>
#include <cstdint>
#include <time.h>

class GnssTime
{
public:
    //time_t time;         /* time (s) expressed by standard time_t */
    int week; /*!< GPS week number (since January 1980) */
    //double sec;          /*!< second inside the GPS \a week */
    int tow_ms; /* time of week [ms]*/

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML serialization. Here is used to save the ephemeris data on disk file.
     */
    inline void serialize(Archive& archive, const uint32_t version)
    {
        using boost::serialization::make_nvp;
        if (version)
            {
            };
        archive& make_nvp("week", week);
        archive& make_nvp("tow_ms", tow_ms);
    };
};

#endif
