/*!
 * \file common_ephemeris.h
 * \brief Base class for GNSS Ephemeris
 * \author Vladislav P, 2022. vladisslav2011(at)gmail.com
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_COMMON_EPHEMERIS_H
#define GNSS_SDR_COMMON_EPHEMERIS_H

#include <cstdint>
#include <memory>
#include <vector>

#ifdef EPHEMERIS_VALIDATOR_DEBUG
#include <iostream>
#define update_eph_deviation(NN)                                                                                                             \
    {                                                                                                                                        \
        if (std::fabs((NN)-tmp.NN) > dev)                                                                                                    \
            {                                                                                                                                \
                dev = std::fabs((NN)-tmp.NN);                                                                                                \
                std::cout << "Gnss_Ephemeris::max_deviation " #NN << ": " << (NN) << "-" << tmp.NN << "=" << std::fabs((NN)-tmp.NN) << "\n"; \
            }                                                                                                                                \
    }
#else
#define update_eph_deviation(NN)              \
    {                                         \
        if (std::fabs((NN)-tmp.NN) > dev)     \
            {                                 \
                dev = std::fabs((NN)-tmp.NN); \
            }                                 \
    }
#endif


class Common_Ephemeris
{
private:
    using last_valid = struct
    {
        std::shared_ptr<Common_Ephemeris> last_eph;
        std::shared_ptr<Common_Ephemeris> valid_eph;
        int valid_eph_count;
        int valid_eph_thr;
    };

protected:
    static constexpr double DEVIATION_THRESHOLD = 0.00001;

public:
    using history_set = std::vector<last_valid>;
    Common_Ephemeris() = default;
    virtual ~Common_Ephemeris() = default;
    virtual double max_deviation(Common_Ephemeris &from) = 0;  //!< Compare a set of ephemeris to another one
    static bool validate(history_set &hist, const std::shared_ptr<Common_Ephemeris> &eph, const int thr, const bool first_pass);
    uint32_t PRN{};  //!< SV ID
};


#endif  // GNSS_SDR_COMMON_EPHEMERIS_H
