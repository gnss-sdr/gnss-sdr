/*!
 * \file agnss_ref_time.h
 * \brief  Interface of an Assisted GNSS REFERENCE TIME storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_AGNSS_REF_TIME_H
#define GNSS_SDR_AGNSS_REF_TIME_H

#include <boost/serialization/nvp.hpp>
#include <ctime>
#include <string>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief  Interface of an Assisted GNSS REFERENCE TIME storage
 *
 */
class Agnss_Ref_Time
{
public:
    /*!
     * Default constructor
     */
    Agnss_Ref_Time() = default;

    double tow{};
    double week{};
    double seconds{};
    double microseconds{};
    bool valid{};

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML
     * serialization. Here is used to save the ref time data on disk file.
     */
    inline void serialize(Archive& archive, const unsigned int version)
    {
        if (version)
            {
            };
        archive& BOOST_SERIALIZATION_NVP(tow);
        archive& BOOST_SERIALIZATION_NVP(week);
        archive& BOOST_SERIALIZATION_NVP(seconds);
        archive& BOOST_SERIALIZATION_NVP(microseconds);
        archive& BOOST_SERIALIZATION_NVP(valid);
    }
};


/*!
 * \brief Parses a GNSS-SDR.AGNSS_ref_utc_time string ("DD/MM/YYYY HH:MM:SS"
 * in UTC) into an Agnss_Ref_Time. An empty ref_time_str is an "educated
 * guess" case, not an error: it returns the host's current wall-clock time,
 * marked valid, matching what a genuinely live run with no fixed reference
 * configured should use. malformed_year and malformed_format distinguish
 * why a non-empty but unparseable string failed, matching the two distinct
 * diagnostics ControlThread::init() has always printed; both are false when
 * ref_time_str is empty or parses successfully. Shared by every caller that
 * needs this parsing (ControlThread::init(), SatelliteVisibility) so the
 * rule -- including the empty-string fallback -- lives in exactly one place.
 */
inline Agnss_Ref_Time parse_agnss_ref_utc_time(const std::string& ref_time_str, bool* malformed_year = nullptr, bool* malformed_format = nullptr)
{
    if (malformed_year != nullptr)
        {
            *malformed_year = false;
        }
    if (malformed_format != nullptr)
        {
            *malformed_format = false;
        }
    Agnss_Ref_Time result{};
    if (ref_time_str.empty())
        {
            result.seconds = static_cast<double>(time(nullptr));
            result.valid = true;
            return result;
        }
    struct tm tm{};
    if (strptime(ref_time_str.c_str(), "%d/%m/%Y %H:%M:%S", &tm) != nullptr)
        {
            const time_t parsed = timegm(&tm);
            if (parsed > 0)
                {
                    result.seconds = static_cast<double>(parsed);
                    result.valid = true;
                }
            else if (malformed_year != nullptr)
                {
                    *malformed_year = true;
                }
        }
    else if (malformed_format != nullptr)
        {
            *malformed_format = true;
        }
    return result;
}


/** \} */
/** \} */
#endif  // GNSS_SDR_AGNSS_REF_TIME_H
