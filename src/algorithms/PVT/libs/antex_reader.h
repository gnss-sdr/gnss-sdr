/*!
 * \file antex_reader.h
 * \brief Interface of a class that reads antenna phase center offsets and
 * variations from a file in the Antenna Exchange Format (ANTEX)
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_ANTEX_READER_H
#define GNSS_SDR_ANTEX_READER_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <istream>
#include <map>
#include <string>
#include <vector>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */


/*!
 * \brief Calibration of an antenna at a frequency
 */
struct Antex_Frequency
{
    //! Phase center offset, in m. Receiver antennas: north, east, up with
    //! respect to the antenna reference point. Satellite antennas: x, y, z in
    //! the satellite-fixed frame, with respect to the center of mass (or to
    //! the point that the file states, as in the ones of the European GNSS
    //! Service Centre for the Galileo satellites)
    std::array<double, 3> offset;
    //! Phase center variation that does not depend on the azimuth, in m: the
    //! NOAZI values of ANTEX 1.4, or the mean over the azimuth of the pattern
    //! of ANTEX 2.0. Receiver antennas: at zenith angles 0, 5, ..., 90 degrees.
    //! Satellite antennas: at nadir angles 0, 1, ..., 18 degrees
    std::array<double, 19> variation;
};


/*!
 * \brief Calibrations of an antenna
 */
struct Antex_Antenna
{
    std::string type;                                    //!< Antenna type and radome, or satellite type
    std::string serial;                                  //!< Serial number, or satellite code ("E11" in ANTEX 1.4, the SVN "E101" in ANTEX 2.0)
    int64_t valid_from;                                  //!< Start of validity, in s since 1970 (0 if not given)
    int64_t valid_until;                                 //!< End of validity, in s since 1970 (0 if not given)
    std::map<std::string, Antex_Frequency> frequencies;  //!< Key: ANTEX frequency code ("G01", "E07", ...)
};


/*!
 * \brief Reads the antenna calibrations of a file in the Antenna Exchange
 * Format (ANTEX), versions 1.4 and 2.0, see
 * https://files.igs.org/pub/data/format/antex14.txt and
 * https://doi.org/10.57677/antex
 *
 * Only the calibrations of the carrier phase are read, and of their patterns
 * the part that does not depend on the azimuth. ANTEX 2.0 identifies the
 * satellite antennas by the space vehicle number (SVN) of the satellite: the
 * PRN that each satellite has had over time is in the IGS satellite metadata
 * file, see https://doi.org/10.57677/metadata-sinex
 */
class Antex_Reader
{
public:
    Antex_Reader() = default;

    /*!
     * \brief Reads an ANTEX file. Returns false if the file cannot be opened
     * or it has no antenna
     */
    bool read(const std::string& filename);

    /*!
     * \brief Reads the PRN of each satellite over time from an IGS satellite
     * metadata file (SINEX, block SATELLITE/PRN). Returns false if the file
     * cannot be opened or it has no assignment
     */
    bool read_satellite_metadata(const std::string& filename);

    /*!
     * \brief Antenna of a satellite ("G05", "E11", ...) at a time, in s since
     * 1970. Returns nullptr if not found. The antennas of an ANTEX 2.0 file
     * are only found after reading the satellite metadata
     */
    const Antex_Antenna* find_satellite(const std::string& satellite, int64_t time_s) const;

    /*!
     * \brief SVN of a satellite ("G05", "E11", ...) at a time, in s since
     * 1970, from the satellite metadata. Empty if not known
     */
    std::string find_svn(const std::string& satellite, int64_t time_s) const;

    /*!
     * \brief Receiver antenna of that type (antenna and radome names as in the
     * ANTEX file, "TRM59800.00     NONE"). If the radome is not given, or the
     * antenna has no calibration with it, the one with no radome (NONE) is
     * returned. Returns nullptr if not found
     */
    const Antex_Antenna* find_receiver(const std::string& antenna_type) const;

    inline std::size_t get_number_of_satellite_antennas() const
    {
        return d_number_of_satellite_antennas;
    }

    inline std::size_t get_number_of_receiver_antennas() const
    {
        return d_receivers.size();
    }

    //! true if satellite antennas were read that are identified by SVN (ANTEX 2.0)
    inline bool has_satellites_by_svn() const
    {
        return !d_satellites_by_svn.empty();
    }

    //! true if the PRN of the satellites over time was read
    inline bool has_satellite_metadata() const
    {
        return !d_prn_assignments.empty();
    }

private:
    struct Prn_Assignment
    {
        std::string svn;
        int64_t valid_from;   // in s since 1970
        int64_t valid_until;  // in s since 1970, 0 if not bounded
    };

    void read_version_1(std::istream& file);
    void read_version_2(std::istream& file);
    void add_receiver(const Antex_Antenna& antenna);

    std::map<std::string, std::vector<Antex_Antenna>> d_satellites;         // key: satellite code
    std::map<std::string, std::vector<Antex_Antenna>> d_satellites_by_svn;  // key: SVN
    std::map<std::string, std::vector<Prn_Assignment>> d_prn_assignments;   // key: satellite code
    std::map<std::string, Antex_Antenna> d_receivers;                       // key: antenna and radome names, single space in between
    std::size_t d_number_of_satellite_antennas = 0;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_ANTEX_READER_H
