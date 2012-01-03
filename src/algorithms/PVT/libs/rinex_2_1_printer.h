/*!
 * \file rinex_2_1_printer.h  (temporal name)
 * \brief Interface of a RINEX 3.01 printer
 * See http://igscb.jpl.nasa.gov/igscb/data/format/rinex301.pdf
 * \author Carles Fernandez Prades, 2011. cfernandez(at)cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#ifndef GNSS_SDR_RINEX_PRINTER_H_
#define	GNSS_SDR_RINEX_PRINTER_H_

#include <string>
#include <fstream>
#include "gps_navigation_message.h"

/*!
 * \brief Class that handles the generation of Receiver
 * INdependent EXchange format (RINEX) files
 */
class rinex_printer
{
private:
    std::ofstream navFile ;
    std::ofstream obsFile ;

    /* Creates RINEX file names according to the naming convention
     *
     * See http://igscb.jpl.nasa.gov/igscb/data/format/rinex301.pdf
     * Section 4, page 6
     *
     * \param[in] type of RINEX file. Can be:
     * "RINEX_FILE_TYPE_OBS" - Observation file.
     * "RINEX_FILE_TYPE_GPS_NAV" - GPS navigation message file.
     * "RINEX_FILE_TYPE_MET" - Meteorological data file.
     * "RINEX_FILE_TYPE_GLO_NAV" - GLONASS navigation file.
     * "RINEX_FILE_TYPE_GAL_NAV"  - Galileo navigation message file.
     * "RINEX_FILE_TYPE_MIXED_NAV" - Mixed GNSS navigation message file.
     * "RINEX_FILE_TYPE_GEO_NAV" - SBAS Payload navigation message file.
     * "RINEX_FILE_TYPE_SBAS" - SBAS broadcast data file.
     * "RINEX_FILE_TYPE_CLK" - Clock file.
     */
    std::string createFilename(std::string type);

    /*
     * Generates the data for the PGM / RUN BY / DATE line
     */
    std::string getLocalTime();

    /*
     *  Checks that the line is 80 characters length
     */
    void lengthCheck(std::string line);

    /*
     * If the string is bigger than length, truncate it from the right.
     * otherwise, add pad characters to its right.
     *
     * Left-justifies the input in a string of the specified
     * length. If the new length (\a length) is larger than the
     * current length, the string is extended by the pad
     * character (\a pad). The default pad character is a
     * blank.
     * \param[in] s string to be modified.
     * \param[in] length new desired length of string.
     * \param[in] pad character to pad string with (blank by default).
     * \return a reference to \a s.  */
    inline std::string& leftJustify(std::string& s,
            const std::string::size_type length,
            const char pad = ' ');

    /*
     * If the string is bigger than length, truncate it from the right.
     * otherwise, add pad characters to its right.
     *
     * Left-justifies the receiver in a string of the specified
     * length (const version). If the new length (\a length) is larger
     * than the current length, the string is extended by the pad
     * character (\a pad). The default pad character is a
     * blank.
     * \param[in] s string to be modified.
     * \param[in] length new desired length of string.
     * \param[in] pad character to pad string with (blank by default).
     * \return a reference to \a s.  */
    inline std::string leftJustify(const std::string& s,
            const std::string::size_type length,
            const char pad = ' ')
    { std::string t(s); return leftJustify(t, length, pad); }



    /*
     *  Generates the Navigation Data header
     */
    void Rinex2NavHeader(std::ofstream& out);

    /*
     *  Generates the Observation data header
     */
    void Rinex2ObsHeader(std::ofstream& out);

    /*
     * Generation of RINEX signal strength indicators
     */
    int signalStrength(double snr);




public:
    /*!
     * \brief Default constructor. Creates GPS Navigation and Observables RINEX files and their headers
     */
    rinex_printer();

    /*!
     * \brief Default destructor. Closes GPS Navigation and Observables RINEX files
     */
    ~rinex_printer();
    void LogRinex2Nav(gps_navigation_message nav_msg);
    void LogRinex2Obs(gps_navigation_message nav_msg,double interframe_seconds, std::map<int,float> pseudoranges);

};



// Implementation of inline function

inline std::string& rinex_printer::leftJustify(std::string& s,
        const std::string::size_type length,
        const char pad)
{

    if(length < s.length())
        {
            s = s.substr(0, length);
        }
    else
        {
            s.append(length-s.length(), pad);
        }
    return s;
}


#endif
