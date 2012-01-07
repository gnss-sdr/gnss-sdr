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
#include <iostream>
#include <sstream>  // for stringstream
#include <iomanip> // for setprecision
#include "gps_navigation_message.h"
#include "boost/date_time/posix_time/posix_time.hpp"

/*!
 * \brief Class that handles the generation of Receiver
 * INdependent EXchange format (RINEX) files
 */
class rinex_printer
{
private:




    /*
     * Generation of RINEX signal strength indicators
     */
    int signalStrength(double snr);


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
     * Right-justifies the receiver in a string of the specified
     * length. If the receiver's data is shorter than the
     * requested length (\a length), it is padded on the left with
     * the pad character (\a pad). The default pad
     * character is a blank. */
    inline std::string& rightJustify(std::string& s,
            const std::string::size_type length,
            const char pad = ' ');

    /*
     * Right-justifies the receiver in a string of the specified
     * length (const version). If the receiver's data is shorter than the
     * requested length (\a length), it is padded on the left with
     * the pad character (\a pad). The default pad
     * character is a blank.*/
    inline std::string rightJustify(const std::string& s,
            const std::string::size_type length,
            const char pad = ' ')
    { std::string t(s); return rightJustify(t, length, pad); }



    /*
     * Convert a double to a scientific notation number.
     * @param d the double to convert
     * @param length length (in characters) of output, including exponent
     * @param expLen length (in characters) of the exponent, with sign
     * @param showSign if true, reserves 1 character for +/- sign
     * @param checkSwitch if true, keeps the exponential sanity check for
     * exponentials above three characters in length.  If false, it removes
     * that check.
     */
    inline std::string doub2sci(const double& d,
            const std::string::size_type length,
            const std::string::size_type expLen,
            const bool showSign = true,
            const bool checkSwitch = true);




    /*
     * Convert scientific notation to FORTRAN notation.
     * As an example, the string "1.5636E5" becomes " .15636D6".
     * Note that the first character of the string will be '-' if
     * the number is negative or ' ' if the first character is positive.
     * @param aStr string with number to convert
     * @param startPos start position of number in string
     * @param length length (in characters) of number, including exponent.
     * @param expLen length (in characters of exponent, not including sign.
     * @param checkSwitch will keep the method running as originally programmed
     * when set to true.  If false, the method will always resize exponentials,
     * produce an exponential with an E instead of a D, and always have a leading
     * zero.  For example -> 0.87654E-0004 or -0.1234E00005.
     */
 inline std::string& sci2for(std::string& aStr,
                             const std::string::size_type startPos = 0,
                             const std::string::size_type length = std::string::npos,
                             const std::string::size_type expLen = 3,
                             const bool checkSwitch = true);





    /*
     * Convert double precision floating point to a string
     * containing the number in FORTRAN notation.
     * As an example, the number 156360 becomes ".15636D6".
     * @param d number to convert.
     * @param length length (in characters) of number, including exponent.
     * @param expLen length (in characters of exponent, including sign.
     * @param checkSwitch if true, keeps the exponential sanity check for
     * exponentials above three characters in length.  If false, it removes
     * that check.
     * @return a string containing \a d in FORTRAN notation.
     */
    inline std::string doub2for(const double& d,
            const std::string::size_type length,
            const std::string::size_type expLen,
            const bool checkSwitch = true);


    /*
     * Convert a string to a double precision floating point number.
     * @param s string containing a number.
     * @return double representation of string.
     */
    inline double asDouble(const std::string& s)
    { return strtod(s.c_str(), 0); }

    /*
     * Convert a string to an integer.
     * @param s string containing a number.
     * @return long integer representation of string.
     */
    inline long asInt(const std::string& s)
    { return strtol(s.c_str(), 0, 10); }



    /*
     * Convert a double to a string in fixed notation.
     * @param x double.
     * @param precision the number of decimal places you want displayed.
     * @return string representation of \a x.
     */
    inline std::string asString(const double x,
            const std::string::size_type precision = 17);


    /*
     * Convert a long double to a string in fixed notation.
     * @param x long double.
     * @param precision the number of decimal places you want displayed.
     * @return string representation of \a x.
     */
    inline std::string asString(const long double x,
            const std::string::size_type precision = 21);

    /*
     * Convert any old object to a string.
     * The class must have stream operators defined.
     * @param x object to turn into a string.
     * @return string representation of \a x.
     */
    template <class X>
    inline std::string asString(const X x);





public:
    /*!
     * \brief Default constructor. Creates GPS Navigation and Observables RINEX files and their headers
     */
    rinex_printer();

    std::ofstream obsFile ;
    std::ofstream navFile ;


    /*!
     *  \brief Generates the Navigation Data header
     */
    void Rinex2NavHeader(std::ofstream& out, gps_navigation_message nav);

    /*!
     *  \brief Generates the Observation data header
     */
    void Rinex2ObsHeader(std::ofstream& out, gps_navigation_message nav);

    boost::posix_time::ptime computeTime(gps_navigation_message nav_msg);

    /*!
     * \brief Default destructor. Closes GPS Navigation and Observables RINEX files
     */
    ~rinex_printer();
    void LogRinex2Nav(gps_navigation_message nav_msg);
    void LogRinex2Obs(gps_navigation_message nav_msg, double interframe_seconds, std::map<int,float> pseudoranges);

    std::map<std::string,std::string> satelliteSystem;
    std::map<std::string,std::string> observationType;
    std::map<std::string,std::string> observationCode;

};



// Implementation of inline functions

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


// if the string is bigger than length, truncate it from the left.
// otherwise, add pad characters to its left.
inline std::string& rinex_printer::rightJustify(std::string& s,
        const std::string::size_type length,
        const char pad)
{
    if(length < s.length())
        {
            s = s.substr(s.length()-length, std::string::npos);
        }
    else
        {
            s.insert((std::string::size_type)0, length-s.length(), pad);
        }
    return s;
}





inline std::string rinex_printer::doub2for(const double& d,
        const std::string::size_type length,
        const std::string::size_type expLen,
        const bool checkSwitch)
{

    short exponentLength = expLen;

    /* Validate the assumptions regarding the input arguments */
    if (exponentLength < 0) exponentLength = 1;
    if (exponentLength > 3 && checkSwitch) exponentLength = 3;

    std::string toReturn = doub2sci(d, length, exponentLength, true, checkSwitch);
    sci2for(toReturn, 0, length, exponentLength, checkSwitch);

    return toReturn;

}


inline std::string rinex_printer::doub2sci(const double& d,
        const std::string::size_type length,
        const std::string::size_type expLen,
        const bool showSign,
        const bool checkSwitch)
{
    std::string toReturn;
    short exponentLength = expLen;

    /* Validate the assumptions regarding the input arguments */
    if (exponentLength < 0) exponentLength = 1;
    if (exponentLength > 3 && checkSwitch) exponentLength = 3;

    std::stringstream c;
    c.setf(std::ios::scientific, std::ios::floatfield);

    // length - 3 for special characters ('.', 'e', '+' or '-')
    // - exponentlength (e04)
    // - 1 for the digit before the decimal (2.)
    // and if showSign == true,
    //    an extra -1 for '-' or ' ' if it's positive or negative
    int expSize = 0;
    if (showSign)
        expSize = 1;
    c.precision(length - 3 - exponentLength - 1 - expSize);
    c << d;
    c >> toReturn;
    return toReturn;
}

inline std::string& rinex_printer::sci2for(std::string& aStr,
        const std::string::size_type startPos,
        const std::string::size_type length,
        const std::string::size_type expLen,
        const bool checkSwitch)
{

    std::string::size_type idx = aStr.find('.', startPos);
    int expAdd = 0;
    std::string exp;
    long iexp;
    //If checkSwitch is false, always redo the exponential. Otherwise,
    //set it to false.
    bool redoexp=!checkSwitch;

    // Check for decimal place within specified boundaries
    if ((idx == 0) || (idx >= (startPos + length - expLen - 1)))
        {
            //StringException e("sci2for: no decimal point in string");
            //GPSTK_THROW(e);
        }

    // Here, account for the possibility that there are
    // no numbers to the left of the decimal, but do not
    // account for the possibility of non-scientific
    // notation (more than one digit to the left of the
    // decimal)
    if (idx > startPos)
        {
            redoexp = true;
            // Swap digit and decimal.
            aStr[idx] = aStr[idx-1];
            aStr[idx-1] = '.';
            // Only add one to the exponent if the number is non-zero
            if (asDouble(aStr.substr(startPos, length)) != 0.0)
                expAdd = 1;
        }

    idx = aStr.find('e', startPos);
    if (idx == std::string::npos)
        {
            idx = aStr.find('E', startPos);
            if (idx == std::string::npos)
                {
                    //StringException e("sci2for:no 'e' or 'E' in string");
                    //GPSTK_THROW(e);
                }
        }
    // Change the exponent character to D normally, or E of checkSwitch is false.
    if (checkSwitch)
        aStr[idx] = 'D';
    else
        aStr[idx] = 'E';

    // Change the exponent itself
    if (redoexp)
        {
            exp = aStr.substr(idx + 1, std::string::npos);
            iexp = asInt(exp);
            iexp += expAdd;

            aStr.erase(idx + 1);
            if (iexp < 0)
                {
                    aStr += "-";
                    iexp -= iexp*2;
                }
            else
                aStr += "+";
            aStr += rinex_printer::rightJustify(asString(iexp),expLen,'0');

        }

    // if the number is positive, append a space
    // (if it's negative, there's a leading '-'
    if (aStr[0] == '.')
        {
            aStr.insert((std::string::size_type)0, 1, ' ');
        }

    //If checkSwitch is false, add on one leading zero to the string
    if (!checkSwitch)
        {
            aStr.insert((std::string::size_type)1, 1, '0');
        }


    return aStr;

}  // end sci2for



inline std::string asString(const long double x, const std::string::size_type precision)
 {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << x ;
    return ss.str();
 }




inline std::string rinex_printer::asString(const double x, const std::string::size_type precision)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << x;
    return ss.str();
}

template<class X>
inline std::string rinex_printer::asString(const X x)
{
    std::ostringstream ss;
    ss << x;
    return ss.str();
}


#endif
