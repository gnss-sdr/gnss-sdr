/*!
 * \file INIReader.h
 * \brief This class reads an INI file into easy-to-access name/value pairs.
 * \author Brush Technologies, 2009.
 *
 * inih (INI Not Invented Here) is a simple .INI file parser written in C++.
 * It's only a couple of pages of code, and it was designed to be small
 * and simple, so it's good for embedded systems. To use it, just give
 * ini_parse() an INI file, and it will call a callback for every
 * name=value pair parsed, giving you strings for the section, name,
 * and value. It's done this way because it works well on low-memory
 * embedded systems, but also because it makes for a KISS implementation.
 *
 * -----------------------------------------------------------------------------
 * inih and INIReader are released under the New BSD license:
 *
 * Copyright (c) 2009, Brush Technology
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Go to the project home page for more info:
 *
 * https://github.com/benhoyt/inih
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_INIREADER_H
#define GNSS_SDR_INIREADER_H

#include <cstdint>
#include <map>
#include <string>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */


/*!
 * \brief Read an INI file into easy-to-access name/value pairs. (Note that I've gone
 * for simplicity here rather than speed, but it should be pretty decent.)
 */
class INIReader
{
public:
    //! Construct INIReader and parse given filename. See ini.h for more info about the parsing.
    explicit INIReader(const std::string& filename);

    //! Return the result of ini_parse(), i.e., 0 on success, line number of first error on parse error, or -1 on file open error.
    int ParseError() const;

    //! Get a string value from INI file, returning default_value if not found.
    std::string Get(const std::string& section, const std::string& name,
        const std::string& default_value);

    //! Get an integer (long) value from INI file, returning default_value if not found.
    int64_t GetInteger(const std::string& section, const std::string& name, int64_t default_value);

private:
    static std::string MakeKey(const std::string& section, const std::string& name);
    static int ValueHandler(void* user, const char* section, const char* name,
        const char* value);

    std::map<std::string, std::string> _values;
    int _error;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_INIREADER_H
