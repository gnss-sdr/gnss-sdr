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
 * -------------------------------------------------------------------------
 * inih and INIReader are released under the New BSD license:
 *
 * Copyright (c) 2009, Brush Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of Brush Technology nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY BRUSH TECHNOLOGY ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL BRUSH TECHNOLOGY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Go to the project home page for more info:
 *
 * http://code.google.com/p/inih/
 * -------------------------------------------------------------------------
 */

#ifndef __INIREADER_H__
#define __INIREADER_H__

#include <map>
#include <string>

/*!
 * \brief Read an INI file into easy-to-access name/value pairs. (Note that I've gone
 * for simplicity here rather than speed, but it should be pretty decent.)
 */
class INIReader
{
public:
    //! Construct INIReader and parse given filename. See ini.h for more info about the parsing.
    INIReader(std::string filename);

    //! Return the result of ini_parse(), i.e., 0 on success, line number of first error on parse error, or -1 on file open error.
    int ParseError();

    //! Get a string value from INI file, returning default_value if not found.
    std::string Get(std::string section, std::string name,
        std::string default_value);

    //! Get an integer (long) value from INI file, returning default_value if not found.
    long GetInteger(std::string section, std::string name, long default_value);

private:
    int _error;
    std::map<std::string, std::string> _values;
    static std::string MakeKey(std::string section, std::string name);
    static int ValueHandler(void* user, const char* section, const char* name,
        const char* value);
};

#endif  // __INIREADER_H__
