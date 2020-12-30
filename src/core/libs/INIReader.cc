/*!
 * \file INIReader.cc
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

#include "INIReader.h"
#include "ini.h"
#include <cctype>   // for tolower
#include <cstdlib>  // for stro
#include <utility>


INIReader::INIReader(const std::string& filename)
{
    _error = ini_parse(filename.c_str(), ValueHandler, this);
}


int INIReader::ParseError() const
{
    return _error;
}


std::string INIReader::Get(const std::string& section, const std::string& name, const std::string& default_value)
{
    std::string key = MakeKey(section, name);
    return _values.count(key) ? _values[key] : default_value;
}


int64_t INIReader::GetInteger(const std::string& section, const std::string& name, int64_t default_value)
{
    std::string valstr = Get(section, name, "");
    const char* value = valstr.c_str();
    char* end;
    // This parses "1234" (decimal) and also "0x4D2" (hex)
    int64_t n = strtol(value, &end, 0);
    return end > value ? n : default_value;
}


std::string INIReader::MakeKey(const std::string& section, const std::string& name)
{
    std::string key = section + "." + name;
    // Convert to lower case to make lookups case-insensitive
    for (char& i : key)
        {
            i = tolower(i);
        }
    return key;
}


int INIReader::ValueHandler(void* user, const char* section, const char* name,
    const char* value)
{
    auto* reader = static_cast<INIReader*>(user);
    reader->_values[MakeKey(section, name)] = value;
    return 1;
}
