/* Copyright (C) 2010-2018 (see AUTHORS file for a list of contributors)
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef VOLK_GNSSSDR_OPTION_HELPERS_H
#define VOLK_GNSSSDR_OPTION_HELPERS_H

#include <climits>
#include <cstring>
#include <map>
#include <string>
#include <vector>


typedef enum
{
    VOID_CALLBACK,
    INT_CALLBACK,
    BOOL_CALLBACK,
    STRING_CALLBACK,
    FLOAT_CALLBACK,
    STRING,
} VOLK_OPTYPE;

class option_t
{
public:
    option_t(std::string longform, std::string shortform, std::string msg, void (*callback)());
    option_t(std::string longform, std::string shortform, std::string msg, void (*callback)(int));
    option_t(std::string longform, std::string shortform, std::string msg, void (*callback)(float));
    option_t(std::string longform, std::string shortform, std::string msg, void (*callback)(bool));
    option_t(std::string longform, std::string shortform, std::string msg, void (*callback)(std::string));
    option_t(std::string longform, std::string shortform, std::string msg, std::string printval);

    std::string longform;
    std::string shortform;
    std::string msg;
    VOLK_OPTYPE option_type;
    std::string printval;
    void (*callback)();
};

class option_list
{
public:
    option_list(std::string program_name);

    void add(const option_t &opt);

    void parse(int argc, char **argv);

    void help();

private:
    std::string program_name;
    std::vector<option_t> internal_list;
};


#endif  //VOLK_GNSSSDR_OPTION_HELPERS_H
