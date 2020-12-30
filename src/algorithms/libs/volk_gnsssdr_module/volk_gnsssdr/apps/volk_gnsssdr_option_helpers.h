/*
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
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
    explicit option_list(std::string program_name);

    void add(const option_t &opt);

    void parse(int argc, char **argv);

    void help();

private:
    std::string program_name;
    std::vector<option_t> internal_list;
};


#endif  // VOLK_GNSSSDR_OPTION_HELPERS_H
