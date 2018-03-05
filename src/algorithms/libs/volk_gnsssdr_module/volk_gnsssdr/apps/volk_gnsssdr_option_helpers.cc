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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 */

#include "volk_gnsssdr_option_helpers.h"
#include <climits>    // IWYU pragma: keep
#include <cstdlib>    // IWYU pragma: keep
#include <cstring>    // IWYU pragma: keep
#include <exception>  // for exception
#include <iostream>   // for operator<<, endl, basic_ostream, cout, ostream
#include <utility>    // for pair


/*
 * Option type
 */
option_t::option_t(std::string longform, std::string shortform, std::string msg, void (*callback)())
    : longform("--" + longform),
      shortform("-" + shortform),
      msg(msg),
      callback(callback) { option_type = VOID_CALLBACK; }

option_t::option_t(std::string longform, std::string shortform, std::string msg, void (*callback)(int))
    : longform("--" + longform),
      shortform("-" + shortform),
      msg(msg),
      callback((void (*)())callback) { option_type = INT_CALLBACK; }

option_t::option_t(std::string longform, std::string shortform, std::string msg, void (*callback)(float))
    : longform("--" + longform),
      shortform("-" + shortform),
      msg(msg),
      callback((void (*)())callback) { option_type = FLOAT_CALLBACK; }

option_t::option_t(std::string longform, std::string shortform, std::string msg, void (*callback)(bool))
    : longform("--" + longform),
      shortform("-" + shortform),
      msg(msg),
      callback((void (*)())callback) { option_type = BOOL_CALLBACK; }

option_t::option_t(std::string longform, std::string shortform, std::string msg, void (*callback)(std::string))
    : longform("--" + longform),
      shortform("-" + shortform),
      msg(msg),
      callback((void (*)())callback) { option_type = STRING_CALLBACK; }

option_t::option_t(std::string longform, std::string shortform, std::string msg, std::string printval)
    : longform("--" + longform),
      shortform("-" + shortform),
      msg(msg),
      printval(printval) { option_type = STRING; }


/*
 * Option List
 */

option_list::option_list(std::string program_name) : program_name(program_name)
{
    {
        internal_list = std::vector<option_t>();
    }
}

void option_list::add(const option_t &opt) { internal_list.push_back(opt); }

void option_list::parse(int argc, char **argv)
{
    for (int arg_number = 0; arg_number < argc; ++arg_number)
        {
            for (std::vector<option_t>::iterator this_option = internal_list.begin();
                 this_option != internal_list.end();
                 this_option++)
                {
                    if (this_option->longform == std::string(argv[arg_number]) ||
                        this_option->shortform == std::string(argv[arg_number]))
                        {
                            switch (this_option->option_type)
                                {
                                case VOID_CALLBACK:
                                    this_option->callback();
                                    break;
                                case INT_CALLBACK:
                                    try
                                        {
                                            int int_val = std::stoi(argv[++arg_number]);
                                            ((void (*)(int))this_option->callback)(int_val);
                                        }
                                    catch (std::exception &exc)
                                        {
                                            std::cout << "An int option can only receive a number" << std::endl;
                                            throw std::exception();
                                        };
                                    break;
                                case FLOAT_CALLBACK:
                                    try
                                        {
                                            int int_val = std::stof(argv[++arg_number]);
                                            ((void (*)(float))this_option->callback)(int_val);
                                        }
                                    catch (std::exception &exc)
                                        {
                                            std::cout << "A float option can only receive a number" << std::endl;
                                            throw std::exception();
                                        };
                                    break;
                                case BOOL_CALLBACK:
                                    try
                                        {
                                            bool int_val = (bool)std::stoi(argv[++arg_number]);
                                            ((void (*)(bool))this_option->callback)(int_val);
                                        }
                                    catch (std::exception &exc)
                                        {
                                            std::cout << "A bool option can only receive 0 or 1" << std::endl;
                                            throw std::exception();
                                        };
                                    break;
                                case STRING_CALLBACK:
                                    try
                                        {
                                            ((void (*)(std::string))this_option->callback)(argv[++arg_number]);
                                        }
                                    catch (std::exception &exc)
                                        {
                                            throw std::exception();
                                        };
                                    break;
                                case STRING:
                                    std::cout << this_option->printval << std::endl;
                                    break;
                                default:
                                    this_option->callback();
                                    break;
                                }
                        }
                }
            if (std::string("--help") == std::string(argv[arg_number]) ||
                std::string("-h") == std::string(argv[arg_number]))
                {
                    help();
                }
        }
}

void option_list::help()
{
    std::cout << program_name << std::endl;
    std::cout << "  -h [ --help ] \t\tDisplay this help message" << std::endl;
    for (std::vector<option_t>::iterator this_option = internal_list.begin();
         this_option != internal_list.end();
         this_option++)
        {
            std::string help_line("  ");
            if (this_option->shortform == "-")
                {
                    help_line += this_option->longform + " ";
                }
            else
                {
                    help_line += this_option->shortform + " [ " + this_option->longform + " ]";
                }

            switch (help_line.size() / 8)
                {
                case 0:
                    help_line += "\t\t\t\t";
                    break;
                case 1:
                    help_line += "\t\t\t";
                    break;
                case 2:
                    help_line += "\t\t";
                    break;
                case 3:
                    help_line += "\t";
                    break;
                default:
                    break;
                }
            help_line += this_option->msg;
            std::cout << help_line << std::endl;
        }
}
