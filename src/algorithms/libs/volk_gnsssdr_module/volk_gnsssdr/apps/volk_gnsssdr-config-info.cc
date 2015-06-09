/* -*- c++ -*- */
/* Copyright (C) 2010-2015 (see AUTHORS file for a list of contributors)
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

#if HAVE_CONFIG_H
#include <config.h>
#endif

#include <iostream>
#include <boost/program_options.hpp>
#include "volk_gnsssdr/constants.h"
#include "volk_gnsssdr/volk_gnsssdr.h"


namespace po = boost::program_options;

int
main(int argc, char **argv)
{
  po::options_description desc("Program options: volk_gnsssdr-config-info [options]");
  po::variables_map vm;

  desc.add_options()
    ("help,h", "print help message")
    ("prefix", "print VOLK installation prefix")
    ("builddate", "print VOLK build date (RFC2822 format)")
    ("cc", "print VOLK C compiler version")
    ("cflags", "print VOLK CFLAGS")
    ("all-machines", "print VOLK machines built into library")
    ("avail-machines", "print VOLK machines the current platform can use")
    ("machine", "print the VOLK machine that will be used")
    ("version,v", "print VOLK version")
    ;

  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
  }
  catch (po::error& error){
    std::cerr << "Error: " << error.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  if(vm.size() == 0 || vm.count("help")) {
    std::cout << desc << std::endl;
    return 1;
  }

  if(vm.count("prefix"))
    std::cout << volk_gnsssdr_prefix() << std::endl;

  if(vm.count("builddate"))
    std::cout << volk_gnsssdr_build_date() << std::endl;

  if(vm.count("version"))
    std::cout << volk_gnsssdr_version() << std::endl;

  if(vm.count("cc"))
    std::cout << volk_gnsssdr_c_compiler() << std::endl;

  if(vm.count("cflags"))
    std::cout << volk_gnsssdr_compiler_flags() << std::endl;

  // stick an extra ';' to make output of this and avail-machines the
  // same structure for easier parsing
  if(vm.count("all-machines"))
    std::cout << volk_gnsssdr_available_machines() << ";" << std::endl;

  if(vm.count("avail-machines")) {
    volk_gnsssdr_list_machines();
  }

  if(vm.count("machine")) {
    std::cout << volk_gnsssdr_get_machine() << std::endl;
  }

  return 0;
}
