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

#include <volk_gnsssdr/constants.h>
#include "volk_gnsssdr/volk_gnsssdr.h"
#include <boost/program_options.hpp>
#include <iostream>

namespace po = boost::program_options;

int
main(int argc, char **argv)
{
  po::options_description desc("Program options: volk_gnsssdr-config-info [options]");
  po::variables_map vm;

  desc.add_options()
    ("help,h", "print help message")
    ("prefix", "print VOLK_GNSSSDR installation prefix")
    ("cc", "print VOLK_GNSSSDR C compiler version")
    ("cflags", "print VOLK_GNSSSDR CFLAGS")
    ("all-machines", "print VOLK_GNSSSDR machines built into library")
    ("avail-machines", "print VOLK_GNSSSDR machines the current platform can use")
    ("machine", "print the VOLK_GNSSSDR machine that will be used")
    ("alignment", "print the alignment that will be used")
    ("malloc", "print malloc implementation that will be used")
    ("version,v", "print VOLK_GNSSSDR version")
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

  if(vm.count("alignment")) {
      std::cout << "Alignment in bytes: " << volk_gnsssdr_get_alignment() << std::endl;
   }

   // You don't want to change the volk_malloc code, so just copy the if/else
   // structure from there and give an explanation for the implementations
   if(vm.count("malloc")) {
     std::cout << "Used malloc implementation: ";
 #if _POSIX_C_SOURCE >= 200112L || _XOPEN_SOURCE >= 600 || HAVE_POSIX_MEMALIGN
     std::cout << "posix_memalign" << std::endl;
 #elif _MSC_VER >= 1400
     std::cout << "aligned_malloc" << std::endl;
 #else
     std::cout << "No standard handler available, using own implementation." << std::endl;
 #endif
   }

  return 0;
}
