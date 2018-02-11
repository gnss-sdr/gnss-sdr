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

#if HAVE_CONFIG_H
#include <config.h>
#endif

#include <volk_gnsssdr/constants.h>
#include "volk_gnsssdr/volk_gnsssdr.h"
#include <iostream>
#include "volk_gnsssdr_option_helpers.h"

void print_alignment()
{
  std::cout << "Alignment in bytes: " << volk_gnsssdr_get_alignment() << std::endl;
}


void print_malloc()
{
  // You don't want to change the volk_malloc code, so just copy the if/else
  // structure from there and give an explanation for the implementations
  std::cout << "Used malloc implementation: ";
  #if _POSIX_C_SOURCE >= 200112L || _XOPEN_SOURCE >= 600 || HAVE_POSIX_MEMALIGN
  std::cout << "posix_memalign" << std::endl;
  #elif _MSC_VER >= 1400
  std::cout << "aligned_malloc" << std::endl;
  #else
      std::cout << "No standard handler available, using own implementation." << std::endl;
  #endif
}


int main(int argc, char **argv)
{
    option_list our_options("volk_gnsssdr-config-info");
    our_options.add(option_t("prefix", "", "print the VOLK_GNSSSDR installation prefix", volk_gnsssdr_prefix()));
    our_options.add(option_t("cc", "", "print the VOLK_GNSSDR C compiler version", volk_gnsssdr_c_compiler()));
    our_options.add(option_t("cflags", "", "print the VOLK_GNSSSDR CFLAGS", volk_gnsssdr_compiler_flags()));
    our_options.add(option_t("all-machines", "", "print VOLK_GNSSSDR machines built", volk_gnsssdr_available_machines()));
    our_options.add(option_t("avail-machines", "", "print VOLK_GNSSSDR machines on the current "
            "platform", volk_gnsssdr_list_machines));
    our_options.add(option_t("machine", "", "print the current VOLK_GNSSSDR machine that will be used",
            volk_gnsssdr_get_machine()));
    our_options.add(option_t("alignment", "", "print the memory alignment", print_alignment));
    our_options.add(option_t("malloc", "", "print the malloc implementation used in volk_gnsssdr_malloc",
            print_malloc));
    our_options.add(option_t("version", "v", "print the VOLK_GNSSSDR version", volk_gnsssdr_version()));

    try
    {
            our_options.parse(argc, argv);
    }
    catch(...)
    {
            return 1;
    }
    return 0;
}
