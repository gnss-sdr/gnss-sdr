/*
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#if HAVE_CONFIG_H
#include <config.h>
#endif

#include "volk_gnsssdr/volk_gnsssdr.h"    // for volk_gnsssdr_get_alignment, volk_gnsssdr_get_machine
#include "volk_gnsssdr_option_helpers.h"  // for option_list, option_t
#include <volk_gnsssdr/constants.h>       // for volk_gnsssdr_available_machines, volk_gnsssdr_c_compiler ...
#include <iostream>                       // for operator<<, cout, ostream
#include <string>                         // for string

void print_alignment()
{
    std::cout << "Alignment in bytes: " << volk_gnsssdr_get_alignment() << '\n';
}


void print_malloc()
{
    // You don't want to change the volk_malloc code, so just copy the if/else
    // structure from there and give an explanation for the implementations
    std::cout << "Used malloc implementation: ";
#if HAVE_POSIX_MEMALIGN
    std::cout << "posix_memalign\n";
#elif defined(_MSC_VER)
    std::cout << "_aligned_malloc\n";
#else
    std::cout << "C11 aligned_alloc.\n";
#endif
}


int main(int argc, char **argv)
{
    option_list our_options("volk_gnsssdr-config-info");
    our_options.add(option_t("prefix", "", "print the VOLK_GNSSSDR installation prefix", volk_gnsssdr_prefix()));
    our_options.add(option_t("cc", "", "print the VOLK_GNSSDR C compiler version", volk_gnsssdr_c_compiler()));
    our_options.add(option_t("cflags", "", "print the VOLK_GNSSSDR CFLAGS", volk_gnsssdr_compiler_flags()));
    our_options.add(option_t("all-machines", "", "print VOLK_GNSSSDR machines built", volk_gnsssdr_available_machines()));
    our_options.add(option_t("avail-machines", "",
        "print VOLK_GNSSSDR machines on the current "
        "platform",
        volk_gnsssdr_list_machines));
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
    catch (...)
        {
            return 1;
        }
    return 0;
}
