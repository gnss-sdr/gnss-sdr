/*
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2019 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

// clang-format off

<% this_machine = machine_dict[args[0]] %>
<% arch_names = this_machine.arch_names %>

%for arch in this_machine.archs:
#define LV_HAVE_${arch.name.upper()} 1
%endfor

#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include "volk_gnsssdr_machines.h"
#include <volk_gnsssdr/volk_gnsssdr_config_fixed.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif


%for kern in kernels:
#include <volk_gnsssdr/${kern.name}.h>
%endfor

struct volk_gnsssdr_machine volk_gnsssdr_machine_${this_machine.name} = {
<% make_arch_have_list = (' | '.join(['(1 << LV_%s)'%a.name.upper() for a in this_machine.archs])) %>    ${make_arch_have_list},
<% this_machine_name = "\""+this_machine.name+"\"" %>    ${this_machine_name},
    ${this_machine.alignment},
##// list all kernels
    %for kern in kernels:
<% impls = kern.get_impls(arch_names) %>
##// kernel name
<% kern_name = "\""+kern.name+"\"" %>    ${kern_name},
##// list of kernel implementations by name
<% make_impl_name_list = "{"+', '.join(['"%s"'%i.name for i in impls])+"}" %>    ${make_impl_name_list},
##// list of arch dependencies per implementation
<% make_impl_deps_list = "{"+', '.join([' | '.join(['(1 << LV_%s)'%d.upper() for d in i.deps]) for i in impls])+"}" %>    ${make_impl_deps_list},
##// alignment required? for each implementation
<% make_impl_align_list = "{"+', '.join(['true' if i.is_aligned else 'false' for i in impls])+"}" %>    ${make_impl_align_list},
##// pointer to each implementation
<% make_impl_fcn_list = "{"+', '.join(['%s_%s'%(kern.name, i.name) for i in impls])+"}" %>    ${make_impl_fcn_list},
##// number of implementations listed here
<% len_impls = len(impls) %>    ${len_impls},
    %endfor
};
// clang-format on
