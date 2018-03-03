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

#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include "volk_gnsssdr_machines.h"
#include <volk_gnsssdr/volk_gnsssdr_typedefs.h>
#include <volk_gnsssdr/volk_gnsssdr_cpu.h>
#include "volk_gnsssdr_rank_archs.h"
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

static size_t __alignment = 0;
static intptr_t __alignment_mask = 0;

struct volk_gnsssdr_machine *get_machine(void)
{
  extern struct volk_gnsssdr_machine *volk_gnsssdr_machines[];
  extern unsigned int n_volk_gnsssdr_machines;
  static struct volk_gnsssdr_machine *machine = NULL;

  if(machine != NULL)
    return machine;
  else {
    unsigned int max_score = 0;
    unsigned int i;
    struct volk_gnsssdr_machine *max_machine = NULL;
    for(i=0; i<n_volk_gnsssdr_machines; i++) {
      if(!(volk_gnsssdr_machines[i]->caps & (~volk_gnsssdr_get_lvarch()))) {
        if(volk_gnsssdr_machines[i]->caps > max_score) {
          max_score = volk_gnsssdr_machines[i]->caps;
          max_machine = volk_gnsssdr_machines[i];
        }
      }
    }
    machine = max_machine;
    //printf("Using Volk machine: %s\n", machine->name);
    __alignment = machine->alignment;
    __alignment_mask = (intptr_t)(__alignment-1);
    return machine;
  }
}

void volk_gnsssdr_list_machines(void)
{
  extern struct volk_gnsssdr_machine *volk_gnsssdr_machines[];
  extern unsigned int n_volk_gnsssdr_machines;

  unsigned int i;
  for(i=0; i<n_volk_gnsssdr_machines; i++) {
    if(!(volk_gnsssdr_machines[i]->caps & (~volk_gnsssdr_get_lvarch()))) {
        printf("%s;", volk_gnsssdr_machines[i]->name);
    }
  }
  printf("\n");
}

const char* volk_gnsssdr_get_machine(void)
{
  extern struct volk_gnsssdr_machine *volk_gnsssdr_machines[];
  extern unsigned int n_volk_gnsssdr_machines;
  static struct volk_gnsssdr_machine *machine = NULL;

  if(machine != NULL)
    return machine->name;
  else {
    unsigned int max_score = 0;
    unsigned int i;
    struct volk_gnsssdr_machine *max_machine = NULL;
    for(i=0; i<n_volk_gnsssdr_machines; i++) {
      if(!(volk_gnsssdr_machines[i]->caps & (~volk_gnsssdr_get_lvarch()))) {
        if(volk_gnsssdr_machines[i]->caps > max_score) {
          max_score = volk_gnsssdr_machines[i]->caps;
          max_machine = volk_gnsssdr_machines[i];
        }
      }
    }
    machine = max_machine;
    return machine->name;
  }
}

size_t volk_gnsssdr_get_alignment(void)
{
    get_machine(); //ensures alignment is set
    return __alignment;
}

bool volk_gnsssdr_is_aligned(const void *ptr)
{
    return ((intptr_t)(ptr) & __alignment_mask) == 0;
}

#define LV_HAVE_GENERIC
#define LV_HAVE_DISPATCHER

%for kern in kernels:

%if kern.has_dispatcher:
#include <volk_gnsssdr/${kern.name}.h> //pulls in the dispatcher
%endif

static inline void __${kern.name}_d(${kern.arglist_full})
{
    %if kern.has_dispatcher:
    ${kern.name}_dispatcher(${kern.arglist_names});
    return;
    %endif

    if (volk_gnsssdr_is_aligned(<% num_open_parens = 0 %>
    %for arg_type, arg_name in kern.args:
        %if '*' in arg_type:
        VOLK_OR_PTR(${arg_name},<% num_open_parens += 1 %>
        %endif
    %endfor
        0<% end_open_parens = ')'*num_open_parens %>${end_open_parens}
    )){
        ${kern.name}_a(${kern.arglist_names});
    }
    else{
        ${kern.name}_u(${kern.arglist_names});
    }
}

static inline void __init_${kern.name}(void)
{
    const char *name = get_machine()->${kern.name}_name;
    const char **impl_names = get_machine()->${kern.name}_impl_names;
    const int *impl_deps = get_machine()->${kern.name}_impl_deps;
    const bool *alignment = get_machine()->${kern.name}_impl_alignment;
    const size_t n_impls = get_machine()->${kern.name}_n_impls;
    const size_t index_a = volk_gnsssdr_rank_archs(name, impl_names, impl_deps, alignment, n_impls, true/*aligned*/);
    const size_t index_u = volk_gnsssdr_rank_archs(name, impl_names, impl_deps, alignment, n_impls, false/*unaligned*/);
    ${kern.name}_a = get_machine()->${kern.name}_impls[index_a];
    ${kern.name}_u = get_machine()->${kern.name}_impls[index_u];

    assert(${kern.name}_a);
    assert(${kern.name}_u);

    ${kern.name} = &__${kern.name}_d;
}

static inline void __${kern.name}_a(${kern.arglist_full})
{
    __init_${kern.name}();
    ${kern.name}_a(${kern.arglist_names});
}

static inline void __${kern.name}_u(${kern.arglist_full})
{
    __init_${kern.name}();
    ${kern.name}_u(${kern.arglist_names});
}

static inline void __${kern.name}(${kern.arglist_full})
{
    __init_${kern.name}();
    ${kern.name}(${kern.arglist_names});
}

${kern.pname} ${kern.name}_a = &__${kern.name}_a;
${kern.pname} ${kern.name}_u = &__${kern.name}_u;
${kern.pname} ${kern.name}   = &__${kern.name};

void ${kern.name}_manual(${kern.arglist_full}, const char* impl_name)
{
    const int index = volk_gnsssdr_get_index(
        get_machine()->${kern.name}_impl_names,
        get_machine()->${kern.name}_n_impls,
        impl_name
    );
    get_machine()->${kern.name}_impls[index](
        ${kern.arglist_names}
    );
}

volk_gnsssdr_func_desc_t ${kern.name}_get_func_desc(void) {
    const char **impl_names = get_machine()->${kern.name}_impl_names;
    const int *impl_deps = get_machine()->${kern.name}_impl_deps;
    const bool *alignment = get_machine()->${kern.name}_impl_alignment;
    const size_t n_impls = get_machine()->${kern.name}_n_impls;
    volk_gnsssdr_func_desc_t desc = {
        impl_names,
        impl_deps,
        alignment,
        n_impls
    };
    return desc;
}

%endfor
