#!/usr/bin/env python
#
# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
# SPDX-License-Identifier: GPL-3.0-or-later

from __future__ import print_function

import argparse
import os
import sys

sys.dont_write_bytecode = True

def extract_generated_python_dir(args):
    option = '--generated-python-dir'
    if option in args:
        option_index = args.index(option)
        if option_index + 1 < len(args):
            return args[option_index + 1]
    option_prefix = option + '='
    for arg in args:
        if arg.startswith(option_prefix):
            return arg[len(option_prefix):]
    return os.environ.get('VOLK_GNSSSDR_GENERATED_PYTHON_DIR')

generated_python_dir = extract_generated_python_dir(sys.argv[1:])
if generated_python_dir:
    sys.path.insert(0, generated_python_dir)

import volk_gnsssdr_arch_defs
import volk_gnsssdr_machine_defs

def do_arch_flags_list(compiler):
    output = list()
    for arch in volk_gnsssdr_arch_defs.archs:
        if not arch.is_supported(compiler): continue
        fields = [arch.name] + arch.get_flags(compiler)
        output.append(','.join(fields))
    print(';'.join(output))


def do_machines_list(arch_names):
    output = list()
    for machine in volk_gnsssdr_machine_defs.machines:
        machine_arch_set = set(machine.arch_names)
        if set(arch_names).intersection(machine_arch_set) == machine_arch_set:
            output.append(machine.name)
    print(';'.join(output))


def do_machine_flags_list(compiler, machine_name):
    output = list()
    machine = volk_gnsssdr_machine_defs.machine_dict[machine_name]
    for arch in machine.archs:
        output.extend(arch.get_flags(compiler))
    print(' '.join(output))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str)
    parser.add_argument('--compiler', type=str)
    parser.add_argument('--archs', type=str)
    parser.add_argument('--machine', type=str)
    parser.add_argument('--generated-python-dir', type=str)
    args = parser.parse_args()

    if args.mode == 'arch_flags': return do_arch_flags_list(args.compiler.lower())
    if args.mode == 'machines': return do_machines_list(args.archs.split(';'))
    if args.mode == 'machine_flags': return do_machine_flags_list(args.compiler.lower(), args.machine)

if __name__ == '__main__':
    main()
