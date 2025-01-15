#!/usr/bin/env python
#
# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
# SPDX-License-Identifier: GPL-3.0-or-later

from __future__ import print_function
import sys
if sys.version_info[0] < 3:
    import six

from volk_gnsssdr_arch_defs import arch_dict

machines = list()
machine_dict = dict()

class machine_class(object):
    def __init__(self, name, archs):
        self.name = name
        self.archs = list()
        self.arch_names = list()
        for arch_name in archs:
            if not arch_name: continue
            arch = arch_dict[arch_name]
            self.archs.append(arch)
            self.arch_names.append(arch_name)
        self.alignment = max([a.alignment for a in self.archs])

    def __repr__(self): return self.name

def register_machine(name, archs):
    for i, arch_name in enumerate(archs):
        if '|' in arch_name: #handle special arch names with the '|'
            for arch_sub in arch_name.split('|'):
                if arch_sub:
                    register_machine(name+'_'+arch_sub, archs[:i] + [arch_sub] + archs[i+1:])
                else:
                    register_machine(name, archs[:i] + archs[i+1:])
            return
    machine = machine_class(name=name, archs=archs)
    machines.append(machine)
    machine_dict[machine.name] = machine

########################################################################
# register the machines
########################################################################
#TODO skip the XML and put it here
from xml.dom import minidom
import os
gendir = os.path.dirname(__file__)
machines_xml = minidom.parse(os.path.join(gendir, 'machines.xml')).getElementsByTagName('machine')
for machine_xml in machines_xml:
    kwargs = dict()
    for attr in machine_xml.attributes.keys():
        kwargs[attr] = machine_xml.attributes[attr].value
    for node in machine_xml.childNodes:
        try:
            name = node.tagName
            val = machine_xml.getElementsByTagName(name)[0].firstChild.data
            kwargs[name] = val
        except: pass
    kwargs['archs'] = kwargs['archs'].split()
    #force kwargs keys to be of type str, not unicode for py25
    if sys.version_info[0] < 3:
        kwargs = dict((str(k), v) for k, v in six.iteritems(kwargs))
    register_machine(**kwargs)

if __name__ == '__main__':
    print(machines)
