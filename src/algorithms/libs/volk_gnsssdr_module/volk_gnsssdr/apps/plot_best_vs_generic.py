#!/usr/bin/env python
#
# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
# SPDX-License-Identifier: GPL-3.0-or-later

# This script is used to compare the generic kernels to the highest performing kernel, for each operation
# Run:
#   ./volk_gnsssdr_profile -j volk_gnsssdr_results.json
# Then run this script under python3

import matplotlib.pyplot as plt
import numpy as np
import json

filename = 'volk_gnsssdr_results.json'

operations = []
metrics = []
with open(filename) as json_file:
    data = json.load(json_file)
    for test in data['volk_gnsssdr_tests']:
        if ('generic' in test['results']) or ('u_generic' in test['results']): # some dont have a generic kernel
            operations.append(test['name'][13:]) # remove volk_gnsssdr_ prefix that they all have
            extension_performance = []
            for key, val in test['results'].items():
                if key not in ['generic', 'u_generic']: # exclude generic results, when trying to find fastest time
                    extension_performance.append(val['time'])
            try:
                generic_time = test['results']['generic']['time']
            except:
                generic_time = test['results']['u_generic']['time']
            metrics.append(extension_performance[np.argmin(extension_performance)]/generic_time)

plt.bar(np.arange(len(metrics)), metrics)
plt.hlines(1.0, -1, len(metrics), colors='r', linestyles='dashed')
plt.axis([-1, len(metrics), 0, 2])
plt.xticks(np.arange(len(operations)), operations, rotation=90)
plt.ylabel('Time taken of fastest kernel relative to generic kernel')
plt.show()
