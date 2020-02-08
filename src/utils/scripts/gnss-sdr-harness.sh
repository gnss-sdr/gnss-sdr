#!/bin/sh
# GNSS-SDR shell script that enables the remote GNSS-SDR restart telecommand
# usage: ./gnss-sdr-harness.sh ./gnss-sdr -c config_file.conf

# SPDX-FileCopyrightText: Javier Arribas <javier.arribas(at)cttc.es>
# SPDX-License-Identifier: GPL-3.0-or-later

echo $@
$@
while [ $? -eq 42 ]
do
 echo "restarting GNSS-SDR..."
 $@
done
