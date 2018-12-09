#!/bin/sh
# GNSS-SDR shell script that enables the remote GNSS-SDR restart telecommand
# usage: ./gnss-sdr-harness.sh ./gnss-sdr -c config_file.conf
echo $@
$@
while [ $? -eq 42 ]
do
 echo "restarting GNSS-SDR..."
 $@
done
