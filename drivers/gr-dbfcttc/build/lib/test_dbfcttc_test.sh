#!/bin/sh
export GR_DONT_LOAD_PREFS=1
export srcdir=/home/usuario/sdr/gnss-sdr/gnss-sdr/drivers/gr-dbfcttc/lib
export PATH=/home/usuario/sdr/gnss-sdr/gnss-sdr/drivers/gr-dbfcttc/build/lib:$PATH
export LD_LIBRARY_PATH=/home/usuario/sdr/gnss-sdr/gnss-sdr/drivers/gr-dbfcttc/build/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$PYTHONPATH
test-dbfcttc 
