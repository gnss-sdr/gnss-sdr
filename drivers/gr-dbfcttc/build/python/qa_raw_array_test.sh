#!/bin/sh
export GR_DONT_LOAD_PREFS=1
export srcdir=/home/usuario/sdr/gnss-sdr/gnss-sdr/drivers/gr-dbfcttc/python
export PATH=/home/usuario/sdr/gnss-sdr/gnss-sdr/drivers/gr-dbfcttc/build/python:$PATH
export LD_LIBRARY_PATH=/home/usuario/sdr/gnss-sdr/gnss-sdr/drivers/gr-dbfcttc/build/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/home/usuario/sdr/gnss-sdr/gnss-sdr/drivers/gr-dbfcttc/build/swig:$PYTHONPATH
/usr/bin/python2 /home/usuario/sdr/gnss-sdr/gnss-sdr/drivers/gr-dbfcttc/python/qa_raw_array.py 
