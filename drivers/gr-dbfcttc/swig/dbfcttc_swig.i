/* -*- c++ -*- */

#define DBFCTTC_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "dbfcttc_swig_doc.i"

%{
#include "dbfcttc/raw_array.h"
%}


%include "dbfcttc/raw_array.h"
GR_SWIG_BLOCK_MAGIC2(dbfcttc, raw_array);
