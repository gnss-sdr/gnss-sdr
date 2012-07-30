/* -*- c++ -*- */

#define GN3S_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "gn3s_swig_doc.i"


%{
#include "gn3s_source_cc.h"
%}

GR_SWIG_BLOCK_MAGIC(gn3s,source_cc);
%include "gn3s_source_cc.h"

