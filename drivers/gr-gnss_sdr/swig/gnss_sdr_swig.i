/* -*- c++ -*- */

#define GNSS_SDR_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "gnss_sdr_swig_doc.i"

%{
#include "gnss_sdr/gnss_sdr_source_b.h"
%}


%include "gnss_sdr/gnss_sdr_source_b.h"
GR_SWIG_BLOCK_MAGIC2(gnss_sdr, gnss_sdr_source_b);
