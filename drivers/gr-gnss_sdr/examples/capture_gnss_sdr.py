#!/usr/bin/env python2
##################################################
# GNU Radio Python Flow Graph
# Title: GNSS SDR Capture Tool
# Author: Ajith Peter
# Description: Capture tool for GNSS-SDR Hacker's Edition
# Generated: Mon Jul 13 16:03:02 2015
##################################################

from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import gnss_sdr

class capture_gnss_sdr(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "GNSS SDR Capture Tool")

        ##################################################
        # Blocks
        ##################################################
        self.gnss_sdr_gnss_sdr_source_b_0 = gnss_sdr.gnss_sdr_source_b()
        self.blocks_file_sink_0 = blocks.file_sink(gr.sizeof_char*1, "capture_log", False)
        self.blocks_file_sink_0.set_unbuffered(False)

        ##################################################
        # Configuration
        ##################################################

	# Connect to LNA1 Antenna Port
	self.gnss_sdr_gnss_sdr_source_b_0.set_frontend_register_LNAMODE(2)

	# Maximum current on ILNA1
	self.gnss_sdr_gnss_sdr_source_b_0.set_frontend_register_ILNA1(15)

	# I and Q enable
	self.gnss_sdr_gnss_sdr_source_b_0.set_frontend_register_IQEN(1)

	# Two's Complement Format
	self.gnss_sdr_gnss_sdr_source_b_0.set_frontend_register_FORMAT(3)

	# 2 Bits I and Q
	self.gnss_sdr_gnss_sdr_source_b_0.set_frontend_register_BITS(2)

	# AGC set by GAININ
	self.gnss_sdr_gnss_sdr_source_b_0.set_frontend_register_AGCMODE(2)

	# PGA Gain
	self.gnss_sdr_gnss_sdr_source_b_0.set_frontend_register_GAININ(63)

	# Enable PGA on Q Channel
	self.gnss_sdr_gnss_sdr_source_b_0.set_frontend_register_PGAQEN(1)

	# Disable DSP Streaming Interface
	self.gnss_sdr_gnss_sdr_source_b_0.set_frontend_register_STRMEN(0)

	# PLL Integer Division Ratio
	self.gnss_sdr_gnss_sdr_source_b_0.set_frontend_register_NDIV(7857)

	# PLL Reference Division Ratio
	self.gnss_sdr_gnss_sdr_source_b_0.set_frontend_register_RDIV(96)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.gnss_sdr_gnss_sdr_source_b_0, 0), (self.blocks_file_sink_0, 0))    



if __name__ == '__main__':
    parser = OptionParser(option_class=eng_option, usage="%prog: [options]")
    (options, args) = parser.parse_args()
    if gr.enable_realtime_scheduling() != gr.RT_OK:
        print "Error: failed to enable realtime scheduling."
    tb = capture_gnss_sdr()
    tb.start()
    try:
        raw_input('Press Enter to quit: ')
    except EOFError:
        pass
    tb.stop()
    tb.wait()
