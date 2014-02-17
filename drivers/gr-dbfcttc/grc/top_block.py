#!/usr/bin/env python
##################################################
# Gnuradio Python Flow Graph
# Title: Top Block
# Generated: Tue Feb 11 16:02:41 2014
##################################################

from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio import wxgui
from gnuradio.eng_option import eng_option
from gnuradio.fft import window
from gnuradio.filter import firdes
from gnuradio.wxgui import fftsink2
from grc_gnuradio import wxgui as grc_wxgui
from optparse import OptionParser
import dbfcttc
import wx

class top_block(grc_wxgui.top_block_gui):

    def __init__(self):
        grc_wxgui.top_block_gui.__init__(self, title="Top Block")

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 32000

        ##################################################
        # Blocks
        ##################################################
        self.wxgui_fftsink2_0 = fftsink2.fft_sink_c(
        	self.GetWin(),
        	baseband_freq=0,
        	y_per_div=10,
        	y_divs=10,
        	ref_level=0,
        	ref_scale=2.0,
        	sample_rate=samp_rate,
        	fft_size=1024,
        	fft_rate=15,
        	average=False,
        	avg_alpha=None,
        	title="FFT Plot",
        	peak_hold=False,
        )
        self.Add(self.wxgui_fftsink2_0.win)
        self.dbfcttc_raw_array_0 = dbfcttc.raw_array("en4", 8, 70, 10, 5000000)
        self.blocks_null_sink_1_5 = blocks.null_sink(gr.sizeof_gr_complex*1)
        self.blocks_null_sink_1_4 = blocks.null_sink(gr.sizeof_gr_complex*1)
        self.blocks_null_sink_1_2 = blocks.null_sink(gr.sizeof_gr_complex*1)
        self.blocks_null_sink_1_1 = blocks.null_sink(gr.sizeof_gr_complex*1)
        self.blocks_null_sink_1_0 = blocks.null_sink(gr.sizeof_gr_complex*1)
        self.blocks_null_sink_1 = blocks.null_sink(gr.sizeof_gr_complex*1)
        self.blocks_null_sink_0 = blocks.null_sink(gr.sizeof_gr_complex*1)

        ##################################################
        # Connections
        ##################################################
        self.connect((self.dbfcttc_raw_array_0, 1), (self.blocks_null_sink_0, 0))
        self.connect((self.dbfcttc_raw_array_0, 2), (self.blocks_null_sink_1_0, 0))
        self.connect((self.dbfcttc_raw_array_0, 3), (self.blocks_null_sink_1, 0))
        self.connect((self.dbfcttc_raw_array_0, 4), (self.blocks_null_sink_1_1, 0))
        self.connect((self.dbfcttc_raw_array_0, 5), (self.blocks_null_sink_1_2, 0))
        self.connect((self.dbfcttc_raw_array_0, 6), (self.blocks_null_sink_1_4, 0))
        self.connect((self.dbfcttc_raw_array_0, 7), (self.blocks_null_sink_1_5, 0))
        self.connect((self.dbfcttc_raw_array_0, 0), (self.wxgui_fftsink2_0, 0))


# QT sink close method reimplementation

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.wxgui_fftsink2_0.set_sample_rate(self.samp_rate)

if __name__ == '__main__':
    import ctypes
    import os
    if os.name == 'posix':
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"
    parser = OptionParser(option_class=eng_option, usage="%prog: [options]")
    (options, args) = parser.parse_args()
    tb = top_block()
    tb.Start(True)
    tb.Wait()

