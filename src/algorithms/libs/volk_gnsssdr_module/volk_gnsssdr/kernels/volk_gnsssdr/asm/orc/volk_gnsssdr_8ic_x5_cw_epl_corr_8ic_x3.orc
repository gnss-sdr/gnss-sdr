#
# ORC implementation: performs the carrier wipe-off mixing and the Early, Prompt, and Late correlation with 16 bits vectors
# Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
# 
# 
# ORC code that performs the carrier wipe-off mixing and the
# Early, Prompt, and Late correlation with 16 bits vectors (8 bits the
# real part and 8 bits the imaginary part):
# - The carrier wipe-off is done by multiplying the input signal by the
#   carrier (multiplication of 16 bits vectors) It returns the input
#   signal in base band (BB)
# - Early values are calculated by multiplying the input signal in BB by the
#   early code (multiplication of 16 bits vectors), accumulating the results
# - Prompt values are calculated by multiplying the input signal in BB by the
#   prompt code (multiplication of 16 bits vectors), accumulating the results
# - Late values are calculated by multiplying the input signal in BB by the
#   late code (multiplication of 16 bits vectors), accumulating the results
# 
# 
# -------------------------------------------------------------------------
# 
# Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
# 
# GNSS-SDR is a software defined Global Navigation
#          Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# GNSS-SDR is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# GNSS-SDR is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
#
# -------------------------------------------------------------------------
# 

.function volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3_first_a_orc_impl
.source 2 input
.source 2 carrier
.source 2 E_code
.source 2 P_code
.accumulator 2 E_out_real
.accumulator 2 E_out_imag
.accumulator 2 P_out_real
.accumulator 2 P_out_imag
.temp 2 bb_signal_sample
.temp 2 iqprod
.temp 1 real
.temp 1 imag
.temp 1 ac
.temp 1 bd
.temp 2 swapped

.temp 2 real2
.temp 2 imag2

x2 mullb iqprod, input, carrier
splitwb bd, ac, iqprod
subb real, ac, bd
swapw swapped, input
x2 mullb iqprod, swapped, carrier
splitwb bd, ac, iqprod
addb imag, ac, bd
mergebw bb_signal_sample, real, imag

swapw swapped, bb_signal_sample

x2 mullb iqprod, bb_signal_sample, E_code
splitwb bd, ac, iqprod
subb real, ac, bd
x2 mullb iqprod, swapped, E_code
splitwb bd, ac, iqprod
addb imag, ac, bd
mergebw real2, 0, real
mergebw imag2, 0, imag
accw E_out_real, real2
accw E_out_imag, imag2

x2 mullb iqprod, bb_signal_sample, P_code
splitwb bd, ac, iqprod
subb real, ac, bd
x2 mullb iqprod, swapped, P_code
splitwb bd, ac, iqprod
addb imag, ac, bd
mergebw real2, 0, real
mergebw imag2, 0, imag
accw P_out_real, real2
accw P_out_imag, imag2

.function volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3_second_a_orc_impl
.source 2 input
.source 2 carrier
.source 2 L_code
.accumulator 2 L_out_real
.accumulator 2 L_out_imag

.temp 2 bb_signal_sample
.temp 2 iqprod
.temp 1 real
.temp 1 imag
.temp 1 ac
.temp 1 bd
.temp 2 swapped

.temp 2 real2
.temp 2 imag2

x2 mullb iqprod, input, carrier
splitwb bd, ac, iqprod
subb real, ac, bd
swapw swapped, input
x2 mullb iqprod, swapped, carrier
splitwb bd, ac, iqprod
addb imag, ac, bd
mergebw bb_signal_sample, real, imag

swapw swapped, bb_signal_sample

x2 mullb iqprod, bb_signal_sample, L_code
splitwb bd, ac, iqprod
subb real, ac, bd
x2 mullb iqprod, swapped, L_code
splitwb bd, ac, iqprod
addb imag, ac, bd
mergebw real2, 0, real
mergebw imag2, 0, imag
accw L_out_real, real2
accw L_out_imag, imag2


