#
# ORC implementation: multiplies two 16 bits vectors and accumulates them
#
# Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
# 
# 
# ORC code that multiplies two 16 bits vectors (8 bits the real part
# and 8 bits the imaginary part) and accumulates them
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

.function volk_gnsssdr_8ic_x2_dot_prod_8ic_a_orc_impl
.source 2 src1
.source 2 src2
.accumulator 2 accreal
.accumulator 2 accimag
.temp 2 iqprod
.temp 1 real
.temp 1 imag
.temp 2 real2
.temp 2 imag2
.temp 1 ac
.temp 1 bd
.temp 2 swapped
x2 mullb iqprod, src1, src2
splitwb bd, ac, iqprod
subb real, ac, bd
swapw swapped, src1
x2 mullb iqprod, swapped, src2
splitwb bd, ac, iqprod
addb imag, ac, bd
mergebw real2, 0, real
accw accreal, real2
mergebw imag2, 0, imag
accw accimag, imag2
