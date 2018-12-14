#
# ORC implementation: multiplies a group of 16 bits vectors by one constant vector
#
#  Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
#
# 
# ORC code that multiplies a group of 16 bits vectors
# (8 bits the real part and 8 bits the imaginary part) by one constant vector
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

.function volk_gnsssdr_8ic_s8ic_multiply_8ic_a_orc_impl
.source 2 src1
.param 2 src2real
.param 2 src2imag
.dest 2 dst
.temp 2 iqprod
.temp 1 real
.temp 1 imag
.temp 1 rr
.temp 1 ii
.temp 1 ri
.temp 1 ir
x2 mullb iqprod, src1, src2real
splitwb ir, rr, iqprod
x2 mullb iqprod, src1, src2imag
splitwb ii, ri, iqprod
subb real, rr, ii
addb imag, ri, ir
mergebw dst, real, imag




