#
# ORC implementation: calculates the magnitude squared of a 16 bits vector
# Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
# 
# 
# ORC code that calculates the magnitude squared of a
# 16 bits vector (8 bits the real part and 8 bits the imaginary part)
# result = (real*real) + (imag*imag)
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

.function volk_gnsssdr_8ic_magnitude_squared_8i_a_orc_impl
.source 2 src1
.dest 1 dst
.temp 2 iqprod
.temp 1 ac
.temp 1 bd
x2 mullb iqprod, src1, src1
splitwb bd, ac, iqprod
addb dst, ac, bd
