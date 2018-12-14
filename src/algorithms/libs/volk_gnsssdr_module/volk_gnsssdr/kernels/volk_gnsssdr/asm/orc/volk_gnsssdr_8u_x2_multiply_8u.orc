#
# ORC implementation: multiplies unsigned char values
# 
# Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
# 
#
# ORC code that multiplies unsigned char values (8 bits data)
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

.function volk_gnsssdr_8u_x2_multiply_8u_a_orc_impl
.source 1 src1
.source 1 src2
.dest 1 dst
mullb dst, src1, src2
