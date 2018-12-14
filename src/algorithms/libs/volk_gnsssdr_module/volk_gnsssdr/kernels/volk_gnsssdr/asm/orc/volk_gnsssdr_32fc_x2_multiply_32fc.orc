# Copyright (C) 2010-2018 (see AUTHORS file for a list of contributors)
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

.function volk_gnsssdr_32fc_x2_multiply_32fc_a_orc_impl
.source 8 src1
.source 8 src2
.dest 8 dst
.temp 8 iqprod
.temp 4 real
.temp 4 imag
.temp 4 ac
.temp 4 bd
.temp 8 swapped
x2 mulf iqprod, src1, src2
splitql bd, ac, iqprod
subf real, ac, bd
swaplq swapped, src1
x2 mulf iqprod, swapped, src2
splitql bd, ac, iqprod
addf imag, ac, bd
mergelq dst, real, imag
