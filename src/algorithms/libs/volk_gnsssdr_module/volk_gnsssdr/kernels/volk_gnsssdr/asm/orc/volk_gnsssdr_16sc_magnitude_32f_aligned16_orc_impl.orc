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

.function volk_gnsssdr_16ic_magnitude_32f_a_orc_impl
.source 4 src
.dest 4 dst
.floatparam 4 scalar
.temp 4 reall
.temp 4 imagl
.temp 2 reals
.temp 2 imags
.temp 4 realf
.temp 4 imagf
.temp 4 sumf



splitlw reals, imags, src
convswl reall, reals
convswl imagl, imags
convlf realf, reall
convlf imagf, imagl
divf realf, realf, scalar
divf imagf, imagf, scalar
mulf realf, realf, realf
mulf imagf, imagf, imagf
addf sumf, realf, imagf
sqrtf dst, sumf
