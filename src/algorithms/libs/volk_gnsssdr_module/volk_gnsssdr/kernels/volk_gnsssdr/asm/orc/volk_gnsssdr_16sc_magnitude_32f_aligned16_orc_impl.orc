# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
# SPDX-License-Identifier: GPL-3.0-or-later

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
