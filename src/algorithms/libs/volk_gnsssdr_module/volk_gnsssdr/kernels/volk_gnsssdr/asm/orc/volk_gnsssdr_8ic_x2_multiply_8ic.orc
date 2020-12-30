#
# ORC implementation: multiplies two 16 bits vectors
#
# Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
#
#
# ORC code that multiplies two 16 bits vectors (8 bits the real part
# and 8 bits the imaginary part)
#
#
# ------------------------------------------------------------------------------
#
# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
# SPDX-License-Identifier: GPL-3.0-or-later
#
# ------------------------------------------------------------------------------
#

.function volk_gnsssdr_8ic_x2_multiply_8ic_a_orc_impl
.source 2 src1
.source 2 src2
.dest 2 dst
.temp 2 iqprod
.temp 1 real
.temp 1 imag
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
mergebw dst, real, imag
