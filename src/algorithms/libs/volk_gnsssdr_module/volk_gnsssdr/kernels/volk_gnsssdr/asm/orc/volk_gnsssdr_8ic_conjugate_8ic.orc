#
# ORC implementation: calculates the conjugate of a 16 bits vector
#
# Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
#
# ORC code that calculates the conjugate of a
# 16 bits vector (8 bits the real part and 8 bits the imaginary part)
# result = (real*real) + (imag*imag)
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

.function volk_gnsssdr_8ic_conjugate_8ic_a_orc_impl
.source 2 src1
.dest 2 dst
.temp 2 merged
mergebw merged, 1, -1
x2 mullb dst, merged, src1
