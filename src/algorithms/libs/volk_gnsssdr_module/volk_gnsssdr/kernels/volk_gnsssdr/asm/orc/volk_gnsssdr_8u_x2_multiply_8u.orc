#
# ORC implementation: multiplies unsigned char values
#
# Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
#
#
# ORC code that multiplies unsigned char values (8 bits data)
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

.function volk_gnsssdr_8u_x2_multiply_8u_a_orc_impl
.source 1 src1
.source 1 src2
.dest 1 dst
mullb dst, src1, src2
