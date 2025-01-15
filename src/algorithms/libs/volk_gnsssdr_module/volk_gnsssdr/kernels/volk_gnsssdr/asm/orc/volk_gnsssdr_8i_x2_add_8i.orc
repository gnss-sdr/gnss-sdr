#
# ORC implementation: adds pairs of 8 bits (char) scalars
#
# Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
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

.function volk_gnsssdr_8i_x2_add_8i_a_orc_impl
.dest 1 dst
.source 1 src1
.source 1 src2
addb dst, src1, src2
