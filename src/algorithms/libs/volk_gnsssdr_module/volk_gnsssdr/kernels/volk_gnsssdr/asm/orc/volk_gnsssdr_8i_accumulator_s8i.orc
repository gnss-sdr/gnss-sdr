#
# ORC implementation: 8 bits (char) scalar accumulator
#
# Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
#
# ORC code that implements an accumulator of char values
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

.function volk_gnsssdr_8i_accumulator_s8i_a_orc_impl
.source 1 src1
.accumulator 2 acc
.temp 2 sum
mergebw sum, 0, src1
accw acc, sum
