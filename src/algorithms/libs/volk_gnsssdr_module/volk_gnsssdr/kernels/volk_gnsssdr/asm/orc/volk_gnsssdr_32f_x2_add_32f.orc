# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
# SPDX-License-Identifier: GPL-3.0-or-later

.function volk_gnsssdr_32f_x2_add_32f_a_orc_impl
.dest 4 dst
.source 4 src1
.source 4 src2
addf dst, src1, src2
