# This is VOLK_GNSSSDR, the Vector Optimized Library of Kernels for GNSS-SDR
#
#
# Copyright (C) 2010-2015 (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# GNSS-SDR is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#  
# GNSS-SDR is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.

This is a set of extra kernels that can be used in combination with VOLK's.
Please see http://libvolk.org for bug tracking, documentation, source code, and 
contact information about VOLK.

The boilerplate of this code was originally generated with volk_modtool, and then
some modifications were added to accomodate the specificities of these kernels.
This library contains kernels of hand-written SIMD code for different mathematical
operations, mainly with 8-bit and 16-bit data types, offering a generic, 
platform/architecture agnostic version that will run in all machines, plus versions 
for different SIMD instruction sets. Then, the application volk_gnsssdr_profile runs
all versions that your machine can execute and annotates which is the fastest, that 
will then be selected at runtime when executing GNSS-SDR. In this way, we can  address
at the same time portability (by creating executables that will run in nearly 
all processor architectures) and efficiency (by providing custom implementations 
specially designed to take advantage of the specific processor that is running the code).

These kernels have some specific features (i.e. saturation arithmetics) that are
aimed to Global Navigation Satellite Systems signal processing, but could make
them not suitable for its general use in other applications.


How to use VOLK_GNSSSDR:

It is automatically built and installed along with GNSS-SDR if it is not found by 
CMake on your system at configure time.

However, you can install and use VOLK_GNSSSDR kernels as you use VOLK's, independently 
from GNSS-SDR.

Build with cmake:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ make test
    $ sudo make install

That's it!

Before its first use, please execute volk_gnsssdr_profile to let the system know which 
is the fastest available implementation. This only has to be done once:

    $ volk_gnsssdr_profile
    
From now on, GNSS-SDR (and any other program of your own that makes use of VOLK_GNSSSDR) 
will benefit from the acceleration provided by SIMD instructions.


VOLK_GNSSSDR was originally created by Andres Cecilia Luque in the framework of the 
Summer Of Code In Space (SOCIS 2014) program organized by the European Space Agency.
