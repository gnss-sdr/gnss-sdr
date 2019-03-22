# Copyright (C) 2011-2019 (see AUTHORS file for a list of contributors)
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

# Avoid using the BLAS and LAPACK implementations that comes with the Accelerate
# framework, which causes a bug when the BeiDou constellation is enabled

find_library(BLAS_LIBRARIES
  libblas.dylib
  PATHS
    /opt/local/lib/lapack
    /usr/local/opt/lapack/lib
    /usr/local/lib
    ${BLAS_ROOT}/lib
    $ENV{BLAS_ROOT}/lib
  NO_DEFAULT_PATH
  NO_SYSTEM_ENVIRONMENT_PATH
  NO_CMAKE_ENVIRONMENT_PATH
  NO_SYSTEM_ENVIRONMENT_PATH
  NO_CMAKE_SYSTEM_PATH
)

if(BLAS_LIBRARIES)
  set(BLAS_FOUND TRUE)
endif()


find_library(LAPACK_LIBRARIES
  liblapack.dylib
  PATHS
    /opt/local/lib/lapack
    /usr/local/opt/lapack/lib
    /usr/local/lib
    ${BLAS_ROOT}/lib
    $ENV{BLAS_ROOT}/lib
  NO_DEFAULT_PATH
  NO_SYSTEM_ENVIRONMENT_PATH
  NO_CMAKE_ENVIRONMENT_PATH
  NO_SYSTEM_ENVIRONMENT_PATH
  NO_CMAKE_SYSTEM_PATH
)

if(LAPACK_LIBRARIES)
  set(LAPACK_FOUND TRUE)
endif()
