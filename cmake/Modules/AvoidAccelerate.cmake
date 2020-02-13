# Copyright (C) 2011-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later

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
