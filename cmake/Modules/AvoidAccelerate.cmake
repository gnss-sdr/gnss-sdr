# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

# Avoid using the BLAS and LAPACK implementations that comes with the Accelerate
# framework, which causes a bug when the BeiDou constellation is enabled

if(NOT BLAS_ROOT)
    set(BLAS_ROOT_USER_DEFINED /usr/local/lib)
else()
    set(BLAS_ROOT_USER_DEFINED ${BLAS_ROOT})
endif()
if(DEFINED ENV{BLAS_ROOT})
    set(BLAS_ROOT_USER_DEFINED
        ${BLAS_ROOT_USER_DEFINED}
        $ENV{BLAS_ROOT}
    )
endif()

find_library(BLAS_LIBRARIES
    NAMES libblas.dylib libopenblas.dylib
    PATHS
        ${BLAS_ROOT_USER_DEFINED}
        ${BLAS_ROOT_USER_DEFINED}/lapack
        /opt/local/lib/lapack
        /opt/local/lib/
        /usr/local/opt/lapack/lib
    NO_DEFAULT_PATH
    NO_SYSTEM_ENVIRONMENT_PATH
    NO_CMAKE_ENVIRONMENT_PATH
    NO_SYSTEM_ENVIRONMENT_PATH
    NO_CMAKE_SYSTEM_PATH
)

if(BLAS_LIBRARIES)
    set(BLAS_FOUND TRUE)
    message(STATUS "BLAS library found at ${BLAS_LIBRARIES}")
endif()


find_library(LAPACK_LIBRARIES
    liblapack.dylib
    PATHS
        ${BLAS_ROOT_USER_DEFINED}
        ${BLAS_ROOT_USER_DEFINED}/lapack
        /opt/local/lib/lapack
        /usr/local/opt/lapack/lib
    NO_DEFAULT_PATH
    NO_SYSTEM_ENVIRONMENT_PATH
    NO_CMAKE_ENVIRONMENT_PATH
    NO_SYSTEM_ENVIRONMENT_PATH
    NO_CMAKE_SYSTEM_PATH
)

if(LAPACK_LIBRARIES)
    set(LAPACK_FOUND TRUE)
    message(STATUS "LAPACK library found at ${LAPACK_LIBRARIES}")
endif()
