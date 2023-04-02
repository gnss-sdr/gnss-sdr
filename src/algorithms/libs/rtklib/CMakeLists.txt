# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


set(RTKLIB_LIB_SOURCES
    rtklib_rtkcmn.cc
    rtklib_ephemeris.cc
    rtklib_preceph.cc
    rtklib_sbas.cc
    rtklib_ionex.cc
    rtklib_pntpos.cc
    rtklib_ppp.cc
    rtklib_tides.cc
    rtklib_lambda.cc
    rtklib_rtkpos.cc
    rtklib_conversions.cc
    rtklib_stream.cc
    rtklib_rtksvr.cc
    rtklib_solution.cc
    rtklib_rtcm.cc
    rtklib_rtcm2.cc
    rtklib_rtcm3.cc
)

set(RTKLIB_LIB_HEADERS
    rtklib_rtkcmn.h
    rtklib_ephemeris.h
    rtklib_preceph.h
    rtklib_sbas.h
    rtklib_ionex.h
    rtklib_pntpos.h
    rtklib_ppp.h
    rtklib_tides.h
    rtklib_lambda.h
    rtklib_rtkpos.h
    rtklib_conversions.h
    rtklib_stream.h
    rtklib_rtksvr.h
    rtklib_solution.h
    rtklib_rtcm.h
    rtklib_rtcm2.h
    rtklib_rtcm3.h
    rtklib.h
)

list(SORT RTKLIB_LIB_HEADERS)
list(SORT RTKLIB_LIB_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(algorithms_libs_rtklib STATIC)
    target_sources(algorithms_libs_rtklib
        PRIVATE
            ${RTKLIB_LIB_SOURCES}
        PUBLIC
            ${RTKLIB_LIB_HEADERS}
    )
else()
    source_group(Headers FILES ${RTKLIB_LIB_HEADERS})
    add_library(algorithms_libs_rtklib
        ${RTKLIB_LIB_SOURCES}
        ${RTKLIB_LIB_HEADERS}
    )
endif()

target_link_libraries(algorithms_libs_rtklib
    PRIVATE
        core_system_parameters
        algorithms_libs
        Armadillo::armadillo
        Gflags::gflags
        Glog::glog
        LAPACK::LAPACK
        BLAS::BLAS
)

if(ENABLE_ARMA_NO_DEBUG)
    target_compile_definitions(algorithms_libs_rtklib
        PRIVATE -DARMA_NO_BOUND_CHECKING=1
    )
endif()

if(FILESYSTEM_FOUND)
    target_compile_definitions(algorithms_libs_rtklib PUBLIC -DHAS_STD_FILESYSTEM=1)
    if(find_experimental)
        target_compile_definitions(algorithms_libs_rtklib PUBLIC -DHAS_STD_FILESYSTEM_EXPERIMENTAL=1)
    endif()
    target_link_libraries(algorithms_libs_rtklib PUBLIC std::filesystem)
else()
    target_link_libraries(algorithms_libs_rtklib PUBLIC Boost::filesystem Boost::system)
endif()

set_property(TARGET algorithms_libs_rtklib
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
