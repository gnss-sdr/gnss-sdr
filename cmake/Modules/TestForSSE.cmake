# Copyright (C) 2011-2018 (see AUTHORS file for a list of contributors)
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


###############################################################################
# Test for availability of SSE
#
# - Anthony Arnold
###############################################################################

function(test_for_sse h_file result_var name)
  if(NOT DEFINED ${result_var})
    execute_process(COMMAND echo "#include <${h_file}>"
      COMMAND ${CMAKE_CXX_COMPILER} ${CMAKE_CXX_COMPILER_ARG1} -c -x c++ -
      RESULT_VARIABLE COMPILE_RESULT
      OUTPUT_QUIET ERROR_QUIET)
    set(detected 0)
    if(COMPILE_RESULT EQUAL 0)
      message(STATUS "Detected ${name}")
      set(detected 1)
    endif()
    set(${result_var} ${detected} CACHE INTERNAL "${name} Available")
  endif()
endfunction()

message(STATUS "Testing for SIMD extensions")

enable_language(C)

test_for_sse("ammintrin.h" SSE4A_AVAILABLE "SSE4A")
test_for_sse("nmmintrin.h" SSE4_2_AVAILABLE "SSE4.2")
test_for_sse("smmintrin.h" SSE4_1_AVAILABLE "SSE4.1")
test_for_sse("tmmintrin.h" SSSE3_AVAILABLE "SSSE3")
test_for_sse("pmmintrin.h" SSE3_AVAILABLE "SSE3")
test_for_sse("emmintrin.h" SSE2_AVAILABLE "SSE2")
test_for_sse("xmmintrin.h" SSE_AVAILABLE "SSE1")
test_for_sse("mmintrin.h" MMX_AVAILABLE "MMX")
test_for_sse("wmmintrin.h" AES_AVAILABLE "AES")
test_for_sse("immintrin.h" AVX_AVAILABLE "AVX")

file(REMOVE "${CMAKE_CURRENT_BINARY_DIR}/-.o")
