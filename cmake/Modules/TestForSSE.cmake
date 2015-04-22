###############################################################################
# Test for availability of SSE
#
# - Anthony Arnold
###############################################################################


function (test_for_sse h_file result_var name)
  if (NOT DEFINED ${result_var})
    execute_process(COMMAND echo "#include <${h_file}>"
                    COMMAND ${CMAKE_CXX_COMPILER} ${CMAKE_CXX_COMPILER_ARG1} -c -x c++ -
		    RESULT_VARIABLE COMPILE_RESULT
		    OUTPUT_QUIET ERROR_QUIET)
    set(detected 0)
    if (COMPILE_RESULT EQUAL 0)
      message(STATUS "Detected ${name}")
      set(detected 1)
    endif(COMPILE_RESULT EQUAL 0)
    set(${result_var} ${detected} CACHE INTERNAL "${name} Available")
  endif (NOT DEFINED ${result_var})
endfunction(test_for_sse)

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
