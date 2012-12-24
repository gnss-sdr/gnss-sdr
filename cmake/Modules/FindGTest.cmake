##############################################################################
00002 # @file  FindGTest.cmake
00003 # @brief Find Google Test package.
00004 #
00005 # @par Input variables:
00006 # <table border="0">
00007 #   <tr>
00008 #     @tp @b GTest_DIR @endtp
00009 #     <td>The Google Test package files are searched under the specified
00010 #         root directory. If they are not found there, the default search
00011 #         paths are considered.
00012 #         This variable can also be set as environment variable.</td>
00013 #   </tr>
00014 #   <tr>
00015 #     @tp @b GTEST_DIR @endtp
00016 #     <td>Alternative environment variable for @p GTest_DIR.</td>
00017 #   </tr>
00018 #   <tr>
00019 #     @tp @b GTest_SHARED_LIBRARIES @endtp
00020 #     <td>Forces this module to search for shared libraries.
00021 #         Otherwise, static libraries are preferred.</td>
00022 #   </tr>
00023 # </table>
00024 #
00025 # @par Output variables:
00026 # <table border="0">
00027 #   <tr>
00028 #     @tp @b GTest_FOUND @endtp
00029 #     <td>Whether the package was found and the following CMake variables are valid.</td>
00030 #   </tr>
00031 #   <tr>
00032 #     @tp @b GTest_INCLUDE_DIR @endtp
00033 #     <td>Package include directories.</td>
00034 #   </tr>
00035 #   <tr>
00036 #     @tp @b GTest_INCLUDES @endtp
00037 #     <td>Include directories including prerequisite libraries.</td>
00038 #   </tr>
00039 #   <tr>
00040 #     @tp @b GTest_LIBRARY @endtp
00041 #     <td>Path of @c gtest library.</td>
00042 #   </tr>
00043 #   <tr>
00044 #     @tp @b GTest_main_LIBRARY @endtp
00045 #     <td>Path of @c gtest_main library (optional).</td>
00046 #   </tr>
00047 #   <tr>
00048 #     @tp @b GTest_LIBRARIES @endtp
00049 #     <td>Package libraries and prerequisite libraries.</td>
00050 #   </tr>
00051 # </table>
00052 #
00053 # Copyright (c) 2011, 2012 University of Pennsylvania. All rights reserved.<br />
00054 # See http://www.rad.upenn.edu/sbia/software/license.html or COPYING file.
00055 #
00056 # Contact: SBIA Group <sbia-software at uphs.upenn.edu>
00057 #
00058 # @ingroup CMakeFindModules
00059 ##############################################################################
00060 
00061 # ----------------------------------------------------------------------------
00062 # initialize search
00063 if (NOT GTest_DIR)
00064   if ($ENV{GTEST_DIR})
00065     set (GTest_DIR "$ENV{GTEST_DIR}" CACHE PATH "Installation prefix for Google Test")
00066   else ()
00067     set (GTest_DIR "$ENV{GTest_DIR}" CACHE PATH "Installation prefix for Google Test")
00068   endif ()
00069 endif ()
00070 
00071 set (GTest_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES})
00072 
00073 if (GTest_SHARED_LIBRARIES)
00074   if (WIN32)
00075     set (CMAKE_FIND_LIBRARY_SUFFIXES .dll)
00076   else ()
00077     set (CMAKE_FIND_LIBRARY_SUFFIXES .so)
00078   endif()
00079 else ()
00080   if (WIN32)
00081     set (CMAKE_FIND_LIBRARY_SUFFIXES .lib)
00082   else ()
00083     set (CMAKE_FIND_LIBRARY_SUFFIXES .a)
00084   endif()
00085 endif ()
00086 
00087 # ----------------------------------------------------------------------------
00088 # find paths/files
00089 if (GTest_DIR)
00090 
00091   find_path (
00092     GTest_INCLUDE_DIR
00093       NAMES         gtest.h
00094       HINTS         "${GTest_DIR}"
00095       PATH_SUFFIXES "include/gtest"
00096       DOC           "Include directory for Google Test."
00097       NO_DEFAULT_PATH
00098   )
00099 
00100   find_library (
00101     GTest_LIBRARY
00102       NAMES         gtest
00103       HINTS         "${GTest_DIR}"
00104       PATH_SUFFIXES "lib"
00105       DOC           "Link library for Google Test (gtest)."
00106       NO_DEFAULT_PATH
00107   )
00108 
00109   find_library (
00110     GTest_main_LIBRARY
00111       NAMES         gtest_main
00112       HINTS         "${GTest_DIR}"
00113       PATH_SUFFIXES "lib"
00114       DOC           "Link library for Google Test's automatic main () definition (gtest_main)."
00115       NO_DEFAULT_PATH
00116   )
00117 
00118 else ()
00119 
00120   find_path (
00121     GTest_INCLUDE_DIR
00122       NAMES gtest.h
00123       HINTS ENV C_INCLUDE_PATH ENV CXX_INCLUDE_PATH
00124       DOC   "Include directory for Google Test."
00125   )
00126 
00127   find_library (
00128     GTest_LIBRARY
00129       NAMES gtest
00130       HINTS ENV LD_LIBRARY_PATH
00131       DOC   "Link library for Google Test (gtest)."
00132   )
00133 
00134   find_library (
00135     GTest_main_LIBRARY
00136       NAMES gtest_main
00137       HINTS ENV LD_LIBRARY_PATH
00138       DOC   "Link library for Google Test's automatic main () definition (gtest_main)."
00139   )
00140 
00141 endif ()
00142 
00143 mark_as_advanced (GTest_INCLUDE_DIR)
00144 mark_as_advanced (GTest_LIBRARY)
00145 mark_as_advanced (GTest_main_LIBRARY)
00146 
00147 # ----------------------------------------------------------------------------
00148 # add prerequisites
00149 set (GTest_INCLUDES "${GTest_INCLUDE_DIR}")
00150 
00151 set (GTest_LIBRARIES)
00152 if (GTest_LIBRARY)
00153   list (APPEND GTest_LIBRARIES "${GTest_LIBRARY}")
00154 endif ()
00155 if (GTest_main_LIBRARY)
00156   list (APPEND GTest_LIBRARIES "${GTest_main_LIBRARY}")
00157 endif ()
00158 
00159 # ----------------------------------------------------------------------------
00160 # reset CMake variables
00161 set (CMAKE_FIND_LIBRARY_SUFFIXES ${GTest_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES})
00162 
00163 # ----------------------------------------------------------------------------
00164 # aliases / backwards compatibility
00165 set (GTest_INCLUDE_DIRS "${GTest_INCLUDES}")
00166 
00167 # ----------------------------------------------------------------------------
00168 # handle the QUIETLY and REQUIRED arguments and set *_FOUND to TRUE
00169 # if all listed variables are found or TRUE
00170 include (FindPackageHandleStandardArgs)
00171 
00172 find_package_handle_standard_args (
00173   GTest
00174   REQUIRED_VARS
00175     GTest_INCLUDE_DIR
00176     GTest_LIBRARY
00177 )
00178 
00179 set (GTest_FOUND "${GTEST_FOUND}")
00180 
00181 # ----------------------------------------------------------------------------
00182 # set GTest_DIR
00183 if (NOT GTest_DIR AND GTest_FOUND)
00184   string (REGEX REPLACE "include(/gtest)?/?" "" GTest_PREFIX "${GTest_INCLUDE_DIR}")
00185   set (GTest_DIR "${GTest_PREFIX}" CACHE PATH "Installation prefix for GTest." FORCE)
00186   unset (GTest_PREFIX)
00187 endif ()
