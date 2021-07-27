# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

# Original code from https://github.com/vector-of-bool/CMakeCM and modified
# by C. Fernandez. The original code is distributed under the OSI-approved
# BSD 3-Clause License. See https://cmake.org/licensing for details.

#[=======================================================================[.rst:

FindFILESYSTEM
##############

This module supports the C++17 standard library's filesystem utilities. Use the
:imp-target:`std::filesystem` imported target to

Options
*******

The ``COMPONENTS`` argument to this module supports the following values:

.. find-component:: Experimental
    :name: fs.Experimental

    Allows the module to find the "experimental" Filesystem TS version of the
    Filesystem library. This is the library that should be used with the
    ``std::experimental::filesystem`` namespace.

.. find-component:: Final
    :name: fs.Final

    Finds the final C++17 standard version of the filesystem library.

If no components are provided, behaves as if the
:find-component:`fs.Final` component was specified.

If both :find-component:`fs.Experimental` and :find-component:`fs.Final` are
provided, first looks for ``Final``, and falls back to ``Experimental`` in case
of failure. If ``Final`` is found, :imp-target:`std::filesystem` and all
:ref:`variables <fs.variables>` will refer to the ``Final`` version.


Imported Targets
****************

.. imp-target:: std::filesystem

    The ``std::filesystem`` imported target is defined when any requested
    version of the C++ filesystem library has been found, whether it is
    *Experimental* or *Final*.

    If no version of the filesystem library is available, this target will not
    be defined.

    .. note::
        This target has ``cxx_std_17`` as an ``INTERFACE``
        :ref:`compile language standard feature <req-lang-standards>`. Linking
        to this target will automatically enable C++17 if no later standard
        version is already required on the linking target.


.. _fs.variables:

Variables
*********

.. variable:: CXX_FILESYSTEM_IS_EXPERIMENTAL

    Set to ``TRUE`` when the :find-component:`fs.Experimental` version of C++
    filesystem library was found, otherwise ``FALSE``.

.. variable:: CXX_FILESYSTEM_HAVE_FS

    Set to ``TRUE`` when a filesystem header was found.

.. variable:: CXX_FILESYSTEM_HEADER

    Set to either ``filesystem`` or ``experimental/filesystem`` depending on
    whether :find-component:`fs.Final` or :find-component:`fs.Experimental` was
    found.

.. variable:: CXX_FILESYSTEM_NAMESPACE

    Set to either ``std::filesystem`` or ``std::experimental::filesystem``
    depending on whether :find-component:`fs.Final` or
    :find-component:`fs.Experimental` was found.


Examples
********

Using `find_package(FILESYSTEM)` with no component arguments:

.. code-block:: cmake

    find_package(FILESYSTEM REQUIRED)

    add_executable(my-program main.cpp)
    target_link_libraries(my-program PRIVATE std::filesystem)


#]=======================================================================]


if(TARGET std::filesystem)
    # This module has already been processed. Don't do it again.
    return()
endif()

include(CMakePushCheckState)
include(CheckIncludeFileCXX)
include(CheckCXXSourceCompiles)

cmake_push_check_state()

if(FILESYSTEM_FIND_QUIETLY)
    set(CMAKE_REQUIRED_QUIET ${FILESYSTEM_FIND_QUIETLY})
endif()

# All of our tests require C++17 or later
set(OLD_CMAKE_CXX_STANDARD ${CMAKE_CXX_STANDARD})
set(CMAKE_CXX_STANDARD 17)
if((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER "8.0.0"))
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "8.99")
        set(UNDEFINED_BEHAVIOR_WITHOUT_LINKING TRUE)
    endif()
    set(CMAKE_REQUIRED_FLAGS "-std=c++17")
endif()
if((CMAKE_CXX_COMPILER_ID STREQUAL "Clang") AND NOT (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "8.99"))
    set(CMAKE_REQUIRED_FLAGS "-std=c++17")
endif()
if((CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang") AND NOT (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "11"))
    set(CMAKE_REQUIRED_FLAGS "-std=c++17")
endif()
if(MSVC AND NOT (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "18"))
    set(CMAKE_REQUIRED_FLAGS "/std:c++17")
endif()

# Normalize and check the component list we were given
set(want_components ${FILESYSTEM_FIND_COMPONENTS})
if(FILESYSTEM_FIND_COMPONENTS STREQUAL "")
    set(want_components Final)
endif()

# Warn on any unrecognized components
set(extra_components ${want_components})
list(REMOVE_ITEM extra_components Final Experimental)
foreach(component IN LISTS extra_components)
    message(WARNING "Extraneous find_package component for FILESYSTEM: ${component}")
endforeach()

# Detect which of Experimental and Final we should look for
set(find_experimental TRUE)
set(find_final TRUE)
if(NOT "Final" IN_LIST want_components)
    set(find_final FALSE)
endif()
if(NOT "Experimental" IN_LIST want_components)
    set(find_experimental FALSE)
endif()

if(find_final)
    check_include_file_cxx("filesystem" _CXX_FILESYSTEM_HAVE_HEADER)
    mark_as_advanced(_CXX_FILESYSTEM_HAVE_HEADER)
    if(_CXX_FILESYSTEM_HAVE_HEADER)
        # We found the non-experimental header. Don't bother looking for the
        # experimental one.
        set(find_experimental FALSE)
    endif()
else()
    set(_CXX_FILESYSTEM_HAVE_HEADER FALSE)
endif()

if(find_experimental)
    check_include_file_cxx("experimental/filesystem" _CXX_FILESYSTEM_HAVE_EXPERIMENTAL_HEADER)
    mark_as_advanced(_CXX_FILESYSTEM_HAVE_EXPERIMENTAL_HEADER)
else()
    set(_CXX_FILESYSTEM_HAVE_EXPERIMENTAL_HEADER FALSE)
endif()

if(_CXX_FILESYSTEM_HAVE_HEADER)
    set(_have_fs TRUE)
    set(_fs_header filesystem)
    set(_fs_namespace std::filesystem)
elseif(_CXX_FILESYSTEM_HAVE_EXPERIMENTAL_HEADER)
    set(_have_fs TRUE)
    set(_fs_header experimental/filesystem)
    set(_fs_namespace std::experimental::filesystem)
else()
    set(_have_fs FALSE)
endif()

set(CXX_FILESYSTEM_HAVE_FS ${_have_fs} CACHE BOOL "TRUE if we have the C++ filesystem headers")
set(CXX_FILESYSTEM_HEADER ${_fs_header} CACHE STRING "The header that should be included to obtain the filesystem APIs")
set(CXX_FILESYSTEM_NAMESPACE ${_fs_namespace} CACHE STRING "The C++ namespace that contains the filesystem APIs")

set(_found FALSE)

if(CXX_FILESYSTEM_HAVE_FS)
    # We have some filesystem library available. Do link checks
    string(CONFIGURE [[
        #include <@CXX_FILESYSTEM_HEADER@>

        int main() {
            auto cwd = @CXX_FILESYSTEM_NAMESPACE@::current_path();
            return static_cast<int>(cwd.string().size());
        }
    ]] code @ONLY)

    # Try to compile a simple filesystem program without any linker flags
    if(NOT UNDEFINED_BEHAVIOR_WITHOUT_LINKING)
        check_cxx_source_compiles("${code}" CXX_FILESYSTEM_NO_LINK_NEEDED)
        set(can_link ${CXX_FILESYSTEM_NO_LINK_NEEDED})
    endif()

    if(NOT CXX_FILESYSTEM_NO_LINK_NEEDED)
        set(prev_libraries ${CMAKE_REQUIRED_LIBRARIES})
        set(CMAKE_REQUIRED_FLAGS "-std=c++17")
        # Add the libstdc++ flag
        set(CMAKE_REQUIRED_LIBRARIES ${prev_libraries} -lstdc++fs)
        check_cxx_source_compiles("${code}" CXX_FILESYSTEM_STDCPPFS_NEEDED)
        set(can_link ${CXX_FILESYSTEM_STDCPPFS_NEEDED})
        if(NOT CXX_FILESYSTEM_STDCPPFS_NEEDED)
            # Try the libc++ flag
            set(CMAKE_REQUIRED_LIBRARIES ${prev_libraries} -lc++fs)
            check_cxx_source_compiles("${code}" CXX_FILESYSTEM_CPPFS_NEEDED)
            set(can_link ${CXX_FILESYSTEM_CPPFS_NEEDED})
        endif()
    endif()

    if(can_link)
        if(CMAKE_VERSION VERSION_LESS 3.12)
            add_library(std::filesystem INTERFACE IMPORTED GLOBAL)
        else()
            add_library(std::filesystem INTERFACE IMPORTED)
            target_compile_features(std::filesystem INTERFACE cxx_std_17)
        endif()
        set(_found TRUE)

        if(CXX_FILESYSTEM_NO_LINK_NEEDED)
            # Nothing to add...
        elseif(CXX_FILESYSTEM_STDCPPFS_NEEDED)
            target_link_libraries(std::filesystem INTERFACE -lstdc++fs)
        elseif(CXX_FILESYSTEM_CPPFS_NEEDED)
            target_link_libraries(std::filesystem INTERFACE -lc++fs)
        endif()
    endif()
endif()

if(NOT ${_found})
    set(CMAKE_CXX_STANDARD ${OLD_CMAKE_CXX_STANDARD})
endif()

cmake_pop_check_state()

set(FILESYSTEM_FOUND ${_found})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FILESYSTEM DEFAULT_MSG FILESYSTEM_FOUND)

if(FILESYSTEM_FIND_REQUIRED AND NOT FILESYSTEM_FOUND)
    message(FATAL_ERROR "Cannot compile a simple program using std::filesystem")
endif()
