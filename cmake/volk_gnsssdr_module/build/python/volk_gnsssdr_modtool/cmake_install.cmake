# Install script for directory: /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/python/volk_gnsssdr_modtool

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xvolk_gnsssdrx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.10/dist-packages/volk_gnsssdr_modtool" TYPE FILE FILES
    "/home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/python/volk_gnsssdr_modtool/__init__.py"
    "/home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/python/volk_gnsssdr_modtool/cfg.py"
    "/home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/python/volk_gnsssdr_modtool/volk_gnsssdr_modtool_generate.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xvolk_gnsssdrx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.10/dist-packages/volk_gnsssdr_modtool" TYPE FILE FILES
    "/home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/python/volk_gnsssdr_modtool/__init__.pyc"
    "/home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/python/volk_gnsssdr_modtool/cfg.pyc"
    "/home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/python/volk_gnsssdr_modtool/volk_gnsssdr_modtool_generate.pyc"
    "/home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/python/volk_gnsssdr_modtool/__init__.pyo"
    "/home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/python/volk_gnsssdr_modtool/cfg.pyo"
    "/home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/python/volk_gnsssdr_modtool/volk_gnsssdr_modtool_generate.pyo"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xvolk_gnsssdrx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE PROGRAM RENAME "volk_gnsssdr_modtool" FILES "/home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/python/volk_gnsssdr_modtool/volk_gnsssdr_modtool.exe")
endif()

