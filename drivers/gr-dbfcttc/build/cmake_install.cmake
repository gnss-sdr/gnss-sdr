# Install script for directory: /home/usuario/sdr/gnss-sdr/gnss-sdr/drivers/gr-dbfcttc

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/dbfcttc" TYPE FILE FILES "/home/usuario/sdr/gnss-sdr/gnss-sdr/drivers/gr-dbfcttc/cmake/Modules/dbfcttcConfig.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/usuario/sdr/gnss-sdr/gnss-sdr/drivers/gr-dbfcttc/build/include/dbfcttc/cmake_install.cmake")
  INCLUDE("/home/usuario/sdr/gnss-sdr/gnss-sdr/drivers/gr-dbfcttc/build/lib/cmake_install.cmake")
  INCLUDE("/home/usuario/sdr/gnss-sdr/gnss-sdr/drivers/gr-dbfcttc/build/swig/cmake_install.cmake")
  INCLUDE("/home/usuario/sdr/gnss-sdr/gnss-sdr/drivers/gr-dbfcttc/build/python/cmake_install.cmake")
  INCLUDE("/home/usuario/sdr/gnss-sdr/gnss-sdr/drivers/gr-dbfcttc/build/grc/cmake_install.cmake")
  INCLUDE("/home/usuario/sdr/gnss-sdr/gnss-sdr/drivers/gr-dbfcttc/build/apps/cmake_install.cmake")
  INCLUDE("/home/usuario/sdr/gnss-sdr/gnss-sdr/drivers/gr-dbfcttc/build/docs/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/usuario/sdr/gnss-sdr/gnss-sdr/drivers/gr-dbfcttc/build/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/usuario/sdr/gnss-sdr/gnss-sdr/drivers/gr-dbfcttc/build/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
