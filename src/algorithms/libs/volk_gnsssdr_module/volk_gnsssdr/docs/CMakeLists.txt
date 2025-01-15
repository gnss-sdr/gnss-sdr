# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2022 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

message(STATUS "Doxygen in docs |${DOXYGEN_FOUND}|")
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
    ${CMAKE_BINARY_DIR}/Doxyfile
@ONLY)

add_custom_target(volk_gnsssdr_doc
    ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating documentation with Doxygen" VERBATIM
)
