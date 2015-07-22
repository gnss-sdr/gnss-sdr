

find_path(USB_INCLUDE_DIR NAMES usb.h
          PATHS /usr/include
                /usr/local/include
)

if(NOT USB_INCLUDE_DIR_FOUND)
   message(STATUS "libusb has not been found.")
   message(STATUS "You can install it by 'sudo apt-get install libusb-dev' ")
   message(FATAL_ERROR "libusb is required for building gr-gn3s")
endif(NOT USB_INCLUDE_DIR_FOUND)


INCLUDE(FindPkgConfig)

if(NOT LIBUSB_FOUND)
   pkg_check_modules (LIBUSB_PKG libusb-1.0)
   find_path(LIBUSB_INCLUDE_DIR NAMES libusb.h
             PATHS ${LIBUSB_PKG_INCLUDE_DIRS}
                   /usr/include/libusb-1.0
                   /usr/include
                   /usr/local/include
                   /opt/local/include/libusb-1.0
   )

   find_library(LIBUSB_LIBRARIES NAMES usb-1.0
                PATHS ${LIBUSB_PKG_LIBRARY_DIRS}
                      /usr/lib
                      /usr/local/lib
                      /opt/local/lib
   )

   if(LIBUSB_INCLUDE_DIR AND LIBUSB_LIBRARIES)
      set(LIBUSB_FOUND TRUE CACHE INTERNAL "libusb-1.0 found")
      message(STATUS "Found libusb-1.0: ${LIBUSB_INCLUDE_DIR}, ${LIBUSB_LIBRARIES}")
   else(LIBUSB_INCLUDE_DIR AND LIBUSB_LIBRARIES)
      set(LIBUSB_FOUND FALSE CACHE INTERNAL "libusb-1.0 found")
      message(STATUS "libusb-1.0 not found.")
      message(STATUS "You can install it by 'sudo apt-get install libusb-1.0-0-dev'")
   endif(LIBUSB_INCLUDE_DIR AND LIBUSB_LIBRARIES)

   mark_as_advanced(LIBUSB_INCLUDE_DIR LIBUSB_LIBRARIES)

endif(NOT LIBUSB_FOUND)
