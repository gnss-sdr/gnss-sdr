# - Try to find OpenBLAS library (not headers!)
#
# The following environment variable is optionally searched 
# OPENBLAS_HOME: Base directory where all OpenBlas components are found

SET(OPEN_BLAS_SEARCH_PATHS  /lib/ 
                            /lib64/  
                            /usr/lib 
                            /usr/lib64 
                            /usr/local/lib 
                            /usr/local/lib64 
                            /opt/OpenBLAS/lib 
                            /opt/local/lib 
                            /usr/lib/openblas-base 
                            $ENV{OPENBLAS_HOME}/lib 
                            )
                            
FIND_LIBRARY(OPENBLAS NAMES openblas PATHS ${OPEN_BLAS_SEARCH_PATHS})

IF (OPENBLAS)
    SET(OPENBLAS_FOUND ON)
    MESSAGE(STATUS "Found OpenBLAS")
ENDIF (OPENBLAS)

MARK_AS_ADVANCED(OPENBLAS)
