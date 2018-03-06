
file(GLOB ASN_RRLP_SOURCES "${CMAKE_SOURCE_DIR}/src/core/libs/supl2/asn-rrlp/*.c")
list(SORT ASN_RRLP_SOURCES)

file(GLOB ASN_SUPL2_SOURCES "${CMAKE_SOURCE_DIR}/src/core/libs/supl2/asn-supl2/*.c")
list(SORT ASN_SUPL2_SOURCES)



if(OPENSSL_FOUND)
    add_definitions( -DUSE_OPENSSL_FALLBACK=1 )
endif(OPENSSL_FOUND)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src/core/libs/supl2
    ${CMAKE_SOURCE_DIR}/src/core/libs/supl2/asn-rrlp
    ${CMAKE_SOURCE_DIR}/src/core/libs/supl2/asn-supl2
    ${GNUTLS_INCLUDE_DIR}
	)
	
set (SUPL2_SOURCES
	supl2.c
	)

if(CMAKE_C_COMPILER_ID MATCHES "Clang")
    if(CMAKE_BUILD_TYPE MATCHES "Release")
        set(MY_C_FLAGS "${MY_C_FLAGS} -Wno-parentheses-equality")         
    endif(CMAKE_BUILD_TYPE MATCHES "Release")
endif(CMAKE_C_COMPILER_ID MATCHES "Clang")	
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MY_C_FLAGS}")

#add_library (supl_library STATIC ${ASN_RRLP_SOURCES} ${ASN_SUPL_SOURCES} ${SUPL2_SOURCES})
add_library(supl2_library STATIC ${ASN_RRLP_SOURCES} ${ASN_SUPL2_SOURCES} ${SUPL2_SOURCES})
target_link_libraries (supl2_library ${GNUTLS_LIBRARIES} ${GNUTLS_OPENSSL_LIBRARY} gnss_system_parameters) 
set_target_properties(supl2_library PROPERTIES LINKER_LANGUAGE C)

