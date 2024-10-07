cmake_minimum_required(VERSION 3.14)

project(sim_data_flow LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB CCFILES "build/*.cc")  
file(GLOB HHFILES "build/*.h")  
MESSAGE("CCFILES :${CCFILES}")
MESSAGE("HHFILES: ${HHFILES}")


set(VN_INCLUDE
    ${CMAKE_SOURCE_DIR}/../../include/foxglove-vn/)

include_directories(${CMAKE_SOURCE_DIR}/build/)
include_directories(${VN_INCLUDE})
MESSAGE("${VN_INCLUDE}")



add_library(sim_data_flow STATIC
   ${HHFILES} ${CCFILES}
   make.sh
   make_clean.sh
)

set(LIBRARY_INSTALL_DIR ../../../../sim_mode_pkg/libraries)
target_link_libraries(sim_data_flow ${LIBRARY_INSTALL_DIR}/libfoxglove-vn.a)
target_compile_definitions(sim_data_flow PRIVATE PROTO_LIBRARY)

set_target_properties(sim_data_flow PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

SET(CMAKE_BUILD_TYPE "Release")

SET(PROJECT_VERSION "1.0.0")

if(UNIX)
    set_target_properties(sim_data_flow
        PROPERTIES
        VERSION "${PROJECT_VERSION}"
        SOVERSION "${PROJECT_VERSION}")
endif()



