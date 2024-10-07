cmake_minimum_required(VERSION 3.14)

project(Singleton LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB CCFILES "src/*.cpp")  

MESSAGE("CCFILES :${CCFILES}")

include_directories(include)
include_directories(/usr/local/webots/include/controller/cpp)

add_library(${PROJECT_NAME} STATIC
   ${CCFILES}
)
set(WEBOTSLIB_SO
    /usr/local/webots/lib/controller)
set(LIBRARY_INSTALL_DIR ../../../../sim_module_pkg/libraries)
target_link_libraries(${PROJECT_NAME} ${WEBOTSLIB_SO}/libCppController.so)

#target_compile_definitions(sim_data_flow PRIVATE PROTO_LIBRARY)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

SET(CMAKE_BUILD_TYPE "Release")

#SET(PROJECT_VERSION "1.0.0")

if(UNIX)
    set_target_properties(${PROJECT_NAME}
        PROPERTIES
        VERSION "${PROJECT_VERSION}"
        SOVERSION "${PROJECT_VERSION}")
endif()



