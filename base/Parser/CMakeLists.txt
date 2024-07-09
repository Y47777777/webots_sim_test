cmake_minimum_required(VERSION 3.14)

project(Parser LANGUAGES CXX
VERSION 1.2.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(LIBRARY_INSTALL_DIR 
    ${CMAKE_SOURCE_DIR}/../../../../simulation_world/webots/libraries)

file(GLOB src_files "src/*.c*")  
file(GLOB header_files "src/*.h*")


SET(USER_INCLUDE /usr/include)
option(ENABLE_EXAMPLE "Build Example" OFF)

include_directories(${USER_INCLUDE} ${CMAKE_SOURCE_DIR}/include)

message(${src_files})

add_library(${PROJECT_NAME} SHARED
  ${src_files}
)

# target_compile_definitions(actionservice PRIVATE ACTIONSERVICE_LIBRARY)
target_compile_definitions(${PROJECT_NAME} PRIVATE ACTIONSERVICE_LIBRARY)

# include_directories(${VN_INCLUDE})
# # include_directories(${GOOGLE_INCLUDE})
# include_directories(/usr/include/boost)

# target_link_libraries(actionservice ${LIB_SO}/libpaho-mqttpp3.so  ${LIB_SO}/libpaho-mqtt3cs.so  ${LIB_SO}/libpaho-mqtt3c.so  ${LIB_SO}/libpaho-mqtt3as.so ${LIB_SO}/libpaho-mqtt3a.so)
# target_link_libraries(actionservice ${VNLIB_SO}/liblogvn.so)


target_link_libraries(${PROJECT_NAME}
    libtinyxml2.so
)

if(ENABLE_EXAMPLE)
message("build example...")
add_subdirectory(example)
endif()

if(UNIX)
    set_target_properties(${PROJECT_NAME}
        PROPERTIES
        VERSION "${PROJECT_VERSION}"
        SOVERSION "${PROJECT_VERSION}")
endif()


