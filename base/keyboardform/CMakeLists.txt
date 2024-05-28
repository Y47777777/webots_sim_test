cmake_minimum_required(VERSION 3.14)

project(keyboardform LANGUAGES CXX
VERSION 1.0.0)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(NLOHMANN_JSON_INCLUDE /usr/include/nlohmann)

set(WEBOTSLIB_SO
    /usr/local/webots/lib/controller)
set(VN_INCLUDE
    ${CMAKE_SOURCE_DIR}/../../include)
set(LIBRARY_INSTALL_DIR 
    ${CMAKE_SOURCE_DIR}/../../../../simulation_world/webots/libraries)

file(GLOB src_files "*.c*")  
file(GLOB header_files "*.h*")

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

include_directories(
    ${NLOHMANN_JSON_INCLUDE}
    ${VN_INCLUDE}
    /usr/local/webots/include/controller/cpp
)

qt5_wrap_cpp(MOC_SOURCES ${header_files})

add_library(${PROJECT_NAME} SHARED
  ${src_files}
  ${MOC_SOURCES}
  keyboardform.ui
)

target_link_libraries(${PROJECT_NAME} 
    Qt${QT_VERSION_MAJOR}::Core 
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt5::Widgets
    ${WEBOTSLIB_SO}/libCppController.so
    ${LIBRARY_INSTALL_DIR}/liblogvn.so
)



if(UNIX)
    set_target_properties(${PROJECT_NAME}
        PROPERTIES
        VERSION "${PROJECT_VERSION}"
        SOVERSION "${PROJECT_VERSION}")
endif()

# install(TARGETS ${PROJECT_NAME}
# PERMISSIONS
# OWNER_READ
# OWNER_WRITE
# OWNER_EXECUTE
# GROUP_READ
# GROUP_EXECUTE
# WORLD_READ
# WORLD_EXECUTE
# DESTINATION ${LIBRARY_INSTALL_DIR}
# RUNTIME DESTINATION ${LIBRARY_INSTALL_DIR})

# install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}
#     DESTINATION ${CMAKE_SOURCE_DIR}/../../include
# )

