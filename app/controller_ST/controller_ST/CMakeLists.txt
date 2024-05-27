# cmake_minimum_required(VERSION 3.14)

# project(general_controller LANGUAGES CXX)

# set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(proto_file ${CMAKE_CURRENT_SOURCE_DIR}/proto_msg/point_cloud2.proto)
FILE(GLOB src_files ${CMAKE_SOURCE_DIR}/controller_${PROJECT_NAME}/*.c*)
FILE(GLOB header_files ${CMAKE_SOURCE_DIR}/controller_${PROJECT_NAME}/*.h*)
FILE(GLOB proto_header_file ${CMAKE_CURRENT_BINARY_DIR}/protobuf/*.h*)
FILE(GLOB proto_src_file ${CMAKE_CURRENT_BINARY_DIR}/protobuf/*.c*)
message("proto header file ${proto_header_file}")
message("proto header file ${proto_src_file}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/build/sub_build")
set(proto_file ${CMAKE_CURRENT_SOURCE_DIR}/proto_msg/point_cloud2.proto)


find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
find_package(QT5Widgets)
find_package(PCL REQUIRED)
find_package(eCAL REQUIRED)


set(VN_INCLUDE
    ${CMAKE_SOURCE_DIR}/../../include)
set(KEYBOARD_INCLUDE
    ${CMAKE_SOURCE_DIR}/../../base/keyboardform/include)

set(WEBOTSLIB_SO
    /usr/local/webots/lib/controller)

set(LIBRARY_INSTALL_DIR ${CMAKE_SOURCE_DIR}/../../../../simulation_world/webots/libraries)

set(BUILD_DIR ${CMAKE_SOURCE_DIR}/build)

set(CMAKE_INSTALL_RPATH "$ORIGIN/../../libraries")

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/../../../../simulation_world/webots/controllers)

set(CONTROLLER_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/controller_${PROJECT_NAME})

add_executable(controller_${PROJECT_NAME}
  ${proto_header_file}
  ${proto_src_file}
  ${src_files}
  ${header_files}
)

set_target_properties(controller_${PROJECT_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

PROTOBUF_TARGET_CPP(controller_${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/proto_msg/ ${proto_file})

include(GNUInstallDirs)

include_directories(${VN_INCLUDE})
include_directories(${ECAL_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${KEYBOARD_INCLUDE})
include_directories(/usr/local/webots/include/controller/cpp)

target_link_libraries(controller_${PROJECT_NAME}
    # TODO: add logvn
    ${WEBOTSLIB_SO}/libCppController.so
    ${LIBRARY_INSTALL_DIR}/libkeyboardform.so
    eCAL::core
    Qt${QT_VERSION_MAJOR}::Core 
    Qt${QT_VERSION_MAJOR}::Widgets
)

install(TARGETS controller_${PROJECT_NAME}
PERMISSIONS
OWNER_READ
OWNER_WRITE
OWNER_EXECUTE
GROUP_READ
GROUP_EXECUTE
WORLD_READ
WORLD_EXECUTE
DESTINATION ${CONTROLLER_INSTALL_DIR}
LIBRARY DESTINATION ${CONTROLLER_INSTALL_DIR}
RUNTIME DESTINATION ${CONTROLLER_INSTALL_DIR})

