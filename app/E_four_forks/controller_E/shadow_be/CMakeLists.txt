# cmake_minimum_required(VERSION 3.14)

# project(general_controller LANGUAGES CXX)


# set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CONTROL_NAME "shadow_be")


file(GLOB src_files "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB header_files "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/build/sub_build")

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

set(CMAKE_INSTALL_RPATH "../../libraries")

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/../../../../simulation_world/webots/controllers)

set(CONTROLLER_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}_${CONTROL_NAME})

add_executable(${PROJECT_NAME}_${CONTROL_NAME}
  ${src_files}
  ${header_files}
  #${VN_INCLUDE}/sim_data_flow/ST_msg.pb.cc
)

set_target_properties(${PROJECT_NAME}_${CONTROL_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

include(GNUInstallDirs)

include_directories(${VN_INCLUDE})
include_directories(${VN_INCLUDE}/foxglove-vn)
include_directories(${ECAL_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${KEYBOARD_INCLUDE})
include_directories(/usr/local/webots/include/controller/cpp)

target_link_libraries(${PROJECT_NAME}_${CONTROL_NAME}
    ${LIBRARY_INSTALL_DIR}/liblogvn.so # TODO: FIXME: can not execute on ST server
    ${LIBRARY_INSTALL_DIR}/libgeometry.a
    ${WEBOTSLIB_SO}/libCppController.so
    ${LIBRARY_INSTALL_DIR}/libkeyboardform.a
    ${LIBRARY_INSTALL_DIR}/libfoxglove-vn.a
    ${LIBRARY_INSTALL_DIR}/libsim_data_flow.a
    ${LIBRARY_INSTALL_DIR}/libSingleton.a
    eCAL::core
    Qt${QT_VERSION_MAJOR}::Core 
    Qt${QT_VERSION_MAJOR}::Widgets
)

install(TARGETS ${PROJECT_NAME}_${CONTROL_NAME}
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

