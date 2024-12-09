# cmake_minimum_required(VERSION 3.14)

project(FusionSim LANGUAGES CXX)


# set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CONTROL_NAME "voyar")

find_package(eCAL REQUIRED)
find_package(PCL REQUIRED)

file(GLOB src_files "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB header_files "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/build/sub_build")

set(VN_INCLUDE
    ${CMAKE_SOURCE_DIR}/../../include)
set(KEYBOARD_INCLUDE
    ${CMAKE_SOURCE_DIR}/../../base/keyboardform/include)

set(WEBOTSLIB_SO
    /usr/local/webots/lib/controller)

set(LIBRARY_INSTALL_DIR ${CMAKE_SOURCE_DIR}/../../../../sim_module_pkg/libraries)

set(BUILD_DIR ${CMAKE_SOURCE_DIR}/build)

set(CMAKE_INSTALL_RPATH "../../libraries")

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/../../../../sim_module_pkg/controllers)

set(CONTROLLER_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME})

add_executable(${PROJECT_NAME}
    main.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

include(GNUInstallDirs)

include_directories(${VN_INCLUDE})
include_directories(${VN_INCLUDE}/foxglove-vn)
include_directories(${ECAL_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${KEYBOARD_INCLUDE})
include_directories(/usr/local/webots/include/controller/cpp)

target_link_libraries(${PROJECT_NAME}
    ${LIBRARY_INSTALL_DIR}/liblogvn.so # TODO: FIXME: can not execute on ST server
    ${WEBOTSLIB_SO}/libCppController.so
    ${LIBRARY_INSTALL_DIR}/libfoxglove-vn.a
    ${LIBRARY_INSTALL_DIR}/libsim_data_flow.a
    eCAL::core
    ${PCL_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
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

