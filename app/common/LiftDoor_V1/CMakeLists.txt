cmake_minimum_required(VERSION 3.16)

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY /home/visonnav/workspace/simulation_world/webots/controllers/LiftDoor_V1)

set(CONTROL_NAME LiftDoor_V1)

set(WBT_DIR /usr/local/webots)

set(CMAKE_INSTALL_RPATH "../../libraries")

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/../../../../simulation_world/webots/controllers)

set(CONTROLLER_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/${CONTROL_NAME})


include_directories(
    ${WBT_DIR}/include/controller/cpp/
)

add_executable(LiftDoor_V1 LiftDoor.cc)
target_link_libraries(LiftDoor_V1 ${WBT_DIR}/lib/controller/libCppController.so)

install(TARGETS ${CONTROL_NAME}
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