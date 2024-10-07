cmake_minimum_required(VERSION 3.5)

project(paralleTest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(${PROJECT_NAME}_Client parallelClientTest.cpp)
add_executable(${PROJECT_NAME}_Server parallelServerTest.cpp)

include(GNUInstallDirs)



SET(GOOGLE_INCLUDE
    /usr/include/google)

SET(LIB_SO
    /usr/local/lib)
SET(SYSTEM_SO
    /usr/lib/x86_64-linux-gnu/)

SET(VN_INCLUDE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../include)
SET(VNLIB_SO
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../sim_mode_pkg/libraries/)



include_directories(${VN_INCLUDE})
# include_directories(${GOOGLE_INCLUDE})
include_directories(/usr/include/boost)

target_link_libraries(${PROJECT_NAME}_Client ${LIB_SO}/libpaho-mqttpp3.so  ${LIB_SO}/libpaho-mqtt3cs.so  ${LIB_SO}/libpaho-mqtt3c.so  ${LIB_SO}/libpaho-mqtt3as.so ${LIB_SO}/libpaho-mqtt3a.so)
target_link_libraries(${PROJECT_NAME}_Client ${VNLIB_SO}/liblogvn.so)
target_link_libraries(${PROJECT_NAME}_Client ${VNLIB_SO}/libactionservice.so)
target_link_libraries(${PROJECT_NAME}_Client ${SYSTEM_SO}/libpthread.so)

target_link_libraries(${PROJECT_NAME}_Server ${LIB_SO}/libpaho-mqttpp3.so  ${LIB_SO}/libpaho-mqtt3cs.so  ${LIB_SO}/libpaho-mqtt3c.so  ${LIB_SO}/libpaho-mqtt3as.so ${LIB_SO}/libpaho-mqtt3a.so)
target_link_libraries(${PROJECT_NAME}_Server ${VNLIB_SO}/liblogvn.so)
target_link_libraries(${PROJECT_NAME}_Server ${VNLIB_SO}/libactionservice.so)
target_link_libraries(${PROJECT_NAME}_Server ${SYSTEM_SO}/libpthread.so)

# set(CMAKE_INSTALL_PREFIX /home/vision/robot)

# install(TARGETS ${PROJECT_NAME}
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION exe
# )


# set_target_properties(${PROJECT_NAME}
#     PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/exe # 可执行文件
#     LIBRARY_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib # 动态库
#     ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib # 静态库
# )
