cmake_minimum_required(VERSION 3.14)

project(actionservice LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(actionservice SHARED
  actionservice_global.h
  actionservice.cpp
  actionservice.h
  responsequeue.h
  make.sh
  make_clean.sh


)

target_compile_definitions(actionservice PRIVATE ACTIONSERVICE_LIBRARY)


SET(SYS_INCLUDE
    /usr/local/include)
SET(GOOGLE_INCLUDE
    /usr/include/google)
SET(LIB_SO
    /usr/local/lib)

SET(VN_INCLUDE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
SET(VNLIB_SO
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../simulation_world/webots/libraries/)


include_directories(${VN_INCLUDE})
# include_directories(${GOOGLE_INCLUDE})
include_directories(/usr/include/boost)

target_link_libraries(actionservice ${LIB_SO}/libpaho-mqttpp3.so  ${LIB_SO}/libpaho-mqtt3cs.so  ${LIB_SO}/libpaho-mqtt3c.so  ${LIB_SO}/libpaho-mqtt3as.so ${LIB_SO}/libpaho-mqtt3a.so)
target_link_libraries(actionservice ${VNLIB_SO}/liblogvn.so)



SET(CMAKE_BUILD_TYPE "Release")

SET(PROJECT_VERSION "1.0.0")

if(UNIX)
    set_target_properties(actionservice
        PROPERTIES
        VERSION "${PROJECT_VERSION}"
        SOVERSION "${PROJECT_VERSION}")
endif()

#转移到make.sh
# set(CMAKE_INSTALL_PREFIX /home/vision/robot)

# install(TARGETS ${PROJECT_NAME}
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION exe
# )

# install(FILES *.h DESTINATION ${VN_INCLUDE})

# set_target_properties(${PROJECT_NAME}
#     PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/exe # 可执行文件俄
#     LIBRARY_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib # 动态库
#     ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib # 静态库
# )


# # 调用execute_process函数来执行Shell命令并获取输出结果
# execute_process(COMMAND mkdir "/home/vision/workspace/vn_source/include/actionservice"
#     COMMAND cp actionservice.h responsequeue.h actionservice_global.h /home/vision/workspace/vn_source/include/actionservice WORKING_DIRECTORY "/home/vision/workspace/vn_source/base/actionservice"
#     RESULTS_VARIABLE  msg_all)
# MESSAGE("Commands execute result: ${msg_all}")


