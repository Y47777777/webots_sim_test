cmake_minimum_required(VERSION 3.14)

project(foxglove-vn LANGUAGES CXX)


#生成.h .cc文件
execute_process(COMMAND python "../build.py"
     RESULTS_VARIABLE  msg_all)
MESSAGE("Commands execute result: ${msg_all}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB CCFILES "foxglove-vn/*.cc")  
file(GLOB HHFILES "foxglove-vn/*.h")  

add_library(foxglove-vn STATIC
   ${HHFILES} ${CCFILES}
   make.sh
   make_clean.sh
)

target_compile_definitions(foxglove-vn PRIVATE PROTO_LIBRARY)

SET(CMAKE_BUILD_TYPE "Release")

SET(PROJECT_VERSION "1.0.0")

if(UNIX)
    set_target_properties(foxglove-vn
        PROPERTIES
        VERSION "${PROJECT_VERSION}"
        SOVERSION "${PROJECT_VERSION}")
endif()

#转移到make.sh
# SET(VN_INCLUDE
#     ~/workspace/vn_source/include)
# SET(LIB_SO
#     /usr/local/lib)
# SET(VNLIB_SO
#     ~/webots/SimRobot/libraies)

# set(CMAKE_INSTALL_PREFIX ~/webots/SimRobot)

# set_target_properties(${PROJECT_NAME}
#     PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/plugins/exe # 可执行文件
#     LIBRARY_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/libraries # 动态库
#     ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/libraries # 静态库
#     #INCLUDE_OUTPUT_DIRECTORY ${VN_INCLUDE}
# )


#install(DIRECTORY foxglove-vn  DESTINATION  /home/vision/workspace/vn_source/include  FILES_MATCHING PATTERN "*.h" PATTERN "*.proto")




