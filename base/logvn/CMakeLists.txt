cmake_minimum_required(VERSION 3.14)

project(logvn LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(logvn SHARED
  logvn.cpp
  logvn.h
  make.sh
  make_clean.sh
)

target_compile_definitions(logvn PRIVATE LOGVN_LIBRARY)




SET(LIB_SO
    /usr/local/lib)

SET(VN_INCLUDE
    ${CMAKE_CURRENT_SOURCE_DIR}../../include)


target_link_libraries(logvn ${LIB_SO}/liblog4cplus.so)


SET(CMAKE_BUILD_TYPE "Release")

SET(PROJECT_VERSION "1.0.0")

if(UNIX)
    set_target_properties(logvn
        PROPERTIES
        VERSION "${PROJECT_VERSION}"
        SOVERSION "${PROJECT_VERSION}")
endif()



#转移到make.sh
# set(CMAKE_INSTALL_PREFIX ~/webots/SimRobot)

# set_target_properties(${PROJECT_NAME}
#     PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/plugins/exe # 可执行文件
#     LIBRARY_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/libraries # 动态库
#     ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/libraries # 静态库
#     INCLUDE_OUTPUT_DIRECTORY ${VN_INCLUDE}
# )

# install(FILES logvn.h DESTINATION ${VN_INCLUDE})

# add_custom_target(COMMAND "cp" "*.h"  "~/workspace/vn_source/include" WORKING_DIRECTORY ".")




