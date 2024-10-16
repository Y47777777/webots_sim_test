cmake_minimum_required(VERSION 3.14)

project(R14 LANGUAGES CXX)

# set(CMAKE_AUTOUIC ON)
# set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTORCC ON)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB children RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*)
foreach(child ${children})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${child})
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${child}/CMakeLists.txt")
            add_subdirectory(${child} ${child}/sub_build)
        else()
            message(STATUS "Skipping directory ${child} as it does not contain a CMakeLists.txt file")
        endif()
    endif()
endforeach()


# add_subdirectory(controller_P controller_P/sub_build)
# add_subdirectory(svc_model_P svc_model_P/sub_build)