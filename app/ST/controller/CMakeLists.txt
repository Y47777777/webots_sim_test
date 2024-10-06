cmake_minimum_required(VERSION 3.14)



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
