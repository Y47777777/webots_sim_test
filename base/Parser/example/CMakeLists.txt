include_directories(${LOCAL_INCLUDE})
add_executable(simulator main.cpp)
target_link_libraries(simulator ${PROJECT_NAME})