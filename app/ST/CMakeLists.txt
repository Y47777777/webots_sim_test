cmake_minimum_required(VERSION 3.14)

project(ST LANGUAGES CXX)

# set(CMAKE_AUTOUIC ON)
# set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTORCC ON)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(controller_ST controller_ST/sub_build)
add_subdirectory(svc_model_ST svc_model_ST/sub_build)