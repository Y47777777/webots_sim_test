cmake_minimum_required(VERSION 3.14)

project(general_controller LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

add_executable(general_controller
  main.cpp
  controllerthrd.h controllerthrd.cpp
  keyboardform.h keyboardform.cpp keyboardform.ui
  proto_to_ros.h
  make.sh make_clean.sh
)
target_link_libraries(general_controller Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Widgets)


include(GNUInstallDirs)
install(TARGETS general_controller
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)


SET(GOOGLE_INCLUDE
    /usr/include/google)
SET(CARTO_INCLUDE
    /home/vision/cartographer_ws/install_isolated/include)

SET(ROS_INCLUDE
    /opt/ros/noetic/include)

SET(LIB_SO
    /usr/local/lib)
SET(VNLIB_SO
    /home/vision/webots/SimRobot/libraries)
SET(SYSTEM_SO
    /usr/lib/x86_64-linux-gnu/)

SET(SYSLIB_SO
    /usr/lib)
SET(WEBOTSLIB_SO
    /usr/local/webots/lib/controller)
SET(CARTO_SO
    /home/vision/cartographer_ws/install_isolated/lib)

SET(ROS_LIB
    /opt/ros/noetic/lib)

include_directories(${VN_INCLUDE})
# include_directories(${GOOGLE_INCLUDE})
include_directories(/usr/include/boost)
include_directories(/usr/local/webots/include/controller/cpp)
include_directories(/usr/include/eigen3)
include_directories(/usr/include/pcl-1.10)
include_directories(${ROS_INCLUDE})
include_directories(${CARTO_INCLUDE})

target_link_libraries(general_controller ${LIB_SO}/libpaho-mqttpp3.so  ${LIB_SO}/libpaho-mqtt3cs.so  ${LIB_SO}/libpaho-mqtt3c.so  ${LIB_SO}/libpaho-mqtt3as.so ${LIB_SO}/libpaho-mqtt3a.so)
target_link_libraries(general_controller ${LIB_SO}/libecal_core.so ${LIB_SO}/libecal_proto.a)
target_link_libraries(general_controller ${VNLIB_SO}/liblogvn.so ${VNLIB_SO}/libfoxglove-vn.a)
target_link_libraries(general_controller ${VNLIB_SO}/libactionservice.so)
target_link_libraries(general_controller ${SYSTEM_SO}/libpthread.so)
target_link_libraries(general_controller ${LIB_SO}/libpcl_io.so ${LIB_SO}/libpcl_common.so ${LIB_SO}/libprotobuf.a)
target_link_libraries(general_controller ${WEBOTSLIB_SO}/libCppController.so)
target_link_libraries(general_controller ${ROS_LIB}/libroscpp.so  ${ROS_LIB}/libroslib.so ${ROS_LIB}/librosconsole.so ${ROS_LIB}/libroscpp_serialization.so ${ROS_LIB}/librostime.so)


# set(CMAKE_INSTALL_PREFIX /home/vision/webots/SimRobot/controllers)

# install(TARGETS ${PROJECT_NAME}
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION exe
# )


# set_target_properties(${PROJECT_NAME}
#     PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/general_controller # 可执行文件
# )

