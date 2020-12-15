cmake_minimum_required(VERSION 3.14)

project(PointCloudSeg LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#find_package(Qt5Core)
find_package(Qt5 COMPONENTS Core Concurrent REQUIRED)
find_package(PCL COMPONENTS io segmentation REQUIRED)
find_package(VTK)


add_executable(PointCloudSeg
  main.cpp
)

include(${VTK_USE_FILE})
target_link_libraries(PointCloudSeg Qt${QT_VERSION_MAJOR}::Core Qt5::Concurrent ${PCL_LIBRARIES} ${VTK_LIBRARIES})
