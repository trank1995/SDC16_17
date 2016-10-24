cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})


include_directories(FADBAD++)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# link_directories(${OpenCV_LIBS}) Spits out error messages, and also doesn't seem necessary to work

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(manager SHARED manager.cc)
target_link_libraries(manager ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcIntersection SHARED sdcIntersection.cc)
target_link_libraries(sdcIntersection manager ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})


add_library(sdcCar SHARED sdcCar.cc)
target_link_libraries(sdcCar sdcIntersection manager ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
