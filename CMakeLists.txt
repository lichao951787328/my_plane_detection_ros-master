cmake_minimum_required(VERSION 3.14)

project(plane_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosbag
  sensor_msgs
  std_msgs
  cv_bridge
)

find_package(GTest REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(glog REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy)
find_package(PythonLibs REQUIRED)
set(tinyxml2_DIR /usr/local/lib/cmake/tinyxml2)
find_package(tinyxml2 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES plane_detction
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

enable_testing()

include_directories(
  ${PYTHON_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIR} 
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/
)

# add_executable(test_type test/test_type.cpp)

# add_executable(test_type_orginazed_points test/test_type_orginazed_points.cpp)
# add_executable(test_type test/test_type.cpp src/quatree_node.cpp src/quadtree_new.cpp)
# add_executable(test_quatree_node test/test_quatree_node.cpp src/quatree_node.cpp)
# add_executable(test_quatree test/test_quatree.cpp src/quatree_node.cpp src/quadtree_new.cpp)
# add_executable(test_plane test/test_plane.cpp src/quatree_node.cpp src/quadtree_new.cpp src/plane_new.cpp)

add_executable(test_segmentation 
test/test_segmentation.cpp 
src/quatree_node.cpp 
src/quadtree_new.cpp 
src/plane_new.cpp 
src/plane_segmentation.cpp 
src/heightmap.cpp 
src/map_for_bipedrobot_copy.cpp 
src/type.cpp)

add_executable(quatree_picture test/quatree_color.cpp)

add_executable(steps_test test/test_steps.cpp)

add_executable(draw_climbed_steps test/draw_climbed_steps.cpp)
add_executable(draw_walk_steps_GZF test/draw_walk_steps_GZF.cpp)
add_executable(draw_walk_steps test/draw_walk_steps.cpp)
add_executable(draw_cutting_result test/draw_cutting_result.cpp)
add_executable(draw_cutting_result_incline test/draw_cutting_result_incline.cpp)
add_executable(test_heightmap test/test_heightmap.cpp src/heightmap.cpp)



target_link_libraries(
  test_segmentation
  ${OpenCV_LIBRARIES} 
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES}
  glog::glog
  tinyxml2::tinyxml2
  # gtest
  # gtest_main
  # gmock
  # gmock_main
  # pthread
)

target_link_libraries(
  quatree_picture
  ${OpenCV_LIBRARIES} 
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES}
)

target_link_libraries(
  steps_test
  # ${OpenCV_LIBRARIES} 
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES}
)

target_link_libraries(
  draw_climbed_steps
  ${OpenCV_LIBRARIES} 
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES}
)

target_link_libraries(
  draw_walk_steps_GZF
  ${OpenCV_LIBRARIES} 
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES}
)

target_link_libraries(
  draw_walk_steps
  ${OpenCV_LIBRARIES} 
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES}
)

target_link_libraries(
  draw_cutting_result
  ${OpenCV_LIBRARIES} 
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(
  draw_cutting_result_incline
  ${OpenCV_LIBRARIES} 
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(
  test_heightmap
  ${OpenCV_LIBRARIES} 
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${PCL_LIBRARIES}
)
