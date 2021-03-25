cmake_minimum_required(VERSION 3.0.2)
project(pepper_vr_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  sensor_msgs
  cv_bridge
  image_transport
  naoqi_bridge_msgs
  )

find_package(OpenCV REQUIRED)

catkin_package(
	CATKIN_DEPENDS roscpp
	INCLUDE_DIRS include
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_subdirectory(vendors/rendering_engine)


add_executable(RobotControllerApp
                src/EntryPoint.cpp
                src/RobotControllerApp.cpp
                src/RosNode.cpp
                src/CameraView.cpp)

target_include_directories(RobotControllerApp PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/vendors/rendering_engine/include")

add_dependencies(RobotControllerApp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(RobotControllerApp PRIVATE ${catkin_LIBRARIES} ${OpenCV_LIBS} rendering_engine glew glfw openvr_api assimp imgui)