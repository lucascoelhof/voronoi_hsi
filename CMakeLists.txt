cmake_minimum_required(VERSION 2.8.3)
project(voronoi_hsi)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  cv_bridge
  image_transport
  stage_ros
  message_generation
)

add_message_files(
   FILES
   GaussianArray.msg
   Gaussian.msg
   VoronoiTesselation.msg
   RobotGain.msg
   RobotGainArray.msg
)

add_service_files(
   FILES
   SetRobotPose.srv
   SetColorPlot.srv
   SetOccGrid.srv
   RemoveColorPlot.srv
   RemoveRobot.srv
 )

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   nav_msgs
   sensor_msgs
)


catkin_package(
  LIBRARIES voronoi_hsi
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport nav_msgs sensor_msgs std_msgs tf message_runtime
  DEPENDS system_lib
)


install(PROGRAMS
   src/voronoi_h.py
   src/image_service.py
   src/ar_track_to_odom.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


