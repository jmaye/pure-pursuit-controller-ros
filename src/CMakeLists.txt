remake_ros_package(
  pure_pursuit_controller
  DEPENDS roscpp rospy rosbash geometry_msgs nav_msgs visualization_msgs tf
  DESCRIPTION "Pure pursuit controller."
)
remake_ros_package_config_generate(PACKAGE pure_pursuit_controller
  EXTRA_CFLAGS -std=c++0x)
