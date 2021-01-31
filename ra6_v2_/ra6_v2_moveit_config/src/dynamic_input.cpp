#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <ra6_v2_moveit_config/DynamicParamConfig.h>

void callback(ra6_v2_moveit_config::DynamicParamConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %s", 
            config.x, config.y, config.z,
            config.roll, config.pitch, config.yaw,
            config.send?"True":"False");
    
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "server");

  dynamic_reconfigure::Server<ra6_v2_moveit_config::DynamicParamConfig> server;
  dynamic_reconfigure::Server<ra6_v2_moveit_config::DynamicParamConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
