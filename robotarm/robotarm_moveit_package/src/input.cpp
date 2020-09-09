#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <robotarm_moveit_package/DynamicParamConfig.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf2/LinearMath/Quaternion.h>

geometry_msgs::Pose pose;
tf2::Quaternion quaternion;

void callback(robotarm_moveit_package::DynamicParamConfig &config, uint32_t level) {
  ROS_INFO_STREAM("\n" <<
                  "x  ="<< config.x << "\n" <<
                  "y  ="<< config.y << "\n" <<
                  "z  ="<< config.z << "\n" <<
                  "r  ="<< config.roll << "\n" <<
                  "p  ="<< config.pitch << "\n" <<
                  "y  ="<< config.yaw);
  pose.position.x = config.x;
  pose.position.y = config.y;
  pose.position.z = config.z;
  quaternion.setRPY( config.roll, config.pitch, config.yaw );
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "input_node");
  ros::NodeHandle nh;
  
  ros::AsyncSpinner spin(1);
  spin.start();
  
  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  moveit::planning_interface::MoveGroupInterface::Plan goal_plan;
  
  ros::Publisher display_pub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  dynamic_reconfigure::Server<robotarm_moveit_package::DynamicParamConfig> server;
  dynamic_reconfigure::Server<robotarm_moveit_package::DynamicParamConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // Set the tolerance to consider the goal achieved
    move_group.setGoalTolerance(0.2);

    // Set the target pose, which is the goal we already defined
    move_group.setPoseTarget(pose);

    // Perform the planning step, and if it succeeds display the current
    // arm trajectory and move the arm
    if (move_group.plan(goal_plan))
    {
        moveit_msgs::DisplayTrajectory display_msg;
        display_msg.trajectory_start = goal_plan.start_state_;
        display_msg.trajectory.push_back(goal_plan.trajectory_);
        display_pub.publish(display_msg);

        sleep(5.0);

        move_group.move();
    }

  ros::waitForShutdown();
  return 0;
}