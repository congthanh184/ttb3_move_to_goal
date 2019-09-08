#include <ros/ros.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

class LocalGoal {
  private:
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
  public:
    int x; int y; int z;
    int roll; int pitch; int yaw;
    LocalGoal(int x, int y, int z, int roll, int pitch, int yaw):
      x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw), tfListener(tfBuffer)
  {
  };
    ~LocalGoal(){};
    void getCurrentPose() {
      geometry_msgs::TransformStamped transform_stamped;
      try {
        transform_stamped = tfBuffer.lookupTransform("odom", "base_footprint", ros::Time(0), ros::Duration(5.0));
      }
      catch (tf2::TransformException ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }
      tf2::Stamped<tf2::Transform> tf2_stamped;
      tf2::convert(transform_stamped, tf2_stamped);
      // tf2::fromMsg(transform_stamped, tf2_stamped);
      ROS_INFO_STREAM(tf2_stamped.getRotation());
      // return tf2_stamped.transform;
    }
    void getWorldGoal() {
      // tf2::Transform local = getCurrentPose();
      // ROS_INFO_STREAM(local.getRotation());
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_turtlebot_to_goal");

  ros::NodeHandle node;
  // LocalGoal goal(0, 0, 0, 0, 0, 0);
  // goal.getCurrentPose();
  // goal.getWorldGoal();
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  ros::Rate rate(10.0);
  rate.sleep();
  ROS_INFO("************* Test ******************");
  // tfListener.waitForTransform("odom", "base_footprint", ros::Time(), ros::Duration(5.0));
  try{
    transformStamped = tfBuffer.lookupTransform("odom", "base_footprint",
        ros::Time(0), ros::Duration(5.0));
    tf2::Stamped<tf2::Transform> tf2_stamped;
    tf2::convert(transformStamped, tf2_stamped);
    // tf2::fromMsg(transform_stamped, tf2_stamped);
    ROS_INFO_STREAM(tf2_stamped.getRotation().getAngle());
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  return 0;
}
