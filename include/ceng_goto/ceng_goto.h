#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <tf/tf.h>
#include <math.h>

class Goto{
public:
  Goto();
  ~Goto();
  void goToPoint(double x,double y);

private:
  ros::NodeHandle n_;
  ros::Subscriber pose_sub;
  ros::Publisher vel_pub;
  geometry_msgs::Pose2D current_pose_;
  geometry_msgs::Twist vel_command_;
  double diff_t_,diff_y_,diff_x_,theta_,dist_tol_;
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);

};
