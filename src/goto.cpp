#include <ceng_goto/ceng_goto.h>

Goto::Goto(){
  pose_sub=n_.subscribe<nav_msgs::Odometry>("odom",1,boost::bind(&Goto::poseCallback,this,_1));
  vel_pub=n_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);
  dist_tol_=5.0;
}

Goto::~Goto(){

}

void Goto::poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
  current_pose_.x=msg->pose.pose.position.x;
  current_pose_.y=msg->pose.pose.position.y;
  current_pose_.theta = tf::getYaw(msg->pose.pose.orientation);
}

void Goto::goToPoint(double x, double y){
  while(std::sqrt(std::pow(diff_x_,2)+std::pow(diff_y_,2))>=dist_tol_){
  diff_x_ = x-current_pose_.x;
  diff_y_ = y-current_pose_.y;
  theta_ = atan2(diff_y_, diff_x_);
  diff_t_ = theta_-current_pose_.theta;
  vel_command_.linear.x=1.5*std::sqrt(std::pow(diff_x_,2)+std::pow(diff_y_,2));
  vel_command_.linear.y=0;
  vel_command_.linear.z=0;
  vel_command_.angular.x=0;
  vel_command_.angular.y=0;
  vel_command_.angular.z=4*diff_t_;

  vel_pub.publish(vel_command_);
  }
  vel_command_.linear.x=0;
  vel_command_.angular.z=0;
  vel_pub.publish(vel_command_);
}

int main(int argc,char** argv){
  ros::init(argc,argv,"goto_xy");
  Goto go;

  go.goToPoint(std::atof(argv[1]), std::atof(argv[2]));
  ros::spin();

  return 0;
}
