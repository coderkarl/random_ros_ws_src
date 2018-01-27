#include <ros/ros.h>
#include <tank_drive_cmd/TankDriveCmd.h>
#include <sensor_msgs/Joy.h>
#include <cmath> //std::abs

//for odom
#include <geometry_msgs/Twist.h>


class TeleopBot
{
public:
  TeleopBot();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int left_vert_, right_vert_, right_horz_, drive_cmd_min_;
  double cmd_scale_, linear_scale_, botwidth_;
  ros::Publisher drive_cmd_pub_;
  ros::Subscriber joy_sub_;

  //for odom
  ros::Publisher twist_pub_;

};


TeleopBot::TeleopBot():
  left_vert_(1),
  right_vert_(2),
  right_horz_(3)
{

  nh_.param("axis_left", left_, left_);
  nh_.param("axis_right", right_, right_);
  nh_.param("scale_cmd", cmd_scale_, cmd_scale_);

  //for odom
  nh_.param("scale_linear_mps", linear_scale_, linear_scale_);
  nh_.param("botwidth_m", botwidth_, botwidth_);
  nh_.param("min_drive_cmd", drive_cmd_min_, drive_cmd_min_);


  //drive_cmd_pub_ = nh_.advertise<tank_drive_cmd::TankDriveCmd>("drive_cmd", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopBot::joyCallback, this);

  //for odom
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",10);

}

void TeleopBot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  float speed_cmd, steer_cmd;
  speed_cmd = cmd_scale_*joy->axes[left_vert_];
  speed_cmd = left_cmd*(std::abs(left_cmd) > drive_cmd_min_);
  steer_cmd = steer_scale_*joy->axes[right_horz_];

  geometry_msgs::Twist tw;
  tw.linear.x = linear_scale_ / cmd_scale_* 0.5 * (left_cmd + right_cmd);
  tw.linear.y = 0;
  tw.angular.z = linear_scale_ / cmd_scale_ / botwidth_ * (right_cmd - left_cmd);
  twist_pub_.publish(tw);
  
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_bot");
  TeleopBot teleop_bot;

  ros::spin();
}
