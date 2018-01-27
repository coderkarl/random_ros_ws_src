#include <ros/ros.h>
#include <tank_drive_cmd/TankDriveCmd.h>
#include <sensor_msgs/Joy.h>
#include <cmath> //std::abs

//for odom
#include <geometry_msgs/Twist.h>

//for blade control
#include <std_msgs/Int16.h>


class TeleopBot
{
public:
  TeleopBot();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int left_, right_, drive_cmd_min_, drive_type_;
  double cmd_scale_, linear_scale_, botwidth_, steer_scale_;
  double wheel_base_, max_steer_deg_;
  ros::Publisher drive_cmd_pub_;
  ros::Publisher blade_cmd_pub_;
  ros::Subscriber joy_sub_;

  //for odom
  ros::Publisher twist_pub_;

};


TeleopBot::TeleopBot():
  drive_type_(0),
  left_(1),
  right_(2)
{

  nh_.param("axis_left", left_, left_);
  nh_.param("axis_right", right_, right_);
  nh_.param("scale_cmd", cmd_scale_, cmd_scale_);
  nh_.param("drive_type", drive_type_, drive_type_);
  nh_.param("scale_steer", steer_scale_, steer_scale_);

  //for odom
  nh_.param("scale_linear_mps", linear_scale_, linear_scale_);
  nh_.param("botwidth_m", botwidth_, botwidth_);
  nh_.param("min_drive_cmd", drive_cmd_min_, drive_cmd_min_);
  nh_.param("wheel_base", wheel_base_, wheel_base_);
  nh_.param("max_steer_deg", max_steer_deg_, max_steer_deg_);


  //drive_cmd_pub_ = nh_.advertise<tank_drive_cmd::TankDriveCmd>("drive_cmd", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopBot::joyCallback, this);

  //for odom
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",10);
  blade_cmd_pub_ = nh_.advertise<std_msgs::Int16>("blade_cmd",10);

}

void TeleopBot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  /*tank_drive_cmd::TankDriveCmd my_cmd;
  my_cmd.left_cmd = left_scale_*joy->axes[left_];
  my_cmd.right_cmd = right_scale_*joy->axes[right_];
  drive_cmd_pub_.publish(my_cmd);

  //for odom
  float left_cmd, right_cmd;
  left_cmd = my_cmd.left_cmd;
  left_cmd = left_cmd*(std::abs(left_cmd) > drive_cmd_min_);
  right_cmd = my_cmd.right_cmd;
  right_cmd = right_cmd*(std::abs(right_cmd) > drive_cmd_min_);
  geometry_msgs::Twist tw;
  tw.linear.x = linear_scale_ / 255.0 * 0.5 * (left_cmd + right_cmd);
  tw.linear.y = 0;
  tw.angular.z = linear_scale_ / 255.0 / angular_scale_ * (right_cmd - left_cmd);
  twist_pub_.publish(tw); */

  float left_cmd, right_cmd;
  geometry_msgs::Twist tw;
  
  if(drive_type_ == 0)
  {
      left_cmd = cmd_scale_*joy->axes[left_];
      left_cmd = left_cmd*(std::abs(left_cmd) > drive_cmd_min_);
      right_cmd = cmd_scale_*joy->axes[right_];
      right_cmd = right_cmd*(std::abs(right_cmd) > drive_cmd_min_);

      tw.linear.x = linear_scale_ / cmd_scale_* 0.5 * (left_cmd + right_cmd);
      tw.linear.y = 0;
      tw.angular.z = linear_scale_ / cmd_scale_ / botwidth_ * (right_cmd - left_cmd);
  }
  else if(drive_type_ == 1) // Literal Drive Commands, +/-100% speed, 225-850 raw steer pos
  {
      left_cmd = cmd_scale_*joy->axes[left_];
      left_cmd = left_cmd*(std::abs(left_cmd) > drive_cmd_min_);
      right_cmd = steer_scale_*joy->axes[right_];
      
      tw.linear.x = left_cmd;
      tw.linear.y = 0;
      tw.angular.z = right_cmd;      
  }
  else // actual linear vel and angular vel
  {
      left_cmd = cmd_scale_*joy->axes[left_];
      left_cmd = left_cmd*(std::abs(left_cmd) > drive_cmd_min_);
      right_cmd = steer_scale_*joy->axes[right_];
      
      double steer_angle_rad = -right_cmd*max_steer_deg_*3.14/180.0; // Positive CCW turns 
      double curvature = steer_angle_rad / wheel_base_;
      
      tw.linear.x = left_cmd*linear_scale_;
      tw.linear.y = 0;
      tw.angular.z = left_cmd*linear_scale_*curvature;      
  }
  twist_pub_.publish(tw);
  
  std_msgs::Int16 blade_msg;
  int blade_ON = joy->buttons[3]; //Y button
  int blade_OFF = joy->buttons[2]; // X button

  if(blade_OFF)
  {
      blade_msg.data = 0;
      blade_cmd_pub_.publish(blade_msg);
  }
  else if(blade_ON)
  {
      blade_msg.data = 1;
      blade_cmd_pub_.publish(blade_msg);
  }
  
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_bot");
  TeleopBot teleop_bot;

  ros::spin();
}
