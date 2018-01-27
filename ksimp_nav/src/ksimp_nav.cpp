#include <ros/ros.h>
#include <cmath> //std::abs
//#include <math.h> //atan2 HELLO
#include <list>

#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

//for cmd_vel
#include <geometry_msgs/Twist.h>


class SimpNav
{
public:
  SimpNav();
  void loopTask();

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void goalCallback(const geometry_msgs::Pose2D::ConstPtr& goal);

  ros::NodeHandle nh_;

  geometry_msgs::Pose2D goal_pose, jeep_pose;
  bool goal_pose_received, jeep_pose_received;
  
  std::list<geometry_msgs::Pose2D> waypoints;
  
  ros::Subscriber odom_sub_;
  ros::Subscriber goal_sub_;

  //for cmd_vel
  ros::Publisher cmd_pub_;

};


SimpNav::SimpNav():
  goal_pose_received(false),
  jeep_pose_received(false)
{
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 20, &SimpNav::odomCallback, this);
  goal_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("goal",2, &SimpNav::goalCallback, this);

  //for cmd_vel
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",10);
}

void SimpNav::goalCallback(const geometry_msgs::Pose2D::ConstPtr& goal_msg)
{
    if(jeep_pose_received)
    {
        //goal_pose.x = goal_msg->x;
        //goal_pose.y = goal_msg->y;
        //goal_pose.theta = goal_msg->theta;
        waypoints.push_back(*goal_msg);
        if(!goal_pose_received)
        {
            goal_pose = waypoints.back();
        }
        goal_pose_received = true;
        ROS_INFO("Goal-> x: [%f], y: [%f], theta: [%f]", waypoints.back().x, waypoints.back().y, waypoints.back().theta);
        ROS_INFO("Pose-> x: [%f], y: [%f], theta: [%f]", jeep_pose.x, jeep_pose.y, jeep_pose.theta);
    }
}

void SimpNav::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(!jeep_pose_received)
    {
      jeep_pose_received = true;
      ROS_INFO("Odom Received");
      ROS_INFO("Pose-> x: [%f], y: [%f], theta: [%f]", jeep_pose.x, jeep_pose.y, jeep_pose.theta);
    }
    //geometry_msgs::Quaternion odom_quat = msg->pose.pose.orientation;
    
    jeep_pose.x = msg->pose.pose.position.x;
    jeep_pose.y = msg->pose.pose.position.y;
    tf::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    jeep_pose.theta = yaw;

	//ROS_INFO("Seq: [%d]", msg->header.seq);
	//ROS_INFO("Pose-> x: [%f], y: [%f], theta: [%f]", jeep_pose.x, jeep_pose.y, jeep_pose.theta);
    //ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z
}

void SimpNav::loopTask()
{
  //~ left_cmd = cmd_scale_*joy->axes[left_];
  //~ left_cmd = left_cmd*(std::abs(left_cmd) > drive_cmd_min_);
  //~ right_cmd = steer_scale_*joy->axes[right_];
  
  //~ double steer_angle_rad = -right_cmd*max_steer_deg_*3.14/180.0; // Positive CCW turns 
  //~ double curvature = steer_angle_rad / wheel_base_;
  
  //~ tw.linear.x = left_cmd*linear_scale_;
  //~ tw.linear.y = 0;
  //~ tw.angular.z = left_cmd*linear_scale_*curvature;
  
  geometry_msgs::Twist tw;
  
  double ex = goal_pose.x - jeep_pose.x;
  double ey = goal_pose.y - jeep_pose.y;
  double edsqd = ex*ex + ey*ey;
  double a = atan2(ey, ex);
  double eth = a - jeep_pose.theta;
  if(eth < -3.14)
  {
    eth += 6.28;
  }
  else if(eth > 3.14)
  {
    eth -= 6.28;
  }
  
  double wheel_base = 0.76;
  double steer_angle, curvature;
  
  tw.linear.y = 0;
  tw.angular.z = 0;
  
  if(goal_pose_received && edsqd > 0.25)
  {
    tw.linear.x = 1.0;
    //tw.angular.z = -eth*50*(double)(fabs(eth) > 0.1);
    steer_angle = -eth;
    if(steer_angle > 18.0*3.14/180.0)
        steer_angle = 18.0*3.14/180.0;
    else if(steer_angle < -18.0*3.14/180.0)
        steer_angle = -18.0*3.14/180.0;
    
    if(fabs(steer_angle) > 6)
        tw.linear.x = 1.3;
    
    curvature = steer_angle / wheel_base;
    tw.angular.z = curvature * tw.linear.x;
  }
  //~ else if(goal_pose_received && (ex < -0.2 || ey )
  //~ {
    //~ tw.linear.x = -0.7;
    //~ tw.angular.z = eth*50*(double)(fabs(eth) > 0.1);
  //~ }
  else
  {
      if(waypoints.size() > 1)
      {
          waypoints.pop_front();
          goal_pose = waypoints.front();
      }
      tw.linear.x = 0;
  }
  cmd_pub_.publish(tw);
    
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ksimp_nav");
  SimpNav simp_nav;

  //ros::spin();

  ros::Rate rate(5);

  while(ros::ok())
	{
		ros::spinOnce();
	    simp_nav.loopTask();
        rate.sleep();
	}

  return 0;
}
