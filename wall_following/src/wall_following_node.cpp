#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

double Front_Sonar_data = 0.0;
double Left_Sonar_data = 0.0;
double Right_Sonar_data = 0.0;
double error_integral = 0.0;
double error_old = 0.0;

void Front_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  Front_Sonar_data = msg->range;
  ROS_INFO("Front Sonar Data: %.2f", Front_Sonar_data);
}

void Left_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  Left_Sonar_data = msg->range;
  ROS_INFO("Left Sonar Data: %.2f", Left_Sonar_data);
}

void Right_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  Right_Sonar_data = msg->range;
  ROS_INFO("Right Sonar Data: %.2f", Right_Sonar_data);
}

void PID_Wall_Following(geometry_msgs::Twist& cmd_vel, double Kp, double Ki, double Kd)
{
  double error = Left_Sonar_data - Right_Sonar_data;
  double error_d = error - error_old;
  double error_sum = 0.0;
  error_sum += error;

  double steering_control = Kp * error + Ki * error_sum + Kd * error_d;

  if (Front_Sonar_data <= 1.3) 
  {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
  } 
  else 
  {
    cmd_vel.linear.x = 0.65;
    cmd_vel.angular.z = steering_control;
  }

  error_old = error;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wall_following");
  ros::NodeHandle n;

  double Kp = 0.5;
  double Ki = 0.00;
  double Kd = 0.1;

  geometry_msgs::Twist cmd_vel;

  ros::Subscriber Front_Sonar_sub = n.subscribe("/range_front", 1000, Front_Sonar_Callback);
  ros::Subscriber Left_Sonar_sub = n.subscribe("/range_front_left", 1000, Left_Sonar_Callback);
  ros::Subscriber Right_Sonar_sub = n.subscribe("/range_front_right", 1000, Right_Sonar_Callback);

  ros::Publisher sonar_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);

  ros::Rate loop_rate(30.0);

  while (ros::ok())
  {
    PID_Wall_Following(cmd_vel, Kp, Ki, Kd);
    sonar_cmd_vel_pub.publish(cmd_vel);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
