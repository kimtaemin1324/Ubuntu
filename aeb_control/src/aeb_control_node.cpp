#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

double front_sonar = 0.0; // 전방 초음파 거리를 저장하는 전역 변수 선언

void Front_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)// 새로운 sensor_msgs::Range 메시지가 수신되면 front_sonar 변수를 업데이트하는 콜백 함수
{
  front_sonar = msg->range;
}

int main(int argc, char **argv)
{
  int count = 0; // 카운터 변수 초기화
  
  geometry_msgs::Twist cmd_vel;// 로봇의 속도를 제어하기 위한 Twist 메시지 변수 선언
  
  ros::init(argc, argv, "aeb_control_sonar");// ROS 노드 초기화
  ros::NodeHandle n;
  
  ros::Subscriber front_sonar_range_sub = n.subscribe("/range_front", 1000, Front_Sonar_Callback);// 토픽에 대한 콜백 함수 Front_Sonar_Callback로 구독

  ros::Publisher sonar_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);//  토픽에 Twist 메시지를 발행

  ros::Rate loop_rate(30.0);  //30.0HZ
  
  while (ros::ok())
  {
    ROS_INFO("Sonar Range: [%f]", front_sonar);// 전방 초음파 거리 값을 로그에 출력

    if (front_sonar < 1.0)// 전방 초음파 거리에 기반한 제어 로직
    {
      cmd_vel.linear.x = 0.0;  // 초음파 거리가 1.0 미만이면 로봇을 정지시킴
    }
    else
    {
      cmd_vel.linear.x = 0.5;  // 초음파 거리가 1.0 이상이면 로봇의 속도를 0.5로 설정
    }
   
   
    sonar_cmd_vel_pub.publish(cmd_vel); // Twist 메시지를 발행
    ros::spinOnce();
    loop_rate.sleep();// 주기적인 대기
    ++count;//카운터 증가
  }
  return 0;
}

