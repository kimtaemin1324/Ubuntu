#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define DEG2RAD(x) (M_PI / 180.0) * x
#define RAD2DEG(x) (180.0 / M_PI) * x

int count = 0;
double roll, pitch, yaw;
double error_old = 0.0;
double target_heading_yaw = 0.0;

double Kp = 0.0015;
double Ki = 0.0;
double Kd = 0.3;

double yaw_deg;
double error = target_heading_yaw - yaw_deg;
double error_d = error - error_old;
double error_sum = error;

double Steering_Angle = Kp * error + Ki * error_sum + Kd * error_d;//PID제어를 사용하여 조향각 계산



double Constrain_Yaw(double yaw_deg)
{
    if (yaw_deg > 360)// 만약 yaw_deg가 360보다 크면
    {
        yaw_deg = yaw_deg - 360;// 360을 빼서 0부터 360도 사이의 값으로 만듭니다.
    }
    else if (yaw_deg < 0)// 만약 yaw_deg가 0보다 작으면
    {
        yaw_deg = yaw_deg + 360;// 360을 더해서 0부터 360도 사이의 값으로 만듭니다.
    }
    return yaw_deg;// 제한된 yaw 각도를 return합니다
}

void target_yaw_control(geometry_msgs::Twist &cmd_vel)
{
    double yaw_deg = RAD2DEG(yaw);

    yaw_deg = Constrain_Yaw(yaw_deg);

    if (fabs(error) < 0.5) // 오차(error)의 절댓값이 0.5 미만이면
    {
        cmd_vel.linear.x = 0.0; //차량 선속도를 0으로 설정
        cmd_vel.angular.z = 0.0;//차량 각속도를 0으로 설정
    }
    else//그렇지 않으면
    {
        cmd_vel.linear.x = 1.0;//차량 선속도를 1.0으로 설정
        cmd_vel.angular.z = Steering_Angle;// 차량 각속도를 Steering_Angle로 설정
    }
    error_old = error; 
}

void imu1Callback(const sensor_msgs::Imu::ConstPtr& msg) 
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    tf2::Matrix3x3 m(q);      
        m.getRPY(roll, pitch, yaw);

    double yaw_deg = Constrain_Yaw(RAD2DEG(yaw));// Yaw 각도를 radian에서 degree로 바꾸고, 0부터 360도 사이의 값으로 제한한다

    printf("%f\n", yaw_deg);
}

int main(int argc, char **argv)
{	
    geometry_msgs::Twist cmd_vel;

    ros::init(argc, argv, "yaw_control");
    ros::NodeHandle n;
    ros::Subscriber yaw_control_sub = n.subscribe("/imu", 1000, imu1Callback); // IMU 데이터를 수신하는 ROS Subscriber 생성하고 콜백 함수를 등록
    ros::Publisher yaw_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);//ROS Publisher 생성
    ros::Rate loop_rate(30.0);//30.0HZ

    while (ros::ok())
    {
        target_yaw_control(cmd_vel);
        yaw_cmd_vel_pub.publish(cmd_vel);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
