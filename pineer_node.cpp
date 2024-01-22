#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define DEG2RAD(x) (M_PI / 180.0) * x
#define RAD2DEG(x) (180.0 / M_PI) * x
#define TSL1401CL_SIZE 320
#define THRESHOLD 0.1
#define Line_Center 160
#define OFFSET 0

int mission_flag = 0;

double tsl1401cl_data[TSL1401CL_SIZE];

int LineSensor_threshold_Data[TSL1401CL_SIZE];

double roll, pitch, yaw;
double error_line_old = 0.0;
double error_yaw_old = 0.0;
double error_wall_old = 0.0;


void threshold(double tsl1401cl_data[], int ThresholdData[], int tsl1401cl_size, double threshold)
{
    for (int i = 0; i < tsl1401cl_size; i++)
    {
        if (tsl1401cl_data[i] > threshold)// 현재 요소가 지정한 threshold값보다 큰지 확인
        {
            ThresholdData[i] = 255;//현재 tsl1401cl_data가 threshold보다 크면 ThresholdData->255
        }
        else
        {
            ThresholdData[i] = 0;//아니면 0
        }
    }
}

double find_line_center()
{
    double centroid = 0.0;
    double mass_sum = 0.0;

    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
		centroid += LineSensor_threshold_Data[i] * i;// 중심값은 해당 위치의 데이터 값에 위치를 곱한 값을 더하여 계산
        mass_sum += LineSensor_threshold_Data[i];// 센서로부터 계산되어 나오는 error의 누적값의 합으로 계산   
    }

    if (mass_sum == 0)
    {
        return 0.0;
    }

    centroid = centroid / mass_sum;

    return centroid;
}

void tsl1401cl_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        tsl1401cl_data[i] = msg->data[i];
    }
    threshold(tsl1401cl_data, LineSensor_threshold_Data, TSL1401CL_SIZE, THRESHOLD);
    
    printf("Threshold Data: \n");

    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        printf("%d ", LineSensor_threshold_Data[i]);
    }
    printf("\n");

    double centroid = find_line_center();
    printf("Line Centroid: %f\n", centroid);
}


geometry_msgs::Twist line_control(double Kp_line, double Ki_line, double Kd_line)
{
    geometry_msgs::Twist cmd_vel;

    double lineCenter = find_line_center();

    double error_line = Line_Center - lineCenter + OFFSET;//error값 구하는 수식
    double error_line_d = error_line - error_line_old;//error의 변화율 계산
    double error_line_sum = 0.0;

    error_line_sum += error_line;//error값들의 합을 구하는 수식

    double steering_angle = Kp_line * error_line + Ki_line * error_line_sum + Kd_line * error_line_d;//PID제어식을 사용하여 조향각 계산

    cmd_vel.linear.x = 0.7; //차량 선속도 설정
    cmd_vel.angular.z = steering_angle; //차량 각속도 Steering_Angle값으로 설정

    error_line_old = error_line;//error_old값 갱신

    return cmd_vel;
}


double Front_Sonar_data = 0.0;
double Left_Sonar_data = 0.0;
double Right_Sonar_data = 0.0;

void Front_Sonar_Callback(const sensor_msgs::Range::ConstPtr &msg)// Front_Sonar 센서에 대한 콜백 함수
{
    Front_Sonar_data = msg->range;
    printf("Front Sonar Data: %.2f", Front_Sonar_data);
}

void Left_Sonar_Callback(const sensor_msgs::Range::ConstPtr &msg)// Left_Sonar 센서에 대한 콜백 함수
{
    Left_Sonar_data = msg->range;
    printf("Left Sonar Data: %.2f", Left_Sonar_data);
}

void Right_Sonar_Callback(const sensor_msgs::Range::ConstPtr &msg)// Right_Sonar 센서에 대한 콜백 함수
{
    Right_Sonar_data = msg->range;
    printf("Right Sonar Data: %.2f", Right_Sonar_data);
}

geometry_msgs::Twist wall_following(double Kp_wall, double Ki_wall, double Kd_wall)// PID_Wall_Following 제어를 구현하는 함수
{
    geometry_msgs::Twist cmd_vel;

    double error_wall = Left_Sonar_data - Right_Sonar_data;// Left_Sonar 센서와 Right_Sonar 센서 간의 거리 차이 계산
    double error_wall_d = error_wall - error_wall_old;// 오차의 변화율 계산
    double error_wall_sum = 0.0;// 오차의 적분값
    error_wall_sum += error_wall;

    double steering_control = Kp_wall * error_wall + Ki_wall * error_wall_sum + Kd_wall * error_wall_d;// PID 공식을 사용하여 조향 제어 계산

    if (Front_Sonar_data< 0.3)// Front_Sonar 센서가 0.3보다 작으면
    {
        cmd_vel.linear.x = 0.0;//차량 선속도를 0으로 설정
        cmd_vel.angular.z = 0.0;//차량 각속도를 0으로 설정
    }
    else
    {
        cmd_vel.linear.x = 0.5;//차량 선속도 설정
        cmd_vel.angular.z = steering_control;//차량 각속도 Steering_Angle값으로 설정
    }

    error_wall_old = error_wall;

    return cmd_vel;
}

double Deg_Yaw(double yaw_deg)
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

geometry_msgs::Twist yaw_control(double Kp_yaw, double Ki_yaw, double Kd_yaw, double target_yaw_degree)
{
    geometry_msgs::Twist cmd_vel;

    double yaw_deg = RAD2DEG(yaw);
    yaw_deg = Deg_Yaw(yaw_deg);

    double error_yaw = target_yaw_degree - yaw_deg;

    if (error_yaw > 180)
    {
        error_yaw = error_yaw - 360;
    }
    else if (error_yaw < -180)
    {
        error_yaw = error_yaw + 360;
    }

    double error_yaw_sum = 0.0;
    double error_yaw_d = error_yaw - error_yaw_old;

    error_yaw_sum += error_yaw;

    double Steering_Angle = Kp_yaw * error_yaw + Ki_yaw * error_yaw_sum + Kd_yaw * error_yaw_d;

    cmd_vel.linear.x = 0.5;
    cmd_vel.angular.z = Steering_Angle;

    if (fabs(error_yaw) < 1.0)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        mission_flag++;
    }

    error_yaw_old = error_yaw;

    return cmd_vel;
}

void imu1Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    double yaw_deg = Deg_Yaw(RAD2DEG(yaw));

    printf("%f\n", yaw_deg);
}




int main(int argc, char **argv)
{
    int count = 0;
    double target_yaw_degree = 0.0;

    ros::init(argc, argv, "pioneer_control");
    ros::NodeHandle nh;

    ros::Subscriber tsl1401cl_sub = nh.subscribe("/tsl1401cl", 10, tsl1401cl_Callback);
    ros::Subscriber yaw_control_sub = nh.subscribe("/imu", 1000, imu1Callback);
    ros::Subscriber front_sonar_sub = nh.subscribe("/range_front", 1000, Front_Sonar_Callback);// 센서 데이터를 받기 위한 Subscriber 초기화
    ros::Subscriber left_sonar_sub = nh.subscribe("/range_front_left", 1000, Left_Sonar_Callback);// 센서 데이터를 받기 위한 Subscriber 초기화
    ros::Subscriber right_sonar_sub = nh.subscribe("/range_front_right", 1000, Right_Sonar_Callback);// 센서 데이터를 받기 위한 Subscriber 초기화

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);// 제어 명령을 발행하기 위한 Publisher 초기화

    ros::Rate loop_rate(30.0);

    geometry_msgs::Twist cmd_vel;
	
    double Kp_line = 0.002;
    double Ki_line = 0.0;
    double Kd_line = 0.02;
  
    double Kp_yaw = 0.02;
    double Ki_yaw = 0.0;
    double Kd_yaw = 0.2;
	
    double Kp_wall = 0.4;
    double Ki_wall = 0.0;
    double Kd_wall = 0.8;

    while (ros::ok())
    {
        find_line_center();

        switch (mission_flag)
        {
        case 0:
            if (find_line_center() == 0.0)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
            }
            else
            {
                mission_flag++;
            }
            break;

        case 1:
            if (find_line_center() != 0.0)
            {
                cmd_vel = line_control(Kp_line, Ki_line, Kd_line);
            }
            if (find_line_center() == 0.0)
            {
                mission_flag++;
            }
            break;

        case 2:
            if (Front_Sonar_data > 1.0)
            {
                cmd_vel.linear.x = 0.7;
                cmd_vel.angular.z = 0.0;
            }
            else
            {
                target_yaw_degree = 270.0;
                mission_flag++;
            }
            break;

        case 3:
            cmd_vel = yaw_control(Kp_yaw, Ki_yaw, Kd_yaw, target_yaw_degree);
            break;

        case 4:
            cmd_vel = wall_following(Kp_wall, Ki_wall, Kd_wall);
            if (Front_Sonar_data < 0.9)
            {
                target_yaw_degree = 180.0;
                mission_flag = 5;
            }
            break;

        case 5:
            cmd_vel = yaw_control(Kp_yaw, Ki_yaw, Kd_yaw, target_yaw_degree);
            break;

        case 6:
            bool finish = true;
            for (int i = 0; i < TSL1401CL_SIZE; i++)
            {
                if (LineSensor_threshold_Data[i] != 255)
                {
                    finish = false;
                    break;
                }
            }

            if (finish)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
            }
            else
            {
                cmd_vel = line_control(Kp_line, Ki_line, Kd_line);
            }
            break;
            
        }
		printf("%d\n",mission_flag);
        cmd_vel_pub.publish(cmd_vel);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
