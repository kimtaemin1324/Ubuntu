#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#define TSL1401CL_SIZE 320
#define THRESHOLD 0.01  
#define DEG2RAD(x) (M_PI/180.0)*x
#define RAD2DEG(x) (180.0/M_PI)*x
#define Line_Center 147  
#define OFFSET 13

double Kp = 0.0015;
double Ki = 0.0;
double Kd = 0.005;

double error = 0.0;
double error_d = 0.0;
double error_sum = 0.0;
double error_old = 0.0;

double Steering_Angle = 0.0;

double tsl1401cl_data[TSL1401CL_SIZE];

int LineSensor_threshold_Data[TSL1401CL_SIZE];

void threshold(double tsl1401cl_data[], int ThresholdData[], int tsl1401cl_size, double threshold)
{
    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        if (tsl1401cl_data[i] > threshold)// 현재 요소가 지정한 threshold값보다 큰지 확인 
        {
            ThresholdData[i] = 255; //현재 tsl1401cl_data가 threshold보다 크면 ThresholdData->255
        }
        else
        {
            ThresholdData[i] = 0; //아니면 0
        }
    }
}

void tsl1401cl_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg) //tsl1401cl을 받아오는 콜백함수
{
    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        tsl1401cl_data[i] = msg->data[i];
    }
    threshold(tsl1401cl_data, LineSensor_threshold_Data, TSL1401CL_SIZE, THRESHOLD);
}

int find_line_center()
{
    double centroid = 0.0;
    double mass_sum = 0.0;

    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
		centroid += LineSensor_threshold_Data[i] * i;// 중심값은 해당 위치의 데이터 값에 위치를 곱한 값을 더하여 계산
        mass_sum += LineSensor_threshold_Data[i];// 센서로부터 계산되어 나오는 error의 누적값의 합으로 계산
    }

    centroid = centroid / mass_sum; 

    return centroid;
}

void PID_lane_control(geometry_msgs::Twist &cmd_vel)
{
    double lineCenter = find_line_center();
    
    error = Line_Center - lineCenter + OFFSET; //error값 구하는 수식
    error_sum += error; //error값들의 합을 구하는 수식
    error_d = error - error_old; //error의 변화율 계산
    
    Steering_Angle = Kp * error + Kd * error_d + Ki * error_sum; //PID제어식을 사용하여 조향각 계산

    cmd_vel.linear.x = 0.8; //차량 선속도 설정
    cmd_vel.angular.z = Steering_Angle; //차량 각속도 Steering_Angle값으로 설정
    
    bool recognize_X = true; //recognize_X의 기본값 true로 설정
    
    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        if (LineSensor_threshold_Data[i] != 0) //LineSensor값이 들어오지 않으면
        {
            recognize_X = false; //recognize_X를 false로 설정
            break;
        }
    }

    if (recognize_X = false) //recognize_X가 false이면
    {
        cmd_vel.linear.x = 0.0; //차량 선속도를 0으로 설정
        cmd_vel.angular.z = 0.0; //차량 각속도를 0으로 설정
        return;
    }
    error_old = error; //error_old값 갱신
}


int main(int argc, char **argv)
{
    int count = 0;

    geometry_msgs::Twist cmd_vel;

    ros::init(argc, argv, "line_control");
    ros::NodeHandle nh;

    ros::Subscriber tsl1401cl_sub = nh.subscribe("/tsl1401cl", 10, tsl1401cl_Callback);
    ros::Publisher tst1401cl_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);

    ros::Rate loop_rate(30.0);
    while (ros::ok())
    {
        printf("Threshold Data: \n");
        
        for (int i = 0; i < TSL1401CL_SIZE; i++)
        {
            printf("%d ", LineSensor_threshold_Data[i]);
        }
        printf("\n");

        double centroid = find_line_center();
        printf("Line Centroid: %f\n", centroid);

        PID_lane_control(cmd_vel);

        tst1401cl_cmd_vel_pub.publish(cmd_vel);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}




