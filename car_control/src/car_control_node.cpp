#include <sys/ioctl.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>

#define I2C_ADDRESS 0x05

static const char *deviceName = "/dev/i2c-0";

#define MAX_RIGHT_ANGLE -45
#define MAX_LEFT_ANGLE 45

#define MAX_ROBOT_SPEED 255
#define MIN_ROBOT_SPEED -255

#define DEG2RAD(x) (M_PI / 180.0 * (x))
#define RAD2DEG(x) ((x) * 180.0 / M_PI)
#define RPM2RPS(x) ((x) / 60)
#define RPS2RPM(x) ((x)*60)

union Data {
    short value;
    char bytes[2];
} steeringAngle, motorSpeed;

unsigned char protocolData[9] = {0};

double speedFactor = 255;
double steerFactor = 20;
int i2cFile;

int openI2C() {
    int file;

    if ((file = open(deviceName, O_RDWR)) < 0) 
    {
        fprintf(stderr, "I2C: Failed to access %s\n", deviceName);
        exit(1);
    }
    printf("I2C: Connected\n");

    printf("I2C: acquiring bus to 0x%x\n", I2C_ADDRESS);
    if (ioctl(file, I2C_SLAVE, I2C_ADDRESS) < 0) 
    {
        fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", I2C_ADDRESS);
        exit(1);
    }

    return file;
}

void closeI2C(int fd) {
    close(fd);
}

void cmdCallback(const geometry_msgs::Twist &cmdVel) 
{
    double angularTemp = cmdVel.angular.z;
    double linearTemp = cmdVel.linear.x;

    if (angularTemp <= MAX_RIGHT_ANGLE)
        angularTemp = MAX_RIGHT_ANGLE;
    if (angularTemp >= MAX_LEFT_ANGLE)
        angularTemp = MAX_LEFT_ANGLE;

    steeringAngle.value = (short)angularTemp;

    if (linearTemp >= MAX_ROBOT_SPEED)
        linearTemp = MAX_ROBOT_SPEED;
    if (linearTemp <= MIN_ROBOT_SPEED)
        linearTemp = MIN_ROBOT_SPEED;

    motorSpeed.value = (short)linearTemp;
}

int main(int argc, char **argv) {
    i2cFile = openI2C();

    if (i2cFile < 0) 
    {
        printf("Unable to open I2C\n");
        return -1;
    } 
    else 
    {
        printf("I2C is Connected\n");
    }

    ros::init(argc, argv, "car_control_node");
    ros::NodeHandle nh;

    std::string cmdVelTopic = "/cmd_vel";

    ros::Subscriber subCarControl = nh.subscribe(cmdVelTopic, 20, cmdCallback);

    ros::Rate loopRate(20);

    while (ros::ok()) 
    {
        protocolData[0] = '#';
        protocolData[1] = 'C';
        protocolData[2] = steeringAngle.bytes[0];
        protocolData[3] = steeringAngle.bytes[1];
        protocolData[4] = motorSpeed.bytes[0];
        protocolData[5] = motorSpeed.bytes[1];
        protocolData[6] = 0;
        protocolData[7] = 0;
        protocolData[8] = '*';

        write(i2cFile, protocolData, 9);

        printf("Robot Speed.data: %d\n", motorSpeed.value);
        printf("Robot Angle.data: %d \n\n", steeringAngle.value);

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
