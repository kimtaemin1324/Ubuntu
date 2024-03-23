#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "nav_2d_msgs/Point2D.h"
#include "sensor_msgs/Range.h"


enum CONTROL
{  
    VISION_CONTROL = 0,        
    YAW_CONTROL    = 1,
    SONAR_CONTROL  = 2,
    LIDAR_CONTROL  = 3
};
//#define VISION_CONTROL  0
//#define YAW_CONTROL     1
//#define SONAR_CONTROL   2
//#define LIDAR_CONTROL   3

geometry_msgs::Twist cmd_vel_line;
geometry_msgs::Twist cmd_vel_yaw;
geometry_msgs::Twist cmd_vel_sonar;
geometry_msgs::Twist type_cmd_vel;



int cmd_vel_rqt_steering = 0;



void VISION_Callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd_vel_line =  *msg;
}

void YAW_Callback(const geometry_msgs::Twist::ConstPtr &msg)
{
   cmd_vel_yaw =   *msg;
}

void SONAR_Callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd_vel_sonar = *msg;
}

void Type_Callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    type_cmd_vel =  *msg;
}


int main(int argc, char **argv)
{
	enum CONTROL control;
    int count = 0;
    
    ros::init(argc, argv, "cmd_vel_mux_node");
    ros::NodeHandle n;
    
   
    std::string line_topic              = "/cmd_vel/line";
    std::string yaw_topic               = "/cmd_vel/yaw";
    std::string sonar_cmd_vel_topic     = "/cmd_vel/sonar";
    std::string type_cmd_vel_topic      = "/type_cmd_vel";
    std::string ackermann_cmd_vel_topic = "/ackermann_steering_controller/cmd_vel";
    
    ros::param::get("~line_topic"              ,line_topic);
    ros::param::get("~yaw_topic"               , yaw_topic);
    ros::param::get("~sonar_cmd_vel_topic"     ,sonar_cmd_vel_topic);
    ros::param::get("~type_cmd_vel_topic"      ,type_cmd_vel_topic);
    ros::param::get("~ackermann_cmd_vel_topic" , ackermann_cmd_vel_topic);
    
    //ros::Subscriber line_sub    = n.subscribe(line_topic, 10, VISION_Callback);
    ros::Subscriber yaw_sub     = n.subscribe(yaw_topic, 10, YAW_Callback);
    //ros::Subscriber sonar_sub   = n.subscribe(sonar_cmd_vel_topic, 10, SONAR_Callback);
    ros::Subscriber type_cmd_vel_sub   = n.subscribe(type_cmd_vel_topic, 10, Type_Callback);
    
      
    ros::Publisher ackermann_cmd_vel_pub = n.advertise<geometry_msgs::Twist>(ackermann_cmd_vel_topic , 1);
    //ros::Publisher type_cmd_vel_pub      = n.advertise<geometry_msgs::Twist>(type_cmd_vel_topic , 1);
    
    
    ros::Rate loop_rate(100.0);

    
    while (ros::ok())
    {
     geometry_msgs::Twist cmd_vel_msg;
   
      switch(cmd_vel_rqt_steering)
      {
         case VISION_CONTROL:
           //type_cmd_vel = cmd_vel_line;
           //printf("type_cmd_vel : %.2lf\n",type_cmd_vel);
           
		   control = VISION_CONTROL;    
		   printf("%d\n", control);
           break;
           
         case YAW_CONTROL:
           //type_cmd_vel = cmd_vel_yaw;
           //printf("type_cmd_vel : %.2lf\n",type_cmd_vel);
           
		   control = YAW_CONTROL;    
		   printf("%d\n", control);
           break;
         
         case SONAR_CONTROL:
         //type_cmd_vel = cmd_vel_sonar;
         //printf("type_cmd_vel : %.2lf\n",type_cmd_vel);
        
		 control = SONAR_CONTROL;    
		 printf("%d\n", control);
           break;

      }
      ackermann_cmd_vel_pub.publish(type_cmd_vel);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
