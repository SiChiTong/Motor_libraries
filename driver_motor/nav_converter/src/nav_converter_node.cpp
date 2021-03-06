#include <ros/ros.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <unistd.h>
#include <XmlRpcValue.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/thread.hpp>
#include "nav_converter/speed_wheel.h"
#include "nav_converter/nav_converter.h"
#include "nav_converter/nav_converter_subscriber.h"

typedef enum EXIT_CODES_ENUM
{
  EXIT_OK = 0,
  EXIT_ERROR = 1,
  EXIT_CONFIG_ERROR 
} EXIT_CODES;
template<class T> static T readMember(XmlRpc::XmlRpcValue & value, const std::string & member, const T & defaultvalue)
{
  if(value.hasMember(member))
    return value[member];
  return defaultvalue;
}
/*global variable */
ros::Publisher Navigation_control;
uint8_t rate = 20; // Hz
navigation_converter *p_navigation_converter = NULL;
nav_converter::Subscriber *p_nav_converter_subscriber = NULL;

/* Create Function Thread */
boost::thread nav_converterThread;
void navgationDoStuff(ros::NodeHandle & nh, uint8_t *publish_rate);

int main(int argc, char **argv)
{
    /* Khoi tao Node */
    ros::init(argc, argv, "nav_converter");
    ros::NodeHandle nh;
    ROS_INFO("nav_converter.cpp::42 -> is ready !!");
    /* Publisher */
    Navigation_control = nh.advertise<nav_converter::speed_wheel>("control_wheel", 20);
    /* Constructors */
    p_navigation_converter = new navigation_converter(nh);
    p_nav_converter_subscriber = new nav_converter::Subscriber(nh,"cmd_vel");
    /*Create thread*/
	if(!nav_converterThread.joinable()) boost::thread(navgationDoStuff,nh, &rate);
    if(nav_converterThread.joinable()) nav_converterThread.join();
	nav_converterThread.join();
    ros::spin();
    delete(p_navigation_converter);
    delete(p_nav_converter_subscriber);
    return EXIT_OK;
}
/**********************************************************
                    Function detail
**********************************************************/

void navgationDoStuff(ros::NodeHandle & nh, uint8_t *publish_rate)
{
    /* Subscriber */
    ROS_INFO("nav_converter.cpp:59 -> navgationDoStuff is ready!!");
    p_nav_converter_subscriber->init();
    ros::Rate loop_rate(*publish_rate);
    nav_converter::speed_wheel motor;
    motor.header.frame_id = "cmd_vel";
    while (ros::ok())
    {
        if(p_nav_converter_subscriber->isCmd_velTriggered())
        {
            speedWheel speedWheel;
            {
                speedWheel = p_navigation_converter->navigationConverter(p_nav_converter_subscriber->cmdVel_msg);
                motor.header.stamp = ros::Time::now();
                motor.wheel_letf = speedWheel.letf;
                motor.wheel_right = speedWheel.right;
            }
            Navigation_control.publish(motor);
            p_nav_converter_subscriber->schedulePublishControlWheel(false);
            ROS_INFO_STREAM("V = " << p_nav_converter_subscriber->cmdVel_msg.linear.x << " W = " << p_nav_converter_subscriber->cmdVel_msg.angular.z 
                                   <<" Banh trai = " << speedWheel.letf << " Banh phai = " << speedWheel.right);
            speedWheel.letf = speedWheel.right = 0;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
}
