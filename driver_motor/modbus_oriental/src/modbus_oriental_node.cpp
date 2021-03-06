#include <ros/ros.h>
#include <boost/thread.hpp>
#include "modbus_oriental/modbus_oriental_lib.h"

/* Globle value */
int rate = 20; // Hz
ORIENTAL_MOTOR::modbus_oriental *p_oriental_motor = NULL;

/* thread*/
boost::thread Thread ;
void doStuff(ros::NodeHandle & nh, int *publish_rate);
/**********************************************************************
 *                   		MAIN 
***********************************************************************/
int main(int argc,char **argv)
{
    /* create ros node */
    ros::init(argc, argv, "ORIRENTAL_MOTOR");
    ros::NodeHandle nh;
    p_oriental_motor = new ORIENTAL_MOTOR::modbus_oriental(POLY,"/dev/ttyS4",115200,PARITY_EVEN,DATABIT_8,STOPBIT_1);
    if(p_oriental_motor->_connected)
    {
        if(Thread.joinable() == false) Thread = boost::thread(doStuff,nh,&rate);
    } else 
    {
        ROS_INFO("Oriental Motor Exit !!");
        return 1;
    }
    if(Thread.joinable() == true)
        Thread.join();
    ros::spin();
    return 0;
}
/**********************************************************************
 *                   		Function detail 
***********************************************************************/
void doStuff(ros::NodeHandle & nh, int *publish_rate)
{
    uint32_t speed[2] = {0,0};
    ros::Rate loop_rate(*publish_rate);
    p_oriental_motor->writeSpeedControlMode(2,0x0001);
    p_oriental_motor->writeAcceleration(2,3);
	p_oriental_motor->writeDeceleration(2,3);
    while (ros::ok() && p_oriental_motor->_connected)  
    {
        ros::Time begin = ros::Time::now();
        //p_oriental_motor->writeForward(1);
        p_oriental_motor->writeReverse(2);
        //p_oriental_motor->writeSpeed(1,500);
        p_oriental_motor->writeSpeed(2,500);
        //p_oriental_motor->feedbackSpeed(1, &speed[0]);
        p_oriental_motor->feedbackSpeed(2, &speed[1]);
        ROS_INFO_STREAM("Time_clock[" << ros::Time::now() - begin << "]");
        ROS_INFO("Speed[0] = %d  Speed[1] = %d", (int32_t)speed[0], (int32_t)speed[1]);
        loop_rate.sleep();
        ros::spinOnce();
    }
    p_oriental_motor->close_port();
    delete(p_oriental_motor);
}