#include <ros/ros.h>
#include <boost/thread.hpp>
#include "modbus_oriental/modbus_oriental_lib.h"
#include "modbus_oriental/modbus_oriental_subscriber.h"
#include "modbus_oriental/modbus_oriental_diagnostic.h"

/* define struct */
typedef struct device{
   char _port[30];
   int _baud;
   int _ID[];
}device_t;

/* Globle value */
int rate = 10; // Hz
device_t dv;
ros::NodeHandlePtr nh;
ORIENTAL_MOTOR::modbus_oriental *p_oriental_motor = NULL;
ORIENTAL_MOTOR::Subscriber *p_oriental_subscriber = NULL;
/* thread */
boost::thread Thread ;
void doStuff(ros::NodeHandle & nh, int *publish_rate);
bool loadParam(ros::NodeHandlePtr nh, std::string node_name,device_t *dv);
/**********************************************************************
 *                   		MAIN 
***********************************************************************/
int main(int argc,char **argv)
{
    /* create ros node */
    ros::init(argc, argv, "ORIRENTAL_MOTOR");
    nh = boost::make_shared<ros::NodeHandle>();
    std::string node_name = ros::this_node::getName();
    if(loadParam(nh, node_name, &dv)){
    ROS_INFO("GGM_node.cpp-Load parameter successfull");
	}else{
		ROS_ERROR("GGM_node.cpp.cpp-Error when load parameter");
        return 1;
	}
    /* connect with the server */
    std::string diagnostic_topic = "diagnostics";
    ORIENTAL_MOTOR::Diagnostic::init(*nh, diagnostic_topic, "Oriental Motor");
    /* Subscriber */
    p_oriental_subscriber = new ORIENTAL_MOTOR::Subscriber(*nh, "control_wheel", 1 ,1 /*second*/);
    /* Thread */
    if(Thread.joinable() == false) Thread = boost::thread(doStuff, *nh, &rate);
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
    int ret;
    ros::Rate loop_rate(*publish_rate);
    while(ros::ok())
    {
        p_oriental_motor = new ORIENTAL_MOTOR::modbus_oriental(nh, POLY, dv._port, dv._baud , PARITY_EVEN, DATABIT_8, STOPBIT_1);
        if(p_oriental_motor->_connected)    
        {
            p_oriental_motor->writeSpeedControlMode(MODE_NO_1);
            CHECK_RETSULT(ret);
            p_oriental_motor->writeAcceleration(3);
            CHECK_RETSULT(ret);
            p_oriental_motor->writeDeceleration(3);
            CHECK_RETSULT(ret);
        }
        while (ros::ok() && p_oriental_motor->_connected)  
        {
            ros::Time begin = ros::Time::now();
            if(p_oriental_subscriber->isSpeed_wheelTriggered())
            {
                p_oriental_motor->publish_speed(p_oriental_subscriber->speed);
                CHECK_RETSULT(ret);
                p_oriental_subscriber->schedulePublishControlWheel(false);
            }
            if(p_oriental_subscriber->ispublishTriggeredOverTime())
            {
                p_oriental_subscriber->speed[0] = p_oriental_subscriber->speed[1] = 0;
                p_oriental_subscriber->publishTriggerControlWheel();
            }
            //  ROS_WARN_STREAM("Time_clock[" << ros::Time::now() - begin << "]");
            loop_rate.sleep();
            ros::spinOnce();
        }
    }
    if(ret == BAD_CON ||  ret == EX_BAD_DATA) 
    {
        std::stringstream err;
        err << p_oriental_motor->error_msg;
        ROS_ERROR_STREAM("Driver motor failed: " << err.str());
    }
    p_oriental_motor->close_port();
    delete(p_oriental_motor);
}

/***************************************************************************/
bool loadParam(ros::NodeHandlePtr nh, std::string node_name, device_t *dv)
{
	ROS_INFO("loadParam() - node_name: %s",  node_name.c_str());
	if(!nh->param(node_name + "/baudrate", dv->_baud,dv->_baud))
    {
		return false;
	}
	std::string port_str;    //port name
	if(!nh->param(node_name + "/port", port_str,port_str))
    {
		return false;
    }
    strcpy(dv->_port, port_str.c_str());
	return true;
}
