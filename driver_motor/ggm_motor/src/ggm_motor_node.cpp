#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <boost/thread/thread.hpp>
#include "ggm_motor/ggm_motor_md200.h"
#include "ggm_motor/ggm_motor_diagnostic.h"
#include "ggm_motor/ggm_motor_subscriber.h"
#include "nav_converter/speed_wheel.h"

/* define struct */
typedef struct device{
   char _port[30];
   int _baud;
   int _ID[];
}device_t;

/* Global varlue*/
uint8_t rate = 20; // Hz
device_t dv;
ros::NodeHandlePtr nh;
boost::thread Thread;
GGM::Driver_md200 *md200 = NULL;
GGM::Subscriber *p_ggm_motor_subscriber = NULL;
/**********************************************************************
* Define Function 
***********************************************************************/
bool loadParam(ros::NodeHandlePtr nh, std::string node_name,device_t *dv);
void doStuff(uint8_t *publish_rate);
/**********************************************************************
* MAIN 
***********************************************************************/
int main(int argc,char **argv)
{
   /* create ros ndoe */
   ros::init(argc, argv, "GGM");
   nh = boost::make_shared<ros::NodeHandle>();
   std::string node_name = ros::this_node::getName();
   ROS_INFO("%s.cpp-node_name: %s", node_name.c_str(), node_name.c_str());
	if(loadParam(nh, node_name, &dv)){
		ROS_INFO("GGM_node.cpp-Load parameter successfull");
	}else{
		ROS_ERROR("GGM_node.cpp.cpp-Error when load parameter");
	}
   /* connect with the server */
   std::string diagnostic_topic = "diagnostics";
   GGM::Diagnostic::init(*nh, diagnostic_topic, "GGM MD200");
   /* Subscriber */
   p_ggm_motor_subscriber = new GGM::Subscriber(*nh, "control_wheel",1,2);
   
   if(Thread.joinable() == false) Thread =  boost::thread(doStuff,&rate);
   if(Thread.joinable() == true) Thread.join();
   return 0;
}

/**********************************************************************
* Function detail 
***********************************************************************/
void doStuff(uint8_t *publish_rate)
{
   ros::Rate loop_rate(*publish_rate); 
   p_ggm_motor_subscriber->init();
   while (ros::ok())
   {
      md200 = new GGM::Driver_md200(nh, dv._port,dv._baud, PARITY_NONE, DATABIT_8, STOPBIT_1);
      while(ros::ok() && md200->_connected)
      {
         if(p_ggm_motor_subscriber->isSpeed_wheelTriggered())
         {
            md200->publish_speed(p_ggm_motor_subscriber->speed);
            p_ggm_motor_subscriber->schedulePublishControlWheel(false);
         }
         if(p_ggm_motor_subscriber->ispublishTriggeredOverTime())
         {
            p_ggm_motor_subscriber->speed[0] = p_ggm_motor_subscriber->speed[1] = 0;
            p_ggm_motor_subscriber->publishTriggerControlWheel();
         }
         loop_rate.sleep();
         ros::spinOnce();
      }
   }
   md200->close_port();
   delete(md200);
   delete(p_ggm_motor_subscriber);
   ros::spin();  
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