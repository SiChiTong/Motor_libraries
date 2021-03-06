#include "nav_converter/nav_converter_subscriber.h"

/*
*  Constructor.
*  @param[in] nh ros::NodeHandle
*  @param[in] ros_topic topic for ros messages
*  @param[in] subscribe_queue_size buffer size for ros messages
*/
nav_converter::Subscriber::Subscriber(ros::NodeHandle & nh, const std::string & ros_topic, int subscribe_queue_size)
 : nh_(nh), topic_name_(ros_topic), m_subscribe_queue_size_(subscribe_queue_size), publish_time(ros::Time(0))
{
   subscriberCmd_velTopics();
}

/*
* Destructor.
*/
nav_converter::Subscriber::~Subscriber()
{

}

/* Initialization
* @param[in] schedule_publish_delay plc publish message are scheduled to be published 0.5 milliseconds after received
*/
void nav_converter::Subscriber::init(double schedule_publish_delay)
{
   schedule_publish_delay_ = ros::Duration(schedule_publish_delay);
   publish_time = ros::Time(0);
}

/*
* @brief subscrier to cmd_vel topics for speed_wheel msg and set callback to handle message 
*/
bool nav_converter::Subscriber::subscriberCmd_velTopics(void)
{
   control_wheel = nh_.subscribe(topic_name_, m_subscribe_queue_size_, &nav_converter::Subscriber::cmd_velCallback, this);
}

/*
* @brief returns true, if publishing of a MLS measurement is scheduled and time has been reached for publishing the current MLS measurement.
*/
bool nav_converter::Subscriber::isCmd_velTriggered(void)
{
   boost::lock_guard<boost::mutex> schedule_lockguard(pulish_mutex);
   return !publish_time.isZero() && ros::Time::now() > publish_time;
}

/*
* @brief schedules the publishing of the goal->action of do stuff.
* @param[in] schedule if true, publishing is scheduled, otherwise a possibly pending schedule is removed.
*/
void nav_converter::Subscriber::schedulePublishControlWheel(bool schedule)
{
   boost::lock_guard<boost::mutex> schedule_lockguard(pulish_mutex);
   if(schedule && publish_time.isZero())
   {
      publish_time = ros::Time::now() + schedule_publish_delay_;
   }
   if(!schedule && !publish_time.isZero())
   {
      publish_time = ros::Time(0);
   }
}

/*
* @brief schedules the current speed_wheel message for publishing.
*/
void nav_converter::Subscriber::publishTriggerControlWheel(void)
{
   schedulePublishControlWheel(true);
}

/*
* Callbacks for ros messages from cmd_vel topic. 
*/
void nav_converter::Subscriber::cmd_velCallback(const geometry_msgs::TwistPtr & msg)
{
   cmdVel_msg.linear.x = msg->linear.x;
   cmdVel_msg.angular.z = msg->angular.z;
   publishTriggerControlWheel();
}