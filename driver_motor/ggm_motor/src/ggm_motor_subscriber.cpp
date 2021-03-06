#include "ggm_motor/ggm_motor_subscriber.h"

/*
 *  Constructor.
 *  @param[in] nh ros::NodeHandle
 *  @param[in] ros_topic topic for ros messages
 *  @param[in] subscribe_queue_size buffer size for ros messages
 */
GGM::Subscriber::Subscriber(ros::NodeHandle & nh, const std::string & ros_topic, int subscribe_queue_size, double max_publish_delay)
 : nh_(nh), topic_name_(ros_topic), m_subscribe_queue_size_(subscribe_queue_size), max_publish_delay_(ros::Duration(max_publish_delay)),
   publish_time(ros::Time(0)), publish_time_lastest(ros::Time(0))
{
   subscriberTopics();
}

/*
 * Destructor.
 */
GGM::Subscriber::~Subscriber()
{

}

/* Initialization
 * @param[in] schedule_publish_delay plc publish message are scheduled to be published 0.5 milliseconds after received
 */
void GGM::Subscriber::init(double schedule_publish_delay)
{
   schedule_publish_delay_ = ros::Duration(schedule_publish_delay);
   publish_time = ros::Time(0);
}

/*
 * @brief subscrier to cmd_vel topics for speed_wheel msg and set callback to handle message 
 */
bool GGM::Subscriber::subscriberTopics(void)
{
   control_wheel = nh_.subscribe(topic_name_, m_subscribe_queue_size_, &GGM::Subscriber::cmd_velCallback, this);
}

/*
 * @brief returns true, if publishing of a MLS measurement is scheduled and time has been reached for publishing the current MLS measurement.
 */
bool GGM::Subscriber::isSpeed_wheelTriggered(void)
{
   boost::lock_guard<boost::mutex> schedule_lockguard(pulish_mutex);
   return !publish_time.isZero() && ros::Time::now() > publish_time;
}

/* 
 * @brief  cmd_vel do not publish. if over wait_time then motor will stop
 * @param  wait_time Time for checking 
 */
bool GGM::Subscriber::ispublishTriggeredOverTime(void)
{
   boost::lock_guard<boost::mutex> schedule_lockguard(pulish_mutex);
   return (ros::Time::now() - publish_time_lastest) > max_publish_delay_;
}

/*
 * @brief schedules the publishing of the goal->action of do stuff.
 * @param[in] schedule if true, publishing is scheduled, otherwise a possibly pending schedule is removed.
 */
void GGM::Subscriber::schedulePublishControlWheel(bool schedule)
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
void GGM::Subscriber::publishTriggerControlWheel(void)
{
   schedulePublishControlWheel(true);
}

/*
* @brief Callbacks for ros messages from cmd_vel topic. 
*/
void GGM::Subscriber::cmd_velCallback(const nav_converter::speed_wheelPtr & msg)
{
	speed[0] = msg->wheel_letf;
  	speed[1] = msg->wheel_right;
   publishTriggerControlWheel();
   publish_time_lastest = ros::Time::now();
}