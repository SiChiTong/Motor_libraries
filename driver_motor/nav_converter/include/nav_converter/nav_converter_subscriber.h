/**
* by HiepLM
*/
#ifndef __NAV_CONVERTER_SUBSCRIBER_H_INCLUDED
#define __NAV_CONVERTER_SUBSCRIBER_H_INCLUDED

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/Twist.h>
#include "libraryparam/agvparam.h"

namespace nav_converter
{
   class Subscriber
   {
   public:
      /*
       *  Constructor.
       *  @param[in] nh ros::NodeHandle
       *  @param[in] ros_topic topic for ros messages
       *  @param[in] subscribe_queue_size buffer size for ros messages
       */
      Subscriber(ros::NodeHandle & nh, const std::string & ros_topic, int subscribe_queue_size = 1);

      /*
       * Destructor.
       */
      virtual ~Subscriber();
      
      /* Initialization
       * @param[in] schedule_publish_delay plc publish message are scheduled to be published 0.5 milliseconds after received
       */
      void init(double schedule_publish_delay = 0.0005);

      /*
       * @brief subscrier to cmd_vel topics for speed_wheel msg and set callback to handle message 
       */
      virtual bool subscriberCmd_velTopics(void);

      /*
       * @brief returns true, if publishing of a MLS measurement is scheduled and time has been reached for publishing the current MLS measurement.
       */
      virtual bool isCmd_velTriggered(void);

      /*
       * @brief schedules the publishing of the goal->action of do stuff.
       * @param[in] schedule if true, publishing is scheduled, otherwise a possibly pending schedule is removed.
       */
      virtual void schedulePublishControlWheel(bool schedule);
     
      /*
       * @brief schedules the current speed_wheel message for publishing.
       */
      virtual void publishTriggerControlWheel(void); 

      cmd_vel cmdVel_msg;

   private:
      /*
       * Callbacks for ros messages from cmd_vel topic. 
       */
      virtual void cmd_velCallback(const geometry_msgs::TwistPtr & msg);

      ros::NodeHandle nh_;
      std::string topic_name_;
      int m_subscribe_queue_size_;
      boost::mutex pulish_mutex;
      ros::Time publish_time;
      ros::Duration schedule_publish_delay_;
      ros::Subscriber control_wheel;
   };
}
#endif