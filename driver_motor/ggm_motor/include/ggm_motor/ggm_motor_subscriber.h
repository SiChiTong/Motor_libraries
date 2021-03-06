/**
* by HiepLM
*/
#ifndef __GGM_MOTOR_SUBSCRIBER_H_INCLUDED
#define __GGM_MOTOR_SUBSCRIBER_H_INCLUDED

#include <ros/ros.h>
#include <boost/thread.hpp>
#include "libraryparam/agvparam.h"
#include "nav_converter/speed_wheel.h"

namespace GGM
{
   class Subscriber
   {
   public:
      /*
       *  @brief Constructor.
       *  @param[in] nh ros::NodeHandle
       *  @param[in] ros_topic topic for ros messages
       *  @param[in] subscribe_queue_size buffer size for ros messages
       *  @param[in] wait_time Time for checking to motor stop (1 second)
       */
      Subscriber(ros::NodeHandle & nh, const std::string & ros_topic, int subscribe_queue_size = 1, double max_publish_delay = 1);

      /*
       * @brief Destructor.
       */
      virtual ~Subscriber();
      
      /* @brief Initialization
       * @param[in] schedule_publish_delay plc publish message are scheduled to be published 0.5 milliseconds after received
       */
      void init(double schedule_publish_delay = 0.0005);

      /*
       * @brief subscrier to cmd_vel topics for speed_wheel msg and set callback to handle message 
       */
      virtual bool subscriberTopics(void);

      /*
       * @brief returns true, if publishing of a speed_wheel is scheduled and time has been reached for publishing the currentspeed_wheel.
       */
      virtual bool isSpeed_wheelTriggered(void);

      /* 
       * @brief  cmd_vel do not publish. if over wait_time then motor will stop
       * @param  wait_time Time for checking 
       */
      virtual bool ispublishTriggeredOverTime(void);

      /*
       * @brief schedules the publishing of the goal->action of do stuff.
       * @param[in] schedule if true, publishing is scheduled, otherwise a possibly pending schedule is removed.
       */
      virtual void schedulePublishControlWheel(bool schedule);
     
      /*
       * @brief schedules the current speed_wheel message for publishing.
       */
      virtual void publishTriggerControlWheel(void); 

      int16_t speed[2] = {0,0};

   protected:
      /*
       * @brief Callbacks for ros messages from cmd_vel topic. 
       */
      virtual void cmd_velCallback(const nav_converter::speed_wheelPtr & msg);

      ros::NodeHandle nh_;
      std::string topic_name_;
      int m_subscribe_queue_size_;
      boost::mutex pulish_mutex;             // llock guard to schedule publishing measurements using speed_wheel
      ros::Time publish_time;                // time to publish next speed_wheel message
      ros::Time publish_time_lastest;
      ros::Duration schedule_publish_delay_; // message are scheduled to be published 5 milliseconds
      ros::Duration max_publish_delay_; 
      ros::Subscriber control_wheel;         // To subcriber speed_wheel message
                                  
   };
}
#endif