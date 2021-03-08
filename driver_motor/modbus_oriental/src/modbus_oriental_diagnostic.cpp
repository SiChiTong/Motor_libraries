#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include "modbus_oriental/modbus_oriental_diagnostic.h"


/**
 * g_diagnostic_handler: singleton, implements the diagnostics for sick_line_guidance
 */
ORIENTAL_MOTOR::Diagnostic::DiagnosticImpl ORIENTAL_MOTOR::Diagnostic::g_diagnostic_handler;


/*
 * Constructor.
 */
ORIENTAL_MOTOR::Diagnostic::DiagnosticImpl::DiagnosticImpl() : m_diagnostic_initialized(false), m_diagnostic_component("")
{
}

/*
 * Initialization.
 *
 * @param[in] nh ros::NodeHandle
 * @param[in] publish_topic ros topic to publish diagnostic messages
 * @param[in] component description of the component reporting
 */
void ORIENTAL_MOTOR::Diagnostic::DiagnosticImpl::init(ros::NodeHandle &nh, const std::string & publish_topic, const std::string & component)
{
  m_diagnostic_publisher = nh.advertise<diagnostic_msgs::DiagnosticArray>(publish_topic, 1);
  m_diagnostic_component = component;
  m_diagnostic_initialized = true;
}

/*
 * Updates and reports the current status.
 *
 * @param[in] status current status (OK, ERROR, ...)
 * @param[in] message optional diagnostic message
 */
void ORIENTAL_MOTOR::Diagnostic::DiagnosticImpl::update(DIAGNOSTIC_STATUS status, const std::string &message)
{
  if (m_diagnostic_initialized)
  {
    static std::map<DIAGNOSTIC_STATUS, std::string> status_description = {
      {DIAGNOSTIC_STATUS::OK, "OK"},
      {DIAGNOSTIC_STATUS::EXIT, "EXIT"},
      {DIAGNOSTIC_STATUS::ERROR_STATUS, "ERROR_STATUS"},
      {DIAGNOSTIC_STATUS::CONFIGURATION_ERROR, "CONFIGURATION_ERROR"},
      {DIAGNOSTIC_STATUS::INITIALIZATION_ERROR, "INITIALIZATION_ERROR"},
      {DIAGNOSTIC_STATUS::INTERNAL_ERROR, "INTERNAL_ERROR"}
    };
  
    // create DiagnosticStatus
    diagnostic_msgs::DiagnosticStatus msg;
    msg.level = (status == DIAGNOSTIC_STATUS::OK) ? (diagnostic_msgs::DiagnosticStatus::OK) : (diagnostic_msgs::DiagnosticStatus::ERROR); // Level of operation
    msg.name = m_diagnostic_component; // description of the test/component reporting
    msg.hardware_id = "";  // hardware unique string (tbd)
    msg.values.clear();    // array of values associated with the status
    // description of the status
    msg.message = status_description[status];
    if(msg.message.empty())
    {
      msg.message = "ERROR";
    }
    if (!message.empty())
    {
      msg.message = msg.message + ": " + message;
    }
    // publish DiagnosticStatus in DiagnosticArray
    diagnostic_msgs::DiagnosticArray msg_array;
    msg_array.header.stamp = ros::Time::now();
    msg_array.header.frame_id = "";
    msg_array.status.clear();
    msg_array.status.push_back(msg);
    m_diagnostic_publisher.publish(msg_array);
  }
}