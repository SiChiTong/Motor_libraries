#ifndef __GGM_MOTOR_DIAGNOTIC_H_INCLUDED
#define __GGM_MOTOR_DIAGNOTIC_H_INCLUDED
#include <ros/ros.h>

namespace GGM 
{
    typedef enum DIAGNOSTIC_STATUS_ENUM
    {
        OK,                       // status okay, no errors
        EXIT,                     // delta exiting
        ERROR_STATUS,             // device signaled an error 
        CONFIGURATION_ERROR,      // invalid configuration, check configuration files
        INITIALIZATION_ERROR,     // initialization of driver failed
        INTERNAL_ERROR            // internal error, should never happen
        
    } DIAGNOSTIC_STATUS;
    /*
    * class Diagnostic publishes diagnostic messages for sick_line_guidance
    */
    class Diagnostic
    {
    public:
        /*
        * Initializes the global diagnostic handler.
        *
        * @param[in] nh ros::NodeHandle
        * @param[in] publish_topic ros topic to publish diagnostic messages
        * @param[in] component description of the component reporting
        */
        static void init(ros::NodeHandle &nh, const std::string & publish_topic, const std::string & component)
        {
            g_diagnostic_handler.init(nh, publish_topic, component);
        }
        
        /*
        * Updates and reports the current status.
        *
        * @param[in] status current status (OK, ERROR, ...)
        * @param[in] message optional diagnostic message
        */
        static void update(DIAGNOSTIC_STATUS status, const std::string & message = "")
        {
            g_diagnostic_handler.update(status, message);
        }
    
    protected:
        /*
        * class DiagnosticImpl implements diagnostics for sick_line_guidance
        */
        class DiagnosticImpl
        {
        public:
    
            /*
            * Constructor.
            */
            DiagnosticImpl();
            
            /*
            * Initialization.
            *
            * @param[in] nh ros::NodeHandle
            * @param[in] publish_topic ros topic to publish diagnostic messages
            * @param[in] component description of the component reporting
            */
            void init(ros::NodeHandle & nh, const std::string & publish_topic, const std::string & component);
            
            /*
            * Updates and reports the current status.
            *
            * @param[in] status current status (OK, ERROR, ...)
            * @param[in] message optional diagnostic message
            */
            void update(DIAGNOSTIC_STATUS status, const std::string & message = "");
            
        protected:
            
            /*
            * member data.
            */
            
            bool m_diagnostic_initialized;         // flag indicating proper initialization of diagnostics
            ros::Publisher m_diagnostic_publisher; // publishes diagnostic messages
            std::string m_diagnostic_component;    // name of the component publishing diagnostic messages
        
        }; // class DiagnosticImpl
        
        
        static DiagnosticImpl g_diagnostic_handler; // singleton, implements the diagnostics for sick_line_guidance
        
  }; // class Diagnostic
}
#endif