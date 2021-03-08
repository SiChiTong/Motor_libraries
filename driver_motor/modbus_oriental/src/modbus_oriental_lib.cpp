#include "modbus_oriental/modbus_oriental_lib.h"


/*
 * @brief Constructor 
 * @param[in] poly      The poly number to caculartion CRC16
 * @param[in] devfile   Example /dev/tty* 
 * @param[in] baud      Number of transmitted bits  per a second
 * @param[in] parity    The parity check bit {Even, None , Old }
 * @param[in] data_bit  Number of bits in a transmission frame 
 * @param[in] stop_bit  End bit
 */
ORIENTAL_MOTOR::modbus_oriental::modbus_oriental(ros::NodeHandle & nh, uint16_t poly , const char* devfile, unsigned int baud, 
                            parity_t parity, data_bits_t data_bit,stop_bits_t stop_bit)
 : nh_(nh), mosbus_rtu(poly, devfile, baud, parity, data_bit, stop_bit)
{
   if(_connected)
   {
        if(!nh_.getParam("drivers", drivers) || drivers.size() < 1)
        {
            ROS_ERROR("modbus_oriental_node: no driver found in yaml-file, please check configuration. Aborting...");
            ORIENTAL_MOTOR::Diagnostic::update(ORIENTAL_MOTOR::DIAGNOSTIC_STATUS::CONFIGURATION_ERROR, " No driver found in yaml-file");
            exit(-1);
        }
        else loadParam();

        if(!check_baudrate(baud))
        {   
            std::stringstream msg;
            msg << "DRIVER not support baudrate" << baud << "bps";
            ORIENTAL_MOTOR::Diagnostic::update(ORIENTAL_MOTOR::DIAGNOSTIC_STATUS::CONFIGURATION_ERROR, msg.str());
            ROS_ERROR_STREAM("modbus_oriental_node: " << msg.str());
            this->close_port();
            exit(-1);
        }
        else
        {
            std::stringstream msg;
            msg << " Connected "<<devfile<<" with "<<baud<<" bps";
            ROS_INFO_STREAM("modbus_oriental_node: " << msg.str());
            ros::Duration(0.7).sleep(); // sleep for 0.5 second
            ORIENTAL_MOTOR::Diagnostic::update(ORIENTAL_MOTOR::DIAGNOSTIC_STATUS::OK, msg.str());
        }
   }
   else
   {
        ros::Duration(2).sleep();
        std::stringstream err;
        err << error_msg;
        ROS_ERROR_STREAM(error_msg);
        ORIENTAL_MOTOR::Diagnostic::update(ORIENTAL_MOTOR::DIAGNOSTIC_STATUS::INITIALIZATION_ERROR, error_msg);
   }
}

/*
 * Destructor.
 */
ORIENTAL_MOTOR::modbus_oriental::~modbus_oriental()
{

}

/**
 * Get parameter from yaml file
 */
void ORIENTAL_MOTOR::modbus_oriental::loadParam(void)
{
    int node_id[drivers.size()];
    int count = 0;

    for(XmlRpc::XmlRpcValue::iterator _iter = drivers.begin(); _iter != drivers.end(); ++_iter)
    {   
        std::string driver_name = readMember<std::string>(_iter->second, "name", _iter->first);
        if(_iter->second.hasMember("id"))
        {
            node_id[count] = _iter->second["id"];
            ROS_INFO("GGM_motor: initializing driver_name \"%s\", id \"0x0%d\"...", driver_name.c_str(),node_id[count]);
            count++;
        } else
        {
            ROS_ERROR_STREAM("Node '" << _iter->first  << "' has no id");
            ORIENTAL_MOTOR::Diagnostic::update(ORIENTAL_MOTOR::DIAGNOSTIC_STATUS::CONFIGURATION_ERROR, " Node name has no id");
        } 
    }
    set_slave_id(node_id,count);
}

/*
 * Set slave id 
 * @param ids       Pointer save array slave id  
 * @param lenght    Lenght of ids array (sizeof(ids)/sizeof(uint8_t))
 */
void ORIENTAL_MOTOR::modbus_oriental::set_slave_id(int *ids, uint8_t lenght)
{
   uint8_t n = lenght; 
   for(uint8_t i = 0; i < n; i++)
   {
       for(int j = 0; j < n; j++)
       {
           if(ids[j] == ids[i] && j != i)
           {
               ids[j] = ids[j+1];
               n--;
           }
       }
   }
   if(n != lenght)  ROS_WARN("There are %d ID input to be same!!", lenght - n);
   slave_id.size = n;
   for(uint8_t k = 0; k < n; k++) slave_id.adr[k] = ids[k];
   ROS_INFO("The slave ID get OK");
}

/*
 * Check baudrate has supported for device
 * @param baud      Package Array size byte
 * @param return    true/false 
 */
bool ORIENTAL_MOTOR::modbus_oriental::check_baudrate(unsigned int baud)
{
    bool status = false;
    unsigned int data_defaut[] = {9600, 19200, 38400, 57600, 115200};
    for(uint8_t i = 0; i <  (uint8_t)sizeof(data_defaut)/sizeof(unsigned int); i++)
    {
        if(baud == data_defaut[i])
        {
            status = true;
            break;
        } 
    }
    return status;
}


/*
 * Delete devfile and close
 */
void ORIENTAL_MOTOR::modbus_oriental::closeDevice(void)
{
    this->close_port();
}

/*
 *  MB-FREE, -, STOP-MODE, REV, FWD, M1, M2, M0
 */
uint16_t ORIENTAL_MOTOR::modbus_oriental::createMotorControl16bit(uint8_t motorDirection, bool freeLockOnStop, bool slowChange, uint8_t motorDataNum) {
    uint16_t bits = 0x0000;
    switch (motorDirection) 
    {
    case MOTOR_DIRECTOIN_STOP:
        bits |= MOTOR_STOP_BIT;
        break;
    case MOTOR_DIRECTOIN_REVERSE:
        bits |= MOTOR_REVERSE_BIT;
        break;
    case MOTOR_DIRECTOIN_FORWARD:
        bits |= MOTOR_FORWARD_BIT;
        break;
    }
    if (freeLockOnStop) {
        bits |= MOTOR_FREE_ON_STOP_BIT;
    }
    if (slowChange) 
    {
        bits |= MOTOR_SLOW_CHANGE_BIT;
    }
    if (motorDataNum != 0 && motorDataNum < 0b1000) 
    {
        bits |= motorDataNum;
    }
  return bits;
}

/*
 *
 */
int ORIENTAL_MOTOR::modbus_oriental::writeStop(uint8_t slave_id)
{
    return modbus_write_register(slave_id, R_DRIVER_INPUT_COMMAND_LOWER, createMotorControl16bit(MOTOR_DIRECTOIN_STOP,true,false));
}

/*
 *
 */
int ORIENTAL_MOTOR::modbus_oriental::writeForward(uint8_t slave_id)
{
    return modbus_write_register(slave_id, R_DRIVER_INPUT_COMMAND_LOWER, createMotorControl16bit(MOTOR_DIRECTOIN_FORWARD,true,false));
}

/*
 *
 */
int ORIENTAL_MOTOR::modbus_oriental::writeReverse(uint8_t slave_id)
{
    ROS_WARN("1");
    return modbus_write_register(slave_id, R_DRIVER_INPUT_COMMAND_LOWER, createMotorControl16bit(MOTOR_DIRECTOIN_REVERSE,true,false));
}

/*
 * 
 */
int ORIENTAL_MOTOR::modbus_oriental::writeConfigTrigger(uint8_t slave_id) 
{
	uint8_t result = writeStop(slave_id);
	if (result != 0) return result;
	return modbus_write_register(slave_id, RW_CONFIGURATION_LOWER, 1);
}

/*
 *
 */
int ORIENTAL_MOTOR::modbus_oriental::writeSpeed(uint8_t slave_id, uint16_t speed)
{
    return modbus_write_register(slave_id, RW_ROTATION_SPEED_NO_0_LOWER, speed);
}

/*
 *
 */
int ORIENTAL_MOTOR::modbus_oriental::writeSpeedControlMode(uint8_t slave_id, uint16_t mode)
{
    uint8_t result;
	result = modbus_write_register(slave_id,RW_ANALOG_INPUT_SIGNAL_SELECTION_LOWER, mode);
	if (result != 0) 
	{
		return result;
	}
	return writeConfigTrigger(slave_id);
}

/*
 *
 */
int ORIENTAL_MOTOR::modbus_oriental::writeResetAlarm(uint8_t slave_id)
{
    return modbus_write_register(slave_id, RW_RESET_ALARM_LOWER, 0x01);
}

/*
 *
 */
int ORIENTAL_MOTOR::modbus_oriental::clearAlarmRecords(uint8_t slave_id)
{
    return modbus_write_register(slave_id, RW_CLEAR_ALARM_HISTORY_LOWER, 0x01);
}

/*
 *
 */
int ORIENTAL_MOTOR::modbus_oriental::clearWarningRecords(uint8_t slave_id)
{
    return modbus_write_register(slave_id, RW_CLEAR_WARNING_HISTORY_LOWER, 0x01);
}

/*
 *
 */
int ORIENTAL_MOTOR::modbus_oriental::resetAndClearAll(uint8_t slave_id)
{
    uint16_t value_in[] = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01}; 
    return modbus_write_registers(slave_id, RW_RESET_ALARM_UPPER, 8, value_in);
}

/*
 *
 */
int ORIENTAL_MOTOR::modbus_oriental::writeAcceleration(uint8_t slave_id, uint16_t time)
{
    return modbus_write_register(slave_id, RW_ACCELERATION_TIME_NO_0_LOWER, time);
}

/*
 *
 */
int ORIENTAL_MOTOR::modbus_oriental::writeDeceleration(uint8_t slave_id, uint16_t time)
{
    return modbus_write_register(slave_id, RW_DECELERATION_TIME_NO_0_LOWER, time);
}

/*
 *
 */
int ORIENTAL_MOTOR::modbus_oriental::writeTorqueLimit(uint8_t slave_id,uint16_t torque)
{
    return modbus_write_register(slave_id, RW_TORQUE_LIMITING_NO_0_LOWER, torque);
}

/*
 *
 */
int ORIENTAL_MOTOR::modbus_oriental::readDirection(uint8_t slave_id,bool *forwarding, bool *reversing, bool *freeLockOnStop)
{
    uint16_t data;
	uint8_t result;
    result = modbus_read_holding_registers(slave_id,R_DRIVER_INPUT_COMMAND_LOWER, 1, &data);
	if (result != 0) 
	{
		return result;
	}
	*forwarding = (MOTOR_FORWARD_BIT & data) != 0x00;
	*reversing = (MOTOR_REVERSE_BIT & data) != 0x00;
	*freeLockOnStop = (MOTOR_FREE_ON_STOP_BIT & data) != 0x00;
	return 0;   
}

/*
 *
 */
int ORIENTAL_MOTOR::modbus_oriental::readRotationSpeed(uint8_t slave_id,uint32_t *speed)
{
    int result;
    uint16_t data_out[2] = {0, 0};
    result = modbus_read_holding_registers(slave_id, RW_ROTATION_SPEED_NO_0_UPPER, 2, data_out);
    speed[0] = (uint32_t)((data_out[0] << 8u) | (data_out[1] & 0x00FFu));
    return result;
}

/*
 *
 */
int ORIENTAL_MOTOR::modbus_oriental::readSpeedControlMode(uint8_t slave_id,uint16_t *mode)
{
    return modbus_read_holding_registers(slave_id, RW_ANALOG_INPUT_SIGNAL_SELECTION_LOWER, 1, mode);
}

/*
 *
 */
int ORIENTAL_MOTOR::modbus_oriental::readAlarm(uint8_t slave_id,uint16_t *alarm)
{
    return modbus_read_holding_registers(slave_id, R_PRESENT_ALARM_LOWER, 1, alarm);
}

/*
 *
 */
int ORIENTAL_MOTOR::modbus_oriental::readWarning(uint8_t slave_id,uint16_t *warning)
{
    return modbus_read_holding_registers(slave_id, R_WARNING_HISTORY_1_LOWER, 1, warning);
}

/*
 *
 */
int ORIENTAL_MOTOR::modbus_oriental::readTorqueLimit(uint8_t slave_id,uint16_t *torque)
{
    return modbus_read_holding_registers(slave_id, RW_TORQUE_LIMITING_NO_0_LOWER, 1, torque);
}


/*
 *
 */
int ORIENTAL_MOTOR::modbus_oriental::feedbackSpeed(uint8_t slave_id, uint32_t *speed)
{
    int result;
    uint16_t data_out[2] = {0, 0};
    result = modbus_read_holding_registers(slave_id, R_FEEDBACK_SPEED_UPPER, 2, data_out);
    speed[0] = (uint32_t)((data_out[0] << 16u) | (data_out[1] & 0x0000FFFFu));
    return result;
}

/*
 *
 */
int ORIENTAL_MOTOR::modbus_oriental::feedbackSpeed(uint8_t slave_id, uint16_t *speed)
{
    return modbus_read_holding_registers(slave_id, R_FEEDBACK_SPEED_LOWER, 1, speed);
}

/*
 * Subcrible speed_wheel to controll motor
 * @param[in] speed The speed of left motor and rigth motor
 */
int ORIENTAL_MOTOR::modbus_oriental::publish_speed(uint16_t *speed)
{
    int ret;
    if(slave_id.size > 0)
    {
        for(uint8_t i = 0; i < slave_id.size; i++)
        {
            if(abs(speed[i]) > 0)
            {
                if(speed[i] > 0 && dir[i] != forward) {         // Trigger
                    ROS_ERROR("Thuan");
                    ret = writeForward(slave_id.adr[i]);
                    dir[i] = forward;
                }else if(speed[i] < 0 && dir[i] != reverse) {   // Trigger
                    ROS_ERROR("nghich");
                    ret = writeReverse(slave_id.adr[i]);
                    dir[i] != reverse;
                }
                CHECK_RETURN(ret);
                ret = writeSpeed(slave_id.adr[i], speed[i]);
                CHECK_RETURN(ret);
            }
            else if(speed[i] == 0)
            { 
                ROS_ERROR("stop");
                dir[i] = stopping;
                ret = writeStop(slave_id.adr[i]);
                CHECK_RETURN(ret);
            }
        }
    }else
    {
        ros::Duration(3).sleep(); // sleep for 3 second
        ROS_ERROR("There are not slave ID of devices GGM Motor");
        return 0;
    }
    return ret; 
}


int ORIENTAL_MOTOR::modbus_oriental::writeSpeedControlMode(uint16_t mode)
{
    int ret;
    if(slave_id.size > 0)
    {
         for(uint8_t i = 0; i < slave_id.size; i++)
        {
            ret = writeSpeedControlMode(slave_id.adr[i], mode);
            CHECK_RETURN(ret);
        }
    }else
    {
        ros::Duration(3).sleep(); // sleep for 3 second
        ROS_ERROR("There are not slave ID of devices GGM Motor");
        return 0;
    }
    return ret; 
}


int ORIENTAL_MOTOR::modbus_oriental::writeAcceleration(uint16_t time)
{
    int ret;
    if(slave_id.size > 0)
    {
         for(uint8_t i = 0; i < slave_id.size; i++)
        {
            ret = writeAcceleration(slave_id.adr[i], time);
            CHECK_RETURN(ret);
        }
    }else
    {
        ros::Duration(3).sleep(); // sleep for 3 second
        ROS_ERROR("There are not slave ID of devices GGM Motor");
        return 0;
    }
    return ret;
}

int ORIENTAL_MOTOR::modbus_oriental::writeDeceleration(uint16_t time)
{
    int ret;
    if(slave_id.size > 0)
    {
        for(uint8_t i = 0; i < slave_id.size; i++)
        {
            ret = writeDeceleration(slave_id.adr[i], time);
            CHECK_RETURN(ret);
        }
    }else
    {
        ros::Duration(3).sleep(); // sleep for 3 second
        ROS_ERROR("There are not slave ID of devices GGM Motor");
        return 0;
    }
    return ret;
}