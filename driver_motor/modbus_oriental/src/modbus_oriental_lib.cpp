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
ORIENTAL_MOTOR::modbus_oriental::modbus_oriental(uint16_t poly , const char* devfile, unsigned int baud, 
                            parity_t parity, data_bits_t data_bit,stop_bits_t stop_bit)
 : mosbus_rtu(poly, devfile, baud, parity, data_bit, stop_bit)
{
    
}

/*
 * Destructor.
 */
ORIENTAL_MOTOR::modbus_oriental::~modbus_oriental()
{

}

/*
 * Delete devfile and close
 */
void ORIENTAL_MOTOR::modbus_oriental::closeDevice(void)
{
    this->close_port();
}
uint16_t ORIENTAL_MOTOR::modbus_oriental::createMotorControl16bit(uint8_t motorDirection, bool freeLockOnStop, bool slowChange, uint8_t motorDataNum) {
  // MB-FREE, -, STOP-MODE, REV, FWD, M1, M2, M0
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

