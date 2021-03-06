#ifndef __MODBUS_OROENTAL_H_INCLUDE_
#define __MODBUS_OROENTAL_H_INCLUDE_
#include "modbus_oriental/define.h"
#include "mbrtu/modbusRTU.h"

ORIENTAL_MOTOR_BEGIN
class modbus_oriental : public mosbus_rtu 
{
public:
    /*
     * @brief Constructor 
     * @param[in] poly      The poly number to caculartion CRC16
     * @param[in] devfile   Example /dev/tty* 
     * @param[in] baud      Number of transmitted bits  per a second
     * @param[in] parity    The parity check bit {Even, None , Old }
     * @param[in] data_bit  Number of bits in a transmission frame 
     * @param[in] stop_bit  End bit
     */
    modbus_oriental(uint16_t poly, const char* devfile, unsigned int baud, parity_t parity, 
                                        data_bits_t data_bit,stop_bits_t stop_bit);
    
    /*
     * Destructor
     */
    ~modbus_oriental();

    /**
     * Delete devfile and close
     */
    virtual void closeDevice(void);

    /*
     *
     */
    virtual int writeStop(uint8_t slave_id);                    

    /*
     *
     */
    virtual int writeConfigTrigger(uint8_t slave_id);

    /*
     *
     */
    virtual int writeForward(uint8_t slave_id);

    /*
     *
     */
    virtual int writeReverse(uint8_t slave_id);

    /*
     *
     */
    virtual int writeSpeed(uint8_t slave_id, uint16_t speed);

    /*
     *
     */
    virtual int writeSpeedControlMode(uint8_t slave_id, uint16_t mode);

    /*
     *
     */
    virtual int writeResetAlarm(uint8_t slave_id);

    /*
     *
     */ 
    virtual int clearAlarmRecords(uint8_t slave_id);

    /*
     *
     */
    virtual int clearWarningRecords(uint8_t slave_id);
    
    /*
     *
     */
    virtual int resetAndClearAll(uint8_t slave_id);
    /*
     *
     */
    virtual int writeAcceleration(uint8_t slave_id, uint16_t time);

    /*
     *
     */
    virtual int writeDeceleration(uint8_t slave_id, uint16_t time);
  
    /*
     *
     */
    virtual int writeTorqueLimit(uint8_t slave_id,uint16_t torque);
    
    /*
     *
     */
    virtual int readDirection(uint8_t slave_id,bool *forwarding, bool *reversing, bool *freeLockOnStop);

    /*
     *
     */
    virtual int readRotationSpeed(uint8_t slave_id,uint32_t *speed);

    /*
     *
     */
    virtual int readSpeedControlMode(uint8_t slave_id,uint16_t *mode);

    /*
     *
     */
    virtual int readAlarm(uint8_t slave_id,uint16_t *alarm);

    /*
     *
     */
    virtual int readWarning(uint8_t slave_id,uint16_t *warning);

    /*
     *
     */
    virtual int readTorqueLimit(uint8_t slave_id,uint16_t *torque);

    /*
     *
     */
    virtual int feedbackSpeed(uint8_t slave_id, uint32_t *speed);


protected:
    virtual uint16_t createMotorControl16bit(uint8_t motorDirection, bool freeLockOnStop = true, bool slowChange = true, uint8_t motorDataNum = 0);
};
ORIENTAL_MOTOR_END
#endif