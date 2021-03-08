#ifndef __MODBUS_OROENTAL_H_INCLUDE_
#define __MODBUS_OROENTAL_H_INCLUDE_
#include "mbrtu/modbusRTU.h"
#include "modbus_oriental/define.h"
#include "modbus_oriental/modbus_oriental_diagnostic.h"

#define CHECK_RETSULT(ret) {if(ret == BAD_CON ||  ret == EX_BAD_DATA) break;}
#define CHECK_RETURN(ret) {if(ret == BAD_CON ||  ret == EX_BAD_DATA) return ret;}

ORIENTAL_MOTOR_BEGIN

    typedef struct _ID_Number
    {
        uint8_t adr[MAX_ID_LENGHT];
        uint8_t size;
    }id_number_t;
    
    typedef enum _direction
    {
        forward,
        reverse,
        stopping    
    }direction_en;

    // shortcut to read a member from a XmlRpcValue, or to return a defaultvalue, it the member does not exist
    template<class T> static T readMember(XmlRpc::XmlRpcValue & value, const std::string & member, const T & defaultvalue)
    {
        if(value.hasMember(member))
            return value[member];
        return defaultvalue;
    }

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
        modbus_oriental(ros::NodeHandle & nh, uint16_t poly, const char* devfile, unsigned int baud, parity_t parity, 
                                            data_bits_t data_bit,stop_bits_t stop_bit);
        
        /*
        * Destructor
        */
        ~modbus_oriental();

        /*
         * Subcrible speed_wheel to controll motor
         * @param[in] speed The speed of left motor and rigth motor
         */
        virtual int publish_speed(uint16_t *speed);

        virtual int writeSpeedControlMode(uint16_t mode);

        virtual int writeAcceleration(uint16_t time);

        virtual int writeDeceleration(uint16_t time);


protected:

        /**
         * Get parameter from yaml file
         */
        virtual void loadParam(void);

        /*
         * Set slave id 
         * @param ids       Pointer save array slave id  
         * @param lenght    Lenght of ids array (sizeof(ids)/sizeof(uint8_t))
         */
        virtual void set_slave_id(int *ids, uint8_t lenght);


        /*
         * Check baudrate has supported for device
         * @param baud      Package Array size byte
         * @param return    true/false 
         */
        bool check_baudrate(unsigned int baud);

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

        /*
        *
        */
        virtual int feedbackSpeed(uint8_t slave_id, uint16_t *speed);

    
        virtual uint16_t createMotorControl16bit(uint8_t motorDirection, bool freeLockOnStop = true, bool slowChange = true, uint8_t motorDataNum = 0);

    private:
        ros::NodeHandle nh_;
        XmlRpc::XmlRpcValue drivers;  // yaml flie 
        id_number_t slave_id;
        direction_en dir[2] = {stopping, stopping};
    };
ORIENTAL_MOTOR_END
#endif