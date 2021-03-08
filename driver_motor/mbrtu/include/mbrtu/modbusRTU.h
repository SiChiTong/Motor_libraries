#ifndef __MOSBUS_RTU_H_INCLUDE_
#define __MOSBUS_RTU_H_INCLUDE_
#include <iostream>
#include <stdio.h>
#include <cstdint>	/* uin8_t */
#include "mbrtu/define.h"
#include "libserial/rs485.h"

class mosbus_rtu : public rs485
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
    mosbus_rtu(uint16_t poly, const char* devfile, unsigned int baud, parity_t parity, 
                                        data_bits_t data_bit,stop_bits_t stop_bit);
    /*
     * Destructor.
     */
    virtual ~mosbus_rtu();

    /*
     * Read Holding Registers
     * @brief MODBUS     FUNCTION 0x03
     * @param[in] slave_id   The id on modbus device 
     * @param[in] address    Reference Address
     * @param[in] amount     Amount of Registers to Read
     * @param[in] buffer     Buffer to Store Data Read from Registers
     */
    virtual int modbus_read_holding_registers(uint8_t slave_id, int address, int amount, uint16_t *buffer);
    
    /*
     * Read Input Registers
     * @brief MODBUS FUNCTION 0x04
     * @param[in] slave_id    The id on modbus device 
     * @param[in] address     Reference Address
     * @param[in] amount      Amount of Registers to Read
     * @param[in] buffer      Buffer to Store Data Read from Registers
     */
    virtual int modbus_read_input_registers(uint8_t slave_id, int address, int amount, uint16_t *buffer);

    /*
     * Write Single Register
     * @brief FUCTION 0x06
     * @param[in] slave_id    The id on modbus device 
     * @param[in] address   Reference Address
     * @param[in] value     Value to Be Written to Register
     */
    virtual int modbus_write_register(uint8_t slave_id, int address, const uint16_t& value);
   
    /*
    * Write Multiple Registers
    * @brief MODBUS FUNCION 0x10
    * @param[in] slave_id    The id on modbus device 
    * @param[in] address Reference Address
    * @param[in] amount  Amount of Value to Write
    * @param[in] value   Values to Be Written to the Registers
    */
    int modbus_write_registers(uint8_t slave_id, int address, int amount, const uint16_t *value);
    
    /*
     * Data Sender
     * @param[in] to_send Request to Be Sent to Server
     * @param[in] length  Length of the Request
     * @param[in]        Size of the request
     */
    // ssize_t (*sendMsgs) (uint8_t *, uint16_t);
    
    // /*
    //  * Data Receiver
    //  * @param[in] buffer Buffer to Store the Data Retrieved
    //  * @return       Size of Incoming Data
    //  */
    // ssize_t (*receiveMsgs) (uint8_t *);
private:
    /*
     * @brief Mb_calcul_crc : compute the crc of a packet and put it at the end
     * @param[in] msg         Message to send 
     * @param[in] length      The length of message to send
     * @param[in] poly        The poly number to caculartion CRC16
     * @return CRC            Result is CRC value 16 bit
     */
    virtual uint16_t getCRC16(uint8_t *to_send, uint16_t length);
    /*
     * @brief Mb_calcul_crc : compute the crc of a packet and put it at the end
     * @param[in] msg         Message to send 
     * @param[in] length      The length of message to send
     * @param[in] poly        The poly number to caculartion CRC16
     * @return bool           Result is CRC value true/false
     */
    virtual bool checkCRC16(uint8_t *to_send, uint16_t length);
    /*
     * Modbus Request Builder
     * @param[in] slave_id   The id on modbus device
     * @param[in] to_send   Message Buffer to Be Sent
     * @param[in] address   Reference Address
     * @param[im] func      Modbus Functional Code
     */
    inline void modbus_build_request(uint8_t slave_id, uint8_t *to_send, int address, uint8_t func) const;
    
    /*
     * Read Request Builder and Sender
     * @param[in] slave_id   The id on modbus device
     * @param[in] address   Reference Address
     * @param[in] amount    Amount of Data to Read
     * @param[in] func      Modbus Functional Code
     */
    ssize_t modbus_read(uint8_t slave_id, int address, uint amount, int func);

    /*
     * Write Request Builder and Sender
     * @param[in] slave_id   The id on modbus device
     * @param[in] address   Reference Address
     * @param[in] amount    Amount of data to be Written
     * @param[in] func      Modbus Functional Code
     * @param[in] value     Data to Be Written
     */
    ssize_t modbus_write(uint8_t slave_id, int address, uint amount, int func, const uint16_t *value);
    
    /*
     * Data Sender
     * @param[in] to_send Request to Be Sent to Server
     * @param[in] length  Length of the Request
     * @param[in]        Size of the request
     */
    inline ssize_t modbus_send(uint8_t *to_send, uint16_t length);

    /*
     * Data Receiver
     * @param[in] buffer Buffer to Store the Data Retrieved
     * @return       Size of Incoming Data
     */
    inline ssize_t modbus_receive(uint8_t *buffer) const;

    /*
     * Error Code Handler
     * @param[in] msg   Message Received from the Server
     * @param[in] func  Modbus Functional Code
     */
    virtual void modbuserror_handle(const uint8_t *msg, int func);

    /*
     * Set Bad Data lenght or Address
     */
    inline void set_bad_input();

    /*
     * Set Bad connection
     */
    void set_bad_con();

    /* Properties */
    bool err{};
    int err_no{};
    uint16_t _poly;          /* POLY is const number to cacurla CRC16 */
};
#endif