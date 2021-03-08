//
// Created by Hiep  on 5/12/2020.
//
#pragma once
#include <ros/ros.h>
#include "serial.h"

typedef struct _rs485 rs485_t;

class rs485: public Serial
{
private:  
    /*
     * Configure
     * @param devfile   Path name of dev  
     * @param baud      Baudrate 
     * @param parity    Parity
     * @param data_bit
     * @param stop_bit 
     */
    virtual int _rs485_connect(rs485_t &ctx); 

    /*
     * create new port
     * @param devfile   Path name of dev  
     * @param baud      Baudrate 
     * @param parity    Parity
     * @param data_bit
     * @param stop_bit 
     */
    virtual void new_port(const char* devfile, unsigned int baud, parity_t parity, 
                                        data_bits_t data_bit,stop_bits_t stop_bit);
public:
    /*
     * Data Sender
     * @param to_send Request to Be Sent to Server
     * @param length  Length of the Request
     * @return        Size of the request
     */
    virtual ssize_t sendMsgs(uint8_t *to_send, uint16_t length);

    /*
     * Data Receiver
     * @param buffer Buffer to Store the Data` Retrieved
     * @return       Size of Incoming Data
     */
    virtual ssize_t receiveMsgs(uint8_t *buffer) const;

    /*
     *  Reconnect to device if rs485 modules was disconected.
     */
    virtual void reconnect(void);
    /*
     * Close serial port
     */
    virtual void close_port();
    /*
     * @brief Constructor 
     * @param[in] devfile   Example /dev/tty* 
     * @param[in] baud      Number of transmitted bits  per a second
     * @param[in] parity    The parity check bit {Even, None , Old }
     * @param[in] data_bit  Number of bits in a transmission frame 
     * @param[in] stop_bit  End bit
     */
    rs485(const char* devfile, unsigned int baud, parity_t parity, 
                                        data_bits_t data_bit,stop_bits_t stop_bit);
    /*
     * Destructor.
     */
    virtual ~rs485();
/* Properties */
protected:
    rs485_t ctx;
    uint8_t size_pkg;
    float param_safety;
public:
    bool _connected{};      /* Write status connection of devices */
    std::string error_msg;  /* To saved error message */
};
