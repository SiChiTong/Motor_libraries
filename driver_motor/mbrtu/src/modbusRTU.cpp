#include "mbrtu/modbusRTU.h"


/*
 * @brief Constructor 
 * @param[in] poly      The poly number to caculartion CRC16
 * @param[in] devfile   Example /dev/tty* 
 * @param[in] baud      Number of transmitted bits  per a second
 * @param[in] parity    The parity check bit {Even, None , Old }
 * @param[in] data_bit  Number of bits in a transmission frame 
 * @param[in] stop_bit  End bit
 */
mosbus_rtu::mosbus_rtu(uint16_t poly, const char* devfile, unsigned int baud, parity_t parity, 
                                data_bits_t data_bit,stop_bits_t stop_bit)
 : _poly(poly), rs485(devfile, baud, parity, data_bit, stop_bit)
{
    err = false;
    err_no = 0;
    error_msg = "";
}      

/*
 * Destructor.
 */                       
mosbus_rtu::~mosbus_rtu()
{
    
}

/*
 * Read Holding Registers
 * MODBUS FUNCTION 0x03
 * @param[in] slave_id   The id on modbus device
 * @param[in] address    Reference Address
 * @param[in] amount     Amount of Registers to Read
 * @param[in] buffer     Buffer to Store Data Read from Registers
 */
int mosbus_rtu::modbus_read_holding_registers(uint8_t slave_id, int address, int amount, uint16_t *buffer)
{
    if(_connected)
    {
        if(address > WORD_BITS || amount > WORD_BITS)
        {
            set_bad_input();
            return EX_BAD_DATA;
        }
        modbus_read(slave_id, address, amount, READ_HOLDING_REGS);
        uint8_t to_rec[MAX_MSG_LENGTH];
        memset(to_rec,'\0',MAX_MSG_LENGTH);
        ssize_t k = 0;
        k = modbus_receive(to_rec);
        if (k == -1) {
            set_bad_con();
            return BAD_CON;
        }
        if(k > 0)
        {
            // for(int i = 0; i < k; i++) ROS_INFO("0x%x",to_rec[i]);
            if(!checkCRC16(to_rec,k)) 
            {
                ROS_ERROR("HERE");
                set_bad_input();
                return EX_BAD_DATA;
            }
            modbuserror_handle(to_rec, READ_HOLDING_REGS);
            if(err) return err_no;
            for(int i = 0; i < to_rec[2u]/2; i++)
            {
                buffer[i] = ((uint16_t)to_rec[3u + 2u * i]) << 8u;
                buffer[i] |= ((uint16_t)to_rec[4u + 2u * i] & 0x00FFu);
            }
            return 0;
        } else {
                ROS_ERROR("Driver motor no response !!");
                set_bad_input();
                return EX_BAD_DATA;
            }
    }else {
        set_bad_con();
        return BAD_CON;
    }
}

/*
 * Read Input Registers
 * MODBUS FUNCTION 0x04
 * @param[in] slave_id    The id on modbus device 
 * @param[in] address     Reference Address
 * @param[in] amount      Amount of Registers to Read
 * @param[in] buffer      Buffer to Store Data Read from Registers
 */
int mosbus_rtu::modbus_read_input_registers(uint8_t slave_id, int address, int amount, uint16_t *buffer)
{
    if(_connected)
    {
        if(amount > WORD_BITS || address)
        {
            set_bad_input();
            return EX_BAD_DATA;           
        }
        modbus_read(slave_id, address, amount, READ_INPUT_REGS);
        uint8_t to_rec[MAX_MSG_LENGTH];
        memset(to_rec,'\0',MAX_MSG_LENGTH);
        ssize_t k = 0;
        k = modbus_receive(to_rec);
        if (k == -1) {
            set_bad_con();
            return BAD_CON;
        }
        if(k > 0)
        {
            if(!checkCRC16(to_rec,k)) 
            {
                set_bad_input();
                return EX_BAD_DATA;
            }
            modbuserror_handle(to_rec, READ_INPUT_REGS);
            if(err) return err_no; 
            for(int i = 0; i < to_rec[2u]/2; i++)
            {
                buffer[i] = ((uint16_t)to_rec[3u + 2u * i]) << 8u;
                buffer[i] |= ((uint16_t)to_rec[4u + 2u * i] & 0x00FFu);
            }
            return 0;
        } else {
                ROS_ERROR("Driver motor no response !!");
                set_bad_input();
                return EX_BAD_DATA;
            }
    }else {
        set_bad_con();
        return BAD_CON;
    }
}

/*
 * Write Single Register
 * @brief FUCTION 0x06
 * @param[in] slave_id      The id on modbus device 
 * @param[in] address       Reference Address
 * @param[in] value         Value to Be Written to Register
 */
int mosbus_rtu::modbus_write_register(uint8_t slave_id, int address, const uint16_t& value)
{
    if(_connected)
    {
        if(address > WORD_BITS)
        {
            set_bad_input();
            return EX_BAD_DATA;
        }
        modbus_write(slave_id, address, 1, WRITE_SINGLE_HOLDING_REG, &value);
        uint8_t to_rec[MAX_MSG_LENGTH];
        memset(to_rec,'\0',MAX_MSG_LENGTH);
        ssize_t k = 0;
        k = modbus_receive(to_rec);
        if (k == -1) 
        {
            set_bad_con();
            return BAD_CON;
        }
        if(k > 0)
        { 
            // for(int i = 0; i < k; i++) ROS_INFO("0x%x",to_rec[i]);
            if(!checkCRC16(to_rec, k)) 
            {
                set_bad_input();
                return EX_BAD_DATA;
            }
            modbuserror_handle(to_rec, WRITE_SINGLE_HOLDING_REG);
            if(err) return err_no;
            return 0;
        } else {
                ROS_ERROR("Driver motor no response !!");
                set_bad_input();
                return EX_BAD_DATA;
            }
    } else {
        set_bad_con();
        return BAD_CON;
    }
}

/*
 * Write Multiple Registers
 * @brief MODBUS FUNCION 0x10
 * @param[in] slave_id      The id on modbus device
 * @param[in] address   Reference Address
 * @param[in] amount    Amount of Value to Write
 * @param[in] value     Values to Be Written to the Registers
 */
int mosbus_rtu::modbus_write_registers(uint8_t slave_id, int address, int amount, const uint16_t *value)
{
    if(_connected)
    {
        if(amount > WORD_BITS || address > WORD_BITS)
        {
            set_bad_input();
            return EX_BAD_DATA;
        }
        modbus_write(slave_id, address, amount, WRITE_MULTIPLE_HOLDING_REGS, value);
        uint8_t to_rec[MAX_MSG_LENGTH];
        memset(to_rec,'\0',MAX_MSG_LENGTH);
        ssize_t k = 0;
        k = modbus_receive(to_rec);
        if(k == -1)
        {
            set_bad_con();
            return BAD_CON;
        }
        if(k > 0)
        { 
            if(!checkCRC16(to_rec, k)) 
            {
                set_bad_input();
                return EX_BAD_DATA;
            }
            modbuserror_handle(to_rec, WRITE_MULTIPLE_HOLDING_REGS);
            if(err) return err_no;
            return 0;
        } else {
                ROS_ERROR("Driver motor no response !!");
                set_bad_input();
                return EX_BAD_DATA;
            }
    } else {
        set_bad_con();
        return BAD_CON;
    }
}

/*
 * @brief Mb_calcul_crc : compute the crc of a packet and put it at the end
 * @param[in] msg         Message to send 
 * @param[in] length      The length of message to send
 * @param[in] poly        The poly number to caculartion CRC16
 * @return CRC            Result is CRC value 16 bit
 */
uint16_t mosbus_rtu::getCRC16(uint8_t *to_send, uint16_t length)
{
    /*
        EX: Ban đầu CRC = 1111 1111 1111 1111 chuyển sang Hex là FFFF
        Chọn data_p là 54 hay 0101 0100(1 byte) là số cần tính. 
        Chọn số poly =  A001h hay 1010 000 000 0001 
        (Poly là một số mà bạn sử dụng làm phép tính số CRC cơ sở của mình.)

        + Bước 1: Dịch CRC và data_p sang phải 1 bit 
        data_p = 54, là 0101 0100 trở thành 0010 1010 
        CRC = 1111 1111 1111 1111 trở thành 0111 1111 1111 1111
        
        + Bước 2: Kiểm tra BIT ngoài cùng bên phải của Dữ liệu và so sánh nó với một trong các CRC
        NẾU chúng bằng nhau, dịch chuyển CRC sang phải 1 bit 
        NẾU chúng không phải, dịch chuyển CRC sang phải 1 bit VÀ cộng thêm số Poly một lần nữa.
        Thực hiện bước 2 đúng 8 lần vì 1 byte có 8 bit.

        +Bước 3: Bước 1 và 2 sẽ được lăp lại theo số lượng data_p.
    */
    unsigned char i;
	unsigned int data;
	unsigned int crc = 0xffff;
    do
	{
	    for (i=0, data=(unsigned int)0xff & *to_send++; 
	    	 i < 8; 
	    	 i++, data >>= 1)
	    {
	        if ((crc & 0x0001) ^ (data & 0x0001))
	            crc = (crc >> 1) ^ _poly;
	        else  crc >>= 1;
	    }
	} while (--length);
	return (crc);
}

/*
 * @brief Mb_calcul_crc : compute the crc of a packet and put it at the end
 * @param[in] msg         Message to send 
 * @param[in] length      The length of message to send
 * @param[in] poly        The poly number to caculartion CRC16
 * @return bool           Result is CRC value true/false
 */
bool mosbus_rtu::checkCRC16(uint8_t *to_send, uint16_t length)
{
    uint16_t to_check;
    to_check = getCRC16(to_send,length-2);
    return to_send[length -2] == (uint8_t)(to_check & 0x00FFu) && to_send[length -1] == (uint8_t)(to_check >> 8u);
}

/*
 * Modbus Request Builder
 * @param[in] slave_id   The id on modbus device
 * @param[in] to_send   Message Buffer to Be Sent
 * @param[in] address   Reference Address
 * @param[in] func      Modbus Functional Code
 */
void mosbus_rtu::modbus_build_request(uint8_t slave_id, uint8_t *to_send, int address, uint8_t func) const 
{
    to_send[0] = slave_id;
    to_send[1] = func;
    to_send[2] = (uint8_t) (address >> 8u);
    to_send[3] = (uint8_t) (address & 0x00FFu);
}

/*
 * Write Request Builder and Sender
 * @param[in] slave_id   The id on modbus device
 * @param[in] address   Reference Address
 * @param[in] amount    Amount of data to be Written
 * @param[in] func      Modbus Functional Code
 * @param[in] value     Data to Be Written
 */
ssize_t mosbus_rtu::modbus_write(uint8_t slave_id, int address, uint amount, int func, const uint16_t *value) 
{
    ssize_t result = 0;
    if(func == WRITE_SINGLE_COIL || func == WRITE_SINGLE_HOLDING_REG)
    {
        uint8_t to_send[8];
        modbus_build_request(slave_id, to_send, address, func);
        to_send[4] = (uint8_t) (value[0] >> 8u);
        to_send[5] = (uint8_t) (value[0] & 0x00FFu);
        uint16_t crc = getCRC16(to_send,6);
        to_send[6] = (uint8_t) (crc & 0x00FFu);
        to_send[7] = (uint8_t) (crc >> 8u);
        // for(int i = 0; i < 8; i++) ROS_INFO("0x%x", to_send[i]);
        result = modbus_send(to_send,8);
    }
    else if(func == WRITE_MULTIPLE_HOLDING_REGS)
    {
        uint8_t to_send[9 + 2 * amount];
        modbus_build_request(slave_id,to_send, address, func);
        to_send[4] = (uint8_t)(amount >> 8u);
        to_send[5] = (uint8_t)(amount & 0x00FFu);
        to_send[6] = (uint8_t)(amount * 2);
        for(int i = 0; i < amount; i++)
        {
            to_send[7 + 2 * i] = (uint8_t)(value[i] >> 8u);
            to_send[8 + 2 * i] = (uint8_t)(value[i] & 0x00FFu);
        }
        uint16_t crc = getCRC16(to_send, 9 + 2 * amount - 2);
        to_send[9 + 2 * amount - 2] = (uint8_t)(crc & 0x00FFu);
        to_send[9 + 2 * amount - 1] = (uint8_t)(crc >> 8u);
        //for(int i = 0; i < 9 + 2 * amount; i++) ROS_INFO("0x%x", to_send[i]);
        result = modbus_send(to_send, 9 + 2 * amount);
    }
    return result;
}
/*
 * Read Request Builder and Sender
 * @param[in] slave_id   The id on modbus device
 * @param[in] address   Reference Address
 * @param[in] amount    Amount of Data to Read
 * @param[in] func      Modbus Functional Code
 */
ssize_t mosbus_rtu::modbus_read(uint8_t slave_id, int address, uint amount, int func)
{
    uint8_t to_send[8];
    modbus_build_request(slave_id, to_send, address, func);
    to_send[4] = (uint8_t) (amount >> 8u);
    to_send[5] = (uint8_t) (amount & 0x00FFu);
    uint16_t crc = getCRC16(to_send,6);
    to_send[6] = (uint8_t) (crc & 0x00FFu);
    to_send[7] = (uint8_t) (crc >> 8u);
    return modbus_send(to_send, 8);
}

/*
 * Data Sender
 * @param[in] to_send Request to Be Sent to Server
 * @param[in] length  Length of the Request
 * @return        Size of the request
 */
ssize_t mosbus_rtu::modbus_send(uint8_t *to_send, uint16_t length)
{
    // if(sendMsgs) 
    //     return (*sendMsgs)(to_send, length);
    // else return BAD_CONFIG;
    return this->sendMsgs(to_send, length);
}

/*
 * Data Receiver
 * @param[in] buffer Buffer to Store the Data Retrieved
 * @return       Size of Incoming Data
 */
ssize_t mosbus_rtu::modbus_receive(uint8_t *buffer) const
{
    // if(receiveMsgs)
    //     return (*receiveMsgs)(buffer);
    // else return BAD_CONFIG;
    return this->receiveMsgs(buffer);
}

/*
 * Error Code Handler
 * @param[in] msg   Message Received from the Server
 * @param[in] func  Modbus Functional Code
 */
void mosbus_rtu::modbuserror_handle(const uint8_t *msg, int func)
{
    if(msg[1] == func + 0x80)
    {
        err = true;
        err_no = 1;
        switch(msg[2])
        {
        case EX_PARAM_NOT_EXIT:
            error_msg = "The parameter number does not exits";
            break;
        case EX_ACCESS_TO_PARAM:
            error_msg = "There is no write access to the parameter";
            break;
        case EX_PARAM_LIMITS:
            error_msg = "The data value exceeds the parameter limits";
            break;
        case EX_SUB_INDEX_NO_EXIT:
            error_msg = "The sub-index in use does not exits";
            break;
        case EX_PARAM_NOT_TYPE:
            error_msg = "The parameter is not of the array type";
            break;
        case EX_TYPE_NO_MATCH:
            error_msg = "The data type does not match the parameter called";
            break;
        case EX_ONLY_RESET:
            error_msg = "Only reset";
            break;
        case EX_NOT_CHANGEABLE:
            error_msg = "Not changeable";
            break;
        case EX_NO_WRITE_ACCESS:
            error_msg = "No write address";
            break;
        case EX_NO_POSSIBLE_PRE_MODE:
            error_msg = "Data change in the parameter called is not possible in the present mode";
            break;
        case EX_OTHER_ERROR:
            error_msg = "Other error";
            break;      
        case EX_INVALID_DATA_ADDR:
            error_msg = "Invalid data address";
            break; 
        case EX_INVALID_MSGS_LENGTH:
            error_msg = "Invalid message length";
            break; 
        case EX_INVALID_LENGTH_VALUE:
            error_msg = "Invalid data length or value";
            break; 
        case EX_INVALID_FUNC_CODE:
            error_msg = "Invalid function code";
            break; 
        case EX_NO_BUS_ACCESS:
            error_msg = "There is no bus access";
            break; 
        case EX_DATA_NOT_POSSIBLE:
            error_msg = "Data change is not possible because factory set-up is selected";
            break;  
        }
    }
    err = false;
    error_msg = "NO ERR";

}

/*
 * Set Bad Data lenght or Address
 */
void mosbus_rtu::set_bad_input() 
{
    err = true;
    error_msg = "Bad Data lenght or Address";
}

/*
 * Set Bad connection
 */
void mosbus_rtu::set_bad_con() 
{
    err = true;
    error_msg = "Bad connection";
}