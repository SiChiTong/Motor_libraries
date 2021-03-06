#pragma once

/*
 * Exception Codes
 */
#define    EX_PARAM_NOT_EXIT       0x00 /* The parameter number does not exits */
#define    EX_ACCESS_TO_PARAM      0x01 /* There is no write access to the parameter */
#define    EX_PARAM_LIMITS         0x02 /* The data value exceeds the parameter limits */
#define    EX_SUB_INDEX_NO_EXIT    0x03 /* The sub-index in use does not exits */
#define    EX_PARAM_NOT_TYPE       0x04 /* The parameter is not of the array type */
#define    EX_TYPE_NO_MATCH        0x05 /* The data type does not match the parameter called */
#define    EX_ONLY_RESET           0x06 /* Only reset */
#define    EX_NOT_CHANGEABLE       0x07 /* Not changeable */
#define    EX_NO_WRITE_ACCESS      0x0B /* No write address */
#define    EX_NO_POSSIBLE_PRE_MODE 0x11 /* Data change in the parameter called is not possible in the present mode */
#define    EX_OTHER_ERROR          0x12 /* Other error */
#define    EX_INVALID_DATA_ADDR    0x40 /* Invalid data address */
#define    EX_INVALID_MSGS_LENGTH  0x41 /* Invalid message length */
#define    EX_INVALID_LENGTH_VALUE 0x42 /* Invalid data length or value */
#define    EX_INVALID_FUNC_CODE    0x43 /* Invalid function code */
#define    EX_NO_BUS_ACCESS        0x82 /* There is no bus access */
#define    EX_DATA_NOT_POSSIBLE    0x83 /* Data change is not possible because factory set-up is selected */
#define    EX_BAD_DATA             0XFF /* Bad Data lenght or Address */

#define    BAD_CON                 -1
#define    BAD_CONFIG              0
#define    WORD_BITS               65535
#define    MAX_MSG_LENGTH          1024
/*
 * Commonly use function codes
 */

#define    READ_COIL_BITS              0x01 /* Read coils status */
#define    READ_INPUT_BITS             0x02 /* Read input status */
#define    READ_HOLDING_REGS           0x03 /* Read holding registers */
#define    READ_INPUT_REGS             0x04 /* Read input register */
#define    WRITE_SINGLE_COIL           0x05 /* Write single coil status */
#define    WRITE_SINGLE_HOLDING_REG    0x06 /* Write single register */
#define    WRITE_MULTIPLE_COILS        0X0F /* Multiple coil write */ 
#define    WRITE_MULTIPLE_HOLDING_REGS 0X10 /* Multiple register write*/



