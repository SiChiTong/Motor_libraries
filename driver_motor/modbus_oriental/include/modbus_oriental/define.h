#pragma once

#define ORIENTAL_MOTOR_BEGIN namespace BLVD20KM {
#define ORIENTAL_MOTOR BLVD20KM
#define ORIENTAL_MOTOR_END  }
#define MAX_ID_LENGHT 31
#define POLY   0xA001

/*
 * Maintenance commands:
 * These commands are used to reset alarms and warnings. They are also used to execute the batch
 * processing for the non-volatile memory. All commands can be read and written (READ/WRITE).
 * They are executed when written from 0 to 1.
 */
#define     RW_RESET_ALARM_UPPER                            0x0180U     /* Resets the alarm that is present. Some alarms cannot be reset with the "reset alarm." */
#define     RW_RESET_ALARM_LOWER                            0x0181U     /*  */
#define     RW_CLEAR_ALARM_HISTORY_UPPER                    0x0184U     /* Clears the alarm history. */
#define     RW_CLEAR_ALARM_HISTORY_LOWER                    0X0185U     /*  */
#define     RW_CLEAR_WARNING_HISTORY_UPPER                  0x0186U     /* Clears the warning history. */
#define     RW_CLEAR_WARNING_HISTORY_LOWER                  0x0187U     /*  */
#define     RW_CLEAR_COMMUNICATION_ERROR_HISTORY_UPPER      0x0188U     /* Clears the communication error history. */
#define     RW_CLEAR_COMMUNICATION_ERROR_HISTORY_LOWER      0x0189U     /*  */
#define     RW_CONFIGURATION_UPPER                          0x018CU     /* Executes the parameter recalculation and thesetup. */
#define     RW_CONFIGURATION_LOWER                          0x018DU     /*  */ 
#define     RW_BATCH_DATA_INITIALIZATION_UPPER              0x018EU     /* Restores the parameters saved in the non-volatile memory to their initial values. (Excluding parameters related to communication setting)*1 */
#define     RW_BATCH_DATA_INITIALIZATION_LOWER              0x018FU     /*  */
#define     RW_BATCH_NV_MEMORY_READ_UPPER                   0x0190U     /* Reads the parameters saved in the non-volatile memory to the RAM. All operation data and parameters previously saved in the RAM are overwritten. */
#define     RW_BATCH_NV_MEMORY_READ_LOWER                   0x0191U     /*  */
#define     RW_BATCH_NV_MEMORY_WRITE_UPPER                  0x0192U     /* Writes the parameters saved in the RAM to the non-volatile memory. The non-volatile memory can be rewritten approximately 100,000 times. */
#define     RW_BATCH_NV_MEMORY_WRITE_LOWER                  0x0193U     /*  */ 
#define     RW_ALL_DATA_BATCH_INITIALIZATION_UPPER          0x0194U     /* Restores all parameters saved in the non-volatile memory to their initial values. (Including parameters related to communication)*2 */

/*
 * Monitor commands
 * These commands are used to monitor the operating speed or the alarm and warning histories. All commands can
 * be read (READ)
 */
#define    R_PRESENT_ALARM_UPPER_                           0x0080U    /* Monitors the alarm code presently generated. */
#define    R_PRESENT_ALARM_LOWER                            0x0081U    /*  */
#define    R_ALARM_HISTORY_1_UPPER                          0x0082U    /* Monitors the alarm history 1  */
#define    R_ALARM_HISTORY_1_LOWER                          0x0083U    /*  */
#define    R_ALARM_HISTORY_2_UPPER                          0x0084U    /* Monitors the alarm history 2 */
#define    R_ALARM_HISTORY_2_LOWER                          0x0085U    /*  */
#define    R_ALARM_HISTORY_3_UPPER                          0x0086U    /* Monitors the alarm history 3 */
#define    R_ALARM_HISTORY_3_LOWER                          0x0087U    /*  */
#define    R_ALARM_HISTORY_4_UPPER                          0x0088U    /* Monitors the alarm history 4 */
#define    R_ALARM_HISTORY_4_LOWER                          0x0089U    /*  */
#define    R_ALARM_HISTORY_5_UPPER                          0x008AU    /* Monitors the alarm history 5 */
#define    R_ALARM_HISTORY_5_LOWER                          0x008BU    /*  */
#define    R_ALARM_HISTORY_6_UPPER                          0x008CU    /* Monitors the alarm history 6 */
#define    R_ALARM_HISTORY_6_LOWER                          0x008DU    /*  */
#define    R_ALARM_HISTORY_7_UPPER                          0x008EU    /* Monitors the alarm history 7 */
#define    R_ALARM_HISTORY_7_LOWER                          0x008FU    /*  */
#define    R_ALARM_HISTORY_8_UPPER                          0x0090U    /* Monitors the alarm history 8 */
#define    R_ALARM_HISTORY_8_LOWER                          0x0091U    /*  */
#define    R_ALARM_HISTORY_9_UPPER                          0x0092U    /* Monitors the alarm history 9 */
#define    R_ALARM_HISTORY_9_LOWER                          0x0093U    /*  */
#define    R_ALARM_HISTORY_10_UPPER                         0x0094U    /* Monitors the alarm history 10 */
#define    R_ALARM_HISTORY_10_LOWER                         0x0095U    /*  */
#define    R_PRESENT_WARNING_UPPER_                         0x0096U    /* Monitors the warning code presently generated. */
#define    R_PRESENT_WARNING_LOWER                          0x0097U    /*  */
#define    R_WARNING_HISTORY_1_UPPER                        0x0098U    /* Monitors the warning history 1 */
#define    R_WARNING_HISTORY_1_LOWER                        0x0099U    /*  */
#define    R_WARNING_HISTORY_2_UPPER                        0x009AU    /* Monitors the warning history 2 */
#define    R_WARNING_HISTORY_2_LOWER                        0x009BU    /*  */
#define    R_WARNING_HISTORY_3_UPPER                        0x009CU    /* Monitors the warning history 3 */
#define    R_WARNING_HISTORY_3_LOWER                        0x009DU    /*  */
#define    R_WARNING_HISTORY_4_UPPER                        0x009EU    /* Monitors the warning history 4 */
#define    R_WARNING_HISTORY_4_LOWER                        0x009FU    /*  */
#define    R_WARNING_HISTORY_5_UPPER                        0x00A0U    /* Monitors the warning history 5 */
#define    R_WARNING_HISTORY_5_LOWER                        0x00A1U    /*  */
#define    R_WARNING_HISTORY_6_UPPER                        0x00A2U    /* Monitors the warning history 6 */
#define    R_WARNING_HISTORY_6_LOWER                        0x00A3U    /*  */
#define    R_WARNING_HISTORY_7_UPPER                        0x00A4U    /* Monitors the warning history 7 */
#define    R_WARNING_HISTORY_7_LOWER                        0x00A5U    /*  */
#define    R_WARNING_HISTORY_8_UPPER                        0x00A6U    /* Monitors the warning history 8 */
#define    R_WARNING_HISTORY_8_LOWER                        0x00A7U    /*  */
#define    R_WARNING_HISTORY_9_UPPER                        0x00A8U    /* Monitors the warning history 9 */
#define    R_WARNING_HISTORY_9_LOWER                        0x00A9U    /*  */
#define    R_WARNING_HISTORY_10_UPPER                       0x00AAU    /* Monitors the warning history 10 */
#define    R_WARNING_HISTORY_10_LOWER                       0x00ABU    /*  */
#define    R_COMMUNICATION_ERROR_CODE_UPPER                 0x00ACU    /* Monitors the communication error code presently generated. */
#define    R_COMMUNICATION_ERROR_CODE_LOWER                 0x00ADU    /*  */
#define    R_COMMUNICATION_ERROR_CODE_HISTORY_1_UPPER       0x00AEU    /* Monitors the communication error code history 1 */
#define    R_COMMUNICATION_ERROR_CODE_HISTORY_1_LOWER       0x00AFU    /*  */
#define    R_COMMUNICATION_ERROR_CODE_HISTORY_2_UPPER       0x00B0U    /* Monitors the communication error code history 2 */
#define    R_COMMUNICATION_ERROR_CODE_HISTORY_2_LOWER       0x00B1U    /*  */
#define    R_COMMUNICATION_ERROR_CODE_HISTORY_3_UPPER       0x00B2U    /* Monitors the communication error code history 3 */
#define    R_COMMUNICATION_ERROR_CODE_HISTORY_3_LOWER       0x00B3U    /* Monitors the communication error code history 4 */
#define    R_COMMUNICATION_ERROR_CODE_HISTORY_4_UPPER       0x00B4U    /* Monitors the communication error code history 4 */
#define    R_COMMUNICATION_ERROR_CODE_HISTORY_4_LOWER       0x00B5U    /*  */
#define    R_COMMUNICATION_ERROR_CODE_HISTORY_5_UPPER       0x00B6U    /* Monitors the communication error code history 5 */
#define    R_COMMUNICATION_ERROR_CODE_HISTORY_5_LOWER       0x00B7U    /*  */
#define    R_COMMUNICATION_ERROR_CODE_HISTORY_6_UPPER       0x00B8U    /* Monitors the communication error code history 6 */
#define    R_COMMUNICATION_ERROR_CODE_HISTORY_6_LOWER       0x00B9U    /*  */
#define    R_COMMUNICATION_ERROR_CODE_HISTORY_7_UPPER       0x00BAU    /* Monitors the communication error code history 7 */
#define    R_COMMUNICATION_ERROR_CODE_HISTORY_7_LOWER       0x00BBU    /*  */
#define    R_COMMUNICATION_ERROR_CODE_HISTORY_8_UPPER       0x00BCU    /* Monitors the communication error code history 8 */
#define    R_COMMUNICATION_ERROR_CODE_HISTORY_8_LOWER       0x00BDU    /*  */
#define    R_COMMUNICATION_ERROR_CODE_HISTORY_9_UPPER       0x00BEU    /* Monitors the communication error code history 9 */
#define    R_COMMUNICATION_ERROR_CODE_HISTORY_9_LOWER       0x00BFU    /*  */
#define    R_COMMUNICATION_ERROR_CODE_HISTORY_10_UPPER      0x00C0U    /* Monitors the communication error code history 10 */
#define    R_COMMUNICATION_ERROR_CODE_HISTORY_10_LOWER      0x00C1U    /*  */
#define    R_PRESENT_OPERATION_DATA_NO_UPPER               0x00C4U    /* Monitors the operation data number presently selected.  */
#define    R_PRESENT_OPERATION_DATA_NO_LOWER               0x00C5U    /*  */
#define    R_COMMAND_SPEED_UPPER                            0x00C8U    /* Monitors the command speed presently selected. */
#define    R_COMMAND_SPEED_LOWER                            0x00C9U    /*  */
#define    R_FEEDBACK_SPEED_UPPER                           0x00CEU    /* Monitors the feedback speed. */
#define    R_FEEDBACK_SPEED_LOWER                           0x00CFU    /*  */
#define    R_DIRECT_IO_AND_ELECTRO_BRAKE_STATUS_UPPER       0x00D4U    /* Monitors the each direct I/O signal (X0 to X5, Y0, Y1) and electromagnetic brake status. */
#define    R_DIRECT_IO_AND_ELECTRO_BRAKE_STATUS_LOWER       0x00D5U    /*  */
#define    R_OPERATING_SPEED_UPPER                          0x0100U    /* Monitors the feedback speed calculated by the "Speed reduction ratio" parameter or "Speed increasing ratio" parameter. (Unit: r/min) */
#define    R_OPERATING_SPEED_LOWER                          0x0101U    /*  */
#define    R_OPERATING_SPEED_DECIMAL_POSITION_UPPER         0x0102U    /* Monitors the decimal position in the operating speed. *1 */
#define    R_OPERATING_SPEED_DECIMAL_POSITION_LOWER         0x0103U    /*  */
#define    R_CONVEYOR_TRANSFER_SPEED_UPPER                  0x0104U    /* Monitors the feedback speed calculated by the "Conveyor speed reduction ratio" parameter or "Conveyor speed increasing ratio" parameter. (Unit: r/min) */
#define    R_CONVEYOR_TRANSFER_SPEED_LOWER                  0x0105U    /*  */
#define    R_CONVEYOR_TRANS_SPEED_DECIMAL_POSITION_UPPER    0x0106U    /* Monitors the decimal position in the conveyor transfer speed. *2 */
#define    R_CONVEYOR_TRANS_SPEED_DECIMAL_POSITION_LOWER    0x0107U    /*  */
#define    R_LOAD_FACTOR_UPPER                              0x0108U    /* Monitors the torque that is output by the motor based on the rated torque being 100%. */
#define    R_LOAD_FACTOR_LOWER                              0x0109U    /*  */
#define    R_EXTERNAL_ANALOG_SPEED_SETTING_UPPER            0x010CU    /* Monitors the operating speed setting value by the analog setting. *3 */
#define    R_EXTERNAL_ANALOG_SPEED_SETTING_LOWER            0x010DU    /*  */
#define    R_EXTERNAL_ANALOG_TORQUE_LIMIT_SETTING_UPPER     0x0110U    /* Monitors the torque limiting value by the analog setting. *3  */
#define    R_EXTERNAL_ANALOG_TORQUE_LIMIT_SETTING_LOWER     0x0111U    /*  */
#define    R_EXTERNAL_ANALOG_VOLTAGE_SETTING_UPPER          0x0116U    /* Monitors the voltage setting value by the analog setting. *4  */
#define    R_EXTERNAL_ANALOG_VOLTAGE_SETTING_LOWER          0x0117U    /*  */

/*
 * Parameter R/W commands [Operation data]
 * These commands are used to read or write parameters. All commands can be read and written (READ/WRITE).
 * For details on parameter, refer to the USER MANUAL (Basic Function).
 */
#define    RW_ROTATION_SPEED_NO_0_UPPER                     0x0480U    /*  */
#define    RW_ROTATION_SPEED_NO_0_LOWER                     0x0481U    /*  */
#define    RW_ROTATION_SPEED_NO_1_UPPER                     0x0482U    /*  */
#define    RW_ROTATION_SPEED_NO_1_LOWER                     0x0483U    /*  */
#define    RW_ROTATION_SPEED_NO_2_UPPER                     0x0484U    /*  */
#define    RW_ROTATION_SPEED_NO_2_LOWER                     0x0485U    /*  */
#define    RW_ROTATION_SPEED_NO_3_UPPER                     0x0486U    /*  */
#define    RW_ROTATION_SPEED_NO_3_LOWER                     0x0487U    /*  */
#define    RW_ROTATION_SPEED_NO_4_UPPER                     0x0488U    /*  */
#define    RW_ROTATION_SPEED_NO_4_LOWER                     0x0489U    /*  */
#define    RW_ROTATION_SPEED_NO_5_UPPER                     0x048AU    /*  */
#define    RW_ROTATION_SPEED_NO_5_LOWER                     0x048BU    /*  */
#define    RW_ROTATION_SPEED_NO_6_UPPER                     0x048CU    /*  */
#define    RW_ROTATION_SPEED_NO_6_LOWER                     0x048DU    /*  */
#define    RW_ROTATION_SPEED_NO_7_UPPER                     0x048EU    /*  */
#define    RW_ROTATION_SPEED_NO_7_LOWER                     0x048FU    /*  */
#define    RW_ACCELERATION_TIME_NO_0_UPPER                  0x0600U    /*  */
#define    RW_ACCELERATION_TIME_NO_0_LOWER                  0x0601U    /*  */
#define    RW_ACCELERATION_TIME_NO_1_UPPER                  0x0602U    /*  */
#define    RW_ACCELERATION_TIME_NO_1_LOWER                  0x0603U    /*  */
#define    RW_ACCELERATION_TIME_NO_2_UPPER                  0x0604U    /*  */
#define    RW_ACCELERATION_TIME_NO_2_LOWER                  0x0605U    /*  */
#define    RW_ACCELERATION_TIME_NO_3_UPPER                  0x0606U    /*  */
#define    RW_ACCELERATION_TIME_NO_3_LOWER                  0x0607U    /*  */
#define    RW_ACCELERATION_TIME_NO_4_UPPER                  0x0608U    /*  */
#define    RW_ACCELERATION_TIME_NO_4_LOWER                  0x0609U    /*  */
#define    RW_ACCELERATION_TIME_NO_5_UPPER                  0x060AU    /*  */
#define    RW_ACCELERATION_TIME_NO_5_LOWER                  0x060BU    /*  */
#define    RW_ACCELERATION_TIME_NO_6_UPPER                  0x060CU    /*  */
#define    RW_ACCELERATION_TIME_NO_6_LOWER                  0x060DU    /*  */
#define    RW_ACCELERATION_TIME_NO_7_UPPER                  0x060EU    /*  */
#define    RW_ACCELERATION_TIME_NO_7_LOWER                  0x060FU    /*  */
#define    RW_DECELERATION_TIME_NO_0_UPPER                  0x0680U    /*  */
#define    RW_DECELERATION_TIME_NO_0_LOWER                  0x0681U    /*  */
#define    RW_DECELERATION_TIME_NO_1_UPPER                  0x0682U    /*  */
#define    RW_DECELERATION_TIME_NO_1_LOWER                  0x0683U    /*  */
#define    RW_DECELERATION_TIME_NO_2_UPPER                  0x0684U    /*  */
#define    RW_DECELERATION_TIME_NO_2_LOWER                  0x0685U    /*  */
#define    RW_DECELERATION_TIME_NO_3_UPPER                  0x0686U    /*  */
#define    RW_DECELERATION_TIME_NO_3_LOWER                  0x0687U    /*  */
#define    RW_DECELERATION_TIME_NO_4_UPPER                  0x0688U    /*  */
#define    RW_DECELERATION_TIME_NO_4_LOWER                  0x0689U    /*  */
#define    RW_DECELERATION_TIME_NO_5_UPPER                  0x068AU    /*  */
#define    RW_DECELERATION_TIME_NO_5_LOWER                  0x068BU    /*  */
#define    RW_DECELERATION_TIME_NO_6_UPPER                  0x068CU    /*  */
#define    RW_DECELERATION_TIME_NO_6_LOWER                  0x068DU    /*  */
#define    RW_DECELERATION_TIME_NO_7_UPPER                  0x068EU    /*  */
#define    RW_DECELERATION_TIME_NO_7_LOWER                  0x068FU    /*  */
#define    RW_TORQUE_LIMITING_NO_0_UPPER                    0x0700U    /*  */
#define    RW_TORQUE_LIMITING_NO_0_LOWER                    0x0701U    /*  */
#define    RW_TORQUE_LIMITING_NO_1_UPPER                    0x0702U    /*  */
#define    RW_TORQUE_LIMITING_NO_1_LOWER                    0x0703U    /*  */
#define    RW_TORQUE_LIMITING_NO_2_UPPER                    0x0704U    /*  */
#define    RW_TORQUE_LIMITING_NO_2_LOWER                    0x0705U    /*  */
#define    RW_TORQUE_LIMITING_NO_3_UPPER                    0x0706U    /*  */
#define    RW_TORQUE_LIMITING_NO_3_LOWER                    0x0707U    /*  */
#define    RW_TORQUE_LIMITING_NO_4_UPPER                    0x0708U    /*  */
#define    RW_TORQUE_LIMITING_NO_4_LOWER                    0x0709U    /*  */
#define    RW_TORQUE_LIMITING_NO_5_UPPER                    0x070AU    /*  */
#define    RW_TORQUE_LIMITING_NO_5_LOWER                    0x070BU    /*  */
#define    RW_TORQUE_LIMITING_NO_6_UPPER                    0x070CU    /*  */
#define    RW_TORQUE_LIMITING_NO_6_LOWER                    0x070DU    /*  */
#define    RW_TORQUE_LIMITING_NO_7_UPPER                    0x070EU    /*  */
#define    RW_TORQUE_LIMITING_NO_7_LOWER                    0x070FU    /*  */

/*
 * Parameter R/W commands [User parameters]
 */
#define    RW_JOG_OPERATING_SPEED_UPPER                     0x0286U    /*  */
#define    RW_JOG_OPERATING_SPEED_LOWER                     0x0287U    /*  */
#define    RW_MOTOR_ROTATION_DIR_SELECTION_UPPER            0x0384U    /*  */
#define    RW_MOTOR_ROTATION_DIR_SELECTION_LOWER            0x0385U    /*  */
#define    RW_LOAD_HOLDING_FUNCTION_SELECTION_UPPER         0x102AU    /*  */
#define    RW_LOAD_HOLDING_FUNCTION_SELECTION_LOWER         0x102BU    /*  */
#define    RW_LOAD_HOLDING_TORQUE_LIMIT_SETTING_VALUE_UPPER 0x1030U    /*  */
#define    RW_LOAD_HOLDING_TORQUE_LIMIT_SETTING_VALUE_LOWER 0x1031U    /*  */
#define    RW_OPERATION_INPUT_MODE_SELECTION_UPPER          0x1040U    /*  */
#define    RW_OPERATION_INPUT_MODE_SELECTION_LOWER          0x1041U    /*  */
#define    RW_JOG_OPERATION_TORQUE_UPPER                    0x1042U    /*  */
#define    RW_JOG_OPERATION_TORQUE_LOWER                    0x1043U    /*  */
#define    RW_SPEED_REDUCTION_RATIO_UPPER                   0x104AU    /*  */
#define    RW_SPEED_REDUCTION_RATIO_LOWER                   0x104BU    /*  */
#define    RW_SPEED_REDUCTION_RATIO_DEC_DIGIT_SETTING_UPPER 0x104CU    /*  */
#define    RW_SPEED_REDUCTION_RATIO_DEC_DIGIT_SETTING_LOWER 0x104DU    /*  */
#define    RW_SPEED_INCREASING_RATIO_UPPER                  0x104EU    /*  */
#define    RW_SPEED_INCREASING_RATIO_LOWER                  0x104FU    /*  */
#define    RW_CONVEYOR_SPEED_REDUCTION_RATIO_UPPER          0x1050U    /*  */
#define    RW_CONVEYOR_SPEED_REDUCTION_RATIO_LOWER          0x1051U    /*  */
#define    RW_CONVEYOR_SPEED_REDUCTION_RATIO_DEC_DIGIT_SETTING_UPPER    0x1052U    /*  */
#define    RW_CONVEYOR_SPEED_REDUCTION_RATIO_DEC_DIGIT_SETTING_LOWER    0x1053U    /*  */
#define    RW_CONVEYOR_SPEED_INCREASING_RATIO_UPPER         0x1054U    /*  */
#define    RW_CONVEYOR_SPEED_INCREASING_RATIO_LOWER         0x1055U    /*  */
#define    RW_ANALOG_INPUT_SIGNAL_SELECTION_UPPER           0x10E2U    /*  */
#define    RW_ANALOG_INPUT_SIGNAL_SELECTION_LOWER           0x10E3U    /*  */
#define    RW_ROTATION_SPEED_ATTAINMENT_BAND_UPPER          0x114EU    /*  */
#define    RW_ROTATION_SPEED_ATTAINMENT_BAND_LOWER          0x114FU    /*  */

/*
 *Parameters R/W commands [Alarm, warning]
 */
#define    RW_UNDERVOLTAGE_WARNING_LEVEL_UPPER              0x0348U    /* Sets the warning level for the undervoltage of the main power supply.  */
#define    RW_UNDERVOLTAGE_WARNING_LEVEL_LOWER              0x0349U    /*  */
#define    RW_ELECTROMAGNETIC_BRAKE_ACTION_AT_ALARM_UPPER   0x1080U    /* Sets the timing to activate the electromagnetic brake when an alarm is generated. If "0" is set, the electromagnetic brake activates to hold the position after the motor coasts to a stop. */
#define    RW_ELECTROMAGNETIC_BRAKE_ACTION_AT_ALARM_LOWER   0x1081U    /*  */
#define    RW_PREVENTION_OPERATION_POWER_ALARM_FUNC_UPPER   0x1082U    /* Switches whether to enable or disable the prevention of operation at power-on alarm. */
#define    RW_PREVENTION_OPERATION_POWER_ALARM_FUNC_LOWER   0x1083U    /*  */
#define    RW_UNDERVOLTAGE_ALARM_LATCH_UPPER                0x1084U    /* Switches whether to enable or disable the retaining state of the undervoltage alarm function. If "0" is set, the undervoltage alarm will automatically be cleared when the main power supply voltage returns to the undervoltage detection level or more. */
#define    RW_UNDERVOLTAGE_ALARM_LATCH_LOWER                0x1085U    /*  */
#define    RW_OVERLOAD_WARNING_FUNCTION_UPPER               0x10A2U    /* Switches whether to enable or disable the overload warning function. */
#define    RW_OVERLOAD_WARNING_FUNCTION_LOWER               0x10A3U    /*  */
#define    RW_UNDERVOLTAGE_WARNING_FUNCTION_UPPER           0x10A8U    /* Switches whether to enable or disable the undervoltage warning function. */
#define    RW_UNDERVOLTAGE_WARNING_FUNCTION_LOWER           0x10A9U    /*  */
#define    RW_OVERLOAD_WARNING_LEVEL_UPPER                  0x10AAU    /* Sets the warning level for the load torque of the motor.  */
#define    RW_OVERLOAD_WARNING_LEVEL_LOWER                  0x10ABU    /*  */
#define    RW_COMMUNICATION_TIMEOUT_UPPER                   0x1200U    /* Sets the condition in which a communication timeout occurs in RS-485 communication. */
#define    RW_COMMUNICATION_TIMEOUT_LOWER                   0x1201U    /*  */
#define    RW_COMMUNICATION_ERROR_ALARM_UPPER               0x1202U    /* Sets the condition in which a RS-485 communication error alarm is generated. The communication error alarm is generated when the RS-485 communication error has occurred by the number of times set here. */
#define    RW_COMMUNICATION_ERROR_ALARM_LOWER               0x1203U    /*  */
#define    RW_COMMUNICATION_PARITY_UPPER                    0x1406U    /* Sets the parity for RS-485 communication. */
#define    RW_COMMUNICATION_PARITY_LOWER                    0x1407U    /*  */
#define    RW_COMMUNICATION_STOP_BIT_UPPER                  0x1408U    /* Sets the stop bit for RS-485 communication. */
#define    RW_COMMUNICATION_STOP_BIT_LOWER                  0x1409U    /*  */
#define    RW_TRANSMISSION_WAITING_TIME_UPPER               0x140AU    /* Sets the transmission waiting time for RS-485 communication. */

/*
 * Operation commands
 * These are commands related to motor operation. Operation commands are not saved in the non-volatile memory
 */
#define    RW_GROUP_UPPER                                   0x0030U    /* Sets the address number for the group send. */
#define    RW_GROUP_LOWER                                   0x0031U    /*  */
#define    RW_DRIVER_INPUT_COMMAND_UPPER                    0x007CU    /* Sets the input command to the driver. */
#define    R_DRIVER_INPUT_COMMAND_LOWER                     0x007DU    /*  */
#define    R_DRIVER_OUTPUT_COMMAND_UPPER                    0x007EU    /* Reads the output status of the driver. */
#define    R_DRIVER_OUTPUT_COMMAND_LOWER                    0x007FU    /*  */

#define MOTOR_STOP_BIT 	   	                                0b00000000
#define MOTOR_FORWARD_BIT                                   0b00001000
#define MOTOR_REVERSE_BIT                                   0b00010000
#define MOTOR_SLOW_CHANGE_BIT                               0b00100000
#define MOTOR_FREE_ON_STOP_BIT                              0b10000000

#define MOTOR_DIRECTOIN_STOP                                0
#define MOTOR_DIRECTOIN_FORWARD                             1
#define MOTOR_DIRECTOIN_REVERSE                             2

#define MODE_NO_0                                           0x0000
#define MODE_NO_1                                           0x0001
#define MODE_NO_2                                           0x0002
#define MODE_NO_3                                           0x0003
#define MODE_NO_4                                           0x0004
#define MODE_NO_5                                           0x0005
