
# Synchronize with agvlib.h
Header header

uint8 agv_status
uint8 EMERGENCY_STOP    = 0   # RED
uint8 SYSTEM_READY      = 1   # GREEN
uint8 WAITING_USER      = 2   # WHITE
uint8 NAVIGATION        = 3   # CYAN
uint8 CHARGING          = 4   # RAINBOW
uint8 CANCEL_ACTION     = 5   # YELLOW
uint8 MANUAL            = 6   # BLUE 
uint8 SERVER_ERROR      = 7   # YELLOW BLINK
uint8 SYSTEM_ERROR      = 8   # RED BLINK
uint8 QUALITY_LOW       = 9   # CYAN BLINK

uint8 NS3_FRONT_ERROR   = 50
uint8 NS3_BACK_ERROR    = 51
uint8 PLC_ERROR         = 52
uint8 NOD_ALARM         = 53
uint8 BATTERY_LOW       = 54
uint8 POWER_MOTOR       = 55
uint8 RASPI_ERROR       = 56

uint8 frequency 
string data