#ifndef AGVLIB_H
#define AGVLIB_H

// Synchronize with agv_action.msg in agv_define pkg
enum ActionName {
    ACTION_FLOAT                = 0,    // Do not Nothing
    ACTION_MANUAL               = 1,    // Control agv by manual
    
    ACTION_INITIAL_POSE         = 5,    // Set Initial pose of Sim1000
    ACTION_QUALITY_POSE         = 6,    // Set Quality pose of Sim1000
    ACTION_SAVE_POSE            = 7,    // Save current pose of sim1000 for initial pose

    ACTION_UPDATE_MAP           = 10,   // Call Map to agv

    ACTION_UPDATE_PARAM         = 15,   // Update Parameter from web to agv
    ACTION_SAVE_PARAM           = 16,   // Save Parameter from web
    ACTION_LOAD_PARAM_DEFAULT   = 17,   // Load Parameter default from web
    ACTION_LOAD_PARAM_RUNTIME   = 18,   // Load Parameter runtime from web

    ACTION_ENCODE_FACE          = 20,   // Encode a face
    ACTION_RECOGNIZE_FACE       = 21,   // Recognie a face
    ACTION_DELETE_ENCODED       = 22,   // Delete a Encoded face

    ACTION_RESTART_NS3          = 30,   // Restart NanoScan3 (front and back) 
    ACTION_SAFETY_DEFAULT       = 31,   // Set default for safety
    ACTION_BYPASS_SAFETY        = 32,   // By pass the safety
    ACTION_MAPPING_SAFETY       = 33,   // Safety field for mapping mode

    ACTION_ROTATE_GOAL          = 100,  // Action move_base - Rotate agv to goal
    ACTION_MOVE_GOAL            = 101,  // Action move_base - Move agv to goal

    ACTION_CHARGING_IN          = 105,  // Action Charger - move IN
    ACTION_CHARGING_OUT         = 106,  // Action Charger - move OUT

    ACTION_LIFT_IN              = 110,  // Action Lift - move IN
    ACTION_LIFT_UP              = 111,  // Action Lift - lift UP
    ACTION_LIFT_DOWN            = 112,  // Action Lift - lift DOWN
    ACTION_LIFT_OUT             = 113,  // Action Lift - move OUT

    ACTION_NOD_HEAD_POS         = 115,  // Action Nod - nod with position
    ACTION_NOD_THE_HEAD         = 116,  // Action Nod - nod the head
    ACTION_NOD_SET_ORIGIN       = 117,  // Action Nod - set origin of nod action
};

enum ActionState {
    STATE_EXECUTE       = 0,    // Execute the action
    STATE_CANCEL        = 1,    // Cancel the action
    STATE_DONE          = 2,    // Action done
};

enum ActionStatus {
    PENDING      = 0,    // The goal has yet to be processed by the action server
    ACTIVE       = 1,    // The goal is currently being processed by the action server
    PREEMPTED    = 2,    // The goal received a cancel request after it started executing and has since completed its execution (Terminal State)
    SUCCEEDED    = 3,    // The goal was achieved successfully by the action server (Terminal State)
    ABORTED      = 4,    // The goal was aborted during execution by the action server due to some failure (Terminal State)
    REJECTED     = 5,    // The goal was rejected by the action server without being processed, because the goal was unattainable or invalid (Terminal State)
    PREEMPTING   = 6,    // The goal received a cancel request after it started executing and has not yet completed execution
    RECALLING    = 7,    // The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled
    RECALLED     = 8,    // The goal received a cancel request before it started executing and was successfully cancelled (Terminal State)
    LOST         = 9,    // An action client can determine that a goal is LOST. This should not be sent over the wire by an action server
};

// Synchronize with robot_dynparam.cfg in agv_define pkg
struct robotDynparamStruct {
    double L                    = 0.500;    // (float - m) Distance between two wheels
    double R                    = 0.085;   // (float - m) Wheel radius (in meters per radian)
    int K                       = 30;       // (int)       He so banh rang dong co
    double SpeedMotorMax        = 4000;     // (int - rpm) Speed maximum of moter before gear
    double SpeedMotorMin        = 80;       // (int - rpm) Speed maximum of moter before gear
    int THRESHOLD_QUALITY_POSE  = 60;       // (int - %)   Threshold of quality pose allow to run Navigation
    int CHARGER_TIME_OUT        = 300;      // (int - s)   Timeout for action Charging
    int NOD_POSE_TIME_OUT       = 3;        // (int - s)   Timeout for action Nod with position
    int NOD_HEAD_TIME_OUT       = 300;      // (int - s)   Timeout for action Nod the head
};

struct ns3StatusStruct {
    bool isCleanWarn        = false;
    bool isCleanError       = false;
    bool isNs3Error         = false;
    bool isDetectField      = false;
    bool isWarnField        = false;
    bool isBrakeField       = false;
    bool isPowerField       = false;
};
struct flexisoftStatusStruct {
    bool isflexisoftError       = false;
};
struct plcStatusStruct {
    bool isPlcError             = false;
    bool isEmegencyStop         = true;
    bool isNodAlarm             = false;
    bool isNodSetOrigin         = true;
    bool isBatteryLow           = false;
    bool isChargerSensor        = false;
    bool isPowerMotorOn         = true;
    bool isRelayForwardOn       = true;
    bool isRelayBackwardOn      = true;
};
struct raspiStatusStruct {
    bool isRaspiError           = false;
};
struct agvStatusStruct {
    bool isSystemGood           = true;
    bool isQualityGood          = true;
    bool isConnectServer        = true;
    ns3StatusStruct ns3_front;
    ns3StatusStruct ns3_back;
    plcStatusStruct plc;
    flexisoftStatusStruct flexisoft;
    raspiStatusStruct raspi;
};
enum AgvStatus {
    EMERGENCY_STOP      = 0,    // RED
    SYSTEM_READY        = 1,    // GREEN
    WAITING_USER        = 2,    // WHITE
    NAVIGATION          = 3,    // CYAN
    CHARGING            = 4,    // RAINBOW
    CANCEL_ACTION       = 5,    // YELLOW
    MANUAL              = 6,    // BLUE
    SERVER_ERROR        = 7,    // YELLOW BLINK
    SYSTEM_ERROR        = 8,    // RED BLINK
    QUALITY_LOW         = 9,    // CYAN BLINK

    NS3_FRONT_ERROR     = 50,
    NS3_BACK_ERROR      = 51,
    PLC_ERROR           = 52,
    NOD_ALARM           = 53,
    BATTERY_LOW         = 54,
    POWER_MOTOR         = 55,
    RASPI_ERROR         = 56,
    FLEXISOFT_ERROR     = 57,
};

#endif

