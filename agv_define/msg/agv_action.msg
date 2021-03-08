# Synchronize with agvlib.h
Header header

uint8 action
uint8 ACTION_FLOAT              = 0   # Do not Nothing
uint8 ACTION_MANUAL             = 1   # Control agv by manual

uint8 ACTION_INITIAL_POSE       = 5   # Set Initial pose of Sim1000
uint8 ACTION_QUALITY_POSE       = 6   # Set Quality pose of Sim1000
uint8 ACTION_SAVE_POSE          = 7   # Save current pose of sim1000 for initial pose 

uint8 ACTION_UPDATE_MAP         = 10  # Call Map to agv

uint8 ACTION_UPDATE_PARAM       = 15  # Update Parameter from web to agv
uint8 ACTION_SAVE_PARAM         = 16  # Save Parameter from web
uint8 ACTION_LOAD_PARAM_DEFAULT = 17  # Load Parameter default from web
uint8 ACTION_LOAD_PARAM_RUNTIME = 18  # Load Parameter runtime from web

uint8 ACTION_ENCODE_FACE        = 20  # Encode a face
uint8 ACTION_RECOGNIZE_FACE     = 21  # Recognie a face
uint8 ACTION_DELETE_ENCODED     = 22  # Delete a Encoded face 

uint8 ACTION_RESTART_NS3        = 30  # Restart NanoScan3 (front and back)
uint8 ACTION_SAFETY_DEFAULT     = 31  # Set default for safety
uint8 ACTION_BYPASS_SAFETY      = 32  # By pass the safety
uint8 ACTION_MAPPING_SAFETY     = 33  # Safety field for mapping mode

uint8 ACTION_ROTATE_GOAL        = 100 # Action move_base - Rotate agv to goal
uint8 ACTION_MOVE_GOAL          = 101 # Action move_base - Move agv to goal

uint8 ACTION_CHARGING_IN        = 105 # Action Charger - move IN
uint8 ACTION_CHARGING_OUT       = 106 # Action Charger - move OUT

uint8 ACTION_LIFT_IN            = 110 # Action Lift - move IN
uint8 ACTION_LIFT_UP            = 111 # Action Lift - lift UP
uint8 ACTION_LIFT_DOWN          = 112 # Action Lift - lift DOWN
uint8 ACTION_LIFT_OUT           = 113 # Action Lift - move OUT

uint8 ACTION_NOD_HEAD_POS       = 115 # Action Nod - nod with position
uint8 ACTION_NOD_THE_HEAD       = 116 # Action Nod - nod the head
uint8 ACTION_NOD_SET_ORIGIN     = 117 # Action Nod - Set Origin of nod action

uint8 state
uint8 STATE_EXECUTE         = 0   # Excute action
uint8 STATE_CANCEL          = 1   # Cancel action
uint8 STATE_DONE            = 2   # Action done

uint8 status
uint8 STATUS_PENDING        = 0   # The goal has yet to be processed by the action server
uint8 STATUS_ACTIVE         = 1   # The goal is currently being processed by the action server
uint8 STATUS_PREEMPTED      = 2   # The goal received a cancel request after it started executing and has since completed its execution (Terminal State)
uint8 STATUS_SUCCEEDED      = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 STATUS_ABORTED        = 4   # The goal was aborted during execution by the action server due to some failure (Terminal State)
uint8 STATUS_REJECTED       = 5   # The goal was rejected by the action server without being processed, because the goal was unattainable or invalid (Terminal State)
uint8 STATUS_PREEMPTING     = 6   # The goal received a cancel request after it started executing and has not yet completed execution
uint8 STATUS_RECALLING      = 7   # The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled
uint8 STATUS_RECALLED       = 8   # The goal received a cancel request before it started executing and was successfully cancelled (Terminal State)
uint8 STATUS_LOST           = 9   # An action client can determine that a goal is LOST. This should not be sent over the wire by an action server

#Allow for the user to associate a string with GoalStatus for debugging
string action_id 

string type
string data