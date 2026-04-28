######################################################
#                   SAFETY EVENT                     #
######################################################

# SUPPORTED_TYPES

int8 TYPE_ESTOP = 1
int8 TYPE_PSTOP = 2
int8 TYPE_SSTOP = 3

# SUPPORTED_CODES

int32 CODE_EXT_E_STOP_ACTIVATED              = 1
int32 CODE_EXT_P_STOP_ACTIVATED              = 2
int32 CODE_IOB_NOT_RESPONDING                = 3
int32 CODE_JBS_NOT_RESPONDING                = 4
int32 CODE_J_RATED_X_SENSOR_POS_SPAN         = 5
int32 CODE_J_RATED_POS_DELTA_SPAN            = 6
int32 CODE_J_RATED_SPEED_EXCEEDED            = 7
int32 CODE_J_RATED_TORQUE_EXCEEDED           = 8
int32 CODE_J_RATED_STATIC_TORQUE_EXCEEDED    = 9
int32 CODE_J_RATED_TEMPERATURE_EXCEEDED      = 10
int32 CODE_J_RATED_TORQUE_DEVIATION_EXCEEDED = 11
int32 CODE_MODEL_JS_SPAN_EXCEEDED            = 12
int32 CODE_MODEL_WS_SPAN_EXCEEDED            = 13
int32 CODE_MODEL_FRAME_SPEED_EXCEEDED        = 14
int32 CODE_TRQ_MODE_JSPEED_EXCEEDED          = 15

int32 CODE_INVALID_MODEL_STATE               = 21
int32 CODE_J_MODEL_VELOCITY_EXCEEDED         = 22
int32 CODE_J_MODEL_TORQUE_EXCEEDED           = 23
int32 CODE_J_MODEL_STATIC_TORQUE_EXCEEDED    = 24
int32 CODE_J_MODEL_LIMIT_ESTIMATION          = 25

int8       type       # see SUPPORTED_TYPES
int32      code       # see SUPPORTED_CODES
uint32       timestamp  # event timestamp
