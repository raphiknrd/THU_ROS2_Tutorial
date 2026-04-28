######################################################
#               MOVE IN JOINT SPACE                  #
######################################################

# TRACKING_TYPE
int8 TT_VEL=0   # Tracking Type - Velocity
int8 TT_TIME=1  # Tracking Type - Time

# BLEND_POINT
int8 BP_STOP=0  # Blend Point - Stop
int8 BP_VIA=1   # Blend Point - Via

# BLEND_TYPE
int8 BT_ACC=0   # Blend Type - Acceleration
int8 BT_TIME=1  # Blend Type - Time

# CHAINING
int8 CH_INT=0   # Chaining - Interrupt previous motion
int8 CH_COM=1   # Chaining - Complete previous motion

float64[7] jsconf   # target joint configuration [degrees]
int8       ttype    # see TRACKING_TYPE
float64    tvalue   # tracking value (in deg/s or seconds)
int8       bpoint   # see BLEND_POINT
int8       btype    # see BLEND_TYPE
float64    bvalue   # blend value (in deg/s^2 or seconds)

