######################################################
#                MOVE IN WORK SPACE                  #
######################################################

# SUPPORTED_FRAMES
int8 WORLD = 0
int8 BASE  = 1
int8 TCP   = 2

# TRACKING_TYPE
int8 TT_VEL=0   # Tracking Type - Velocity
int8 TT_TIME=1  # Tracking Type - Time

# BLEND_POINT
int8 BP_STOP=0  # Blend Point - Stop
int8 BP_VIA=1   # Blend Point - Via

# BLEND_TYPE
int8 BT_ACC=0   # Blend Type - Acceleration
int8 BT_TIME=1  # Blend Type - Time
int8 BT_RAD=2   # Blend Type - Radius

float64[3] pos      # position - x, y, z [mm]
float64[3] rot      # orientation - r, p, y [degrees]
int8       ref      # see SUPPORTED_FRAMES or use custom
int8       ttype    # see TRACKING_TYPE 
float64    tvalue   # tracking value (in mm/s or seconds)
int8       bpoint   # see BLEND_POINT
int8       btype    # see BLEND_TYPE
float64    bvalue   # blend value (in m/s^2, seconds or mm)