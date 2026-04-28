######################################################
#                 ROBOT STATE MSG                    #
######################################################

RobotMode   robot_mode 
RobotState  robot_state  

float64[7]  sensed_pos  # sensed joint configuration
float64[7]  sensed_vel  # sensed joint velocity
float64[7]  sensed_trq  # sensed joint torque

float64[7]  target_pos  # target joint configuration
float64[7]  target_vel  # target joint configuration
float64[7]  target_trq  # target joint configuration

float64[3]  pos		# position of TCP
float64[3]  rot         # orientation of TCP

# IOBoard - Outputs

int8[8]     iob_digital_out   # IOBoard digital outputs
int8[4]     iob_relays        # IOBoard relays
float32[2]  iob_current_out   # IOBoard current outputs
float32[2]  iob_voltage_out   # IOBoard voltage outputs

# IOBoard - Inputs

int8[16]    iob_digital_in    # IOBoard digital inputs
int8[4]     iob_safe_in       # IOBoard safe inputs
float32[2]  iob_current_in    # IOBoard current inputs
float32[2]  iob_voltage_in    # IOBoard voltage inputs
Quadrature[2] iob_quadratures # IOBoard quadratures

# Tool IO - Outputs

int8[4]     tio_digital_out   # Tool IO digital outputs
int8[3]     tio_power_supply  # Tool IO pose supply
float32[2]  tio_analog_out    # Tool IO analog outputs

# Tool IO - Inputs

float32[1]  tio_current_in    # Tool IO current inputs
float32[4]  tio_voltage_in    # Tool IO voltage inputs

# Button flags

bool e_stop 
bool p_stop
bool toggle
bool backdrive 
