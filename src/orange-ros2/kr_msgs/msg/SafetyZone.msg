######################################################
#                    SAFETY ZONE                     #
######################################################

# SAFETY_IO
int8 SDIO01 = 1
int8 SDIO02 = 2
int8 SDIO03 = 3
int8 SDIO04 = 4

string          name         # user defined name
float64         speed        # rated speed [mm/s]
float64         buffer       # buffer size [mm]
float64[7]      sensitivity  # max torque dev [N.m]
int8            safety_io    # see SAFETY_IO
GeometryPlane[] planes       # list of planes