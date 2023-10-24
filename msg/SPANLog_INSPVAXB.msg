# page 1082
std_msgs/Header header

BinaryHeader log_header

# INS status
SPANLog_INSStatus ins_status

#Position type
Log_PositionVelocityType position_type

#Data
float64 latitude
float64 longitude
float64 height
float64 undulation

float64 north_velocity
float64 east_velocity
float64 up_velocity

float64 roll
float64 pitch
float64 azimuth

float64 latitude_std
float64 longitude_std
float64 height_std

float64 north_velocity_std
float64 east_velocity_std
float64 up_velocity_std

float64 roll_std
float64 pitch_std
float64 azimuth_std

#Extended solution status
SPANLog_ExtendedSolutionStatus extended_solution_status

uint16 time_since_update