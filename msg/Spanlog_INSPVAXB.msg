std_msgs/Header header

Spanlog_BinaryHeader log_header

# INS status
uint8 INS_INACTIVE = 0
uint8 INS_ALIGNING = 1
uint8 INS_HIGH_VARIANCE = 2
uint8 INS_SOLUTION_GOOD = 3
uint8 INS_SOLUTION_FREE = 6
uint8 INS_ALIGNMENT_COMPLETE = 7
uint8 DETERMINING_ORIENTATION = 8
uint8 WAITING_INITIALPOS = 9
uint8 WAITING_AZIMUTH = 10
uint8 INITIALIZING_BIASES = 11
uint8 MOTION_DETECT = 12
uint8 WAITING_ALIGNMENTORIENTATION = 14
uint8 ins_status

#Position type
uint8 NONE = 0
uint8 FIXEDPOS = 1
uint8 FIXEDHEIGHT = 2
uint8 DOPPLER_VELOCITY = 8
uint8 SINGLE = 16
uint8 PSRDIFF = 17
uint8 WAAS = 18
uint8 PROPAGATED = 19
uint8 L1_FLOAT = 32
uint8 NARROW_FLOAT = 34
uint8 L1_INT = 48
uint8 WIDE_INT = 49
uint8 NARROW_INT = 50
uint8 RTK_DIRECT_INS = 51
uint8 INS_SBAS = 52
uint8 INS_PSRSP = 53
uint8 INS_PSRDIFF = 54
uint8 INS_RTKFLOAT = 55
uint8 INS_RTKFIXED = 56
uint8 PPP_CONVERGING = 68
uint8 PPP = 69
uint8 OPERATIONAL = 70
uint8 WARNING = 71
uint8 OUT_OF_BOUNDS = 72
uint8 INS_PPP_CONVERGING = 73
uint8 INS_PPP = 74
uint8 PPP_BASIC_CONVERGING = 77
uint8 PPP_BASIC = 78
uint8 INS_PPP_BASIC_CONVERGING = 79
uint8 INS_PPP_BASIC = 80
uint8 position_type

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
string extended_solution_status

uint16 time_since_update