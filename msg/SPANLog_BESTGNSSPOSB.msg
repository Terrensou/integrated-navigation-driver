# page 1033
std_msgs/Header header

BinaryHeader log_header

# Solution status
Log_SolutionStatus solution_status

#Position type
Log_PositionVelocityType position_type

#Data
float64 latitude
float64 longitude
float64 height
float64 undulation

uint32 datum_WGS84 = 61
uint32 datum_USER = 63
uint32 datum_id

float64 latitude_std
float64 longitude_std
float64 height_std

char[4] base_station_id
float64 differential_age
float64 solution_age

uint8 satellites_tracked
uint8 satellites_solutions_used
uint8 satellites_L1E1B1_used
uint8 satellites_multi_frequency_used

uint8 reserved

#Extended solution status
Log_ExtendedSolutionStatus extended_solution_status

Log_GalileoBeiDouSignalUsedMask signal_mask_galileo_beidou
Log_GPSGLONASSSignalUsedMask signal_mask_gps_glonass