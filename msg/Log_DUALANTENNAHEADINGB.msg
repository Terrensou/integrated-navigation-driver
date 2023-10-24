# page 532
std_msgs/Header header

BinaryHeader log_header

# INS status
Log_SolutionStatus solution_status

#Position type
Log_PositionVelocityType position_type

#Data
float64 length

float64 heading
float64 pitch
float64 reserved

float64 heading_std
float64 pitch_std

string station_id

uint8 satellites_tracked
uint8 satellites_solutions_used
uint8 satellites_above_elevation_mask
uint8 satellites_multi_frequency_used

Log_SolutionSource solution_source

#Extended solution status
Log_ExtendedSolutionStatus extended_solution_status

Log_GalileoBeiDouSignalUsedMask signal_mask_galileo_beidou
Log_GPSGLONASSSignalUsedMask signal_mask_gps_glonass