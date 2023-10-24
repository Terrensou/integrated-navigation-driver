#INSPVAX header

string port
int32 sequence

float32 idle_time

uint8 UNKNOWN = 20
uint8 APPROXIMATE = 60
uint8 COARSEADJUSTING = 80
uint8 COARSE = 100
uint8 COARSESTEERING = 120
uint8 FREEWHEELING = 130
uint8 FINEADJUSTING = 140
uint8 FINE = 160
uint8 FINEBACKUPSTEERING = 170
uint8 FINESTEERING = 180
uint8 SATTIME = 200
uint8 time_status

uint32 week
float64 seconds

Log_ReceiverStatus receiver_status
uint32 reserved
uint32 receiver_sw_version