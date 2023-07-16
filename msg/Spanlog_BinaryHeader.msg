#INSPVAX header
uint8 Binary_Format = 0
uint8 ASCII_Format = 1
uint8 Abbreviated_ASCII_NMEA_Format = 10
uint8 Reserved = 11
uint8 Original_Message = 0
uint8 Response_Message = 1
uint8[2] message_type

uint16 NO_PORTS = 0
uint16 COM1 = 32
uint16 COM2 = 64
uint16 COM3 = 96
uint16 SPECIAL = 160
uint16 THISPORT = 192
uint16 FILE = 224
uint16 USB1 = 1440
uint16 USB2 = 1696
uint16 USB3 = 1952
uint16 AUX = 2208
uint16 COM4 = 2976
uint16 ETH1 = 3232
uint16 IMU = 3488
uint16 ICOM1 = 4000
uint16 ICOM2 = 4256
uint16 ICOM3 = 4512
uint16 NCOM1 = 4768
uint16 NCOM2 = 5024
uint16 NCOM3 = 5280
uint16 ICOM4 = 5536
uint16 WCOM1 = 5792
uint16 COM5 = 6048
uint16 COM6 = 6304
uint16 BT1 = 6560
uint16 COM7 = 6816
uint16 COM8 = 7072
uint16 COM9 = 7328
uint16 COM10 = 7584
uint16 CCOM1 = 7840
uint16 CCOM2 = 8096
uint16 CCOM3 = 8352
uint16 CCOM4 = 8608
uint16 CCOM5 = 8864
uint16 CCOM6 = 9120
uint16 ICOM5 = 9888
uint16 ICOM6 = 10144
uint16 ICOM7 = 10400
uint16 SCOM1 = 10656
uint16 SCOM2 = 10912
uint16 SCOM3 = 11168
uint16 SCOM4 = 11424
uint16 port_address

uint16 message_length
int32 sequence

float32 idle_time

uint8 UNKNOWN = 20
uint8 APPROXIMATE = 20
uint8 COARSEADJUSTING = 20
uint8 COARSE = 20
uint8 COARSESTEERING = 20
uint8 FREEWHEELING = 20
uint8 FINEADJUSTING = 20
uint8 FINE = 20
uint8 FINEBACKUPSTEERING = 20
uint8 FINESTEERING = 20
uint8 SATTIME = 20
uint8 time_status

uint16 week
uint64 milliseconds

string receiver_status
string reserved
uint32 receiver_sw_version