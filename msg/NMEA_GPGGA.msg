Header  header

# UTC time
string utc_time

string latitude
string LATITUDE_NORTH = N
string LATITUDE_SOUTH = S
string latitude_direction

string longitude
string LONGITUDE_EAST = E
string LONGITUDE_WEST = W
string longitude_direction

uint8 GNSS_FIX_INVALID = 0
uint8 GNSS_SINGLE_POINT = 1
uint8 GNSS_PSEUDORANGE_DIFFERENTIAL = 2
uint8 GNSS_FIX_SOLUTION = 4
uint8 GNSS_FLOATING_SOLUTION = 5
uint8 GNSS_RECKONING_MODE = 6
uint8 GNSS_MANUAL_FIXED = 7
uint8 GNSS_SIMULATOR_MODE = 8
uint8 GNSS_WAAS = 9
uint8 gnss_quality

uint8 number_of_using_satellites

float64 horizontal_dilution_of_precision

string UNITS_KILOMETER = KM
string UNITS_METER = M
string UNITS_DECIMETER = DM
string UNITS_CENTIMETER = CM

float64 altitude
string altitude_units

float64 undulation
string undulation_units

uint8 differential_age
string differential_station_id