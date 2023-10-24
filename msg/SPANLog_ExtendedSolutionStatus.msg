# Solution status - page 1060

bool Unused = 0
bool Used = 1

# N0
bool position_update
bool phase_update
bool zero_velocity_update
bool wheel_sensor_update

# N1
bool ALIGN_heading_update
bool external_position_update

bool Not_converged = 0
bool Converged = 1
bool INS_solution_convergence_flag

bool doppler_update

# N2
bool pseudorange_update
bool velocity_update
bool reserved_bit_10
bool dead_reckoning_update

# N3
bool phase_wind_up_update
bool course_over_ground_update
bool external_velocity_update
bool external_attitude_update

# N4
bool external_heading_update
bool external_height_update
bool reserved_bit_18
bool reserved_bit_19

# N5
bool reserved_bit_20
bool reserved_bit_21

bool secondary_INS_solution
bool reserved_bit_23

# N6-7
bool Biases_not_estimated = 0
bool Biases_estimated = 1
bool turn_on_biases_estimated

bool Not_verified = 0
bool Verified = 1
bool alignment_direction_verified

uint8 Incomplete_Alignment = 0
uint8 Static = 1
uint8 Kinematic = 2
uint8 Dual_Antenna = 3
uint8 User_Command = 4
uint8 NVM_Seed = 5
uint8 alignment_indication

uint8 Not_Injected = 0
uint8 Valid_Not_Found = 1
uint8 Failed_Validation_and_Discarded = 2
uint8 Awaiting_Validation = 3
uint8 Alignment_Data_Injected = 4
uint8 Ignored = 5
uint8 Error_Model_Data_Injected = 6
uint8 NVM_seed_indication