# Receiver status - page 858

bool OK = 0
bool Valid = 0
bool No_error = 0
bool No_overrun = 0

bool Warning = 1
bool Error = 1
bool Failure = 1
bool Overrun = 1
bool Invalid = 1

# N0
bool error_flag

bool Within_specifications = 0
bool temperature_status
bool voltage_supply_status

bool Powered = 0
bool Not_powered = 1
bool primary_antenna_power_status

# N1
bool LNA_failure

bool Open_and_Antenna_disconnected = 1
bool primary_antenna_open_circuit_flag

bool Short_circuit_detected = 1
bool primary_antenna_short_circuit_flag

bool No_overload = 0
bool Overload = 1
bool CPU_overload_flag

# N2
bool COM_port_transmit_buffer_overrun

bool Not_detected = 0
bool Detected = 1
bool spoofing_detection_status

bool reserved
bool link_overrun_flag

# N3
bool input_overrun_flag
bool aux_transmit_overrun_flag

bool Out_of_range = 1
bool antenna_gain_state

bool Jammer_Detected = 1
bool jammer_detected

# N4
bool No_INS_reset = 0
bool INS_reset = 1
bool INS_reset_flag

bool No_IMU_communication = 1
bool IMU_communication_failure

bool GPS_almanac_flag_UTC_known
bool position_solution_flag

# N5
bool Not_fixed = 0
bool Fixed = 1
bool position_fixed_flag

bool Enabled_clock_steering = 0
bool Disabled_clock_steering = 1
bool clock_steering_status

bool clock_model_flag

bool UnLocked = 0
bool Locked = 1
bool external_oscillator_locked_flag

# N6
bool software_resource

bool OEM6_or_earlier_format = 0
bool OEM7_format = 1
bool status_error_format_version_bit

bool version_bit_1

bool Normal_tracking = 0
bool HDR_tracking = 1
bool tracking_mode

# N7
bool Disables_digital_filtering = 0
bool Enables_digital_filtering = 1
bool digital_filtering_enabled

bool No_event = 0
bool Event = 1
bool auxiliary_3_status_event_flag
bool auxiliary_2_status_event_flag
bool auxiliary_1_status_event_flag