// Generated by gencpp from file integrated_navigation_reader/SPANLog_BESTGNSSPOSB.msg
// DO NOT EDIT!


#ifndef INTEGRATED_NAVIGATION_READER_MESSAGE_SPANLOG_BESTGNSSPOSB_H
#define INTEGRATED_NAVIGATION_READER_MESSAGE_SPANLOG_BESTGNSSPOSB_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <integrated_navigation_reader/BinaryHeader.h>
#include <integrated_navigation_reader/Log_SolutionStatus.h>
#include <integrated_navigation_reader/Log_PositionVelocityType.h>
#include <integrated_navigation_reader/Log_ExtendedSolutionStatus.h>
#include <integrated_navigation_reader/Log_GalileoBeiDouSignalUsedMask.h>
#include <integrated_navigation_reader/Log_GPSGLONASSSignalUsedMask.h>

namespace integrated_navigation_reader
{
template <class ContainerAllocator>
struct SPANLog_BESTGNSSPOSB_
{
  typedef SPANLog_BESTGNSSPOSB_<ContainerAllocator> Type;

  SPANLog_BESTGNSSPOSB_()
    : header()
    , log_header()
    , solution_status()
    , position_type()
    , latitude(0.0)
    , longitude(0.0)
    , height(0.0)
    , undulation(0.0)
    , datum_id(0)
    , latitude_std(0.0)
    , longitude_std(0.0)
    , height_std(0.0)
    , base_station_id()
    , differential_age(0.0)
    , solution_age(0.0)
    , satellites_tracked(0)
    , satellites_solutions_used(0)
    , satellites_L1E1B1_used(0)
    , satellites_multi_frequency_used(0)
    , reserved(0)
    , extended_solution_status()
    , signal_mask_galileo_beidou()
    , signal_mask_gps_glonass()  {
      base_station_id.assign(0);
  }
  SPANLog_BESTGNSSPOSB_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , log_header(_alloc)
    , solution_status(_alloc)
    , position_type(_alloc)
    , latitude(0.0)
    , longitude(0.0)
    , height(0.0)
    , undulation(0.0)
    , datum_id(0)
    , latitude_std(0.0)
    , longitude_std(0.0)
    , height_std(0.0)
    , base_station_id()
    , differential_age(0.0)
    , solution_age(0.0)
    , satellites_tracked(0)
    , satellites_solutions_used(0)
    , satellites_L1E1B1_used(0)
    , satellites_multi_frequency_used(0)
    , reserved(0)
    , extended_solution_status(_alloc)
    , signal_mask_galileo_beidou(_alloc)
    , signal_mask_gps_glonass(_alloc)  {
  (void)_alloc;
      base_station_id.assign(0);
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::integrated_navigation_reader::BinaryHeader_<ContainerAllocator>  _log_header_type;
  _log_header_type log_header;

   typedef  ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator>  _solution_status_type;
  _solution_status_type solution_status;

   typedef  ::integrated_navigation_reader::Log_PositionVelocityType_<ContainerAllocator>  _position_type_type;
  _position_type_type position_type;

   typedef double _latitude_type;
  _latitude_type latitude;

   typedef double _longitude_type;
  _longitude_type longitude;

   typedef double _height_type;
  _height_type height;

   typedef double _undulation_type;
  _undulation_type undulation;

   typedef uint32_t _datum_id_type;
  _datum_id_type datum_id;

   typedef double _latitude_std_type;
  _latitude_std_type latitude_std;

   typedef double _longitude_std_type;
  _longitude_std_type longitude_std;

   typedef double _height_std_type;
  _height_std_type height_std;

   typedef boost::array<uint8_t, 4>  _base_station_id_type;
  _base_station_id_type base_station_id;

   typedef double _differential_age_type;
  _differential_age_type differential_age;

   typedef double _solution_age_type;
  _solution_age_type solution_age;

   typedef uint8_t _satellites_tracked_type;
  _satellites_tracked_type satellites_tracked;

   typedef uint8_t _satellites_solutions_used_type;
  _satellites_solutions_used_type satellites_solutions_used;

   typedef uint8_t _satellites_L1E1B1_used_type;
  _satellites_L1E1B1_used_type satellites_L1E1B1_used;

   typedef uint8_t _satellites_multi_frequency_used_type;
  _satellites_multi_frequency_used_type satellites_multi_frequency_used;

   typedef uint8_t _reserved_type;
  _reserved_type reserved;

   typedef  ::integrated_navigation_reader::Log_ExtendedSolutionStatus_<ContainerAllocator>  _extended_solution_status_type;
  _extended_solution_status_type extended_solution_status;

   typedef  ::integrated_navigation_reader::Log_GalileoBeiDouSignalUsedMask_<ContainerAllocator>  _signal_mask_galileo_beidou_type;
  _signal_mask_galileo_beidou_type signal_mask_galileo_beidou;

   typedef  ::integrated_navigation_reader::Log_GPSGLONASSSignalUsedMask_<ContainerAllocator>  _signal_mask_gps_glonass_type;
  _signal_mask_gps_glonass_type signal_mask_gps_glonass;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(datum_WGS84)
  #undef datum_WGS84
#endif
#if defined(_WIN32) && defined(datum_USER)
  #undef datum_USER
#endif

  enum {
    datum_WGS84 = 61u,
    datum_USER = 63u,
  };


  typedef boost::shared_ptr< ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator> const> ConstPtr;

}; // struct SPANLog_BESTGNSSPOSB_

typedef ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<std::allocator<void> > SPANLog_BESTGNSSPOSB;

typedef boost::shared_ptr< ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB > SPANLog_BESTGNSSPOSBPtr;
typedef boost::shared_ptr< ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB const> SPANLog_BESTGNSSPOSBConstPtr;

// constants requiring out of line definition

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator1> & lhs, const ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.log_header == rhs.log_header &&
    lhs.solution_status == rhs.solution_status &&
    lhs.position_type == rhs.position_type &&
    lhs.latitude == rhs.latitude &&
    lhs.longitude == rhs.longitude &&
    lhs.height == rhs.height &&
    lhs.undulation == rhs.undulation &&
    lhs.datum_id == rhs.datum_id &&
    lhs.latitude_std == rhs.latitude_std &&
    lhs.longitude_std == rhs.longitude_std &&
    lhs.height_std == rhs.height_std &&
    lhs.base_station_id == rhs.base_station_id &&
    lhs.differential_age == rhs.differential_age &&
    lhs.solution_age == rhs.solution_age &&
    lhs.satellites_tracked == rhs.satellites_tracked &&
    lhs.satellites_solutions_used == rhs.satellites_solutions_used &&
    lhs.satellites_L1E1B1_used == rhs.satellites_L1E1B1_used &&
    lhs.satellites_multi_frequency_used == rhs.satellites_multi_frequency_used &&
    lhs.reserved == rhs.reserved &&
    lhs.extended_solution_status == rhs.extended_solution_status &&
    lhs.signal_mask_galileo_beidou == rhs.signal_mask_galileo_beidou &&
    lhs.signal_mask_gps_glonass == rhs.signal_mask_gps_glonass;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator1> & lhs, const ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace integrated_navigation_reader

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b36b1ad9b26924585d3eadbc130c3aec";
  }

  static const char* value(const ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb36b1ad9b2692458ULL;
  static const uint64_t static_value2 = 0x5d3eadbc130c3aecULL;
};

template<class ContainerAllocator>
struct DataType< ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator> >
{
  static const char* value()
  {
    return "integrated_navigation_reader/SPANLog_BESTGNSSPOSB";
  }

  static const char* value(const ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# page 1033\n"
"std_msgs/Header header\n"
"\n"
"BinaryHeader log_header\n"
"\n"
"# Solution status\n"
"Log_SolutionStatus solution_status\n"
"\n"
"#Position type\n"
"Log_PositionVelocityType position_type\n"
"\n"
"#Data\n"
"float64 latitude\n"
"float64 longitude\n"
"float64 height\n"
"float64 undulation\n"
"\n"
"uint32 datum_WGS84 = 61\n"
"uint32 datum_USER = 63\n"
"uint32 datum_id\n"
"\n"
"float64 latitude_std\n"
"float64 longitude_std\n"
"float64 height_std\n"
"\n"
"char[4] base_station_id\n"
"float64 differential_age\n"
"float64 solution_age\n"
"\n"
"uint8 satellites_tracked\n"
"uint8 satellites_solutions_used\n"
"uint8 satellites_L1E1B1_used\n"
"uint8 satellites_multi_frequency_used\n"
"\n"
"uint8 reserved\n"
"\n"
"#Extended solution status\n"
"Log_ExtendedSolutionStatus extended_solution_status\n"
"\n"
"Log_GalileoBeiDouSignalUsedMask signal_mask_galileo_beidou\n"
"Log_GPSGLONASSSignalUsedMask signal_mask_gps_glonass\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: integrated_navigation_reader/BinaryHeader\n"
"#INSPVAX header\n"
"uint8 Binary_Format = 0\n"
"uint8 ASCII_Format = 1\n"
"uint8 Abbreviated_ASCII_NMEA_Format = 10\n"
"uint8 Reserved = 11\n"
"uint8 Original_Message = 0\n"
"uint8 Response_Message = 1\n"
"uint8[2] message_type\n"
"\n"
"uint16 NO_PORTS = 0\n"
"uint16 COM1 = 32\n"
"uint16 COM2 = 64\n"
"uint16 COM3 = 96\n"
"uint16 SPECIAL = 160\n"
"uint16 THISPORT = 192\n"
"uint16 FILE = 224\n"
"uint16 USB1 = 1440\n"
"uint16 USB2 = 1696\n"
"uint16 USB3 = 1952\n"
"uint16 AUX = 2208\n"
"uint16 COM4 = 2976\n"
"uint16 ETH1 = 3232\n"
"uint16 IMU = 3488\n"
"uint16 ICOM1 = 4000\n"
"uint16 ICOM2 = 4256\n"
"uint16 ICOM3 = 4512\n"
"uint16 NCOM1 = 4768\n"
"uint16 NCOM2 = 5024\n"
"uint16 NCOM3 = 5280\n"
"uint16 ICOM4 = 5536\n"
"uint16 WCOM1 = 5792\n"
"uint16 COM5 = 6048\n"
"uint16 COM6 = 6304\n"
"uint16 BT1 = 6560\n"
"uint16 COM7 = 6816\n"
"uint16 COM8 = 7072\n"
"uint16 COM9 = 7328\n"
"uint16 COM10 = 7584\n"
"uint16 CCOM1 = 7840\n"
"uint16 CCOM2 = 8096\n"
"uint16 CCOM3 = 8352\n"
"uint16 CCOM4 = 8608\n"
"uint16 CCOM5 = 8864\n"
"uint16 CCOM6 = 9120\n"
"uint16 ICOM5 = 9888\n"
"uint16 ICOM6 = 10144\n"
"uint16 ICOM7 = 10400\n"
"uint16 SCOM1 = 10656\n"
"uint16 SCOM2 = 10912\n"
"uint16 SCOM3 = 11168\n"
"uint16 SCOM4 = 11424\n"
"uint16 port_address\n"
"\n"
"uint16 message_length\n"
"int32 sequence\n"
"\n"
"float64 idle_time\n"
"\n"
"uint8 UNKNOWN = 20\n"
"uint8 APPROXIMATE = 60\n"
"uint8 COARSEADJUSTING = 80\n"
"uint8 COARSE = 100\n"
"uint8 COARSESTEERING = 120\n"
"uint8 FREEWHEELING = 130\n"
"uint8 FINEADJUSTING = 140\n"
"uint8 FINE = 160\n"
"uint8 FINEBACKUPSTEERING = 170\n"
"uint8 FINESTEERING = 180\n"
"uint8 SATTIME = 200\n"
"uint8 time_status\n"
"\n"
"uint16 week\n"
"uint64 milliseconds\n"
"\n"
"Log_ReceiverStatus receiver_status\n"
"string reserved\n"
"uint32 receiver_sw_version\n"
"================================================================================\n"
"MSG: integrated_navigation_reader/Log_ReceiverStatus\n"
"# Receiver status - page 858\n"
"\n"
"bool OK = 0\n"
"bool Valid = 0\n"
"bool No_error = 0\n"
"bool No_overrun = 0\n"
"\n"
"bool Warning = 1\n"
"bool Error = 1\n"
"bool Failure = 1\n"
"bool Overrun = 1\n"
"bool Invalid = 1\n"
"\n"
"# N0\n"
"bool error_flag\n"
"\n"
"bool Within_specifications = 0\n"
"bool temperature_status\n"
"bool voltage_supply_status\n"
"\n"
"bool Powered = 0\n"
"bool Not_powered = 1\n"
"bool primary_antenna_power_status\n"
"\n"
"# N1\n"
"bool LNA_failure\n"
"\n"
"bool Open_and_Antenna_disconnected = 1\n"
"bool primary_antenna_open_circuit_flag\n"
"\n"
"bool Short_circuit_detected = 1\n"
"bool primary_antenna_short_circuit_flag\n"
"\n"
"bool No_overload = 0\n"
"bool Overload = 1\n"
"bool CPU_overload_flag\n"
"\n"
"# N2\n"
"bool COM_port_transmit_buffer_overrun\n"
"\n"
"bool Not_detected = 0\n"
"bool Detected = 1\n"
"bool spoofing_detection_status\n"
"\n"
"bool reserved\n"
"bool link_overrun_flag\n"
"\n"
"# N3\n"
"bool input_overrun_flag\n"
"bool aux_transmit_overrun_flag\n"
"\n"
"bool Out_of_range = 1\n"
"bool antenna_gain_state\n"
"\n"
"bool Jammer_Detected = 1\n"
"bool jammer_detected\n"
"\n"
"# N4\n"
"bool No_INS_reset = 0\n"
"bool INS_reset = 1\n"
"bool INS_reset_flag\n"
"\n"
"bool No_IMU_communication = 1\n"
"bool IMU_communication_failure\n"
"\n"
"bool GPS_almanac_flag_UTC_known\n"
"bool position_solution_flag\n"
"\n"
"# N5\n"
"bool Not_fixed = 0\n"
"bool Fixed = 1\n"
"bool position_fixed_flag\n"
"\n"
"bool Enabled_clock_steering = 0\n"
"bool Disabled_clock_steering = 1\n"
"bool clock_steering_status\n"
"\n"
"bool clock_model_flag\n"
"\n"
"bool UnLocked = 0\n"
"bool Locked = 1\n"
"bool external_oscillator_locked_flag\n"
"\n"
"# N6\n"
"bool software_resource\n"
"\n"
"bool OEM6_or_earlier_format = 0\n"
"bool OEM7_format = 1\n"
"bool status_error_version_bit\n"
"\n"
"bool version_bit_1\n"
"\n"
"bool Normal_tracking = 0\n"
"bool HDR_tracking = 1\n"
"bool tracking_mode\n"
"\n"
"# N7\n"
"bool Disables_digital_filtering = 0\n"
"bool Enables_digital_filtering = 1\n"
"bool digital_filtering_enabled\n"
"\n"
"bool No_event = 0\n"
"bool Event = 1\n"
"bool auxiliary_3_status_event_flag\n"
"bool auxiliary_2_status_event_flag\n"
"bool auxiliary_1_status_event_flag\n"
"================================================================================\n"
"MSG: integrated_navigation_reader/Log_SolutionStatus\n"
"# Solution status - page 500\n"
"\n"
"uint8 SOL_COMPUTED = 0\n"
"uint8 INSUFFICIENT_OBS = 1\n"
"uint8 NO_CONVERGENCE = 2\n"
"uint8 SINGULARITY = 3\n"
"uint8 COV_TRACE = 4\n"
"uint8 TEST_DIST = 5\n"
"uint8 COLD_START = 6\n"
"uint8 V_H_LIMIT = 7\n"
"uint8 VARIANCE = 8\n"
"uint8 RESIDUALS = 9\n"
"uint8 INTEGRITY_WARNING = 13\n"
"uint8 PENDING = 18\n"
"uint8 INVALID_FIX = 19\n"
"uint8 UNAUTHORIZED = 20\n"
"uint8 INVALID_RATE = 22\n"
"uint8 status\n"
"================================================================================\n"
"MSG: integrated_navigation_reader/Log_PositionVelocityType\n"
"# Position type - page 501\n"
"\n"
"uint8 NONE = 0\n"
"uint8 FIXEDPOS = 1\n"
"uint8 FIXEDHEIGHT = 2\n"
"uint8 DOPPLER_VELOCITY = 8\n"
"uint8 SINGLE = 16\n"
"uint8 PSRDIFF = 17\n"
"uint8 WAAS = 18\n"
"uint8 PROPAGATED = 19\n"
"uint8 L1_FLOAT = 32\n"
"uint8 NARROW_FLOAT = 34\n"
"uint8 L1_INT = 48\n"
"uint8 WIDE_INT = 49\n"
"uint8 NARROW_INT = 50\n"
"uint8 RTK_DIRECT_INS = 51\n"
"uint8 INS_SBAS = 52\n"
"uint8 INS_PSRSP = 53\n"
"uint8 INS_PSRDIFF = 54\n"
"uint8 INS_RTKFLOAT = 55\n"
"uint8 INS_RTKFIXED = 56\n"
"uint8 PPP_CONVERGING = 68\n"
"uint8 PPP = 69\n"
"uint8 OPERATIONAL = 70\n"
"uint8 WARNING = 71\n"
"uint8 OUT_OF_BOUNDS = 72\n"
"uint8 INS_PPP_CONVERGING = 73\n"
"uint8 INS_PPP = 74\n"
"uint8 PPP_BASIC_CONVERGING = 77\n"
"uint8 PPP_BASIC = 78\n"
"uint8 INS_PPP_BASIC_CONVERGING = 79\n"
"uint8 INS_PPP_BASIC = 80\n"
"uint8 type\n"
"================================================================================\n"
"MSG: integrated_navigation_reader/Log_ExtendedSolutionStatus\n"
"# Solution status - page 504\n"
"\n"
"bool RTK_or_PDP_GLIDE_solution\n"
"\n"
"uint8 Unknown_or_default_Klobuchar_model = 0\n"
"uint8 Klobuchar_Broadcast = 1\n"
"uint8 SBAS_Broadcast = 2\n"
"uint8 Multi_frequency_Computed = 3\n"
"uint8 PSRDiff_correction = 4\n"
"uint8 Novatel_Blended_Iono_Value = 5\n"
"uint8 pseudorange_iono_correction\n"
"\n"
"bool RTK_assist_active\n"
"bool antenna_information_status\n"
"bool reserved\n"
"bool used_terrain_compensation_corrections\n"
"================================================================================\n"
"MSG: integrated_navigation_reader/Log_GalileoBeiDouSignalUsedMask\n"
"# Solution status - page 503\n"
"\n"
"bool Unused = 0\n"
"bool Used = 1\n"
"bool galileo_E1\n"
"bool galileo_E5a\n"
"bool galileo_E5b\n"
"bool galileo_ALTBOC\n"
"bool beidou_B1\n"
"bool beidou_B2\n"
"bool beidou_B3\n"
"bool beidou_E6\n"
"================================================================================\n"
"MSG: integrated_navigation_reader/Log_GPSGLONASSSignalUsedMask\n"
"# Solution status - page 503\n"
"\n"
"bool Unused = 0\n"
"bool Used = 1\n"
"bool gps_L1\n"
"bool gps_L2\n"
"bool gps_L5\n"
"bool reserved_bit_3\n"
"bool glonass_L1\n"
"bool glonass_L2\n"
"bool glonass_L3\n"
"bool reserved_bit_7\n"
;
  }

  static const char* value(const ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.log_header);
      stream.next(m.solution_status);
      stream.next(m.position_type);
      stream.next(m.latitude);
      stream.next(m.longitude);
      stream.next(m.height);
      stream.next(m.undulation);
      stream.next(m.datum_id);
      stream.next(m.latitude_std);
      stream.next(m.longitude_std);
      stream.next(m.height_std);
      stream.next(m.base_station_id);
      stream.next(m.differential_age);
      stream.next(m.solution_age);
      stream.next(m.satellites_tracked);
      stream.next(m.satellites_solutions_used);
      stream.next(m.satellites_L1E1B1_used);
      stream.next(m.satellites_multi_frequency_used);
      stream.next(m.reserved);
      stream.next(m.extended_solution_status);
      stream.next(m.signal_mask_galileo_beidou);
      stream.next(m.signal_mask_gps_glonass);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SPANLog_BESTGNSSPOSB_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::integrated_navigation_reader::SPANLog_BESTGNSSPOSB_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "log_header: ";
    s << std::endl;
    Printer< ::integrated_navigation_reader::BinaryHeader_<ContainerAllocator> >::stream(s, indent + "  ", v.log_header);
    s << indent << "solution_status: ";
    s << std::endl;
    Printer< ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator> >::stream(s, indent + "  ", v.solution_status);
    s << indent << "position_type: ";
    s << std::endl;
    Printer< ::integrated_navigation_reader::Log_PositionVelocityType_<ContainerAllocator> >::stream(s, indent + "  ", v.position_type);
    s << indent << "latitude: ";
    Printer<double>::stream(s, indent + "  ", v.latitude);
    s << indent << "longitude: ";
    Printer<double>::stream(s, indent + "  ", v.longitude);
    s << indent << "height: ";
    Printer<double>::stream(s, indent + "  ", v.height);
    s << indent << "undulation: ";
    Printer<double>::stream(s, indent + "  ", v.undulation);
    s << indent << "datum_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.datum_id);
    s << indent << "latitude_std: ";
    Printer<double>::stream(s, indent + "  ", v.latitude_std);
    s << indent << "longitude_std: ";
    Printer<double>::stream(s, indent + "  ", v.longitude_std);
    s << indent << "height_std: ";
    Printer<double>::stream(s, indent + "  ", v.height_std);
    s << indent << "base_station_id[]" << std::endl;
    for (size_t i = 0; i < v.base_station_id.size(); ++i)
    {
      s << indent << "  base_station_id[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.base_station_id[i]);
    }
    s << indent << "differential_age: ";
    Printer<double>::stream(s, indent + "  ", v.differential_age);
    s << indent << "solution_age: ";
    Printer<double>::stream(s, indent + "  ", v.solution_age);
    s << indent << "satellites_tracked: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.satellites_tracked);
    s << indent << "satellites_solutions_used: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.satellites_solutions_used);
    s << indent << "satellites_L1E1B1_used: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.satellites_L1E1B1_used);
    s << indent << "satellites_multi_frequency_used: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.satellites_multi_frequency_used);
    s << indent << "reserved: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reserved);
    s << indent << "extended_solution_status: ";
    s << std::endl;
    Printer< ::integrated_navigation_reader::Log_ExtendedSolutionStatus_<ContainerAllocator> >::stream(s, indent + "  ", v.extended_solution_status);
    s << indent << "signal_mask_galileo_beidou: ";
    s << std::endl;
    Printer< ::integrated_navigation_reader::Log_GalileoBeiDouSignalUsedMask_<ContainerAllocator> >::stream(s, indent + "  ", v.signal_mask_galileo_beidou);
    s << indent << "signal_mask_gps_glonass: ";
    s << std::endl;
    Printer< ::integrated_navigation_reader::Log_GPSGLONASSSignalUsedMask_<ContainerAllocator> >::stream(s, indent + "  ", v.signal_mask_gps_glonass);
  }
};

} // namespace message_operations
} // namespace ros

#endif // INTEGRATED_NAVIGATION_READER_MESSAGE_SPANLOG_BESTGNSSPOSB_H
