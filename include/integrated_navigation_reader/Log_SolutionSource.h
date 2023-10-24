// Generated by gencpp from file integrated_navigation_reader/Log_SolutionSource.msg
// DO NOT EDIT!


#ifndef INTEGRATED_NAVIGATION_READER_MESSAGE_LOG_SOLUTIONSOURCE_H
#define INTEGRATED_NAVIGATION_READER_MESSAGE_LOG_SOLUTIONSOURCE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace integrated_navigation_reader
{
template <class ContainerAllocator>
struct Log_SolutionSource_
{
  typedef Log_SolutionSource_<ContainerAllocator> Type;

  Log_SolutionSource_()
    : reserved_bit_0(0)
    , reserved_bit_1(0)
    , source_antenna(0)
    , reserved_bit_4(0)
    , reserved_bit_5(0)
    , reserved_bit_6(0)
    , reserved_bit_7(0)  {
    }
  Log_SolutionSource_(const ContainerAllocator& _alloc)
    : reserved_bit_0(0)
    , reserved_bit_1(0)
    , source_antenna(0)
    , reserved_bit_4(0)
    , reserved_bit_5(0)
    , reserved_bit_6(0)
    , reserved_bit_7(0)  {
  (void)_alloc;
    }



   typedef uint8_t _reserved_bit_0_type;
  _reserved_bit_0_type reserved_bit_0;

   typedef uint8_t _reserved_bit_1_type;
  _reserved_bit_1_type reserved_bit_1;

   typedef uint8_t _source_antenna_type;
  _source_antenna_type source_antenna;

   typedef uint8_t _reserved_bit_4_type;
  _reserved_bit_4_type reserved_bit_4;

   typedef uint8_t _reserved_bit_5_type;
  _reserved_bit_5_type reserved_bit_5;

   typedef uint8_t _reserved_bit_6_type;
  _reserved_bit_6_type reserved_bit_6;

   typedef uint8_t _reserved_bit_7_type;
  _reserved_bit_7_type reserved_bit_7;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(Primary_antenna)
  #undef Primary_antenna
#endif
#if defined(_WIN32) && defined(Secondary_antenna)
  #undef Secondary_antenna
#endif

  enum {
    Primary_antenna = 0u,
    Secondary_antenna = 1u,
  };


  typedef boost::shared_ptr< ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator> const> ConstPtr;

}; // struct Log_SolutionSource_

typedef ::integrated_navigation_reader::Log_SolutionSource_<std::allocator<void> > Log_SolutionSource;

typedef boost::shared_ptr< ::integrated_navigation_reader::Log_SolutionSource > Log_SolutionSourcePtr;
typedef boost::shared_ptr< ::integrated_navigation_reader::Log_SolutionSource const> Log_SolutionSourceConstPtr;

// constants requiring out of line definition

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator1> & lhs, const ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator2> & rhs)
{
  return lhs.reserved_bit_0 == rhs.reserved_bit_0 &&
    lhs.reserved_bit_1 == rhs.reserved_bit_1 &&
    lhs.source_antenna == rhs.source_antenna &&
    lhs.reserved_bit_4 == rhs.reserved_bit_4 &&
    lhs.reserved_bit_5 == rhs.reserved_bit_5 &&
    lhs.reserved_bit_6 == rhs.reserved_bit_6 &&
    lhs.reserved_bit_7 == rhs.reserved_bit_7;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator1> & lhs, const ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace integrated_navigation_reader

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator> >
{
  static const char* value()
  {
    return "81d0b5eee283d00968abfc30c58b6022";
  }

  static const char* value(const ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x81d0b5eee283d009ULL;
  static const uint64_t static_value2 = 0x68abfc30c58b6022ULL;
};

template<class ContainerAllocator>
struct DataType< ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator> >
{
  static const char* value()
  {
    return "integrated_navigation_reader/Log_SolutionSource";
  }

  static const char* value(const ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Solution status - page 625\n"
"\n"
"uint8 reserved_bit_0\n"
"uint8 reserved_bit_1\n"
"\n"
"uint8 Primary_antenna = 0\n"
"uint8 Secondary_antenna = 1\n"
"uint8 source_antenna\n"
"\n"
"uint8 reserved_bit_4\n"
"uint8 reserved_bit_5\n"
"uint8 reserved_bit_6\n"
"uint8 reserved_bit_7\n"
;
  }

  static const char* value(const ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.reserved_bit_0);
      stream.next(m.reserved_bit_1);
      stream.next(m.source_antenna);
      stream.next(m.reserved_bit_4);
      stream.next(m.reserved_bit_5);
      stream.next(m.reserved_bit_6);
      stream.next(m.reserved_bit_7);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Log_SolutionSource_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::integrated_navigation_reader::Log_SolutionSource_<ContainerAllocator>& v)
  {
    s << indent << "reserved_bit_0: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reserved_bit_0);
    s << indent << "reserved_bit_1: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reserved_bit_1);
    s << indent << "source_antenna: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.source_antenna);
    s << indent << "reserved_bit_4: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reserved_bit_4);
    s << indent << "reserved_bit_5: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reserved_bit_5);
    s << indent << "reserved_bit_6: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reserved_bit_6);
    s << indent << "reserved_bit_7: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reserved_bit_7);
  }
};

} // namespace message_operations
} // namespace ros

#endif // INTEGRATED_NAVIGATION_READER_MESSAGE_LOG_SOLUTIONSOURCE_H
