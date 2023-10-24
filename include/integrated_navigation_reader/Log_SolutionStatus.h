// Generated by gencpp from file integrated_navigation_reader/Log_SolutionStatus.msg
// DO NOT EDIT!


#ifndef INTEGRATED_NAVIGATION_READER_MESSAGE_LOG_SOLUTIONSTATUS_H
#define INTEGRATED_NAVIGATION_READER_MESSAGE_LOG_SOLUTIONSTATUS_H


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
struct Log_SolutionStatus_
{
  typedef Log_SolutionStatus_<ContainerAllocator> Type;

  Log_SolutionStatus_()
    : status(0)  {
    }
  Log_SolutionStatus_(const ContainerAllocator& _alloc)
    : status(0)  {
  (void)_alloc;
    }



   typedef uint8_t _status_type;
  _status_type status;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(SOL_COMPUTED)
  #undef SOL_COMPUTED
#endif
#if defined(_WIN32) && defined(INSUFFICIENT_OBS)
  #undef INSUFFICIENT_OBS
#endif
#if defined(_WIN32) && defined(NO_CONVERGENCE)
  #undef NO_CONVERGENCE
#endif
#if defined(_WIN32) && defined(SINGULARITY)
  #undef SINGULARITY
#endif
#if defined(_WIN32) && defined(COV_TRACE)
  #undef COV_TRACE
#endif
#if defined(_WIN32) && defined(TEST_DIST)
  #undef TEST_DIST
#endif
#if defined(_WIN32) && defined(COLD_START)
  #undef COLD_START
#endif
#if defined(_WIN32) && defined(V_H_LIMIT)
  #undef V_H_LIMIT
#endif
#if defined(_WIN32) && defined(VARIANCE)
  #undef VARIANCE
#endif
#if defined(_WIN32) && defined(RESIDUALS)
  #undef RESIDUALS
#endif
#if defined(_WIN32) && defined(INTEGRITY_WARNING)
  #undef INTEGRITY_WARNING
#endif
#if defined(_WIN32) && defined(PENDING)
  #undef PENDING
#endif
#if defined(_WIN32) && defined(INVALID_FIX)
  #undef INVALID_FIX
#endif
#if defined(_WIN32) && defined(UNAUTHORIZED)
  #undef UNAUTHORIZED
#endif
#if defined(_WIN32) && defined(INVALID_RATE)
  #undef INVALID_RATE
#endif

  enum {
    SOL_COMPUTED = 0u,
    INSUFFICIENT_OBS = 1u,
    NO_CONVERGENCE = 2u,
    SINGULARITY = 3u,
    COV_TRACE = 4u,
    TEST_DIST = 5u,
    COLD_START = 6u,
    V_H_LIMIT = 7u,
    VARIANCE = 8u,
    RESIDUALS = 9u,
    INTEGRITY_WARNING = 13u,
    PENDING = 18u,
    INVALID_FIX = 19u,
    UNAUTHORIZED = 20u,
    INVALID_RATE = 22u,
  };


  typedef boost::shared_ptr< ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator> const> ConstPtr;

}; // struct Log_SolutionStatus_

typedef ::integrated_navigation_reader::Log_SolutionStatus_<std::allocator<void> > Log_SolutionStatus;

typedef boost::shared_ptr< ::integrated_navigation_reader::Log_SolutionStatus > Log_SolutionStatusPtr;
typedef boost::shared_ptr< ::integrated_navigation_reader::Log_SolutionStatus const> Log_SolutionStatusConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator1> & lhs, const ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator2> & rhs)
{
  return lhs.status == rhs.status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator1> & lhs, const ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace integrated_navigation_reader

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fb2a32e20ac0f0791a5861a8ae931ac0";
  }

  static const char* value(const ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfb2a32e20ac0f079ULL;
  static const uint64_t static_value2 = 0x1a5861a8ae931ac0ULL;
};

template<class ContainerAllocator>
struct DataType< ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "integrated_navigation_reader/Log_SolutionStatus";
  }

  static const char* value(const ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Solution status - page 500\n"
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
;
  }

  static const char* value(const ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Log_SolutionStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::integrated_navigation_reader::Log_SolutionStatus_<ContainerAllocator>& v)
  {
    s << indent << "status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // INTEGRATED_NAVIGATION_READER_MESSAGE_LOG_SOLUTIONSTATUS_H
